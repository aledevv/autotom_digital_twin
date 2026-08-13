"""
usd_exporter_builder.py

Builds a physically simulated plant using the high-level PlantBuilder API.
Implements a 10x "Baked Scale" and merges biological internodes to respect PhysX limits.
"""

import math
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

from .models import PlantSnapshot, InternodeNode, LeafNode, FruitsNode
from .plant_builder import PlantBuilder

BAKED_SCALE = 10.0
MAX_STEM_SEGMENTS = 25  # Budget for the main stem
PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemBuilder"

def _compute_world_base_z(node: InternodeNode) -> float:
    """Computes and caches the world Z coordinate (unscaled) of the node's base."""
    if hasattr(node, 'world_base_z'):
        return node.world_base_z
    if node.parent is None or not isinstance(node.parent, InternodeNode):
        node.world_base_z = 0.0
    else:
        node.world_base_z = _compute_world_base_z(node.parent) + node.parent.length
    return node.world_base_z

def _find_parent_segment(parent_segments: list[dict], target_world_z_scaled: float) -> dict:
    """Finds the physical segment whose [base_z, base_z+height] contains the target Z."""
    for seg in parent_segments:
        if seg['base_z'] <= target_world_z_scaled <= seg['base_z'] + seg['height'] + 1e-9:
            return seg
    return min(parent_segments, key=lambda s: abs(s['base_z'] - target_world_z_scaled))

def build_merged_stem(builder: PlantBuilder, internodes: list[InternodeNode], max_segments: int) -> list[dict]:
    """
    Takes a list of biological internodes (order 0) and merges them into at most max_segments physical segments.
    Returns a list of dictionaries with segment data so leaves can be attached.
    """
    total_length = sum(n.length for n in internodes)
    target_seg_length = total_length / max_segments if max_segments > 0 else total_length
    
    physical_segments = []
    current_physical_len = 0.0
    current_physical_vol = 0.0
    
    prev_id = None
    seg_idx = 1
    current_phys_base_z = 0.0
    
    for i, node in enumerate(internodes):
        current_physical_len += node.length
        # Approximate volume sum to get average radius
        radius = node.width_m / 2.0
        current_physical_vol += (radius ** 2) * node.length 
        
        # If we reached the target length for a segment, OR it's the last node
        if current_physical_len >= target_seg_length or i == len(internodes) - 1:
            avg_radius = (current_physical_vol / current_physical_len) ** 0.5 if current_physical_len > 0 else radius
            
            # Scale geometry
            scaled_len = current_physical_len * BAKED_SCALE
            scaled_rad = avg_radius * BAKED_SCALE
            
            seg_id = f"Stem_{seg_idx:03d}"
            
            # Artificial mass for stability: 500 kg/m3 density applied to scaled geometry
            mass = max(math.pi * (scaled_rad**2) * scaled_len * 500.0, 0.05)
            
            if prev_id is None:
                builder.create_root(seg_id, radius=scaled_rad, length=scaled_len, mass=mass)
            else:
                # Taper stiffness along the stem
                stiffness = max(500000.0 / seg_idx, 1000.0)
                damping = max(100.0 / seg_idx, 10.0)
                builder.add_internode(prev_id, seg_id, radius=scaled_rad, length=scaled_len,
                                      mass=mass, stiffness=stiffness, damping=damping)
            
            physical_segments.append({
                'id': seg_id,
                'path': builder._segments[seg_id]['path'],
                'base_z': current_phys_base_z,
                'height': scaled_len
            })
            
            current_phys_base_z += scaled_len
            prev_id = seg_id
            seg_idx += 1
            
            current_physical_len = 0.0
            current_physical_vol = 0.0
            
    return physical_segments

def attach_leaves(builder: PlantBuilder, leaves: list[LeafNode], stem_segments: list[dict]):
    """Attaches leaves to the appropriate merged stem segments."""
    for leaf in leaves:
        if leaf.parent is None or not isinstance(leaf.parent, InternodeNode):
            continue
            
        # The leaf attaches at the top of its parent internode
        tip_world_z_unscaled = leaf.parent.world_base_z + leaf.parent.length
        tip_world_z_scaled = tip_world_z_unscaled * BAKED_SCALE
        
        parent_seg = _find_parent_segment(stem_segments, tip_world_z_scaled)
        
        # Calculate where along the physical segment it attaches (0.0 to 1.0)
        local_z = tip_world_z_scaled - parent_seg['base_z']
        z_offset_ratio = max(0.0, min(1.0, local_z / parent_seg['height']))
        
        leaf_id = f"Leaf_o{leaf.key.order}_r{leaf.key.rank}_i{leaf.key.organ_index}"
        
        # The true length is derived from the blade area
        n_blades = max(leaf.blades_nr, 1)
        blade_area = leaf.area_blades_total / n_blades
        blade_length_m = math.sqrt(blade_area / 0.6) if blade_area > 0 else 0.0
        blade_width_m = blade_length_m * 0.6
        
        # Note: PlantBuilder expects scaled values for lengths
        # Ensure a minimum size so PhysX doesn't fail on 0-sized geometry
        leaf_length_scaled = max(blade_length_m * BAKED_SCALE, 0.01)
        leaf_width_scaled = max(blade_width_m * BAKED_SCALE, 0.005)
        
        petiole_len_scaled = max(leaf.length_petiole * BAKED_SCALE, 0.005)
        
        builder.add_leaf(
            parent_id=parent_seg['id'], 
            id=leaf_id,
            leaf_length=leaf_length_scaled, 
            leaf_width=leaf_width_scaled,
            petiole_length=petiole_len_scaled,
            z_offset_ratio=z_offset_ratio,
            tilt_angle=leaf.angle_petiole if hasattr(leaf, 'angle_petiole') and leaf.angle_petiole else 60.0,
            rot_around_parent=leaf.ccw_orientation if hasattr(leaf, 'ccw_orientation') else 0.0
        )

def attach_fruits(builder: PlantBuilder, fruits: list[FruitsNode], stem_segments: list[dict]):
    """Placeholder for attaching fruits and trusses."""
    # TODO: Implement truss and fruit attachment logic here
    # 1. Find parent segment (similar to leaves)
    # 2. Use builder.add_truss_rachis(...)
    # 3. Use builder.add_fruit(...) for each fruit in the truss
    pass

def attach_lateral_branches(builder: PlantBuilder, internodes: list[InternodeNode], stem_segments: list[dict]):
    """Placeholder for attaching lateral branches (order > 0)."""
    # TODO: Implement lateral branches (order > 0)
    pass


def validate_usd_dimensions(stage: Usd.Stage, snapshot: PlantSnapshot, stem_path: str):
    """Verifies that the generated USD bounding boxes match the expected scaled dimensions."""
    print("\n" + "="*50)
    print("  USD DIMENSION VERIFICATION (POST-BUILD)")
    print("="*50)
    
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode) and n.parent and n.parent.key.order == 0]
    if not leaves:
        print("No leaves to verify.")
        return
        
    print(f"Checking {len(leaves)} leaves against expected scaled dimensions...")
    for leaf in leaves:
        leaf_id = f"Leaf_o{leaf.key.order}_r{leaf.key.rank}_i{leaf.key.organ_index}"
        blade_path = f"{stem_path}/{leaf_id}/BladeXform/Blade"
        prim = stage.GetPrimAtPath(blade_path)
        if prim:
            bbox = bbox_cache.ComputeLocalBound(prim)
            size = bbox.GetRange().GetSize()
            
            actual_width = size[0]
            actual_length = size[1]
            
            n_blades = max(leaf.blades_nr, 1)
            blade_area = leaf.area_blades_total / n_blades
            blade_length_m = math.sqrt(blade_area / 0.6) if blade_area > 0 else 0.0
            blade_width_m = blade_length_m * 0.6
            
            expected_length = max(blade_length_m * BAKED_SCALE, 0.01)
            expected_width = max(blade_width_m * BAKED_SCALE, 0.005)
            
            print(f"  {leaf_id}:")
            print(f"    Expected: length={expected_length:.4f}, width={expected_width:.4f}")
            print(f"    Actual:   length={actual_length:.4f}, width={actual_width:.4f}")
            if abs(actual_length - expected_length) > 1e-3:
                print("    -> [WARNING] Mismatch in length!")
    print("="*50 + "\n")


def build_plant_stage(snapshot: PlantSnapshot, output_path: str) -> tuple:
    """
    Builds a USD Stage using PlantBuilder for the given PlantSnapshot.
    Returns (stage, stem_path).
    """
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    plant_path = PLANT_ROOT_PATH_TEMPLATE.format(plant_id=snapshot.plant_id)
    plant_prim = UsdGeom.Xform.Define(stage, plant_path).GetPrim()
    stage.SetDefaultPrim(plant_prim)

    stem_path = f"{plant_path}/Stem"
    builder = PlantBuilder(stage, stem_path, global_scale=BAKED_SCALE)

    all_internodes = [n for n in snapshot.organs if isinstance(n, InternodeNode)]
    if not all_internodes:
        print("[WARN] No internodes found. Stage is empty.")
        return stage, stem_path

    # Only process main stem (minimum order) for now
    min_order = min(n.key.order for n in all_internodes)
    main_internodes = [n for n in all_internodes if n.key.order == min_order]
    main_internodes.sort(key=lambda n: n.key.rank)

    # Compute absolute Z heights (unscaled)
    for n in main_internodes:
        _compute_world_base_z(n)

    # 1. Build the merged main stem
    stem_segments = build_merged_stem(builder, main_internodes, max_segments=MAX_STEM_SEGMENTS)
    
    # 2. Attach Leaves
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode) and n.parent and n.parent.key.order == min_order]
    attach_leaves(builder, leaves, stem_segments)
    
    # 3. Attach Fruits (Placeholder)
    fruits = [n for n in snapshot.organs if isinstance(n, FruitsNode) and n.parent and n.parent.key.order == min_order]
    attach_fruits(builder, fruits, stem_segments)

    # 4. Attach Lateral Branches (Placeholder)
    # lateral_internodes = [n for n in all_internodes if n.key.order > min_order]
    # attach_lateral_branches(builder, lateral_internodes, stem_segments)

    print(f"[INFO] Created {len(stem_segments)} physical stem segments and attached {len(leaves)} leaves.")
    
    # Fix contact offsets for tiny geometry to avoid PhysX errors
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collider = UsdPhysics.MeshCollisionAPI(prim) if prim.IsA(UsdGeom.Mesh) else PhysxSchema.PhysxCollisionAPI.Apply(prim)
            if not prim.IsA(UsdGeom.Mesh):
                collider = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            if prim.IsA(UsdGeom.Cylinder):
                cyl = UsdGeom.Cylinder(prim)
                rad = cyl.GetRadiusAttr().Get()
                if rad:
                    collider.CreateContactOffsetAttr().Set(rad * 0.05)
                    collider.CreateRestOffsetAttr().Set(rad * 0.01)

    return stage, stem_path
