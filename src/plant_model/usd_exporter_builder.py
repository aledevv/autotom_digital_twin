"""
usd_exporter_builder.py

Builds a physically simulated plant using the high-level PlantBuilder API.
Implements a 10x "Baked Scale" and merges biological internodes to respect PhysX limits.
"""

from .builder_constants import (
    LATERAL_BRANCH_MAX_BEND_ANGLE,
    LATERAL_BRANCH_DENSITY,
    LATERAL_BRANCH_DAMPING_TIP,
    LATERAL_BRANCH_DAMPING_BASE,
    LATERAL_BRANCH_STIFFNESS_TIP,
    LATERAL_BRANCH_STIFFNESS_BASE
)
import math
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

from .models import PlantSnapshot, InternodeNode, LeafNode, FruitsNode
from .plant_builder import PlantBuilder

BAKED_SCALE = 10.0
MAX_STEM_SEGMENTS = 25  # Budget for the main stem
PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemBuilder"

from .constants import PHYLLOTAXIS

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
        
        # Extract and scale compound leaf arrays
        n_blades = max(leaf.blades_nr, 1)
        area_scale = BAKED_SCALE ** 2
        
        raw_areas = getattr(leaf, 'leaf_area_m2blades', [])
        if not raw_areas or len(raw_areas) == 0:
            avg_area = leaf.area_blades_total / n_blades if n_blades > 0 else 0.0004
            raw_areas = [avg_area] * n_blades
        scaled_areas = [a * area_scale for a in raw_areas]
        
        raw_segs = getattr(leaf, 'leaf_segments_length', [])
        if not raw_segs or len(raw_segs) == 0:
            avg_seg = leaf.rachis_length / max(n_blades - 1, 1)
            raw_segs = [avg_seg] * max(n_blades - 1, 1)
        scaled_segs = [s * BAKED_SCALE for s in raw_segs]
        
        raw_incl = getattr(leaf, 'leaf_inclination_segments', [])
        if not raw_incl or len(raw_incl) == 0:
            raw_incl = [50.0] * max(n_blades - 1, 1)
            
        petiole_len_scaled = max(leaf.length_petiole * BAKED_SCALE, 0.005)
        rachis_len_scaled = max(leaf.rachis_length * BAKED_SCALE, 0.005)
        
        num_segments = max(n_blades, 2)*2  # Use at least 2 segments for the physics chain
        
        # Derive petiole/rachis radius from CSV diameter field
        petiole_radius_m = getattr(leaf, 'diameter_petiole', 0.002) / 2.0
        start_radius_scaled = max(petiole_radius_m * BAKED_SCALE, 0.0015)
        end_radius_scaled = max(start_radius_scaled * 0.5, 0.001)

        raw_ccw = getattr(leaf, 'ccw_orientation', 0.0)
        if abs(raw_ccw) > 1e-3:
            azimuth_deg = raw_ccw
        else:
            azimuth_deg = (leaf.parent.key.rank * PHYLLOTAXIS) % 360.0

        rot_angle = (azimuth_deg - 90.0) % 360.0

        # Debug: print expected sizes
        print(f"  [LEAF {leaf_id}] n_blades={n_blades}, "
              f"petiole_r={start_radius_scaled:.4f}m, "
              f"petiole_L={petiole_len_scaled:.4f}m, rachis_L={rachis_len_scaled:.4f}m, "
              f"areas_scaled={[f'{a:.4f}' for a in scaled_areas]}")
        
        builder.add_compound_leaf(
            parent_id=parent_seg['id'],
            base_id=leaf_id,
            petiole_length=petiole_len_scaled,
            rachis_length=rachis_len_scaled,
            start_radius=start_radius_scaled,
            end_radius=end_radius_scaled,
            num_segments=num_segments,
            blade_area_array=scaled_areas,
            seg_len_array=scaled_segs,
            incl_array=raw_incl,
            z_offset_ratio=z_offset_ratio,
            tilt_angle=leaf.angle_petiole if hasattr(leaf, 'angle_petiole') and leaf.angle_petiole else 60.0,
            lateral_tilt_angle=leaf.lateral_tilt_angle if hasattr(leaf, 'lateral_tilt_angle') and leaf.lateral_tilt_angle else 70.0,
<<<<<<< HEAD
            rot_around_parent=rot_angle
=======
            rot_around_parent=leaf.ccw_orientation if hasattr(leaf, 'ccw_orientation') else 0.0
>>>>>>> 5d61e47 (code refactory and cleaning)
        )

def attach_fruits(builder: PlantBuilder, fruits: list[FruitsNode], stem_segments: list[dict]):
    """Placeholder for attaching fruits and trusses."""
    # TODO: Implement truss and fruit attachment logic here
    # 1. Find parent segment (similar to leaves)
    # 2. Use builder.add_truss_rachis(...)
    # 3. Use builder.add_fruit(...) for each fruit in the truss
    pass

def attach_lateral_branches(builder: PlantBuilder, internodes: list[InternodeNode], stem_segments: list[dict]):
    """
    Attaches first-order lateral branches (order > 0) to the merged main stem.

    Current scope:
    - builds each lateral branch chain starting from the first internode of that order/rank group
    - attaches only branches whose parent is on the main stem (order 0 / min order)
    - returns metadata so leaves can later be attached to the lateral branch segments too
    """
    if not internodes:
        return []

    lateral_branch_infos = []

    # Keep only internodes that have an internode parent
    valid_nodes = [n for n in internodes if n.parent is not None and isinstance(n.parent, InternodeNode)]
    if not valid_nodes:
        return []

    # Group branch internodes by their first node:
    # key = (order, rank of parent on main stem, organ rank of branch start)
    # simpler practical grouping: each distinct first internode node starts one branch chain
    branch_starts = []
    seen_branch_starts = set()

    for node in valid_nodes:
        if not isinstance(node.parent, InternodeNode):
            continue
        if node.parent.key.order < node.key.order:
            start_key = (
                node.key.order,
                node.key.rank,
                node.parent.key.order,
                node.parent.key.rank,
            )
            if start_key in seen_branch_starts:
                continue
            seen_branch_starts.add(start_key)
            branch_starts.append(node)

    branch_starts.sort(key=lambda n: (n.key.order, n.parent.key.rank, n.key.rank))

    for start_node in branch_starts:
        # Collect the whole internode chain of this branch by walking descendants
        chain = [start_node]
        current = start_node
        while True:
            children = [
                n for n in valid_nodes
                if n.parent is current and isinstance(n.parent, InternodeNode)
            ]
            if not children:
                break
            # if multiple children exist, keep the smallest rank as continuation
            children.sort(key=lambda n: n.key.rank)
            current = children[0]
            chain.append(current)

        # Attach branch to the main stem only if the branch start comes off the main stem
        parent_main = start_node.parent
        if parent_main is None or not isinstance(parent_main, InternodeNode):
            continue

        tip_world_z_unscaled = parent_main.world_base_z + parent_main.length
        tip_world_z_scaled = tip_world_z_unscaled * BAKED_SCALE
        parent_seg = _find_parent_segment(stem_segments, tip_world_z_scaled)

        local_z = tip_world_z_scaled - parent_seg['base_z']
        z_offset_ratio = max(0.0, min(1.0, local_z / parent_seg['height']))

        total_len_scaled = sum(n.length for n in chain) * BAKED_SCALE
        start_radius_scaled = max((chain[0].width_m / 2.0) * BAKED_SCALE, 0.0015)
        end_radius_scaled = max((chain[-1].width_m / 2.0) * BAKED_SCALE, 0.0010)

        num_segments = max(len(chain), 1)

        raw_ccw = getattr(start_node, 'ccw_orientation', 0.0)
        if abs(raw_ccw) > 1e-3:
            azimuth_deg = raw_ccw
        else:
            azimuth_deg = (start_node.key.rank * PHYLLOTAXIS) % 360.0

        rot_angle = (azimuth_deg - 90.0) % 360.0

        branch_base_id = (
            f"Branch_o{start_node.key.order}"
            f"_r{start_node.key.rank}"
            f"_po{start_node.parent.key.order}"
            f"_pr{start_node.parent.key.rank}"
        )

        print(
            f" [BRANCH {branch_base_id}] "
            f"parent={parent_seg['id']} "
            f"n_segments={num_segments} "
            f"L={total_len_scaled:.4f}m "
            f"r0={start_radius_scaled:.4f}m "
            f"r1={end_radius_scaled:.4f}m "
            f"rot={rot_angle:.1f}°"
        )

        tip_id = builder.add_branch(
            parent_id=parent_seg['id'],
            base_id=branch_base_id,
            total_length=total_len_scaled,
            start_radius=start_radius_scaled,
            end_radius=end_radius_scaled,
            num_segments=num_segments,
            z_offset_ratio=z_offset_ratio,
            tilt_angle=45.0,
            rot_around_parent=rot_angle,
            stiffness_base=LATERAL_BRANCH_STIFFNESS_BASE,
            stiffness_tip=LATERAL_BRANCH_STIFFNESS_TIP,
            damping_base=LATERAL_BRANCH_DAMPING_BASE,
            damping_tip=LATERAL_BRANCH_DAMPING_TIP,
            density=LATERAL_BRANCH_DENSITY,
            max_bend_angle=LATERAL_BRANCH_MAX_BEND_ANGLE
        )

        built_segments = [f"{branch_base_id}_{i:02d}" for i in range(num_segments)]
        lateral_branch_infos.append({
            "start_node": start_node,
            "chain": chain,
            "base_id": branch_base_id,
            "tip_id": tip_id,
            "segments": built_segments,
        })

    return lateral_branch_infos

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
    builder = PlantBuilder(stage, stem_path)

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

    # 4. Attach Lateral Branches
    lateral_internodes = [n for n in all_internodes if n.key.order > min_order]
    # branch_infos = attach_lateral_branches(builder, lateral_internodes, stem_segments)
    # print(f"[INFO] Attached {len(branch_infos)} lateral branch chains.")

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