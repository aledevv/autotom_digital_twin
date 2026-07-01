import math
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import (
    PlantSnapshot, OrganNode, InternodeNode, RootNode, LeafNode, FruitsNode
)

from .constants import (
    INTERNODE_TRUSS_LENGTH_M,
    INTERNODE_TRUSS_DIAMETER_M,
    ANGLE_AMONG_SUBSEQUENT_FRUITS_DEG,
    FRUIT_PAIRING, ROOT_SPHERE_RADIUS,
    PETIOLE_LENGTH_M,
    TRUSS_LENGTH, TRUSS_RADIUS, PHYLLOTAXIS,
    JOINT_STIFFNESS_BASE, JOINT_STIFFNESS_TIP,
    JOINT_DAMPING, JOINT_MAX_ANGLE_DEG, STEM_DENSITY_KG_M3,
    RACHIS_SEG, PEDICEL_LEN, PEDICEL_R, INITIAL_TILT,
    ENABLE_STEM_PHYSICS, ENABLE_FRUIT_PHYSICS, FRUIT_DENSITY_KG_M3,
    ENABLE_LEAF_PHYSICS, LEAF_MASS_KG, LEAF_JOINT_STIFFNESS, LEAF_JOINT_DAMPING
)

from .usd_helpers import (
    OVERRIDE_LEAF_INCLINATION,
    _translate, _mat_to_gf, _set_transform,
    _make_cylinder, _make_sphere, _set_leaf_mesh_geometry, _make_leaf,
    _align_z_to, _blade_transform,
    _make_material, _bind_material,
    _apply_rigid_body, _make_stem_joint,
    _apply_rigid_body_to_leaf, _make_leaf_joint
)

# ─────────────────────────────────────────────────────────────────────────────
# Main exporter
# ─────────────────────────────────────────────────────────────────────────────

def export_plant_usd(snapshot: PlantSnapshot, output_path: str) -> None:

    # ── Stage setup ──────────────────────────────────────────────────────────
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)  # set axis z for height
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)  # set meters as length unit

    plant_path = f"/World/Plant_{snapshot.plant_id}"
    plant_prim = UsdGeom.Xform.Define(stage, plant_path).GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(plant_prim)

    stem_path  = f"{plant_path}/Stem"
    roots_path = f"{plant_path}/Roots"
    UsdGeom.Xform.Define(stage, stem_path)
    UsdGeom.Xform.Define(stage, roots_path)
    
    # ── Materials ────────────────────────────────────────────────────────────────
    mats_path = f"{plant_path}/Materials"
    UsdGeom.Xform.Define(stage, mats_path)

    mat_stem         = _make_material(stage, f"{mats_path}/Stem",         (0.45, 0.30, 0.10))  # brown
    mat_root         = _make_material(stage, f"{mats_path}/Root",         (0.55, 0.35, 0.15))  # dark brown
    mat_leaf         = _make_material(stage, f"{mats_path}/Leaf",         (0.15, 0.55, 0.10))  # green
    mat_pedicel      = _make_material(stage, f"{mats_path}/Pedicel",      (0.20, 0.50, 0.10))  # dark green
    mat_fruit_ripe   = _make_material(stage, f"{mats_path}/FruitRipe",    (0.90, 0.17, 0.10))  # ripe tomato red (from GroIMP spectra)
    mat_fruit_unripe = _make_material(stage, f"{mats_path}/FruitUnripe",  (0.45, 0.58, 0.25))  # unripe green-yellowish (from GroIMP spectra)
    
    materials = {
        "stem":         mat_stem,
        "root":         mat_root,
        "leaf":         mat_leaf,
        "pedicel":      mat_pedicel,
        "fruit_ripe":   mat_fruit_ripe,
        "fruit_unripe": mat_fruit_unripe,
    }

    # ── Separate organs ──────────────────────────────────────────────────────
    root_node = next((n for n in snapshot.organs if isinstance(n, RootNode)), None)
    
    def get_base_z(n) -> float:
        if n is None or not isinstance(n, InternodeNode): return 0.0
        if hasattr(n, 'world_base_z'): return n.world_base_z
        z = get_base_z(n.parent) + n.parent.length if (n.parent and isinstance(n.parent, InternodeNode)) else 0.0
        n.world_base_z = z
        return z

    for n in snapshot.organs:
        if isinstance(n, InternodeNode):
            get_base_z(n)

    internodes = sorted(
        [n for n in snapshot.organs if isinstance(n, InternodeNode)],
        key=lambda n: (n.key.order, n.key.rank)
    )

    print(f"\n{'='*50}")
    print(f"  Plant {snapshot.plant_id}  |  Day {snapshot.day}")
    print(f"  Internodes found: {len(internodes)}")
    print(f"{'='*50}\n")

    # ── Root sphere ───────────────────────────────────────────────────────────
    # Placed just below z=0 so it sits at ground level
    if root_node:
        print("[ROOT]")
        sph = _make_sphere(
            stage,
            path=f"{roots_path}/Root",
            radius=ROOT_SPHERE_RADIUS,
            cx=0.0, cy=0.0, cz=-ROOT_SPHERE_RADIUS,
        )
        
        _bind_material(sph, materials["root"])

    # ── Internode hierarchical rendering ──────────────────────────────────────
    max_z = 0.0
    for node in internodes:
        L = node.length
        R = node.width_m / 2.0
        rank = node.key.rank
        order = node.key.order
        base_z = getattr(node, 'world_base_z', 0.0)
        max_z = max(max_z, base_z + L)

        print(f"\n[INTERNODE order={order} rank={rank}]")
        cyl = _make_cylinder(
            stage,
            path=f"{stem_path}/Internode_o{order}_r{rank}",
            height=L,
            radius=R,
            base_z=base_z,
        )
        _bind_material(cyl, materials["stem"])
        

    # ── Physics: stem joint chain ─────────────────────────────────────────────
    # Toggle: ENABLE_STEM_PHYSICS in constants.py
    if ENABLE_STEM_PHYSICS:
        joints_path = f"{plant_path}/Joints"
        UsdGeom.Xform.Define(stage, joints_path)

        # Anchor rank-0 to ground
        if internodes:
            anchor_path = f"{stem_path}/Internode_o{internodes[0].key.order}_r{internodes[0].key.rank}"
            fixed = UsdPhysics.FixedJoint.Define(stage, f"{joints_path}/GroundAnchor")
            fixed.GetBody1Rel().SetTargets([Sdf.Path(anchor_path)])

        n_ranks = len(internodes)

        for i, node in enumerate(internodes):
            L = node.length
            R = node.width_m / 2.0
            path = f"{stem_path}/Internode_o{node.key.order}_r{node.key.rank}"

            # Mass: cylinder volume * density
            mass = math.pi * R**2 * L * STEM_DENSITY_KG_M3
            _apply_rigid_body(stage, path, mass)

            # Stiffness linearly decreases from bottom to the top
            t = i / max(n_ranks - 1, 1)
            stiffness = JOINT_STIFFNESS_BASE + t * (JOINT_STIFFNESS_TIP - JOINT_STIFFNESS_BASE)

            # Joint with parent
            if node.parent and isinstance(node.parent, InternodeNode):
                prev_path = f"{stem_path}/Internode_o{node.parent.key.order}_r{node.parent.key.rank}"
                _make_stem_joint(
                    stage,
                    joint_path=f"{joints_path}/Joint_o{node.key.order}_r{node.key.rank}",
                    body0_path=prev_path,
                    body1_path=path,
                    pivot0_z=node.parent.length / 2.0,
                    pivot1_z=-node.length / 2.0,
                    stiffness=stiffness,
                )
        
        

    # ── Leaves ───────────────────────────────────────────────────────────────
    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode)]

    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)

    for node in leaves:
        if node.parent and isinstance(node.parent, InternodeNode):
            tip_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            tip_z = 0.0

        leaf_id = f"o{node.key.order}_r{node.key.rank}_i{node.key.organ_index}"
        leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
        UsdGeom.Xform.Define(stage, leaf_group)

        print(f"\n[LEAF order={node.key.order} rank={node.key.rank} idx={node.key.organ_index}] "
                f"attaches at z={tip_z:.4f}m")
        _make_leaf(stage, leaf_group, node, tip_z, materials)
        
        if ENABLE_LEAF_PHYSICS:
            _apply_rigid_body_to_leaf(stage, leaf_group, LEAF_MASS_KG)
            
            # Disable collisions between leaf and parent internode to prevent explosion
            if node.parent and isinstance(node.parent, InternodeNode):
                parent_path = f"{stem_path}/Internode_o{node.parent.key.order}_r{node.parent.key.rank}"
                filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(leaf_group))
                filtered_pairs.GetFilteredPairsRel().AddTarget(Sdf.Path(parent_path))

                _make_leaf_joint(
                    stage,
                    joint_path=f"{plant_path}/Joints/Joint_Leaf_{leaf_id}",
                    body0_path=parent_path,
                    body1_path=leaf_group,
                    pivot0_z=node.parent.length / 2.0,
                    pivot1_world=Gf.Vec3f(0.0, 0.0, tip_z),
                    stiffness=LEAF_JOINT_STIFFNESS,
                    damping=LEAF_JOINT_DAMPING
                )
        
        
    # ── Fruits ───────────────────────────────────────────────────────────────────
    # Replicate GroIMP truss structure:
    #   - A main rachis made of INTERNODETRUSSLENGTH segments, tilting by
    #     internodeTrussAngle (9°) between each fruit.
    #   - Lateral pedicels (PETIOLELENGTH) branching off alternately (RU ±90°)
    #     from each rachis node, with a fruit sphere at the tip.
    #   - The last fruit sits at the terminal end of the rachis.

    trusses_path = f"{plant_path}/Trusses"
    UsdGeom.Xform.Define(stage, trusses_path)

    fruits_nodes = sorted(
        [n for n in snapshot.organs if isinstance(n, FruitsNode)],
        key=lambda n: n.key.rank
    )

    for node in fruits_nodes:
        if node.parent and isinstance(node.parent, InternodeNode):
            attach_z = getattr(node.parent, 'world_base_z', 0.0) + node.parent.length
        else:
            attach_z = 0.0

        truss_az = math.radians((node.key.rank * PHYLLOTAXIS) % 360)
        bend_per_fruit = node.truss_angle   # 9° per segment

        radii = [r for r in node.fruit_radii if r > 1e-5][:node.fruit_nr]
        n_fruits = len(radii)
        if n_fruits == 0:
            continue

        truss_group = f"{trusses_path}/Truss_r{node.key.rank}_i{node.key.organ_index}"
        UsdGeom.Xform.Define(stage, truss_group)

        print(f"\n[TRUSS rank={node.key.rank}] {n_fruits} fruits, attach_z={attach_z:.4f}m")

        # Current tip of the rachis — starts at attachment point on stem
        # Direction: initially tilted INITIAL_TILT° from Z towards the azimuth
        tilt_from_z = math.radians(INITIAL_TILT)
        # Rachis direction vector (in world coords)
        rach_dx = math.sin(tilt_from_z) * math.cos(truss_az)
        rach_dy = math.sin(tilt_from_z) * math.sin(truss_az)
        rach_dz = math.cos(tilt_from_z)

        cur_x, cur_y, cur_z = 0.0, 0.0, attach_z

        # Lateral direction for pedicels: perpendicular to rachis in the
        # horizontal plane. We alternate sign for RU(±90).
        lat_dx = -math.sin(truss_az)
        lat_dy =  math.cos(truss_az)
        lat_dz = 0.0

        seg_idx = 0

        for fi in range(n_fruits):
            r = radii[fi]
            is_last = (fi == n_fruits - 1)

            if fi == 0:
                # First segment: rachis from attachment point
                seg_len = RACHIS_SEG
            elif is_last:
                # Terminal fruit: no rachis segment, just a pedicel from tip
                seg_len = 0.0
            else:
                # Intermediate: rachis continues with a bend
                # Apply cumulative bend (RL in GroIMP = tilt further from Z)
                tilt_from_z += math.radians(bend_per_fruit)
                rach_dx = math.sin(tilt_from_z) * math.cos(truss_az)
                rach_dy = math.sin(tilt_from_z) * math.sin(truss_az)
                rach_dz = math.cos(tilt_from_z)
                seg_len = RACHIS_SEG

            # Draw rachis segment (if any)
            if seg_len > 0:
                seg_cx = cur_x + rach_dx * seg_len / 2.0
                seg_cy = cur_y + rach_dy * seg_len / 2.0
                seg_cz = cur_z + rach_dz * seg_len / 2.0
                seg_mat = _align_z_to(rach_dx, rach_dy, rach_dz, seg_cx, seg_cy, seg_cz)

                seg_prim = UsdGeom.Cylinder.Define(
                    stage, f"{truss_group}/Rachis_{seg_idx}")
                seg_prim.GetHeightAttr().Set(seg_len)
                seg_prim.GetRadiusAttr().Set(PEDICEL_R)
                seg_prim.GetAxisAttr().Set(UsdGeom.Tokens.z)
                _set_transform(seg_prim, seg_mat)
                _bind_material(seg_prim, materials["pedicel"])
                seg_idx += 1

                # Advance tip
                cur_x += rach_dx * seg_len
                cur_y += rach_dy * seg_len
                cur_z += rach_dz * seg_len

            # Pedicel branch to the fruit
            if is_last and n_fruits > 1:
                # Terminal fruit: pedicel continues in rachis direction
                ped_dx, ped_dy, ped_dz = rach_dx, rach_dy, rach_dz
            else:
                # Lateral pedicel: alternate sides (RU ±90°)
                sign = -1.0 if (fi % 2 == 0) else 1.0
                ped_dx = sign * lat_dx
                ped_dy = sign * lat_dy
                ped_dz = sign * lat_dz

            ped_cx = cur_x + ped_dx * PEDICEL_LEN / 2.0
            ped_cy = cur_y + ped_dy * PEDICEL_LEN / 2.0
            ped_cz = cur_z + ped_dz * PEDICEL_LEN / 2.0
            ped_mat = _align_z_to(ped_dx, ped_dy, ped_dz, ped_cx, ped_cy, ped_cz)

            ped_prim = UsdGeom.Cylinder.Define(
                stage, f"{truss_group}/Pedicel_{fi}")
            ped_prim.GetHeightAttr().Set(PEDICEL_LEN)
            ped_prim.GetRadiusAttr().Set(PEDICEL_R * 0.8)
            ped_prim.GetAxisAttr().Set(UsdGeom.Tokens.z)
            _set_transform(ped_prim, ped_mat)
            _bind_material(ped_prim, materials["pedicel"])

            # Fruit sphere at the tip of the pedicel
            fx = cur_x + ped_dx * (PEDICEL_LEN + r)
            fy = cur_y + ped_dy * (PEDICEL_LEN + r)
            fz = cur_z + ped_dz * (PEDICEL_LEN + r)

            sph = _make_sphere(stage, f"{truss_group}/Fruit_{fi}", r, fx, fy, fz)

            # Ripe vs unripe coloring based on fruit age (GroIMP logic)
            age = node.fruit_age_dd[fi] if fi < len(node.fruit_age_dd) else 0.0
            is_ripe = age >= node.ripening_dd
            fruit_mat_key = "fruit_ripe" if is_ripe else "fruit_unripe"
            _bind_material(sph, materials[fruit_mat_key])

            # Physics: fruit collider (toggle: ENABLE_FRUIT_PHYSICS in constants.py)
            if ENABLE_FRUIT_PHYSICS:
                fruit_mass = (4.0/3.0) * math.pi * r**3 * FRUIT_DENSITY_KG_M3
                _apply_rigid_body(stage, f"{truss_group}/Fruit_{fi}", fruit_mass, kinematic=True)

            state = "ripe" if is_ripe else "unripe"
            print(f"  [Fruit {fi}] r={r:.4f}m {state} (age={age:.0f}/{node.ripening_dd:.0f}dd) at ({fx:.4f},{fy:.4f},{fz:.4f})")

    print(f"\n  Max stem height: {max_z:.6f} m\n")

    # ── Save ─────────────────────────────────────────────────────────────────
    stage.GetRootLayer().Save()
    print(f"[USD] Saved → {output_path}")