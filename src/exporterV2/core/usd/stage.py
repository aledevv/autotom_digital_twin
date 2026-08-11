"""
stage.py - USD Stage Setup and Orchestration

Top-level functions for building tree USD stages with articulated physics.
"""

import math
import os
import sys
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# Support both direct execution and module import
if __name__ == "__main__" or "exporterV2" not in sys.modules:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    SRC_DIR = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
    if SRC_DIR not in sys.path:
        sys.path.insert(0, SRC_DIR)
    from exporterV2.core.tree_config import (
        GLOBAL_SCALE, BRANCHES, GAP,
        compute_mass, calculate_physics_params, calculate_truss_physics_params, scaled,
        validate_branches, compute_flexural_rigidity, compute_hinge_stiffness_rad,
        BioConfig, TrussPhysicsConfig, PhysicsRuntimeConfig
    )
else:
    from ..tree_config import (
        GLOBAL_SCALE, BRANCHES, GAP,
        compute_mass, calculate_physics_params, calculate_truss_physics_params, scaled,
        validate_branches, compute_flexural_rigidity, compute_hinge_stiffness_rad,
        BioConfig, TrussPhysicsConfig, PhysicsRuntimeConfig
    )

from .geometry import create_rigid_segment, create_sphere_rigid_body
from .joints import (
    anchor_link_to_world,
    create_internal_joint,
    create_attachment_joint,
    create_internal_revolute_joint,
    create_attachment_revolute_joint,
    create_internal_joint_locked,
    create_attachment_joint_locked,
    create_fixed_joint_to_tip,
)
from .collision import (
    add_sibling_collision_filtering,
    check_sphere_sphere_intersection,
    check_sphere_cylinder_intersection,
)


def _branch_inner_radius_world(branch_def: dict) -> float:
    return scaled(branch_def.get("inner_radius", 0.0))


def _branch_density(branch_def: dict, use_truss_physics: bool = False) -> float:
    if "density" in branch_def:
        return branch_def["density"]
    if use_truss_physics:
        return TrussPhysicsConfig.PLANT_DENSITY
    return BioConfig.PLANT_DENSITY


def _branch_young_modulus(branch_def: dict, use_truss_physics: bool = False) -> float:
    if "young_modulus" in branch_def:
        return branch_def["young_modulus"]
    if use_truss_physics:
        return TrussPhysicsConfig.YOUNG_MODULUS
    return BioConfig.YOUNG_MODULUS


def _branch_damping_ratio(branch_def: dict, use_truss_physics: bool = False):
    if "damping_ratio" in branch_def:
        return branch_def["damping_ratio"]
    if use_truss_physics:
        return TrussPhysicsConfig.DAMPING_RATIO
    return None


def get_output_usd_path() -> str:
    """Get the default output path for generated USD file."""
    # Navigate from usd → exporterV2 → src → project_root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "tree_v2.usda")


def setup_base_stage(path: str, legacy_physics: bool = False):
    """Create or clear USD stage with World and Stem prims."""
    existing_layer = Sdf.Layer.Find(path)
    if existing_layer:
        existing_layer.Clear()
        stage = Usd.Stage.Open(existing_layer)
    else:
        if os.path.exists(path):
            try:
                os.remove(path)
            except Exception:
                pass
        stage = Usd.Stage.CreateNew(path)

    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    if not legacy_physics:
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    stem_path = "/World/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())

    return stage, stem_path


def validate_terminal_body_clearance(terminal_body_records, branch_registry, branches, margin: float = 0.002):
    """
    Warn about terminal body intersections before the simulation starts.
    """
    if not terminal_body_records:
        return

    warnings = []
    branch_defs = {b["id"]: b for b in branches}

    for i, body_a in enumerate(terminal_body_records):
        for body_b in terminal_body_records[i + 1:]:
            intersects, distance, overlap = check_sphere_sphere_intersection(
                body_a["pos"],
                body_a["radius"],
                body_b["pos"],
                body_b["radius"],
                margin,
            )
            if intersects:
                warnings.append(
                    f"terminal bodies '{body_a['id']}' and '{body_b['id']}' overlap "
                    f"by {overlap * 1000.0:.1f}mm (distance={distance * 1000.0:.1f}mm)"
                )

    for body in terminal_body_records:
        for branch_id, (_, link_bases, axis, _) in branch_registry.items():
            if branch_id == body["parent_branch_id"]:
                continue

            branch_def = branch_defs[branch_id]
            branch_radius = scaled(branch_def["radius"])
            branch_height = scaled(branch_def["height"])

            for link_idx, link_base in enumerate(link_bases):
                intersects, _, overlap = check_sphere_cylinder_intersection(
                    body["pos"],
                    body["radius"],
                    link_base,
                    axis,
                    branch_height,
                    branch_radius,
                    margin,
                )
                if intersects:
                    warnings.append(
                        f"terminal body '{body['id']}' intersects '{branch_id}_Link_{link_idx + 1:02d}' "
                        f"by {overlap * 1000.0:.1f}mm"
                    )

    if warnings:
        print("\n" + "=" * 80)
        print("  TERMINAL BODY GEOMETRY WARNINGS")
        print("=" * 80)
        for warning in warnings[:25]:
            print(f"[WARNING] {warning}")
        if len(warnings) > 25:
            print(f"[WARNING] ... {len(warnings) - 25} additional geometry warnings omitted")
        print("=" * 80 + "\n")
    else:
        print("[INFO] Terminal body geometry validation: no intersections detected")


def build_chain(
    stage,
    stem_path: str,
    branch_def: dict,
    start_world_pos: Gf.Vec3d,
    chain_axis: Gf.Vec3d,
    is_root: bool = False,
    parent_link_path: str = None,
    attachment_local_pos0: Gf.Vec3f = None,
    attachment_local_rot0: Gf.Quatf = None,
    chain_orientation: Gf.Quatf = None,
    locked_joints: bool = False,
    use_truss_physics: bool = False,
    parent_def: dict = None,
    legacy_physics: bool = False,
):
    """
    Build one chain of n_links rigid segments.

    Args:
        chain_axis: Unit vector in world space along chain axis
                    (0,0,1) for trunk, computed for tilted branches
        chain_orientation: World-space orientation quaternion
                          (None for trunk = vertical)
        locked_joints: If True, use FixedJoint instead of flexible D6
                      (can be overridden by branch_def["joint_type"])
        use_truss_physics: If True, use calculate_truss_physics_params instead of calculate_physics_params
                          (for rachis and pedicels with custom stiffness/damping)

    Returns:
        Tuple (link_paths, link_world_bases):
            link_paths: List of USD paths (index 0 = base link)
            link_world_bases: List of world-space base positions
    """
    # Check if branch has joint_type metadata (from optimization)
    # Priority: branch_def["joint_type"] > locked_joints parameter
    branch_joint_type = branch_def.get("joint_type", None)
    if branch_joint_type == "fixed":
        locked_joints = True
    elif branch_joint_type in {"d6", "d6_planar", "revolute_planar"}:
        locked_joints = False
    elif is_root and PhysicsRuntimeConfig.RIGID_TRUNK:
        locked_joints = True
    # else: use locked_joints parameter as-is
    
    r_world = scaled(branch_def["radius"])
    h_world = scaled(branch_def["height"])
    inner_radius_world = _branch_inner_radius_world(branch_def)
    gap     = scaled(GAP)
    n_links = branch_def["n_links"]
    bid     = branch_def["id"]
    density = _branch_density(branch_def, use_truss_physics=use_truss_physics)
    mass    = compute_mass(r_world, h_world, density=density, inner_radius=inner_radius_world)
    
    # Use truss physics if requested (for rachis/pedicels)
    if use_truss_physics:
        young_modulus = _branch_young_modulus(branch_def, use_truss_physics=True)
        damping_ratio = _branch_damping_ratio(branch_def, use_truss_physics=True)
        if "young_modulus" in branch_def or "inner_radius" in branch_def or damping_ratio is not None:
            K, D = calculate_physics_params(
                r_world,
                h_world,
                mass,
                legacy_physics=legacy_physics,
                young_modulus=young_modulus,
                damping_ratio=damping_ratio,
                inner_radius=inner_radius_world,
            )
        else:
            K, D = calculate_truss_physics_params(r_world, h_world, mass)
    else:
        young_modulus = _branch_young_modulus(branch_def)
        K, D = calculate_physics_params(
            r_world,
            h_world,
            mass,
            legacy_physics=legacy_physics,
            young_modulus=young_modulus,
            damping_ratio=_branch_damping_ratio(branch_def),
            inner_radius=inner_radius_world,
        )

    if not legacy_physics and not is_root and parent_def is not None:
        p_r_world = scaled(parent_def["radius"])
        p_h_world = scaled(parent_def["height"])
        p_inner_radius_world = _branch_inner_radius_world(parent_def)
        
        p_use_truss = parent_def.get("physics_profile") == "truss"
        p_ym = _branch_young_modulus(parent_def, use_truss_physics=p_use_truss)
        
        branch_EI = compute_flexural_rigidity(r_world, young_modulus, inner_radius_world)
        parent_EI = compute_flexural_rigidity(p_r_world, p_ym, p_inner_radius_world)
        
        K_attach_rad = compute_hinge_stiffness_rad(p_h_world, parent_EI, h_world, branch_EI)
        
        rad_to_deg = math.pi / 180.0
        K_attach_deg = K_attach_rad * rad_to_deg
        
        # Scale damping to maintain same damping ratio: D scales with sqrt(K)
        stiffness_ratio = K_attach_deg / K if K > 0 else 1.0
        D_attach_deg = D * math.sqrt(stiffness_ratio)
        
        K_attach = K_attach_deg
        D_attach = D_attach_deg
    else:
        # Fallback if no parent def is provided
        K_attach = K * 5.0
        D_attach = D * 2.236  # sqrt(5) ≈ 2.236

    if branch_def.get("attachment_stiffness_rad") is not None:
        K_attach_override = float(branch_def["attachment_stiffness_rad"]) * (3.141592653589793 / 180.0)
        stiffness_ratio = K_attach_override / K if K > 0 else 1.0
        K_attach = K_attach_override
        D_attach = D * math.sqrt(stiffness_ratio)

    drive_stiffness_scale = float(branch_def.get("drive_stiffness_scale", 1.0))
    if drive_stiffness_scale <= 0.0:
        raise ValueError(
            f"Branch '{bid}' drive_stiffness_scale must be positive, "
            f"got {drive_stiffness_scale}"
        )
    damping_scale = math.sqrt(drive_stiffness_scale)
    K *= drive_stiffness_scale
    D *= damping_scale
    K_attach *= drive_stiffness_scale
    D_attach *= damping_scale
    bend_limit_deg = branch_def.get("bend_limit_deg")

    step = chain_axis * (h_world + gap)

    link_paths       = []
    link_world_bases = []
    prev_link        = None
    cur_pos          = start_world_pos

    for i in range(n_links):
        link_name = f"{bid}_Link_{i + 1:02d}"
        link_path = create_rigid_segment(
            stage, stem_path, link_name,
            r_world, h_world, cur_pos, mass,
            orientation=chain_orientation,
            collision_enabled=branch_def.get("collision_enabled", True),
        )

        if prev_link is None:
            # First link in chain
            if is_root:
                anchor_link_to_world(stage, link_path)
            else:
                # Attach to parent chain
                if locked_joints or branch_def.get("attachment_joint_type") == "fixed":
                    create_attachment_joint_locked(
                        stage, parent_link_path, link_path,
                        attachment_local_pos0, attachment_local_rot0,
                    )
                elif branch_joint_type == "revolute_planar":
                    create_attachment_revolute_joint(
                        stage, parent_link_path, link_path,
                        attachment_local_pos0, attachment_local_rot0,
                        K_attach, D_attach,
                    )
                else:
                    create_attachment_joint(
                        stage, parent_link_path, link_path,
                        attachment_local_pos0, attachment_local_rot0,
                        K_attach, D_attach,
                        bend_axes=("rotX",) if branch_joint_type == "d6_planar" else ("rotX", "rotY"),
                        bend_limit_deg=bend_limit_deg,
                    )
        else:
            # Internal joint to previous link
            if locked_joints:
                create_internal_joint_locked(
                    stage, prev_link, link_path,
                    f"Joint_{i:02d}_{i + 1:02d}",
                    h_world, gap,
                )
            elif branch_joint_type == "revolute_planar":
                create_internal_revolute_joint(
                    stage, prev_link, link_path,
                    f"Joint_{i:02d}_{i + 1:02d}",
                    h_world, gap, K, D,
                )
            else:
                create_internal_joint(
                    stage, prev_link, link_path,
                    f"Joint_{i:02d}_{i + 1:02d}",
                    h_world, gap, K, D,
                    bend_axes=("rotX",) if branch_joint_type == "d6_planar" else ("rotX", "rotY"),
                    bend_limit_deg=bend_limit_deg,
                )

        link_paths.append(link_path)
        link_world_bases.append(cur_pos)
        prev_link = link_path
        cur_pos   = cur_pos + step

    return link_paths, link_world_bases


def build_stage(
    output_path: str,
    branches=None,
    locked_joints: bool = False,
    skip_limit_check: bool = False,
    terminal_bodies=None,
    legacy_physics: bool = False,
):
    """
    Build the full tree USD stage from BRANCHES configuration.
    
    Args:
        output_path: Path where to save the USD file
        branches: List of branch definitions (uses BRANCHES from tree_config if None)
        locked_joints: If True, use FixedJoint instead of flexible D6 joints
                      (for integration tests to verify rigid geometry)
        skip_limit_check: If True, skip the link count limit check
        terminal_bodies: Optional rigid bodies attached to branch tips. This is a
                         generic hook used by adapter-generated tomatoes.
    
    Returns:
        Tuple (stage, stem_path)
    """
    if branches is None:
        branches = BRANCHES
    if terminal_bodies is None:
        terminal_bodies = []

    validate_branches(branches, skip_limit_check=skip_limit_check)

    stage, stem_path = setup_base_stage(output_path, legacy_physics=legacy_physics)
    
    if legacy_physics:
        # Revert to legacy non-physics units to simulate original behavior
        from pxr import UsdGeom, UsdPhysics
        UsdGeom.SetStageMetersPerUnit(stage, 0.01) # fallback to default cm

    # Registry: branch_id → (link_paths, base_positions, axis_vector, orientation_quat)
    branch_registry = {}

    for b in branches:
        bid     = b["id"]
        is_root = b.get("parent") is None
        h_world = scaled(b["height"])
        r_world = scaled(b["radius"])
        gap     = scaled(GAP)

        if is_root:
            # Root trunk (vertical)
            chain_axis = Gf.Vec3d(0.0, 0.0, 1.0)
            start_pos  = Gf.Vec3d(0.0, 0.0, 0.0)

            print(f"[INFO] '{bid}' (root): {b['n_links']} links, "
                  f"r={r_world:.3f}m, h={h_world:.3f}m")

            link_paths, link_bases = build_chain(
                stage, stem_path, b,
                start_pos, chain_axis,
                is_root=True,
                chain_orientation=None,
                locked_joints=locked_joints,
            )
            
            branch_registry[bid] = (link_paths, link_bases, chain_axis, Gf.Quatf(1, 0, 0, 0))

        else:
            # Branch attached to parent
            parent_id  = b["parent"]
            attach_idx = b["attach_link"] - 1
            tilt_deg   = b["tilt"]
            rot_deg    = b["rot"]
            roll_deg   = b.get("roll", 0.0)

            parent_paths, parent_bases, parent_axis, parent_orientation = branch_registry[parent_id]
            parent_def = next(x for x in branches if x["id"] == parent_id)
            p_h_world  = scaled(parent_def["height"])
            p_r_world  = scaled(parent_def["radius"])
            
            # Compute branch orientation: rot_z → tilt → roll
            rot_z    = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
            rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)
            rot_roll = Gf.Rotation(Gf.Vec3d(0, 0, 1), roll_deg)
            branch_rot_in_parent_frame = rot_roll * rot_tilt * rot_z
            
            # FIX 2: For leaf-internal branches, always use parent orientation
            # (don't reset to identity, just don't add extra trunk rotation accumulation)
            parent_rot = Gf.Rotation(Gf.Quatd(parent_orientation))
            combined = branch_rot_in_parent_frame * parent_rot
            chain_axis_raw = combined.TransformDir(Gf.Vec3d(0, 0, 1))
            chain_axis     = Gf.Vec3d(*chain_axis_raw).GetNormalized()
            chain_orientation = Gf.Quatf(combined.GetQuat())

            # FIX 1: Zero radial offset for coaxial branches (tilt=0, rot=0)
            # Coaxial branches (like rachis continuing from petiole) should have
            # no radial offset - they continue in the same direction as the parent.
            # Non-coaxial branches (like petiole from trunk, or petiolules from rachis)
            # attach at the surface of the parent cylinder (radial offset = parent_radius/2).
            if tilt_deg == 0.0 and rot_deg == 0.0:
                radial_distance = 0.0
            else:
                radial_distance = p_r_world / 2.0

            # attach_frac: fractional position within the parent link [0.0, 1.0].
            #   1.0 (default) = top of link + small gap (original behaviour, coaxial seams)
            #   <1.0          = mid-link attachment (used by remapping; no gap needed,
            #                   branch emerges from the side of the cylinder)
            # This field is set by the remapping code after stem reduction.
            attach_frac = b.get("attach_frac", 1.0)
            if attach_frac >= 1.0:
                z_local = p_h_world + gap        # top of link + gap (default)
            else:
                z_local = attach_frac * p_h_world  # sub-link: exact fraction, no gap
            base_offset_local = Gf.Vec3d(0.0, radial_distance, z_local)
            
            rot_z_local = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
            offset_in_parent_frame = rot_z_local.TransformDir(base_offset_local)
            offset_in_world = parent_rot.TransformDir(offset_in_parent_frame)
            
            attach_base  = parent_bases[attach_idx]
            start_pos    = attach_base + offset_in_world

            # Joint frame in parent-link local frame
            local_pos0 = Gf.Vec3f(
                offset_in_parent_frame[0],
                offset_in_parent_frame[1],
                offset_in_parent_frame[2]
            )
            local_rot0 = Gf.Quatf(branch_rot_in_parent_frame.GetQuat())

            print(f"[INFO] '{bid}': {b['n_links']} links, "
                  f"r={r_world:.3f}m, h={h_world:.3f}m, "
                  f"parent='{parent_id}' link {b['attach_link']}, "
                  f"tilt={tilt_deg}deg, rot={rot_deg}deg, roll={roll_deg}deg")

            link_paths, link_bases = build_chain(
                stage, stem_path, b,
                start_pos, chain_axis,
                is_root=False,
                parent_link_path=parent_paths[attach_idx],
                attachment_local_pos0=local_pos0,
                attachment_local_rot0=local_rot0,
                chain_orientation=chain_orientation,
                locked_joints=locked_joints,
                use_truss_physics=b.get("physics_profile") == "truss",
                parent_def=parent_def,
                legacy_physics=legacy_physics,
            )
            
            branch_registry[bid] = (link_paths, link_bases, chain_axis, chain_orientation)

    # Attach generic terminal bodies, such as tomatoes generated by the GroIMP adapter.
    branch_defs = {b["id"]: b for b in branches}
    terminal_body_records = []

    for body in terminal_bodies:
        shape = body.get("shape", "sphere")
        if shape != "sphere":
            print(f"[WARNING] Skipping terminal body '{body.get('id')}' with unsupported shape '{shape}'")
            continue

        parent_branch_id = body.get("parent_branch_id")
        if parent_branch_id not in branch_registry:
            print(
                f"[WARNING] Skipping terminal body '{body.get('id')}' because parent branch "
                f"'{parent_branch_id}' was not built"
            )
            continue

        radius = scaled(body["radius"])
        mass = body["mass"]
        if radius <= 0.0 or mass <= 0.0:
            print(f"[WARNING] Skipping terminal body '{body.get('id')}' with invalid radius or mass")
            continue

        parent_paths, parent_bases, parent_axis, parent_orientation = branch_registry[parent_branch_id]
        parent_def = branch_defs[parent_branch_id]
        parent_height = scaled(parent_def["height"])
        parent_link_path = parent_paths[-1]
        parent_base = parent_bases[-1]
        body_pos = parent_base + parent_axis * (parent_height + radius)

        body_path = create_sphere_rigid_body(
            stage,
            stem_path,
            body["id"],
            radius,
            body_pos,
            mass,
            orientation=parent_orientation,
        )
        create_fixed_joint_to_tip(
            stage,
            parent_link_path,
            body_path,
            parent_height=parent_height,
            child_offset=radius,
            joint_name="TerminalBodyFixedJoint",
        )

        terminal_body_records.append({
            "id": body["id"],
            "parent_branch_id": parent_branch_id,
            "pos": (body_pos[0], body_pos[1], body_pos[2]),
            "radius": radius,
        })

        print(
            f"[INFO] terminal body '{body['id']}': sphere r={radius:.3f}m, "
            f"parent='{parent_branch_id}'"
        )

    validate_terminal_body_clearance(terminal_body_records, branch_registry, branches)

    # Add sibling collision filtering
    add_sibling_collision_filtering(stage, branches, branch_registry)

    return stage, stem_path


def build_stage_locked(output_path: str, branches=None):
    """
    Convenience wrapper for build_stage() with locked_joints=True.
    
    Creates a USD stage where all joints are FixedJoint (completely rigid).
    Used for Isaac Sim integration tests to verify geometry doesn't change
    during simulation when joints have no flexibility.
    
    Args:
        output_path: Path where to save the USD file
        branches: List of branch definitions (uses BRANCHES if None)
    
    Returns:
        Tuple (stage, stem_path)
    
    Example:
        stage, stem_path = build_stage_locked("test_locked.usda")
    """
    return build_stage(output_path, branches, locked_joints=True)
