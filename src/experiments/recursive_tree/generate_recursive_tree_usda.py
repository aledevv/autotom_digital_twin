"""
generate_recursive_tree_usda.py

Generates an articulated tree USD stage from the explicit BRANCHES list in tree_config.py.

Each branch specifies its parent id and the 1-based index of the parent link
to attach to, allowing branches anywhere along a chain (not just the top).

Key design:
- All links live directly under /World/Stem (no intermediate container Xforms).
  This matches the cantilever pattern and avoids USD local-vs-world transform confusion.
- Links use only AddTranslateOp (world-space position for initial render pose).
- All orientation/angle information lives exclusively in joint LocalPos0/LocalRot0.

Physics (Euler-Bernoulli):
    K = E * I / L      [N*m/rad]
    D = 2*zeta*sqrt(K*M)  [N*m*s/rad]

Run standalone (no Isaac Sim required):
    cd <project_root>
    env -i HOME=$HOME PATH=$PATH uv run src/experiments/recursive_tree/generate_recursive_tree_usda.py
"""

import os
import math
import sys

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from tree_config import (
    GLOBAL_SCALE, BioConfig, BRANCHES,
    BEND_LIMIT_DEG, GAP,
    compute_mass, calculate_physics_params, scaled,
    validate_branches,
)


# ==============================================================================
# OUTPUT PATH
# ==============================================================================

def get_output_usd_path() -> str:
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "recursive_tree.usda")


# ==============================================================================
# STAGE SETUP
# ==============================================================================

def setup_base_stage(path: str):
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

    stem_path = "/World/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())

    return stage, stem_path


# ==============================================================================
# PRIMITIVE HELPERS
# ==============================================================================

def create_rigid_segment(
    stage,
    stem_path: str,
    link_name: str,
    radius: float,
    height: float,
    world_pos: Gf.Vec3d,
    mass: float,
    orientation: Gf.Quatf = None,
) -> str:
    """
    Create one rigid cylinder link directly under stem_path.

    - Xform at world_pos with optional orientation.
    - Trunk links: translate only (vertical = identity orientation).
    - Branch links: translate + orient (world-space rotation quaternion).
    - Cylinder child offset +height/2 along local Z for visual centering.
    - RigidBody + MassAPI + CollisionAPI applied.

    Returns the USD path of the link Xform.
    """
    link_path = f"{stem_path}/{link_name}"

    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.AddTranslateOp().Set(world_pos)
    if orientation is not None:
        xform.AddOrientOp().Set(orientation)

    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)
    # Explicitly set center of mass to cylinder's geometric center (offset along Z)
    # Without this, PhysX may assume COM at link origin (base) instead of geometric center,
    # causing spurious torques on inclined branches
    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, height / 2.0))

    cyl = UsdGeom.Cylinder.Define(stage, f"{link_path}/Cylinder")
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))

    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    return link_path


def _configure_joint_drives(joint, stiff: float, damp: float) -> None:
    """Lock translations, spring-drive rotX/rotY, lock rotZ."""
    for axis in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    for axis in ["rotX", "rotY"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(-BEND_LIMIT_DEG)
        lim.CreateHighAttr().Set(BEND_LIMIT_DEG)

        drv = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiff)
        drv.CreateDampingAttr().Set(damp)
        drv.CreateTargetPositionAttr().Set(0.0)

    lim_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    lim_z.CreateLowAttr().Set(1.0)
    lim_z.CreateHighAttr().Set(-1.0)


def anchor_link_to_world(stage, link_path: str) -> None:
    joint = UsdPhysics.FixedJoint.Define(stage, f"{link_path}/RootFixedJoint")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def _add_collision_filtering(stage, child_link_path: str, parent_link_path: str) -> None:
    """
    Add collision filtering between parent and child links.
    
    Applies FilteredPairsAPI at RigidBody level (child Xform → parent Xform).
    The filtering automatically propagates to child collision shapes (Cylinder).
    """
    parent_prim = stage.GetPrimAtPath(parent_link_path)
    child_prim = stage.GetPrimAtPath(child_link_path)
    if parent_prim and child_prim:
        filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(child_prim)
        filtered_pairs.GetFilteredPairsRel().AddTarget(Sdf.Path(parent_link_path))


def _add_sibling_collision_filtering(stage, branches, branch_registry) -> None:
    """
    Add collision filtering between sibling branches (branches attached to same parent link).
    
    When multiple branches attach to the same parent link, they need to filter
    collisions with each other to prevent spurious contact forces.
    
    Example:
        main_petiole Link_02 (parent)
             ├─ petiolule_1 (child 1)
             ├─ petiolule_2 (child 2)  ← These must filter each other!
             ├─ petiolule_3 (child 3)
             └─ ... (etc)
    """
    # Build map: (parent_id, attach_link_idx) → [list of child branch first links]
    attachment_map = {}
    
    for b in branches:
        if b.get("parent") is None:
            continue  # Skip root
        
        parent_id = b["parent"]
        attach_idx = b["attach_link"] - 1  # Convert to 0-based
        key = (parent_id, attach_idx)
        
        # Get first link path of this branch
        link_paths, _, _, _ = branch_registry[b["id"]]
        first_link = link_paths[0]
        
        if key not in attachment_map:
            attachment_map[key] = []
        attachment_map[key].append(first_link)
    
    # Now for each attachment point with multiple children, filter them pairwise
    filtered_count = 0
    for (parent_id, attach_idx), sibling_links in attachment_map.items():
        if len(sibling_links) <= 1:
            continue  # No siblings, skip
        
        # Filter each sibling with all other siblings
        for i, link_a in enumerate(sibling_links):
            for link_b in sibling_links[i+1:]:  # Only pairs, avoid duplicates
                prim_a = stage.GetPrimAtPath(link_a)
                prim_b = stage.GetPrimAtPath(link_b)
                
                if prim_a and prim_b:
                    # Add bidirectional filtering
                    filtered_pairs_a = UsdPhysics.FilteredPairsAPI.Apply(prim_a)
                    filtered_pairs_a.GetFilteredPairsRel().AddTarget(Sdf.Path(link_b))
                    
                    filtered_pairs_b = UsdPhysics.FilteredPairsAPI.Apply(prim_b)
                    filtered_pairs_b.GetFilteredPairsRel().AddTarget(Sdf.Path(link_a))
                    
                    filtered_count += 2
    
    if filtered_count > 0:
        print(f"[INFO] Added {filtered_count} sibling collision filters")


def _add_collision_filtering_with_neighbors(stage, child_link_path: str, parent_link_path: str) -> None:
    """
    Add collision filtering for attachment joints.
    
    When a branch attaches to a parent chain, it needs to filter collisions with:
    1. The parent link it attaches to
    2. The NEXT link in the parent chain (above the attachment point)
    
    This prevents the branch from colliding with the link above the attachment point,
    which can cause instability (found via manual testing in Isaac Sim).
    
    Example:
        stem_Link_03 (parent - attachment point)
             |
             └─ petiole_1_Link_01 (child branch)
             |
        stem_Link_04 (next sibling - ALSO needs filtering!)
    
    Without filtering stem_Link_04, the petiole can collide with it → instability.
    """
    # First, filter the parent link
    _add_collision_filtering(stage, child_link_path, parent_link_path)
    
    # Now find the next sibling link in the parent chain
    # Parent link naming convention: {branch_id}_Link_{N:02d}
    # We need to find Link_{N+1:02d} with same branch_id
    
    import re
    match = re.match(r'(.*/(\w+)_Link_(\d+))$', parent_link_path)
    if match:
        parent_base = match.group(1)
        branch_id = match.group(2)
        link_num = int(match.group(3))
        
        # Try to find next link (N+1)
        next_link_num = link_num + 1
        next_link_path = f"{parent_base[:-len(str(link_num).zfill(2))]}{next_link_num:02d}"
        
        next_link_prim = stage.GetPrimAtPath(next_link_path)
        if next_link_prim and next_link_prim.IsValid():
            # Filter this next link too
            child_prim = stage.GetPrimAtPath(child_link_path)
            if child_prim:
                filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(child_prim)
                rel = filtered_pairs.GetFilteredPairsRel()
                
                targets = list(rel.GetTargets())
                if Sdf.Path(next_link_path) not in targets:
                    targets.append(Sdf.Path(next_link_path))
                
                rel.SetTargets(targets)


def create_internal_joint(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_height: float,
    gap: float,
    stiff: float,
    damp: float,
) -> None:
    """
    D6 bending joint between consecutive links in the same chain.

    LocalPos0 = top of parent link in parent-link local frame = (0, 0, h + gap).
    LocalPos1 = base of child link = (0, 0, 0).
    Both LocalRot are identity — the chain direction is encoded in each link's
    world position (set by the Xform translate), not in the joint rotation.
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    _configure_joint_drives(joint, stiff, damp)
    
    # Add collision filtering (both RigidBody and collision shape level)
    _add_collision_filtering(stage, child_path, parent_path)


def create_attachment_joint(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
    stiff: float,
    damp: float,
) -> None:
    """
    D6 joint attaching the first link of a branch to a specific link on the parent chain.

    local_pos0 : attachment point in parent-link local frame (top of parent link).
    local_rot0 : branch direction expressed in parent-link local frame.
    LocalPos1  : always (0,0,0) — base of the first child link.
    LocalRot1  : always identity.
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    _configure_joint_drives(joint, stiff, damp)
    
    # Add collision filtering (parent + next sibling in parent chain)
    _add_collision_filtering_with_neighbors(stage, child_link_path, parent_link_path)


# ==============================================================================
# LOCKED JOINT HELPERS (for testing)
# ==============================================================================

def create_internal_joint_locked(
    stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_height: float,
    gap: float,
) -> None:
    """
    FixedJoint between consecutive links in the same chain.
    
    Used for Isaac Sim integration tests to verify that geometry doesn't change
    when joints are completely rigid (no flexibility at all).
    
    LocalPos0/LocalPos1 are the same as the flexible D6 joint, but this creates
    a FixedJoint instead, which is completely rigid with no drives needed.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    # Add collision filtering (both RigidBody and collision shape level)
    _add_collision_filtering(stage, child_path, parent_path)


def create_attachment_joint_locked(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
) -> None:
    """
    FixedJoint attaching the first link of a branch to a parent link.
    
    Used for Isaac Sim integration tests. Creates a completely rigid attachment
    with no flexibility.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    # Add collision filtering (parent + next sibling in parent chain)
    _add_collision_filtering_with_neighbors(stage, child_link_path, parent_link_path)


# ==============================================================================
# CHAIN BUILDER
# ==============================================================================

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
):
    """
    Build one chain of n_links rigid segments directly under stem_path.

    chain_axis : unit vector in world space pointing along this chain's axis.
                 For trunk: (0,0,1). For a branch tilted 45deg: computed by caller.
    chain_orientation : world-space orientation quaternion for branch links.
                        None for trunk (vertical = identity).
    locked_joints : if True, use FixedJoint instead of flexible D6 joints.
                    Used for Isaac Sim integration tests.

    Returns:
        link_paths : list[str], USD paths ordered bottom to top (index 0 = base link)
        link_world_bases : list[Gf.Vec3d], world-space base position of each link
    """
    r_world = scaled(branch_def["radius"])
    h_world = scaled(branch_def["height"])
    gap     = scaled(GAP)
    n_links = branch_def["n_links"]
    bid     = branch_def["id"]
    mass    = compute_mass(r_world, h_world)
    K, D    = calculate_physics_params(r_world, h_world, mass)

    # Attachment joint: stiffer to handle branch connection
    # Scale damping by sqrt(5) to maintain same damping ratio ζ
    # (since ζ = D / (2*sqrt(K*J)), and K is scaled by 5)
    K_attach = K * 5.0
    D_attach = D * 2.236  # sqrt(5) ≈ 2.236

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
        )

        if prev_link is None:
            if is_root:
                anchor_link_to_world(stage, link_path)
            else:
                if locked_joints:
                    create_attachment_joint_locked(
                        stage,
                        parent_link_path,
                        link_path,
                        attachment_local_pos0,
                        attachment_local_rot0,
                    )
                else:
                    create_attachment_joint(
                        stage,
                        parent_link_path,
                        link_path,
                        attachment_local_pos0,
                        attachment_local_rot0,
                        K_attach,
                        D_attach,
                    )
        else:
            if locked_joints:
                create_internal_joint_locked(
                    stage, prev_link, link_path,
                    f"Joint_{i:02d}_{i + 1:02d}",
                    h_world, gap,
                )
            else:
                create_internal_joint(
                    stage, prev_link, link_path,
                    f"Joint_{i:02d}_{i + 1:02d}",
                    h_world, gap, K, D,
                )

        link_paths.append(link_path)
        link_world_bases.append(cur_pos)
        prev_link = link_path
        cur_pos   = cur_pos + step

    return link_paths, link_world_bases


# ==============================================================================
# TOP-LEVEL BUILD
# ==============================================================================

def build_stage(output_path: str, branches=None, locked_joints: bool = False, skip_limit_check: bool = False):
    """
    Build the full tree USD stage from the BRANCHES list.
    
    Args:
        output_path: Path where to save the USD file
        branches: List of branch definitions (uses BRANCHES from tree_config if None)
        locked_joints: If True, use FixedJoint instead of flexible D6 joints.
                      Used for Isaac Sim integration tests to verify geometry
                      doesn't change when joints are completely rigid.
        skip_limit_check: If True, skip the 64-link PhysX limit check (for experimental tests)
    
    Returns:
        (stage, stem_path) tuple
    """
    if branches is None:
        branches = BRANCHES

    validate_branches(branches, skip_limit_check=skip_limit_check)

    stage, stem_path = setup_base_stage(output_path)

    # Consolidated registry: branch_id -> (link_paths, base_positions, axis_vector, orientation_quat)
    branch_registry = {}

    for b in branches:
        bid     = b["id"]
        is_root = b.get("parent") is None
        h_world = scaled(b["height"])
        r_world = scaled(b["radius"])
        gap     = scaled(GAP)

        if is_root:
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
            parent_id  = b["parent"]
            attach_idx = b["attach_link"] - 1
            tilt_deg   = b["tilt"]
            rot_deg    = b["rot"]
            roll_deg   = b.get("roll", 0.0)  # New: roll around branch's own axis

            parent_paths, parent_bases, parent_axis, parent_orientation = branch_registry[parent_id]
            parent_def = next(x for x in branches if x["id"] == parent_id)
            p_h_world  = scaled(parent_def["height"])
            p_r_world  = scaled(parent_def["radius"])
            
            # Compute branch orientation relative to parent's frame
            # Order: rot_z (azimuthal) → rot_tilt (polar) → rot_roll (around branch axis)
            rot_z    = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)      # Step 1: rotate around parent's Z (azimuthal)
            rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)    # Step 2: tilt away from parent
            rot_roll = Gf.Rotation(Gf.Vec3d(0, 0, 1), roll_deg)     # Step 3: roll around branch's own axis
            branch_rot_in_parent_frame = rot_roll * rot_tilt * rot_z
            
            parent_rot = Gf.Rotation(Gf.Quatd(parent_orientation))
            combined = branch_rot_in_parent_frame * parent_rot
            
            chain_axis_raw = combined.TransformDir(Gf.Vec3d(0, 0, 1))
            chain_axis     = Gf.Vec3d(*chain_axis_raw).GetNormalized()
            chain_orientation = Gf.Quatf(combined.GetQuat())

            # Compute radial offset for branch attachment in parent's local frame
            radial_distance = p_r_world / 2.0
            base_offset_local = Gf.Vec3d(0.0, radial_distance, p_h_world + gap)
            
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
            )
            
            branch_registry[bid] = (link_paths, link_bases, chain_axis, chain_orientation)

    # Add sibling collision filtering (branches attached to same parent link)
    _add_sibling_collision_filtering(stage, branches, branch_registry)

    return stage, stem_path


def build_stage_locked(output_path: str, branches=None):
    """
    Convenience wrapper for build_stage() with locked_joints=True.
    
    Creates a USD stage where all joints are FixedJoint (completely rigid).
    Used for Isaac Sim integration tests to verify that geometry doesn't
    change during simulation when joints have no flexibility.
    
    Args:
        output_path: Path where to save the USD file
        branches: List of branch definitions (uses BRANCHES from tree_config if None)
    
    Returns:
        (stage, stem_path) tuple
    
    Example:
        stage, stem_path = build_stage_locked("test_locked.usda")
        # All joints will be FixedJoint - no bending possible
    """
    return build_stage(output_path, branches, locked_joints=True)


def main():
    output_path = get_output_usd_path()
    stage, stem_path = build_stage(output_path)
    stage.GetRootLayer().Save()

    link_count = sum(
        1 for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    )
    expected = sum(b["n_links"] for b in BRANCHES)
    status = "OK" if link_count == expected else f"MISMATCH expected {expected}"
    print(f"[OK] Saved: {output_path}")
    print(f"[OK] Rigid links in USD: {link_count} [{status}]")


if __name__ == "__main__":
    main()
