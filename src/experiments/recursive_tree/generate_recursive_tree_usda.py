"""
generate_recursive_tree_usda.py

Generates a recursive articulated tree USD stage.

Structure (configurable via TREE_CONFIG in tree_config.py):
  Level 0 — Trunk   : vertical chain anchored to world
  Level 1 — Branches: attach to the top link of their parent chain, tilted
  Level 2 — Sub-branches: attach to the top link of their parent branch, tilted

Physics (Euler-Bernoulli, same as generate_cantilever_usda.py):
  K = E · I / L       [N·m/rad]
  D = 2ζ √(K · M)    [N·m·s/rad]
  computed per-level using world-unit (scaled) dimensions.

Run standalone to generate the USD file (no Isaac Sim required):
    python generate_recursive_tree_usda.py
"""

import os
import math
import sys

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# ---------------------------------------------------------------------------
# Local imports
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from tree_config import (
    GLOBAL_SCALE, BioConfig, TREE_CONFIG,
    compute_mass, calculate_physics_params, scaled,
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
    """Create a fresh USD stage with /World and /World/Stem (ArticulationRoot)."""
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
    stage: Usd.Stage,
    parent_path: str,
    name: str,
    radius: float,
    height: float,
    world_pos: Gf.Vec3d,
    orientation: Gf.Quatf,
    mass: float,
) -> str:
    """
    Create one rigid cylinder link under parent_path.

    The Xform is placed at world_pos with orientation. The visual/collision
    cylinder is a child offset by +height/2 along local Z so that the Xform
    origin sits at the BASE of the cylinder (matching joint attachment logic).
    """
    link_path = f"{parent_path}/{name}"

    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(world_pos)
    xform.AddOrientOp().Set(orientation)

    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)

    cyl_path = f"{link_path}/Cylinder"
    cyl = UsdGeom.Cylinder.Define(stage, cyl_path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))

    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    return link_path


def _configure_joint_drives(
    joint: UsdPhysics.Joint,
    stiff: float,
    damp: float,
    bend_limit_deg: float,
):
    """Lock translations, spring-drive rotX/rotY, lock rotZ."""
    for axis in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)   # low > high → locked

    for axis in ["rotX", "rotY"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(-bend_limit_deg)
        lim.CreateHighAttr().Set(bend_limit_deg)

        drv = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiff)
        drv.CreateDampingAttr().Set(damp)
        drv.CreateTargetPositionAttr().Set(0.0)

    lim_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    lim_z.CreateLowAttr().Set(1.0)
    lim_z.CreateHighAttr().Set(-1.0)     # locked


def anchor_link_to_world(stage: Usd.Stage, link_path: str) -> None:
    """Fix the first link of the trunk to the world (static anchor)."""
    joint = UsdPhysics.FixedJoint.Define(stage, f"{link_path}/RootFixedJoint")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def create_internal_joint(
    stage: Usd.Stage,
    parent_path: str,
    child_path: str,
    joint_name: str,
    parent_height: float,
    gap: float,
    stiff: float,
    damp: float,
) -> None:
    """
    D6 joint connecting two co-axial links in the same chain.
    LocalPos0 points from parent origin to its top face (+parent_height + gap along Z).
    LocalPos1 is at the child origin (base of child cylinder).
    """
    joint_path = f"{child_path}/{joint_name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    _configure_joint_drives(joint, stiff, damp, TREE_CONFIG["bend_limit_deg"])


def create_attachment_joint(
    stage: Usd.Stage,
    parent_link_path: str,
    child_link_path: str,
    joint_name: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
    stiff: float,
    damp: float,
) -> None:
    """
    D6 joint attaching a branch base to a parent link.

    local_pos0 and local_rot0 encode the attachment point + orientation in the
    parent frame.  LocalPos1 / LocalRot1 are always identity (base of child).
    """
    joint_path = f"{child_link_path}/{joint_name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])

    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    _configure_joint_drives(joint, stiff, damp, TREE_CONFIG["bend_limit_deg"])


# ==============================================================================
# CHAIN BUILDER
# ==============================================================================

def build_chain(
    stage: Usd.Stage,
    parent_usd_path: str,
    chain_name_prefix: str,
    level: int,
    start_world_pos: Gf.Vec3d,
    chain_orientation: Gf.Quatf,
    first_link_is_trunk_anchor: bool = False,
    parent_link_for_attachment: str = None,
    attachment_local_pos0: Gf.Vec3f = None,
    attachment_local_rot0: Gf.Quatf = None,
) -> tuple[str, Gf.Vec3d, Gf.Quatf]:
    """
    Build a chain of N rigid links at the given level.

    Returns:
        (last_link_path, last_link_world_top_pos, chain_orientation)
        where last_link_world_top_pos is the world-space top of the final link
        (used as start point for child branches).
    """
    cfg          = TREE_CONFIG
    gap          = scaled(cfg["gap"])
    n_links      = cfg["n_links_per_level"][level]
    r_world      = scaled(cfg["radius_per_level"][level])
    h_world      = scaled(cfg["height_per_level"][level])
    mass         = compute_mass(r_world, h_world)
    K, D         = calculate_physics_params(r_world, h_world, mass)

    # For the attachment joint (connecting this chain to its parent), use
    # a stiffer spring to avoid flapping at the branch root.
    K_attach = K * 5.0
    D_attach = D * 2.0

    # Rotation matrix for this chain's local Z axis
    rot_matrix = Gf.Matrix3d(chain_orientation)

    # Ensure a Xform container exists for this chain under parent_usd_path
    chain_container = f"{parent_usd_path}/{chain_name_prefix}"
    if not stage.GetPrimAtPath(chain_container):
        UsdGeom.Xform.Define(stage, chain_container)

    prev_link_path = None
    cur_world_pos  = start_world_pos

    for i in range(n_links):
        link_name = f"{chain_name_prefix}_Link_{i+1:02d}"
        link_path = create_rigid_segment(
            stage, chain_container, link_name,
            r_world, h_world, cur_world_pos, chain_orientation, mass,
        )

        if prev_link_path is None:
            # First link of this chain
            if first_link_is_trunk_anchor:
                # Trunk: fix to world
                anchor_link_to_world(stage, link_path)
            else:
                # Branch/sub-branch: attach to parent with stiff base joint
                create_attachment_joint(
                    stage,
                    parent_link_for_attachment,
                    link_path,
                    f"AttachJoint",
                    attachment_local_pos0,
                    attachment_local_rot0,
                    K_attach,
                    D_attach,
                )
        else:
            # Internal chain joint
            create_internal_joint(
                stage,
                prev_link_path,
                link_path,
                f"Joint_{i:02d}_{i+1:02d}",
                h_world,
                gap,
                K,
                D,
            )

        prev_link_path = link_path
        # Advance world position along chain's local Z
        step = rot_matrix * Gf.Vec3d(0.0, 0.0, h_world + gap)
        cur_world_pos = cur_world_pos + step

    # cur_world_pos is now just past the top of the last link (link top = cur - gap step)
    # Compute the exact top of the last link
    last_link_top = cur_world_pos - rot_matrix * Gf.Vec3d(0.0, 0.0, gap)

    return prev_link_path, last_link_top, chain_orientation


# ==============================================================================
# RECURSIVE TREE BUILDER
# ==============================================================================

def attach_branch_recursive(
    stage: Usd.Stage,
    stem_root_path: str,
    parent_link_path: str,
    parent_top_world: Gf.Vec3d,
    parent_orientation: Gf.Quatf,
    parent_height_world: float,
    level: int,
    child_idx: int,
    chain_id: str,
) -> None:
    """
    Recursively attach a branch chain at `level` to `parent_link_path`,
    then attach its own children at level+1.

    Args:
        parent_link_path    : USD path of the parent link to attach to
        parent_top_world    : world-space position of the parent link's top face
        parent_orientation  : quaternion of the parent chain (for axis transform)
        parent_height_world : height of the parent link (used for LocalPos0 Z)
        level               : current level index (1 = first branch, 2 = sub, …)
        child_idx           : index of this child among siblings (for azimuth offset)
        chain_id            : unique string identifier for USD naming
    """
    cfg  = TREE_CONFIG
    depth = cfg["depth"]

    if level >= depth:
        return

    # ---- Orientation of this branch in world space ----
    tilt_deg  = cfg["tilt_per_level"][level - 1]
    base_rot  = cfg["rot_per_level"][level - 1]
    n_siblings = cfg["children_per_level"][level - 1]
    azimuth   = base_rot + child_idx * (360.0 / max(n_siblings, 1))

    # Build the combined rotation:
    #   1. Rotate around world Z by azimuth  (chooses compass direction)
    #   2. Tilt away from parent's local Z by tilt_deg (around the new X)
    rot_z   = Gf.Rotation(Gf.Vec3d(0, 0, 1), azimuth)
    rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)
    combined = rot_z * rot_tilt
    branch_orientation = Gf.Quatf(combined.GetQuat())

    # ---- Attachment joint parameters (in parent frame) ----
    parent_h = parent_height_world
    gap      = scaled(cfg["gap"])

    # LocalPos0: top of parent link (+parent_height along parent local Z)
    local_pos0 = Gf.Vec3f(0.0, 0.0, parent_h + gap)

    # LocalRot0: orientation of this branch relative to parent frame
    # = combined rotation expressed in parent's local frame
    # Since parent frame may itself be rotated, we compose:
    parent_rot_inv = Gf.Rotation(Gf.Quatd(parent_orientation)).GetInverse()
    local_rot_gfd  = parent_rot_inv * combined
    local_rot0     = Gf.Quatf(local_rot_gfd.GetQuat())

    # ---- World start position for this branch ----
    # Branch starts at the top of the parent link
    branch_start_world = parent_top_world

    # ---- Build branch chain ----
    chain_prefix = f"Branch_{chain_id}"
    last_link, branch_top, branch_orient = build_chain(
        stage             = stage,
        parent_usd_path   = stem_root_path,
        chain_name_prefix = chain_prefix,
        level             = level,
        start_world_pos   = branch_start_world,
        chain_orientation = branch_orientation,
        first_link_is_trunk_anchor = False,
        parent_link_for_attachment = parent_link_path,
        attachment_local_pos0      = local_pos0,
        attachment_local_rot0      = local_rot0,
    )

    h_world_this = scaled(cfg["height_per_level"][level])

    print(f"[INFO] Level {level} chain '{chain_prefix}': "
          f"{cfg['n_links_per_level'][level]} links, "
          f"r={scaled(cfg['radius_per_level'][level]):.3f}m, "
          f"tilt={tilt_deg}°, azimuth={azimuth:.1f}°")

    # ---- Recurse into children ----
    if level + 1 < depth:
        n_children = cfg["children_per_level"][level]
        for cidx in range(n_children):
            child_id = f"{chain_id}_{cidx}"
            attach_branch_recursive(
                stage              = stage,
                stem_root_path     = stem_root_path,
                parent_link_path   = last_link,
                parent_top_world   = branch_top,
                parent_orientation = branch_orient,
                parent_height_world= h_world_this,
                level              = level + 1,
                child_idx          = cidx,
                chain_id           = child_id,
            )


# ==============================================================================
# TOP-LEVEL BUILD
# ==============================================================================

def build_stage(output_path: str) -> tuple:
    """
    Build the full recursive tree USD stage.

    Returns (stage, stem_path).
    """
    stage, stem_path = setup_base_stage(output_path)

    cfg    = TREE_CONFIG
    depth  = cfg["depth"]
    gap    = scaled(cfg["gap"])
    r0     = scaled(cfg["radius_per_level"][0])
    h0     = scaled(cfg["height_per_level"][0])
    mass0  = compute_mass(r0, h0)

    # ---- Trunk (level 0) ----
    print(f"[INFO] Building trunk: {cfg['n_links_per_level'][0]} links, "
          f"r={r0:.3f}m, h={h0:.3f}m")

    trunk_last, trunk_top, trunk_orient = build_chain(
        stage                      = stage,
        parent_usd_path            = stem_path,
        chain_name_prefix          = "Trunk",
        level                      = 0,
        start_world_pos            = Gf.Vec3d(0.0, 0.0, 0.0),
        chain_orientation          = Gf.Quatf(1.0, 0.0, 0.0, 0.0),
        first_link_is_trunk_anchor = True,
    )

    # ---- Branches (level 1 … depth-1) ----
    if depth > 1:
        n_children = cfg["children_per_level"][0]
        for cidx in range(n_children):
            attach_branch_recursive(
                stage               = stage,
                stem_root_path      = stem_path,
                parent_link_path    = trunk_last,
                parent_top_world    = trunk_top,
                parent_orientation  = trunk_orient,
                parent_height_world = h0,
                level               = 1,
                child_idx           = cidx,
                chain_id            = str(cidx),
            )

    return stage, stem_path


def main():
    output_path = get_output_usd_path()
    stage, stem_path = build_stage(output_path)
    stage.GetRootLayer().Save()

    # Count links for verification
    link_count = 0
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            link_count += 1

    expected = sum(
        TREE_CONFIG["n_links_per_level"][lvl] * (
            1 if lvl == 0 else
            int(math.prod(TREE_CONFIG["children_per_level"][:lvl]))
        )
        for lvl in range(TREE_CONFIG["depth"])
    )

    status = "✅" if link_count == expected else f"⚠️  expected {expected}"
    print(f"[OK] Saved: {output_path}")
    print(f"[OK] Rigid links in USD: {link_count} {status}")


if __name__ == "__main__":
    main()
