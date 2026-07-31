"""
generate_recursive_tree_usda.py

Generates a tree USD stage from the explicit BRANCHES list in tree_config.py.

Each branch specifies its parent id and the 1-based index of the parent link
to attach to, allowing branches anywhere along a chain (not just the top).

Physics (Euler-Bernoulli):
    K = E * I / L    [N*m/rad]
    D = 2*zeta * sqrt(K*M)   [N*m*s/rad]

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
    stage,
    parent_path: str,
    name: str,
    radius: float,
    height: float,
    world_pos,
    orientation,
    mass: float,
) -> str:
    """
    Create one rigid cylinder link.
    The Xform origin is at the BASE of the cylinder (joint attachment point).
    The cylinder mesh child is offset +height/2 along local Z.
    """
    link_path = f"{parent_path}/{name}"

    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(world_pos)
    xform.AddOrientOp().Set(orientation)

    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)

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
        lim.CreateHighAttr().Set(-1.0)   # low > high -> locked

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
    lim_z.CreateHighAttr().Set(-1.0)     # locked


def anchor_link_to_world(stage, link_path: str) -> None:
    """Fix the root link of the trunk to the world."""
    joint = UsdPhysics.FixedJoint.Define(stage, f"{link_path}/RootFixedJoint")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


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
    """D6 joint between two co-axial links inside the same chain."""
    joint = UsdPhysics.Joint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    _configure_joint_drives(joint, stiff, damp)


def create_attachment_joint(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0,
    local_rot0,
    stiff: float,
    damp: float,
) -> None:
    """
    D6 joint connecting a branch base link to a specific link on the parent chain.
    local_pos0/rot0 are in the parent link's local frame.
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    _configure_joint_drives(joint, stiff, damp)


# ==============================================================================
# CHAIN BUILDER
# ==============================================================================

def build_chain(
    stage,
    stem_path: str,
    branch_def: dict,
    start_world_pos,
    chain_orientation,
    is_root: bool = False,
    parent_link_path: str = None,
    attachment_local_pos0=None,
    attachment_local_rot0=None,
):
    """
    Build one chain of n_links rigid segments.

    Returns:
        link_paths  : list of USD paths, one per link (index 0 = first/bottom link)
        chain_orient: Gf.Quatf of this chain's orientation (same for all links)
    """
    r_world = scaled(branch_def["radius"])
    h_world = scaled(branch_def["height"])
    gap     = scaled(GAP)
    n_links = branch_def["n_links"]
    mass    = compute_mass(r_world, h_world)
    K, D    = calculate_physics_params(r_world, h_world, mass)

    # Attachment joint uses 5x stiffer spring to prevent root flapping
    K_attach = K * 5.0
    D_attach = D * 2.0

    rot_matrix = Gf.Matrix3d(chain_orientation)

    # Container Xform for this chain under /World/Stem
    chain_id   = branch_def["id"]
    container  = f"{stem_path}/{chain_id}"
    if not stage.GetPrimAtPath(container):
        UsdGeom.Xform.Define(stage, container)

    link_paths    = []
    prev_link     = None
    cur_world_pos = start_world_pos

    for i in range(n_links):
        link_name = f"{chain_id}_Link_{i + 1:02d}"
        link_path = create_rigid_segment(
            stage, container, link_name,
            r_world, h_world, cur_world_pos, chain_orientation, mass,
        )

        if prev_link is None:
            if is_root:
                anchor_link_to_world(stage, link_path)
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
            create_internal_joint(
                stage, prev_link, link_path,
                f"Joint_{i:02d}_{i + 1:02d}",
                h_world, gap, K, D,
            )

        link_paths.append(link_path)
        prev_link = link_path
        cur_world_pos = cur_world_pos + rot_matrix * Gf.Vec3d(0.0, 0.0, h_world + gap)

    return link_paths, chain_orientation


# ==============================================================================
# TOP-LEVEL BUILD
# ==============================================================================

def build_stage(output_path: str, branches=None):
    """
    Build the full tree USD stage from the BRANCHES list.

    Returns (stage, stem_path).
    """
    if branches is None:
        branches = BRANCHES

    # Validate first — raises ValueError with a clear message on bad config
    validate_branches(branches)

    stage, stem_path = setup_base_stage(output_path)

    # Registry: branch_id -> list of link USD paths (index = link_index - 1)
    chain_registry = {}

    # Orientation registry: branch_id -> Gf.Quatf
    orient_registry = {}

    # World-position of each link's BASE: branch_id -> list of Gf.Vec3d
    pos_registry = {}

    for b in branches:
        bid         = b["id"]
        is_root     = b.get("parent") is None
        r_world     = scaled(b["radius"])
        h_world     = scaled(b["height"])
        gap         = scaled(GAP)

        if is_root:
            # Trunk: vertical, starts at world origin
            chain_orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            start_pos         = Gf.Vec3d(0.0, 0.0, 0.0)

            print(f"[INFO] Building '{bid}' (root): {b['n_links']} links, "
                  f"r={r_world:.3f}m, h={h_world:.3f}m")

            link_paths, chain_orient = build_chain(
                stage, stem_path, b,
                start_pos, chain_orientation,
                is_root=True,
            )

        else:
            parent_id   = b["parent"]
            attach_idx  = b["attach_link"] - 1   # convert to 0-based
            tilt_deg    = b["tilt"]
            rot_deg     = b["rot"]

            parent_links  = chain_registry[parent_id]
            parent_orient = orient_registry[parent_id]
            parent_pos    = pos_registry[parent_id]

            # World position of the attachment link's base
            attach_link_base_world = parent_pos[attach_idx]

            # Parent chain dimensions (from parent branch def)
            parent_def   = next(x for x in branches if x["id"] == parent_id)
            p_h_world    = scaled(parent_def["height"])

            # ---- Branch orientation in world space ----
            #   1. Azimuthal rotation around world Z
            #   2. Tilt away from parent's local Z
            rot_z    = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
            rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)
            combined = rot_z * rot_tilt
            chain_orientation = Gf.Quatf(combined.GetQuat())

            # ---- Attachment joint in parent local frame ----
            # LocalPos0: top face of the attachment link
            local_pos0 = Gf.Vec3f(0.0, 0.0, p_h_world + gap)

            # LocalRot0: branch orientation expressed in parent frame
            parent_rot_inv = Gf.Rotation(Gf.Quatd(parent_orient)).GetInverse()
            local_rot_gfd  = parent_rot_inv * combined
            local_rot0     = Gf.Quatf(local_rot_gfd.GetQuat())

            # World start pos of this branch = top of attachment link
            start_pos = attach_link_base_world + Gf.Matrix3d(parent_orient) * Gf.Vec3d(0.0, 0.0, p_h_world + gap)

            print(f"[INFO] Building '{bid}': {b['n_links']} links, "
                  f"r={r_world:.3f}m, h={h_world:.3f}m, "
                  f"parent='{parent_id}' link {b['attach_link']}, "
                  f"tilt={tilt_deg}deg, rot={rot_deg}deg")

            link_paths, chain_orient = build_chain(
                stage, stem_path, b,
                start_pos, chain_orientation,
                is_root=False,
                parent_link_path=parent_links[attach_idx],
                attachment_local_pos0=local_pos0,
                attachment_local_rot0=local_rot0,
            )

        # Register results
        chain_registry[bid]  = link_paths
        orient_registry[bid] = chain_orient

        # Compute world-space base position of every link in this chain
        rot_mat  = Gf.Matrix3d(chain_orient)
        h_w      = scaled(b["height"])
        g        = scaled(GAP)
        positions = []
        p = start_pos
        for _ in range(b["n_links"]):
            positions.append(p)
            p = p + rot_mat * Gf.Vec3d(0.0, 0.0, h_w + g)
        pos_registry[bid] = positions

    return stage, stem_path


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
