"""
joints.py - USD Joint Creation

Creates flexible (D6) and locked (Fixed) joints for articulated structures.
"""

from pxr import Usd, UsdPhysics, Gf, Sdf
from .collision import add_collision_filter, add_attachment_collision_filters


# BEND_LIMIT_DEG imported from tree_config at runtime
def _get_bend_limit():
    """Get BEND_LIMIT_DEG from tree_config."""
    import sys
    import os
    # Dynamically import tree_config
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    sys.path.insert(0, parent_dir)
    from tree_config import BEND_LIMIT_DEG
    return BEND_LIMIT_DEG


def configure_joint_drives(joint, stiff: float, damp: float) -> None:
    """Configure D6 joint drives: lock translations, spring-drive rotX/rotY, lock rotZ."""
    BEND_LIMIT_DEG = _get_bend_limit()
    
    # Lock all translations
    for axis in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    # Spring-drive on rotX/rotY (bending)
    for axis in ["rotX", "rotY"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(-BEND_LIMIT_DEG)
        lim.CreateHighAttr().Set(BEND_LIMIT_DEG)

        drv = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiff)
        drv.CreateDampingAttr().Set(damp)
        drv.CreateTargetPositionAttr().Set(0.0)

    # Lock rotZ (prevent twisting)
    lim_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    lim_z.CreateLowAttr().Set(1.0)
    lim_z.CreateHighAttr().Set(-1.0)


def anchor_link_to_world(stage, link_path: str) -> None:
    """Anchor root link to world with a FixedJoint."""
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
    """
    D6 bending joint between consecutive links in the same chain.

    LocalPos0 = top of parent link = (0, 0, parent_height + gap)
    LocalPos1 = base of child link = (0, 0, 0)
    Both LocalRot are identity (chain direction encoded in world position)
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_joint_drives(joint, stiff, damp)
    
    # Filter collision between parent and child
    add_collision_filter(stage, child_path, parent_path)


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
    D6 joint attaching first link of a branch to a parent chain link.

    Args:
        local_pos0: Attachment point in parent-link local frame
        local_rot0: Branch direction in parent-link local frame
        LocalPos1: Always (0,0,0) - base of child link
        LocalRot1: Always identity
    """
    joint = UsdPhysics.Joint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    configure_joint_drives(joint, stiff, damp)
    
    # Filter collisions with parent and its neighbor
    add_attachment_collision_filters(stage, child_link_path, parent_link_path)


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
    FixedJoint between consecutive links (for rigid testing).
    
    Used for Isaac Sim integration tests to verify geometry doesn't change
    when joints are completely rigid (no flexibility).
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_path}/{joint_name}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_height + gap))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    add_collision_filter(stage, child_path, parent_path)


def create_attachment_joint_locked(
    stage,
    parent_link_path: str,
    child_link_path: str,
    local_pos0: Gf.Vec3f,
    local_rot0: Gf.Quatf,
) -> None:
    """
    FixedJoint attaching branch to parent (for rigid testing).
    
    Creates completely rigid attachment with no flexibility.
    """
    joint = UsdPhysics.FixedJoint.Define(stage, f"{child_link_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link_path)])
    joint.CreateLocalPos0Attr().Set(local_pos0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rot0)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    add_attachment_collision_filters(stage, child_link_path, parent_link_path)
