"""Capsule rigid links, joints, and junction filtering for skinned branches."""

from typing import Dict

from pxr import Gf, UsdGeom, UsdPhysics, Vt

from ..tree_config import GAP, scaled
from ..usd.collision import add_collision_filter
from ..usd.joints import (
    anchor_link_to_world,
    create_attachment_joint,
    create_attachment_joint_locked,
    create_attachment_revolute_joint,
    create_internal_joint,
    create_internal_joint_locked,
    create_internal_revolute_joint,
)
from .model import BranchData


def _create_capsule_proxy(
    stage,
    link_path: str,
    height: float,
    radius: float,
    index: int,
    collider_count: int,
    radius_scale: float,
    length_scale: float,
) -> None:
    chunk_length = height / float(collider_count)
    total_length = chunk_length * length_scale
    min_spine = 1e-5
    collider_radius = min(
        radius * radius_scale,
        max((total_length - min_spine) * 0.49, 1e-5),
    )
    spine_height = max(total_length - 2.0 * collider_radius, min_spine)
    half_total = 0.5 * (spine_height + 2.0 * collider_radius)

    capsule = UsdGeom.Capsule.Define(
        stage,
        f"{link_path}/Collider_{index + 1:02d}",
    )
    capsule.CreateAxisAttr().Set("Z")
    capsule.CreateRadiusAttr().Set(collider_radius)
    capsule.CreateHeightAttr().Set(spine_height)
    capsule.CreateExtentAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-collider_radius, -collider_radius, -half_total),
        Gf.Vec3f(collider_radius, collider_radius, half_total),
    ]))
    capsule.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, (index + 0.5) * chunk_length)
    )
    UsdGeom.Imageable(capsule.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())


def author_rigid_links(stage, branch: BranchData) -> None:
    """Author rigid bodies with invisible compound capsule collision proxies."""
    UsdGeom.Xform.Define(stage, branch.physics_root_path)
    collision_enabled = branch.definition.get("collision_enabled", True)
    for index, path in enumerate(branch.link_paths):
        link = UsdGeom.Xform.Define(stage, path)
        link.AddTranslateOp().Set(branch.link_bases[index])
        link.AddOrientOp().Set(branch.orientation)
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        rigid_body.CreateRigidBodyEnabledAttr().Set(True)
        mass_api = UsdPhysics.MassAPI.Apply(link.GetPrim())
        mass_api.CreateMassAttr().Set(branch.mass)
        mass_api.CreateCenterOfMassAttr().Set(
            Gf.Vec3f(0.0, 0.0, branch.link_height / 2.0)
        )
        if collision_enabled:
            for collider_index in range(branch.spec.colliders_per_link):
                _create_capsule_proxy(
                    stage,
                    path,
                    branch.link_height,
                    branch.radius,
                    collider_index,
                    branch.spec.colliders_per_link,
                    branch.spec.collider_radius_scale,
                    branch.spec.collider_length_scale,
                )


def _filter_pair(stage, path_a: str, path_b: str) -> None:
    add_collision_filter(stage, path_a, path_b)
    add_collision_filter(stage, path_b, path_a)


def author_branch_joints(
    stage,
    branch: BranchData,
    resolved_by_id: Dict[str, BranchData],
) -> None:
    """Author V2-compatible root, internal, and branch-attachment joints."""
    gap = scaled(GAP)
    if branch.parent_id is None:
        anchor_link_to_world(stage, branch.link_paths[0])
    else:
        parent = resolved_by_id[branch.parent_id]
        parent_path = parent.link_paths[branch.parent_link_index]
        child_path = branch.link_paths[0]
        mode = branch.attachment_joint_type
        if branch.locked_joints or mode == "fixed":
            create_attachment_joint_locked(
                stage,
                parent_path,
                child_path,
                branch.attachment_local_pos0,
                branch.attachment_local_rot0,
            )
        elif mode == "revolute_planar":
            create_attachment_revolute_joint(
                stage,
                parent_path,
                child_path,
                branch.attachment_local_pos0,
                branch.attachment_local_rot0,
                branch.gains.attachment_stiffness,
                branch.gains.attachment_damping,
            )
        else:
            create_attachment_joint(
                stage,
                parent_path,
                child_path,
                branch.attachment_local_pos0,
                branch.attachment_local_rot0,
                branch.gains.attachment_stiffness,
                branch.gains.attachment_damping,
                bend_axes=("rotX",) if mode == "d6_planar" else ("rotX", "rotY"),
                bend_limit_deg=branch.bend_limit_deg,
            )

        neighbor_indices = {
            branch.parent_link_index,
            max(0, branch.parent_link_index - 1),
            min(parent.n_links - 1, branch.parent_link_index + 1),
        }
        for index in neighbor_indices:
            _filter_pair(stage, child_path, parent.link_paths[index])

    for child_index in range(1, branch.n_links):
        parent_path = branch.link_paths[child_index - 1]
        child_path = branch.link_paths[child_index]
        joint_name = f"Joint_{child_index:02d}_{child_index + 1:02d}"
        if branch.locked_joints:
            create_internal_joint_locked(
                stage,
                parent_path,
                child_path,
                joint_name,
                branch.link_height,
                gap,
            )
        elif branch.joint_type == "revolute_planar":
            create_internal_revolute_joint(
                stage,
                parent_path,
                child_path,
                joint_name,
                branch.link_height,
                gap,
                branch.gains.stiffness,
                branch.gains.damping,
            )
        else:
            create_internal_joint(
                stage,
                parent_path,
                child_path,
                joint_name,
                branch.link_height,
                gap,
                branch.gains.stiffness,
                branch.gains.damping,
                bend_axes=(
                    ("rotX",)
                    if branch.joint_type == "d6_planar"
                    else ("rotX", "rotY")
                ),
                bend_limit_deg=branch.bend_limit_deg,
            )
