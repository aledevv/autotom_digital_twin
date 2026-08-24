"""OpenUSD authoring and deterministic audit for canonical ExporterV2 stages."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import re
from typing import Any

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

from plant_state import FruitsProperties

from .core.tree_config import (
    BEND_LIMIT_DEG,
    BioConfig,
    PhysicsRuntimeConfig,
    PlantColors,
    TrussPhysicsConfig,
    calculate_physics_params,
    compute_flexural_rigidity,
    compute_hinge_stiffness_rad,
    compute_mass,
)
from .core.physics import (
    apply_physx_articulation_settings,
    apply_physx_rigid_body_solver_settings,
    apply_physx_scene_settings,
)
from .core.usd.collision import add_collision_filter
from .core.usd.joints import configure_detachable_joint, configure_joint_drives
from .core.usd.materials import (
    get_or_create_tomato_fruit_material,
    get_or_create_tomato_leaf_material,
    get_or_create_tomato_stem_material,
)
from .core.skinning.leaf_blade import (
    LEAF_ARCH_LIFT_FRACTION,
    LEAF_HALF_WIDTH_FRACTION,
    LEAF_LENGTH_FRACTION,
    LEAF_LONGITUDINAL_FOLD_FRACTION,
    LEAF_TIP_SAG_FRACTION,
    author_leaf_blade,
)
from .plant_state_adapter import (
    COLLIDER_LENGTH_SCALE,
    V2_MANIFEST_SCHEMA_VERSION,
    Pose,
    V2AuthoringPlan,
)


class V2ExportError(ValueError):
    """Raised when a generated stage fails completeness or physics audit."""


@dataclass(frozen=True)
class V2ExportManifest:
    metadata: dict[str, Any]
    canonical: dict[str, Any]
    visual: dict[str, Any]
    physics: dict[str, Any]
    collisions: dict[str, Any]
    topology: dict[str, Any]
    errors: tuple[str, ...] = ()
    schema_version: str = V2_MANIFEST_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _safe(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if not result or result[0].isdigit():
        result = f"Item_{result}"
    return result


def _typed_prim_name(entity_type: str, canonical_id: str) -> str:
    """Keep the Stage tree human-readable without weakening canonical identity."""

    return f"{_safe(entity_type)}_{_safe(canonical_id)}"


def _string(prim, name: str, value: str) -> None:
    prim.CreateAttribute(name, Sdf.ValueTypeNames.String, custom=True).Set(str(value))


def _strings(prim, name: str, values) -> None:
    prim.CreateAttribute(name, Sdf.ValueTypeNames.StringArray, custom=True).Set(
        Vt.StringArray([str(value) for value in values])
    )


def _quat_from_rotation(rotation) -> Gf.Quatf:
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x, y, z = 0.25 * scale, (matrix[0, 1] + matrix[1, 0]) / scale, (
                matrix[0, 2] + matrix[2, 0]
            ) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x, y, z = (matrix[0, 1] + matrix[1, 0]) / scale, 0.25 * scale, (
                matrix[1, 2] + matrix[2, 1]
            ) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x, y, z = (matrix[0, 2] + matrix[2, 0]) / scale, (
                matrix[1, 2] + matrix[2, 1]
            ) / scale, 0.25 * scale
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    return Gf.Quatf(w / norm, x / norm, y / norm, z / norm)


def _pose_matrix(pose: Pose) -> Gf.Matrix4d:
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(Gf.Rotation(Gf.Quatd(_quat_from_rotation(pose.rotation))), Gf.Vec3d(*pose.start))
    return matrix


def _set_pose(xform: UsdGeom.Xform, pose: Pose) -> None:
    xform.AddTranslateOp().Set(Gf.Vec3d(*pose.start))
    xform.AddOrientOp().Set(_quat_from_rotation(pose.rotation))


def _role_color(role: str):
    if role in {"truss_rachis", "pedicel"}:
        return PlantColors.TRUSS_RACHIS if role == "truss_rachis" else PlantColors.PEDICEL
    if role in {
        "petiole",
        "leaf_rachis",
        "petiolule_left",
        "petiolule_right",
        "rachis_terminal",
    }:
        return PlantColors.PETIOLE
    return PlantColors.STEM


def _author_axis_visual(stage, path: str, length: float, radius: float, role: str) -> None:
    cylinder = UsdGeom.Cylinder.Define(stage, f"{path}/Visual")
    cylinder.CreateAxisAttr().Set(UsdGeom.Tokens.z)
    cylinder.CreateHeightAttr().Set(float(length))
    cylinder.CreateRadiusAttr().Set(float(radius))
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, length / 2.0))
    cylinder.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*_role_color(role))]))
    material = get_or_create_tomato_stem_material(stage)
    UsdShade.MaterialBindingAPI.Apply(cylinder.GetPrim()).Bind(material)


def _author_collider(stage, link_path: str, length: float, radius: float) -> None:
    total = max(length * COLLIDER_LENGTH_SCALE, 1e-5)
    radius = min(radius, max(total * 0.49, 1e-5))
    spine = max(total - 2.0 * radius, 1e-5)
    capsule = UsdGeom.Capsule.Define(stage, f"{link_path}/Collider")
    capsule.CreateAxisAttr().Set(UsdGeom.Tokens.z)
    capsule.CreateRadiusAttr().Set(float(radius))
    capsule.CreateHeightAttr().Set(float(spine))
    capsule.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, length / 2.0))
    UsdGeom.Imageable(capsule.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
    _string(capsule.GetPrim(), "autotom:entityKind", "collider")


def _local_pose(child: Pose, parent: Pose) -> tuple[Gf.Vec3f, Gf.Quatf]:
    parent_rotation = np.asarray(parent.rotation)
    position = parent_rotation.T @ (
        np.asarray(child.start) - np.asarray(parent.start)
    )
    rotation = parent_rotation.T @ np.asarray(child.rotation)
    return Gf.Vec3f(*position), _quat_from_rotation(rotation)


DRIVE_SCALE_CHOICES = (1.0, 0.5, 0.25, 0.1)


def _material_properties(link) -> tuple[float, float, float]:
    is_truss = link.role in {"truss_rachis", "pedicel"}
    density = TrussPhysicsConfig.PLANT_DENSITY if is_truss else BioConfig.PLANT_DENSITY
    young = TrussPhysicsConfig.YOUNG_MODULUS if is_truss else BioConfig.YOUNG_MODULUS
    damping_ratio = TrussPhysicsConfig.DAMPING_RATIO if is_truss else BioConfig.DAMPING_RATIO
    return density, young, damping_ratio


def _link_physics(link) -> tuple[float, float, float]:
    density, young, damping_ratio = _material_properties(link)
    mass = compute_mass(link.visual_radius, link.authored_pose.length, density=density)
    stiffness, damping = calculate_physics_params(
        link.visual_radius,
        link.authored_pose.length,
        mass,
        young_modulus=young,
        damping_ratio=damping_ratio,
    )
    return mass, stiffness, damping


def _joint_drive_gains(link, parent) -> tuple[float, float]:
    """Use the established V2 parent-child series hinge at attachments."""

    _, child_stiffness, child_damping = _link_physics(link)
    _, child_young, _ = _material_properties(link)
    _, parent_young, _ = _material_properties(parent)
    child_ei = compute_flexural_rigidity(link.visual_radius, child_young)
    parent_ei = compute_flexural_rigidity(parent.visual_radius, parent_young)
    attachment_stiffness_rad = compute_hinge_stiffness_rad(
        parent.authored_pose.length,
        parent_ei,
        link.authored_pose.length,
        child_ei,
    )
    attachment_stiffness = attachment_stiffness_rad * (math.pi / 180.0)
    ratio = (
        attachment_stiffness / child_stiffness
        if child_stiffness > 0.0
        else 1.0
    )
    return attachment_stiffness, child_damping * math.sqrt(ratio)


def _author_joint(
    stage,
    link,
    parent,
    paths,
    stiffness_scale: float,
    *,
    drives_enabled: bool,
    fixed_joints_enabled: bool,
    leaf_stiffness_scale: float,
    truss_stiffness_scale: float,
) -> None:
    child_path = paths[link.id]
    if parent is None:
        joint = UsdPhysics.FixedJoint.Define(stage, f"{child_path}/RootFixedJoint")
        joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
        # With body0 omitted, frame 0 is expressed in world coordinates.  It
        # must match the complete root pose (not just its origin), otherwise
        # PhysX snaps the whole plant to an identity-oriented world anchor.
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*link.authored_pose.start))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLocalRot0Attr().Set(
            _quat_from_rotation(link.authored_pose.rotation)
        )
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        if not fixed_joints_enabled:
            joint.CreateJointEnabledAttr().Set(False)
        return
    parent_path = paths[parent.id]
    local_position, local_rotation = _local_pose(link.authored_pose, parent.authored_pose)
    if link.joint_type == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, f"{child_path}/AttachJoint")
        if not fixed_joints_enabled:
            joint.CreateJointEnabledAttr().Set(False)
    else:
        joint = UsdPhysics.Joint.Define(stage, f"{child_path}/AttachJoint")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])
    joint.CreateLocalPos0Attr().Set(local_position)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(local_rotation)
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    if link.joint_type == "d6" and drives_enabled:
        stiffness, damping = _joint_drive_gains(link, parent)
        role_scale = (
            truss_stiffness_scale
            if link.role in {"truss_rachis", "pedicel"}
            else leaf_stiffness_scale
        )
        if link.role == "pedicel":
            role_scale *= TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE
        total_scale = stiffness_scale * role_scale
        configure_joint_drives(
            joint,
            stiffness * total_scale,
            damping * math.sqrt(total_scale),
            bend_limit_deg=(
                TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG
                if link.role == "pedicel"
                else BEND_LIMIT_DEG
            ),
        )
    add_collision_filter(stage, child_path, parent_path)
    add_collision_filter(stage, parent_path, child_path)


def _author_metadata(stage, plan: V2AuthoringPlan, plant_path: str) -> None:
    topology_root = f"{plant_path}/Topology"
    organ_root = f"{plant_path}/Organs"
    UsdGeom.Scope.Define(stage, topology_root)
    UsdGeom.Scope.Define(stage, organ_root)
    for node in sorted(plan.state.nodes, key=lambda item: item.id):
        prim = UsdGeom.Xform.Define(
            stage,
            f"{topology_root}/{_typed_prim_name(node.source_type, node.id)}",
        ).GetPrim()
        _string(prim, "autotom:entityKind", "topology_node")
        _string(prim, "autotom:canonicalNodeId", node.id)
        _string(prim, "autotom:sourceType", node.source_type)
        if node.parent_id is not None:
            _string(prim, "autotom:canonicalParentId", node.parent_id)
    for organ in sorted(plan.state.organs, key=lambda item: item.id):
        prim = UsdGeom.Xform.Define(
            stage,
            f"{organ_root}/{_typed_prim_name(organ.organ_type, organ.id)}",
        ).GetPrim()
        _string(prim, "autotom:entityKind", "organ")
        _string(prim, "autotom:canonicalOrganId", organ.id)
        _string(prim, "autotom:canonicalNodeId", organ.node_id)
        _string(prim, "autotom:organType", organ.organ_type)


def _fruit_maturation(plan: V2AuthoringPlan, owner_node_id: str, index: int) -> float:
    organ = next(organ for organ in plan.state.organs if organ.node_id == owner_node_id)
    properties = organ.properties
    if not isinstance(properties, FruitsProperties):
        return 0.0
    ages = properties.fruit_degree_days or ()
    if index >= len(ages) or properties.ripening_degree_days <= 0.0:
        return 0.0
    return max(0.0, min(float(ages[index]) / properties.ripening_degree_days, 1.0))


def _author_leaf_blades(stage, plan, visual_paths, link_paths, source_link_by_id) -> int:
    count = 0
    by_owner: dict[str, int] = {}
    for axis in plan.visual_axes:
        if not axis.render_geometry or axis.role not in {
            "petiolule_left",
            "petiolule_right",
            "rachis_terminal",
        }:
            continue
        host = next(link for link in plan.physical_links if link.id == axis.host_link_id)
        host_world = _pose_matrix(host.authored_pose)
        length = min(0.18, max(0.08, axis.authored_pose.length * LEAF_LENGTH_FRACTION))
        root = Gf.Vec3d(*axis.authored_pose.end)
        forward = Gf.Vec3d(*axis.authored_pose.direction)
        owner_index = by_owner.get(axis.owner_node_id, 0)
        by_owner[axis.owner_node_id] = owner_index + 1
        author_leaf_blade(
            stage,
            f"{link_paths[axis.host_link_id]}/LeafBlade_{_safe(axis.id)}",
            root,
            forward,
            length=length,
            half_width=length * LEAF_HALF_WIDTH_FRACTION,
            fold_depth=length * LEAF_LONGITUDINAL_FOLD_FRACTION,
            arch_lift=length * LEAF_ARCH_LIFT_FRACTION,
            tip_sag=length * LEAF_TIP_SAG_FRACTION,
            color=PlantColors.LEAF_BLADE,
            world_to_link=host_world.GetInverse(),
        )
        count += 1
    return count


def export_plant_state_v2(
    plan: V2AuthoringPlan,
    output_path: str | Path,
    *,
    stiffness_scale: float = 1.0,
    leaf_stiffness_scale: float = 1.0,
    truss_stiffness_scale: float = 1.0,
    physics_hz: int = 480,
) -> Path:
    """Author a canonical V2 USDA stage and return its absolute path."""

    if stiffness_scale not in {1.0, 2.0, 4.0}:
        raise V2ExportError("stiffness_scale must be one of 1, 2, or 4")
    if leaf_stiffness_scale not in DRIVE_SCALE_CHOICES:
        raise V2ExportError(
            f"leaf_stiffness_scale must be one of {DRIVE_SCALE_CHOICES}"
        )
    if truss_stiffness_scale not in DRIVE_SCALE_CHOICES:
        raise V2ExportError(
            f"truss_stiffness_scale must be one of {DRIVE_SCALE_CHOICES}"
        )
    if physics_hz not in {480, 960}:
        raise V2ExportError("physics_hz must be 480 or 960")
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)
    plant_path = f"/World/Plant_{plan.state.metadata.plant_id}"
    plant = UsdGeom.Xform.Define(stage, plant_path).GetPrim()
    _string(plant, "autotom:plantStateSchema", plan.state.schema_version)
    _string(plant, "autotom:renderer", "exporterV2/plant_state")
    _string(plant, "autotom:originPolicy", "plant_base_at_stage_origin")
    _string(plant, "autotom:physicsPreset", plan.physics_preset)
    _string(plant, "autotom:debugProfile", plan.debug_profile)
    _string(
        plant,
        "autotom:lockedImplementation",
        "kinematic_bodies_with_disabled_fixed_topology_joints"
        if plan.physics_preset == "locked"
        else "not_applicable",
    )
    plant.CreateAttribute("autotom:sourceWorldOrigin", Sdf.ValueTypeNames.Double3, custom=True).Set(
        Gf.Vec3d(*plan.source_origin)
    )
    plant.CreateAttribute("autotom:canonicalScale", Sdf.ValueTypeNames.Double, custom=True).Set(plan.scale)
    plant.CreateAttribute("autotom:stiffnessScale", Sdf.ValueTypeNames.Double, custom=True).Set(stiffness_scale)
    plant.CreateAttribute("autotom:leafStiffnessScale", Sdf.ValueTypeNames.Double, custom=True).Set(leaf_stiffness_scale)
    plant.CreateAttribute("autotom:trussStiffnessScale", Sdf.ValueTypeNames.Double, custom=True).Set(truss_stiffness_scale)
    plant.CreateAttribute("autotom:physicsHz", Sdf.ValueTypeNames.Int, custom=True).Set(physics_hz)
    plant.CreateAttribute("autotom:collidersEnabled", Sdf.ValueTypeNames.Bool, custom=True).Set(plan.colliders_enabled)
    plant.CreateAttribute("autotom:drivesEnabled", Sdf.ValueTypeNames.Bool, custom=True).Set(plan.drives_enabled)
    plant.CreateAttribute("autotom:articulationEnabled", Sdf.ValueTypeNames.Bool, custom=True).Set(plan.articulation_enabled)
    plant.CreateAttribute("autotom:terminalBodiesPhysical", Sdf.ValueTypeNames.Bool, custom=True).Set(plan.terminal_bodies_physical)
    _author_metadata(stage, plan, plant_path)

    physics_root = f"{plant_path}/Physics"
    visual_root = f"{plant_path}/Visual"
    physics_prim = UsdGeom.Xform.Define(stage, physics_root).GetPrim()
    if plan.physics_preset == "flexible" and plan.articulation_enabled:
        UsdPhysics.ArticulationRootAPI.Apply(physics_prim)
    UsdGeom.Scope.Define(stage, visual_root)
    organ_type_by_node = {
        organ.node_id: organ.organ_type for organ in plan.state.organs
    }
    link_paths = {
        link.id: (
            f"{physics_root}/"
            f"{_typed_prim_name(organ_type_by_node[link.owner_node_id], link.id)}"
        )
        for link in plan.physical_links
    }
    link_by_id = {link.id: link for link in plan.physical_links}
    for link in plan.physical_links:
        path = link_paths[link.id]
        xform = UsdGeom.Xform.Define(stage, path)
        _set_pose(xform, link.authored_pose)
        prim = xform.GetPrim()
        _string(prim, "autotom:entityKind", "physical_link")
        _string(prim, "autotom:canonicalPrimitiveId", link.canonical_axis_id)
        _string(prim, "autotom:role", link.role)
        _string(prim, "autotom:jointType", link.joint_type)
        prim.CreateAttribute("autotom:sourceLength", Sdf.ValueTypeNames.Double, custom=True).Set(link.authored_pose.length)
        if link.parent_id is not None:
            _string(prim, "autotom:physicalParentId", link.parent_id)
        _strings(prim, "autotom:canonicalOrganIds", link.canonical_organ_ids)
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body.CreateRigidBodyEnabledAttr().Set(True)
        rigid_body.CreateKinematicEnabledAttr().Set(plan.physics_preset == "locked")
        mass, _, _ = _link_physics(link)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr().Set(float(mass))
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, link.authored_pose.length / 2.0))
        if plan.colliders_enabled:
            _author_collider(stage, path, link.authored_pose.length, link.collider_radius)

    for link in plan.physical_links:
        _author_joint(
            stage,
            link,
            link_by_id.get(link.parent_id),
            link_paths,
            stiffness_scale,
            drives_enabled=plan.drives_enabled,
            fixed_joints_enabled=plan.physics_preset == "flexible",
            leaf_stiffness_scale=leaf_stiffness_scale,
            truss_stiffness_scale=truss_stiffness_scale,
        )

    visual_paths = {}
    for axis in plan.visual_axes:
        if not axis.render_geometry:
            continue
        host = link_by_id[axis.host_link_id]
        host_path = link_paths[axis.host_link_id]
        local_position, local_rotation = _local_pose(axis.authored_pose, host.authored_pose)
        path = (
            f"{host_path}/VisualAxis_"
            f"{_typed_prim_name(axis.organ_type, axis.id)}"
        )
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp().Set(local_position)
        xform.AddOrientOp().Set(local_rotation)
        prim = xform.GetPrim()
        _string(prim, "autotom:entityKind", "visual_axis")
        _string(prim, "autotom:canonicalPrimitiveId", axis.id)
        _string(prim, "autotom:canonicalNodeId", axis.owner_node_id)
        _string(prim, "autotom:role", axis.role)
        _author_axis_visual(stage, path, axis.authored_pose.length, axis.radius, axis.role)
        visual_paths[axis.id] = path

    leaf_blade_count = _author_leaf_blades(stage, plan, visual_paths, link_paths, link_by_id)
    terminal_root = "/World/TerminalBodies"
    UsdGeom.Xform.Define(stage, terminal_root)
    sphere_paths = {}
    owner_indices: dict[str, int] = {}
    for sphere in plan.visual_spheres:
        if not sphere.render_geometry:
            continue
        index = owner_indices.get(sphere.owner_node_id, 0)
        owner_indices[sphere.owner_node_id] = index + 1
        path = f"{terminal_root}/{_typed_prim_name(sphere.organ_type, sphere.id)}"
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*sphere.authored_center))
        prim = body.GetPrim()
        _string(
            prim,
            "autotom:entityKind",
            "terminal_body" if plan.terminal_bodies_physical else "visual_sphere",
        )
        _string(prim, "autotom:canonicalPrimitiveId", sphere.id)
        _string(prim, "autotom:canonicalNodeId", sphere.owner_node_id)
        geometry = UsdGeom.Sphere.Define(stage, f"{path}/Visual")
        geometry.CreateRadiusAttr().Set(float(sphere.radius))
        material = get_or_create_tomato_fruit_material(
            stage, _fruit_maturation(plan, sphere.owner_node_id, index)
        )
        UsdShade.MaterialBindingAPI.Apply(geometry.GetPrim()).Bind(material)
        if plan.terminal_bodies_physical:
            _string(prim, "autotom:hostLinkId", sphere.host_link_id)
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
            rigid_body.CreateRigidBodyEnabledAttr().Set(True)
            rigid_body.CreateKinematicEnabledAttr().Set(plan.physics_preset == "locked")
            mass = 4.0 / 3.0 * math.pi * sphere.radius**3 * 1000.0
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(float(mass))
            if plan.colliders_enabled:
                UsdPhysics.CollisionAPI.Apply(geometry.GetPrim())
            host = link_by_id[sphere.host_link_id]
            local = np.asarray(host.authored_pose.rotation).T @ (
                np.asarray(sphere.authored_center) - np.asarray(host.authored_pose.start)
            )
            joint = UsdPhysics.FixedJoint.Define(stage, f"{path}/AttachJoint")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(link_paths[host.id])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*local))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            joint.CreateLocalRot0Attr().Set(
                _quat_from_rotation(np.asarray(host.authored_pose.rotation).T)
            )
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            if plan.physics_preset == "flexible":
                configure_detachable_joint(
                    joint,
                    break_force=TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N,
                    exclude_from_articulation=True,
                )
            else:
                joint.CreateJointEnabledAttr().Set(False)
            if plan.colliders_enabled:
                add_collision_filter(stage, path, link_paths[host.id])
            sphere_paths[sphere.id] = path

    apply_physx_scene_settings(stage, physics_hz=physics_hz)
    if plan.physics_preset == "flexible" and plan.articulation_enabled:
        apply_physx_articulation_settings(stage, physics_root)
    for path in sphere_paths.values():
        apply_physx_rigid_body_solver_settings(stage, path)

    body_paths = {**link_paths, **sphere_paths}
    for record in (
        *plan.intentional_collision_filters,
        *plan.unresolved_collision_filters,
    ):
        if record.body_a in body_paths and record.body_b in body_paths:
            add_collision_filter(stage, body_paths[record.body_a], body_paths[record.body_b])
            add_collision_filter(stage, body_paths[record.body_b], body_paths[record.body_a])

    plant.CreateAttribute("autotom:leafBladeCount", Sdf.ValueTypeNames.Int, custom=True).Set(leaf_blade_count)
    stage.GetRootLayer().Save()
    return destination


def manifest_path_for(usd_path: str | Path) -> Path:
    return Path(usd_path).expanduser().resolve().with_suffix(".manifest.json")


def audit_v2_stage(plan: V2AuthoringPlan, usd_path: str | Path) -> V2ExportManifest:
    path = Path(usd_path).expanduser().resolve()
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise V2ExportError(f"cannot open V2 stage: {path}")
    counts = Counter()
    canonical_visual_ids = set()
    canonical_physical_ids = set()
    topology_nodes = set()
    organ_ids = set()
    rigid_body_paths = set()
    collider_paths = set()
    d6_joint_paths = set()
    fixed_joint_paths = set()
    paths = []
    for prim in stage.Traverse():
        paths.append(str(prim.GetPath()))
        kind_attr = prim.GetAttribute("autotom:entityKind")
        kind = kind_attr.Get() if kind_attr else None
        if kind:
            counts[str(kind)] += 1
        primitive_attr = prim.GetAttribute("autotom:canonicalPrimitiveId")
        primitive_id = primitive_attr.Get() if primitive_attr else None
        if kind in {"visual_axis", "terminal_body", "visual_sphere"} and primitive_id:
            canonical_visual_ids.add(str(primitive_id))
        if kind == "physical_link" and primitive_id:
            canonical_physical_ids.add(str(primitive_id))
        if kind == "topology_node":
            topology_nodes.add(str(prim.GetAttribute("autotom:canonicalNodeId").Get()))
        if kind == "organ":
            organ_ids.add(str(prim.GetAttribute("autotom:canonicalOrganId").Get()))
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body_paths.add(str(prim.GetPath()))
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collider_paths.add(str(prim.GetPath()))
        if prim.IsA(UsdPhysics.FixedJoint):
            fixed_joint_paths.add(str(prim.GetPath()))
        elif prim.IsA(UsdPhysics.Joint):
            d6_joint_paths.add(str(prim.GetPath()))
    expected_visual = {
        axis.id for axis in plan.visual_axes if axis.render_geometry
    } | {sphere.id for sphere in plan.visual_spheres if sphere.render_geometry}
    expected_physical = {link.canonical_axis_id for link in plan.physical_links}
    errors = []
    if canonical_visual_ids != expected_visual:
        errors.append("canonical visual primitive coverage differs")
    if canonical_physical_ids != expected_physical:
        errors.append("canonical physical primitive coverage differs")
    if topology_nodes != {node.id for node in plan.state.nodes}:
        errors.append("canonical topology node coverage differs")
    if organ_ids != {organ.id for organ in plan.state.organs}:
        errors.append("canonical organ coverage differs")
    if len(paths) != len(set(paths)):
        errors.append("duplicate USD paths")
    plant = stage.GetPrimAtPath(f"/World/Plant_{plan.state.metadata.plant_id}")
    source_origin = plant.GetAttribute("autotom:sourceWorldOrigin").Get()
    if tuple(float(value) for value in source_origin) != plan.source_origin:
        errors.append("source origin metadata differs")
    root = next(link for link in plan.physical_links if link.parent_id is None)
    if any(abs(value) > 1e-12 for value in root.authored_pose.start):
        errors.append(f"physical root is not at origin: {root.authored_pose.start}")
    physical_sphere_count = (
        sum(sphere.render_geometry for sphere in plan.visual_spheres)
        if plan.terminal_bodies_physical
        else 0
    )
    if len(rigid_body_paths) != len(plan.physical_links) + physical_sphere_count:
        errors.append("rigid body count differs from the physical plan")
    expected_colliders = (
        len(plan.physical_links) + physical_sphere_count
        if plan.colliders_enabled
        else 0
    )
    if len(collider_paths) != expected_colliders:
        errors.append("collider count differs from the physical plan")
    if len(d6_joint_paths) != plan.predicted_d6_joints:
        errors.append("D6 joint count differs from the physical plan")
    expected_fixed_joints = 1 + sum(
        link.joint_type == "fixed" and link.parent_id is not None
        for link in plan.physical_links
    )
    if len(fixed_joint_paths) != expected_fixed_joints:
        errors.append("FixedJoint count differs from the physical plan")
    enabled_fixed_joints = 0
    for joint_path in fixed_joint_paths:
        enabled = stage.GetPrimAtPath(joint_path).GetAttribute(
            "physics:jointEnabled"
        ).Get()
        if enabled is not False:
            enabled_fixed_joints += 1
    expected_enabled_fixed = (
        expected_fixed_joints if plan.physics_preset == "flexible" else 0
    )
    if enabled_fixed_joints != expected_enabled_fixed:
        errors.append("enabled FixedJoint count differs from the physical plan")
    stiffness_scale = float(plant.GetAttribute("autotom:stiffnessScale").Get())
    leaf_stiffness_scale = float(
        plant.GetAttribute("autotom:leafStiffnessScale").Get()
    )
    truss_stiffness_scale = float(
        plant.GetAttribute("autotom:trussStiffnessScale").Get()
    )
    physics_hz = int(plant.GetAttribute("autotom:physicsHz").Get())
    physical_organ_ids = {
        organ_id for link in plan.physical_links for organ_id in link.canonical_organ_ids
    }
    all_organ_ids = {organ.id for organ in plan.state.organs}
    return V2ExportManifest(
        metadata={
            "status": "passed" if not errors else "failed",
            "usd_file": path.name,
            "plant_id": plan.state.metadata.plant_id,
            "simulation_time": plan.state.metadata.simulation_time,
            "plant_state_schema": plan.state.schema_version,
            "origin_policy": "plant_base_at_stage_origin",
            "source_world_origin": list(plan.source_origin),
            "global_scale": plan.scale,
            "physics_preset": plan.physics_preset,
            "debug_profile": plan.debug_profile,
            "physics_hz": physics_hz,
            "stiffness_scale": stiffness_scale,
            "leaf_stiffness_scale": leaf_stiffness_scale,
            "truss_stiffness_scale": truss_stiffness_scale,
        },
        canonical={
            "nodes": len(plan.state.nodes),
            "edges": len(plan.state.edges),
            "organs": dict(sorted(Counter(o.organ_type for o in plan.state.organs).items())),
            "axes": len(plan.state.axes),
            "spheres": len(plan.state.spheres),
            "selected_axis_ids": plan.diagnostics["selected_axis_ids"],
            "omitted_axis_ids": plan.diagnostics["omitted_axis_ids"],
            "selected_sphere_ids": plan.diagnostics["selected_sphere_ids"],
            "omitted_sphere_ids": plan.diagnostics["omitted_sphere_ids"],
        },
        visual={
            "axes_expected": sum(axis.render_geometry for axis in plan.visual_axes),
            "spheres_expected": sum(sphere.render_geometry for sphere in plan.visual_spheres),
            "canonical_primitive_ids": len(canonical_visual_ids),
            "leaf_blades": int(plant.GetAttribute("autotom:leafBladeCount").Get()),
            "visual_only_axes": plan.diagnostics["visual_only_axis_count"],
            "duplicate_geometry_of": plan.diagnostics["duplicate_geometry_of"],
            "degenerate_axis_ids": plan.diagnostics["degenerate_axis_ids"],
            "spheres": [
                {
                    "id": sphere.id,
                    "canonical_node_id": sphere.owner_node_id,
                    "host_link_id": sphere.host_link_id,
                    "source_center": list(sphere.source_center),
                    "authored_center": list(sphere.authored_center),
                    "radius": sphere.radius,
                    "render_geometry": sphere.render_geometry,
                }
                for sphere in plan.visual_spheres
            ],
        },
        physics={
            "physical_links": len(plan.physical_links),
            "d6_joints": plan.predicted_d6_joints,
            "fixed_links": sum(link.joint_type == "fixed" for link in plan.physical_links),
            "terminal_bodies": physical_sphere_count,
            "static_visual_spheres": (
                sum(sphere.render_geometry for sphere in plan.visual_spheres)
                - physical_sphere_count
            ),
            "rigid_bodies_authored": len(rigid_body_paths),
            "colliders_authored": len(collider_paths),
            "fixed_joints_authored": len(fixed_joint_paths),
            "fixed_joints_enabled": enabled_fixed_joints,
            "locked_implementation": (
                "kinematic_bodies_with_disabled_fixed_topology_joints"
                if plan.physics_preset == "locked"
                else None
            ),
            "kinematic_bodies": (
                len(rigid_body_paths) if plan.physics_preset == "locked" else 0
            ),
            "colliders_enabled": plan.colliders_enabled,
            "drives_enabled": plan.drives_enabled,
            "articulation_enabled": plan.articulation_enabled,
            "dynamic_organ_ids": sorted(
                {
                    organ_id
                    for link in plan.physical_links
                    if link.joint_type == "d6"
                    for organ_id in link.canonical_organ_ids
                }
            ),
            "structural_fixed_organ_ids": sorted(
                {
                    organ_id
                    for link in plan.physical_links
                    if link.joint_type == "fixed"
                    for organ_id in link.canonical_organ_ids
                }
            ),
            "static_metadata_organ_ids": sorted(all_organ_ids - physical_organ_ids),
            "joint_target": plan.diagnostics["joint_target"],
            "joint_warning_max": plan.diagnostics["joint_warning_max"],
            "aggregated_physical_link_ids": plan.diagnostics["aggregated_physical_link_ids"],
            "links": [
                {
                    "id": link.id,
                    "parent_id": link.parent_id,
                    "role": link.role,
                    "joint_type": link.joint_type,
                    "canonical_organ_ids": list(link.canonical_organ_ids),
                    "canonical_primitive_ids": list(link.canonical_primitive_ids),
                    "source_pose": asdict(link.source_pose),
                    "authored_pose": asdict(link.authored_pose),
                    "visual_radius": link.visual_radius,
                    "collider_radius": link.collider_radius,
                }
                for link in plan.physical_links
            ],
        },
        collisions={
            "adjustments": [asdict(item) for item in plan.collision_adjustments],
            "intentional_filtered_pairs": [asdict(item) for item in plan.intentional_collision_filters],
            "unresolved_filtered_pairs": [asdict(item) for item in plan.unresolved_collision_filters],
            "active_unfiltered_initial_overlaps": 0,
            "correction_limits": {
                "azimuth_degrees": 5.0,
                "tilt_degrees": 3.0,
                "shift_canonical_m": 0.002,
            },
            "collider_radius_scale": plan.diagnostics["collider_radius_scale"],
            "collider_length_scale": plan.diagnostics["collider_length_scale"],
            "minimum_collider_radius_world_m": plan.diagnostics[
                "minimum_collider_radius_world_m"
            ],
        },
        topology={
            "usd_topology_nodes": len(topology_nodes),
            "usd_organs": len(organ_ids),
            "path_count": len(paths),
        },
        errors=tuple(errors),
    )


def save_v2_manifest(manifest: V2ExportManifest, path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(manifest.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination
