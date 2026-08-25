"""PlantState checkpoints routed through the established ExporterV2 backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from plant_state import PlantState

from .core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
from .core.skinning import (
    VisualAxisData,
    VisualProfile,
    VisualSegment,
    build_visual_axes,
    resolve_vegetative_graph,
)
from .core.skinning.adapter import _quat_from_column_rotation
from .core.skinning.leaf_blade import (
    LEAF_ARCH_LIFT_FRACTION,
    LEAF_HALF_WIDTH_FRACTION,
    LEAF_LENGTH_FRACTION,
    LEAF_LONGITUDINAL_FOLD_FRACTION,
    LEAF_TIP_SAG_FRACTION,
    author_leaf_blade,
)
from .core.skinning.visual_segmented import author_segmented_visual_axis
from .core.tree_config import GLOBAL_SCALE, PlantColors
from .core.usd import build_stage
from .core.usd.collision import add_collision_filter
from .plant_state_branches import (
    LEAF_JOINT_POLICIES,
    VISUAL_QUALITY_MODES,
    StemBranchesResult,
    build_leaf_branches,
    build_leaf_support_branches,
    build_lateral_branches,
    build_stem_branches,
)


STEM_CHECKPOINT_SCHEMA = "exporter_v2_stem_checkpoint/1.0"
LATERALS_CHECKPOINT_SCHEMA = "exporter_v2_laterals_checkpoint/1.0"
LEAF_SUPPORTS_CHECKPOINT_SCHEMA = "exporter_v2_leaf_supports_checkpoint/1.0"
LEAVES_CHECKPOINT_SCHEMA = "exporter_v2_leaves_checkpoint/1.0"
INCREMENTAL_PROFILES = ("stem", "laterals", "leaf-supports", "leaves")


class IncrementalCheckpointError(ValueError):
    """Raised when a conservative V2 checkpoint fails its audit."""


# Compatibility names retained for callers of the completed stem checkpoint.
StemCheckpointError = IncrementalCheckpointError


@dataclass(frozen=True)
class IncrementalCheckpointManifest:
    metadata: dict[str, Any]
    expected: dict[str, int]
    authored: dict[str, Any]
    topology: dict[str, Any]
    physics: dict[str, Any]
    collisions: dict[str, Any]
    errors: tuple[str, ...]
    schema_version: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


StemCheckpointManifest = IncrementalCheckpointManifest


@dataclass(frozen=True)
class IncrementalCheckpointPlan:
    adapter: StemBranchesResult
    physical_link_count: int
    predicted_d6_joints: int
    debug_profile: str


StemCheckpointPlan = IncrementalCheckpointPlan


def manifest_path_for(path: Path) -> Path:
    return path.with_suffix(".manifest.json")


def _custom(prim, name: str, value_type, value) -> None:
    prim.CreateAttribute(name, value_type, custom=True).Set(value)


def _author_stage_metadata(
    stage,
    state: PlantState,
    adapter: StemBranchesResult,
    physics_preset: str,
) -> None:
    stem = stage.GetPrimAtPath("/World/Stem")
    _custom(stem, "autotom:plantStateSchema", Sdf.ValueTypeNames.String, state.schema_version)
    _custom(stem, "autotom:debugProfile", Sdf.ValueTypeNames.String, adapter.debug_profile)
    _custom(stem, "autotom:poseMode", Sdf.ValueTypeNames.String, adapter.pose_mode)
    _custom(stem, "autotom:physicsPreset", Sdf.ValueTypeNames.String, physics_preset)
    _custom(
        stem,
        "autotom:visualQuality",
        Sdf.ValueTypeNames.String,
        adapter.visual_quality,
    )
    if adapter.leaf_joint_policy is not None:
        _custom(
            stem,
            "autotom:leafJointPolicy",
            Sdf.ValueTypeNames.String,
            adapter.leaf_joint_policy,
        )
    _custom(stem, "autotom:collidersEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:drivesEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:articulationEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:terminalBodiesPhysical", Sdf.ValueTypeNames.Bool, False)
    root = next(node for node in state.nodes if node.id == state.root_node_id)
    _custom(
        stem,
        "autotom:groimpOrigin",
        Sdf.ValueTypeNames.Double3,
        Gf.Vec3d(*root.pose.world_start),
    )

    for branch in adapter.branches:
        prim = stage.GetPrimAtPath(f"/World/Stem/Vegetative/{branch['id']}")
        if not prim or not prim.IsValid():
            continue
        _custom(prim, "autotom:branchId", Sdf.ValueTypeNames.String, branch["id"])
        _custom(prim, "autotom:branchKind", Sdf.ValueTypeNames.String, branch["kind"])
        _custom(prim, "autotom:jointType", Sdf.ValueTypeNames.String, branch["joint_type"])
        if branch.get("source_parent_node_id") is not None:
            _custom(
                prim,
                "autotom:sourceParentNodeId",
                Sdf.ValueTypeNames.String,
                branch["source_parent_node_id"],
            )

    for record in adapter.degenerate_organs:
        prim = UsdGeom.Xform.Define(
            stage,
            f"/World/Stem/Vegetative/{record['xform_name']}",
        ).GetPrim()
        _custom(prim, "autotom:entityKind", Sdf.ValueTypeNames.String, "degenerate_organ")
        _custom(prim, "autotom:organType", Sdf.ValueTypeNames.String, "Leaf")
        _custom(prim, "autotom:canonicalOrganId", Sdf.ValueTypeNames.String, record["organ_id"])
        _custom(prim, "autotom:canonicalNodeId", Sdf.ValueTypeNames.String, record["node_id"])
        _custom(prim, "autotom:groimpNodeId", Sdf.ValueTypeNames.Int64, record["groimp_node_id"])
        _custom(prim, "autotom:diagnostic", Sdf.ValueTypeNames.String, record["reason"])


def _author_rigid_leaf_visuals(stage, adapter: StemBranchesResult) -> list[dict[str, Any]]:
    """Bind canonical petiolules and V2 blades directly to support bodies."""

    if not adapter.rigid_leaf_visuals:
        return []
    body_by_axis = {}
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() != "physical_link":
            continue
        axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if axis_id is not None:
            body_by_axis[str(axis_id)] = str(prim.GetPath())

    authored = []
    for record in adapter.rigid_leaf_visuals:
        host_path = body_by_axis.get(record["host_axis_id"])
        if host_path is None:
            raise IncrementalCheckpointError(
                f"leaf visual {record['axis_id']} cannot resolve host "
                f"{record['host_axis_id']}"
            )
        host_matrix = UsdGeom.Xformable(
            stage.GetPrimAtPath(host_path)
        ).ComputeLocalToWorldTransform(0)
        world_to_host = host_matrix.GetInverse()
        frame = record["rest_frame"]
        start = Gf.Vec3d(
            *(float(frame[row][3]) * GLOBAL_SCALE for row in range(3))
        )
        head = Gf.Vec3d(*(float(frame[row][2]) for row in range(3))).GetNormalized()
        length = float(record["length"]) * GLOBAL_SCALE
        radius = float(record["radius"]) * GLOBAL_SCALE

        root_path = f"{host_path}/RigidLeafVisuals/{record['id']}"
        root_xform = UsdGeom.Xform.Define(stage, root_path)
        root = root_xform.GetPrim()
        orientation = _quat_from_column_rotation(frame)
        desired_world = Gf.Matrix4d(1.0)
        desired_world.SetTransform(
            Gf.Rotation(Gf.Quatd(orientation)), start
        )
        root_xform.AddTransformOp().Set(desired_world * world_to_host)
        actual_world = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0)
        frame_error = max(
            abs(float(actual_world[row][column] - desired_world[row][column]))
            for row in range(4)
            for column in range(4)
        )
        if frame_error > 1e-8:
            raise IncrementalCheckpointError(
                f"leaf visual {record['axis_id']} could not author its canonical "
                f"frame (matrix error {frame_error:.3g})"
            )
        _custom(root, "autotom:entityKind", Sdf.ValueTypeNames.String, "rigid_leaf_visual")
        _custom(root, "autotom:attachmentMode", Sdf.ValueTypeNames.String, "rigid_visual")
        _custom(root, "autotom:canonicalOrganId", Sdf.ValueTypeNames.String, record["organ_id"])
        _custom(root, "autotom:canonicalPrimitiveId", Sdf.ValueTypeNames.String, record["axis_id"])
        _custom(root, "autotom:groimpNodeId", Sdf.ValueTypeNames.Int64, record["groimp_node_id"])
        _custom(root, "autotom:role", Sdf.ValueTypeNames.String, record["role"])
        _custom(root, "autotom:hostPrimitiveId", Sdf.ValueTypeNames.String, record["host_axis_id"])

        profile_raw = adapter.branches[0]["visual_profile"]
        profile = VisualProfile(
            radial_segments=int(profile_raw["radial_segments"]),
            axial_spacing_m=float(profile_raw["axial_spacing_m"]),
            radius_transition_samples=int(
                profile_raw["radius_transition_samples"]
            ),
        )
        member = SimpleNamespace(
            definition={"id": record["id"], "kind": "petiolule"},
            spec=SimpleNamespace(visual=profile),
            centered_terminal=False,
            centered_terminal_host=False,
        )
        visual_axis = VisualAxisData(
            axis_id=record["axis_id"],
            members=[member],
            member_offsets={record["id"]: 0.0},
            member_lengths={record["id"]: length},
            visual_segments=[
                VisualSegment(
                    source_id=record["axis_id"],
                    start_arc=0.0,
                    length=length,
                    radius=radius,
                    end_radius=radius * 0.65,
                )
            ],
            link_paths=[root_path],
            link_bases=[start],
            link_orientations=[orientation],
            bone_starts=[0.0],
            bone_lengths=[length],
            start=start,
            axis=head,
            orientation=orientation,
            total_length=length,
            visual_root_path=root_path,
            skel_root_path=f"{root_path}/UnusedSkelRoot",
            skeleton_path=f"{root_path}/UnusedSkeleton",
            animation_path=f"{root_path}/UnusedAnimation",
            mesh_path=f"{root_path}/UnusedMesh",
            parent_radius=float(
                stage.GetPrimAtPath(host_path)
                .GetAttribute("autotom:visualRadius")
                .Get()
            ),
            attachment_arcs=[],
        )
        visual_stats = author_segmented_visual_axis(stage, visual_axis)

        blade_length = min(0.09, max(0.04, length * LEAF_LENGTH_FRACTION))
        author_leaf_blade(
            stage,
            f"{root_path}/LeafBlade",
            start + head * length,
            head,
            length=blade_length,
            half_width=blade_length * LEAF_HALF_WIDTH_FRACTION,
            fold_depth=blade_length * LEAF_LONGITUDINAL_FOLD_FRACTION,
            arch_lift=blade_length * LEAF_ARCH_LIFT_FRACTION,
            tip_sag=blade_length * LEAF_TIP_SAG_FRACTION,
            color=PlantColors.LEAF_BLADE,
            world_to_link=world_to_host,
        )
        added_mass = float(record.get("aggregated_mass_kg", 0.0))
        if added_mass > 0.0:
            # Preserve the lean V2 topology (no petiolule/blade rigid bodies),
            # while retaining the canonical leaf load. The dry biomass is
            # concentrated near the blade centroid and combined with the host
            # mass/COM, so gravity produces the expected bending moment.
            host_prim = stage.GetPrimAtPath(host_path)
            mass_api = UsdPhysics.MassAPI(host_prim)
            mass_attr = mass_api.GetMassAttr()
            com_attr = mass_api.GetCenterOfMassAttr()
            previous_mass = float(mass_attr.Get())
            previous_com_value = com_attr.Get()
            previous_com = Gf.Vec3d(*previous_com_value)
            blade_centroid_world = start + head * (
                length + 0.4 * blade_length
            )
            blade_centroid_local = world_to_host.Transform(blade_centroid_world)
            combined_mass = previous_mass + added_mass
            combined_com = (
                previous_com * previous_mass + blade_centroid_local * added_mass
            ) / combined_mass
            mass_attr.Set(combined_mass)
            com_attr.Set(Gf.Vec3f(*combined_com))
            current_visual_mass = float(
                host_prim.GetAttribute("autotom:aggregatedLeafVisualMassKg").Get()
                or 0.0
            )
            _custom(
                host_prim,
                "autotom:aggregatedLeafVisualMassKg",
                Sdf.ValueTypeNames.Double,
                current_visual_mass + added_mass,
            )
        _custom(
            root,
            "autotom:aggregatedMassKg",
            Sdf.ValueTypeNames.Double,
            added_mass,
        )
        _custom(
            root,
            "autotom:massSource",
            Sdf.ValueTypeNames.String,
            str(record.get("mass_source", "none")),
        )
        authored.append(
            {
                **record,
                "host_body_path": host_path,
                "root_path": root_path,
                "blade_length": blade_length,
                "renderer": "v2_segmented_organic",
                "visual_stats": visual_stats,
            }
        )
    return authored


def _segment_distance(a0, a1, b0, b1) -> float:
    """Shortest distance between finite 3-D line segments."""

    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    aa = u * u
    bb = u * v
    cc = v * v
    dd = u * w
    ee = v * w
    denominator = aa * cc - bb * bb
    small = 1e-15
    s_num = 0.0
    s_den = denominator
    t_num = 0.0
    t_den = denominator

    if denominator < small:
        s_num = 0.0
        s_den = 1.0
        t_num = ee
        t_den = cc
    else:
        s_num = bb * ee - cc * dd
        t_num = aa * ee - bb * dd
        if s_num < 0.0:
            s_num = 0.0
            t_num = ee
            t_den = cc
        elif s_num > s_den:
            s_num = s_den
            t_num = ee + bb
            t_den = cc

    if t_num < 0.0:
        t_num = 0.0
        if -dd < 0.0:
            s_num = 0.0
        elif -dd > aa:
            s_num = s_den
        else:
            s_num = -dd
            s_den = aa
    elif t_num > t_den:
        t_num = t_den
        if -dd + bb < 0.0:
            s_num = 0.0
        elif -dd + bb > aa:
            s_num = s_den
        else:
            s_num = -dd + bb
            s_den = aa

    sc = 0.0 if abs(s_num) < small else s_num / s_den
    tc = 0.0 if abs(t_num) < small else t_num / t_den
    delta = w + sc * u - tc * v
    return math.sqrt(delta * delta)


def _capsule_records(stage):
    records = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Capsule):
            continue
        capsule = UsdGeom.Capsule(prim)
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        center = matrix.ExtractTranslation()
        direction = Gf.Vec3d(
            matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        ).GetNormalized()
        half_spine = float(capsule.GetHeightAttr().Get()) * 0.5
        records.append(
            {
                "path": str(prim.GetPath()),
                "body_path": str(prim.GetParent().GetPath()),
                "start": center - direction * half_spine,
                "end": center + direction * half_spine,
                "radius": float(capsule.GetRadiusAttr().Get()),
            }
        )
    return records


def _filtered_targets(stage, body_path: str) -> set[str]:
    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        return set()
    relationship = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
    return {str(path) for path in relationship.GetTargets()}


def _audit_capsule_overlaps(stage) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    capsules = _capsule_records(stage)
    filtered_cache = {
        record["body_path"]: _filtered_targets(stage, record["body_path"])
        for record in capsules
    }
    filtered = []
    active = []
    for index, left in enumerate(capsules):
        for right in capsules[index + 1 :]:
            if left["body_path"] == right["body_path"]:
                continue
            distance = _segment_distance(
                left["start"], left["end"], right["start"], right["end"]
            )
            depth = left["radius"] + right["radius"] - distance
            if depth <= 1e-9:
                continue
            is_filtered = (
                right["body_path"] in filtered_cache[left["body_path"]]
                or left["body_path"] in filtered_cache[right["body_path"]]
            )
            record = {
                "body_a": left["body_path"],
                "body_b": right["body_path"],
                "capsule_a": left["path"],
                "capsule_b": right["path"],
                "depth": round(float(depth), 12),
                "filtered": is_filtered,
            }
            (filtered if is_filtered else active).append(record)
    key = lambda value: (
        value["body_a"], value["body_b"], value["capsule_a"], value["capsule_b"]
    )
    return sorted(filtered, key=key), sorted(active, key=key)


def _audit_stage(
    stage,
    state: PlantState,
    adapter: StemBranchesResult,
    physics_preset: str,
    usd_path: Path,
    approved_collision_filters: list[dict[str, Any]],
    rigid_leaf_visuals: list[dict[str, Any]],
) -> IncrementalCheckpointManifest:
    specs = [spec for branch in adapter.branches for spec in branch["link_specs"]]
    spec_by_axis = {spec["canonical_axis_id"]: spec for spec in specs}
    bodies = [
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
    ]
    fixed_joints = [
        prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsFixedJoint"
    ]
    d6_joints = [
        prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"
    ]
    meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    all_organic_meshes = [
        prim for prim in meshes if prim.GetName().startswith("OrganicVisual_")
    ]
    organic_meshes = [
        prim
        for prim in all_organic_meshes
        if prim.GetParent().GetAttribute("autotom:entityKind").Get()
        == "physical_link"
    ]
    petiolule_meshes = [
        prim
        for prim in all_organic_meshes
        if prim.GetParent().GetAttribute("autotom:entityKind").Get()
        == "rigid_leaf_visual"
    ]
    leaf_blades = [prim for prim in meshes if prim.GetName() == "LeafBlade"]
    terminal_fork_shoots = [
        prim for prim in meshes if prim.GetName() == "TerminalForkYoungShoot"
    ]
    terminal_fork_leaves = [
        prim for prim in meshes if prim.GetName() == "TerminalForkYoungLeaf"
    ]
    cylinders = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Cylinder)]
    capsules = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Capsule)]
    paths = [str(prim.GetPath()) for prim in bodies]
    errors = []

    def mesh_complexity(selected) -> dict[str, int]:
        points = 0
        faces = 0
        triangles = 0
        for prim in selected:
            mesh = UsdGeom.Mesh(prim)
            points += len(mesh.GetPointsAttr().Get() or ())
            counts = list(mesh.GetFaceVertexCountsAttr().Get() or ())
            faces += len(counts)
            triangles += sum(max(0, int(count) - 2) for count in counts)
        return {
            "meshes": len(selected),
            "points": points,
            "faces": faces,
            "triangles": triangles,
        }

    mesh_complexity_by_role = {
        "all": mesh_complexity(meshes),
        "organic": mesh_complexity(organic_meshes),
        "organic_all": mesh_complexity(all_organic_meshes),
        "petiolules": mesh_complexity(petiolule_meshes),
        "leaf_blades": mesh_complexity(leaf_blades),
    }

    resolved = resolve_vegetative_graph(
        list(adapter.branches), locked_joints=physics_preset == "locked"
    )
    resolved_axes = build_visual_axes(resolved, "/World/PlantVisual")

    expected_count = len(specs)
    expected_d6 = (
        sum(branch["n_links"] for branch in adapter.branches if branch["joint_type"] != "fixed")
        if physics_preset == "flexible"
        else 0
    )
    expected_fixed = expected_count - expected_d6
    expected_axes = len({branch["visual_axis_id"] for branch in adapter.branches})
    branch_by_id = {branch["id"]: branch for branch in adapter.branches}
    expected_terminal_forks = sum(
        branch.get("kind") == "leaf_petiole"
        and not branch.get("disable_centered_terminal", False)
        and branch.get("parent") in branch_by_id
        and branch_by_id[branch["parent"]].get("kind") == "lateral_branch"
        and int(branch.get("attach_link", -1))
        == int(branch_by_id[branch["parent"]]["n_links"])
        for branch in adapter.branches
    )
    if len(bodies) != expected_count:
        errors.append(f"physical links {len(bodies)} != expected {expected_count}")
    if len(fixed_joints) != expected_fixed:
        errors.append(f"fixed joints {len(fixed_joints)} != expected {expected_fixed}")
    if len(d6_joints) != expected_d6:
        errors.append(f"D6 joints {len(d6_joints)} != expected {expected_d6}")
    if len(organic_meshes) != expected_count:
        errors.append(
            f"organic meshes {len(organic_meshes)} != expected {expected_count}"
        )
    if len(capsules) != expected_count * 2:
        errors.append(f"capsule colliders {len(capsules)} != expected {expected_count * 2}")
    if len(resolved_axes) != expected_axes:
        errors.append(f"visual axes {len(resolved_axes)} != expected {expected_axes}")
    if cylinders:
        errors.append(f"checkpoint authored {len(cylinders)} forbidden visual cylinders")
    if len(paths) != len(set(paths)):
        errors.append("duplicate USD physical-link paths")
    if len(terminal_fork_shoots) != expected_terminal_forks:
        errors.append(
            f"terminal fork shoots {len(terminal_fork_shoots)} != expected "
            f"{expected_terminal_forks}"
        )
    if len(terminal_fork_leaves) != expected_terminal_forks:
        errors.append(
            f"terminal fork leaves {len(terminal_fork_leaves)} != expected "
            f"{expected_terminal_forks}"
        )

    authored_poses = []
    if adapter.pose_mode == "canonical":
        for prim in bodies:
            axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
            spec = spec_by_axis.get(axis_id)
            if spec is None:
                errors.append(f"body {prim.GetPath()} has unknown canonical axis {axis_id!r}")
                continue
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            position = matrix.ExtractTranslation()
            direction = Gf.Vec3d(
                matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
            ).GetNormalized()
            length = float(spec["length"]) * GLOBAL_SCALE
            endpoint = position + direction * length
            expected_position = Gf.Vec3d(
                *(float(spec["rest_frame"][row][3]) * GLOBAL_SCALE for row in range(3))
            )
            expected_direction = Gf.Vec3d(
                *(float(spec["rest_frame"][row][2]) for row in range(3))
            ).GetNormalized()
            expected_endpoint = expected_position + expected_direction * length
            if (position - expected_position).GetLength() > 1e-6:
                errors.append(f"rest-position mismatch for {prim.GetPath()}")
            if (direction - expected_direction).GetLength() > 1e-6:
                errors.append(f"rest-direction mismatch for {prim.GetPath()}")
            if (endpoint - expected_endpoint).GetLength() > 1e-6:
                errors.append(f"rest-endpoint mismatch for {prim.GetPath()}")
            expected_radius = float(spec["radius"]) * GLOBAL_SCALE
            actual_radius = float(prim.GetAttribute("autotom:visualRadius").Get())
            if not math.isclose(actual_radius, expected_radius, abs_tol=1e-6):
                errors.append(f"visual-radius mismatch for {prim.GetPath()}")
            authored_poses.append(
                {
                    "axis_id": axis_id,
                    "body_path": str(prim.GetPath()),
                    "source_frame": spec["rest_frame"],
                    "authored_start": [float(value) for value in position],
                    "authored_direction": [float(value) for value in direction],
                    "authored_endpoint": [float(value) for value in endpoint],
                }
            )

    filtered_overlaps, active_overlaps = _audit_capsule_overlaps(stage)
    if active_overlaps and adapter.pose_mode == "canonical":
        errors.append(
            f"{len(active_overlaps)} active capsule overlaps are not covered by collision filtering"
        )

    gains = [
        {
            "branch_id": branch.branch_id,
            "joint_type": "fixed" if branch.locked_joints else branch.joint_type,
            "stiffness": branch.gains.stiffness,
            "damping": branch.gains.damping,
            "attachment_stiffness": branch.gains.attachment_stiffness,
            "attachment_damping": branch.gains.attachment_damping,
            "link_masses": list(branch.link_masses),
            "visual_radii": list(branch.link_radii),
            "collider_radii": list(branch.link_collider_radii),
        }
        for branch in resolved
    ]
    authored_body_masses = []
    for prim in bodies:
        mass_api = UsdPhysics.MassAPI(prim)
        authored_body_masses.append(
            {
                "body_path": str(prim.GetPath()),
                "mass_kg": float(mass_api.GetMassAttr().Get()),
                "center_of_mass": [
                    float(value) for value in mass_api.GetCenterOfMassAttr().Get()
                ],
                "aggregated_leaf_visual_mass_kg": float(
                    prim.GetAttribute("autotom:aggregatedLeafVisualMassKg").Get()
                    or 0.0
                ),
            }
        )

    schema = {
        "stem": STEM_CHECKPOINT_SCHEMA,
        "laterals": LATERALS_CHECKPOINT_SCHEMA,
        "leaf-supports": LEAF_SUPPORTS_CHECKPOINT_SCHEMA,
        "leaves": LEAVES_CHECKPOINT_SCHEMA,
    }[adapter.debug_profile]
    expected = {
        "internodes": sum(spec.get("axis_role") == "internode" for spec in specs),
        "rigid_bodies": expected_count,
        "fixed_joints": expected_fixed,
        "d6_joints": expected_d6,
        "capsule_colliders": expected_count * 2,
        "organic_meshes": expected_count,
        "visual_axes": expected_axes,
    }
    if adapter.debug_profile in {"leaf-supports", "leaves"}:
        expected.update(
            {
                "leaf_organs": len(
                    {
                        organ_id
                        for organ_id in adapter.represented_organ_ids
                        if any(
                            organ.id == organ_id and organ.organ_type == "Leaf"
                            for organ in state.organs
                        )
                    }
                ),
                "leaf_visual_axes": sum(
                    axis.axis_id.startswith(("Leaf_", "LatLeaf_"))
                    for axis in resolved_axes
                ),
                "leaf_support_links": sum(
                    spec.get("axis_role") in {"petiole", "leaf_rachis"}
                    for spec in specs
                ),
                "degenerate_leaf_organs": len(adapter.degenerate_organs),
            }
        )
    if adapter.debug_profile == "leaves":
        expected.update(
            {
                "rigid_leaf_visuals": len(adapter.rigid_leaf_visuals),
                "petiolule_visual_meshes": len(adapter.rigid_leaf_visuals),
                "leaf_blades": len(adapter.rigid_leaf_visuals),
                "total_meshes": expected_count + 2 * len(adapter.rigid_leaf_visuals),
                "terminal_forks": expected_terminal_forks,
            }
        )
        expected["total_meshes"] += 2 * expected_terminal_forks
        if len(rigid_leaf_visuals) != len(adapter.rigid_leaf_visuals):
            errors.append(
                f"rigid leaf visuals {len(rigid_leaf_visuals)} != expected "
                f"{len(adapter.rigid_leaf_visuals)}"
            )
        if len(petiolule_meshes) != len(adapter.rigid_leaf_visuals):
            errors.append(
                f"petiolule meshes {len(petiolule_meshes)} != expected "
                f"{len(adapter.rigid_leaf_visuals)}"
            )
        if len(leaf_blades) != len(adapter.rigid_leaf_visuals):
            errors.append(
                f"leaf blades {len(leaf_blades)} != expected "
                f"{len(adapter.rigid_leaf_visuals)}"
            )
    return IncrementalCheckpointManifest(
        metadata={
            "day": state.metadata.simulation_time,
            "plant_id": state.metadata.plant_id,
            "debug_profile": adapter.debug_profile,
            "pose_mode": adapter.pose_mode,
            "physics_preset": physics_preset,
            "leaf_joint_policy": adapter.leaf_joint_policy,
            "visual_quality": adapter.visual_quality,
            "visual_profile": (
                dict(adapter.branches[0]["visual_profile"])
                if adapter.branches
                else None
            ),
            "visual_renderer": "historical_v2_skinned_segmented",
            "global_scale": GLOBAL_SCALE,
            "usd": str(usd_path),
        },
        expected=expected,
        authored={
            "rigid_bodies": len(bodies),
            "fixed_joints": len(fixed_joints),
            "d6_joints": len(d6_joints),
            "capsule_colliders": len(capsules),
            "organic_meshes": len(organic_meshes),
            "rigid_visual_organic_meshes": len(petiolule_meshes),
            "petiolule_visual_meshes": len(petiolule_meshes),
            "leaf_blades": len(leaf_blades),
            "terminal_fork_shoots": len(terminal_fork_shoots),
            "terminal_fork_leaves": len(terminal_fork_leaves),
            "total_meshes": len(meshes),
            "visual_axes": len(resolved_axes),
            "visual_cylinders": len(cylinders),
            "paths": paths,
            "poses": authored_poses,
            "mesh_complexity": mesh_complexity_by_role,
        },
        topology={
            "branches": [
                {
                    "id": branch["id"],
                    "parent": branch["parent"],
                    "attach_link": branch["attach_link"],
                    "joint_type": branch["joint_type"],
                    "visual_profile": branch.get("visual_profile"),
                    "link_axis_ids": [spec["canonical_axis_id"] for spec in branch["link_specs"]],
                    "link_groimp_node_ids": [spec["groimp_node_id"] for spec in branch["link_specs"]],
                }
                for branch in adapter.branches
            ],
            "attachment_map": list(adapter.attachment_map),
            "source_axis_ids": list(adapter.source_axis_ids),
            "represented_organ_ids": list(adapter.represented_organ_ids),
            "collapsed_duplicates": list(adapter.collapsed_duplicates),
            "degenerate_organs": list(adapter.degenerate_organs),
            "rigid_leaf_visuals": rigid_leaf_visuals,
        },
        physics={
            "branch_gains": gains,
            "authored_body_masses": authored_body_masses,
            "aggregated_leaf_visual_mass_kg": sum(
                float(record.get("aggregated_mass_kg", 0.0))
                for record in rigid_leaf_visuals
            ),
            "visual_axis_ids": [axis.axis_id for axis in resolved_axes],
            "leaf_support_policy": (
                {
                    "petiole": "d6",
                    "leaf_rachis": (
                        "d6_distributed"
                        if adapter.leaf_joint_policy == "distributed"
                        else "fixed_to_petiole"
                    ),
                    "policy": adapter.leaf_joint_policy,
                    "visual_geometry": "complete_canonical_segmented",
                }
                if adapter.debug_profile in {"leaf-supports", "leaves"}
                else None
            ),
        },
        collisions={
            "blocking_policy": (
                "canonical_active_overlaps_fail"
                if adapter.pose_mode == "canonical"
                else "legacy_diagnostic_only"
            ),
            "filtered_overlaps": filtered_overlaps,
            "active_overlaps": active_overlaps,
            "approved_native_groimp_filters": approved_collision_filters,
        },
        errors=tuple(errors),
        schema_version=schema,
    )


def _apply_approved_collision_filters(stage, adapter: StemBranchesResult) -> list[dict[str, Any]]:
    """Apply only explicitly approved source-overlap pairs, never a general rule."""

    if adapter.pose_mode != "canonical":
        return []
    body_by_axis = {}
    for prim in stage.Traverse():
        axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if axis_id is not None:
            body_by_axis[str(axis_id)] = str(prim.GetPath())

    applied = []
    for record in adapter.approved_collision_filters:
        axis_a = record["axis_a"]
        axis_b = record["axis_b"]
        body_a = body_by_axis.get(axis_a)
        body_b = body_by_axis.get(axis_b)
        if body_a is None or body_b is None:
            raise IncrementalCheckpointError(
                f"approved collision pair cannot resolve bodies: {axis_a}, {axis_b}"
            )
        add_collision_filter(stage, body_a, body_b)
        add_collision_filter(stage, body_b, body_a)
        applied.append({**record, "body_a": body_a, "body_b": body_b})
    return applied


def save_manifest(manifest: IncrementalCheckpointManifest, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            manifest.to_dict(),
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return path


def export_incremental_checkpoint(
    state: PlantState,
    output_path: str | Path,
    *,
    debug_profile: str,
    pose_mode: str = "canonical",
    physics_preset: str = "flexible",
    physics_hz: int = 480,
    leaf_joint_policy: str = "distributed",
    visual_quality: str = "realistic",
) -> tuple[IncrementalCheckpointPlan, Path, Path]:
    """Build and audit one PlantState profile with the original V2 backend."""

    if debug_profile not in INCREMENTAL_PROFILES:
        raise IncrementalCheckpointError(
            f"incremental profile must be one of {INCREMENTAL_PROFILES}, got {debug_profile!r}"
        )
    if physics_preset not in ("locked", "flexible"):
        raise IncrementalCheckpointError(
            f"physics_preset must be locked or flexible, got {physics_preset!r}"
        )
    if leaf_joint_policy not in LEAF_JOINT_POLICIES:
        raise IncrementalCheckpointError(
            "leaf_joint_policy must be one of "
            f"{LEAF_JOINT_POLICIES}, got {leaf_joint_policy!r}"
        )
    if visual_quality not in VISUAL_QUALITY_MODES:
        raise IncrementalCheckpointError(
            "visual_quality must be one of "
            f"{VISUAL_QUALITY_MODES}, got {visual_quality!r}"
        )
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    adapter_builders = {
        "stem": build_stem_branches,
        "laterals": build_lateral_branches,
        "leaf-supports": build_leaf_support_branches,
        "leaves": build_leaf_branches,
    }
    adapter_kwargs = {
        "pose_mode": pose_mode,
        "visual_quality": visual_quality,
    }
    if debug_profile in {"leaf-supports", "leaves"}:
        adapter_kwargs["leaf_joint_policy"] = leaf_joint_policy
    adapter = adapter_builders[debug_profile](state, **adapter_kwargs)
    locked = physics_preset == "locked"
    stage, stem_path = build_stage(
        str(destination),
        branches=list(adapter.branches),
        terminal_bodies=[],
        locked_joints=locked,
        branch_backend="skinned",
        skinning_visual_mode="segmented",
    )
    _author_stage_metadata(stage, state, adapter, physics_preset)
    rigid_leaf_visuals = _author_rigid_leaf_visuals(stage, adapter)
    approved_filters = _apply_approved_collision_filters(stage, adapter)
    apply_physx_scene_settings(stage, physics_hz=physics_hz)
    apply_physx_articulation_settings(stage, stem_path)
    stage.GetRootLayer().Save()
    manifest = _audit_stage(
        stage,
        state,
        adapter,
        physics_preset,
        destination,
        approved_filters,
        rigid_leaf_visuals,
    )
    manifest_path = save_manifest(manifest, manifest_path_for(destination))
    if manifest.errors:
        raise IncrementalCheckpointError("; ".join(manifest.errors))
    predicted_d6 = (
        sum(branch["n_links"] for branch in adapter.branches if branch["joint_type"] != "fixed")
        if physics_preset == "flexible"
        else 0
    )
    return (
        IncrementalCheckpointPlan(
            adapter=adapter,
            physical_link_count=sum(branch["n_links"] for branch in adapter.branches),
            predicted_d6_joints=predicted_d6,
            debug_profile=debug_profile,
        ),
        destination,
        manifest_path,
    )


def export_stem_checkpoint(
    state: PlantState,
    output_path: str | Path,
    *,
    pose_mode: str = "canonical",
    physics_preset: str = "flexible",
    physics_hz: int = 480,
) -> tuple[IncrementalCheckpointPlan, Path, Path]:
    """Compatibility wrapper for the completed stem checkpoint."""

    return export_incremental_checkpoint(
        state,
        output_path,
        debug_profile="stem",
        pose_mode=pose_mode,
        physics_preset=physics_preset,
        physics_hz=physics_hz,
    )
