"""PlantState checkpoints routed through the established ExporterV2 backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Any

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from plant_state import PlantState

from .core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
from .core.skinning import build_visual_axes, resolve_vegetative_graph
from .core.tree_config import GLOBAL_SCALE
from .core.usd import build_stage
from .plant_state_branches import (
    StemBranchesResult,
    build_lateral_branches,
    build_stem_branches,
)


STEM_CHECKPOINT_SCHEMA = "exporter_v2_stem_checkpoint/1.0"
LATERALS_CHECKPOINT_SCHEMA = "exporter_v2_laterals_checkpoint/1.0"
INCREMENTAL_PROFILES = ("stem", "laterals")


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
    cylinders = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Cylinder)]
    capsules = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Capsule)]
    paths = [str(prim.GetPath()) for prim in bodies]
    errors = []

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
    if len(bodies) != expected_count:
        errors.append(f"physical links {len(bodies)} != expected {expected_count}")
    if len(fixed_joints) != expected_fixed:
        errors.append(f"fixed joints {len(fixed_joints)} != expected {expected_fixed}")
    if len(d6_joints) != expected_d6:
        errors.append(f"D6 joints {len(d6_joints)} != expected {expected_d6}")
    if len(meshes) != expected_count:
        errors.append(f"organic meshes {len(meshes)} != expected {expected_count}")
    if len(capsules) != expected_count * 2:
        errors.append(f"capsule colliders {len(capsules)} != expected {expected_count * 2}")
    if len(resolved_axes) != expected_axes:
        errors.append(f"visual axes {len(resolved_axes)} != expected {expected_axes}")
    if cylinders:
        errors.append(f"checkpoint authored {len(cylinders)} forbidden visual cylinders")
    if len(paths) != len(set(paths)):
        errors.append("duplicate USD physical-link paths")

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

    schema = STEM_CHECKPOINT_SCHEMA if adapter.debug_profile == "stem" else LATERALS_CHECKPOINT_SCHEMA
    return IncrementalCheckpointManifest(
        metadata={
            "day": state.metadata.simulation_time,
            "plant_id": state.metadata.plant_id,
            "debug_profile": adapter.debug_profile,
            "pose_mode": adapter.pose_mode,
            "physics_preset": physics_preset,
            "global_scale": GLOBAL_SCALE,
            "usd": str(usd_path),
        },
        expected={
            "internodes": expected_count,
            "rigid_bodies": expected_count,
            "fixed_joints": expected_fixed,
            "d6_joints": expected_d6,
            "capsule_colliders": expected_count * 2,
            "organic_meshes": expected_count,
            "visual_axes": expected_axes,
        },
        authored={
            "rigid_bodies": len(bodies),
            "fixed_joints": len(fixed_joints),
            "d6_joints": len(d6_joints),
            "capsule_colliders": len(capsules),
            "organic_meshes": len(meshes),
            "visual_axes": len(resolved_axes),
            "visual_cylinders": len(cylinders),
            "paths": paths,
            "poses": authored_poses,
        },
        topology={
            "branches": [
                {
                    "id": branch["id"],
                    "parent": branch["parent"],
                    "attach_link": branch["attach_link"],
                    "joint_type": branch["joint_type"],
                    "link_axis_ids": [spec["canonical_axis_id"] for spec in branch["link_specs"]],
                    "link_groimp_node_ids": [spec["groimp_node_id"] for spec in branch["link_specs"]],
                }
                for branch in adapter.branches
            ],
            "attachment_map": list(adapter.attachment_map),
            "source_axis_ids": list(adapter.source_axis_ids),
            "represented_organ_ids": list(adapter.represented_organ_ids),
            "collapsed_duplicates": list(adapter.collapsed_duplicates),
        },
        physics={
            "branch_gains": gains,
            "visual_axis_ids": [axis.axis_id for axis in resolved_axes],
        },
        collisions={
            "blocking_policy": (
                "canonical_active_overlaps_fail"
                if adapter.pose_mode == "canonical"
                else "legacy_diagnostic_only"
            ),
            "filtered_overlaps": filtered_overlaps,
            "active_overlaps": active_overlaps,
        },
        errors=tuple(errors),
        schema_version=schema,
    )


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
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    adapter = (
        build_stem_branches(state, pose_mode=pose_mode)
        if debug_profile == "stem"
        else build_lateral_branches(state, pose_mode=pose_mode)
    )
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
    apply_physx_scene_settings(stage, physics_hz=physics_hz)
    apply_physx_articulation_settings(stage, stem_path)
    stage.GetRootLayer().Save()
    manifest = _audit_stage(stage, state, adapter, physics_preset, destination)
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
