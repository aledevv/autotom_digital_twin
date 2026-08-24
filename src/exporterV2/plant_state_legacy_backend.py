"""PlantState checkpoint routed through the established ExporterV2 backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from plant_state import PlantState

from .core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
from .core.tree_config import GLOBAL_SCALE
from .core.usd import build_stage
from .plant_state_branches import StemBranchesResult, build_stem_branches


STEM_CHECKPOINT_SCHEMA = "exporter_v2_stem_checkpoint/1.0"


class StemCheckpointError(ValueError):
    """Raised when the conservative stem export fails its audit."""


@dataclass(frozen=True)
class StemCheckpointManifest:
    metadata: dict[str, Any]
    expected: dict[str, int]
    authored: dict[str, Any]
    topology: dict[str, Any]
    errors: tuple[str, ...] = ()
    schema_version: str = STEM_CHECKPOINT_SCHEMA

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class StemCheckpointPlan:
    adapter: StemBranchesResult
    physical_link_count: int
    predicted_d6_joints: int = 0
    debug_profile: str = "stem"


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
    _custom(stem, "autotom:debugProfile", Sdf.ValueTypeNames.String, "stem")
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


def _audit_stage(
    stage,
    state: PlantState,
    adapter: StemBranchesResult,
    physics_preset: str,
    usd_path: Path,
) -> StemCheckpointManifest:
    branch = adapter.branches[0]
    specs = branch["link_specs"]
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
    expected_count = len(specs)
    if len(bodies) != expected_count:
        errors.append(f"physical links {len(bodies)} != expected {expected_count}")
    if len(fixed_joints) != expected_count:
        errors.append(f"fixed joints {len(fixed_joints)} != expected {expected_count}")
    if d6_joints:
        errors.append(f"stem authored {len(d6_joints)} unexpected D6 joints")
    if len(meshes) != expected_count:
        errors.append(f"organic meshes {len(meshes)} != expected {expected_count}")
    if len(capsules) != expected_count * 2:
        errors.append(f"capsule colliders {len(capsules)} != expected {expected_count * 2}")
    if cylinders:
        errors.append(f"stem authored {len(cylinders)} forbidden visual cylinders")
    if len(paths) != len(set(paths)):
        errors.append("duplicate USD physical-link paths")

    if adapter.pose_mode == "canonical":
        for prim, spec in zip(bodies, specs, strict=True):
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            position = matrix.ExtractTranslation()
            expected_position = Gf.Vec3d(
                *(float(spec["rest_frame"][row][3]) * GLOBAL_SCALE for row in range(3))
            )
            if (position - expected_position).GetLength() > 1e-6:
                errors.append(f"rest-position mismatch for {prim.GetPath()}")
            direction = Gf.Vec3d(matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))).GetNormalized()
            expected_direction = Gf.Vec3d(
                *(float(spec["rest_frame"][row][2]) for row in range(3))
            ).GetNormalized()
            if (direction - expected_direction).GetLength() > 1e-6:
                errors.append(f"rest-direction mismatch for {prim.GetPath()}")

    return StemCheckpointManifest(
        metadata={
            "day": state.metadata.simulation_time,
            "plant_id": state.metadata.plant_id,
            "pose_mode": adapter.pose_mode,
            "physics_preset": physics_preset,
            "global_scale": GLOBAL_SCALE,
            "usd": str(usd_path),
        },
        expected={
            "internodes": expected_count,
            "rigid_bodies": expected_count,
            "fixed_joints": expected_count,
            "capsule_colliders": expected_count * 2,
            "organic_meshes": expected_count,
        },
        authored={
            "rigid_bodies": len(bodies),
            "fixed_joints": len(fixed_joints),
            "d6_joints": len(d6_joints),
            "capsule_colliders": len(capsules),
            "organic_meshes": len(meshes),
            "visual_cylinders": len(cylinders),
            "paths": paths,
        },
        topology={
            "branch_id": branch["id"],
            "joint_type": branch["joint_type"],
            "source_axis_ids": list(adapter.source_axis_ids),
            "represented_organ_ids": list(adapter.represented_organ_ids),
            "collapsed_duplicates": list(adapter.collapsed_duplicates),
        },
        errors=tuple(errors),
    )


def save_manifest(manifest: StemCheckpointManifest, path: Path) -> Path:
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


def export_stem_checkpoint(
    state: PlantState,
    output_path: str | Path,
    *,
    pose_mode: str = "canonical",
    physics_preset: str = "flexible",
    physics_hz: int = 480,
) -> tuple[StemCheckpointPlan, Path, Path]:
    """Build and audit the stem with the original V2 segmented backend."""

    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    adapter = build_stem_branches(state, pose_mode=pose_mode)
    stage, stem_path = build_stage(
        str(destination),
        branches=list(adapter.branches),
        terminal_bodies=[],
        branch_backend="skinned",
        skinning_visual_mode="segmented",
    )
    _author_stage_metadata(stage, state, adapter, physics_preset)
    apply_physx_scene_settings(stage, physics_hz=physics_hz)
    apply_physx_articulation_settings(stage, stem_path)
    stage.GetRootLayer().Save()
    manifest = _audit_stage(
        stage, state, adapter, physics_preset, destination
    )
    manifest_path = save_manifest(manifest, manifest_path_for(destination))
    if manifest.errors:
        raise StemCheckpointError("; ".join(manifest.errors))
    return (
        StemCheckpointPlan(adapter=adapter, physical_link_count=len(adapter.branches[0]["link_specs"])),
        destination,
        manifest_path,
    )

