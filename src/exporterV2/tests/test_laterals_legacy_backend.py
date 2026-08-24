from __future__ import annotations

import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom

from exporterV2.cli import main as exporter_main
from exporterV2.core.skinning.adapter import resolve_vegetative_graph
from exporterV2.core.tree_config import validate_branches
from exporterV2.plant_state_branches import build_lateral_branches
from exporterV2.plant_state_legacy_backend import export_incremental_checkpoint
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DAY50_PATH = PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_50.json"


@pytest.fixture(scope="module")
def day50_state():
    return load_plant_state(DAY50_PATH)


def test_day50_native_lateral_chains_and_attachments(day50_state):
    result = build_lateral_branches(day50_state)
    branches = list(result.branches)
    assert [(branch["id"], branch["parent"], branch["attach_link"], branch["n_links"]) for branch in branches] == [
        ("trunk", None, None, 10),
        ("Branch_s2_o0_g421238", "trunk", 2, 4),
        ("Branch_s2_o1_g421244", "trunk", 2, 4),
        ("Branch_s3_o0_g421250", "trunk", 3, 4),
        ("Branch_s7_o0_g421301", "trunk", 7, 4),
    ]
    assert [
        [spec["groimp_node_id"] for spec in branch["link_specs"]]
        for branch in branches[1:]
    ] == [
        [421238, 421317, 421412, 421499],
        [421244, 421323, 421418, 421505],
        [421250, 421329, 421424, 421511],
        [421301, 421388, 421467, 421554],
    ]
    assert [item["parent_groimp_node_id"] for item in result.attachment_map] == [
        421099,
        421099,
        421107,
        421217,
    ]
    assert result.collapsed_duplicates == ()
    assert len(set(result.source_axis_ids)) == 26
    validate_branches(branches)


def test_canonical_and_legacy_lateral_pose_modes(day50_state):
    canonical = build_lateral_branches(day50_state, pose_mode="canonical")
    legacy = build_lateral_branches(day50_state, pose_mode="legacy")
    assert all(
        "rest_frame" in spec
        for branch in canonical.branches
        for spec in branch["link_specs"]
    )
    assert all(
        "rest_frame" not in spec
        for branch in legacy.branches
        for spec in branch["link_specs"]
    )
    lateral = list(legacy.branches)[1:]
    assert all(branch["tilt"] == pytest.approx(45.0) for branch in lateral)
    assert lateral[0]["rot"] != lateral[1]["rot"]
    resolved = resolve_vegetative_graph(list(canonical.branches))
    assert sum(branch.n_links for branch in resolved) == 26
    assert sum(not branch.locked_joints for branch in resolved) == 4


def test_day50_flexible_stage_counts_pose_and_collisions(tmp_path, day50_state):
    plan, usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "laterals.usda",
        debug_profile="laterals",
        pose_mode="canonical",
        physics_preset="flexible",
    )
    assert plan.physical_link_count == 26
    assert plan.predicted_d6_joints == 16
    stage = Usd.Stage.Open(str(usd_path))
    assert stage is not None
    assert sum(prim.GetTypeName() == "PhysicsFixedJoint" for prim in stage.Traverse()) == 10
    assert sum(prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()) == 16
    assert sum(prim.IsA(UsdGeom.Capsule) for prim in stage.Traverse()) == 52
    assert sum(prim.IsA(UsdGeom.Mesh) for prim in stage.Traverse()) == 26
    assert sum(prim.IsA(UsdGeom.Cylinder) for prim in stage.Traverse()) == 0
    expected_path = (
        "/World/Stem/Vegetative/Branch_s2_o0_g421238/"
        "Branch_s2_o0_g421238_Link_01_Internode_g421238"
    )
    assert stage.GetPrimAtPath(expected_path).IsValid()

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "exporter_v2_laterals_checkpoint/1.0"
    assert manifest["expected"] == {
        "capsule_colliders": 52,
        "d6_joints": 16,
        "fixed_joints": 10,
        "internodes": 26,
        "organic_meshes": 26,
        "rigid_bodies": 26,
        "visual_axes": 5,
    }
    assert manifest["authored"]["visual_cylinders"] == 0
    assert len(manifest["authored"]["poses"]) == 26
    assert manifest["collisions"]["filtered_overlaps"]
    assert manifest["collisions"]["active_overlaps"] == []
    assert manifest["errors"] == []


def test_locked_preset_replaces_lateral_d6_with_fixed(tmp_path, day50_state):
    plan, _usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "laterals_locked.usda",
        debug_profile="laterals",
        physics_preset="locked",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert plan.predicted_d6_joints == 0
    assert manifest["authored"]["fixed_joints"] == 26
    assert manifest["authored"]["d6_joints"] == 0


def test_legacy_overlap_is_reported_as_non_blocking_diagnostic(tmp_path, day50_state):
    _plan, _usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "laterals_legacy.usda",
        debug_profile="laterals",
        pose_mode="legacy",
        physics_preset="flexible",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["collisions"]["blocking_policy"] == "legacy_diagnostic_only"
    assert manifest["collisions"]["active_overlaps"]
    assert manifest["errors"] == []


def test_laterals_cli_and_manifest_are_deterministic(tmp_path):
    destination = tmp_path / "cli_laterals.usda"
    arguments = [
        "--day", "50",
        "--debug-profile", "laterals",
        "--pose-mode", "canonical",
        "--physics-preset", "flexible",
        "--input", str(DAY50_PATH),
        "--output", str(destination),
        "--generate-only",
    ]
    assert exporter_main(arguments) == 0
    manifest_path = destination.with_suffix(".manifest.json")
    first = manifest_path.read_bytes()
    assert exporter_main(arguments) == 0
    assert manifest_path.read_bytes() == first
