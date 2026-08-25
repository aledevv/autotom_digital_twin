from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom

from exporterV2.cli import main as exporter_main
from exporterV2.core.skinning.adapter import resolve_vegetative_graph
from exporterV2.core.tree_config import validate_branches
from exporterV2.plant_state_branches import build_leaf_support_branches
from exporterV2.plant_state_legacy_backend import export_incremental_checkpoint
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DAY50_PATH = PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_50.json"


@pytest.fixture(scope="module")
def day50_state():
    return load_plant_state(DAY50_PATH)


def test_day50_leaf_support_adapter_is_native_and_complete(day50_state):
    result = build_leaf_support_branches(day50_state)
    branches = list(result.branches)
    kinds = [branch["kind"] for branch in branches]
    assert kinds.count("stem") == 1
    assert kinds.count("lateral_branch") == 4
    assert kinds.count("leaf_petiole") == 27
    assert kinds.count("leaf_rachis") == 23
    assert sum(branch["n_links"] for branch in branches) == 81
    assert len({branch["visual_axis_id"] for branch in branches}) == 32
    assert len(result.approved_collision_filters) == 3
    assert result.collapsed_duplicates == ()
    assert {
        branch["joint_type"]
        for branch in branches
        if branch["kind"] == "leaf_petiole"
    } == {"d6"}
    assert {
        branch["joint_type"]
        for branch in branches
        if branch["kind"] == "leaf_rachis"
    } == {"d6"}
    assert result.leaf_joint_policy == "distributed"

    leaf_organs = {
        organ.id for organ in day50_state.organs if organ.organ_type == "Leaf"
    }
    assert leaf_organs <= set(result.represented_organ_ids)
    assert result.degenerate_organs == (
        {
            "organ_id": "organ:421489",
            "node_id": "node:421489",
            "groimp_node_id": 421489,
            "xform_name": "Leaf_r9_o0_g421489",
            "axis_ids": [
                "node-421489:petiole:0",
                "node-421489:leaf_rachis:0",
                "node-421489:leaf_rachis:1",
            ],
            "reason": "non-positive canonical leaf-support dimensions",
        },
    )
    assert next(
        branch for branch in branches if branch["id"] == "LatLeaf_r1_o0_g421243_petiole"
    )["parent"] == "Branch_s2_o0_g421238"
    assert next(
        branch for branch in branches if branch["id"] == "LatLeaf_r3_o0_g421423_petiole"
    )["attach_link"] == 3
    validate_branches(branches)


def test_leaf_radii_and_frames_come_from_groimp(day50_state):
    result = build_leaf_support_branches(day50_state)
    branches = {branch["id"]: branch for branch in result.branches}
    petiole = branches["Leaf_r2_o0_g421110_petiole"]["link_specs"][0]
    rachis = branches["Leaf_r2_o0_g421110_rachis"]["link_specs"][0]
    axes = {axis.id: axis for axis in day50_state.axes}
    assert petiole["radius"] == pytest.approx(
        axes["node-421110:petiole:0"].radius
    )
    assert rachis["radius"] == pytest.approx(
        axes["node-421110:leaf_rachis:0"].radius
    )
    assert rachis["radius"] != pytest.approx(petiole["radius"] * 0.6)
    assert [row[:3] for row in petiole["rest_frame"][:3]] == [
        list(row[:3]) for row in axes["node-421110:petiole:0"].world_frame[:3]
    ]
    assert axes["node-421110:petiole:0"].world_end == pytest.approx(
        axes["node-421110:leaf_rachis:0"].world_start
    )


def test_legacy_leaf_mode_uses_procedural_pose_without_filtering_organs(day50_state):
    canonical = build_leaf_support_branches(day50_state)
    legacy = build_leaf_support_branches(day50_state, pose_mode="legacy")
    assert len(canonical.represented_organ_ids) == len(legacy.represented_organ_ids)
    assert all(
        "rest_frame" not in spec
        for branch in legacy.branches
        for spec in branch["link_specs"]
    )
    assert legacy.approved_collision_filters == canonical.approved_collision_filters
    resolved = resolve_vegetative_graph(
        list(legacy.branches),
        all_branch_defs={branch["id"]: branch for branch in legacy.branches},
    )
    assert sum(branch.n_links for branch in resolved) == 81


def test_optimized_leaf_policy_preserves_rigid_rachides(day50_state):
    result = build_leaf_support_branches(
        day50_state, leaf_joint_policy="optimized"
    )
    assert result.leaf_joint_policy == "optimized"
    assert {
        branch["joint_type"]
        for branch in result.branches
        if branch["kind"] == "leaf_petiole"
    } == {"d6"}
    assert {
        branch["joint_type"]
        for branch in result.branches
        if branch["kind"] == "leaf_rachis"
    } == {"fixed"}
    assert sum(
        branch["n_links"]
        for branch in result.branches
        if branch["joint_type"] != "fixed"
    ) == 43


def test_approved_filters_are_bound_to_the_validated_day50_source(day50_state):
    changed = replace(
        day50_state,
        metadata=replace(day50_state.metadata, source_project_sha256="0" * 64),
    )
    assert build_leaf_support_branches(changed).approved_collision_filters == ()


def test_day50_flexible_leaf_support_stage_and_manifest(tmp_path, day50_state):
    plan, usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "leaf_supports.usda",
        debug_profile="leaf-supports",
        pose_mode="canonical",
        physics_preset="flexible",
    )
    assert plan.physical_link_count == 81
    assert plan.predicted_d6_joints == 71
    stage = Usd.Stage.Open(str(usd_path))
    assert stage is not None
    assert sum(prim.GetTypeName() == "PhysicsFixedJoint" for prim in stage.Traverse()) == 10
    assert sum(prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()) == 71
    assert sum(prim.IsA(UsdGeom.Capsule) for prim in stage.Traverse()) == 162
    assert sum(prim.IsA(UsdGeom.Mesh) for prim in stage.Traverse()) == 81
    assert sum(prim.IsA(UsdGeom.Cylinder) for prim in stage.Traverse()) == 0
    assert stage.GetPrimAtPath(
        "/World/Stem/Vegetative/Leaf_r9_o0_g421489"
    ).GetAttribute("autotom:entityKind").Get() == "degenerate_organ"

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "exporter_v2_leaf_supports_checkpoint/1.0"
    assert manifest["expected"] == {
        "capsule_colliders": 162,
        "d6_joints": 71,
        "degenerate_leaf_organs": 1,
        "fixed_joints": 10,
        "internodes": 26,
        "leaf_organs": 28,
        "leaf_support_links": 55,
        "leaf_visual_axes": 27,
        "organic_meshes": 81,
        "rigid_bodies": 81,
        "visual_axes": 32,
    }
    assert len(manifest["collisions"]["approved_native_groimp_filters"]) == 3
    assert manifest["collisions"]["active_overlaps"] == []
    assert manifest["authored"]["visual_cylinders"] == 0
    assert manifest["physics"]["leaf_support_policy"] == {
        "leaf_rachis": "d6_distributed",
        "petiole": "d6",
        "policy": "distributed",
        "visual_geometry": "complete_canonical_segmented",
    }
    assert manifest["metadata"]["leaf_joint_policy"] == "distributed"
    assert manifest["errors"] == []
    assert {
        prim.GetAttribute("autotom:role").Get()
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
    } == {"internode", "petiole", "leaf_rachis"}


def test_locked_leaf_support_stage_is_all_fixed(tmp_path, day50_state):
    plan, _usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "leaf_supports_locked.usda",
        debug_profile="leaf-supports",
        physics_preset="locked",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert plan.predicted_d6_joints == 0
    assert manifest["authored"]["fixed_joints"] == 81
    assert manifest["authored"]["d6_joints"] == 0


def test_optimized_leaf_support_stage_retains_checkpoint_counts(
    tmp_path, day50_state
):
    plan, usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "leaf_supports_optimized.usda",
        debug_profile="leaf-supports",
        physics_preset="flexible",
        leaf_joint_policy="optimized",
    )
    assert plan.predicted_d6_joints == 43
    stage = Usd.Stage.Open(str(usd_path))
    assert sum(
        prim.GetTypeName() == "PhysicsFixedJoint" for prim in stage.Traverse()
    ) == 38
    assert sum(
        prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()
    ) == 43
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["metadata"]["leaf_joint_policy"] == "optimized"
    assert manifest["physics"]["leaf_support_policy"]["leaf_rachis"] == (
        "fixed_to_petiole"
    )


def test_leaf_support_cli_manifest_is_deterministic(tmp_path):
    destination = tmp_path / "cli_leaf_supports.usda"
    arguments = [
        "--day", "50",
        "--debug-profile", "leaf-supports",
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
