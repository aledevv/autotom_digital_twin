from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from exporterV2.cli import main as exporter_main
from exporterV2.plant_state_branches import (
    PlantStateBranchesError,
    build_leaf_branches,
)
from exporterV2.plant_state_legacy_backend import export_incremental_checkpoint
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]
DAY50_PATH = PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_50.json"


@pytest.fixture(scope="module")
def day50_state():
    return load_plant_state(DAY50_PATH)


def test_day50_rigid_leaf_visual_adapter_is_complete(day50_state):
    result = build_leaf_branches(day50_state)
    assert result.debug_profile == "leaves"
    assert len(result.rigid_leaf_visuals) == 131
    assert Counter(record["role"] for record in result.rigid_leaf_visuals) == {
        "petiolule_left": 53,
        "petiolule_right": 53,
        "rachis_terminal": 25,
    }
    assert len(result.branches) == 55
    assert sum(branch["n_links"] for branch in result.branches) == 81
    assert sum(
        branch["n_links"]
        for branch in result.branches
        if branch["joint_type"] != "fixed"
    ) == 43
    assert all(
        record["host_axis_id"].endswith(("petiole:0", "leaf_rachis:0", "leaf_rachis:1"))
        for record in result.rigid_leaf_visuals
    )
    assert all(record["host_fraction"] in {0.0, 1.0} for record in result.rigid_leaf_visuals)


def test_degenerate_leaf_visuals_remain_metadata_only(day50_state):
    result = build_leaf_branches(day50_state)
    assert not any(
        record["groimp_node_id"] == 421489
        for record in result.rigid_leaf_visuals
    )
    assert {
        axis_id
        for axis_id in result.source_axis_ids
        if axis_id.startswith("node-421489:petiolule")
        or axis_id.startswith("node-421489:rachis_terminal")
    } == {
        "node-421489:petiolule_left:0",
        "node-421489:petiolule_left:1",
        "node-421489:petiolule_left:2",
        "node-421489:petiolule_right:0",
        "node-421489:petiolule_right:1",
        "node-421489:petiolule_right:2",
        "node-421489:rachis_terminal:2",
    }


def test_leaves_checkpoint_requires_canonical_frames(day50_state):
    with pytest.raises(PlantStateBranchesError, match="requires canonical"):
        build_leaf_branches(day50_state, pose_mode="legacy")


def test_day50_leaves_are_visual_only_and_do_not_expand_physics(
    tmp_path, day50_state
):
    plan, usd_path, manifest_path = export_incremental_checkpoint(
        day50_state,
        tmp_path / "leaves.usda",
        debug_profile="leaves",
        physics_preset="flexible",
    )
    assert plan.physical_link_count == 81
    assert plan.predicted_d6_joints == 43
    stage = Usd.Stage.Open(str(usd_path))
    assert stage is not None

    bodies = [
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
    ]
    visuals = [
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "rigid_leaf_visual"
    ]
    assert len(bodies) == 81
    assert len(visuals) == 131
    assert all(not prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in visuals)
    assert all(not prim.HasAPI(UsdPhysics.CollisionAPI) for prim in visuals)
    assert sum(prim.GetTypeName() == "PhysicsFixedJoint" for prim in stage.Traverse()) == 38
    assert sum(prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()) == 43
    assert sum(prim.IsA(UsdGeom.Capsule) for prim in stage.Traverse()) == 162
    assert sum(prim.GetName() == "PetioluleVisual" for prim in stage.Traverse()) == 131
    assert sum(prim.GetName() == "LeafBlade" for prim in stage.Traverse()) == 131
    assert sum(prim.IsA(UsdGeom.Mesh) for prim in stage.Traverse()) == 343

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "exporter_v2_leaves_checkpoint/1.0"
    assert manifest["expected"]["rigid_leaf_visuals"] == 131
    assert manifest["expected"]["leaf_blades"] == 131
    assert manifest["expected"]["total_meshes"] == 343
    assert manifest["authored"]["mesh_complexity"]["all"]["triangles"] < 40_000
    assert manifest["authored"]["mesh_complexity"]["organic"]["triangles"] < 23_000
    assert manifest["authored"]["mesh_complexity"]["petiolules"]["triangles"] == 7_860
    assert manifest["collisions"]["active_overlaps"] == []
    assert manifest["errors"] == []


def test_leaves_cli_manifest_is_deterministic(tmp_path):
    destination = tmp_path / "cli_leaves.usda"
    arguments = [
        "--day", "50",
        "--debug-profile", "leaves",
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
