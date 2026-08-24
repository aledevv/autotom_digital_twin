from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from exporterV2.cli import main as exporter_main
from exporterV2.core.skinning.adapter import resolve_vegetative_graph
from exporterV2.core.tree_config import GLOBAL_SCALE, validate_branches
from exporterV2.plant_state_branches import build_stem_branches
from exporterV2.plant_state_legacy_backend import export_stem_checkpoint
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope="module")
def day10_state():
    return load_plant_state(
        PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_10.json"
    )


def test_stem_adapter_preserves_five_main_internodes(day10_state):
    result = build_stem_branches(day10_state)
    branch = result.branches[0]
    assert branch["id"] == "trunk"
    assert branch["joint_type"] == "fixed"
    assert branch["n_links"] == 5
    assert [spec["groimp_node_id"] for spec in branch["link_specs"]] == [
        421092,
        421099,
        421107,
        421158,
        421177,
    ]
    assert all("rest_frame" in spec for spec in branch["link_specs"])
    assert branch["link_specs"][0]["rest_frame"][0][3] == pytest.approx(0.0)
    assert branch["link_specs"][0]["rest_frame"][1][3] == pytest.approx(0.0)
    assert branch["link_specs"][0]["rest_frame"][2][3] == pytest.approx(0.0)


def test_legacy_pose_mode_keeps_dimensions_but_omits_frames(day10_state):
    branch = build_stem_branches(day10_state, pose_mode="legacy").branches[0]
    assert len(branch["link_specs"]) == 5
    assert all("rest_frame" not in spec for spec in branch["link_specs"])
    resolved = resolve_vegetative_graph([branch])[0]
    assert all(base[0] == pytest.approx(0.0) for base in resolved.link_bases)
    assert all(base[1] == pytest.approx(0.0) for base in resolved.link_bases)


def test_link_specs_reject_partial_poses(day10_state):
    branch = copy.deepcopy(build_stem_branches(day10_state).branches[0])
    del branch["link_specs"][2]["rest_frame"]
    with pytest.raises(ValueError, match="cannot mix explicit and legacy"):
        validate_branches([branch])


def test_legacy_branch_paths_are_unchanged_without_link_specs():
    branch = {
        "id": "trunk",
        "system": "vegetative",
        "parent": None,
        "attach_link": None,
        "n_links": 2,
        "radius": 0.01,
        "height": 0.10,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "fixed",
    }
    resolved = resolve_vegetative_graph([branch])[0]
    assert resolved.link_paths == [
        "/World/Stem/Vegetative/trunk/trunk_Link_01",
        "/World/Stem/Vegetative/trunk/trunk_Link_02",
    ]
    assert resolved.link_lengths == pytest.approx([0.20, 0.20])
    assert resolved.link_radii == pytest.approx([0.02, 0.02])


def test_stem_stage_uses_segmented_meshes_and_explicit_frames(tmp_path, day10_state):
    _, usd_path, manifest_path = export_stem_checkpoint(
        day10_state, tmp_path / "stem.usda", physics_preset="flexible"
    )
    stage = Usd.Stage.Open(str(usd_path))
    bodies = [
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
    ]
    assert len(bodies) == 5
    assert sum(prim.GetTypeName() == "PhysicsFixedJoint" for prim in stage.Traverse()) == 5
    assert sum(prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()) == 0
    assert sum(prim.IsA(UsdGeom.Mesh) for prim in stage.Traverse()) == 5
    assert sum(prim.IsA(UsdGeom.Capsule) for prim in stage.Traverse()) == 10
    assert sum(prim.IsA(UsdGeom.Cylinder) for prim in stage.Traverse()) == 0
    assert bodies[0].GetName() == "trunk_Link_01_Internode_g421092"

    axis_by_id = {
        axis.id: axis for axis in day10_state.axes if axis.role == "internode"
    }
    root = next(node for node in day10_state.nodes if node.id == day10_state.root_node_id)
    for body in bodies:
        axis = axis_by_id[body.GetAttribute("autotom:canonicalPrimitiveId").Get()]
        matrix = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0)
        expected_start = Gf.Vec3d(
            *((axis.world_start[index] - root.pose.world_start[index]) * GLOBAL_SCALE for index in range(3))
        )
        assert (matrix.ExtractTranslation() - expected_start).GetLength() <= 1e-6
        direction = Gf.Vec3d(matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))).GetNormalized()
        assert (direction - Gf.Vec3d(*axis.world_direction)).GetLength() <= 1e-6
        assert body.GetAttribute("autotom:sourceLength").Get() == pytest.approx(
            axis.length * GLOBAL_SCALE
        )
        assert body.GetAttribute("autotom:visualRadius").Get() == pytest.approx(
            axis.radius * GLOBAL_SCALE
        )

    manifest = json.loads(manifest_path.read_text())
    assert manifest["schema_version"] == "exporter_v2_stem_checkpoint/1.0"
    assert manifest["errors"] == []
    assert manifest["authored"]["visual_cylinders"] == 0


def test_stem_cli_writes_loadable_output(tmp_path):
    destination = tmp_path / "cli_stem.usda"
    assert exporter_main([
        "--day", "10",
        "--debug-profile", "stem",
        "--pose-mode", "canonical",
        "--physics-preset", "flexible",
        "--input", str(PROJECT_ROOT / "data/plant_states/plant_state_day_10.json"),
        "--output", str(destination),
        "--generate-only",
    ]) == 0
    assert Usd.Stage.Open(str(destination)) is not None
    assert destination.with_suffix(".manifest.json").exists()

