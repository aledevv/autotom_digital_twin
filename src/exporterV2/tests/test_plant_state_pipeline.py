"""Offline coverage for the canonical Phase-J V2 pipeline."""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
import sys

import pytest
from pxr import Usd, UsdGeom

from exporterV2.cli import main
from exporterV2.isaac_app import _arguments, _open_stage_and_wait
from exporterV2.plant_state_adapter import (
    Pose,
    V2PlantStateError,
    authored_capsule_pose,
    build_v2_authoring_plan,
    capsule_capsule_overlap,
    sphere_capsule_overlap,
    sphere_sphere_overlap,
    validate_joint_budget,
)
from exporterV2.plant_state_usd import (
    audit_v2_stage,
    export_plant_state_v2,
    manifest_path_for,
    save_v2_manifest,
)
from groimp_bridge.inspector import override_model_duration
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def _state(day: int):
    return load_plant_state(
        PROJECT_ROOT / "data" / "plant_states" / f"plant_state_day_{day}.json"
    )


@pytest.mark.parametrize(
    ("day", "axes", "spheres", "physical", "d6"),
    ((1, 20, 0, 9, 8), (25, 130, 1, 51, 50), (80, 347, 72, 216, 215), (160, 347, 72, 216, 215)),
)
def test_real_days_preserve_all_visuals_and_physical_budget(
    day, axes, spheres, physical, d6
):
    state = _state(day)
    plan = build_v2_authoring_plan(state)
    assert len(plan.visual_axes) == axes == len(state.axes)
    assert len(plan.visual_spheres) == spheres == len(state.spheres)
    assert len(plan.physical_links) == physical
    assert plan.predicted_d6_joints == d6 <= 220
    assert {axis.id for axis in plan.visual_axes} == {axis.id for axis in state.axes}
    assert {sphere.id for sphere in plan.visual_spheres} == {
        sphere.id for sphere in state.spheres
    }
    assert not plan.diagnostics["duplicate_geometry_of"]
    assert all("correction limits exhausted" in record.reason for record in plan.unresolved_collision_filters if record.kind.startswith("sphere"))


def test_day_160_is_structurally_mature_but_not_a_copy_of_day_80():
    day_80, day_160 = _state(80), _state(160)
    assert Counter(organ.organ_type for organ in day_80.organs) == Counter(
        organ.organ_type for organ in day_160.organs
    )
    assert day_160.metadata.simulation_time == 160
    assert min(sphere.radius for sphere in day_160.spheres) > min(
        sphere.radius for sphere in day_80.spheres
    )


def test_collision_geometry_uses_finite_authored_shapes():
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    left = Pose((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), identity)
    right = Pose((0.2, 0.0, 0.0), (0.2, 0.0, 1.0), identity)
    assert capsule_capsule_overlap(left, 0.11, right, 0.11) == pytest.approx(0.02)
    assert sphere_capsule_overlap((0.0, 0.0, 0.5), 0.1, right, 0.11) == pytest.approx(0.01)
    assert sphere_sphere_overlap((0, 0, 0), 0.1, (0.15, 0, 0), 0.1) == pytest.approx(0.05)
    shortened = authored_capsule_pose(left, 0.05)
    assert shortened.start[2] > 0.0
    assert shortened.end[2] < 1.0


def test_joint_budget_thresholds():
    assert validate_joint_budget(220) == "unchanged"
    with pytest.raises(V2PlantStateError, match="221-230"):
        validate_joint_budget(221)
    assert validate_joint_budget(230, allow_near_budget=True) == "unchanged"
    with pytest.raises(V2PlantStateError, match="exceed"):
        validate_joint_budget(231)
    assert validate_joint_budget(231, optimize=True) == "optimize"


def test_exact_duplicate_is_the_only_visual_collapsing_rule():
    state = _state(1)
    original = next(axis for axis in state.axes if axis.role == "petiolule_left")
    duplicate = replace(original, id=f"{original.id}:exact_duplicate")
    organs = list(state.organs)
    organ_index = next(
        index for index, organ in enumerate(organs) if organ.node_id == original.owner_node_id
    )
    organs[organ_index] = replace(
        organs[organ_index],
        primitive_ids=(*organs[organ_index].primitive_ids, duplicate.id),
    )
    duplicated = replace(state, axes=(*state.axes, duplicate), organs=tuple(organs))
    plan = build_v2_authoring_plan(duplicated)
    item = next(axis for axis in plan.visual_axes if axis.id == duplicate.id)
    assert item.render_geometry is False
    assert item.duplicate_of == original.id
    assert len(plan.visual_axes) == len(duplicated.axes)


def test_usd_audit_origin_topology_and_deterministic_manifest(tmp_path):
    state = _state(1)
    plan = build_v2_authoring_plan(state)
    destination = export_plant_state_v2(plan, tmp_path / "plant.usda")
    manifest = audit_v2_stage(plan, destination)
    assert manifest.errors == ()
    assert manifest.metadata["status"] == "passed"
    assert manifest.physics["d6_joints"] == 8
    assert manifest.physics["rigid_bodies_authored"] == 9
    first = save_v2_manifest(manifest, tmp_path / "one.json").read_bytes()
    second = save_v2_manifest(manifest, tmp_path / "two.json").read_bytes()
    assert first == second
    stage = Usd.Stage.Open(str(destination))
    root = next(
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
        and prim.GetChild("RootFixedJoint")
    )
    translation = UsdGeom.XformCache().GetLocalToWorldTransform(root).ExtractTranslation()
    assert tuple(translation) == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


def test_cli_uses_only_explicit_canonical_metadata(tmp_path, capsys):
    source = PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_1.json"
    output = tmp_path / "plant.usda"
    assert main(["--day", "1", "--input", str(source), "--output", str(output)]) == 0
    assert manifest_path_for(output).is_file()
    assert main(["--day", "2", "--input", str(source), "--output", str(output)]) == 1
    assert "requested day 2" in capsys.readouterr().err
    assert "parse_csv_to_branches" not in (PROJECT_ROOT / "src" / "exporterV2" / "main.py").read_text()


def test_isolated_duration_override_is_exact_and_safe():
    source = "class P { static int DURATION_DAYS = 80; }"
    assert "DURATION_DAYS = 160" in override_model_duration(source, 160)
    with pytest.raises(ValueError, match="locate"):
        override_model_duration("class P {}", 160)


def test_isaac_arguments_do_not_leak_to_kit(monkeypatch, tmp_path):
    stage = tmp_path / "plant.usda"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "isaac_app.py",
            "--usd",
            str(stage),
            "--headless",
            "--duration",
            "30",
            "--physics-preset",
            "locked",
            "--/kit/test=true",
        ],
    )
    args = _arguments()
    assert args.usd == stage
    assert args.duration == 30
    assert sys.argv == ["isaac_app.py", "--/kit/test=true"]


def test_isaac_45_open_stage_none_return_is_supported(tmp_path):
    destination = (tmp_path / "plant.usda").resolve()

    class Layer:
        realPath = str(destination)

    class Stage:
        def GetRootLayer(self):
            return Layer()

    class Context:
        def open_stage(self, path):
            return None

        def get_stage(self):
            return Stage()

    class App:
        def update(self):
            pass

    assert _open_stage_and_wait(Context(), App(), destination, lambda: False) == destination
