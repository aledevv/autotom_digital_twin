"""Offline coverage for the canonical Phase-J V2 pipeline."""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
import sys

import pytest
from pxr import Sdf, Usd, UsdGeom

from exporterV2.cli import main
from exporterV2.isaac_app import (
    _arguments,
    _authored_physics_hz,
    _open_stage_and_wait,
    _timing_metrics,
    _world_endpoints,
)
from exporterV2.plant_state_adapter import (
    DEBUG_PROFILES,
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
    (
        (1, 20, 0, 9, 6),
        (10, 39, 0, 16, 11),
        (25, 130, 1, 51, 36),
        (50, 274, 27, 133, 107),
        (80, 347, 72, 216, 190),
        (160, 347, 72, 216, 190),
    ),
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
    assert all(
        link.joint_type == ("fixed" if link.role == "internode" else "d6")
        for link in plan.physical_links
    )
    assert {axis.id for axis in plan.visual_axes} == {axis.id for axis in state.axes}
    assert {sphere.id for sphere in plan.visual_spheres} == {
        sphere.id for sphere in state.spheres
    }
    if day == 50:
        assert len(plan.diagnostics["duplicate_geometry_of"]) == 5
        assert len(plan.diagnostics["degenerate_axis_ids"]) == 10
    else:
        assert not plan.diagnostics["duplicate_geometry_of"]
        assert not plan.diagnostics["degenerate_axis_ids"]
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


def test_debug_profiles_are_incremental_and_dependency_closed():
    state = _state(25)
    plans = {
        profile: build_v2_authoring_plan(
            state,
            debug_profile=profile,
            colliders_enabled=False if profile != "full" else True,
        )
        for profile in DEBUG_PROFILES
    }
    expected_physical = {
        "stem": 8,
        "leaf-supports": 28,
        "leaves": 28,
        "laterals": 49,
        "truss-supports": 51,
        "fruit-visual": 51,
        "full": 51,
    }
    for profile, count in expected_physical.items():
        plan = plans[profile]
        assert len(plan.physical_links) == count
        link_ids = {link.id for link in plan.physical_links}
        assert all(
            link.parent_id is None or link.parent_id in link_ids
            for link in plan.physical_links
        )
        assert len([link for link in plan.physical_links if link.parent_id is None]) == 1
    assert len(plans["stem"].visual_axes) == 8
    assert len(plans["leaf-supports"].visual_axes) == 28
    assert len(plans["laterals"].visual_axes) == 128
    assert len(plans["truss-supports"].visual_axes) == 130
    assert not plans["truss-supports"].visual_spheres
    assert len(plans["fruit-visual"].visual_spheres) == 1
    assert plans["fruit-visual"].terminal_bodies_physical is False
    assert plans["full"].terminal_bodies_physical is True
    assert plans["full"].diagnostics["omitted_axis_ids"] == []


def test_diagnostic_switches_cannot_change_full_production_profile():
    with pytest.raises(V2PlantStateError, match="non-full"):
        build_v2_authoring_plan(
            _state(1), debug_profile="full", colliders_enabled=False
        )


def test_day_50_immature_zero_leaf_stays_metadata_only():
    state = _state(50)
    plan = build_v2_authoring_plan(state, debug_profile="full")
    immature = next(
        organ
        for organ in state.organs
        if organ.node_id == "node:421489"
    )
    assert immature.organ_type == "Leaf"
    assert immature.properties.blade_area_total == 0.0
    assert immature.id not in {
        organ_id for link in plan.physical_links for organ_id in link.canonical_organ_ids
    }
    its_axes = [
        axis for axis in plan.visual_axes if axis.owner_node_id == immature.node_id
    ]
    assert len(its_axes) == 10
    assert all(not axis.render_geometry for axis in its_axes)
    assert {axis.id for axis in its_axes} <= set(
        plan.diagnostics["degenerate_axis_ids"]
    )


def test_fruit_visual_profile_authors_no_terminal_physics(tmp_path):
    plan = build_v2_authoring_plan(
        _state(25), debug_profile="fruit-visual", colliders_enabled=False
    )
    destination = export_plant_state_v2(plan, tmp_path / "fruit_visual.usda")
    manifest = audit_v2_stage(plan, destination)
    assert manifest.errors == ()
    assert manifest.metadata["debug_profile"] == "fruit-visual"
    assert manifest.physics["terminal_bodies"] == 0
    assert manifest.physics["static_visual_spheres"] == 1
    assert manifest.physics["colliders_authored"] == 0


def test_usd_audit_origin_topology_and_deterministic_manifest(tmp_path):
    state = _state(1)
    plan = build_v2_authoring_plan(state)
    destination = export_plant_state_v2(plan, tmp_path / "plant.usda")
    manifest = audit_v2_stage(plan, destination)
    assert manifest.errors == ()
    assert manifest.metadata["status"] == "passed"
    assert manifest.physics["d6_joints"] == 6
    assert manifest.physics["fixed_links"] == 3
    assert manifest.physics["fixed_joints_enabled"] == 3
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
    organ_names = {
        prim.GetName()
        for prim in stage.GetPrimAtPath("/World/Plant_1/Organs").GetChildren()
    }
    assert "Internode_organ_421092" in organ_names
    assert all(not name.startswith("organ_") for name in organ_names)
    physical_names = {
        prim.GetName()
        for prim in stage.GetPrimAtPath("/World/Plant_1/Physics").GetChildren()
    }
    assert any(name.startswith("Internode_node_") for name in physical_names)


def test_world_endpoint_uses_scalar_first_orientation():
    positions = [(1.0, 2.0, 3.0)]
    # +90 degrees around world Y maps local Z to world X.
    root_half = 2.0**-0.5
    orientations = [(root_half, 0.0, root_half, 0.0)]
    metadata = {"/Body": {"length": 2.0}}
    endpoint = _world_endpoints(
        positions, orientations, ["/Body"], metadata
    )["/Body"]
    assert tuple(endpoint) == pytest.approx((3.0, 2.0, 3.0), abs=1e-12)


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
    assert args.physics_hz == 480
    assert args.interactive_physics_hz == 60
    assert sys.argv == ["isaac_app.py", "--/kit/test=true"]


def test_isaac_interactive_rate_is_explicit_and_does_not_leak_to_kit(
    monkeypatch, tmp_path
):
    stage = tmp_path / "plant.usda"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "isaac_app.py",
            "--usd",
            str(stage),
            "--physics-preset",
            "flexible",
            "--physics-hz",
            "960",
            "--interactive-physics-hz",
            "120",
            "--/kit/test=true",
        ],
    )
    args = _arguments()
    assert args.physics_hz == 960
    assert args.interactive_physics_hz == 120
    assert sys.argv == ["isaac_app.py", "--/kit/test=true"]


def test_timing_metrics_count_render_updates_and_physics_substeps():
    metrics = _timing_metrics(
        authored_physics_hz=480,
        runtime_physics_hz=120,
        render_hz=60,
        render_update_count=30,
        physics_step_count=60,
        simulated_seconds=0.5,
        wall_seconds=1.0,
    )
    assert metrics == {
        "physics_hz": 120,
        "authored_physics_hz": 480,
        "runtime_physics_hz": 120,
        "render_hz": 60,
        "physics_substeps_per_render": 2,
        "render_update_count": 30,
        "physics_step_count": 60,
        "simulated_seconds": 0.5,
        "wall_seconds": 1.0,
        "render_updates_per_second": 30.0,
        "physics_steps_per_second": 60.0,
        "simulation_realtime_ratio": 0.5,
    }


def test_authored_physics_rate_is_read_before_runtime_override():
    stage = Usd.Stage.CreateInMemory()
    scene = stage.DefinePrim("/World/PhysicsScene", "PhysicsScene")
    scene.CreateAttribute(
        "physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.Int
    ).Set(480)
    assert _authored_physics_hz(stage) == 480


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
