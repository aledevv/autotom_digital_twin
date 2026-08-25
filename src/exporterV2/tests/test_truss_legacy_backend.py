from __future__ import annotations

import json
from pathlib import Path

import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from exporterV2.cli import main as exporter_main
from exporterV2.core.tree_config import TrussGeometryConfig, TrussPhysicsConfig
from exporterV2.plant_state_branches import (
    PlantStateBranchesError,
    apply_checkpoint_physics_policy,
    build_truss_branches,
)
from exporterV2.plant_state_legacy_backend import (
    IncrementalCheckpointError,
    _audit_collider_overlaps,
    _auto_filter_initial_overlaps,
    export_incremental_checkpoint,
    manifest_path_for,
)
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def test_active_day160_truss_and_historical_detachment_configuration():
    assert TrussPhysicsConfig.RACHIS_YOUNG_MODULUS == 20.0e9
    assert TrussPhysicsConfig.PEDICEL_YOUNG_MODULUS == 4.0e9
    assert TrussPhysicsConfig.RACHIS_DAMPING_RATIO == 4.0
    assert TrussPhysicsConfig.PEDICEL_DAMPING_RATIO == 4.0
    assert TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE == 0.2
    assert TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N == 6.0
    assert TrussPhysicsConfig.TOMATO_DETACHMENT_EXCLUDE_FROM_ARTICULATION is True


def test_day160_balanced_fixed_laterals_are_applied_in_memory_only():
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_160.json"
    )
    source = build_truss_branches(state)
    calibrated = apply_checkpoint_physics_policy(
        source,
        lateral_joint_policy="fixed",
        truss_calibration_preset="balanced",
    )

    assert all(
        branch["joint_type"] == "d6"
        for branch in source.branches
        if branch.get("kind") == "lateral_branch"
    )
    assert sum(
        branch["n_links"]
        for branch in calibrated.branches
        if branch.get("kind") == "lateral_branch"
        and branch["joint_type"] == "fixed"
    ) == 16
    rachides = [
        branch
        for branch in calibrated.branches
        if branch.get("truss_component") == "rachis"
    ]
    pedicels = [
        branch
        for branch in calibrated.branches
        if branch.get("truss_component") == "pedicel"
    ]
    assert rachides and pedicels
    assert {branch["young_modulus"] for branch in rachides} == {20.0e9}
    assert {branch["young_modulus"] for branch in pedicels} == {4.0e9}
    assert {branch["damping_ratio"] for branch in [*rachides, *pedicels]} == {2.0}
    assert {branch["drive_stiffness_scale"] for branch in pedicels} == {0.2}
    assert {branch["density"] for branch in [*rachides, *pedicels]} == {
        TrussPhysicsConfig.PLANT_DENSITY
    }
    assert TrussPhysicsConfig.PLANT_DENSITY == 2000.0


def test_day160_truss_supports_balanced_fixed_joint_audit(tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_160.json"
    )
    plan, _usd_path, manifest_path = export_incremental_checkpoint(
        state,
        tmp_path / "day160_balanced_fixed.usda",
        debug_profile="truss-supports",
        physics_preset="flexible",
        lateral_joint_policy="fixed",
        truss_calibration_preset="balanced",
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert plan.physical_link_count == 216
    assert plan.predicted_d6_joints == 190
    assert manifest["authored"]["rigid_bodies"] == 216
    assert manifest["authored"]["d6_joints"] == 190
    assert manifest["authored"]["fixed_joints"] == 26
    assert manifest["authored"]["fruit_spheres"] == 0
    assert manifest["metadata"]["lateral_joint_policy"] == "fixed"
    assert manifest["metadata"]["truss_calibration_preset"] == "balanced"
    truss = manifest["physics"]["truss_profile"]
    assert truss["density_kg_m3"] == 2000.0
    assert truss["rachis_young_modulus_pa"] == 20.0e9
    assert truss["pedicel_young_modulus_pa"] == 4.0e9
    assert truss["rachis_damping_ratio"] == 2.0
    assert truss["pedicel_damping_ratio"] == 2.0
    assert truss["pedicel_drive_stiffness_scale"] == 0.2
    assert manifest["errors"] == []


@pytest.mark.parametrize(
    ("day", "support_links", "d6", "fixed", "fruits"),
    (
        (25, 51, 43, 9, 1),
        (50, 133, 123, 37, 27),
        (80, 216, 206, 82, 72),
        (160, 216, 206, 82, 72),
    ),
)
def test_canonical_full_adapter_counts(day, support_links, d6, fixed, fruits):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states" / f"plant_state_day_{day}.json"
    )
    result = build_truss_branches(
        state, include_fruits=True, physical_fruits=True
    )
    assert sum(branch["n_links"] for branch in result.branches) == support_links
    assert sum(
        branch["n_links"]
        for branch in result.branches
        if branch["joint_type"] != "fixed"
    ) == d6
    assert support_links - d6 + len(result.terminal_bodies) == fixed
    assert len(result.terminal_bodies) == fruits
    assert all(branch["system"] == "vegetative" for branch in result.branches)
    assert {
        branch["visual_style"]
        for branch in result.branches
        if branch.get("physics_profile") == "truss"
    } == {"historical_truss_rachis", "historical_pedicel"}


def test_day50_full_stage_uses_continuous_historical_truss_and_v2_poses(tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_50.json"
    )
    plan, usd_path, manifest_path = export_incremental_checkpoint(
        state,
        tmp_path / "full.usda",
        debug_profile="full",
        physics_preset="flexible",
        allow_experimental_fruit_physics=True,
    )
    assert plan.physical_link_count == 133
    assert plan.predicted_d6_joints == 123
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["schema_version"] == "exporter_v2_full_checkpoint/1.0"
    assert manifest["authored"]["rigid_bodies"] == 160
    assert manifest["authored"]["support_rigid_bodies"] == 133
    assert manifest["authored"]["d6_joints"] == 123
    assert manifest["authored"]["fixed_joints"] == 37
    assert manifest["authored"]["capsule_colliders"] == 266
    assert manifest["authored"]["fruit_spheres"] == 27
    assert len(manifest["authored"]["historical_truss_visuals"]) == 52
    assert len(manifest["authored"]["terminal_body_poses"]) == 27
    assert manifest["physics"]["truss_profile"]["tomato_break_force_n"] \
        == pytest.approx(6.0)
    assert manifest["metadata"]["experimental_fruit_physics"] is True
    assert manifest["metadata"]["fruit_physics_support_status"] \
        == "unsupported_experimental"
    assert manifest["collisions"]["active_overlaps"] == []
    assert manifest["collisions"]["applied_initial_filters"]
    assert not any(
        record["body_a"] == "/World/Stem"
        or record["body_b"] == "/World/Stem"
        for record in manifest["collisions"]["applied_initial_filters"]
    )

    stage = Usd.Stage.Open(str(usd_path))
    assert not any(
        prim.GetName() == "HistoricalTrussRachisVisual"
        for prim in stage.Traverse()
    )
    assert manifest["authored"]["visual_cylinders"] == 0
    assert {
        record["visual_topology"]
        for record in manifest["authored"]["historical_truss_visuals"]
    } == {"continuous_segmented_organic_axis", "gravity_elbow"}
    assert sum(
        prim.GetName() == "GravityElbowPedicelVisual"
        for prim in stage.Traverse()
    ) == 27
    terminal_joints = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == "TerminalBodyFixedJoint"
    ]
    assert len(terminal_joints) == 27
    assert all(
        UsdPhysics.Joint(prim).GetBreakForceAttr().Get() == pytest.approx(6.0)
        for prim in terminal_joints
    )
    assert all(
        UsdPhysics.Joint(prim).GetExcludeFromArticulationAttr().Get() is True
        for prim in terminal_joints
    )
    # The canonical sphere and pedicel orientations may differ, but their
    # FixedJoint frames must coincide in world space at the authored rest pose.
    for joint_prim in terminal_joints:
        joint = UsdPhysics.Joint(joint_prim)
        parent = stage.GetPrimAtPath(joint.GetBody0Rel().GetTargets()[0])
        child = stage.GetPrimAtPath(joint.GetBody1Rel().GetTargets()[0])
        parent_q = parent.GetAttribute("xformOp:orient").Get()
        child_q = child.GetAttribute("xformOp:orient").Get()
        frame0 = parent_q * joint.GetLocalRot0Attr().Get()
        frame1 = child_q * joint.GetLocalRot1Attr().Get()
        parent_v = (frame0.GetReal(), *frame0.GetImaginary())
        child_v = (frame1.GetReal(), *frame1.GetImaginary())
        assert abs(sum(float(a) * float(b) for a, b in zip(parent_v, child_v))) \
            == pytest.approx(1.0, abs=1e-6)
    assert all(
        pose["source_frame"] is not None
        and pose["authored_frame"] is not None
        and pose["authored_orientation"] is not None
        for pose in manifest["authored"]["terminal_body_poses"]
    )
    assert any(
        pose["source_frame"] != pose["authored_frame"]
        for pose in manifest["authored"]["terminal_body_poses"]
    )
    stem = stage.GetPrimAtPath("/World/Stem")
    assert stem.GetAttribute("autotom:experimentalFruitPhysics").Get() is True
    assert stem.GetAttribute("autotom:fruitPhysicsSupportStatus").Get() \
        == "unsupported_experimental"


def test_full_fruit_physics_requires_explicit_unsupported_opt_in(tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_25.json"
    )
    with pytest.raises(
        IncrementalCheckpointError,
        match="unsupported.*allow-experimental-fruit-physics",
    ):
        export_incremental_checkpoint(
            state,
            tmp_path / "guarded_full.usda",
            debug_profile="full",
            physics_preset="flexible",
        )

    output = tmp_path / "guarded_cli.usda"
    assert exporter_main(
        [
            "--day",
            "25",
            "--debug-profile",
            "full",
            "--output",
            str(output),
        ]
    ) == 1
    assert not output.exists()


def test_v2_pedicels_use_historical_lateral_angles_and_terminal_alignment():
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_50.json"
    )
    result = build_truss_branches(state, appendage_pose_mode="v2-aesthetic")
    pedicels = [
        branch for branch in result.branches if branch.get("kind") == "pedicel"
    ]
    assert pedicels
    lateral = [
        branch["link_specs"][0]
        for branch in pedicels
        if branch["link_specs"][0].get("v2_tilt_deg") == 56.0
    ]
    terminal = [
        branch["link_specs"][0]
        for branch in pedicels
        if branch["link_specs"][0].get("v2_tilt_deg") == 0.0
    ]
    assert {spec["v2_rotation_deg"] for spec in lateral} == {90.0, 270.0}
    assert terminal
    assert all(spec["source_rest_frame"] != spec["rest_frame"] for spec in lateral)
    assert all(
        spec["source_length"]
        * TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE
        == pytest.approx(spec["length"])
        for branch in pedicels
        for spec in branch["link_specs"]
    )


def test_pedicel_length_scale_preserves_source_and_authors_complete_geometry(
    tmp_path,
):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_25.json"
    )
    _plan, usd_path, manifest_path = export_incremental_checkpoint(
        state,
        tmp_path / "scaled_pedicels.usda",
        debug_profile="full",
        physics_preset="flexible",
        allow_experimental_fruit_physics=True,
    )
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    pedicel_poses = [
        pose
        for pose in manifest["authored"]["poses"]
        if ":pedicel:" in pose["axis_id"]
    ]
    assert pedicel_poses
    assert all(
        pose["authored_length"]
        == pytest.approx(
            pose["source_length"]
            * TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE
        )
        for pose in pedicel_poses
    )

    stage = Usd.Stage.Open(str(usd_path))
    for pose in pedicel_poses:
        prim = stage.GetPrimAtPath(pose["body_path"])
        source = prim.GetAttribute("autotom:canonicalSourceLength").Get()
        authored = prim.GetAttribute("autotom:sourceLength").Get()
        scale = prim.GetAttribute("autotom:authoredLengthScale").Get()
        assert scale == pytest.approx(
            TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE
        )
        assert authored == pytest.approx(source * scale)


def test_pedicel_length_scale_must_be_positive(monkeypatch):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_25.json"
    )
    monkeypatch.setattr(
        TrussGeometryConfig, "PLANT_STATE_PEDICEL_LENGTH_SCALE", 0.0
    )
    with pytest.raises(PlantStateBranchesError, match="must be finite and positive"):
        build_truss_branches(state)


def test_canonical_appendage_mode_preserves_raw_pedicel_frames():
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_25.json"
    )
    result = build_truss_branches(state, appendage_pose_mode="canonical")
    specs = [
        branch["link_specs"][0]
        for branch in result.branches
        if branch.get("kind") == "pedicel"
    ]
    assert specs
    assert all(spec["source_rest_frame"] == spec["rest_frame"] for spec in specs)


def test_fruit_visual_has_no_terminal_physics(tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_25.json"
    )
    _plan, usd_path, manifest_path = export_incremental_checkpoint(
        state,
        tmp_path / "fruit_visual.usda",
        debug_profile="fruit-visual",
        physics_preset="flexible",
    )
    stage = Usd.Stage.Open(str(usd_path))
    terminal = stage.GetPrimAtPath(
        "/World/Stem/TerminalVisuals/Truss_r6_o0_g421361_tomato_01"
    )
    assert terminal.IsValid()
    assert not terminal.HasAPI(UsdPhysics.RigidBodyAPI)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["expected"]["terminal_visual_bodies"] == 1
    assert manifest["expected"]["terminal_physical_bodies"] == 0


def test_physical_petiolules_are_explicit_and_budget_guarded(tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_50.json"
    )
    leaves, _usd, manifest_path = export_incremental_checkpoint(
        state,
        tmp_path / "physical_leaves.usda",
        debug_profile="leaves",
        physics_preset="flexible",
        physical_petiolules=True,
    )
    assert leaves.physical_link_count == 212
    assert leaves.predicted_d6_joints == 202
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["metadata"]["physical_petiolules"] is True
    assert manifest["authored"]["leaf_blades"] == 131

    with pytest.raises(IncrementalCheckpointError, match="exceed.*230"):
        export_incremental_checkpoint(
            state,
            tmp_path / "rejected_full.usda",
            debug_profile="full",
            physics_preset="flexible",
            physical_petiolules=True,
            allow_experimental_fruit_physics=True,
        )


def test_strict_initial_overlap_policy_reports_day50_conflicts(tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data/plant_states/plant_state_day_50.json"
    )
    with pytest.raises(IncrementalCheckpointError, match="active collider overlaps"):
        export_incremental_checkpoint(
            state,
            tmp_path / "strict.usda",
            debug_profile="full",
            physics_preset="flexible",
            initial_overlap_policy="error",
            allow_experimental_fruit_physics=True,
        )


def test_cli_propagates_new_flags(tmp_path):
    output = tmp_path / "cli.usda"
    assert exporter_main(
        [
            "--day",
            "25",
            "--debug-profile",
            "truss-supports",
            "--physics-preset",
            "flexible",
            "--initial-overlap-policy",
            "filter",
            "--appendage-pose-mode",
            "canonical",
            "--truss-armature-multiplier",
            "1",
            "--terminal-solver-preset",
            "stabilized",
            "--output",
            str(output),
            "--generate-only",
        ]
    ) == 0
    assert output.exists()
    manifest = json.loads(manifest_path_for(output).read_text(encoding="utf-8"))
    assert manifest["metadata"]["appendage_pose_mode"] == "canonical"
    assert manifest["metadata"]["truss_armature_multiplier"] == 1.0
    assert manifest["physics"]["truss_armatures"]
    assert manifest["physics"]["terminal_solver"] == {
        "preset": "stabilized",
        "position_iterations": 64,
        "velocity_iterations": 4,
    }
    stage = Usd.Stage.Open(str(output))
    for record in manifest["physics"]["truss_armatures"]:
        joint = stage.GetPrimAtPath(record["joint_path"])
        # The plain OpenUSD wheel preserves unknown NVIDIA API tokens in the
        # authored list-op but cannot resolve them through GetAppliedSchemas.
        assert "PhysxJointAPI" in joint.GetMetadata("apiSchemas").explicitItems
        assert joint.GetAttribute("physxJoint:armature").Get() == pytest.approx(
            record["armature_kg_m2"]
        )
        assert not joint.GetAttribute("physxJoint:armature").IsCustom()


def test_day160_cli_default_is_validated_fruit_free_profile(tmp_path):
    output = tmp_path / "day160_default.usda"
    assert exporter_main(
        ["--day", "160", "--output", str(output), "--generate-only"]
    ) == 0
    manifest = json.loads(manifest_path_for(output).read_text(encoding="utf-8"))
    assert manifest["metadata"]["debug_profile"] == "truss-supports"
    assert manifest["metadata"]["physics_preset"] == "flexible"
    assert manifest["metadata"]["lateral_joint_policy"] == "dynamic"
    assert manifest["metadata"]["experimental_fruit_physics"] is False
    assert manifest["metadata"]["fruit_physics_support_status"] == "not_authored"
    assert manifest["authored"]["rigid_bodies"] == 216
    assert manifest["authored"]["d6_joints"] == 206
    assert manifest["authored"]["fixed_joints"] == 10
    assert manifest["authored"]["capsule_colliders"] == 432
    assert manifest["authored"]["fruit_spheres"] == 0
    assert manifest["errors"] == []


def _collision_shape(stage, body_name: str, shape_name: str, x: float):
    body = UsdGeom.Xform.Define(stage, f"/World/{body_name}")
    body.AddTranslateOp().Set(Gf.Vec3d(x, 0.0, 0.0))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    path = f"{body.GetPath()}/Collider"
    if shape_name == "sphere":
        shape = UsdGeom.Sphere.Define(stage, path)
        shape.CreateRadiusAttr().Set(0.1)
    elif shape_name == "capsule":
        shape = UsdGeom.Capsule.Define(stage, path)
        shape.CreateAxisAttr().Set("Z")
        shape.CreateRadiusAttr().Set(0.1)
        shape.CreateHeightAttr().Set(0.2)
    else:
        shape = UsdGeom.Cylinder.Define(stage, path)
        shape.CreateAxisAttr().Set("Z")
        shape.CreateRadiusAttr().Set(0.1)
        shape.CreateHeightAttr().Set(0.2)
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())


@pytest.mark.parametrize(
    ("left_shape", "right_shape"),
    (
        ("capsule", "capsule"),
        ("cylinder", "capsule"),
        ("cylinder", "cylinder"),
        ("sphere", "capsule"),
        ("capsule", "sphere"),
        ("sphere", "cylinder"),
        ("cylinder", "sphere"),
        ("sphere", "sphere"),
    ),
)
def test_shape_aware_initial_overlap_filtering(left_shape, right_shape):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World")
    _collision_shape(stage, "Left", left_shape, 0.0)
    _collision_shape(stage, "Right", right_shape, 0.15)
    _filtered, active = _audit_collider_overlaps(stage)
    assert active
    assert {active[0]["shape_a"], active[0]["shape_b"]} == {
        left_shape,
        right_shape,
    }
    applied = _auto_filter_initial_overlaps(stage)
    assert len(applied) == 1
    filtered, active = _audit_collider_overlaps(stage)
    assert filtered
    assert active == []
