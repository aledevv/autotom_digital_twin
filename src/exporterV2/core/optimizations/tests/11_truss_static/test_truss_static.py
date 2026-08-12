from __future__ import annotations

from copy import deepcopy
from pathlib import Path
import sys

import pytest
import yaml
from pxr import Usd, UsdPhysics


SRC_DIR = Path(__file__).resolve().parents[5]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.adapters.groimp_csv.truss_builder import truss_to_complete_config
from exporterV2.core.optimizations.optimizer import BudgetOptimizer
from exporterV2.core.optimizations.techniques.base import count_d6_joints
from exporterV2.core.optimizations.techniques.truss_static import TrussStaticTechnique
from exporterV2.core.tree_config import TrussPhysicsConfig
from exporterV2.core.usd.stage import build_stage


def make_truss() -> tuple[list[dict], list[dict]]:
    trunk = {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 1,
        "radius": 0.01,
        "height": 0.20,
        "tilt": 0.0,
        "rot": 0.0,
    }
    truss = {
        "rachis_length": 0.24,
        "rachis_radius": 0.0025,
        "n_fruits": 7,
        "pedicel_length": 0.035,
        "pedicel_radius": 0.0015,
        "pedicel_angle": 90.0,
        "parent_rank": 0,
        "tilt_deg": 58.0,
        "azimuth_deg": 90.0,
        "tomato_radii": [0.018] * 7,
    }
    truss_branches, tomatoes = truss_to_complete_config(truss, "trunk", rank=0)
    terminal_bodies = [
        {
            "id": tomato["id"],
            "parent_branch_id": tomato["pedicel_id"],
            "shape": "sphere",
            "radius": tomato["radius"],
            "mass": tomato["mass"],
        }
        for tomato in tomatoes
    ]
    return [trunk, *truss_branches], terminal_bodies


def test_default_pedicels_are_soft_limited_d6():
    branches, _ = make_truss()
    pedicels = [branch for branch in branches if "_pedicel_" in branch["id"]]

    assert len(pedicels) == 7
    assert all(branch["joint_type"] == "d6" for branch in pedicels)
    assert all(
        branch["bend_limit_deg"] == TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG
        for branch in pedicels
    )
    assert all(
        branch["drive_stiffness_scale"]
        == TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE
        for branch in pedicels
    )


def test_truss_optimization_is_progressive_and_preserves_pedicels():
    branches, terminal_bodies = make_truss()
    technique = TrussStaticTechnique()
    original_length = next(
        branch["height"] * branch["n_links"]
        for branch in branches
        if branch["id"].endswith("_rachis")
    )

    fixed, fixed_report = technique.apply(branches)
    assert fixed_report.details["stage"] == "pedicels_fixed"
    assert fixed_report.joints_saved == 7
    assert all(
        branch.get("joint_type") == "fixed"
        for branch in fixed
        if "_pedicel_" in branch["id"]
    )

    static, static_report = technique.apply(fixed)
    assert static_report.details["stage"] == "static_prebent"
    assert count_d6_joints(branches) == 12
    assert count_d6_joints(fixed) == 5
    assert count_d6_joints(static) == 2
    assert not technique.can_apply(static)
    assert technique.validate(fixed, static).valid

    curve = [branch for branch in static if "_static_curve_" in branch["id"]]
    assert len(curve) == 5
    assert curve[0]["joint_type"] == "d6"
    assert all(branch["joint_type"] == "fixed" for branch in curve[1:])
    assert sum(branch["height"] for branch in curve) == pytest.approx(original_length)
    assert curve[0]["bend_limit_deg"] == 18.0
    assert curve[0]["drive_stiffness_scale"] == 0.40

    static_ids = {branch["id"] for branch in static}
    for body in terminal_bodies:
        assert body["parent_branch_id"] in static_ids
    for pedicel in (branch for branch in static if "_pedicel_" in branch["id"]):
        assert "_static_curve_" in pedicel["parent"]


def test_optimizer_dispatches_both_truss_steps(tmp_path):
    branches, _ = make_truss()
    config_path = Path(__file__).resolve().parents[2] / "budget_config.yaml"
    config = yaml.safe_load(config_path.read_text())
    config["budget"]["max_joints"] = 2
    for technique in config["techniques"]:
        technique["enabled"] = technique["id"] == "truss_static"

    custom_config = tmp_path / "truss_budget.yaml"
    custom_config.write_text(yaml.safe_dump(config))
    optimizer = BudgetOptimizer(str(custom_config))
    optimized, report = optimizer.optimize(branches)

    assert report.success
    assert report.final_joints == 2
    assert [item.details["stage"] for item in report.technique_reports] == [
        "pedicels_fixed",
        "static_prebent",
    ]
    assert any("_static_curve_" in branch["id"] for branch in optimized)


def test_one_link_rachis_is_curved_without_changing_joint_count():
    branches, _ = make_truss()
    rachis = next(branch for branch in branches if branch["id"].endswith("_rachis"))
    rachis["height"] *= rachis["n_links"]
    rachis["n_links"] = 1
    for pedicel in (branch for branch in branches if "_pedicel_" in branch["id"]):
        pedicel["attach_link"] = 1
        pedicel["attach_frac"] = 1.0

    technique = TrussStaticTechnique()
    fixed, _ = technique.apply(branches)
    before_static = count_d6_joints(fixed)
    static, report = technique.apply(fixed)

    assert report.details["stage"] == "static_prebent"
    assert report.joints_saved == 0
    assert count_d6_joints(static) == before_static
    assert len([branch for branch in static if "_static_curve_" in branch["id"]]) == 5
    assert not technique.can_apply(static)


def test_usd_uses_official_pedicel_and_static_root_overrides(tmp_path):
    branches, terminal_bodies = make_truss()
    dynamic_path = tmp_path / "dynamic.usda"
    dynamic_stage, stem_path = build_stage(
        str(dynamic_path),
        branches=branches,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    dynamic_stage.GetRootLayer().Save()

    pedicel_joints = [
        prim
        for prim in dynamic_stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_pedicel_" in str(prim.GetPath())
    ]
    assert len(pedicel_joints) == 7
    terminal_joints = [
        prim
        for prim in dynamic_stage.Traverse()
        if prim.GetTypeName() == "PhysicsFixedJoint"
        and prim.GetName().startswith("TerminalBodyFixedJoint_")
    ]
    assert terminal_joints == []
    attached_tomatoes = [
        prim
        for prim in dynamic_stage.Traverse()
        if prim.GetAttribute("tomatoPlant:detachable").Get() is True
    ]
    assert len(attached_tomatoes) == len(terminal_bodies)
    assert all(not prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in attached_tomatoes)

    detached_bodies = [
        prim
        for prim in dynamic_stage.Traverse()
        if str(prim.GetPath()).startswith(TrussPhysicsConfig.TOMATO_DETACHED_BODY_PARENT_PATH)
        and prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    assert len(detached_bodies) == len(terminal_bodies)
    assert all(
        UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get() is True
        and UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get() is True
        for prim in detached_bodies
    )
    for prim in pedicel_joints:
        limit = UsdPhysics.LimitAPI.Get(prim, "rotX")
        assert limit.GetLowAttr().Get() == -TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG
        assert limit.GetHighAttr().Get() == TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG

    unscaled = deepcopy(branches)
    for branch in unscaled:
        if "_pedicel_" in branch["id"]:
            branch["drive_stiffness_scale"] = 1.0
    unscaled_stage, _ = build_stage(
        str(tmp_path / "unscaled.usda"),
        branches=unscaled,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    unscaled_pedicel = next(
        prim
        for prim in unscaled_stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_pedicel_" in str(prim.GetPath())
    )
    soft_stiffness = UsdPhysics.DriveAPI.Get(pedicel_joints[0], "rotX").GetStiffnessAttr().Get()
    full_stiffness = UsdPhysics.DriveAPI.Get(unscaled_pedicel, "rotX").GetStiffnessAttr().Get()
    assert soft_stiffness == pytest.approx(
        full_stiffness * TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE
    )

    technique = TrussStaticTechnique()
    fixed, _ = technique.apply(branches)
    static, _ = technique.apply(fixed)
    static_path = tmp_path / "static.usda"
    static_stage, _ = build_stage(
        str(static_path),
        branches=static,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    static_stage.GetRootLayer().Save()

    truss_d6 = [
        prim
        for prim in static_stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_static_curve_" in str(prim.GetPath())
    ]
    assert len(truss_d6) == 1
    root_limit = UsdPhysics.LimitAPI.Get(truss_d6[0], "rotX")
    assert root_limit.GetLowAttr().Get() == -18.0
    assert root_limit.GetHighAttr().Get() == 18.0
