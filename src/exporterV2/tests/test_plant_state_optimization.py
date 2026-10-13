from __future__ import annotations

import json

import pytest

from exporterV2 import cli
from exporterV2.core.optimizations.techniques.truss_static import PedicelLockTechnique
from exporterV2.plant_state_branches import PlantStateBranchesError, StemBranchesResult
from exporterV2.plant_state_optimization import (
    optimize_plant_state_branches,
    plant_state_budget_config,
)


def _branch(branch_id, parent, kind, n_links=1, **extra):
    return {
        "id": branch_id,
        "parent": parent,
        "attach_link": 1,
        "n_links": n_links,
        "radius": 0.005,
        "height": 0.02,
        "kind": kind,
        "joint_type": "d6",
        "attachment_joint_type": "d6",
        **extra,
    }


def _result():
    branches = [
        _branch("trunk", None, "stem", n_links=4, joint_type="fixed", attachment_joint_type="fixed"),
        _branch("Leaf_r0_o0_g1_petiole", "trunk", "leaf_petiole"),
        _branch("Leaf_r0_o0_g1_rachis", "Leaf_r0_o0_g1_petiole", "leaf_rachis", n_links=2),
        _branch("LeafVisual_g1_petiolule_left_01_physical", "Leaf_r0_o0_g1_rachis", "petiolule"),
        _branch("LeafVisual_g1_petiolule_right_01_physical", "Leaf_r0_o0_g1_rachis", "petiolule"),
        # Terminal petiolules are only recognisable by their kind.
        _branch("LeafVisual_g1_rachis_terminal_01_physical", "Leaf_r0_o0_g1_rachis", "petiolule"),
        _branch("Truss_r1_o0_g2_rachis", "trunk", "truss_rachis", n_links=3, physics_profile="truss"),
        *[
            _branch(f"Truss_r1_o0_g2_pedicel_0{i}", "Truss_r1_o0_g2_rachis", "pedicel", physics_profile="truss")
            for i in range(1, 3)
        ],
        _branch("Truss_r2_o0_g3_rachis", "trunk", "truss_rachis", n_links=3, physics_profile="truss"),
        *[
            _branch(f"Truss_r2_o0_g3_pedicel_0{i}", "Truss_r2_o0_g3_rachis", "pedicel", physics_profile="truss")
            for i in range(1, 3)
        ],
    ]
    return StemBranchesResult(
        branches=tuple(branches),
        source_axis_ids=(),
        represented_organ_ids=(),
        collapsed_duplicates=(),
        pose_mode="canonical",
    )


def _d6(result):
    return sum(b["n_links"] for b in result.branches if b["joint_type"] != "fixed")


def test_within_budget_is_unchanged():
    result = _result()
    assert _d6(result) == 16
    optimized, report = optimize_plant_state_branches(result, 16)
    assert optimized.branches == result.branches
    assert report["final_d6"] == 16
    assert report["locked_branch_ids"] == []


def test_petiolules_lock_by_kind_and_sync_attachment_joint():
    optimized, report = optimize_plant_state_branches(_result(), 13)
    by_id = {b["id"]: b for b in optimized.branches}
    for suffix in ("petiolule_left_01", "petiolule_right_01", "rachis_terminal_01"):
        branch = by_id[f"LeafVisual_g1_{suffix}_physical"]
        assert branch["joint_type"] == "fixed"
        assert branch["attachment_joint_type"] == "fixed"
    assert by_id["Leaf_r0_o0_g1_rachis"]["joint_type"] == "d6"
    assert report["final_d6"] == 13
    assert report["techniques"]["petiole_lock"]["joints_saved"] == 3
    assert "pedicel_lock" not in report["techniques"]


def test_pedicels_lock_one_truss_per_pass_without_touching_rachis():
    original = _result()
    optimized, report = optimize_plant_state_branches(original, 11)
    by_id = {b["id"]: b for b in optimized.branches}
    assert report["final_d6"] == 11
    assert report["techniques"]["pedicel_lock"]["passes"] == 1
    locked = [b["id"] for b in optimized.branches if "_pedicel_" in b["id"] and b["joint_type"] == "fixed"]
    assert locked == ["Truss_r1_o0_g2_pedicel_01", "Truss_r1_o0_g2_pedicel_02"]
    assert by_id["Truss_r1_o0_g2_rachis"] == dict(original.branches[6])
    assert len(optimized.branches) == len(original.branches)


def test_unreachable_budget_reports_locking_minimum():
    with pytest.raises(PlantStateBranchesError, match=r"minimum possible \(9 joints\)"):
        optimize_plant_state_branches(_result(), 8)


def test_unknown_techniques_are_rejected():
    with pytest.raises(PlantStateBranchesError, match="does not implement"):
        plant_state_budget_config(100, ("petiole_lock", "magic_reduce"))
    with pytest.raises(PlantStateBranchesError, match="technique set"):
        plant_state_budget_config(100, "everything")


def test_full_set_follows_yaml_priorities():
    config = plant_state_budget_config(100, "full")
    assert [entry["id"] for entry in config.techniques] == [
        "thin_link_lock", "petiole_lock", "lateral_reduce",
        "stem_collapse", "truss_static", "leaf_branch_reduce",
    ]


def test_pedicel_lock_never_builds_static_rachis_curves():
    branches = [dict(b, joint_type="fixed") if "_pedicel_" in b["id"] else dict(b) for b in _result().branches]
    assert not PedicelLockTechnique().can_apply(branches)


def test_optimize_flag_maps_to_joint_target(monkeypatch, tmp_path):
    captured = {}

    def fake_export(state, destination, **kwargs):
        captured.update(kwargs)
        raise RuntimeError("stop")

    monkeypatch.setattr(cli, "export_incremental_checkpoint", fake_export)
    for argv, expected in (
        (["--day", "50"], None),
        (["--day", "50", "--optimize"], cli.JOINT_TARGET),
        (["--day", "50", "--optimize", "--joint-budget", "120"], 120),
    ):
        args = cli.build_argument_parser().parse_args([*argv, "--output", str(tmp_path / "x.usda")])
        with pytest.raises(RuntimeError, match="stop"):
            cli.generate_from_args(args)
        assert captured["joint_budget"] == expected


def test_day_160_lock_only_export_refuses_the_physx_link_limit(tmp_path, capsys):
    # Locking keeps all 347 bodies; PhysX crashes from 256 articulation links.
    assert cli.main([
        "--day", "160", "--physical-petiolules", "--joint-budget", "150",
        "--output", str(tmp_path / "day160.usda"),
    ]) == 1
    assert "PhysX limit of 255" in capsys.readouterr().err


def test_day_160_full_export_merges_fixed_links_under_the_limit(tmp_path):
    from pxr import Usd, UsdPhysics

    destination = tmp_path / "day160.usda"
    assert cli.main([
        "--day", "160", "--physical-petiolules", "--joint-budget", "206",
        "--optimizer-techniques", "full", "--output", str(destination),
    ]) == 0
    manifest = json.loads(destination.with_suffix(".manifest.json").read_text())
    optimization = manifest["physics"]["joint_budget"]["optimization"]
    assert manifest["metadata"]["joint_budget"] == 206
    assert optimization["original_d6"] == 337
    assert optimization["final_d6"] == 206
    assert optimization["techniques"]["petiole_lock"]["joints_saved"] == 131
    assert optimization["articulation_links"] == 255
    assert optimization["link_limit_merges"] == 92
    stage = Usd.Stage.Open(str(destination))
    assert sum(prim.GetTypeName() == "PhysicsJoint" for prim in stage.Traverse()) == 206
    assert sum(prim.HasAPI(UsdPhysics.RigidBodyAPI) for prim in stage.Traverse()) == 255
