from __future__ import annotations

import math
from pathlib import Path
import sys

from pxr import Usd, UsdPhysics


SRC_DIR = Path(__file__).resolve().parents[5]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.adapters.groimp_csv.tests.truss_visual_lab.generate_truss_visual_lab import (
    PEDICEL_BEND_LIMIT_DEG,
    PEDICEL_DRIVE_STIFFNESS_SCALE,
    STATIC_ROOT_BEND_LIMIT_DEG,
    STATIC_ROOT_DRIVE_STIFFNESS_SCALE,
    STAGE_FILENAMES,
    build_stage_configurations,
    count_stage_d6_joints,
    create_synthetic_truss,
    generate_visual_stages,
    rachis_total_length,
    reduce_rachis_to_one_link,
)


def _pedicel_ids(branches: list[dict]) -> set[str]:
    return {branch["id"] for branch in branches if "_pedicel_" in branch["id"]}


def test_transformations_preserve_length_and_tomato_parents():
    base, terminal_bodies = create_synthetic_truss()
    configurations = build_stage_configurations()
    expected_parent_ids = {body["parent_branch_id"] for body in terminal_bodies}

    assert len(configurations) == len(STAGE_FILENAMES) == 4
    for branches in configurations.values():
        assert expected_parent_ids <= _pedicel_ids(branches)
        assert math.isclose(rachis_total_length(branches), rachis_total_length(base))

    one_link = reduce_rachis_to_one_link(base)
    rachis = [branch for branch in one_link if "_rachis" in branch["id"] and "_pedicel_" not in branch["id"]]
    assert len(rachis) == 1
    assert rachis[0]["n_links"] == 1


def test_generated_usds_capture_joint_reduction(tmp_path):
    generated = generate_visual_stages(tmp_path)
    configurations = build_stage_configurations()
    _, terminal_bodies = create_synthetic_truss()

    assert set(generated) == set(STAGE_FILENAMES)
    stages = {}
    for filename, path in generated.items():
        assert path.is_file()
        stage = Usd.Stage.Open(str(path))
        assert stage
        stages[filename] = stage

        pedicel_ids = _pedicel_ids(configurations[filename])
        for body in terminal_bodies:
            assert body["parent_branch_id"] in pedicel_ids
            pedicel_path = f"/World/Stem/{body['parent_branch_id']}_Link_01"
            assert stage.GetPrimAtPath(pedicel_path).IsValid()

    counts = {filename: count_stage_d6_joints(stage) for filename, stage in stages.items()}
    assert counts["00_current_simplified.usda"] == counts["02_opt_fixed_pedicels.usda"]
    assert counts["01_dynamic_pedicels.usda"] > counts["02_opt_fixed_pedicels.usda"]
    assert (
        counts["01_dynamic_pedicels.usda"]
        > counts["02_opt_fixed_pedicels.usda"]
        > counts["03_opt_static_prebent_truss.usda"]
    )
    assert counts["03_opt_static_prebent_truss.usda"] == 1


def test_dynamic_pedicel_limits_are_patched_in_usd(tmp_path):
    dynamic_path = generate_visual_stages(tmp_path)["01_dynamic_pedicels.usda"]
    stage = Usd.Stage.Open(str(dynamic_path))
    pedicel_joints = [
        prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "PhysicsJoint" and "_pedicel_" in str(prim.GetPath())
    ]

    assert len(pedicel_joints) == 7
    for joint in pedicel_joints:
        for axis in ("rotX", "rotY"):
            limit = UsdPhysics.LimitAPI.Get(joint, axis)
            assert limit.GetLowAttr().Get() == -PEDICEL_BEND_LIMIT_DEG
            assert limit.GetHighAttr().Get() == PEDICEL_BEND_LIMIT_DEG
            drive = UsdPhysics.DriveAPI.Get(joint, axis)
            assert drive.GetStiffnessAttr().Get() > 0.0
        assert joint.GetCustomDataByKey("trussLab:driveStiffnessScale") == PEDICEL_DRIVE_STIFFNESS_SCALE


def test_static_prebent_truss_has_one_soft_root_d6(tmp_path):
    static_path = generate_visual_stages(tmp_path)["03_opt_static_prebent_truss.usda"]
    stage = Usd.Stage.Open(str(static_path))
    d6_joints = [prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"]

    assert len(d6_joints) == 1
    root_joint = d6_joints[0]
    assert "_rachis_curve_01_Link_01/AttachJoint" in str(root_joint.GetPath())
    assert root_joint.GetCustomDataByKey("trussLab:bendLimitDeg") == STATIC_ROOT_BEND_LIMIT_DEG
    assert (
        root_joint.GetCustomDataByKey("trussLab:driveStiffnessScale")
        == STATIC_ROOT_DRIVE_STIFFNESS_SCALE
    )
    fixed_curve_joints = [
        prim
        for prim in stage.Traverse()
        if prim.GetTypeName() == "PhysicsFixedJoint" and "_rachis_curve_" in str(prim.GetPath())
    ]
    assert len(fixed_curve_joints) >= 4
