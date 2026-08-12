from copy import deepcopy

import numpy as np
import pytest
from pxr import UsdGeom, UsdPhysics

from exporterV2.core.detachment.metadata import PlantMetadataParser
from exporterV2.core.detachment.models import (
    CombinedForceTorqueModel,
    ForceThresholdModel,
)
from exporterV2.core.detachment.runtime import TomatoPlantRuntime
from exporterV2.core.detachment.state import FruitRuntimeData, FruitState, JointWrench
from exporterV2.core.tree_config import TrussPhysicsConfig
from exporterV2.core.usd.stage import build_stage


def make_fruit(fruit_id="tomato_1", model="force", duration=0.020):
    return FruitRuntimeData(
        fruit_id=fruit_id,
        attached_prim_path=f"/World/Stem/pedicel/{fruit_id}_Attached",
        detached_prim_path=f"/World/DetachedTomatoes/{fruit_id}_Detached",
        attachment_body_path=f"/World/Stem/pedicel_{fruit_id}",
        fruit_mass=0.08,
        fruit_radius=0.02,
        local_center=np.array([0.0, 0.0, 0.06]),
        model=model,
        force_threshold=10.0,
        torque_threshold=2.0,
        force_exponent=2.0,
        torque_exponent=2.0,
        minimum_break_duration=duration,
    )


class StaticParser:
    def __init__(self, fruits):
        self.fruits = fruits

    def parse(self, stage, root):
        return deepcopy(self.fruits)


class FakeReader:
    def __init__(self, wrench_by_path):
        self.wrench_by_path = wrench_by_path
        self.refresh_count = 0
        self.batch_read_count = 0

    def initialize(self, fruits):
        self.paths = [fruit.attachment_body_path for fruit in fruits]

    def read(self, path):
        return self.wrench_by_path[path]

    def read_many(self, paths):
        self.batch_read_count += 1
        return {path: self.wrench_by_path[path] for path in paths}

    def refresh(self, fruits):
        self.refresh_count += 1
        self.initialize(fruits)


class FakeBackend:
    def __init__(self):
        self.batches = []
        self.reset_count = 0

    def initialize(self, fruits):
        pass

    def detach_batch(self, fruits):
        self.batches.append([fruit.fruit_id for fruit in fruits])

    def reset(self, fruits):
        self.reset_count += 1


def make_runtime(fruits, wrench_by_path, *, sensor_hz=None):
    reader = FakeReader(wrench_by_path)
    backend = FakeBackend()
    runtime = TomatoPlantRuntime(
        stage=object(),
        metadata_parser=StaticParser(fruits),
        wrench_reader=reader,
        backend=backend,
        sensor_hz=sensor_hz,
    ).initialize()
    return runtime, reader, backend


def test_force_and_combined_models():
    fruit = make_fruit()
    force = np.array([6.0, 8.0, 0.0])
    torque = np.array([0.0, 0.0, 1.0])
    assert ForceThresholdModel().evaluate(fruit, force, torque, 0.01) == pytest.approx(1.0)
    assert CombinedForceTorqueModel().evaluate(fruit, force, torque, 0.01) == pytest.approx(1.25)


def test_threshold_must_persist_and_event_is_emitted_once():
    fruit = make_fruit(duration=0.020)
    wrench = JointWrench(np.array([11.0, 0.0, 0.0]), np.zeros(3))
    runtime, reader, backend = make_runtime([fruit], {fruit.attachment_body_path: wrench})

    assert runtime.step(0.010) == []
    events = runtime.step(0.010)
    assert [event.fruit_id for event in events] == [fruit.fruit_id]
    assert runtime.step(0.010) == []
    assert backend.batches == [[fruit.fruit_id]]
    assert reader.refresh_count == 0


def test_subthreshold_sample_resets_persistence():
    fruit = make_fruit(duration=0.020)
    reader_wrench = JointWrench(np.array([11.0, 0.0, 0.0]), np.zeros(3))
    runtime, reader, _ = make_runtime([fruit], {fruit.attachment_body_path: reader_wrench})
    runtime.step(0.010)
    reader.wrench_by_path[fruit.attachment_body_path] = JointWrench(np.zeros(3), np.zeros(3))
    runtime.step(0.010)
    reader.wrench_by_path[fruit.attachment_body_path] = reader_wrench
    assert runtime.step(0.010) == []


def test_multiple_fruits_detach_in_one_batch_and_reset():
    fruits = [make_fruit(f"tomato_{index}", duration=0.010) for index in range(3)]
    wrenches = {
        fruit.attachment_body_path: JointWrench(
            np.array([11.0 if fruit.fruit_id != "tomato_1" else 0.0, 0.0, 0.0]),
            np.zeros(3),
        )
        for fruit in fruits
    }
    runtime, reader, backend = make_runtime(fruits, wrenches)
    events = runtime.step(0.010)
    assert [event.fruit_id for event in events] == ["tomato_0", "tomato_2"]
    assert backend.batches == [["tomato_0", "tomato_2"]]
    assert reader.batch_read_count == 1
    assert runtime.fruits[1].state is FruitState.ATTACHED

    runtime.reset()
    assert all(fruit.state is FruitState.ATTACHED for fruit in runtime.fruits)
    assert backend.reset_count == 1


def test_reset_is_repeatable_for_rl_episode_reuse():
    fruit = make_fruit(duration=0.010)
    wrench = JointWrench(np.array([11.0, 0.0, 0.0]), np.zeros(3))
    runtime, _, backend = make_runtime([fruit], {fruit.attachment_body_path: wrench})
    for _ in range(100):
        assert len(runtime.step(0.010)) == 1
        runtime.reset()
        assert runtime.fruits[0].state is FruitState.ATTACHED
        assert runtime.fruits[0].damage == 0.0
    assert backend.reset_count == 100


def test_sensor_rate_limits_tensor_reads_without_skipping_physics_time():
    fruit = make_fruit(duration=0.020)
    wrench = JointWrench(np.zeros(3), np.zeros(3))
    runtime, reader, _ = make_runtime(
        [fruit],
        {fruit.attachment_body_path: wrench},
        sensor_hz=60.0,
    )
    for _ in range(480):
        assert runtime.step(1.0 / 480.0) == []
    assert reader.batch_read_count == 60


def test_generated_usd_contains_equivalent_attached_and_detached_bodies(tmp_path):
    branches = [{
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 1,
        "radius": 0.01,
        "height": 0.05,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "fixed",
    }]
    terminal = [{
        "id": "tomato_test",
        "kind": "tomato",
        "shape": "sphere",
        "parent_branch_id": "trunk",
        "radius": 0.02,
        "mass": 0.08,
    }]
    stage, root = build_stage(
        str(tmp_path / "detachment.usda"),
        branches=branches,
        terminal_bodies=terminal,
        skip_limit_check=True,
    )
    fruits = PlantMetadataParser().parse(stage, root)
    assert len(fruits) == 1
    fruit = fruits[0]
    assert fruit.force_threshold == pytest.approx(
        TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N
    )
    assert fruit.minimum_break_duration == pytest.approx(0.020)

    attached = stage.GetPrimAtPath(fruit.attached_prim_path)
    detached = stage.GetPrimAtPath(fruit.detached_prim_path)
    assert str(attached.GetPath()).startswith(f"{root}/")
    assert not str(detached.GetPath()).startswith(f"{root}/")
    assert fruit.fruit_mass == pytest.approx(
        UsdPhysics.MassAPI(detached).GetMassAttr().Get()
    )
    assert UsdGeom.Sphere.Get(stage, f"{fruit.attached_prim_path}/Sphere").GetRadiusAttr().Get() == (
        UsdGeom.Sphere.Get(stage, f"{fruit.detached_prim_path}/Sphere").GetRadiusAttr().Get()
    )
    attachment_body = stage.GetPrimAtPath(fruit.attachment_body_path)
    assert attachment_body.HasAPI(UsdPhysics.RigidBodyAPI)
    assert not attached.HasAPI(UsdPhysics.RigidBodyAPI)
    assert UsdPhysics.RigidBodyAPI(detached).GetKinematicEnabledAttr().Get() is True
