"""Headless Isaac Sim validation for wrench sensing and tomato swapping.

Run with:
    ~/isaacsim/python.sh src/exporterV2/core/detachment/tests/isaac_detachment_smoke.py --sensor-only
    ~/isaacsim/python.sh src/exporterV2/core/detachment/tests/isaac_detachment_smoke.py
"""

import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--sensor-only", action="store_true")
parser.add_argument("--fruits", type=int, default=1, choices=(1, 3))
parser.add_argument("--reset-cycles", type=int, default=0)
args = parser.parse_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation, RigidPrim, XFormPrim

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", ".."))
if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from exporterV2.core.detachment import TomatoPlantRuntime
from exporterV2.core.physics import apply_physx_articulation_settings, apply_physx_scene_settings
from exporterV2.core.usd import build_stage


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    usd_path = "/tmp/exporterV2_detachment_smoke.usda"
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 1,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
            "joint_type": "fixed",
        },
        {
            "id": "pedicel",
            "parent": "trunk",
            "attach_link": 1,
            "n_links": 1,
            "radius": 0.003,
            "height": 0.04,
            "tilt": 45.0,
            "rot": 0.0,
            "joint_type": "d6",
            "physics_profile": "truss",
        },
    ]
    terminal_bodies = [
        {
            "id": f"tomato_sensor_{index}",
            "kind": "tomato",
            "shape": "sphere",
            "parent_branch_id": "pedicel",
            "radius": 0.02,
            "mass": 0.08,
            "break_force": 5.0 if index == 0 else 50.0,
            "detachment_model": "force",
            "minimum_break_duration": 0.020,
        }
        for index in range(args.fruits)
    ]
    stage, root_path = build_stage(
        usd_path,
        branches=branches,
        terminal_bodies=terminal_bodies,
        skip_limit_check=True,
    )
    print("[GATE] USD built", flush=True)
    apply_physx_scene_settings(stage)
    print("[GATE] PhysX scene settings applied", flush=True)
    apply_physx_articulation_settings(stage, root_path)
    print("[GATE] articulation settings applied", flush=True)
    stage.GetRootLayer().Save()
    print("[GATE] USD saved", flush=True)
    omni.usd.get_context().open_stage(usd_path)
    print("[GATE] USD opened", flush=True)

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 480.0, rendering_dt=1.0 / 60.0)
    print("[GATE] World created", flush=True)
    articulation = Articulation(root_path, name="detachment_smoke_articulation")
    world.scene.add(articulation)
    print("[GATE] Articulation added to scene", flush=True)
    world.reset()
    print("[GATE] World reset", flush=True)
    articulation.initialize()
    print("[GATE] Articulation initialized", flush=True)
    print(f"[GATE] body_names={list(articulation.body_names)}", flush=True)
    print(f"[GATE] joint_names={list(articulation.joint_names)}", flush=True)
    runtime = TomatoPlantRuntime(
        root_path,
        articulation=articulation,
        world=world,
    ).initialize()
    print("[GATE] Detachment runtime initialized", flush=True)
    fruit = runtime.fruits[0]
    attached = XFormPrim(fruit.attached_prim_path)
    parent_body = RigidPrim(fruit.attachment_body_path)
    parent_body.initialize()

    print("[GATE] Settling simulation", flush=True)
    for _ in range(120):
        world.step(render=False)
    print("[GATE] Settling complete", flush=True)
    gravity_wrench = runtime._reader.read(fruit.attachment_body_path)
    positions, orientations = attached.get_world_poses()
    velocities = parent_body.get_velocities()
    body_name = os.path.basename(fruit.attachment_body_path)
    body_index = articulation.get_body_index(body_name)
    print(
        "[SENSOR] "
        f"body={body_name} body_index={body_index} "
        f"force={gravity_wrench.force.tolist()} torque={gravity_wrench.torque.tolist()} "
        f"pose={positions[0].tolist()} velocity={velocities[0].tolist()}",
        flush=True,
    )
    require(np.isfinite(gravity_wrench.force).all(), "gravity wrench is not finite")
    require(np.linalg.norm(gravity_wrench.force) > 0.1, "gravity wrench is unexpectedly zero")
    require(np.isfinite(positions).all() and np.isfinite(orientations).all(), "pose is not finite")

    for _ in range(12):
        parent_body.apply_forces(np.array([[0.0, 0.0, 2.0]], dtype=np.float32))
        world.step(render=False)
    loaded_wrench = runtime._reader.read(fruit.attachment_body_path)
    print(
        f"[SENSOR] controlled_force={loaded_wrench.force.tolist()} "
        f"norm={np.linalg.norm(loaded_wrench.force):.6f}",
        flush=True,
    )
    require(np.linalg.norm(loaded_wrench.force) > 1.0, "controlled force did not reach the joint sensor")
    if args.sensor_only:
        print("[PASS] sensor gate", flush=True)
        return

    events = []
    for _ in range(30):
        parent_body.apply_forces(np.array([[0.0, 0.0, 8.0]], dtype=np.float32))
        world.step(render=False)
        events.extend(runtime.step(1.0 / 480.0))
        if events:
            break
    require(len(events) == 1, f"expected one detachment event, got {len(events)}")

    detached = RigidPrim(fruit.detached_prim_path)
    detached.initialize()
    after_position, after_orientation = detached.get_world_poses()
    after_velocity = detached.get_velocities()
    transfer_snapshot = runtime._backend.last_snapshot_by_fruit[fruit.fruit_id]
    position_error = float(
        np.linalg.norm(after_position - transfer_snapshot["positions"])
    )
    velocity_error = float(
        np.linalg.norm(after_velocity - transfer_snapshot["velocities"])
    )
    print(
        f"[SWAP] position_error={position_error:.6e} velocity_error={velocity_error:.6e} "
        f"bodies_after={runtime.articulation.num_bodies} "
        f"body_names_after={list(runtime.articulation.body_names)}",
        flush=True,
    )
    require(position_error < 0.05, "detached tomato teleported")
    require(velocity_error < 0.05, "detached tomato did not inherit velocity")
    expected_bodies = 2
    require(
        runtime.articulation.num_bodies == expected_bodies,
        "attached tomato remains in articulation topology",
    )
    require(len(runtime.get_detached_fruits()) == 1, "runtime state did not detach")
    require(
        all(item.state.name == "ATTACHED" for item in runtime.fruits[1:]),
        "a non-target tomato detached",
    )
    dynamic_start = after_position.copy()
    for _ in range(12):
        world.step(render=False)
    dynamic_position, _ = detached.get_world_poses()
    dynamic_displacement = float(np.linalg.norm(dynamic_position - dynamic_start))
    print(f"[SWAP] dynamic_displacement={dynamic_displacement:.6e}", flush=True)
    require(
        dynamic_displacement > 1e-5,
        "detached proxy remained kinematic after activation",
    )
    require(runtime.step(1.0 / 480.0) == [], "detachment event repeated")
    # RigidPrim subscribes to physics-ready events. Drop this diagnostic wrapper
    # before reset disables its target body, otherwise it tries to rebind to an
    # intentionally absent PhysX object.
    del detached
    import gc

    gc.collect()
    for cycle in range(args.reset_cycles):
        runtime.reset()
        require(
            runtime.articulation.num_bodies == 2,
            f"reset cycle {cycle}: articulation topology was not restored",
        )
        require(
            len(runtime.get_detached_fruits()) == 0,
            f"reset cycle {cycle}: stale detached runtime state",
        )
    print("[PASS] sensor and swap gates", flush=True)


try:
    main()
except BaseException:
    import traceback

    print("[FAIL] uncaught exception in Isaac detachment smoke", flush=True)
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    raise
finally:
    simulation_app.close()
