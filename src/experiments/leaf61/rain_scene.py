"""Staggered free-fall tomatoes; bounded interactive records and contact evidence."""

import json
import time

import numpy as np
from plant_model import yaw_matrix
from scene import save


def rain_schedule(offsets, yaws, points, height, radius, count, seed, wave_sizes=None):
    rng = np.random.default_rng(seed)
    # Sample real lamina vertices in the free distal half, then jitter slightly.
    eligible = points[(points[:, 0] > 0.035) & (points[:, 0] < 0.075)]
    if not len(eligible):
        raise ValueError("No distal lamina points for rain targeting")
    top = max(float(points[:, 2].max() + o[2]) for o in offsets)
    if wave_sizes:
        if sum(wave_sizes) != count or any(v < 1 or v > 12 for v in wave_sizes):
            raise ValueError("Invalid wave sizes")
        result, release = [], 0.0
        for wave, size in enumerate(wave_sizes):
            if wave:
                release += float(rng.uniform(2.0, 3.5))
            spawn_z = top + radius + rng.uniform(0.10, 0.18)
            positions = []
            for _ in range(size):
                for attempt in range(1000):
                    leaf = int(rng.integers(len(offsets)))
                    point = (
                        eligible[rng.integers(len(eligible))] @ yaw_matrix(yaws[leaf]).T
                        + offsets[leaf]
                    )
                    point[:2] += rng.uniform(-0.008, 0.008, 2)
                    point[2] = spawn_z
                    if all(
                        np.linalg.norm(point - previous) > 2 * radius + 0.005
                        for previous in positions
                    ):
                        break
                else:
                    raise ValueError(
                        "Cannot place a rain wave without overlapping tomatoes"
                    )
                positions.append(point.copy())
                result.append(
                    {
                        "tomato": len(result),
                        "wave": wave,
                        "wave_size": size,
                        "after_start_s": release,
                        "position_m": point.tolist(),
                        "target_leaf": leaf,
                    }
                )
        return result
    leaves = rng.permutation(len(offsets))
    result, release = [], 0.0
    for i in range(count):
        leaf = int(leaves[i % len(leaves)])
        point = (
            eligible[rng.integers(len(eligible))] @ yaw_matrix(yaws[leaf]).T
            + offsets[leaf]
        )
        point[:2] += rng.uniform(-0.008, 0.008, 2)
        point[2] = top + radius + rng.uniform(0.10, 0.18)
        release += float(rng.uniform(0.6, 1.4)) if i else 0.0
        result.append(
            {
                "tomato": i,
                "after_start_s": release,
                "position_m": point.tolist(),
                "target_leaf": leaf,
            }
        )
    return result


class TomatoRain:
    def __init__(self, stage, a, c, physical, visual, offsets, yaws, points):
        from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

        self.stage, self.a, self.c = stage, a, c
        self.offsets, self.yaws, self.points = offsets, yaws, points
        self.paths, self.visuals = [physical], [visual]
        for i in range(1, a.rain_count):
            body, shown = f"/World/RainBodies/T{i:03d}", f"/World/RainVisuals/T{i:03d}"
            UsdGeom.Xform.Define(stage, "/World/RainBodies")
            UsdGeom.Xform.Define(stage, "/World/RainVisuals")
            Sdf.CopySpec(stage.GetRootLayer(), physical, stage.GetRootLayer(), body)
            Sdf.CopySpec(stage.GetRootLayer(), visual, stage.GetRootLayer(), shown)
            stage.GetPrimAtPath(body).GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(2, 0, 2 + i * 0.1)
            )
            self.paths.append(body)
            self.visuals.append(shown)
        self.gravity, self.collisions, self.channels = [], [], []
        for body, shown in zip(self.paths, self.visuals):
            prim = stage.GetPrimAtPath(body)
            self.gravity.append(
                PhysxSchema.PhysxRigidBodyAPI(prim).GetDisableGravityAttr()
            )
            collision = UsdPhysics.CollisionAPI(
                stage.GetPrimAtPath(body + "/Collider")
            ).GetCollisionEnabledAttr()
            collision.Set(False)
            self.collisions.append(collision)
            vp = stage.GetPrimAtPath(shown)
            UsdGeom.Imageable(vp).MakeInvisible()
            self.channels.append(
                (
                    vp.GetAttribute("xformOp:translate"),
                    vp.GetAttribute("xformOp:orient"),
                )
            )
        self.wave_sizes = (
            [int(v) for v in a.rain_waves.split(",")] if a.rain_waves else None
        )
        self.seed = a.rain_seed
        self.schedule = rain_schedule(
            offsets,
            yaws,
            points,
            c.height,
            c.radius,
            a.rain_count,
            self.seed,
            self.wave_sizes,
        )

    def initialize(self, world):
        self.view = world.physics_sim_view.create_rigid_body_view(self.paths)
        self.indices = np.array(
            [list(self.view.prim_paths).index(p) for p in self.paths], dtype=np.int32
        )

    def release(self, i):
        from pxr import UsdGeom

        point = self.schedule[i]["position_m"]
        indices = np.array([self.indices[i]], dtype=np.int32)
        transforms = np.asarray(self.view.get_transforms()).copy()
        velocities = np.asarray(self.view.get_velocities()).copy()
        transforms[self.indices[i]] = [*point, 0, 0, 0, 1]
        velocities[self.indices[i]] = 0
        # Tensor API requires full-view buffers; indices restrict the actual write.
        self.view.set_transforms(transforms, indices)
        self.view.set_velocities(velocities, indices)
        self.collisions[i].Set(True)
        self.gravity[i].Set(False)
        UsdGeom.Imageable(self.stage.GetPrimAtPath(self.visuals[i])).MakeVisible()

    def reset(self):
        from pxr import UsdGeom

        for i in range(len(self.paths)):
            self.gravity[i].Set(True)
            self.collisions[i].Set(False)
            UsdGeom.Imageable(self.stage.GetPrimAtPath(self.visuals[i])).MakeInvisible()
        parking = np.array(
            [[2, 0, 2 + i * 0.1, 0, 0, 0, 1] for i in range(len(self.paths))],
            dtype=np.float32,
        )
        transforms = np.asarray(self.view.get_transforms()).copy()
        transforms[self.indices] = parking
        self.view.set_transforms(transforms, self.indices)
        self.view.set_velocities(
            np.zeros((len(self.paths), 6), dtype=np.float32), self.indices
        )
        self.seed += 1
        self.schedule = rain_schedule(
            self.offsets,
            self.yaws,
            self.points,
            self.c.height,
            self.c.radius,
            len(self.paths),
            self.seed,
            self.wave_sizes,
        )

    def read(self):
        return np.asarray(self.view.get_transforms())[self.indices].copy()

    def draw(self, transforms, released):
        from pxr import Gf

        # Visual proxies only; no transforms are commanded to released bodies.
        for i in range(released):
            t = transforms[i]
            pos, rot = self.channels[i]
            pos.Set(Gf.Vec3d(*map(float, t[:3])))
            rot.Set(Gf.Quatf(float(t[6]), Gf.Vec3f(*map(float, t[3:6]))))


def run_rain(app, world, a, c, rain, read_state, animate, initial, info):
    import omni.usd
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    rain.initialize(world)
    state = {"quit": False, "pause": False, "restart": False}
    label, window = None, None
    if a.gui:
        from omni import ui

        window = ui.Window("Tomato rain", width=470, height=230)
        with window.frame, ui.VStack():
            ui.Label(
                f"{a.rain_count} tomatoes at {a.ball_mass * 1000:g} g"
                + (
                    " | increasing intensity"
                    if rain.wave_sizes
                    else ", one every 0.6–1.4 s"
                )
            )
            ui.Label(
                "Bursts: " + " → ".join(map(str, rain.wave_sizes))
                if rain.wave_sizes
                else "Random positions above the canopy; free fall onto all leaves"
            )
            label = ui.Label("Plant settling...")
            ui.Button(
                "Pause / Resume rain",
                clicked_fn=lambda: state.update(pause=not state["pause"]),
            )
            ui.Button("Restart rain", clicked_fn=lambda: state.update(restart=True))
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    hits, events = set(), []
    actor_to_ball = {p: i for i, p in enumerate(rain.paths)}
    actor_to_leaf = {
        f"/World/Leaves/L{i:03d}/Link{j}": i
        for i in range(len(initial))
        for j in range(3)
    }
    n, released, batch = 0, 0, 0
    start_at = 1 + c.settle
    base_step = world.current_time_step_index
    stage_id = int(omni.usd.get_context().get_stage_id())
    sim = get_physx_simulation_interface()
    saved = False
    records, frame_work = [], []
    releases = []
    equilibrium = None
    max_motion = np.zeros(len(initial))
    max_root_drift = 0.0
    fallen = set()
    block = 0.0
    measured_start = None
    maximum_separation = 0.0

    def contact(headers, details):
        nonlocal maximum_separation
        for h in headers:
            names = [
                str(PhysicsSchemaTools.intToSdfPath(v)) for v in (h.actor0, h.actor1)
            ]
            balls = [actor_to_ball[p] for p in names if p in actor_to_ball]
            leaves = [actor_to_leaf[p] for p in names if p in actor_to_leaf]
            if not balls or not leaves:
                continue
            for j in range(
                h.contact_data_offset, h.contact_data_offset + h.num_contact_data
            ):
                d = details[j]
                impulse = float(np.linalg.norm(d.impulse))
                if impulse > 0:
                    hits.add(leaves[0])
                    if not saved:
                        maximum_separation = max(
                            maximum_separation, -float(d.separation)
                        )
                        events.append(
                            [
                                n / c.hz,
                                balls[0],
                                leaves[0],
                                impulse,
                                float(d.separation),
                            ]
                        )

    subscription = sim.subscribe_contact_report_events(contact)

    def write_report(completed):
        wall = time.perf_counter() - measured_start if measured_start else 0
        checks = {
            "completed": completed,
            "all_released": released == len(rain.paths),
            "all_fell": len(fallen) == len(rain.paths),
            "multiple_leaves_contacted": len(hits) >= 2,
            "attachment": max_root_drift < 0.0002,
            "motion": int((max_motion > 0.001).sum()) >= 2,
            "finite_and_clock": True,
        }
        target = a.run_dir / f"rain-{batch:03d}"
        target.mkdir(exist_ok=True)
        save(
            target / "schedule.json",
            {"seed": rain.seed, "schedule": rain.schedule, "actual_releases": releases},
        )
        np.savez_compressed(
            target / "trace.npz",
            times=np.array([r[0] for r in records]),
            tomatoes=np.array([r[1] for r in records]),
            leaf_poses=np.array([r[2] for r in records]),
            contacts=np.asarray(events).reshape(-1, 5),
        )
        report = {
            "status": "diagnostic"
            if not completed or all(checks.values())
            else "failed",
            "interrupted": not completed,
            "checks": checks,
            "rain_count": len(rain.paths),
            "leaves_hit": sorted(hits),
            "leaf_max_segment_motion_m": max_motion.tolist(),
            "maximum_root_drift_m": max_root_drift,
            "maximum_physx_contact_penetration_m": maximum_separation,
            "numeric_acceptance": "Rain smoke test only; full surface stretch, recovery and visual penetration not certified",
            "visual_acceptance": "pending",
            "physics_hz": c.hz,
            "render_hz": a.render_hz,
            "rendered_fps": len(frame_work) / wall
            if wall and (a.gui or a.render)
            else None,
            "p95_frame_work_ms": float(np.percentile(frame_work, 95) * 1000)
            if frame_work
            else None,
            "sampling_hz": 10,
            "no_body_commands_during_flight": True,
        }
        save(target / "report.json", report)
        save(a.run_dir / "report.json", report)
        print(
            "RAIN_REPORT",
            json.dumps(
                {k: v for k, v in report.items() if k != "leaf_max_segment_motion_m"}
            ),
            flush=True,
        )

    save(
        a.run_dir / "rain_config.json",
        {
            "count": a.rain_count,
            "seed": a.rain_seed,
            "wave_sizes": rain.wave_sizes,
            "leaf_leaf_collisions": False,
            "all_tomato_leaf_collisions": True,
            "tomato_tomato_collisions": True,
            "support_collisions": False,
        },
    )
    info["scenario"] = "staggered_tomato_rain"
    save(a.run_dir / "runtime.json", info)
    stride = max(1, round(c.hz / a.render_hz))
    deadline = time.perf_counter()
    while app.is_running() and not state["quit"]:
        tick = time.perf_counter()
        t = n / c.hz
        if not world.is_playing():
            raise RuntimeError("Timeline stopped during rain")
        if n == c.hz:
            world.get_physics_context().set_gravity(-9.81)
            for i in range(len(initial)):
                sim.wake_up(
                    stage_id,
                    PhysicsSchemaTools.sdfPathToInt(
                        f"/World/Leaves/L{i:03d}/FixedPetiole"
                    ),
                )
        if state.pop("restart", False):
            if not saved:
                write_report(False)
            rain.reset()
            batch += 1
            released, saved = 0, False
            start_at = max(t + 3, 1 + c.settle)
            hits.clear()
            events.clear()
            records.clear()
            frame_work.clear()
            releases.clear()
            fallen.clear()
            max_motion[:] = 0
            max_root_drift = 0.0
            maximum_separation = 0.0
            equilibrium, measured_start = None, None
        if state["pause"]:
            start_at += 1 / c.hz
        while (
            not state["pause"]
            and released < len(rain.paths)
            and t >= start_at + rain.schedule[released]["after_start_s"]
        ):
            if equilibrium is None:
                equilibrium = read_state()[0].copy()
                measured_start = time.perf_counter()
            rain.release(released)
            releases.append(
                {
                    "tomato": released,
                    "time_s": t,
                    "position_m": rain.schedule[released]["position_m"],
                }
            )
            released += 1
        n += 1
        world.step(render=False)
        if world.current_time_step_index != base_step + n:
            raise RuntimeError("Rain physics step count mismatch")
        poses, _, _ = read_state()
        tomatoes = rain.read()
        if not np.isfinite(poses).all() or not np.isfinite(tomatoes).all():
            raise RuntimeError("Nonfinite rain state")
        if not saved:
            max_root_drift = max(
                max_root_drift,
                float(
                    np.linalg.norm(poses[:, 0, :3] - initial[:, 0, :3], axis=1).max()
                ),
            )
            if equilibrium is not None:
                max_motion = np.maximum(
                    max_motion,
                    np.linalg.norm(
                        poses[:, 1:, :3] - equilibrium[:, 1:, :3], axis=2
                    ).max(axis=1),
                )
            for i in range(released):
                if tomatoes[i, 2] < rain.schedule[i]["position_m"][2] - 0.05:
                    fallen.add(i)
            if n % max(1, c.hz // 10) == 0:
                records.append((n / c.hz, tomatoes.copy(), poses.copy()))
        if n % stride == 0 and (a.gui or a.render):
            animate(poses)
            rain.draw(tomatoes, released)
            if label:
                label.text = (
                    f"Released {released}/{len(rain.paths)} | leaves hit {len(hits)}"
                    + (" | releases paused" if state["pause"] else "")
                )
            world.render()
        block += time.perf_counter() - tick
        if n % stride == 0:
            if measured_start is not None and not saved:
                frame_work.append(block)
            block = 0
        end = start_at + rain.schedule[-1]["after_start_s"] + 8
        if not saved and released == len(rain.paths) and n / c.hz >= end:
            write_report(True)
            saved = True
            if not a.gui or a.gui_benchmark:
                break
        if a.gui and not a.gui_benchmark:
            deadline += 1 / c.hz
            time.sleep(max(0, deadline - time.perf_counter()))
    if not saved:
        write_report(False)
    subscription.unsubscribe()
    _ = window
