"""Rain momentum loading plus a bounded, purely visual field of streaks."""

import time

import numpy as np
from plant_model import summary
from scene import save
from storm_model import RainImpulses, rain_envelope

from model import area, edges, skin


class RainVisual:
    def __init__(self, stage, offsets, points, seed):
        from pxr import Gf, UsdGeom, Vt

        self.rng = np.random.default_rng(seed)
        self.low = np.r_[offsets[:, :2].min(0) - 0.14, 0.015]
        self.high = np.r_[
            offsets[:, :2].max(0) + 0.14, max(points[:, 2].max() + offsets[:, 2]) + 0.25
        ]
        self.xyz = self.rng.uniform(self.low, self.high, (512, 3))
        self.curve = UsdGeom.BasisCurves.Define(stage, "/World/WaterRainVisual")
        self.curve.CreateTypeAttr().Set("linear")
        self.curve.CreateCurveVertexCountsAttr().Set(
            Vt.IntArray.FromNumpy(np.full(512, 2, np.int32))
        )
        self.curve.CreateWidthsAttr().Set([0.0006])
        self.curve.SetWidthsInterpolation("constant")
        self.curve.CreateDisplayColorAttr().Set([Gf.Vec3f(0.5, 0.72, 0.9)])
        self.curve.CreateDisplayOpacityAttr().Set([0.6])
        self.channel = self.curve.CreatePointsAttr()

    def set_rate(self, rate):
        from pxr import Vt

        count = max(1, min(2560, round(512 * rate / 200)))
        self.xyz = self.rng.uniform(self.low, self.high, (count, 3))
        self.curve.GetCurveVertexCountsAttr().Set(
            Vt.IntArray.FromNumpy(np.full(count, 2, np.int32))
        )

    def update(self, t, envelope):
        from pxr import UsdGeom, Vt

        if envelope <= 0:
            UsdGeom.Imageable(self.curve).MakeInvisible()
            return
        UsdGeom.Imageable(self.curve).MakeVisible()
        p = self.xyz.copy()
        p[:, 2] = self.low[2] + np.mod(
            p[:, 2] - self.low[2] - 7 * t, self.high[2] - self.low[2]
        )
        p[int(len(p) * envelope) :, 2] = -10
        endpoints = np.repeat(p, 2, axis=0)
        endpoints[1::2, 2] += 0.012
        self.channel.Set(Vt.Vec3fArray.FromNumpy(endpoints.astype(np.float32)))


def run_storm(
    app,
    world,
    a,
    c,
    read_state,
    animate,
    initial,
    points,
    faces,
    geometry,
    offsets,
    yaws,
    info,
):
    import omni.usd
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    count = len(offsets)
    paths = [f"/World/Leaves/L{i:03d}/Link{j}" for i in range(count) for j in range(3)]
    view = world.physics_sim_view.create_rigid_body_view(paths)
    order = list(view.prim_paths)
    mapping = np.array([order.index(p) for p in paths], dtype=np.int32)
    if c.hz % a.storm_load_hz or a.storm_load_hz > c.hz:
        raise ValueError("Storm load frequency must divide physics frequency")
    load_stride = c.hz // a.storm_load_hz
    active_indices = np.empty(0, dtype=np.int32)
    force_buffer = np.zeros((view.count, 3), np.float32)
    torque_buffer = np.zeros_like(force_buffer)
    model = RainImpulses(
        points,
        faces,
        geometry[0][0],
        offsets,
        yaws,
        c.fixed_length,
        c.length,
        a.rain_seed,
    )
    visual = RainVisual(world.stage, offsets, points, a.rain_seed)
    visual.set_rate(a.storm_rate)
    visual.update(0, 0)
    state = {
        "quit": False,
        "restart": False,
        "enabled": True,
        "gain": a.storm_gain,
        "rate": a.storm_rate,
    }
    trial_gain = a.storm_gain
    trial_rate = a.storm_rate
    label = None
    window = None
    if a.gui:
        from omni import ui

        window = ui.Window("Heavy rain on leaves", width=560, height=340)
        with window.frame, ui.VStack():
            ui.Label(f"{count} leaves | initial rate {a.storm_rate:g} mm/h | uniform exposure")
            ui.Label("Impulse model; no shielding, pooling or splashes")
            label = ui.Label("Settling...")
            ui.Button(
                "Repeat rain ×1",
                clicked_fn=lambda: state.update(
                    restart=True, gain=1.0, rate=a.storm_rate, enabled=True
                ),
            )
            ui.Button(
                "Stress ×3 (amplified load, not realistic rain)",
                clicked_fn=lambda: state.update(
                    restart=True, gain=3.0, rate=a.storm_rate, enabled=True
                ),
            )
            ui.Button(
                "Stress ×10 (amplified load, not realistic rain)",
                clicked_fn=lambda: state.update(
                    restart=True, gain=10.0, rate=a.storm_rate, enabled=True
                ),
            )
            ui.Button(
                "Deluge ×5 frequency (1000 mm/h, load ×1)",
                clicked_fn=lambda: state.update(
                    restart=True, gain=1.0, rate=1000.0, enabled=True
                ),
            )
            ui.Button("Stop rain", clicked_fn=lambda: state.update(enabled=False))
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    sim = get_physx_simulation_interface()
    stage_id = int(omni.usd.get_context().get_stage_id())
    root_ids = [
        PhysicsSchemaTools.sdfPathToInt(f"/World/Leaves/L{i:03d}/FixedPetiole")
        for i in range(count)
    ]
    base_step, n, trial = world.current_time_step_index, 0, 0
    start_at = 1 + c.settle
    stop_at = start_at + 28
    stride = max(1, round(c.hz / a.render_hz))
    poses = initial.copy()
    equilibrium = None
    max_motion = np.zeros(count)
    max_simultaneous = 0
    root_drift = 0.0
    records, clocks, frames, components = [], [], [], []
    block = 0.0
    wall_start = wall_end = None
    saved = False
    deadline = time.perf_counter()
    stopped_early = False

    def report(completed):
        nonlocal saved
        target = a.run_dir / f"storm-{trial:03d}"
        target.mkdir(exist_ok=True)
        sample = np.asarray(records)
        stamps = np.asarray(clocks)
        sampled_checks = []
        e = edges(faces)
        rest_edge = np.linalg.norm(points[e[:, 1]] - points[e[:, 0]], axis=1)
        rest_area = area(points, faces)
        if equilibrium is not None and len(sample) and not a.gui:
            for i, g in enumerate(geometry):
                centers, idx, weights, _ = g
                baseline = skin(
                    points,
                    centers,
                    idx,
                    weights,
                    equilibrium[i, :, :3],
                    equilibrium[i, :, 3:],
                )
                trace = np.array(
                    [
                        skin(points, centers, idx, weights, q[i, :, :3], q[i, :, 3:])
                        for q in sample
                    ]
                )
                recovery = float(
                    np.linalg.norm(
                        trace[stamps >= stamps[-1] - 1] - baseline, axis=2
                    ).max()
                )
                motion = float(
                    np.linalg.norm(trace[stamps >= start_at] - baseline, axis=2).max()
                )
                sampled_checks.append(
                    {
                        "leaf": i,
                        "hits": int(model.hits[i]),
                        "max_displacement_m": motion,
                        "recovery_m": recovery,
                        "max_edge_extension": float(
                            (
                                np.linalg.norm(
                                    trace[:, e[:, 1]] - trace[:, e[:, 0]], axis=2
                                )
                                / rest_edge
                            ).max()
                            - 1
                        ),
                        "min_area_ratio": min(
                            float((area(v, faces) / rest_area).min()) for v in trace
                        ),
                    }
                )
        checks = {
            "completed": completed,
            "finite_and_clock": True,
            "attachment": root_drift < 0.0002,
            "all_leaves_loaded": bool(np.all(model.hits > 0)) if trial_rate else True,
            "all_leaves_respond": bool(np.all(max_motion > 1e-5))
            if trial_rate
            else True,
            "sampled_structure": bool(sampled_checks)
            and all(
                r["max_edge_extension"] < 0.05 and r["min_area_ratio"] > 0.05
                for r in sampled_checks
            ),
            "sampled_recovery": bool(sampled_checks)
            and all(r["recovery_m"] < 0.002 for r in sampled_checks),
        }
        if a.gui:
            checks.pop("sampled_structure")
            checks.pop("sampled_recovery")
        wall = (wall_end or time.perf_counter()) - wall_start if wall_start else 0
        stats = summary(frames)
        result = {
            "status": "diagnostic"
            if not completed or all(checks.values())
            else "failed",
            "checks": checks,
            "interrupted": not completed,
            "stopped_early": stopped_early,
            "leaf_count": count,
            "rain_rate_mm_h": trial_rate,
            "force_gain": trial_gain,
            "exposure": "all leaves independently exposed; no canopy shielding",
            "drop_diameter_m": model.drop_diameter_m,
            "assumed_speed_m_s": model.speed_m_s,
            "drop_mass_kg": model.mass_kg,
            "expected_drops": model.expected_drops,
            "sampled_drops": model.total_drops,
            "total_applied_impulse_Ns": model.impulse_ns,
            "physics_hz": c.hz,
            "rain_load_hz": a.storm_load_hz,
            "force_reconstruction": "Piecewise constant over load interval; applied every physics step",
            "render_hz": a.render_hz,
            "rendered_fps": len(frames) / wall
            if wall and (a.render or a.gui)
            else None,
            "realtime_factor": 20 / wall
            if wall and completed and not stopped_early
            else None,
            "frame_work": stats,
            "components": {
                key: summary([r[k] for r in components])
                for k, key in enumerate(
                    ("load", "physics", "read", "skinning", "render")
                )
            },
            "all_leaf_max_segment_motion_m": max_motion.tolist(),
            "maximum_simultaneous_leaves_above_10um": max_simultaneous,
            "maximum_root_drift_m": root_drift,
            "leaves": sampled_checks,
            "surface_sampling_hz": 10,
            "surface_validation": "offline sampled surfaces"
            if not a.gui
            else "not evaluated in live GUI; use offscreen report",
            "acceptance": "Engineering load demonstration, not calibrated biomechanical or hydrodynamic validation",
            "visual_acceptance": "pending",
            "gui_paced": bool(a.gui and not a.gui_benchmark),
            "rigid_droplets": 0,
            "visual_streak_count": len(visual.xyz),
        }
        save(target / "report.json", result)
        save(a.run_dir / "report.json", result)
        np.savez_compressed(
            target / "trace.npz",
            times=stamps,
            leaf_poses=sample,
            equilibrium=equilibrium,
        )
        saved = True
        if a.render and not a.gui and completed and len(sample):
            import asyncio

            from omni.kit.viewport.utility import (
                capture_viewport_to_file,
                get_active_viewport,
            )

            world.pause()
            replay_time = start_at + 15
            animate(sample[np.argmin(abs(stamps - replay_time))])
            visual.update(replay_time, 1.0)
            for _ in range(5):
                app.update()
            capture = capture_viewport_to_file(
                get_active_viewport(), str(target / "peak_replay.png")
            )
            task = asyncio.ensure_future(capture.wait_for_result())
            for _ in range(300):
                app.update()
                if task.done():
                    break
            if not task.done():
                raise RuntimeError("Storm replay capture did not complete")
            task.result()
        print(
            "STORM_REPORT",
            {
                k: v
                for k, v in result.items()
                if k not in ("leaves", "all_leaf_max_segment_motion_m", "components")
            },
            flush=True,
        )

    info.update(
        scenario="water_rain_reduced_momentum",
        storm_rate_mm_h=a.storm_rate,
        storm_gain=a.storm_gain,
    )
    save(a.run_dir / "runtime.json", info)
    while app.is_running() and not state["quit"]:
        tick = time.perf_counter()
        t = n / c.hz
        if not world.is_playing():
            raise RuntimeError("Timeline stopped during storm")
        if n == c.hz:
            world.get_physics_context().set_gravity(-9.81)
            for root in root_ids:
                sim.wake_up(stage_id, root)
        if state.pop("restart", False):
            if not saved:
                report(False)
            trial += 1
            trial_gain = state["gain"]
            trial_rate = state["rate"]
            visual.set_rate(trial_rate)
            model = RainImpulses(
                points,
                faces,
                geometry[0][0],
                offsets,
                yaws,
                c.fixed_length,
                c.length,
                a.rain_seed + trial,
            )
            start_at, stop_at = t + c.settle, t + c.settle + 28
            equilibrium = None
            records, clocks, frames, components = [], [], [], []
            wall_start = wall_end = None
            max_motion[:] = 0
            max_simultaneous = 0
            root_drift = 0
            saved = False
            stopped_early = False
        if t >= start_at and equilibrium is None:
            equilibrium = poses.copy()
            wall_start = time.perf_counter()
        env = rain_envelope(t - start_at) if state["enabled"] else 0
        if not state["enabled"] and t < start_at + 20:
            stopped_early = True
        load_start = time.perf_counter()
        if env > 0 and trial_rate > 0:
            if n % load_stride == 0:
                forces, torques, hit_leaves = model.sample(
                    poses, trial_rate * env, 1 / a.storm_load_hz, trial_gain
                )
                for i in hit_leaves:
                    sim.wake_up(stage_id, root_ids[i])
                force_buffer[mapping] = forces.reshape(-1, 3)
                torque_buffer[mapping] = torques.reshape(-1, 3)
                active_indices = mapping[np.flatnonzero(forces[:, :, 2].reshape(-1))]
            if len(active_indices):
                view.apply_forces_and_torques_at_position(
                    force_buffer, torque_buffer, None, active_indices, True
                )
        else:
            active_indices = np.empty(0, dtype=np.int32)
        load_time = time.perf_counter() - load_start
        physics_start = time.perf_counter()
        n += 1
        world.step(render=False)
        physics_time = time.perf_counter() - physics_start
        if world.current_time_step_index != base_step + n:
            raise RuntimeError("Storm step count mismatch")
        read_start = time.perf_counter()
        poses, _, _ = read_state()
        if not np.isfinite(poses).all():
            raise RuntimeError("Nonfinite storm state")
        if not saved:
            root_drift = max(
                root_drift,
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
                max_simultaneous = max(
                    max_simultaneous,
                    int(
                        (
                            np.linalg.norm(
                                poses[:, 1:, :3] - equilibrium[:, 1:, :3], axis=2
                            ).max(axis=1)
                            > 1e-5
                        ).sum()
                    ),
                )
            if n % max(1, c.hz // 10) == 0:
                records.append(poses.copy())
                clocks.append(n / c.hz)
        read_time = time.perf_counter() - read_start
        skin_time = render_time = 0.0
        if n % stride == 0 and (a.render or a.gui):
            s = time.perf_counter()
            animate(poses)
            visual.update(t, env)
            if label:
                label.text = f"{trial_rate * env:.0f} mm/h | load ×{trial_gain:g} | leaves reached {(model.hits > 0).sum()}/{count}"
            skin_time = time.perf_counter() - s
            s = time.perf_counter()
            world.render()
            render_time = time.perf_counter() - s
        if start_at <= t < start_at + 20 and not saved:
            block += time.perf_counter() - tick
            components.append(
                (load_time, physics_time, read_time, skin_time, render_time)
            )
            if n % stride == 0:
                frames.append(block)
                block = 0
        if wall_start is not None and wall_end is None and n / c.hz >= start_at + 20:
            wall_end = time.perf_counter()
        if not saved and n / c.hz >= stop_at:
            report(True)
            if not a.gui or a.gui_benchmark:
                break
            deadline = time.perf_counter()
        if a.gui and not a.gui_benchmark:
            deadline += 1 / c.hz
            time.sleep(max(0, deadline - time.perf_counter()))
    if not saved:
        report(False)
    _ = window
