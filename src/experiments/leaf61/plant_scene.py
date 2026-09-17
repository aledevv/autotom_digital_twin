"""Shared-world fixed-plant smoke; imported after SimulationApp startup."""

import resource
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np
from plant_model import (
    layout,
    local_poses,
    summary,
    verify_layout,
    visual_crossing,
    yaw_matrix,
)
from scene import build, save

from model import (
    area,
    collider_mismatch,
    evaluate,
    first_contact_height,
    material_target,
    ramp,
    skin,
)


def run(app, a, c):
    import omni.kit.app
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.viewports import set_camera_view
    from omni.kit.viewport.utility import get_active_viewport
    from omni.physx import get_physx_simulation_interface
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, UsdGeom, UsdPhysics, Vt

    if c.hz != 120:
        raise ValueError("Plant comparison requires 120 Hz")
    initialization = time.perf_counter()
    data = np.load(a.run_dir / "input_mesh.npz")
    p, f = data["points"], data["faces"]
    canopy = a.layout == "canopy"
    placements = [] if canopy else layout(a.leaves, a.layout)
    count = len(placements)
    if a.layout == "plant":
        verify_layout(p, placements, c.thickness)
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1 / c.hz,
        rendering_dt=1 / 60,
        backend="numpy",
        device="cpu",
    )
    stage = world.stage
    if canopy:
        from canopy_fixture import build_canopy

        placements, paths, animations, geometry, info = build_canopy(world, a, c, p, f)
        count = len(placements)
        probe_path = "/World/Probe"
    else:
        paths, animations, geometry = [], [], []
        for i, (offset, yaw) in enumerate(placements):
            prefix = f"/World/Leaves/L{i:03d}"
            root = UsdGeom.Xform.Define(stage, prefix)
            # Only the first instance authors the shared probe/light/camera.
            _, _, anim, geo, info = build(
                world, c, "skinning", p, f, "real", prefix, i == 0
            )
            root.AddTranslateOp().Set(Gf.Vec3d(*offset))
            root.AddOrientOp().Set(
                Gf.Quatf(float(np.cos(yaw / 2)), Gf.Vec3f(0, 0, float(np.sin(yaw / 2))))
            )
            fixed = UsdPhysics.FixedJoint.Get(stage, prefix + "/FixedPetiole")
            fixed.CreateLocalPos0Attr().Set(
                Gf.Vec3f(*(geo[0][0] @ yaw_matrix(yaw).T + offset))
            )
            fixed.CreateLocalRot0Attr().Set(
                Gf.Quatf(float(np.cos(yaw / 2)), Gf.Vec3f(0, 0, float(np.sin(yaw / 2))))
            )
            paths += [prefix + "/Petiole"] + [prefix + f"/Link{j}" for j in range(3)]
            animations.append(anim)
            geometry.append(geo)
        probe_path = "/World/Leaves/L000/Probe"
        # The shared probe is under L000, whose transform is identity in all fixtures.
        assert np.allclose(placements[0][0], 0) and placements[0][1] == 0
    for i in range(count):
        other = []
        if a.layout in ("plant", "canopy") or a.pair_collisions == "off":
            other += [
                path
                for j in range(count)
                if j != i
                for path in paths[4 * j : 4 * j + 4]
            ]
        if a.layout == "contact-pair" and i == 1:
            other += [probe_path]
        for path in paths[4 * i : 4 * i + 4]:
            UsdPhysics.FilteredPairsAPI.Apply(
                stage.GetPrimAtPath(path)
            ).CreateFilteredPairsRel().SetTargets(other)

    # Static cylinders stop before the basal band: no initial support intersection.
    if a.layout == "plant":

        def bar(name, x, y, radius):
            axis = np.asarray(y) - x
            shape = UsdGeom.Cylinder.Define(stage, "/World/Structure/" + name)
            shape.CreateRadiusAttr().Set(radius)
            shape.CreateHeightAttr().Set(float(np.linalg.norm(axis)))
            shape.AddTranslateOp().Set(Gf.Vec3d(*((np.asarray(x) + y) / 2)))
            shape.AddOrientOp().Set(
                Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), Gf.Vec3d(*axis)).GetQuat())
            )
            shape.CreateDisplayColorAttr().Set([Gf.Vec3f(0.25, 0.32, 0.08)])
            UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
            # Broad-phase bound is sufficient: all lamina vertices lie at x >= -0.2 mm.
            if max(x[0], y[0]) + radius >= float(p[:, 0].min()):
                raise ValueError("Structure reaches lamina envelope")

        bar(
            "Stem",
            np.array([-0.03, -0.035, 0.05]),
            np.array([-0.03, -0.035, 0.40]),
            0.004,
        )
        for branch in range(3):
            z = c.height + branch * 0.12
            bar(
                f"Branch{branch}",
                np.array([-0.03, -0.035, z]),
                np.array([-0.03, 0.50, z]),
                0.002,
            )
        for i, (offset, _) in enumerate(placements):
            z = c.height + offset[2]
            bar(
                f"Petiole{i}",
                np.array([-0.03, offset[1], z]),
                np.array([-0.002, offset[1], z]),
                0.001,
            )

    set_camera_view(
        eye=np.array(
            info["canopy"]["camera_eye"]
            if canopy
            else ([0.70, -0.65, 0.75] if a.layout == "plant" else [0.16, -0.19, 0.25])
        ),
        target=np.array(
            info["canopy"]["camera_target"]
            if canopy
            else (
                [0.025, 0.23, 0.27] if a.layout == "plant" else [0.045, 0.0, c.height]
            )
        ),
        camera_prim_path="/World/Camera",
    )
    camera = UsdGeom.Camera.Get(stage, "/World/Camera")
    camera.CreateFocalLengthAttr().Set(35.0)
    camera.CreateHorizontalApertureAttr().Set(36.0)
    camera.CreateVerticalApertureAttr().Set(20.25)
    # Check the envelope of all 20 leaves, even for smaller benchmark subsets.
    view = np.asarray(
        UsdGeom.Xformable(camera).ComputeLocalToWorldTransform(0).GetInverse()
    )
    if canopy:
        low, high = np.asarray(info["canopy"]["bounds"])
        envelope = np.array(
            [
                [x, y, z]
                for x in (low[0], high[0])
                for y in (low[1], high[1])
                for z in (low[2], high[2])
            ]
        )
    else:
        envelope = np.concatenate(
            [
                p @ yaw_matrix(y).T + o
                for o, y in (layout(20) if a.layout == "plant" else placements)
            ]
        )
    projected = np.column_stack((envelope, np.ones(len(envelope)))) @ view
    depth = -projected[:, 2]
    framing = np.max(
        np.abs(projected[:, :2]) / (depth[:, None] * np.array([36.0, 20.25]) / 70.0)
    )
    if np.any(depth <= 0) or framing >= 0.95:
        raise RuntimeError(f"Plant does not fit benchmark camera: {framing}")
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = "/World/Camera"
        viewport.set_texture_resolution((1280, 720))
    contacts = []
    current_step = [0]

    def on_contact(headers, details):
        for header in headers:
            names = [
                str(PhysicsSchemaTools.intToSdfPath(v))
                for v in (header.actor0, header.actor1)
            ]
            if not (
                any("/L000/Link" in v for v in names)
                and any("/L001/Link" in v for v in names)
            ):
                continue
            for j in range(
                header.contact_data_offset,
                header.contact_data_offset + header.num_contact_data,
            ):
                d = details[j]
                contacts.append(
                    [
                        current_step[0] / c.hz,
                        float(d.separation),
                        float(np.linalg.norm(d.impulse)),
                    ]
                )

    subscription = None
    if a.layout == "contact-pair":
        for path in paths:
            PhysxSchema.PhysxContactReportAPI.Apply(
                stage.GetPrimAtPath(path)
            ).CreateThresholdAttr().Set(0.0)
        subscription = get_physx_simulation_interface().subscribe_contact_report_events(
            on_contact
        )
    links = (
        RigidPrim(paths, name="plant_links", reset_xform_properties=False)
        if count
        else None
    )
    world.reset()
    if links is not None:
        links.initialize()
    world.step(render=False)
    probe = world.physics_sim_view.create_rigid_body_view(probe_path)

    def read():
        if links is None:
            return np.empty((0, 4, 7))
        pos, quat = links.get_world_poses()
        out = np.empty((count, 4, 7))
        for i, (offset, yaw) in enumerate(placements):
            lp, lq = local_poses(
                pos[4 * i : 4 * i + 4], quat[4 * i : 4 * i + 4], offset, yaw
            )
            out[i, :, :3], out[i, :, 3:] = lp, lq
        return out

    def points(i, pose):
        centers, indices, weights, _ = geometry[i]
        return skin(p, centers, indices, weights, pose[:, :3], pose[:, 3:])

    poses = [read()]
    times = [0.0]
    probes = [np.asarray(probe.get_transforms())[0, :3].copy()]
    commands = [probes[0].copy()]
    initial = poses[0].copy()
    # Export a complete initial UsdSkel pose, including before the first render.
    for i, anim in enumerate(animations):
        anim.CreateTranslationsAttr().Set(Vt.Vec3fArray(initial[i, :, :3].tolist()))
        anim.CreateRotationsAttr().Set(
            Vt.QuatfArray(
                [
                    Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:])))
                    for q in initial[i, :, 3:]
                ]
            )
        )
    rest = [points(i, initial[i]) for i in range(count)]
    rest_area = area(p, f)
    save(
        a.run_dir / "layout.json",
        {
            "kind": a.layout,
            "count": count,
            "pair_collisions": a.pair_collisions,
            "instances": [
                {"id": i, "offset": o.tolist(), "yaw": y}
                for i, (o, y) in enumerate(placements)
            ],
        },
    )
    stage.GetRootLayer().Export(str(a.run_dir / "scene.usda"))
    manager = omni.kit.app.get_app().get_extension_manager()

    def gpu_info():
        return subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,memory.used",
                "--format=csv,noheader",
            ],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip()

    info.update(
        python=sys.executable,
        isaac_version=(Path.home() / "isaacsim-6.1/VERSION").read_text().strip(),
        gpu_start=gpu_info(),
        config=asdict(c),
        physx_extensions=[
            e["id"]
            for e in manager.get_extensions()
            if e["id"].startswith("omni.physx-")
        ],
        count=count,
        diagnostics=a.diagnostics,
        render_resolution=[1280, 720],
        camera={
            "focal_length": 35.0,
            "horizontal_aperture": 36.0,
            "vertical_aperture": 20.25,
            "max_normalized_extent": float(framing),
        },
        render_hz=60,
        dynamic_bodies=3 * count,
        elastic_joints=3 * count,
        dofs=5 * count,
        initialization_seconds=time.perf_counter() - initialization,
    )
    save(a.run_dir / "runtime.json", info)
    state = {"quit": False, "command": None, "selected": 0, "manual_used": False}
    window = None
    if a.gui:
        from omni import ui

        window = ui.Window(
            "Full plant 6.1" if canopy else "Small plant 6.1", width=410, height=275
        )
        with window.frame, ui.VStack():
            ui.Label(f"{count} leaf blades | physics 120 Hz | rendering 60 Hz")
            ui.Label("Kinematic sphere: slow press, no free fall")
            if a.gui_benchmark:
                ui.Label("Automatic benchmark: manual controls disabled")
            if a.layout in ("plant", "canopy") and count:
                selector = ui.ComboBox(
                    0,
                    *[f"Leaf {i + 1}" for i in range(count)],
                    enabled=not a.gui_benchmark,
                )
                selector.model.add_item_changed_fn(
                    lambda model, item: state.update(
                        selected=model.get_item_value_model().as_int
                    )
                )
            elif not count:
                ui.Label("Baseline: fully static plant")
            else:
                ui.Label("The sphere presses only the upper leaf")
            for label, command in (
                ("Press", "press"),
                ("Release", "release"),
                ("Repeat test", "repeat"),
            ):
                ui.Button(
                    label,
                    clicked_fn=lambda cmd=command: state.update(command=cmd),
                    enabled=not a.gui_benchmark and count > 0,
                )
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    total_cycles = 6 if a.scenario == "cycle" else 1
    begin = 1 + c.settle
    limit = begin + total_cycles * (5 + c.recovery)
    active = None
    cycles = []
    number = 0
    manual = False
    release = None
    parking = None
    latest = poses[0]
    center = probes[0].copy()
    n = frames = 0
    component = {
        key: [] for key in ("physics", "read", "skinning", "diagnostics", "render")
    }
    frame_work, frame_wall = [], []
    block_work = 0.0
    baseline_step = world.current_time_step_index
    baseline_clock = float(world.current_time)
    start = time.perf_counter()
    block_start = start
    measured_wall_start = None
    finite = True
    light_geometry_ok = True
    tracking_error = 0.0
    while app.is_running() and not state["quit"]:
        t = n / c.hz
        if not world.is_playing():
            raise RuntimeError("Timeline stopped before completion")
        if t >= limit - 1e-9 and (not a.gui or a.gui_benchmark):
            break
        tick_step = time.perf_counter()
        if n == c.hz:
            world.get_physics_context().set_gravity(-9.81)
        if n == int((1 + c.settle) * c.hz):
            measured_wall_start = time.perf_counter()
        command = state.pop("command", None)
        if command == "repeat":
            release = None
            parking = (t, center.copy())
            active = None
            manual = False
            number = 0
            begin = t + 2 + c.settle
            limit = begin + total_cycles * (5 + c.recovery)
        if command == "press":
            state["manual_used"] = True
            manual = True
            release = None
            if active is not None:
                active["start"] = t
            if active is None:
                begin = t
                number = 0
        if command == "release" and active:
            state["manual_used"] = True
            manual = True
            release = (t, float(center[2]))
        if (
            count
            and active is None
            and t >= begin
            and number < total_cycles
            and parking is None
        ):
            selected = state["selected"] if a.layout in ("plant", "canopy") else 0
            curr = points(selected, latest[selected])
            target = material_target(
                rest[selected],
                curr,
                f,
                c.fixed_length + 0.65 * (c.length - c.fixed_length),
                0.0 if number < 3 else 0.01,
            )
            z = first_contact_height(curr, f, *target[:2], c.radius)
            offset, yaw = placements[selected]
            target = target @ yaw_matrix(yaw).T + offset
            active = {
                "start": t,
                "end": t + 5 + c.recovery,
                "location": "central" if number < 3 else "offcenter",
                "baseline_index": len(times) - 1,
                "leaf": selected,
                "x": float(target[0]),
                "y": float(target[1]),
                "z": z + offset[2],
            }
            number += 1
        if active:
            dt = t - active["start"]
            fraction = (
                ramp(dt / 2) if dt < 2 else (1.0 if dt < 3 else 1 - ramp((dt - 3) / 2))
            )
            if manual:
                fraction = ramp(dt / 2)
            z = active["z"] + 0.004 - fraction * (c.depth + 0.004)
            if release:
                z = release[1] + ramp((t - release[0]) / 2) * (
                    active["z"] + 0.004 - release[1]
                )
            center = np.array([active["x"], active["y"], z])
            if dt >= 5 + c.recovery - 1 / c.hz - 1e-8 and not manual:
                cycles.append(active)
                active = None
        if parking:
            pt, origin = parking
            center = origin + np.array([0, 0, 0.08]) * ramp((t - pt) / 2)
            if t - pt >= 2:
                parking = None
        probe.set_kinematic_targets(
            np.array([[*center, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32),
            np.array([0], dtype=np.int32),
        )
        current_step[0] = n + 1
        tick = time.perf_counter()
        world.step(render=False)
        phys = time.perf_counter() - tick
        if world.current_time_step_index != baseline_step + n + 1:
            raise RuntimeError("Physics step count mismatch")
        tick = time.perf_counter()
        latest = read()
        actual = np.asarray(probe.get_transforms())[0, :3].copy()
        read_time = time.perf_counter() - tick
        tracking_error = max(tracking_error, float(np.linalg.norm(actual - center)))
        finite = finite and bool(np.isfinite(latest).all())
        if not finite:
            raise RuntimeError("Nonfinite physical poses")
        n += 1
        sampled = a.diagnostics == "full" or n % 12 == 0
        diagnostic_time = 0.0
        if sampled:
            tick = time.perf_counter()
            poses.append(latest.copy())
            times.append(n / c.hz)
            probes.append(actual)
            commands.append(center.copy())
            if a.diagnostics == "light":
                for i in range(count):
                    pts = points(i, latest[i])
                    light_geometry_ok &= bool(
                        np.isfinite(pts).all()
                        and np.min(area(pts, f) / rest_area) > 0.05
                    )
            diagnostic_time = time.perf_counter() - tick
        skel_time = render_time = 0.0
        if n % 2 == 0 and (a.gui or a.render):
            tick = time.perf_counter()
            for i, anim in enumerate(animations):
                anim.CreateTranslationsAttr().Set(
                    Vt.Vec3fArray(latest[i, :, :3].tolist())
                )
                anim.CreateRotationsAttr().Set(
                    Vt.QuatfArray(
                        [
                            Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:])))
                            for q in latest[i, :, 3:]
                        ]
                    )
                )
            skel_time = time.perf_counter() - tick
            tick = time.perf_counter()
            world.render()
            render_time = time.perf_counter() - tick
            frames += 1
        work = time.perf_counter() - tick_step
        block_work += work
        if t >= 1 + c.settle:
            for key, value in zip(
                component, (phys, read_time, skel_time, diagnostic_time, render_time)
            ):
                component[key].append(value)
        if n % 2 == 0:
            if t >= 1 + c.settle:
                frame_work.append(block_work)
                frame_wall.append(time.perf_counter() - block_start)
            block_work = 0.0
            block_start = time.perf_counter()
        # GUI is wall-clock paced; frame-work metrics exclude this deliberate sleep.
        if a.gui and not a.gui_benchmark:
            time.sleep(max(0.0, start + n / c.hz - time.perf_counter()))
        if not light_geometry_ok:
            raise RuntimeError("Geometry collapsed during light diagnostics")
    loop_seconds = time.perf_counter() - start
    measurement_wall = (
        time.perf_counter() - measured_wall_start if measured_wall_start else 0.0
    )
    elapsed = float(world.current_time) - baseline_clock
    if not np.isclose(elapsed, n / c.hz, rtol=5e-5, atol=5e-5):
        raise RuntimeError("Physics clock mismatch")
    simulation_peak_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024
    report_start = time.perf_counter()
    poses = np.asarray(poses)
    times = np.asarray(times)
    probes = np.asarray(probes)
    leaf_reports = []
    pair_surfaces = []
    equilibrium_index = int(np.argmin(abs(times - (1 + c.settle))))
    for i in range(count):
        trace = np.asarray([points(i, v[i]) for v in poses])
        own = [cy for cy in cycles if cy["leaf"] == i]
        offset, yaw = placements[i]
        local_probe = (probes - offset) @ yaw_matrix(yaw)
        result = evaluate(
            c,
            rest[i],
            f,
            trace,
            times,
            local_probe,
            own,
            complete=n / c.hz >= limit - 1e-8,
        )
        if not own:
            result["checks"].pop("cycles")
        after = times >= 1 + c.settle
        deviation = (
            float(np.linalg.norm(trace[after] - trace[equilibrium_index], axis=2).max())
            if after.any()
            else None
        )
        result["metrics"]["unpressed_max_deviation_m"] = deviation
        if not own and a.layout in ("plant", "canopy"):
            result["checks"]["independence"] = (
                deviation is not None and deviation < 0.0005
            )
        if a.layout == "contact-pair":
            final = times >= limit - 1 - 1e-8
            recovery = (
                float(
                    np.linalg.norm(
                        trace[final] - trace[equilibrium_index], axis=2
                    ).max()
                )
                if final.any()
                else None
            )
            residual = (
                float(np.linalg.norm(np.ptp(trace[final], axis=0), axis=1).max())
                if final.any()
                else None
            )
            result["metrics"].update(pair_recovery_m=recovery, pair_residual_m=residual)
            result["checks"]["pair_recovery"] = (
                recovery is not None and recovery < 0.002 and residual < 0.0005
            )
            pair_surfaces.append(
                trace[
                    :: max(
                        1, round(1 / np.median(np.diff(times))) if len(times) > 1 else 1
                    )
                ]
                @ yaw_matrix(yaw).T
                + offset
            )
        sample_ids = np.flatnonzero(np.isclose((times * 10) % 1, 0, atol=1e-6))
        result["metrics"]["visual_collider_sampled_max_gap_m"] = max(
            collider_mismatch(
                c, trace[k], poses[k, i, :, :3], poses[k, i, :, 3:], geometry[i][3]
            )
            for k in sample_ids
        )
        result["status"] = "passed" if all(result["checks"].values()) else "failed"
        result["leaf"] = i
        leaf_reports.append(result)
        if count == 1:
            np.savez_compressed(
                a.run_dir / "regression_trace.npz", points=trace, times=times
            )
        del trace
    contact_array = np.asarray(contacts, dtype=float).reshape(-1, 3)
    pair_report = None
    if a.layout == "contact-pair":
        pre = contact_array[contact_array[:, 0] <= 1 + c.settle]
        during = contact_array[contact_array[:, 0] > 1 + c.settle]
        crossing = max(visual_crossing(up, lo, f) for up, lo in zip(*pair_surfaces))
        pair_report = {
            "initial_valid": bool(not len(pre) or pre[:, 1].min() >= 0),
            "contact_samples": len(during),
            "max_impulse": float(during[:, 2].max()) if len(during) else 0.0,
            "max_collider_penetration_m": max(0.0, float(-contact_array[:, 1].min()))
            if len(contact_array)
            else 0.0,
            "visual_crossing_sampled_1hz_m": crossing,
            "control_comparison": "pending",
        }
        pair_report["passed_local"] = bool(
            pair_report["initial_valid"]
            and pair_report["max_collider_penetration_m"] < 0.001
            and (a.pair_collisions == "off" or pair_report["max_impulse"] > 0)
        )
    measured_sim = max(0.0, n / c.hz - (1 + c.settle))
    work_stats = summary(frame_work)
    timing = {
        "initialization_seconds": info["initialization_seconds"],
        "loop_seconds": loop_seconds,
        "simulated_seconds": n / c.hz,
        "measurement_simulated_seconds": measured_sim,
        "measurement_wall_seconds": measurement_wall,
        "realtime_factor": measured_sim / max(measurement_wall, 1e-9),
        "frame_work": work_stats,
        "frame_wall": summary(frame_wall),
        "components": {k: summary(v) for k, v in component.items()},
        "rendered_frames": frames,
        "gui_fps": (len(frame_work) / max(measurement_wall, 1e-9)) if a.gui else None,
        "physics_clock_elapsed": elapsed,
    }
    timing["performance_passed"] = bool(
        measured_sim
        and timing["realtime_factor"] >= 1
        and work_stats["p95_seconds"] is not None
        and work_stats["p95_seconds"] <= 1 / 60
    )
    report = {
        "status": "passed"
        if all(r["status"] == "passed" for r in leaf_reports)
        and (pair_report is None or pair_report["passed_local"])
        and tracking_error < 1e-4
        else "failed",
        "leaves": leaf_reports,
        "pair": pair_report,
        "timing": timing,
        "probe_tracking_error_m": tracking_error,
        "visual_acceptance": "pending",
        "diagnostics": a.diagnostics,
        "count": count,
        "geometry_sampling_hz": 120 if a.diagnostics == "full" else 10,
        "memory": {
            "simulation_peak_rss_bytes": simulation_peak_rss,
            "peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * 1024,
            "gpu_end": gpu_info(),
        },
        "manual_used": state["manual_used"],
    }
    if state["manual_used"]:
        report["status"] = (
            "manual"
            if all(
                all(
                    leaf["checks"].get(key, False)
                    for key in ("finite", "attachment", "stretch", "noncollapsed")
                )
                for leaf in leaf_reports
            )
            and tracking_error < 1e-4
            else "failed"
        )
    np.savez_compressed(
        a.run_dir / "trace.npz",
        times=times,
        rigid_poses=poses,
        probe=probes,
        probe_commands=commands,
        contacts=contact_array,
    )
    np.savez_compressed(a.run_dir / "rest_mesh.npz", points=p, faces=f)
    np.savez_compressed(
        a.run_dir / "timings.npz",
        frame_work=frame_work,
        frame_wall=frame_wall,
        **component,
    )
    report["offline_analysis_and_write_seconds"] = time.perf_counter() - report_start
    save(a.run_dir / "report.json", report)
    print(
        "PLANT_REPORT", report["status"], count, timing["realtime_factor"], flush=True
    )
    if a.render or a.gui_benchmark:
        import asyncio

        from omni.kit.viewport.utility import capture_viewport_to_file

        world.pause()
        for _ in range(5):
            app.update()
        capture = capture_viewport_to_file(
            get_active_viewport(), str(a.run_dir / "final.png")
        )
        task = asyncio.ensure_future(capture.wait_for_result())
        for _ in range(300):
            app.update()
            if task.done():
                break
        if not task.done():
            raise RuntimeError("Final viewport capture did not complete")
        task.result()
    # Keep the subscription/window alive until after all measurements.
    _ = subscription, window
