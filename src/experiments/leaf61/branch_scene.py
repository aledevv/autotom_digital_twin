"""Isolated coupled branch / skinned leaf diagnostic; run with branch_run.py."""

import argparse
import json
import time
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--fixed", action="store_true")
    parser.add_argument("--tomato", action="store_true")
    parser.add_argument(
        "--runtime", choices=["baseline", "optimized"], default="optimized"
    )
    parser.add_argument("--branches", type=int, choices=[1, 2, 3, 5, 10], default=1)
    parser.add_argument("--shared-stem", action="store_true")
    parser.add_argument("--fixed-stem", action="store_true")
    a = parser.parse_args()
    if a.shared_stem and (
        a.branches != 3 or not a.tomato or a.fixed or a.runtime != "optimized"
    ):
        parser.error(
            "Shared stem requires three elastic branches, tomato mode and optimized runtime"
        )
    if a.fixed_stem and not a.shared_stem:
        parser.error("--fixed-stem requires --shared-stem")
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": not a.gui,
            "width": 1280,
            "height": 720,
            "renderer": "RaytracedLighting",
            "fast_shutdown": True,
        }
    )
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, Vt
    from scene import build, save

    from model import Config, area, rotation, skin

    omni.usd.get_context().new_stage()
    c = Config(hz=480, iterations=32)
    data = np.load(a.run_dir / "input_mesh.npz")
    p, f = data["points"], data["faces"]
    world = World(
        stage_units_in_meters=1,
        physics_dt=1 / c.hz,
        rendering_dt=1 / 30,
        backend="numpy",
        device="cpu",
    )
    stage = world.stage
    offsets = np.array([[0, y, 0.05] for y in (0.06, 0.14, 0.22)])
    paths, animations, geometries = [], [], []
    for i, offset in enumerate(offsets):
        prefix = f"/World/Leaves/L{i:03d}"
        root = UsdGeom.Xform.Define(stage, prefix)
        _, _, anim, geo, _ = build(
            world, c, "skinning", p, f, "real", prefix, i == 0, create_view=False
        )
        root.AddTranslateOp().Set(Gf.Vec3d(*offset))
        stage.RemovePrim(prefix + "/Probe")
        joint = UsdPhysics.FixedJoint.Get(stage, prefix + "/FixedPetiole")
        joint.GetPrim().RemoveAPI(UsdPhysics.ArticulationRootAPI)
        joint.GetPrim().RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        joint.CreateBody0Rel().SetTargets(["/World/Branch"])
        joint.CreateLocalPos0Attr().Set(
            Gf.Vec3f(*(geo[0][0] + offset - [0, 0.14, 0.20]))
        )
        joint.CreateCollisionEnabledAttr().Set(False)
        paths += [prefix + "/Petiole"] + [prefix + f"/Link{j}" for j in range(3)]
        animations.append(anim)
        geometries.append(geo)

    branch = UsdGeom.Xform.Define(stage, "/World/Branch")
    collider = UsdGeom.Cube.Define(stage, "/World/Branch/Collider")
    collider.CreateSizeAttr().Set(1)
    branch.AddTranslateOp().Set(Gf.Vec3d(0, 0.14, 0.20))
    collider.AddScaleOp().Set(Gf.Vec3f(0.008, 0.28, 0.008))
    collider.CreateDisplayColorAttr().Set([Gf.Vec3f(0.28, 0.43, 0.12)])
    UsdPhysics.RigidBodyAPI.Apply(branch.GetPrim())
    UsdPhysics.MassAPI.Apply(branch.GetPrim()).CreateMassAttr().Set(0.025)
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    anchor = UsdGeom.Xform.Define(stage, "/World/Anchor")
    anchor.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.20))
    UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(anchor.GetPrim())
    mass.CreateMassAttr().Set(0.1)
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.0001))
    fixed_root = UsdPhysics.FixedJoint.Define(stage, "/World/FixedRoot")
    fixed_root.CreateBody1Rel().SetTargets([anchor.GetPath()])
    fixed_root.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0.20))
    if a.fixed:
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/BranchRoot")
    else:
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/BranchRoot")
        joint.CreateAxisAttr().Set("X")
        joint.CreateLowerLimitAttr().Set(-40)
        joint.CreateUpperLimitAttr().Set(40)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(0.4 * np.pi / 180)
        drive.CreateDampingAttr().Set(0.04 * np.pi / 180)
        drive.CreateTargetPositionAttr().Set(0)
    joint.CreateBody0Rel().SetTargets([anchor.GetPath()])
    joint.CreateBody1Rel().SetTargets([branch.GetPath()])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, -0.14, 0))
    UsdPhysics.ArticulationRootAPI.Apply(fixed_root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(fixed_root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSolverPositionIterationCountAttr().Set(32)
    art.CreateSolverVelocityIterationCountAttr().Set(4)
    art.CreateSleepThresholdAttr().Set(0)
    support = UsdGeom.Cube.Define(stage, "/World/Support")
    support.CreateSizeAttr().Set(1)
    support.AddTranslateOp().Set(Gf.Vec3d(0, -0.008, 0.10))
    support.AddScaleOp().Set(Gf.Vec3f(0.018, 0.018, 0.20))
    support.CreateDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.3, 0.3)])
    set_camera_view(
        eye=np.array([0.48, -0.35, 0.42]),
        target=np.array([0.045, 0.13, 0.17]),
        camera_prim_path="/World/Camera",
    )
    from branch_layout import attachment_error, isolate, replicate

    prefixes, shifts, offsets = replicate(
        stage,
        a.branches,
        paths,
        animations,
        geometries,
        offsets,
        shifts=np.array([[0.0, 0, 0.12 * i] for i in range(3)])
        if a.shared_stem
        else None,
    )
    stem_local_roots = None
    if a.shared_stem:
        from shared_stem import attachment_frames, connect

        stem_local_roots = connect(stage, prefixes, shifts, a.fixed_stem)
    target_branches = [2] if a.shared_stem else list(range(a.branches))
    leaf_count = 3 * a.branches
    if a.branches > 1:
        centre = shifts.mean(0) + [0.04, 0.14, 0.17]
        span = max(np.ptp(shifts[:, 0]) + 0.3, np.ptp(shifts[:, 1]) + 0.4)
        set_camera_view(
            eye=centre + np.array([span, -span, span * 0.85]),
            target=centre,
            camera_prim_path="/World/Camera",
        )
    if a.shared_stem:
        set_camera_view(
            eye=np.array([0.8, -0.9, 0.65]),
            target=np.array([0.03, 0.13, 0.24]),
            camera_prim_path="/World/Camera",
        )
    tomato = None
    tomatoes = []
    if a.tomato:
        from branch_tomato import BranchTomato

        tomatoes = [BranchTomato(stage, prefixes[i]) for i in target_branches]
        if a.shared_stem:
            floor = stage.GetPrimAtPath(prefixes[2] + "/Floor")
            floor.GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(0.1, 0.1, -0.015 - shifts[2, 2])
            )
        tomato = tomatoes[0]
        world.get_physics_context().enable_ccd(True)
    if a.branches > 1:
        isolate(stage, prefixes)
    optimized = a.runtime == "optimized"
    from omni.physx import get_physx_interface

    if optimized:
        import carb

        settings = carb.settings.get_settings()
        settings.set_bool("/physics/updateToUsd", False)
        settings.set_bool("/physics/updateVelocitiesToUsd", False)
    world.reset()
    mouse_settings = None
    if a.gui:
        from mouse_grab import configure

        mouse_settings = configure(app)
        save(a.run_dir / "mouse_interaction.json", mouse_settings)
    for ball in tomatoes:
        ball.initialize(world)
    view = world.physics_sim_view.create_rigid_body_view(
        paths
        + [prefix + "/Branch" for prefix in prefixes]
        + [ball.path for ball in tomatoes]
        + (["/World/Stem"] if a.shared_stem else [])
    )
    order = list(view.prim_paths)
    leaf_ids = np.array([order.index(x) for x in paths])
    branch_ids = np.array([order.index(prefix + "/Branch") for prefix in prefixes])
    tomato_ids = [order.index(ball.path) for ball in tomatoes]
    stem_id = order.index("/World/Stem") if a.shared_stem else None
    animation_channels = [
        (anim.CreateTranslationsAttr(), anim.CreateRotationsAttr())
        for anim in animations
    ]
    attachment_offsets = np.array(
        [
            geo[0][0] + offset - shifts[i // 3] - [0, 0.14, 0.20]
            for i, (geo, offset) in enumerate(zip(geometries, offsets))
        ]
    )
    tips = np.array(
        [order.index(paths[4 * i + 3]) for i in range(leaf_count)], dtype=np.int32
    )
    force = np.zeros((view.count, 3), np.float32)
    world.step(render=False)
    stage.GetRootLayer().Export(str(a.run_dir / "scene.usda"))
    save(
        a.run_dir / "config.json",
        {
            "fixed_branch": a.fixed,
            "mode": "tomato" if tomato else "controlled_force",
            "tomato_mass_kg": 0.02 if tomato else None,
            "tomato_radius_m": 0.02 if tomato else None,
            "tomato_clearance_m": 0.06 if tomato else None,
            "runtime": a.runtime,
            "pose_paths": order,
            "branches": a.branches,
            "shared_stem": a.shared_stem,
            "fixed_stem": a.fixed_stem,
            "tomato_target_branches": target_branches,
            "stem_parameters": {
                "length_m": 0.5,
                "mass_kg": 0.06,
                "stiffness_Nm_rad": 3.0,
                "damping_Nms_rad": 0.25,
            }
            if a.shared_stem
            else None,
            "branch_shifts_m": shifts.tolist(),
            "physics_hz": 480,
            "render_hz": 30,
            "branch_mass_kg": 0.025,
            "branch_stiffness_Nm_rad": 0.4,
            "branch_damping_Nms_rad": 0.04,
            "leaf_joint_stiffness_Nm_rad": c.joint_stiffness,
            "leaf_mass_kg": float(area(p, f).sum() * c.thickness * c.density),
            "petiole_mass_kg": 0.01,
            "tip_load_N_per_leaf": 0 if tomato else 0.03,
            "scope": "Three branches on a shared hinged stem"
            if a.shared_stem
            else "Independent hinged branches with three skinned leaves each; not full plant integration",
        },
    )
    state = {"quit": False, "repeat": False, "release": False}
    window = None
    if a.gui:
        from omni import ui

        window = ui.Window(
            "Shared stem + branches + leaves"
            if a.shared_stem
            else "Moving branch + skinned leaves",
            width=520,
            height=280,
        )
        with window.frame, ui.VStack():
            ui.Label(f"{a.branches} branches | {leaf_count} leaves | physics 480 Hz")
            ui.Label(
                "Tomato: 20 g, radius 20 mm, free fall"
                if tomato
                else "Downward force: 0.03 N per leaf tip; no water model"
            )
            if a.shared_stem:
                ui.Label("Tomato contacts: upper branch only (coupling test)")
            ui.Label("Shift + left-drag: pull stem, branches or leaves")
            label = ui.Label("Settling...")
            ui.Button(
                (
                    "Drop tomatoes / Repeat test"
                    if len(tomatoes) > 1
                    else "Drop tomato / Repeat test"
                )
                if tomato
                else "Load leaves / Repeat test",
                clicked_fn=lambda: state.update(repeat=True),
            )
            ui.Button(
                ("Remove tomatoes" if len(tomatoes) > 1 else "Remove tomato")
                if tomato
                else "Release load",
                clicked_fn=lambda: state.update(release=True),
            )
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    base = world.current_time_step_index
    load_at = 9.0
    stop_at = 22.0
    samples, stamps, frames = [], [], []
    max_error = 0.0
    equilibrium = None
    saved = False
    trial = 0

    component_names = ("physics", "read", "diagnostics", "skinning", "render")
    component_block = np.zeros(5)
    component_frames = []
    work_frames = []
    work_block = 0.0
    performance_start = performance_end = None

    def record(completed):
        values = np.asarray(samples)
        if not len(values):
            return
        eq = equilibrium if equilibrium is not None else values[0]
        loaded = values[np.asarray(stamps) >= load_at]
        if not len(loaded):
            loaded = values[-1:]
        branch_motions = np.linalg.norm(
            loaded[:, branch_ids, :3] - eq[branch_ids, :3], axis=2
        ).max(axis=0)
        branch_motion = float(branch_motions.max())
        recovery = float(
            np.linalg.norm(values[-1, leaf_ids, :3] - eq[leaf_ids, :3], axis=1).max()
        )
        bending = []
        for i in range(leaf_count):
            root_id, tip_id = leaf_ids[4 * i], leaf_ids[4 * i + 3]
            local = np.einsum(
                "nji,nj->ni",
                rotation(loaded[:, root_id][:, [6, 3, 4, 5]]),
                loaded[:, tip_id, :3] - loaded[:, root_id, :3],
            )
            reference = rotation(eq[root_id][None, [6, 3, 4, 5]])[0].T @ (
                eq[tip_id, :3] - eq[root_id, :3]
            )
            bending.append(float(np.linalg.norm(local - reference, axis=1).max()))
        residual = values[np.asarray(stamps) >= max(stamps) - 1]
        oscillation = float(
            np.linalg.norm(np.ptp(residual[:, leaf_ids, :3], axis=0), axis=1).max()
        )
        rest_area = area(p, f)
        edges = np.unique(
            np.sort(np.concatenate([f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]]]), axis=1),
            axis=0,
        )
        lengths = np.linalg.norm(p[edges[:, 0]] - p[edges[:, 1]], axis=1)
        extension, min_area = 0.0, 1.0
        for poses in values:
            for i, (centers, indices, weights, _) in enumerate(geometries):
                v = poses[leaf_ids[4 * i : 4 * i + 4]]
                mesh = skin(
                    p,
                    centers,
                    indices,
                    weights,
                    v[:, :3] - offsets[i],
                    v[:, [6, 3, 4, 5]],
                )
                extension = max(
                    extension,
                    float(
                        (
                            np.linalg.norm(
                                mesh[edges[:, 0]] - mesh[edges[:, 1]], axis=1
                            )
                            / lengths
                            - 1
                        ).max()
                    ),
                )
                min_area = min(min_area, float((area(mesh, f) / rest_area).min()))
        checks = {
            "completed": completed,
            "finite": bool(np.isfinite(values).all()),
            "attachment": max_error < 0.0002,
            "sampled_stretch": extension < 0.05,
            "sampled_area": min_area > 0.05,
            "recovery": recovery < 0.002,
            "residual_oscillation": oscillation < 0.0005,
            "lamina_response": (
                all(bending[3 * i + 2] > 0.0001 for i in target_branches)
                if tomato
                else all(v > 0.0001 for v in bending)
            )
            if completed
            else False,
            "branch_response": bool(np.all(branch_motions[target_branches] > 0.0001))
            if not a.fixed and completed
            else True,
        }
        stem_motion = None
        if a.shared_stem:
            stem_motion = float(
                np.linalg.norm(loaded[:, stem_id, :3] - eq[stem_id, :3], axis=1).max()
            )
            checks["shared_stem_response"] = (
                stem_motion < 0.00001 if a.fixed_stem else stem_motion > 0.0001
            )
            if not a.fixed_stem:
                checks["passive_branches_respond"] = bool(
                    np.all(branch_motions[:2] > 0.0001)
                )
            else:
                checks["passive_branches_quiet"] = bool(
                    np.all(branch_motions[:2] < 0.0001)
                )
        if a.shared_stem:
            checks["isolated_load_path"] = all(
                all(
                    e["leaf_body"].startswith(ball.prefix + "/Leaves/")
                    for e in ball.events
                )
                and all(
                    actor == ball.prefix + "/Floor" for actor in ball.other_contacts
                )
                for ball in tomatoes
            )
        if tomato:
            checks.update(
                tomato_fell=all(ball.fall_distance > 0.04 for ball in tomatoes),
                eight_seconds_after_contact=all(
                    bool(ball.events)
                    and max(stamps) - max(e["time_s"] for e in ball.events) >= 8
                    for ball in tomatoes
                ),
                target_contact=all(
                    any(
                        e["leaf_body"].startswith(ball.prefix + "/Leaves/L002/")
                        for e in ball.events
                    )
                    for ball in tomatoes
                ),
                contact_penetration=all(
                    ball.summary()["maximum_contact_penetration_m"] < 0.001
                    for ball in tomatoes
                ),
            )
            save(
                a.run_dir / f"tomatoes-{trial:03d}.json",
                [ball.summary() for ball in tomatoes],
            )
            np.savez_compressed(
                a.run_dir / f"tomato-traces-{trial:03d}.npz",
                **{
                    f"branch{i}": np.array(ball.traces)
                    for i, ball in enumerate(tomatoes)
                },
            )
            save(a.run_dir / f"tomato-{trial:03d}.json", tomato.summary())
            np.save(
                a.run_dir / f"tomato-trace-{trial:03d}.npy", np.array(tomato.traces)
            )
        save(
            a.run_dir / f"report-{trial:03d}.json",
            {
                "checks": checks,
                "runtime": a.runtime,
                "gui_paced": a.gui,
                "native_mouse_grab": mouse_settings,
                "manual_input_tracking": "not recorded" if a.gui else "headless",
                "performance_window": "load start to trial end, excluding initialization and settling",
                "performance": {
                    "frames": len(work_frames),
                    "rendered_fps": len(work_frames)
                    / (performance_end - performance_start)
                    if performance_start and performance_end
                    else None,
                    "work_p95_ms": float(np.percentile(work_frames, 95) * 1000)
                    if work_frames
                    else None,
                    "components_ms_per_frame": {
                        name: {
                            "median": float(
                                np.median(np.array(component_frames)[:, j]) * 1000
                            ),
                            "p95": float(
                                np.percentile(np.array(component_frames)[:, j], 95)
                                * 1000
                            ),
                        }
                        for j, name in enumerate(component_names)
                    }
                    if component_frames
                    else {},
                },
                "branch_max_motion_from_equilibrium_m": branch_motion,
                "branch_motions_m": branch_motions.tolist(),
                "stem_motion_m": stem_motion,
                "shared_stem": a.shared_stem,
                "fixed_stem": a.fixed_stem,
                "branches": a.branches,
                "leaf_recovery_error_m": recovery,
                "lamina_tip_motion_in_petiole_frame_m": bending,
                "residual_oscillation_m": oscillation,
                "attachment_max_error_m": max_error,
                "max_edge_extension": extension,
                "min_area_ratio": min_area,
                "frame_work_p95_ms": float(np.percentile(frames, 95) * 1000)
                if frames
                else None,
                "geometry_sample_hz": 10,
                "fixed_branch": a.fixed,
                "visual_acceptance": "pending",
                "completed": completed,
            },
        )
        np.savez_compressed(
            a.run_dir / f"trace-{trial:03d}.npz",
            poses=values,
            times=stamps,
            equilibrium=eq,
        )
        print("BRANCH_REPORT", json.dumps(checks), flush=True)

    frame_start = time.perf_counter()
    n = 0
    pace_origin = time.perf_counter()
    while app.is_running() and not state["quit"]:
        tick = time.perf_counter()
        t = n / c.hz
        if n == c.hz:
            world.get_physics_context().set_gravity(-9.81)
        if state.pop("repeat", False):
            if not saved:
                record(False)
            trial += 1
            samples, stamps, frames = [], [], []
            component_frames, work_frames = [], []
            performance_start = performance_end = None
            component_block[:] = 0
            work_block = 0.0
            max_error = 0.0
            equilibrium = None
            saved = False
            load_at = t + 8
            stop_at = load_at + 13
            state["release"] = False
            for ball in tomatoes:
                ball.remove()
                ball.reset_stats()
        if performance_start is None and t >= load_at:
            performance_start = time.perf_counter()
            component_block[:] = 0
            work_block = 0.0
        phase = t - load_at
        envelope = max(0, min(1, phase, 5 - phase)) if not state["release"] else 0.0
        for i, ball in enumerate(tomatoes):
            ball.time = t
            ball.recording = not saved
            if state["release"] and not ball.removed:
                ball.remove()
            if not ball.released and t >= load_at and not state["release"]:
                before = np.asarray(view.get_transforms()).copy()
                if i == 0:
                    equilibrium = before.copy()
                ball.drop(before[leaf_ids[12 * target_branches[i] + 10], :3])
        if envelope and not tomato:
            force[:] = 0
            force[tips, 2] = -0.03 * envelope
            view.apply_forces_and_torques_at_position(force, None, None, tips, True)
        section = time.perf_counter()
        world.step(render=False)
        component_block[0] += time.perf_counter() - section
        section = time.perf_counter()
        n += 1
        if world.current_time_step_index != base + n:
            raise RuntimeError("Unexpected physics step count")
        poses = np.asarray(view.get_transforms()).copy()
        component_block[1] += time.perf_counter() - section
        section = time.perf_counter()
        if not np.isfinite(poses).all():
            raise RuntimeError("Nonfinite poses")
        if equilibrium is None and t >= load_at:
            equilibrium = poses.copy()
        root_positions = None
        if a.shared_stem:
            root_positions, stem_error = attachment_frames(
                poses[stem_id], stem_local_roots
            )
            max_error = max(max_error, stem_error)
        if optimized:
            max_error = max(
                max_error,
                attachment_error(
                    poses[branch_ids],
                    poses[leaf_ids[::4], :3],
                    shifts,
                    attachment_offsets,
                    root_positions=root_positions,
                ),
            )
        else:
            for k, branch_id in enumerate(branch_ids):
                b = poses[branch_id]
                R = rotation(b[None, [6, 3, 4, 5]])[0]
                max_error = max(
                    max_error,
                    float(
                        np.linalg.norm(
                            b[:3] + R @ [0, -0.14, 0] - shifts[k] - [0, 0, 0.20]
                        )
                    ),
                )
                expected = b[:3] + attachment_offsets[3 * k : 3 * k + 3] @ R.T
                max_error = max(
                    max_error,
                    float(
                        np.linalg.norm(
                            expected - poses[leaf_ids[12 * k : 12 * k + 12 : 4], :3],
                            axis=1,
                        ).max()
                    ),
                )
        for ball, ball_id in zip(tomatoes, tomato_ids):
            ball.sample(
                n / c.hz,
                not saved and n % 48 == 0,
                poses[ball_id] if optimized else None,
            )
        if not saved and n % 48 == 0:
            samples.append(poses)
            stamps.append(n / c.hz)
        component_block[2] += time.perf_counter() - section
        if n % 16 == 0:
            section = time.perf_counter()
            if optimized:
                get_physx_interface().update_transformations(False, True, False)
            for i, anim in enumerate(animations):
                v = poses[leaf_ids[4 * i : 4 * i + 4]]
                (
                    animation_channels[i][0]
                    if optimized
                    else anim.CreateTranslationsAttr()
                ).Set(
                    Vt.Vec3fArray.FromNumpy((v[:, :3] - offsets[i]).astype(np.float32))
                )
                (
                    animation_channels[i][1]
                    if optimized
                    else anim.CreateRotationsAttr()
                ).Set(
                    Vt.QuatfArray(
                        [
                            Gf.Quatf(float(q[6]), Gf.Vec3f(*map(float, q[3:6])))
                            for q in v
                        ]
                    )
                )
            if a.gui:
                label.text = (
                    f"Tomato: {'released' if tomato.released else 'settling'} | contacts {sum(len(ball.events) for ball in tomatoes)}"
                    if tomato
                    else f"Load {envelope * 0.03:.3f} N / leaf | t = {t:.1f} s"
                )
            component_block[3] += time.perf_counter() - section
            section = time.perf_counter()
            world.render()
            component_block[4] += time.perf_counter() - section
            if not saved:
                frames.append(time.perf_counter() - frame_start)
            frame_start = time.perf_counter()
        work_block += time.perf_counter() - tick
        if n % 16 == 0:
            if performance_start is not None and not saved:
                component_frames.append(component_block.copy())
                work_frames.append(work_block)
                performance_end = time.perf_counter()
            component_block[:] = 0
            work_block = 0.0
        if not saved and t >= stop_at:
            record(True)
            saved = True
            if not a.gui:
                break
        if a.gui:
            if optimized and n % 16 == 0:
                deadline = pace_origin + n / c.hz
                time.sleep(max(0, deadline - time.perf_counter()))
                # Never run a catch-up burst after a slow frame or saving a report.
                pace_origin = max(pace_origin, time.perf_counter() - n / c.hz)
            elif not optimized:
                time.sleep(max(0, 1 / c.hz - (time.perf_counter() - tick)))
    if not saved:
        record(False)
    for ball in tomatoes:
        ball.subscription.unsubscribe()
    app.close()


if __name__ == "__main__":
    main()
