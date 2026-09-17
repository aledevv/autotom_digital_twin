"""Free-falling sphere: two-leaf fixture or a tomato on the frozen full plant."""

import json
import resource
import subprocess
import sys
import time
from dataclasses import asdict
from itertools import pairwise
from pathlib import Path

import numpy as np
from plant_model import local_poses, summary, yaw_matrix
from scene import build, save

from model import area, edges, prism_distance, rotation, skin, sphere_gap


def run(app, a, c):
    import omni.kit.app
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.viewports import set_camera_view
    from isaacsim.core.utils.xforms import get_world_pose
    from omni.kit.viewport.utility import get_active_viewport
    from omni.physx import get_physx_interface, get_physx_simulation_interface
    from pxr import (
        Gf,
        PhysicsSchemaTools,
        PhysxSchema,
        Sdf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdShade,
        Vt,
    )

    if min(a.ball_mass, a.drop_height, a.leaf_gap, a.playback_speed) <= 0:
        raise ValueError("Mass, height, gap and playback speed must be positive")
    if a.leaf_gap >= c.height - 0.02 or not c.fixed_length < a.ball_x < c.length:
        raise ValueError(
            "Leaves must be above the floor and the ball above the free lamina"
        )
    canopy = a.layout == "canopy-drop"
    data = np.load(a.run_dir / "input_mesh.npz")
    p, f = data["points"], data["faces"]
    offsets = np.array([[0.0, 0.0, 0.0], [a.lower_offset, a.lower_y, -a.leaf_gap]])
    if a.drop_test == "cascade":
        if a.cascade_layout:
            offsets = np.asarray(json.loads(a.cascade_layout.read_text()), dtype=float)
            if offsets.shape != (a.leaves, 3) or not np.isfinite(offsets).all():
                raise ValueError(
                    "Cascade layout must contain one finite xyz offset per leaf"
                )
        else:
            offsets = np.array(
                [
                    [i * a.lower_offset, i * a.lower_y, (a.leaves - 1 - i) * a.leaf_gap]
                    for i in range(a.leaves)
                ]
            )
        from plant_model import verify_layout

        verify_layout(p, [(o, 0.0) for o in offsets], c.thickness)
        if float(p[:, 2].min() + offsets[:, 2].min() - c.thickness / 2) <= 0:
            raise ValueError("Cascade leaves must start above the floor")
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1 / c.hz,
        rendering_dt=1 / a.render_hz,
        backend="numpy",
        device="cpu",
    )
    stage = world.stage
    optimized = a.drop_runtime != "baseline"
    batched = a.drop_runtime in ("batched", "visual")
    visual_only_sync = a.drop_runtime == "visual"
    if canopy:
        from canopy_fixture import build_canopy

        placements, paths, animations, geometry, info = build_canopy(world, a, c, p, f)
        offsets = np.array([o for o, _ in placements])
        yaws = np.array([y for _, y in placements])
        ball_path = "/World/Probe"
    else:
        paths, animations, geometry = [], [], []
        for i, offset in enumerate(offsets):
            prefix = f"/World/Leaves/L{i:03d}"
            root = UsdGeom.Xform.Define(stage, prefix)
            _, _, anim, geo, info = build(
                world, c, "skinning", p, f, "real", prefix, i == 0, create_view=False
            )
            root.AddTranslateOp().Set(Gf.Vec3d(*offset))
            joint = UsdPhysics.FixedJoint.Get(stage, prefix + "/FixedPetiole")
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(geo[0][0] + offset)))
            paths += [prefix + "/Petiole"] + [prefix + f"/Link{j}" for j in range(3)]
            animations.append(anim)
            geometry.append(geo)
        ball_path = "/World/Leaves/L000/Probe"
        if a.drop_test == "cascade":
            Sdf.CopySpec(
                stage.GetRootLayer(), ball_path, stage.GetRootLayer(), "/World/Probe"
            )
            stage.RemovePrim(ball_path)
            ball_path = "/World/Probe"
        yaws = np.zeros(len(offsets))
    count = len(offsets)
    if a.drop_test == "scale" and a.collision_filter == "pairs":
        # Isolate instance count from leaf-leaf and unintended tomato contacts.
        for i in range(count):
            other = [
                path
                for j in range(count)
                if j != i
                for path in paths[4 * j : 4 * j + 4]
            ]
            if i:
                other.append(ball_path)
            for path in paths[4 * i : 4 * i + 4]:
                UsdPhysics.FilteredPairsAPI.Apply(
                    stage.GetPrimAtPath(path)
                ).CreateFilteredPairsRel().SetTargets(other)
    if a.drop_test == "scale" and a.collision_filter == "groups":
        members = {
            "Pressed": [p + "/Collider" for p in paths[:4]],
            "Others": [p + "/Collider" for p in paths[4:]],
            "Ball": [ball_path + "/Collider"],
        }
        excluded = {
            "Pressed": ["Pressed", "Others"],
            "Others": ["Pressed", "Others", "Ball"],
            "Ball": ["Others"],
        }
        if a.rain:
            excluded["Others"].remove("Ball")
            excluded["Ball"] = []
        for name, targets in members.items():
            group = UsdPhysics.CollisionGroup.Define(stage, "/World/Filters/" + name)
            collection = Usd.CollectionAPI.Get(group.GetPrim(), "colliders")
            collection.CreateIncludesRel().SetTargets(targets)
            group.CreateFilteredGroupsRel().SetTargets(
                ["/World/Filters/" + v for v in excluded[name]]
            )
            query = collection.ComputeMembershipQuery()
            if not all(query.IsPathIncluded(Sdf.Path(target)) for target in targets):
                raise RuntimeError("Collision collection membership mismatch")
    ball_prim = stage.GetPrimAtPath(ball_path)
    UsdPhysics.RigidBodyAPI(ball_prim).CreateKinematicEnabledAttr().Set(False)
    UsdPhysics.MassAPI(ball_prim).CreateMassAttr().Set(a.ball_mass)
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(ball_prim)
    rb.CreateEnableCCDAttr().Set(True)
    rb.CreateEnableSpeculativeCCDAttr().Set(True)
    for path in paths:
        PhysxSchema.PhysxRigidBodyAPI.Apply(
            stage.GetPrimAtPath(path)
        ).CreateEnableSpeculativeCCDAttr().Set(True)
    gravity_attr = rb.CreateDisableGravityAttr()
    gravity_attr.Set(True)
    spawn = (
        np.array([a.ball_x, 0.0, c.height + c.radius + a.drop_height])
        @ yaw_matrix(yaws[0]).T
        + offsets[0]
    )
    ball_prim.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*spawn))
    if canopy or a.drop_test == "cascade":
        UsdGeom.Sphere.Get(stage, ball_path + "/Collider").CreateDisplayColorAttr().Set(
            [Gf.Vec3f(0.8, 0.035, 0.015)]
        )
        calyx = UsdGeom.Sphere.Define(stage, ball_path + "/Calyx")
        calyx.CreateRadiusAttr().Set(c.radius * 0.28)
        calyx.AddTranslateOp().Set(Gf.Vec3d(0, 0, c.radius))
        calyx.AddScaleOp().Set(Gf.Vec3f(1, 1, 0.3))
        calyx.CreateDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.3, 0.025)])
    visible_ball_path = ball_path
    if visual_only_sync:
        from canopy_fixture import freeze

        visible_ball_path = "/World/TomatoVisual"
        Sdf.CopySpec(
            stage.GetRootLayer(), ball_path, stage.GetRootLayer(), visible_ball_path
        )
        freeze(stage, visible_ball_path)
        UsdGeom.Imageable(ball_prim).MakeInvisible()
        visual_prim = stage.GetPrimAtPath(visible_ball_path)
        visual_position = visual_prim.GetAttribute("xformOp:translate")
        visual_orientation = visual_prim.GetAttribute("xformOp:orient")

    def update_ball_visual(transform):
        visual_position.Set(Gf.Vec3d(*map(float, transform[:3])))
        visual_orientation.Set(
            Gf.Quatf(float(transform[6]), Gf.Vec3f(*map(float, transform[3:6])))
        )

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            PhysxSchema.PhysxSceneAPI.Apply(prim).CreateEnableCCDAttr().Set(True)
    material = UsdShade.Material.Define(stage, "/World/DropContactMaterial")
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr().Set(0.3)
    mat.CreateDynamicFrictionAttr().Set(0.25)
    mat.CreateRestitutionAttr().Set(0.02)
    # Same explicit contact material for ball and leaf colliders; drives unchanged.
    for path in paths + [ball_path]:
        collider = stage.GetPrimAtPath(path + "/Collider")
        UsdShade.MaterialBindingAPI.Apply(collider).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        PhysxSchema.PhysxContactReportAPI.Apply(
            stage.GetPrimAtPath(path)
        ).CreateThresholdAttr().Set(0.0)
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr().Set(1.0)
    floor.AddTranslateOp().Set(Gf.Vec3d(0 if canopy else 0.1, 0, -0.005))
    floor.AddScaleOp().Set(
        Gf.Vec3f(1.2, 1.2, 0.01) if canopy else Gf.Vec3f(0.8, 0.6, 0.01)
    )
    floor.CreateDisplayColorAttr().Set([Gf.Vec3f(0.18, 0.20, 0.22)])
    UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, "physics"
    )
    if canopy:
        for axis in (0, 1):
            for sign in (-1, 1):
                wall = UsdGeom.Cube.Define(stage, f"/World/CatchWall{axis}_{sign + 1}")
                wall.CreateSizeAttr().Set(1.0)
                center, size = [0, 0, 0.04], [1.2, 1.2, 0.08]
                center[axis], size[axis] = sign * 0.6, 0.01
                wall.AddTranslateOp().Set(Gf.Vec3d(*center))
                wall.AddScaleOp().Set(Gf.Vec3f(*size))
                wall.CreateDisplayColorAttr().Set([Gf.Vec3f(0.18, 0.20, 0.22)])
                UsdPhysics.CollisionAPI.Apply(wall.GetPrim())
    focus = spawn - np.array([0, 0, c.radius + a.drop_height / 2])
    set_camera_view(
        eye=focus + np.array([0.28, 0.28, 0.17])
        if canopy
        else np.array([0.34, -0.42, 0.29]),
        target=focus if canopy else np.array([0.075, 0, 0.13]),
        camera_prim_path="/World/Camera",
    )
    if a.drop_test == "scale":
        set_camera_view(
            eye=np.array(info["canopy"]["camera_eye"]),
            target=np.array(info["canopy"]["camera_target"]),
            camera_prim_path="/World/Camera",
        )
    elif a.drop_test == "cascade":
        center = offsets.mean(axis=0) + np.array([0.045, 0, c.height + 0.03])
        extent = max(0.4, float(np.ptp(offsets, axis=0).max()) + 0.25)
        set_camera_view(
            eye=center + np.array([0.6, -1.2, 0.5]) * extent,
            target=center,
            camera_prim_path="/World/Camera",
        )
    cam = UsdGeom.Camera.Get(stage, "/World/Camera")
    cam.CreateFocalLengthAttr().Set(35.0)
    cam.CreateHorizontalApertureAttr().Set(36.0)
    cam.CreateVerticalApertureAttr().Set(20.25)
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = "/World/Camera"
        viewport.set_texture_resolution((1280, 720))
    n = 0
    contact_rows = []
    recording = True
    live_hits = []
    render_position_errors = []

    def contact(headers, details):
        for h in headers:
            names = [
                str(PhysicsSchemaTools.intToSdfPath(v)) for v in (h.actor0, h.actor1)
            ]
            if ball_path not in names:
                continue
            leaf = next(
                (
                    i
                    for i in range(count)
                    if any(f"/L{i:03d}/Link" in name for name in names)
                ),
                None,
            )
            if leaf is None:
                continue
            for j in range(
                h.contact_data_offset, h.contact_data_offset + h.num_contact_data
            ):
                d = details[j]
                impulse = float(np.linalg.norm(d.impulse))
                if impulse > 0 and leaf + 1 not in live_hits:
                    live_hits.append(leaf + 1)
                if recording:
                    contact_rows.append([n / c.hz, leaf, float(d.separation), impulse])

    subscription = get_physx_simulation_interface().subscribe_contact_report_events(
        contact
    )
    if a.sleep_threshold:
        for path in paths:
            PhysxSchema.PhysxRigidBodyAPI.Apply(
                stage.GetPrimAtPath(path)
            ).CreateSleepThresholdAttr().Set(a.sleep_threshold)
        for i in range(count):
            PhysxSchema.PhysxArticulationAPI.Apply(
                stage.GetPrimAtPath(f"/World/Leaves/L{i:03d}/FixedPetiole")
            ).CreateSleepThresholdAttr().Set(a.sleep_threshold)
    # CPU physics remains unchanged; publish USD poses only at visual frames.
    if optimized:
        import carb

        settings = carb.settings.get_settings()
        settings.set_bool("/physics/updateToUsd", False)
        settings.set_bool("/physics/updateVelocitiesToUsd", False)
    links = (
        None
        if optimized
        else RigidPrim(paths, name="drop_links", reset_xform_properties=False)
    )
    if a.storm:
        UsdPhysics.CollisionAPI(
            stage.GetPrimAtPath(ball_path + "/Collider")
        ).GetCollisionEnabledAttr().Set(False)
        UsdGeom.Imageable(stage.GetPrimAtPath(visible_ball_path)).MakeInvisible()
    rain = None
    if a.rain:
        from rain_scene import TomatoRain

        rain = TomatoRain(stage, a, c, ball_path, visible_ball_path, offsets, yaws, p)
    world.get_physics_context().set_solver_type(a.solver)
    info["solver"] = a.solver + "/CPU"
    world.reset()
    if links is not None:
        links.initialize()
    world.step(render=False)
    ball = world.physics_sim_view.create_rigid_body_view(ball_path)

    pose_view = (
        world.physics_sim_view.create_rigid_body_view(paths + [ball_path])
        if optimized
        else None
    )
    if pose_view is not None:
        order = list(pose_view.prim_paths)
        leaf_indices = np.array([order.index(path) for path in paths])
        ball_index = order.index(ball_path)

    if batched:
        from batch_runtime import LocalPoseBatch

        batch_converter = LocalPoseBatch(offsets, yaws)

    def read_state():
        if pose_view is not None:
            transforms = np.asarray(pose_view.get_transforms())
            values = transforms[leaf_indices]
            pos, quat = values[:, :3], values[:, [6, 3, 4, 5]]
            bt = transforms[ball_index].copy()
        else:
            pos, quat = links.get_world_poses()
            bt = np.asarray(ball.get_transforms())[0].copy()
        if batched:
            pose = batch_converter(pos, quat)
        else:
            pose = np.empty((count, 4, 7))
            for i in range(count):
                pose[i, :, :3], pose[i, :, 3:] = local_poses(
                    pos[4 * i : 4 * i + 4], quat[4 * i : 4 * i + 4], offsets[i], yaws[i]
                )
        return pose, bt, np.asarray(ball.get_velocities())[0].copy()

    def points(i, pose):
        centers, indices, weights, _ = geometry[i]
        return skin(p, centers, indices, weights, pose[:, :3], pose[:, 3:])

    animation_channels = [
        (anim.CreateTranslationsAttr(), anim.CreateRotationsAttr())
        for anim in animations
    ]

    last_visual = np.full((count, 4, 7), np.nan, dtype=np.float32)
    if batched:
        probe_quat = Vt.QuatfArray.FromNumpy(
            np.array([[1, 2, 3, 4]], dtype=np.float32)
        )[0]
        if probe_quat.GetReal() == 1 and tuple(probe_quat.GetImaginary()) == (2, 3, 4):
            quat_order = [0, 1, 2, 3]
        elif probe_quat.GetReal() == 4 and tuple(probe_quat.GetImaginary()) == (
            1,
            2,
            3,
        ):
            quat_order = [1, 2, 3, 0]
        else:
            raise RuntimeError("Unexpected Vt quaternion storage order")

    sdf_channels = (
        [
            tuple(
                stage.GetRootLayer().GetAttributeAtPath(attr.GetPath())
                for attr in channels
            )
            for channels in animation_channels
        ]
        if a.skin_writes == "sdf"
        else None
    )
    if sdf_channels is not None and not all(all(pair) for pair in sdf_channels):
        raise RuntimeError("Missing authored skin channel specs")
    merged_skins = None
    merged_errors = []
    if a.skin_batch_size:
        from skin_batches import SkinBatches

        merged_skins = SkinBatches(
            stage, p, f, geometry, offsets, yaws, a.skin_batch_size, quat_order
        )
        last_visual = np.full((count, 4, 7), np.nan)
    visual_error_max = 0.0
    bone_radii = np.array(
        [np.linalg.norm(p - center, axis=1).max() for center in geometry[0][0]]
    )

    def visual_error(pose, displayed):
        dr = rotation(pose[:, :, 3:].reshape(-1, 4)) - rotation(
            displayed[:, :, 3:].reshape(-1, 4)
        )
        position_error = np.linalg.norm(pose[:, :, :3] - displayed[:, :, :3], axis=2)
        return np.max(
            position_error
            + np.linalg.norm(dr, axis=(1, 2)).reshape(count, 4) * bone_radii,
            axis=1,
        )

    def animate(pose):
        nonlocal visual_error_max
        if batched:
            visual = pose.astype(np.float32)
            if a.visual_tolerance and np.isfinite(last_visual).all():
                change = visual_error(pose, last_visual) > a.visual_tolerance * (
                    0.7 if merged_skins is not None else 1.0
                )
            else:
                change = np.any(visual != last_visual, axis=(1, 2))
            if merged_skins is not None:
                last_visual[:] = merged_skins.update(pose, change)
            elif sdf_channels is not None:
                updates = [
                    (
                        int(i),
                        Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(visual[i, :, :3])),
                        Vt.QuatfArray.FromNumpy(
                            np.ascontiguousarray(visual[i, :, 3:][:, quat_order])
                        ),
                    )
                    for i in np.flatnonzero(change)
                ]
                # Only Sdf layer edits inside the block; no USD queries or setters.
                with Sdf.ChangeBlock():
                    for i, translations, rotations in updates:
                        sdf_channels[i][0].default = translations
                        sdf_channels[i][1].default = rotations
                last_visual[change] = visual[change]
            else:
                for i in np.flatnonzero(change):
                    translations, rotations = animation_channels[i]
                    translations.Set(
                        Vt.Vec3fArray.FromNumpy(np.ascontiguousarray(visual[i, :, :3]))
                    )
                    rotations.Set(
                        Vt.QuatfArray.FromNumpy(
                            np.ascontiguousarray(visual[i, :, 3:][:, quat_order])
                        )
                    )
                last_visual[change] = visual[change]
            if a.visual_tolerance:
                remaining = float(
                    (
                        visual_error(pose, last_visual)
                        + (merged_skins.rest_error if merged_skins is not None else 0)
                    ).max()
                )
                visual_error_max = max(visual_error_max, remaining)
                if remaining > a.visual_tolerance + 1e-7:
                    raise RuntimeError(
                        "Visual skinning error exceeds the explicit tolerance"
                    )
            return
        for i, anim in enumerate(animations):
            translations, rotations = (
                animation_channels[i]
                if optimized
                else (anim.CreateTranslationsAttr(), anim.CreateRotationsAttr())
            )
            translations.Set(Vt.Vec3fArray(pose[i, :, :3].tolist()))
            rotations.Set(
                Vt.QuatfArray(
                    [
                        Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:])))
                        for q in pose[i, :, 3:]
                    ]
                )
            )

    initial, initial_ball, initial_velocity = read_state()
    if not np.allclose(initial_ball[:3], spawn, rtol=0, atol=1e-6):
        raise RuntimeError("Ball spawn does not match the requested world position")
    animate(initial)
    if visual_only_sync:
        update_ball_visual(initial_ball)
    if merged_skins is not None:
        merged_errors.append(merged_skins.validate(initial))
    stage.GetRootLayer().Export(str(a.run_dir / "scene.usda"))
    manager = omni.kit.app.get_app().get_extension_manager()
    info.update(
        python=sys.executable,
        isaac_version=(Path.home() / "isaacsim-6.1/VERSION").read_text().strip(),
        physx_extensions=[
            e["id"]
            for e in manager.get_extensions()
            if e["id"].startswith("omni.physx-")
        ],
        gpu=subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            check=False,
        ).stdout.strip(),
        config=asdict(c),
        drop_runtime=a.drop_runtime,
        visible_ball_path=visible_ball_path,
        collision_filter=a.collision_filter,
        visual_tolerance_m=a.visual_tolerance,
        skin_batch_size=a.skin_batch_size,
        sleep_threshold=a.sleep_threshold,
        stress_all=a.stress_all,
        physics_usd_sync=world.get_physics_context().get_physx_update_transformations_settings(),
        ball_mass_kg=a.ball_mass,
        ball_radius_m=c.radius,
        ball_dynamic=True,
        ccd=True,
        speculative_ccd=True,
        ball_spawn_m=spawn.tolist(),
        leaf_offsets_m=offsets.tolist(),
        leaf_yaws_rad=yaws.tolist(),
        count=count,
        render_hz=a.render_hz,
        drop_test=a.drop_test,
        render_resolution=[1280, 720],
        drop_height_m=a.drop_height,
        friction={"static": 0.3, "dynamic": 0.25, "restitution": 0.02},
        playback_speed=a.playback_speed,
        no_ball_commands_during_flight=True,
    )
    save(a.run_dir / "runtime.json", info)
    save(
        a.run_dir / "layout.json",
        {"kind": a.layout, "offsets": offsets.tolist(), "yaws": yaws.tolist()},
    )
    if a.storm:
        from storm_scene import run_storm

        subscription.unsubscribe()
        return run_storm(
            app,
            world,
            a,
            c,
            read_state,
            animate,
            initial,
            p,
            f,
            geometry,
            offsets,
            yaws,
            info,
        )
    if rain is not None:
        from rain_scene import run_rain

        subscription.unsubscribe()
        return run_rain(app, world, a, c, rain, read_state, animate, initial, info)
    state = {"quit": False, "repeat": False, "release": False}
    window = None
    status_label = None
    contact_label = None
    if a.gui:
        from omni import ui

        window = ui.Window(
            "Tomato: free fall" if canopy else f"Free fall onto {count} leaves",
            width=430,
            height=280,
        )
        with window.frame, ui.VStack():
            ui.Label(
                f"Dynamic ball: {a.ball_mass * 1000:g} g | radius {c.radius * 1000:g} mm"
            )
            ui.Label(f"After release: playback speed {a.playback_speed:g}x")
            if canopy:
                ui.Label(f"{count} dynamic leaves; remaining plant is visual only")
            if a.recording == "first-trial":
                ui.Label(
                    "Record first trial only; further interaction is not recorded"
                )
            status_label = ui.Label("Settling before release...")
            contact_label = ui.Label("Leaves hit: none")
            if a.gui_benchmark:
                ui.Label("Automatic benchmark: closes after the test")
            else:
                ui.Label("Window stays open until Finish and save")
            ui.Button(
                "Drop ball",
                clicked_fn=lambda: state.update(release=True),
                enabled=not a.gui_benchmark,
            )
            ui.Button(
                "Repeat drop",
                clicked_fn=lambda: state.update(repeat=True),
                enabled=not a.gui_benchmark,
            )
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    release_at = 1 + c.settle
    stop_at = release_at + (10 if a.stress_all else 8)
    releases = []
    resets = []
    baselines = []
    released = False
    poses = [initial]
    times = [0.0]
    balls = [initial_ball]
    velocities = [initial_velocity]
    sleep_before = []
    sleep_during_contact = None
    stage_id = int(omni.usd.get_context().get_stage_id())
    sleep_ids = [PhysicsSchemaTools.sdfPathToInt(path) for path in paths[::4]]
    base_step = world.current_time_step_index
    base_clock = float(world.current_time)
    component = {k: [] for k in ("physics", "read", "skinning", "render")}
    frame_work, frame_wall, frame_times = [], [], []
    block_work = 0.0
    measured_wall_start = None
    start = time.perf_counter()
    block_start = start
    pace_deadline = start
    live_gui_saved = False
    frozen = None
    initial_stop_at = stop_at

    while app.is_running() and not state["quit"]:
        tick = time.perf_counter()
        t = n / c.hz
        if not world.is_playing():
            raise RuntimeError("Timeline stopped")
        if t >= (initial_stop_at + a.tail_seconds if a.tail_seconds else stop_at) and (
            not a.gui or a.gui_benchmark
        ):
            break
        if n == round((1 + c.settle) * c.hz):
            measured_wall_start = time.perf_counter()
        if n == c.hz:
            world.get_physics_context().set_gravity(-9.81)
            if a.sleep_threshold:
                for body_id in sleep_ids:
                    get_physx_simulation_interface().wake_up(stage_id, body_id)
        if a.sleep_threshold and n == round((1 + c.settle) * c.hz) - 1:
            sleep_before = [
                get_physx_simulation_interface().is_sleeping(stage_id, body_id)
                for body_id in sleep_ids
            ]
        if a.stress_all and n in (
            round((2 + c.settle) * c.hz),
            round((3 + c.settle) * c.hz),
        ):
            world.get_physics_context().set_gravity(
                -19.62 if n == round((2 + c.settle) * c.hz) else -9.81
            )
            if a.sleep_threshold:
                for body_id in sleep_ids:
                    get_physx_simulation_interface().wake_up(stage_id, body_id)
        if state.pop("repeat", False):
            gravity_attr.Set(True)
            ball.set_transforms(
                np.array([[*spawn, 0, 0, 0, 1]], dtype=np.float32),
                np.array([0], dtype=np.int32),
            )
            ball.set_velocities(
                np.zeros((1, 6), dtype=np.float32), np.array([0], dtype=np.int32)
            )
            released = False
            release_at = t + c.settle
            stop_at = release_at + (10 if a.stress_all else 8)
            live_hits.clear()
            if recording:
                resets.append(t)
        if state.pop("release", False) and not released:
            release_at = max(t, 1 + c.settle)
            stop_at = release_at + (10 if a.stress_all else 8)
        if t >= release_at and not released:
            gravity_attr.Set(False)
            released = True
            if recording:
                releases.append(t)
                baselines.append(len(poses) - 1)
        # No pose, target, velocity or force commands to the ball during flight.
        n += 1
        step_start = time.perf_counter()
        world.step(render=False)
        physics_time = time.perf_counter() - step_start
        read_start = time.perf_counter()
        if world.current_time_step_index != base_step + n:
            raise RuntimeError("Physics step count mismatch")
        pose, bt, bv = read_state()
        if (
            a.sleep_threshold
            and sleep_during_contact is None
            and any(row[1] == 0 and row[3] > 0 for row in contact_rows)
        ):
            sleep_during_contact = get_physx_simulation_interface().is_sleeping(
                stage_id, sleep_ids[0]
            )
        if (
            not np.isfinite(pose).all()
            or not np.isfinite(bt).all()
            or not np.isfinite(bv).all()
        ):
            raise RuntimeError("Nonfinite drop state")
        if recording:
            poses.append(pose)
            times.append(n / c.hz)
            balls.append(bt)
            velocities.append(bv)
        read_time = time.perf_counter() - read_start
        skin_time = render_time = 0.0
        speed = a.playback_speed if released else 1.0
        stride = max(
            1,
            round(
                c.hz * (speed if a.gui and not a.gui_benchmark else 1.0) / a.render_hz
            ),
        )
        if (a.gui or a.render) and n % stride == 0:
            skin_start = time.perf_counter()
            if visual_only_sync:
                update_ball_visual(bt)
            elif optimized:
                get_physx_interface().update_transformations(False, True, False)
            animate(pose)
            if status_label is not None:
                status_label.text = (
                    "Gravity enabled: free fall"
                    if released
                    else f"Releasing in {max(0, release_at - t):.1f} s"
                )
            if contact_label is not None:
                contact_label.text = "Leaves hit: " + (
                    " → ".join(map(str, live_hits)) or "none"
                )
            skin_time = time.perf_counter() - skin_start
            render_start = time.perf_counter()
            world.render()
            render_time = time.perf_counter() - render_start
            # Verify the actual visible collider in USD independently of tensors.
            if releases and releases[0] <= n / c.hz <= releases[0] + 1:
                visible_position, _ = get_world_pose(
                    visible_ball_path + "/Collider", fabric=False
                )
                render_position_errors.append(
                    float(np.linalg.norm(np.asarray(visible_position) - bt[:3]))
                )
        work = time.perf_counter() - tick
        if recording and t >= 1 + c.settle:
            for key, value in zip(
                component, (physics_time, read_time, skin_time, render_time)
            ):
                component[key].append(value)
            block_work += work
            if n % stride == 0:
                frame_work.append(block_work)
                frame_wall.append(time.perf_counter() - block_start)
                frame_times.append(n / c.hz)
                block_work = 0.0
        if n % stride == 0:
            block_start = time.perf_counter()
        if a.gui and not a.gui_benchmark:
            if optimized:
                # Rendering occasionally exceeds one physics dt; recover that
                # time in subsequent substeps instead of adding it to every frame.
                pace_deadline += 1 / c.hz / speed
                time.sleep(max(0.0, pace_deadline - time.perf_counter()))
            else:
                time.sleep(max(0.0, 1 / c.hz / speed - (time.perf_counter() - tick)))
            if not live_gui_saved and not resets and releases and n / c.hz >= stop_at:
                wall = time.perf_counter() - measured_wall_start
                save(
                    a.run_dir / "gui_initial_window.json",
                    {
                        "drop_runtime": a.drop_runtime,
                        "playback_speed": a.playback_speed,
                        "simulated_seconds": n / c.hz - (1 + c.settle),
                        "wall_seconds": wall,
                        "realtime_factor": (n / c.hz - (1 + c.settle)) / wall,
                        "render_frames_per_wall_second": len(frame_work) / wall,
                        "frame_work": summary(frame_work),
                        "render_position_max_error_m": max(render_position_errors)
                        if render_position_errors
                        else None,
                        "scope": "Paced interactive GUI; first automatic drop only; no application shutdown",
                    },
                )
                live_gui_saved = True
        if recording and a.recording == "first-trial" and n / c.hz >= initial_stop_at:
            frozen = {
                "n": n,
                "clock": float(world.current_time),
                "stop_at": stop_at,
                "elapsed": time.perf_counter() - start,
                "measurement_wall": time.perf_counter() - measured_wall_start
                if measured_wall_start
                else 0.0,
                "releases": list(releases),
                "resets": list(resets),
                "baselines": list(baselines),
            }
            recording = False
            save(
                a.run_dir / "recording_window.json",
                {
                    "start_s": 0.0,
                    "end_s": n / c.hz,
                    "pose_samples": len(poses),
                    "scope": "First trial only; subsequent interaction is live, not recorded",
                },
            )
    live_end = {"steps": n, "time_s": n / c.hz, "pose_samples": len(poses)}
    end_clock = float(world.current_time)
    elapsed = time.perf_counter() - start
    measurement_wall = (
        time.perf_counter() - measured_wall_start if measured_wall_start else 0.0
    )
    if frozen:
        n, end_clock, stop_at = frozen["n"], frozen["clock"], frozen["stop_at"]
        elapsed, measurement_wall = frozen["elapsed"], frozen["measurement_wall"]
        releases, resets, baselines = (
            frozen["releases"],
            frozen["resets"],
            frozen["baselines"],
        )
    save(a.run_dir / "live_end.json", live_end)
    save(
        a.run_dir / "timing_preview.json",
        {
            "leaf_count": count,
            "realtime_factor": max(0, n / c.hz - (1 + c.settle))
            / max(measurement_wall, 1e-9),
            "rendered_fps": len(frame_work) / max(measurement_wall, 1e-9)
            if a.gui or a.render
            else None,
            "frame_work": summary(frame_work),
            "numerical_acceptance": "pending offline analysis",
        },
    )
    analysis_start = time.perf_counter()
    poses = np.asarray(poses)
    times = np.asarray(times)
    balls = np.asarray(balls)
    velocities = np.asarray(velocities)
    contacts = np.asarray(contact_rows, dtype=float).reshape(-1, 4)
    np.savez_compressed(
        a.run_dir / "trace.npz",
        times=times,
        rigid_poses=poses,
        ball=balls,
        ball_velocities=velocities,
        contacts=contacts,
    )
    np.savez_compressed(a.run_dir / "rest_mesh.npz", points=p, faces=f)
    if merged_skins is not None:
        for sample in (9.15, 10.5, times[-1]):
            merged_errors.append(
                merged_skins.validate(poses[np.argmin(abs(times - sample))])
            )
        merged_skins.update(poses[-1], np.ones(count, dtype=bool))
    usd_skin_errors = []
    if sdf_channels is not None:
        from pxr import UsdSkel

        for sample in (0.0, 9.15, 10.5, times[-1]):
            sample_pose = poses[np.argmin(abs(times - sample))]
            animate(sample_pose)
            error = 0.0
            for i in sorted({0, count // 2, count - 1}):
                root = UsdSkel.Root.Get(stage, f"/World/Leaves/L{i:03d}/Leaf")
                skeleton = UsdSkel.Skeleton.Get(
                    stage, str(root.GetPath()) + "/Skeleton"
                )
                mesh = UsdGeom.Mesh.Get(stage, str(root.GetPath()) + "/Mesh")
                cache = UsdSkel.Cache()
                cache.Populate(root, Usd.PrimDefaultPredicate)
                query = cache.GetSkelQuery(skeleton)
                skinner = cache.GetSkinningQuery(mesh.GetPrim())
                output = mesh.GetPointsAttr().Get()
                if not skinner.ComputeSkinnedPoints(
                    query.ComputeSkinningTransforms(Usd.TimeCode.Default()),
                    output,
                    Usd.TimeCode.Default(),
                ):
                    raise RuntimeError("USD skin evaluation failed")
                error = max(
                    error,
                    float(
                        np.linalg.norm(
                            np.asarray(output) - points(i, sample_pose[i]), axis=1
                        ).max()
                    ),
                )
            if error > a.visual_tolerance + 2e-7:
                raise RuntimeError(f"Authored skin disagrees with physics by {error} m")
            usd_skin_errors.append(error)
        animate(poses[-1])
    e = edges(f)
    edge_length = np.linalg.norm(p[e[:, 1]] - p[e[:, 0]], axis=1)
    ar = area(p, f)
    fixed = p[:, 0] <= c.fixed_length + 1e-7
    results = []
    bound_reference = None
    if a.geometry_method == "bounded":
        if not all(
            all(np.array_equal(g[k], geometry[0][k]) for k in range(3))
            for g in geometry
        ):
            raise RuntimeError(
                "Bounded analysis requires identical skin geometry and weights"
            )
        if np.min(geometry[0][2]) < 0 or not np.allclose(
            geometry[0][2].sum(axis=1), 1, atol=1e-12, rtol=0
        ):
            raise RuntimeError(
                "Bounded analysis requires normalized nonnegative skin weights"
            )
    if a.analysis_reference:
        reference = a.analysis_reference
        old_runtime = json.loads((reference / "runtime.json").read_text())
        for key in (
            "config",
            "leaf_offsets_m",
            "leaf_yaws_rad",
            "ball_mass_kg",
            "ball_radius_m",
            "solver",
        ):
            if old_runtime[key] != info[key]:
                raise RuntimeError(f"Analysis reference mismatch: {key}")
        with np.load(reference / "trace.npz") as old:
            for key, value in (
                ("times", times),
                ("rigid_poses", poses),
                ("ball", balls),
                ("ball_velocities", velocities),
                ("contacts", contacts),
            ):
                if not np.array_equal(old[key], value):
                    raise RuntimeError(f"Analysis reference trajectory mismatch: {key}")
        with np.load(reference / "rest_mesh.npz") as old:
            if not np.array_equal(old["points"], p) or not np.array_equal(
                old["faces"], f
            ):
                raise RuntimeError("Analysis reference mesh mismatch")
        old_report = json.loads((reference / "report.json").read_text())
        if (
            old_report["status"] != "passed"
            or not old_report.get("process_ok", False)
            or not old_report.get("geometry_full_rate", True)
        ):
            raise RuntimeError("Analysis reference must have passed all checks")
        results = old_report["leaves"]
        save(
            a.run_dir / "analysis_reuse.json",
            {
                "reference": str(reference),
                "all_inputs_bit_identical": True,
                "scope": "Leaf geometric metrics only; timing, clock, contact ordering and free fall rechecked",
            },
        )
    for i in range(0 if results else count):
        event = contacts[(contacts[:, 1] == i) & (contacts[:, 3] > 0)]
        if (
            a.geometry_method == "bounded"
            and not len(event)
            and bound_reference is not None
        ):
            from geometry_bounds import (
                edge_error_bound,
                metric_bounds,
                skin_error_bound,
            )

            reference_metrics, reference_poses, reference_low, reference_high = (
                bound_reference
            )
            error = skin_error_bound(poses[:, i], reference_poses, p, geometry[i][0])
            edge_error = edge_error_bound(
                poses[:, i], reference_poses, p, *geometry[i][:3], f
            )
            bounded = metric_bounds(
                reference_metrics, error, times, baselines[0], p, f, edge_error
            )
            # If a bound is too loose, fall back to exact mesh reconstruction.
            if (
                bounded["max_edge_extension"] < 0.05
                and bounded["min_area_ratio"] > 0.05
                and bounded["attachment_drift_m"] < 0.0002
                and bounded["recovery_last_second_m"] < 0.002
                and bounded["residual_motion_last_second_m"] < 0.0005
                and 0.00001 < bounded["gravity_sag_lower_bound_m"]
                and bounded["gravity_sag_upper_bound_m"]
                < 0.3 * (c.length - c.fixed_length)
            ):
                local_ball = (balls[:, :3] - offsets[i]) @ yaw_matrix(yaws[i])
                distances = []
                for j in range(1, 4):
                    relative = np.einsum(
                        "ti,tij->tj",
                        local_ball - poses[:, i, j, :3],
                        rotation(poses[:, i, j, 3:]),
                    )
                    distances.append(
                        prism_distance(relative, geometry[i][3][j], c.thickness)
                        - c.radius
                    )
                near = np.flatnonzero(
                    np.all(
                        local_ball >= reference_low - error[:, None] - c.radius, axis=1
                    )
                    & np.all(
                        local_ball <= reference_high + error[:, None] + c.radius, axis=1
                    )
                )
                bounded.update(
                    leaf=i,
                    max_geometric_collider_penetration_m=max(
                        0.0, -float(np.min(distances))
                    ),
                    max_visual_surface_penetration_m=max(
                        [0.0]
                        + [
                            -sphere_gap(
                                points(i, poses[k, i]), f, local_ball[k], c.radius
                            )
                            for k in near
                        ]
                    ),
                )
                results.append(bounded)
                continue
        steps = np.arange(len(times))
        if a.diagnostics == "light" and not len(event):
            steps = np.unique(
                np.r_[
                    np.arange(0, len(times), max(1, c.hz // 60)),
                    baselines,
                    len(times) - 1,
                ]
            ).astype(int)
        leaf_times = times[steps]
        leaf_poses = poses[steps, i]
        trace = np.asarray([points(i, pose) for pose in leaf_poses])
        baseline = (
            trace[np.searchsorted(steps, baselines[0])] if baselines else trace[-1]
        )
        ratios = (
            np.linalg.norm(trace[:, e[:, 1]] - trace[:, e[:, 0]], axis=2) / edge_length
        )
        deformation = float(
            np.linalg.norm(
                trace[leaf_times >= (releases[0] if releases else times[-1])]
                - baseline,
                axis=2,
            ).max()
        )
        recovery = float(
            np.linalg.norm(trace[leaf_times >= times[-1] - 1] - baseline, axis=2).max()
        )
        residual = float(
            np.linalg.norm(
                np.ptp(trace[leaf_times >= times[-1] - 1], axis=0), axis=1
            ).max()
        )
        tip = p[:, 0] > p[:, 0].max() - 0.0031
        sag = float(np.mean(trace[0, tip, 2] - baseline[tip, 2]))
        drift = float(np.linalg.norm(trace[:, fixed] - trace[0, fixed], axis=2).max())
        min_area = min(float(np.min(area(v, f) / ar)) for v in trace)
        local_ball = (balls[steps, :3] - offsets[i]) @ yaw_matrix(yaws[i])
        distances = []
        for j in range(1, 4):
            relative = np.einsum(
                "ti,tij->tj",
                local_ball - leaf_poses[:, j, :3],
                rotation(leaf_poses[:, j, 3:]),
            )
            distances.append(
                prism_distance(relative, geometry[i][3][j], c.thickness) - c.radius
            )
        geometric_penetration = max(0.0, -float(np.min(distances)))
        near = np.flatnonzero(
            np.all(local_ball >= trace.min(axis=1) - c.radius, axis=1)
            & np.all(local_ball <= trace.max(axis=1) + c.radius, axis=1)
        )
        visual_penetration = max(
            [0.0] + [-sphere_gap(trace[k], f, local_ball[k], c.radius) for k in near]
        )
        results.append(
            {
                "leaf": i,
                "first_contact_s": float(event[:, 0].min()) if len(event) else None,
                "last_contact_s": float(event[:, 0].max()) if len(event) else None,
                "contact_samples": len(event),
                "max_contact_impulse_Ns": float(event[:, 3].max())
                if len(event)
                else 0.0,
                "max_contact_penetration_m": max(0.0, float(-event[:, 2].min()))
                if len(event)
                else 0.0,
                "max_deflection_from_equilibrium_m": deformation,
                "recovery_last_second_m": recovery,
                "residual_motion_last_second_m": residual,
                "gravity_sag_m": sag,
                "attachment_drift_m": drift,
                "max_edge_extension": float(ratios.max() - 1),
                "min_area_ratio": min_area,
                "max_geometric_collider_penetration_m": geometric_penetration,
                "max_visual_surface_penetration_m": visual_penetration,
            }
        )
        if (
            a.geometry_method == "bounded"
            and bound_reference is None
            and not len(event)
            and len(steps) == len(times)
        ):
            bound_reference = (
                dict(results[-1]),
                poses[:, i].copy(),
                trace.min(axis=1),
                trace.max(axis=1),
            )
    targets = results[:1] if a.drop_test == "scale" else results
    ordered = all(r["first_contact_s"] is not None for r in targets) and all(
        left["first_contact_s"] < right["first_contact_s"]
        for left, right in pairwise(targets)
    )
    physical_ok = all(
        r["attachment_drift_m"] < 0.0002
        and r["max_edge_extension"] < 0.05
        and r["min_area_ratio"] > 0.05
        for r in results
    )
    checks = {
        "completed": bool(releases and times[-1] >= stop_at - 1 / c.hz),
        "ordered_ball_contacts": bool(ordered),
        "leaves_recover": all(
            r["recovery_last_second_m"] < 0.002
            and r["residual_motion_last_second_m"] < 0.0005
            for r in results
        ),
        "gravity_sag": all(
            0.00001 < r.get("gravity_sag_lower_bound_m", r["gravity_sag_m"])
            and r.get("gravity_sag_upper_bound_m", r["gravity_sag_m"])
            < 0.3 * (c.length - c.fixed_length)
            for r in results
        ),
        "leaves_move": all(
            r.get("deflection_lower_bound_m", r["max_deflection_from_equilibrium_m"])
            > 0.001
            for r in targets
        ),
        "structure": physical_ok,
        "contact_penetration": all(
            r["max_contact_penetration_m"] < 0.001 for r in results
        ),
        "geometric_collider_penetration": all(
            r["max_geometric_collider_penetration_m"] < 0.001 for r in results
        ),
        "physics_clock": bool(
            np.isclose(end_clock - base_clock, n / c.hz, rtol=5e-5, atol=5e-5)
        ),
    }
    if a.drop_test == "scale" and not a.stress_all:
        checks["independent_leaves_still"] = all(
            r["max_deflection_from_equilibrium_m"] < 0.0005
            and r["contact_samples"] == 0
            for r in results[1:]
        )
    if a.stress_all:
        checks["all_leaves_respond"] = all(
            r.get("deflection_lower_bound_m", r["max_deflection_from_equilibrium_m"])
            > 0.001
            for r in results
        )
    if not canopy:
        checks["both_leaves_recover"] = checks["leaves_recover"]
        checks["both_leaves_move"] = checks["leaves_move"]
    freefall = {"passed": False}
    if releases and results[0]["first_contact_s"] is not None:
        dt = times - releases[0]
        pre = (
            (dt > 0) & (dt <= 0.08) & (times < results[0]["first_contact_s"] - 1 / c.hz)
        )
        if pre.sum() >= 3:
            predicted_z = balls[baselines[0], 2] - 0.5 * 9.81 * dt[pre] * (
                dt[pre] + 1 / c.hz
            )
            error = float(np.max(abs(balls[pre, 2] - predicted_z)))
            freefall = {
                "samples": int(pre.sum()),
                "max_position_error_m": error,
                "passed": error < 0.0002,
            }
    checks["free_fall_before_contact"] = freefall["passed"]
    if a.render or a.gui:
        checks["visible_ball_follows_physics"] = bool(
            render_position_errors and max(render_position_errors) < 1e-5
        )
    measured_sim = max(0, times[-1] - (1 + c.settle))
    timing = {
        "physics_hz": c.hz,
        "drop_runtime": a.drop_runtime,
        "measurement_simulated_seconds": measured_sim,
        "measurement_wall_seconds": measurement_wall,
        "realtime_factor": measured_sim / max(measurement_wall, 1e-9),
        "frame_work": summary(frame_work),
        "components": {k: summary(v) for k, v in component.items()},
        "render_hz": a.render_hz,
        "rendered_fps_unpaced": len(frame_work) / max(measurement_wall, 1e-9)
        if a.render or a.gui
        else None,
        "p95_capacity_fps": 1 / max(float(np.percentile(frame_work, 95)), 1e-9)
        if frame_work
        else None,
        "paced": bool(a.gui and not a.gui_benchmark),
        "playback_speed": a.playback_speed,
    }
    impact = [
        work
        for t, work in zip(frame_times, frame_work)
        if releases and releases[0] <= t <= releases[0] + 1
    ]
    timing["first_second_after_release"] = summary(impact)
    timing["first_second_after_release"]["max_seconds"] = (
        max(impact) if impact else None
    )
    timing["performance_passed"] = bool(
        (a.render or a.gui_benchmark)
        and not timing["paced"]
        and measured_sim > 0
        and timing["realtime_factor"] >= 1
        and timing["frame_work"]["p95_seconds"] <= 1 / a.render_hz
    )
    timing["display_20fps_passed"] = bool(
        (a.render or a.gui_benchmark)
        and not timing["paced"]
        and timing["rendered_fps_unpaced"] > 20
        and timing["frame_work"]["p95_seconds"] < 0.05
    )
    timing["target_20fps_passed"] = bool(
        (a.render or a.gui_benchmark)
        and not timing["paced"]
        and timing["realtime_factor"] >= 1
        and timing["rendered_fps_unpaced"] > 20
        and timing["frame_work"]["p95_seconds"] < 0.05
    )
    np.savez_compressed(
        a.run_dir / "timings.npz",
        frame_work=frame_work,
        frame_wall=frame_wall,
        frame_times=frame_times,
        **component,
    )
    report = {
        "status": "passed" if all(checks.values()) else "failed",
        "geometry_full_rate": a.diagnostics == "full"
        or a.analysis_reference is not None,
        "geometry_method": a.geometry_method,
        "geometry_sampling_note": "Full rate for contacted leaves; other leaves sampled at 60 Hz in light mode",
        "checks": checks,
        "freefall": freefall,
        "maximum_visual_leaf_error_bound_m": visual_error_max,
        "merged_skin_reference_errors_m": merged_errors,
        "usd_skin_reference_errors_m": usd_skin_errors,
        "skin_writes": a.skin_writes,
        "sleeping_before_release": sleep_before,
        "contacted_leaf_sleeping_during_contact": sleep_during_contact,
        "render_position_check": {
            "samples": len(render_position_errors),
            "max_error_m": max(render_position_errors)
            if render_position_errors
            else None,
        },
        "timing": timing,
        "memory_peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        * 1024,
        "offline_analysis_seconds": time.perf_counter() - analysis_start,
        "leaves": results,
        "release_times_s": releases,
        "reset_times_s": resets,
        "simulated_seconds": times[-1],
        "loop_wall_seconds": elapsed,
        "ball_dynamic": True,
        "kinematic_targets_during_flight": 0,
        "visual_acceptance": "pending",
        "scope": "Dynamic drop demonstration; not the earlier quasi-static acceptance protocol",
    }
    save(a.run_dir / "report.json", report)
    print("DROP_REPORT", report, flush=True)
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
            raise RuntimeError("Capture did not complete")
        task.result()
    _ = subscription, window
