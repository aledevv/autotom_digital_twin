"""Isaac 4.5: dynamic petiole + spring skeleton + continuous UsdSkel leaf."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np
from geometry import Config, geometry, half_width, rotations, skin, press_fraction


def save(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def build(world, c):
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade, UsdSkel, Vt
    from omni.physx.scripts import physicsUtils
    from isaacsim.core.prims import RigidPrim

    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    physics = world.get_physics_context()
    physics.enable_gpu_dynamics(False)
    physics.set_broadphase_type("MBP")
    physics.set_solver_type("PGS")
    physics.set_gravity(0.)
    phys_scene = PhysxSchema.PhysxSceneAPI.Apply(physics.get_current_physics_scene_prim())
    phys_scene.CreateEnableCCDAttr().Set(True)
    root = UsdGeom.Xform.Define(stage, "/World/Plant").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(4)
    # Body sleep settings do not control reduced-coordinate articulations.
    # Keep the whole small leaf awake to measure actual settling and hold loads.
    art.CreateSleepThresholdAttr().Set(0.)
    material = UsdShade.Material.Define(stage, "/World/ContactMaterial")
    pm = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    pm.CreateStaticFrictionAttr().Set(0.6)
    pm.CreateDynamicFrictionAttr().Set(0.5)
    pm.CreateRestitutionAttr().Set(0.05)

    def rigid(path, position, size, mass, color, hidden=False, sphere=False, kinematic=False):
        # Keep unit scale on every body: USD joint anchors inherit body scale.
        # Shape dimensions belong on a child collider, not on the rigid frame.
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*position))
        body.AddOrientOp().Set(Gf.Quatf(1.))
        body.AddScaleOp().Set(Gf.Vec3f(1.))
        prim = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        if sphere:
            shape = UsdGeom.Sphere.Define(stage, path+"/Collider")
            shape.CreateRadiusAttr().Set(size)
        else:
            shape = UsdGeom.Cube.Define(stage, path+"/Collider")
            shape.CreateSizeAttr().Set(1.)
            shape.AddScaleOp().Set(Gf.Vec3f(*size))
        shape.CreateDisplayColorAttr().Set([Gf.Vec3f(*color)])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(kinematic)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSolverPositionIterationCountAttr().Set(64)
        rb.CreateSolverVelocityIterationCountAttr().Set(4)
        rb.CreateEnableCCDAttr().Set(not kinematic)
        rb.CreateLinearDampingAttr().Set(0.0)
        rb.CreateAngularDampingAttr().Set(0.05)
        rb.CreateSleepThresholdAttr().Set(0.0)
        collider = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        collider.CreateContactOffsetAttr().Set(0.00025)
        collider.CreateRestOffsetAttr().Set(0.)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose="physics")
        if hidden:
            UsdGeom.Imageable(prim).MakeInvisible()
        return prim

    base = rigid("/World/Plant/Base", [0, 0, c.height], [.006, .006, .006], .01, [.3, .32, .35])
    fixed = UsdPhysics.FixedJoint.Define(stage, "/World/Plant/FixedBase")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])
    fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, c.height))
    fixed.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
    petiole_pos = np.array([c.petiole_length/2, 0, c.height])
    petiole = rigid("/World/Plant/Petiole", petiole_pos,
                    [c.petiole_length, .0025, .0025], c.petiole_mass, [.2, .4, .07])
    points, faces, centers, indices, weights = geometry(c)
    bodies = [petiole]
    ds = c.length/c.links
    for i, center in enumerate(centers):
        width = max(float(half_width((i+0.5)/c.links, c))*2, .004)
        bodies.append(rigid(f"/World/Plant/Leaf_{i}", center,
            [ds+0.0002, width, c.thickness], c.leaf_mass/c.links, [.15, .42, .08], hidden=True))

    joint_specs = []
    def joint(name, parent, child, parent_center, child_center, anchor, stiffness, inertia, limit):
        j = UsdPhysics.Joint.Define(stage, "/World/Plant/"+name)
        j.CreateBody0Rel().SetTargets([parent.GetPath()])
        j.CreateBody1Rel().SetTargets([child.GetPath()])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(anchor-parent_center)))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(*(anchor-child_center)))
        j.CreateCollisionEnabledAttr().Set(False)
        for axis in ("transX", "transY", "transZ", "rotZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr().Set(1.)
            lim.CreateHighAttr().Set(-1.)
        damping = 2*c.damping_ratio*np.sqrt(stiffness*inertia)
        for axis, factor in (("rotY", 1.), ("rotX", 1.5)):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr().Set(-limit)
            lim.CreateHighAttr().Set(limit)
            drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(stiffness*factor*np.pi/180)
            drive.CreateDampingAttr().Set(damping*np.pi/180)
            drive.CreateTargetPositionAttr().Set(0.)
            drive.CreateTargetVelocityAttr().Set(0.)
        joint_specs.append((anchor-parent_center, anchor-child_center))
    joint("PetioleJoint", base, petiole, np.array([0, 0, c.height]), petiole_pos,
          np.array([0, 0, c.height]), c.support_stiffness,
          c.leaf_mass*(c.petiole_length+c.length/2)**2+c.petiole_mass*c.petiole_length**2/3, 25.)
    for i in range(c.links):
        anchor = np.array([c.petiole_length+i*ds, 0, c.height])
        upstream_center = petiole_pos if i == 0 else centers[i-1]
        inertia = sum(c.leaf_mass/c.links*((k-i+0.5)*ds)**2 for k in range(i, c.links))
        joint(f"LeafJoint_{i}", bodies[i], bodies[i+1], upstream_center, centers[i],
              anchor, c.bend_stiffness, inertia, 30.)

    # Flat skeleton: each bone transform is supplied in world space. Physical
    # hierarchy remains the articulation; skin weights blend adjacent bones.
    skelroot = UsdSkel.Root.Define(stage, "/World/LeafVisual")
    skeleton = UsdSkel.Skeleton.Define(stage, "/World/LeafVisual/Skeleton")
    animation = UsdSkel.Animation.Define(stage, "/World/LeafVisual/Animation")
    names = [f"bone{i}" for i in range(c.links)]
    binds = [Gf.Matrix4d().SetTranslate(Gf.Vec3d(*v)) for v in centers]
    skeleton.CreateJointsAttr().Set(names)
    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(binds))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(binds))
    animation.CreateJointsAttr().Set(names)
    animation.CreateTranslationsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in centers]))
    animation.CreateRotationsAttr().Set(Vt.QuatfArray([Gf.Quatf(1.)]*c.links))
    animation.CreateScalesAttr().Set(Vt.Vec3hArray([Gf.Vec3h(1.)]*c.links))
    UsdSkel.BindingAPI.Apply(skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets([animation.GetPath()])
    mesh = UsdGeom.Mesh.Define(stage, "/World/LeafVisual/Blade")
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in points]))
    mesh.CreateFaceVertexCountsAttr().Set([3]*len(faces))
    mesh.CreateFaceVertexIndicesAttr().Set(faces.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr().Set("none")
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(.13, .4, .045)])
    binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    binding.CreateSkeletonRel().SetTargets([skeleton.GetPath()])
    binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.))
    binding.CreateJointIndicesPrimvar(False, 2).Set(indices.ravel().tolist())
    binding.CreateJointWeightsPrimvar(False, 2).Set(weights.ravel().tolist())

    target_x = c.petiole_length+c.target_fraction*c.length
    rigid("/World/Probe", [target_x, c.target_y, c.height+c.probe_radius+.025], c.probe_radius,
          .005, [1., .58, .05], sphere=True, kinematic=True)
    rigid("/World/Ball", [.15, .055, .055], c.ball_radius,
          c.ball_mass, [.08, .3, .9], sphere=True)
    ground = physicsUtils.add_box(stage, "/World/Table", size=Gf.Vec3f(.28, .20, .008),
        position=Gf.Vec3f(.06, 0, .036), color=Gf.Vec3f(.55, .52, .45))
    UsdPhysics.CollisionAPI.Apply(ground)
    UsdShade.MaterialBindingAPI.Apply(ground).Bind(material, materialPurpose="physics")
    physicsUtils.add_box(stage, "/World/Stand", size=Gf.Vec3f(.004, .004, c.height-.04),
        position=Gf.Vec3f(0, 0, (.04+c.height)/2), color=Gf.Vec3f(.25, .27, .29))
    lamp = UsdLux.DomeLight.Define(stage, "/World/Light")
    lamp.CreateIntensityAttr().Set(900)
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(.001, 10))
    camera.CreateFocalLengthAttr().Set(35.)
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([.17, -.20, .24]), target=np.array([.052, 0, .15]), camera_prim_path="/World/Camera")
    from omni.kit.viewport.utility import get_active_viewport
    get_active_viewport().camera_path = "/World/Camera"
    leaves = RigidPrim([str(b.GetPath()) for b in bodies], name="leaf_links",
                       reset_xform_properties=False, track_contact_forces=True,
                       contact_filter_prim_paths_expr=[[] for _ in bodies])
    probe = RigidPrim("/World/Probe", name="probe", reset_xform_properties=False)
    ball = RigidPrim("/World/Ball", name="ball", reset_xform_properties=False)
    for obj in (leaves, probe, ball):
        world.scene.add(obj)
    return leaves, probe, ball, animation, (points, faces, centers, indices, weights), joint_specs


def run(app, args, c):
    from isaacsim.core.api import World
    from pxr import Gf, Vt
    world = World(stage_units_in_meters=1., physics_dt=1/c.hz,
                  rendering_dt=1/60, backend="numpy", device="cpu")
    leaves, probe, ball, anim, geo, joint_specs = build(world, c)
    points, faces, centers, indices, weights = geo
    world.reset()
    world.get_physics_context().set_gravity(-9.81)
    world.stage.GetRootLayer().Export(str(args.run_dir/"scene.usda"))
    state = dict(press=0., drop=False, clear=False, demo=False, demo_start=0., quit=False)
    controls = {
        "press": lambda: state.update(press=1., demo=False),
        "release": lambda: state.update(press=0., demo=False),
        "drop": lambda: state.update(drop=True, demo=False),
        "clear": lambda: state.update(clear=True),
        "demo": lambda: state.update(demo=True, demo_start=float(world.current_time)),
        "quit": lambda: state.update(quit=True),
    }
    window = None
    label = None
    if args.gui:
        import omni.ui as ui
        window = ui.Window("Interactive leaf", width=390, height=270)
        with window.frame:
            with ui.VStack(spacing=8):
                ui.Label("Spring leaf | dynamic petiole | continuous skin")
                ui.Label(f"Orange sphere: press/release. Blue ball: {c.ball_mass*1000:g} g.")
                with ui.HStack():
                    ui.Button("Press leaf", clicked_fn=controls["press"])
                    ui.Button("Release", clicked_fn=controls["release"])
                with ui.HStack():
                    ui.Button("Drop ball", clicked_fn=controls["drop"])
                    ui.Button("Remove ball", clicked_fn=controls["clear"])
                ui.Button("Run automatic contact demo", clicked_fn=controls["demo"])
                label = ui.Label("Settling...")
                ui.Label(f"Physics: {c.hz} Hz. Use these controls to interact.")
                ui.Button("End session", clicked_fn=controls["quit"])
    poses0, quats0 = leaves.get_world_poses()
    save(args.run_dir/"initial_poses.json", dict(positions=np.asarray(poses0).tolist(), quaternions=np.asarray(quats0).tolist(), paths=leaves.prim_paths))
    expected_poses = np.vstack(([c.petiole_length/2, 0, c.height], centers))
    if not np.allclose(poses0, expected_poses, rtol=0, atol=1e-6):
        raise RuntimeError("Joint frames changed the initial geometry during zero-gravity reset")
    physics_start, step_start = float(world.current_time), int(world.current_time_step_index)
    rows, poses, quats = [], [], []
    start = time.perf_counter()
    last_phase = -1
    press = 0.
    snapshots = set()
    render_seconds = []
    step_seconds = []
    n = 0
    ui_events = []
    while app.is_running() and not state["quit"] and (args.gui and not args.gui_test or n < round(args.duration*c.hz)):
        if not world.is_playing():
            break
        t = n/c.hz
        phase = t % 20
        if args.gui_test:
            for second, action in ((2, "press"), (5, "release"), (9, "drop"), (13, "clear")):
                if n % (20*c.hz) == second*c.hz:
                    controls[action]()
                    ui_events.append(dict(time_s=t, action=action))
        if args.gui and not args.gui_test:
            if state["demo"]:
                phase = float(world.current_time)-state["demo_start"]
                if phase > 20:
                    state.update(demo=False, press=0.)
            else:
                phase = -1
        auto = not args.gui or state["demo"]
        target_press = press_fraction(phase) if auto and args.scenario in ("press", "cycle") else state["press"]
        press += np.clip(target_press-press, -1/(1.5*c.hz), 1/(1.5*c.hz))
        probe_position = [c.petiole_length+c.target_fraction*c.length, c.target_y,
                          c.height+c.probe_radius+.025-(.025+c.press_depth)*press]
        # USD pose changes produce a kinematic target. Tensor set_transforms
        # teleports the actor but leaves its previous kinematic target active.
        world.stage.GetPrimAtPath("/World/Probe").GetAttribute("xformOp:translate").Set(Gf.Vec3d(*map(float, probe_position)))
        drop = auto and args.scenario in ("drop", "cycle") and last_phase < 9 <= phase
        clear = auto and last_phase < 13 <= phase
        if drop or state["drop"]:
            ball.set_world_poses(positions=np.array([[c.petiole_length+c.length*.5, c.target_y, c.height+.05]], dtype=np.float32))
            ball.set_velocities(np.zeros((1, 6), dtype=np.float32))
            state["drop"] = False
        if clear or state["clear"]:
            ball.set_world_poses(positions=np.array([[.15, .055, .055]], dtype=np.float32))
            ball.set_velocities(np.zeros((1, 6), dtype=np.float32))
            state["clear"] = False
        last_phase = phase
        before = time.perf_counter()
        world.step(render=False)
        step_seconds.append(time.perf_counter()-before)
        actual_t = float(world.current_time)-physics_start
        if int(world.current_time_step_index)-step_start != n+1 or abs(actual_t-(n+1)/c.hz) > 2e-5:
            raise RuntimeError("Physics clock mismatch")
        position, quaternion = leaves.get_world_poses()
        position, quaternion = np.asarray(position), np.asarray(quaternion)
        r = rotations(quaternion)
        visual = skin(points, centers, indices, weights, position[1:], quaternion[1:])
        tip = visual[-9:].mean(axis=0)
        local_tip = r[0].T@(tip-position[0])
        # Joint anchor separation independently measures constraint stability.
        all_p = np.vstack(([0., 0., c.height], position))
        all_r = np.concatenate((np.eye(3)[None], r))
        errors = [np.linalg.norm(all_p[j]+all_r[j]@a-all_p[j+1]-all_r[j+1]@b)
                  for j, (a, b) in enumerate(joint_specs)]
        contact = np.asarray(leaves.get_net_contact_forces(dt=1/c.hz))
        contact_n = float(np.linalg.norm(contact, axis=1).sum())
        bp, _ = ball.get_world_poses()
        pp, _ = probe.get_world_poses()
        rows.append([actual_t, phase, press, *tip, *local_tip, max(errors), contact_n, *bp[0],
                     float(np.arccos(np.clip(r[0, 0, 0], -1, 1))), *pp[0]])
        poses.append(position.copy())
        quats.append(quaternion.copy())
        if not np.isfinite(visual).all() or max(errors) > .003 or np.linalg.norm(tip-[c.petiole_length, 0, c.height]) > c.length*1.5:
            np.savez_compressed(args.run_dir/"failed_trace.npz", samples=rows, positions=poses, quaternions=quats)
            raise RuntimeError(f"Unbounded motion or broken chain constraint: time={actual_t}, tip={tip.tolist()}, joint_errors={errors}, positions={position.tolist()}")
        if (args.gui or args.render) and n % (c.hz//60) == 0:
            anim.GetTranslationsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*map(float, v)) for v in position[1:]]))
            anim.GetRotationsAttr().Set(Vt.QuatfArray([Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:]))) for q in quaternion[1:]]))
            before = time.perf_counter()
            world.render()
            render_seconds.append(time.perf_counter()-before)
            if abs(float(world.current_time)-physics_start-actual_t) > 1e-7:
                raise RuntimeError("Render advanced physics")
            if label:
                label.text = f"Time {actual_t:.1f} s | tip {(tip[2]-c.height)*1000:.1f} mm | contact {contact_n:.3f} N"
            if args.render:
                for instant, name in ((1.8, "rest"), (4.8, "press"), (9.15, "drop"), (17., "recovered")):
                    if t >= instant and name not in snapshots:
                        from omni.kit.viewport.utility import capture_viewport_to_file, get_active_viewport
                        capture_viewport_to_file(get_active_viewport(), str(args.run_dir/(name+".png")))
                        if args.gui:
                            import omni.renderer_capture
                            omni.renderer_capture.acquire_renderer_capture_interface().capture_next_frame_swapchain(str(args.run_dir/("gui-"+name+".png")))
                        snapshots.add(name)
        if n % c.hz == 0:
            save(args.run_dir/"progress.json", dict(time_s=actual_t, tip_z_m=float(tip[2]), contact_n=contact_n))
        if args.gui:
            time.sleep(max(0., start+(n+1)/c.hz-time.perf_counter()))
        n += 1
    elapsed = time.perf_counter()-start
    if not rows:
        save(args.run_dir/"report.json", dict(status="failed", reason="Session ended before the first physics step"))
        return 1
    data = np.asarray(rows)
    np.savez_compressed(args.run_dir/"trace.npz", samples=data, positions=poses, quaternions=quats,
        columns=["time_s", "cycle_time_s", "press_fraction", "tip_x", "tip_y", "tip_z",
                 "local_tip_x", "local_tip_y", "local_tip_z", "joint_error_m", "contact_n",
                 "ball_x", "ball_y", "ball_z", "petiole_angle_rad", "probe_x", "probe_y", "probe_z"])
    save(args.run_dir/"runtime.json", dict(python=sys.executable, physics="PGS/CPU", hz=c.hz,
        articulation_links=c.links+2, rigid_leaf_links=c.links, skin_vertices=len(points),
        skin_triangles=len(faces), simulation_wall_seconds=elapsed,
        simulated_seconds=float(data[-1, 0]), mean_step_ms=1000*float(np.mean(step_seconds)),
        simulation_speed=float(data[-1, 0])/elapsed,
        mean_render_ms=1000*float(np.mean(render_seconds)) if render_seconds else None,
        rendered_fps=len(render_seconds)/elapsed if args.gui else None))
    from evaluate import evaluate
    if args.gui_test:
        save(args.run_dir/"ui_events.json", ui_events)
    report = evaluate(data, c, args.scenario, args.duration) if not args.gui or args.gui_test else dict(status="manual", user_review="pending")
    save(args.run_dir/"report.json", report)
    print(json.dumps(report, indent=2), flush=True)
    return 0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", type=Path, required=True)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--gui-test", action="store_true")
    p.add_argument("--render", action="store_true")
    p.add_argument("--scenario", default="cycle")
    p.add_argument("--duration", type=float, default=20)
    args = p.parse_args()
    c = Config(**json.loads((args.run_dir/"config.json").read_text()))
    c.validate()
    save(args.run_dir/"config.json", asdict(c))
    # Keep custom CLI flags away from Kit and use a bounded CPU worker count.
    sys.argv = [sys.argv[0], "--/plugins/carb.tasking.plugin/threadCount=4"]
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=not args.gui, width=1280, height=800, fast_shutdown=False))
    code = 1
    try:
        code = run(app, args, c)
    except Exception:
        error = traceback.format_exc()
        print(error, flush=True)
        save(args.run_dir/"report.json", dict(status="failed", error=error))
    finally:
        app.close()
    return code


if __name__ == "__main__":
    import os
    result = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(result)
