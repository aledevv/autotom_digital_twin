"""Isolated Isaac Sim 6.1 leaf bench. Run via run.py."""

import argparse
import json
import os
import sys
import time
import traceback
from dataclasses import asdict
from pathlib import Path

import numpy as np

from model import (
    Config,
    area,
    collider_mismatch,
    convex_strip,
    evaluate,
    first_contact_height,
    material_target,
    prism_mesh,
    ramp,
    rectangle,
    skin,
    skin_setup,
)


def save(path, data):
    path.write_text(json.dumps(data, indent=2, allow_nan=False) + "\n")


def build(
    world, c, model, p, f, shape_kind, prefix="/World", shared=True, create_view=True
):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdSkel, Vt

    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, "Z")
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    stage.SetDefaultPrim(UsdGeom.Xform.Define(stage, "/World").GetPrim())
    context = world.get_physics_context()
    context.enable_gpu_dynamics(model == "surface")
    context.set_broadphase_type("GPU" if model == "surface" else "MBP")
    context.set_solver_type("TGS" if model == "surface" else "PGS")
    context.set_gravity(0.0)

    def rigid(
        path,
        center,
        size,
        mass,
        hidden=False,
        sphere=False,
        kinematic=False,
        polygon=None,
    ):
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        body.AddOrientOp().Set(Gf.Quatf(1.0))
        prim = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(kinematic)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateSleepThresholdAttr().Set(0.0)
        rb.CreateSolverPositionIterationCountAttr().Set(c.iterations)
        if sphere:
            shape = UsdGeom.Sphere.Define(stage, path + "/Collider")
            shape.CreateRadiusAttr().Set(size)
        elif polygon is not None:
            cp, cf = prism_mesh(polygon, c.thickness)
            shape = UsdGeom.Mesh.Define(stage, path + "/Collider")
            shape.CreatePointsAttr().Set(Vt.Vec3fArray(cp.tolist()))
            shape.CreateFaceVertexCountsAttr().Set([3] * len(cf))
            shape.CreateFaceVertexIndicesAttr().Set(cf.ravel().tolist())
            UsdPhysics.MeshCollisionAPI.Apply(
                shape.GetPrim()
            ).CreateApproximationAttr().Set("convexHull")
        else:
            shape = UsdGeom.Cube.Define(stage, path + "/Collider")
            shape.CreateSizeAttr().Set(1.0)
            shape.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        coll = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        coll.CreateContactOffsetAttr().Set(c.contact_offset)
        coll.CreateRestOffsetAttr().Set(0.0)
        shape.CreateDisplayColorAttr().Set(
            [Gf.Vec3f(0.85, 0.45, 0.05) if sphere else Gf.Vec3f(0.25, 0.4, 0.12)]
        )
        if hidden:
            UsdGeom.Imageable(prim).MakeInvisible()
        return prim

    xmin = min(float(p[:, 0].min()), 0.0)
    support_center = np.array([(xmin + c.fixed_length) / 2, 0, c.height])
    support_width = max(c.width, 2 * float(abs(p[p[:, 0] <= c.fixed_length, 1]).max()))
    support = rigid(
        f"{prefix}/Petiole",
        support_center,
        [c.fixed_length - xmin + 0.0001, support_width + 0.002, 0.002],
        0.01,
    )
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{prefix}/FixedPetiole")
    fixed.CreateBody1Rel().SetTargets([support.GetPath()])
    fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(*support_center))
    fixed.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
    probe_prim = (
        rigid(
            f"{prefix}/Probe",
            [0.06, 0, c.height + 0.05],
            c.radius,
            0.01,
            sphere=True,
            kinematic=True,
        )
        if shared
        else None
    )
    UsdGeom.Xform.Define(stage, f"{prefix}/Leaf")
    mesh = UsdGeom.Mesh.Define(stage, f"{prefix}/Leaf/Mesh")
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(p.tolist()))
    mesh.CreateFaceVertexCountsAttr().Set([3] * len(f))
    mesh.CreateFaceVertexIndicesAttr().Set(f.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr().Set("none")
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.12, 0.42, 0.05)])
    mass = float(area(p, f).sum() * c.thickness * c.density)
    info = {"mass_kg": mass, "solver": "TGS/GPU" if model == "surface" else "PGS/CPU"}
    anim = None
    geo = None
    if model == "surface":
        from isaacsim.core.experimental.materials import SurfaceDeformableMaterial
        from isaacsim.core.experimental.prims import DeformablePrim
        from omni.physx import get_physx_cooking_interface
        from omni.physx.scripts import deformableUtils, physicsUtils

        assert deformableUtils.create_auto_surface_deformable_hierarchy(
            stage,
            f"{prefix}/Leaf",
            f"{prefix}/Leaf/Sim",
            f"{prefix}/Leaf/Mesh",
            False,
            True,
        )
        root = stage.GetPrimAtPath(f"{prefix}/Leaf")
        root.ApplyAPI("PhysxSurfaceDeformableBodyAPI")
        attrs = {
            "selfCollision": False,
            "solverPositionIterationCount": c.iterations,
            "enableSpeculativeCCD": True,
            "collisionPairUpdateFrequency": 4,
            "collisionIterationMultiplier": 4,
        }
        for name, value in attrs.items():
            root.GetAttribute("physxDeformableBody:" + name).Set(value)
        for name in ("sleepThreshold", "settlingThreshold"):
            attr = root.GetAttribute("physxDeformableBody:" + name)
            if attr:
                attr.Set(0.0)
        mat = SurfaceDeformableMaterial(
            f"{prefix}/LeafMaterial",
            youngs_moduli=c.young,
            poissons_ratios=c.poisson,
            densities=c.density,
            dynamic_frictions=0.3,
        )
        mat.set_surface_thicknesses(c.thickness)
        mat.set_surface_stiffnesses(
            stretch_stiffnesses=0.0, shear_stiffnesses=0.0, bend_stiffnesses=c.bend
        )
        mp = stage.GetPrimAtPath(f"{prefix}/LeafMaterial")
        mp.ApplyAPI("PhysxSurfaceDeformableMaterialAPI")
        mp.GetAttribute("physxDeformableMaterial:elasticityDamping").Set(c.damping)
        mp.GetAttribute("physxDeformableMaterial:bendDamping").Set(c.bend_damping)
        physicsUtils.add_physics_material_to_prim(stage, root, f"{prefix}/LeafMaterial")
        sim = stage.GetPrimAtPath(f"{prefix}/Leaf/Sim")
        coll = PhysxSchema.PhysxCollisionAPI.Apply(sim)
        coll.CreateRestOffsetAttr().Set(c.rest_offset)
        coll.CreateContactOffsetAttr().Set(c.contact_offset)
        get_physx_cooking_interface().cook_auto_deformable_body(f"{prefix}/Leaf")
        assert deformableUtils.create_auto_deformable_attachment(
            stage,
            root.GetPath().AppendChild("Attachment"),
            root.GetPath(),
            support.GetPath(),
        )
        leaf = DeformablePrim(f"{prefix}/Leaf", deformable_type="surface")
        info["material_authored"] = {
            a.GetName(): a.Get()
            for a in mp.GetAttributes()
            if a.GetName().startswith(("omniphysics:", "physxDeformableMaterial:"))
        }
    else:
        from isaacsim.core.prims import RigidPrim

        UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim())
        articulation = PhysxSchema.PhysxArticulationAPI.Apply(fixed.GetPrim())
        articulation.CreateEnabledSelfCollisionsAttr().Set(False)
        articulation.CreateSolverPositionIterationCountAttr().Set(c.iterations)
        articulation.CreateSolverVelocityIterationCountAttr().Set(4)
        articulation.CreateSleepThresholdAttr().Set(0.0)
        centers, indices, weights = skin_setup(p, c)
        polygons = None
        if shape_kind == "real":
            ds = (c.length - c.fixed_length) / 3
            polygons = [
                np.array(
                    [
                        [
                            -(c.fixed_length - xmin) / 2 - 0.00005,
                            -support_width / 2 - 0.001,
                        ],
                        [
                            (c.fixed_length - xmin) / 2 + 0.00005,
                            -support_width / 2 - 0.001,
                        ],
                        [
                            (c.fixed_length - xmin) / 2 + 0.00005,
                            support_width / 2 + 0.001,
                        ],
                        [
                            -(c.fixed_length - xmin) / 2 - 0.00005,
                            support_width / 2 + 0.001,
                        ],
                    ]
                )
            ]
            polygons += [
                convex_strip(
                    p,
                    f,
                    c.fixed_length + i * ds,
                    c.fixed_length + (i + 1) * ds,
                    centers[i + 1],
                )
                for i in range(3)
            ]
        geo = (centers, indices, weights, polygons)
        bodies = [support]
        ds = (c.length - c.fixed_length) / 3
        for i, center in enumerate(centers[1:]):
            bodies.append(
                rigid(
                    f"{prefix}/Link{i}",
                    center,
                    [ds, c.width, c.thickness],
                    mass / 3,
                    hidden=True,
                    polygon=None if polygons is None else polygons[i + 1],
                )
            )
        for i in range(3):
            anchor = np.array([c.fixed_length + i * ds, 0, c.height])
            j = UsdPhysics.Joint.Define(stage, f"{prefix}/Joint{i}")
            j.CreateBody0Rel().SetTargets([bodies[i].GetPath()])
            j.CreateBody1Rel().SetTargets([bodies[i + 1].GetPath()])
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(*(anchor - centers[i])))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(*(anchor - centers[i + 1])))
            j.CreateCollisionEnabledAttr().Set(False)
            for axis in (
                ("transX", "transY", "transZ", "rotZ", "rotX")
                if i == 0
                else ("transX", "transY", "transZ", "rotZ")
            ):
                limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
                limit.CreateLowAttr().Set(1.0)
                limit.CreateHighAttr().Set(-1.0)
            inertia = sum(mass / 3 * ((k - i + 0.5) * ds) ** 2 for k in range(i, 3))
            for axis in ("rotY",) if i == 0 else ("rotY", "rotX"):
                limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
                limit.CreateLowAttr().Set(-45.0)
                limit.CreateHighAttr().Set(45.0)
                drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
                drive.CreateTypeAttr().Set("force")
                drive.CreateStiffnessAttr().Set(c.joint_stiffness * np.pi / 180)
                drive.CreateDampingAttr().Set(
                    2
                    * c.damping_ratio
                    * np.sqrt(c.joint_stiffness * inertia)
                    * np.pi
                    / 180
                )
                drive.CreateTargetPositionAttr().Set(0.0)
            # Exclude adjacent contact, including the root band.
        UsdSkel.Root.Define(stage, f"{prefix}/Leaf")
        skeleton = UsdSkel.Skeleton.Define(stage, f"{prefix}/Leaf/Skeleton")
        anim = UsdSkel.Animation.Define(stage, f"{prefix}/Leaf/Animation")
        names = [f"bone{i}" for i in range(4)]
        binds = Vt.Matrix4dArray(
            [Gf.Matrix4d().SetTranslate(Gf.Vec3d(*v)) for v in centers]
        )
        skeleton.CreateJointsAttr().Set(names)
        skeleton.CreateBindTransformsAttr().Set(binds)
        skeleton.CreateRestTransformsAttr().Set(binds)
        anim.CreateJointsAttr().Set(names)
        anim.CreateScalesAttr().Set(Vt.Vec3hArray([Gf.Vec3h(1.0)] * 4))
        UsdSkel.BindingAPI.Apply(
            skeleton.GetPrim()
        ).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
        binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
        binding.CreateSkeletonRel().SetTargets([skeleton.GetPath()])
        binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))
        binding.CreateJointIndicesPrimvar(False, 2).Set(indices.ravel().tolist())
        binding.CreateJointWeightsPrimvar(False, 2).Set(weights.ravel().tolist())
        leaf = (
            RigidPrim(
                [str(b.GetPath()) for b in bodies],
                name="leaf_links",
                reset_xform_properties=False,
            )
            if create_view
            else None
        )
        # Read-only rigid view; do not reset individual articulation links.
    if not shared:
        return leaf, None, anim, geo, info
    probe = probe_prim.GetAttribute("xformOp:translate")
    UsdLux.DomeLight.Define(stage, "/World/Light").CreateIntensityAttr().Set(1000.0)
    cam = UsdGeom.Camera.Define(stage, "/World/Camera")
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.001, 10))
    from isaacsim.core.utils.viewports import set_camera_view

    set_camera_view(
        eye=np.array([0.16, -0.19, 0.25]),
        target=np.array([0.045, 0, c.height]),
        camera_prim_path="/World/Camera",
    )
    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport is not None:
        viewport.camera_path = "/World/Camera"
    return leaf, probe, anim, geo, info


def run(app, a, c):
    print("BENCH: importing core", flush=True)
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from pxr import Gf, UsdGeom, Vt

    enable_extension("isaacsim.core.experimental.prims")
    enable_extension("isaacsim.core.experimental.materials")
    p, f = rectangle(c)
    if a.shape == "real":
        data = np.load(a.run_dir / "input_mesh.npz")
        p = data["points"]
        f = data["faces"]
    print("BENCH: creating world", flush=True)
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1 / c.hz,
        rendering_dt=1 / 60,
        backend="torch" if a.model == "surface" else "numpy",
        device="cuda:0" if a.model == "surface" else "cpu",
    )
    print("BENCH: building scene", flush=True)
    leaf, probe, anim, geo, info = build(world, c, a.model, p, f, a.shape)
    print("BENCH: resetting", flush=True)
    world.reset()
    if a.model == "skinning":
        leaf.initialize()
    print("BENCH: reset complete", flush=True)

    def read():
        if a.model == "surface":
            return leaf.get_nodal_positions()[0].numpy()[0].copy()
        positions, quaternions = leaf.get_world_poses()
        centers, indices, weights, _polygons = geo
        anim.CreateTranslationsAttr().Set(Vt.Vec3fArray(positions.tolist()))
        anim.CreateRotationsAttr().Set(
            Vt.QuatfArray(
                [
                    Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:])))
                    for q in quaternions
                ]
            )
        )
        return skin(p, centers, indices, weights, positions, quaternions)

    # One non-gravitating step initializes experimental tensor views.
    world.step(render=False)
    rest = read()
    probe_view = world.physics_sim_view.create_rigid_body_view("/World/Probe")

    def actual_probe():
        value = probe_view.get_transforms()
        if hasattr(value, "cpu"):
            value = value.cpu().numpy()
        elif hasattr(value, "numpy"):
            value = value.numpy()
        return np.asarray(value)[0, :3].copy()

    if a.model == "surface":
        sim = UsdGeom.Mesh.Get(world.stage, "/World/Leaf/Sim")
        f = np.asarray(sim.GetFaceVertexIndicesAttr().Get(), dtype=np.int32).reshape(
            -1, 3
        )
        if len(rest) != len(p):
            raise RuntimeError("Unexpected cooking vertex count")
        info["cooked_mesh_vertices"] = len(rest)
    np.savez_compressed(a.run_dir / "rest_mesh.npz", points=rest, faces=f)
    world.stage.GetRootLayer().Export(str(a.run_dir / "scene.usda"))
    import subprocess

    gpu = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
        check=False,
        capture_output=True,
        text=True,
    )
    info.update(
        python=sys.executable,
        isaac_version=(Path.home() / "isaacsim-6.1/VERSION").read_text().strip(),
        gpu=gpu.stdout.strip(),
        config=asdict(c),
        model=a.model,
        shape=a.shape,
    )
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    info["physx_extensions"] = [
        e["id"] for e in manager.get_extensions() if e["id"].startswith("omni.physx-")
    ]
    save(a.run_dir / "runtime.json", info)
    poses = []
    mismatch = []
    trace = [rest]
    times = [0.0]
    probes = [actual_probe()]
    probe_commands = [probes[0].copy()]
    cycles = []
    active = None
    cycle_number = 0
    step_seconds = read_seconds = render_seconds = 0.0
    frames = 0
    state = {"command": None, "quit": False, "manual": False, "manual_used": False}
    window = None
    if a.gui:
        from omni import ui

        window = ui.Window("Leaf 6.1", width=340, height=180)
        with window.frame, ui.VStack():
            ui.Label(f"{a.model} | fixed petiole | {c.hz} Hz")
            ui.Button(
                "Press",
                clicked_fn=lambda: state.update(
                    command="press", manual=True, manual_used=True
                ),
            )
            ui.Button(
                "Release",
                clicked_fn=lambda: state.update(
                    command="release", manual=True, manual_used=True
                ),
            )
            ui.Button("Repeat test", clicked_fn=lambda: state.update(command="repeat"))
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    limit = 1 + c.settle + (6 if a.scenario == "cycle" else 1) * (5 + c.recovery)
    if a.scenario == "smoke":
        limit = 2.0
    if a.scenario == "rest":
        limit = 1 + c.settle
    base_step = world.current_time_step_index
    base_time = float(world.current_time)
    start = time.perf_counter()
    done = False
    abort_reason = None
    n = 0
    clearance = 0.004
    center = probes[0].copy()
    manual_release = None
    while app.is_running() and not state["quit"]:
        if not world.is_playing():
            abort_reason = "Timeline stopped before completion"
            break
        t = n / c.hz
        if t >= limit - 1e-9 and not a.gui:
            done = True
            break
        if a.gui and t >= limit - 1e-9:
            done = True
        if n == c.hz:
            world.get_physics_context().set_gravity(-9.81)
        command = state.pop("command", None)
        if command == "repeat":
            state["manual"] = False
            cycle_number = 0
            active = None
            manual_release = None
            done = False
            state["parking"] = (t, center.copy())
            limit = (
                t
                + 2
                + c.settle
                + (6 if a.scenario == "cycle" else 1) * (5 + c.recovery)
            )
            state["restart"] = t + 2 + c.settle
        begin = 1 + c.settle if "restart" not in state else state["restart"]
        if command == "press":
            state.pop("parking", None)
        if command == "press" and active is None:
            state["restart"] = t
            begin = t
            cycle_number = 0
        if (
            a.scenario in ("cycle", "press")
            and t >= begin
            and (active is None)
            and cycle_number < (6 if a.scenario == "cycle" else 1)
        ):
            y = 0.0 if cycle_number < 3 else 0.01
            x = c.fixed_length + 0.65 * (c.length - c.fixed_length)
            location = "central" if y == 0 else "offcenter"
            try:
                target = material_target(rest, trace[-1], f, x, y)
                x, y = float(target[0]), float(target[1])
                z = first_contact_height(trace[-1], f, x, y, c.radius)
            except ValueError as exc:
                abort_reason = str(exc)
                break
            active = {
                "start": t,
                "end": t + 5 + c.recovery,
                "location": location,
                "baseline_index": len(trace) - 1,
                "x": x,
                "y": y,
                "z": z,
            }
            cycle_number += 1
        if active:
            if command == "press":
                active["start"] = t
            dt = t - active["start"]
            fraction = (
                ramp(dt / 2) if dt < 2 else (1.0 if dt < 3 else 1 - ramp((dt - 3) / 2))
            )
            if state["manual"]:
                if command == "release":
                    manual_release = (t, float(center[2]))
                if command == "press":
                    manual_release = None
                if manual_release:
                    z = manual_release[1] + ramp((t - manual_release[0]) / 2) * (
                        active["z"] + clearance - manual_release[1]
                    )
                else:
                    z = active["z"] + clearance - ramp(dt / 2) * (c.depth + clearance)
            else:
                z = active["z"] + clearance - fraction * (c.depth + clearance)
            center = np.array([active["x"], active["y"], z])
            if dt >= 5 + c.recovery - 1 / c.hz - 1e-8 and not state["manual"]:
                cycles.append(active)
                print(f"BENCH: completed contact {len(cycles)} at {t:.2f}s", flush=True)
                active = None
        if "parking" in state:
            park_start, park_origin = state["parking"]
            center = park_origin + ramp((t - park_start) / 2) * (
                np.array([0.06, 0, c.height + 0.05]) - park_origin
            )
            if t - park_start >= 2:
                state.pop("parking")
        probe.Set(Gf.Vec3d(*map(float, center)))
        target_pose = np.array([[*center, 0.0, 0.0, 0.0, 1.0]], dtype=np.float32)
        target_ids = np.array([0], dtype=np.int32)
        if a.model == "surface":
            import torch

            target_pose = torch.as_tensor(target_pose, device="cuda:0")
            target_ids = torch.as_tensor(target_ids, device="cuda:0")
        if a.model == "surface":
            # PhysX 110.3.2 GPU view does not implement set_kinematic_targets.
            # Prescribe the quasi-static pose each substep and verify readback.
            probe_view.set_transforms(target_pose, target_ids)
        else:
            probe_view.set_kinematic_targets(target_pose, target_ids)
        tick = time.perf_counter()
        world.step(render=False)
        step_seconds += time.perf_counter() - tick
        if world.current_time_step_index != base_step + n + 1:
            raise RuntimeError("Physics step count mismatch")
        tick = time.perf_counter()
        points = read()
        read_seconds += time.perf_counter() - tick
        if a.model == "skinning":
            pos, quat = leaf.get_world_poses()
            poses.append(np.concatenate((pos, quat), axis=1))
            if n % max(1, c.hz // 10) == 0:
                mismatch.append(collider_mismatch(c, points, pos, quat, geo[3]))
        trace.append(points)
        times.append((n + 1) / c.hz)
        probes.append(actual_probe())
        probe_commands.append(center.copy())
        n += 1
        if (
            not np.isfinite(points).all()
            or np.max(np.linalg.norm(points - rest, axis=1)) > 2 * c.length
        ):
            break
        if (a.gui or a.render) and n % max(1, c.hz // 60) == 0:
            tick = time.perf_counter()
            world.render()
            render_seconds += time.perf_counter() - tick
            frames += 1
        if a.gui:
            time.sleep(max(0.0, start + n / c.hz - time.perf_counter()))
    if active and times[-1] >= active["end"] - 1 / c.hz and not state["manual"]:
        cycles.append(active)
    loop_seconds = time.perf_counter() - start
    clock_elapsed = float(world.current_time) - base_time
    np.savez_compressed(
        a.run_dir / "trace.npz",
        points=np.asarray(trace),
        times=times,
        probe=probes,
        probe_commands=probe_commands,
        rigid_poses=np.asarray(poses),
    )
    if a.scenario in ("smoke", "rest"):
        report = evaluate(c, rest, f, trace, times, probes, [], complete=done)
        report["status"] = "diagnostic"
        report["checks"].pop("cycles", None)
    else:
        report = evaluate(
            c,
            rest,
            f,
            trace,
            times,
            probes,
            cycles,
            complete=done and len(cycles) >= (6 if a.scenario == "cycle" else 1),
        )
    if state["manual_used"]:
        report["status"] = (
            "manual"
            if all(
                report.get("checks", {}).get(k, False)
                for k in ("finite", "attachment", "stretch", "noncollapsed")
            )
            else "failed"
        )
        report["visual_acceptance"] = "pending"
    if not np.isclose(clock_elapsed, times[-1], rtol=5e-5, atol=5e-5):
        report.update(status="failed", reason="Physics clock mismatch")
    if mismatch:
        report.setdefault("metrics", {})["visual_collider_sampled_max_gap_m"] = max(
            mismatch
        )
    report["collider_gap_sampling_hz"] = 10 if a.model == "skinning" else None
    if abort_reason:
        report.update(status="failed", reason=abort_reason)
    report["timing"] = {
        "physics_seconds": step_seconds,
        "read_seconds": read_seconds,
        "render_seconds": render_seconds,
        "simulated_seconds": times[-1],
        "physics_realtime_factor": times[-1] / max(step_seconds, 1e-9),
        "instrumented_realtime_factor": times[-1] / max(loop_seconds, 1e-9),
        "loop_seconds": loop_seconds,
        "physics_clock_elapsed": clock_elapsed,
        "rendered_frames": frames,
        "gui_fps": frames / loop_seconds if a.gui else None,
    }
    report["probe_tracking_error_m"] = float(
        np.linalg.norm(np.asarray(probes) - np.asarray(probe_commands), axis=1).max()
    )
    if report["probe_tracking_error_m"] > 1e-4:
        report.update(
            status="failed", reason="Probe does not follow commanded trajectory"
        )
    report["cycles"] = report.get("cycles", [])
    save(a.run_dir / "report.json", report)
    print(json.dumps(report), flush=True)
    if a.render and not a.gui:
        # Capture only after measurements, with physics paused.
        import asyncio

        from omni.kit.viewport.utility import (
            capture_viewport_to_file,
            get_active_viewport,
        )

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
            raise RuntimeError("Viewport capture did not complete")
        task.result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--model", choices=["surface", "skinning"], default="surface")
    parser.add_argument(
        "--scenario", choices=["smoke", "rest", "press", "cycle"], default="cycle"
    )
    parser.add_argument("--shape", choices=["rectangle", "real"], default="rectangle")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--gui-benchmark", action="store_true")
    parser.add_argument("--leaves", type=int)
    parser.add_argument(
        "--layout",
        choices=["plant", "contact-pair", "drop-stack", "canopy", "canopy-drop"],
        default="plant",
    )
    parser.add_argument("--diagnostics", choices=["full", "light"], default="full")
    parser.add_argument("--pair-collisions", choices=["on", "off"], default="on")
    parser.add_argument(
        "--ball-mass", type=float, default=0.005, help="Dynamic ball mass in kg"
    )
    parser.add_argument(
        "--drop-height",
        type=float,
        default=0.06,
        help="Initial ball-surface clearance in metres",
    )
    parser.add_argument("--leaf-gap", type=float, default=0.06)
    parser.add_argument("--lower-offset", type=float, default=0.075)
    parser.add_argument("--lower-y", type=float, default=0.0)
    parser.add_argument("--ball-x", type=float, default=0.055)
    parser.add_argument("--playback-speed", type=float, default=1.0)
    parser.add_argument(
        "--drop-runtime",
        choices=["baseline", "optimized", "batched", "visual"],
        default="baseline",
    )
    parser.add_argument(
        "--drop-test", choices=["single", "scale", "cascade"], default="single"
    )
    parser.add_argument("--render-hz", type=int, choices=[20, 30, 60], default=60)
    parser.add_argument("--cascade-layout", type=Path)
    parser.add_argument("--analysis-reference", type=Path)
    parser.add_argument(
        "--geometry-method", choices=["exact", "bounded"], default="exact"
    )
    parser.add_argument("--plant-copies", type=int, default=1)
    parser.add_argument(
        "--collision-filter", choices=["pairs", "groups"], default="pairs"
    )
    parser.add_argument("--visual-tolerance", type=float, default=0.0)
    parser.add_argument("--skin-batch-size", type=int, default=0)
    parser.add_argument("--sleep-threshold", type=float, default=0.0)
    parser.add_argument("--stress-all", action="store_true")
    parser.add_argument("--rain", action="store_true")
    parser.add_argument("--storm", action="store_true")
    parser.add_argument("--storm-rate", type=float, default=200.0)
    parser.add_argument("--storm-gain", type=float, default=1.0)
    parser.add_argument(
        "--storm-load-hz", type=int, choices=[60, 120, 480], default=120
    )
    parser.add_argument("--rain-count", type=int, default=24)
    parser.add_argument("--rain-seed", type=int, default=42)
    parser.add_argument("--rain-waves", default="")
    parser.add_argument(
        "--canopy-selection", choices=["nearby", "primary"], default="nearby"
    )
    parser.add_argument("--skin-writes", choices=["usd", "sdf"], default="usd")
    parser.add_argument("--solver", choices=["PGS", "TGS"], default="PGS")
    parser.add_argument("--recording", choices=["full", "first-trial"], default="full")
    parser.add_argument("--tail-seconds", type=float, default=0.0)
    parser.add_argument("--physics-threads", type=int, choices=range(33), default=0)
    a = parser.parse_args()
    c = Config(**json.loads((a.run_dir / "config.json").read_text()))
    c.validate()
    import faulthandler

    faulthandler.enable()
    faulthandler.dump_traceback_later(180, repeat=True)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": not a.gui,
            "width": 1280,
            "height": 720 if a.leaves is not None else 800,
            "renderer": "RaytracedLighting",
            "disable_viewport_updates": not (a.gui or a.render),
            "fast_shutdown": True,
            "create_new_stage": False,
        }
    )
    import carb
    import omni.usd

    settings = carb.settings.get_settings()
    if a.physics_threads:
        settings.set_int("/persistent/physics/numThreads", a.physics_threads)
    save(
        a.run_dir / "thread_settings.json",
        {
            key: settings.get(key)
            for key in (
                "/persistent/physics/numThreads",
                "/physics/physxDispatcher",
                "/plugins/carb.tasking.plugin/threadCount",
                "/plugins/omni.tbb.globalcontrol/maxThreadCount",
            )
        },
    )
    omni.usd.get_context().new_stage()
    save(a.run_dir / "config.json", asdict(c))
    code = 0
    try:
        if a.layout in ("drop-stack", "canopy-drop"):
            from drop_scene import run as run_drop

            run_drop(app, a, c)
        elif a.leaves is not None:
            from plant_scene import run as run_plant

            run_plant(app, a, c)
        else:
            run(app, a, c)
    except Exception:  # noqa: BLE001 -- preserve failure evidence at worker boundary
        traceback.print_exc()
        code = 1
    finally:
        app.close(wait_for_replicator=False, exit_code=code)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


if __name__ == "__main__":
    main()
