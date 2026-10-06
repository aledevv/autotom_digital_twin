"""Native dynamic plant + three-link blades. Independent of the production exporter."""

import argparse
import json
import os
import subprocess
import time
import traceback
from dataclasses import asdict
from pathlib import Path

import numpy as np
from scene import build, save

from model import Config, area, rotation, skin


def run(app, a):
    import carb
    import omni.usd
    from isaacsim.core.api import World
    from omni.physx import get_physx_interface
    from pxr import Gf, UsdGeom, UsdPhysics, UsdSkel, Vt

    started = time.perf_counter()
    omni.usd.get_context().open_stage(str(a.run_dir / "input.usda"))
    stage = omni.usd.get_context().get_stage()
    # All edits are saved to a new scene; the immutable input remains untouched.
    world = World(
        physics_dt=1 / a.physics_hz,
        rendering_dt=1 / 30,
        backend="numpy",
        device="cpu",
        physics_prim_path="/World/PhysicsScene",
    )
    cache = UsdGeom.XformCache()
    native = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    mass_before = sum(
        float(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0) for p in native
    )
    originals = [p for p in stage.Traverse() if p.GetName() == "LeafBlade"][: a.leaves]
    rows = []
    host_updates = {}

    def transform(prim):
        m = np.asarray(cache.GetLocalToWorldTransform(prim))
        return m[:3, :3].T.copy(), m[3, :3].copy()

    def quat(r):
        return Gf.Matrix3d(*r.T.ravel().tolist()).ExtractRotation().GetQuat()

    for i, original in enumerate(originals):
        host = original.GetParent()
        while not host.HasAPI(UsdPhysics.RigidBodyAPI):
            host = host.GetParent()
        hr, ht = transform(host)
        mr, mt = transform(original)
        vr, _vt = transform(original.GetParent())
        mesh = UsdGeom.Mesh(original)
        source_points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        if not np.all(counts == 3):
            raise RuntimeError("Expected native triangular blades")
        faces = np.asarray(
            mesh.GetFaceVertexIndicesAttr().Get(), dtype=np.int32
        ).reshape(-1, 3)
        base = np.asarray(original.GetAttribute("autotom:leafShapeBase").Get())
        origin = mr @ base + mt
        # The native petiolule frame has longitudinal +Z, lateral -Y.
        # This also recovers the original mass-lumping point exactly.
        frame = np.column_stack((vr[:, 2], -vr[:, 1], vr[:, 0]))
        if a.leaf_profile == "soft":
            # A cylindrical petiolule's roll is not the blade's transverse axis.
            # Align bending to the actual native sheet, keeping its forward axis.
            world_points = source_points @ mr.T + mt
            triangles = world_points[faces]
            normal = np.cross(
                triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
            ).sum(axis=0)
            lateral = np.cross(normal, frame[:, 0])
            if np.linalg.norm(lateral) < 1e-12:
                raise RuntimeError("Cannot derive the native blade plane")
            lateral /= np.linalg.norm(lateral)
            frame = np.column_stack(
                (frame[:, 0], lateral, np.cross(frame[:, 0], lateral))
            )
        points = ((source_points @ mr.T + mt) - origin) @ frame
        # Recover the exporter length before its world-vertical tip sag.
        # The pinned source uses L=min(.09,max(.04,petiolule_length*5.35)).
        length = min(0.09, max(0.04, float(np.linalg.norm(base)) * 5.35))
        if np.linalg.norm(base[:2]) > 1e-6:
            raise RuntimeError("Native leaf basal frame contract changed")
        leaf_mass = float(
            original.GetParent().GetAttribute("autotom:aggregatedMassKg").Get() or 0
        )
        if leaf_mass <= 0 or length <= 0:
            raise RuntimeError(
                f"Missing native biomass/length for {original.GetPath()}"
            )
        c = Config(
            hz=a.physics_hz,
            iterations=a.iterations,
            joint_stiffness=0.003 if a.leaf_profile == "soft" else 0.024,
            damping_ratio=1.3 if a.leaf_profile == "soft" else 1.0,
            height=0,
            length=length,
            width=float(np.ptp(points[:, 1])),
            fixed_length=0.0 if a.leaf_profile == "soft" else length / 15,
            density=leaf_mass / (float(area(points, faces).sum()) * 0.0005),
        )
        prefix = f"/World/FlexibleLeaves/Leaf_{i:03d}"
        if a.leaf_profile == "soft":
            from soft_leaf import build_soft

            _, _, anim, geo, _info = build_soft(world, c, points, faces, prefix)
        else:
            _, _, anim, geo, _info = build(
                world,
                c,
                "skinning",
                points,
                faces,
                "real",
                prefix=prefix,
                shared=False,
                create_view=False,
            )
        centers, indices, weights, _ = geo
        # Native blades are already curved. A hard bone switch across the
        # basal hinge tears short edges with nonzero normal offset. Blend from
        # the fixed band to the first segment center, preserving the fixed band.
        if a.leaf_profile == "original":
            blend = (points[:, 0] > c.fixed_length) & (points[:, 0] < centers[1, 0])
            fraction = (points[blend, 0] - c.fixed_length) / (
                centers[1, 0] - c.fixed_length
            )
            indices[blend] = [0, 1]
            weights[blend] = np.column_stack((1 - fraction, fraction))
            binding = UsdSkel.BindingAPI(stage.GetPrimAtPath(prefix + "/Leaf/Mesh"))
            binding.GetJointIndicesPrimvar().Set(indices.ravel().tolist())
            binding.GetJointWeightsPrimvar().Set(weights.ravel().tolist())
        rest_q = quat(frame)
        count = len(centers) - 1
        rest_quats = np.tile([rest_q.GetReal(), *rest_q.GetImaginary()], (count + 1, 1))
        reconstructed = skin(
            points, centers, indices, weights, centers @ frame.T + origin, rest_quats
        )
        visual_error = float(
            np.linalg.norm(reconstructed - (source_points @ mr.T + mt), axis=1).max()
        )
        if visual_error > 1e-6:
            raise RuntimeError("Initial skinned mesh differs from native geometry")
        stage.RemovePrim(prefix + "/FixedPetiole")
        stage.RemovePrim(prefix + "/Petiole")
        for j in range(count):
            link = stage.GetPrimAtPath(prefix + f"/Link{j}")
            link.GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(*(origin + frame @ centers[j + 1]))
            )
            link.GetAttribute("xformOp:orient").Set(Gf.Quatf(quat(frame)))
        # Extend the native articulation tree: the basal constraint is solved
        # together with the host, avoiding external-joint drift at tiny masses.
        joint = UsdPhysics.Joint.Get(stage, prefix + "/Joint0")
        joint.CreateBody0Rel().SetTargets([host.GetPath()])
        joint.CreateExcludeFromArticulationAttr().Set(False)
        anchor = origin + frame @ np.array([c.fixed_length, 0, 0])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(hr.T @ (anchor - ht))))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(quat(hr.T @ frame)))
        UsdGeom.Imageable(original).MakeInvisible()
        # Transfer the already aggregated mass AND its moment out of the host.
        hp = str(host.GetPath())
        if hp not in host_updates:
            ma = UsdPhysics.MassAPI(host)
            mass = float(ma.GetMassAttr().Get())
            com = np.asarray(ma.GetCenterOfMassAttr().Get(), dtype=float)
            host_updates[hp] = [mass, mass * com, 0.0]
        lump = hr.T @ (origin + frame[:, 0] * 0.4 * length - ht)
        host_updates[hp][0] -= leaf_mass
        host_updates[hp][1] -= leaf_mass * lump
        host_updates[hp][2] += leaf_mass
        rows.append(
            {
                "prefix": prefix,
                "segments": count,
                "initial_frame": frame,
                "initial_origin": origin,
                "profile": _info,
                "host": hp,
                "source": str(original.GetPath()),
                "mass_kg": leaf_mass,
                "initial_visual_error_m": visual_error,
                "config": asdict(c),
                "points": points,
                "faces": faces,
                "centers": centers,
                "indices": indices,
                "weights": weights,
                "host_offset": hr.T @ (origin + frame @ centers[0] - ht),
                "host_frame": hr.T @ frame,
                "channels": (anim.CreateTranslationsAttr(), anim.CreateRotationsAttr()),
                "anchor": hr.T @ (anchor - ht),
            }
        )
    for hp, (mass, moment, transferred) in host_updates.items():
        if mass <= 0 or not np.isfinite(moment).all():
            raise RuntimeError(
                "Mass redistribution would invalidate native host: " + hp
            )
        host = stage.GetPrimAtPath(hp)
        ma = UsdPhysics.MassAPI(host)
        ma.GetMassAttr().Set(mass)
        ma.GetCenterOfMassAttr().Set(Gf.Vec3f(*(moment / mass)))
        attr = host.GetAttribute("autotom:aggregatedLeafVisualMassKg")
        attr.Set(float(attr.Get() or 0) - transferred)
    if a.leaf_armature:
        from pxr import PhysxSchema

        for r in rows:
            for j in range(r["segments"]):
                prim = stage.GetPrimAtPath(r["prefix"] + f"/Joint{j}")
                PhysxSchema.PhysxJointAPI.Apply(prim).CreateArmatureAttr().Set(
                    a.leaf_armature
                )
                for axis in ("rotX", "rotY"):
                    drive = UsdPhysics.DriveAPI(prim, axis)
                    if not drive:
                        continue
                    k = float(drive.GetStiffnessAttr().Get() or 0) * 180 / np.pi
                    damping = float(drive.GetDampingAttr().Get() or 0) * 180 / np.pi
                    drive.GetDampingAttr().Set(
                        float(
                            np.sqrt(damping**2 + 4 * 1.2**2 * k * a.leaf_armature)
                            * np.pi
                            / 180
                        )
                    )
    mass_after = sum(
        float(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0)
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    )
    if not np.isclose(mass_before, mass_after, rtol=1e-7, atol=1e-9):
        raise RuntimeError("Total native mass changed")
    from full_plant_contacts import ContactMonitor
    from full_plant_contacts import configure as configure_contacts

    contact_policy = configure_contacts(stage, native, rows, a.leaf_contacts)
    if a.exclude_initial_from:
        from full_plant_contacts import exclude_initial_pairs

        contact_policy["excluded_initial_overlaps"] = exclude_initial_pairs(
            stage, json.loads(a.exclude_initial_from.read_text())
        )
    save(a.run_dir / "contact_policy.json", contact_policy)
    probe = None
    if a.contact_probe:
        from soft_probe import ContactProbe

        probe = ContactProbe(stage, rows[0], rows, native)
    if a.diagnostics == "live":
        from pxr import PhysxSchema

        for r in rows:
            for j in range(r["segments"]):
                stage.GetPrimAtPath(r["prefix"] + f"/Link{j}").RemoveAPI(
                    PhysxSchema.PhysxContactReportAPI
                )
        if probe:
            stage.GetPrimAtPath(probe.path).RemoveAPI(PhysxSchema.PhysxContactReportAPI)
    contacts = (
        ContactMonitor(sample_hz=10 if a.diagnostics == "light" else None)
        if (a.leaf_contacts != "off" or probe) and a.diagnostics != "live"
        else None
    )
    settings = carb.settings.get_settings()
    settings.set_bool("/physics/updateToUsd", False)
    settings.set_bool("/physics/updateVelocitiesToUsd", False)
    context = world.get_physics_context()
    partition = None
    if a.partition_trunk or a.partition_branches:
        from partition_plant import partition_fixed_trunk

        partition = partition_fixed_trunk(stage, a.partition_branches)
        save(a.run_dir / "partition.json", partition)
    fabric = None
    if a.fabric:
        context.enable_fabric(True)
        from omni.physxfabric import get_physx_fabric_interface

        fabric = get_physx_fabric_interface()
    if a.gpu_physics:
        context.set_gpu_found_lost_pairs_capacity(2**20)
        context.set_gpu_found_lost_aggregate_pairs_capacity(2**20)
        context.set_gpu_total_aggregate_pairs_capacity(2**20)
        context.enable_gpu_dynamics(True)
        context.set_broadphase_type("GPU")
        settings.set_bool("/physics/suppressReadback", False)
    context.set_solver_type(a.solver)
    from pxr import PhysxSchema

    settings.set_int("/persistent/physics/numThreads", a.physics_threads)
    for prim in stage.Traverse():
        if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
            api = PhysxSchema.PhysxArticulationAPI(prim)
            api.CreateSolverVelocityIterationCountAttr().Set(4)
            api.CreateSolverPositionIterationCountAttr().Set(a.iterations)
            api.CreateSleepThresholdAttr().Set(a.sleep_threshold)
        if prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
            api = PhysxSchema.PhysxRigidBodyAPI(prim)
            api.CreateSolverPositionIterationCountAttr().Set(a.iterations)
            api.CreateSleepThresholdAttr().Set(a.sleep_threshold)
    context.set_gravity(-9.81)
    world.reset()
    av = world.physics_sim_view.create_articulation_view(
        [r["joint"] for r in partition["independent_roots"]]
        if partition
        else "/World/Stem"
    )
    save(
        a.run_dir / "articulations.json",
        {
            "count": av.count,
            "max_links": av.max_links,
            "max_dofs": av.max_dofs,
            "paths": list(av.prim_paths),
        },
    )
    if a.leaf_armature:
        armatures = np.asarray(av.get_dof_armatures())
        save(
            a.run_dir / "armature_validation.json",
            {
                "nonzero_dofs": int(np.count_nonzero(armatures)),
                "maximum_kg_m2": float(armatures.max()),
                "requested_kg_m2": a.leaf_armature,
            },
        )
        if not np.any(armatures > 0):
            raise RuntimeError("Authored armature is not active in the solver")
    if a.gui:
        from mouse_grab import configure

        save(a.run_dir / "mouse_interaction.json", configure(app))
    paths = sorted(
        set(
            [r["host"] for r in rows]
            + [r["prefix"] + f"/Link{j}" for r in rows for j in range(r["segments"])]
        )
    )
    view = world.physics_sim_view.create_rigid_body_view(paths)
    order = list(view.prim_paths)
    for r in rows:
        r["ids"] = [
            order.index(r["prefix"] + f"/Link{j}") for j in range(r["segments"])
        ]
        r["host_id"] = order.index(r["host"])
    skin_batch = None
    if a.skin_batch:
        from full_plant_skin import NativeSkinBatch

        skin_batch = NativeSkinBatch(stage, rows, a.skin_batch)
        initial_poses = np.asarray(view.get_transforms()).copy()
        skin_batch.validate(initial_poses, rotation(initial_poses[:, [6, 3, 4, 5]]))
    # Include all native links in the finite/bounds check, even unmodified ones.
    native_view = world.physics_sim_view.create_rigid_body_view(
        [str(p.GetPath()) for p in native]
    )
    initial_native = np.asarray(native_view.get_transforms()).copy()
    if probe:
        probe.initialize(world)
    from isaacsim.core.utils.viewports import set_camera_view

    bounds = (
        UsdGeom.BBoxCache(0, ["default", "render"])
        .ComputeWorldBound(stage.GetPrimAtPath("/World/Stem"))
        .ComputeAlignedRange()
    )
    lo, hi = np.array(bounds.GetMin()), np.array(bounds.GetMax())
    target = (lo + hi) / 2
    span = float(max(hi - lo))
    set_camera_view(eye=target + np.array([1.1, -1.5, 0.45]) * span, target=target)
    if a.camera == "leaf" and (probe or a.leaf_profile == "soft"):
        r = rows[0]
        target = r["initial_origin"] + r["initial_frame"] @ np.array([0.045, 0, 0])
        eye = target + r["initial_frame"] @ np.array([0.08, -0.16, 0.13])
        set_camera_view(eye=eye, target=target)
    state = {"quit": False, "load": False, "selected": 0, "load_target": a.load_target}

    def focus_selected():
        r = rows[state["selected"]]
        current = np.asarray(view.get_transforms())
        frame = (
            rotation(current[r["host_id"] : r["host_id"] + 1, [6, 3, 4, 5]])[0]
            @ r["host_frame"]
        )
        target = current[r["ids"], :3].mean(axis=0)
        set_camera_view(
            eye=target + frame @ np.array([0.08, -0.16, 0.13]), target=target
        )

    window = None
    status_label = None
    if a.gui:
        from omni import ui

        window = ui.Window("Full plant | flexible leaves", width=420, height=260)
        with window.frame, ui.VStack():
            ui.Label(
                f"{len(rows)} flexible blades | physics {a.physics_hz} Hz | display 30 Hz"
            )
            ui.Label(f"{a.leaf_profile} | {rows[0]['segments']} segments per blade")
            ui.Label("Shift + left-drag: pull branches or leaves")
            ui.Label(f"Leaf contacts: {a.leaf_contacts}")
            if a.exclude_initial_from:
                ui.Label(
                    f"Stress demo | {len(contact_policy.get('excluded_initial_overlaps', []))} initial pairs excluded"
                )
            status_label = ui.Label("Starting...")
            selection = ui.IntSlider(min=0, max=len(rows) - 1)
            selection.model.add_value_changed_fn(
                lambda m: state.update(selected=m.as_int)
            )
            ui.Button("Focus selected leaf", clicked_fn=focus_selected)
            ui.Button(
                "View whole plant",
                clicked_fn=lambda: set_camera_view(
                    eye=(lo + hi) / 2 + np.array([1.1, -1.5, 0.45]) * span,
                    target=(lo + hi) / 2,
                ),
            )
            if probe:
                ui.Button(
                    "Repeat branch contact",
                    clicked_fn=lambda: state.update(repeat_probe=True),
                )
            else:
                ui.Button(
                    "Apply 3 mN tip load",
                    clicked_fn=lambda: state.update(load=True, load_target="leaf"),
                )
                ui.Button(
                    "Apply 30 mN branch load",
                    clicked_fn=lambda: state.update(load=True, load_target="branch"),
                )
                ui.Button("Release", clicked_fn=lambda: state.update(load=False))
            ui.Button("Finish and save", clicked_fn=lambda: state.update(quit=True))
    np.savez_compressed(
        a.run_dir / "meshes.npz",
        **{
            f"{key}_{i}": r[key]
            for i, r in enumerate(rows)
            for key in ("points", "faces", "centers", "indices", "weights")
        },
    )
    save(
        a.run_dir / "layout.json",
        [
            {
                k: (v.tolist() if isinstance(v, np.ndarray) else v)
                for k, v in r.items()
                if k
                not in ("channels", "points", "faces", "centers", "indices", "weights")
            }
            for r in rows
        ],
    )
    gpu = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
        capture_output=True,
        text=True,
        check=False,
    )
    save(
        a.run_dir / "runtime.json",
        {
            "isaac": (Path.home() / "isaacsim-6.1/VERSION").read_text().strip(),
            "gpu": gpu.stdout.strip(),
            "physics_hz": a.physics_hz,
            "optimized": a.optimized,
            "adaptive_render": a.adaptive_render,
            "camera": a.camera,
            "load_target": a.load_target,
            "skin_batch": a.skin_batch,
            "fabric": a.fabric,
            "gpu_physics": a.gpu_physics,
            "partition_trunk": a.partition_trunk,
            "partition_branches": a.partition_branches,
            "diagnostics": a.diagnostics,
            "solver": a.solver,
            "iterations": a.iterations,
            "leaf_armature_kg_m2": a.leaf_armature,
            "physics_threads": a.physics_threads,
            "sleep_threshold": a.sleep_threshold,
            "render_hz": 30,
            "leaves": len(rows),
            "native_bodies": len(native),
            "added_bodies": sum(r["segments"] for r in rows),
            "leaf_profile": a.leaf_profile,
            "leaf_profile_revision": 3 if a.leaf_profile == "soft" else 1,
            "mass_before_kg": mass_before,
            "mass_after_kg": mass_after,
            "initialization_seconds": time.perf_counter() - started,
            "collision_policy": contact_policy["policy"],
            "topology": f"{rows[0]['segments']} links per blade in native articulation, root spring to native host",
        },
    )
    times, traces, timings = [], [], []
    native_traces = []
    finite, attachment_max, stretch_max, native_max = True, 0.0, 0.0, 0.0
    edges = [
        np.unique(
            np.sort(
                np.concatenate(
                    [
                        r["faces"][:, [0, 1]],
                        r["faces"][:, [1, 2]],
                        r["faces"][:, [2, 0]],
                    ]
                ),
                axis=1,
            ),
            axis=0,
        )
        for r in rows
    ]
    rest_lengths = [
        np.linalg.norm(r["points"][e[:, 0]] - r["points"][e[:, 1]], axis=1)
        for r, e in zip(rows, edges)
    ]
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools, Sdf

    sleep_paths = (
        [r["body"] for r in partition["independent_roots"]]
        if partition
        else [str(native[0].GetPath())]
    )
    sleep_ids = [PhysicsSchemaTools.sdfPathToInt(Sdf.Path(p)) for p in sleep_paths]
    sleep_samples = []
    sim_interface = get_physx_simulation_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    origin_step = world.current_time_step_index
    start = time.perf_counter()
    frame_start = start
    block = np.zeros(5)
    n = 0
    frame_steps = 0
    render_overhead = 0.012
    physics_step_cost = 0.005
    frame_step_counts, frame_wall_times = [], []
    last_status_time = start
    exported = False
    render_every = a.physics_hz // 30
    diag_every = a.physics_hz // 10
    force = np.zeros((len(order), 3), dtype=np.float32)
    while app.is_running() and not state["quit"]:
        t = n / a.physics_hz
        if not a.gui and t >= a.seconds:
            break
        if not world.is_playing():
            raise RuntimeError("Timeline stopped during test")
        # Reproducible mild load after settling, headless only; GUI is manual.
        loaded = (state["load"] if a.gui else 8 <= t < 10) and not probe
        if probe:
            if state.pop("repeat_probe", False):
                probe.repeat(t)
            probe.update(t, lambda: np.asarray(view.get_transforms()), rows[0])
        if loaded:
            force.fill(0)
            row = rows[state["selected"]]
            branch_load = state["load_target"] == "branch"
            force[row["host_id"] if branch_load else row["ids"][-1], 2] = (
                -0.03 if branch_load else -0.003
            )
            view.apply_forces_and_torques_at_position(
                force, None, None, np.arange(len(order), dtype=np.int32), True
            )
        tick = time.perf_counter()
        contact_before = contacts.callback_seconds if contacts else 0.0
        if contacts:
            contacts.time = t
        world.step(render=False)
        contact_cost = contacts.callback_seconds - contact_before if contacts else 0.0
        block[0] += time.perf_counter() - tick - contact_cost
        block[1] += contact_cost
        physics_step_cost = 0.9 * physics_step_cost + 0.1 * (time.perf_counter() - tick)
        n += 1
        frame_steps += 1
        if a.physics_hz < 480:
            check = np.asarray(view.get_transforms())
            if not np.isfinite(check).all() or np.abs(check[:, :3]).max() > 10:
                raise RuntimeError(
                    f"Unstable physical state at step {n}; stopped before next step"
                )
        if world.current_time_step_index != origin_step + n:
            raise RuntimeError("More than one physics step per iteration")
        frame_due = n % render_every == 0
        if a.adaptive_render:
            frame_due = (
                frame_steps >= render_every
                or time.perf_counter() - frame_start + physics_step_cost
                >= max(0.001, 1 / 30 - render_overhead)
            )
        if n % a.physics_hz == 0:
            sleep_samples.append(
                {
                    "time": n / a.physics_hz,
                    "sleeping": [
                        bool(sim_interface.is_sleeping(stage_id, p)) for p in sleep_ids
                    ],
                }
            )
            print(
                f"PROGRESS sim={n / a.physics_hz:.1f}s wall={time.perf_counter() - start:.1f}s",
                flush=True,
            )
        if not frame_due:
            if n % diag_every == 0 and n / a.physics_hz <= a.seconds:
                tick = time.perf_counter()
                sample = np.asarray(view.get_transforms()).copy()
                native_sample = np.asarray(native_view.get_transforms()).copy()
                if (
                    not np.isfinite(sample).all()
                    or not np.isfinite(native_sample).all()
                ):
                    raise RuntimeError("Nonfinite physical state")
                times.append(n / a.physics_hz)
                traces.append(sample)
                native_traces.append(native_sample)
                block[1] += time.perf_counter() - tick
            continue
        tick = time.perf_counter()
        poses = np.asarray(view.get_transforms()).copy()
        np_native = np.asarray(native_view.get_transforms()).copy()
        block[1] += time.perf_counter() - tick
        finite &= bool(np.isfinite(poses).all() and np.isfinite(np_native).all())
        if not finite:
            raise RuntimeError("Nonfinite physical state")
        native_max = max(
            native_max,
            float(
                np.linalg.norm(np_native[:, :3] - initial_native[:, :3], axis=1).max()
            ),
        )
        tick = time.perf_counter()
        all_rot = rotation(poses[:, [6, 3, 4, 5]])
        diag = n % diag_every == 0
        if skin_batch:
            skin_batch.update(poses, all_rot)
        for i, r in enumerate(rows):
            if skin_batch and a.optimized:
                continue
            if a.optimized:
                # USD quaternion arrays use xyzw; PhysX already provides xyzw.
                current = poses[r["ids"]]
                host_pose = poses[r["host_id"]]
                key = np.concatenate((current.ravel(), host_pose))
                if a.gui or a.render:
                    previous = r.get("displayed_pose")
                    if previous is None or not np.array_equal(previous, key):
                        h = r["host_id"]
                        hr = all_rot[h]
                        hp = poses[h, :3]
                        host_q = quat(hr @ r["host_frame"])
                        bp = np.vstack([hp + hr @ r["host_offset"], current[:, :3]])
                        bq = np.vstack(
                            [[*host_q.GetImaginary(), host_q.GetReal()], current[:, 3:]]
                        )
                        r["channels"][0].Set(
                            Vt.Vec3fArray.FromNumpy(
                                np.ascontiguousarray(bp, dtype=np.float32)
                            )
                        )
                        r["channels"][1].Set(
                            Vt.QuatfArray.FromNumpy(
                                np.ascontiguousarray(bq, dtype=np.float32)
                            )
                        )
                        r["displayed_pose"] = key.copy()
                continue
            h = r["host_id"]
            hp, hr = poses[h, :3], all_rot[h]
            bp = np.vstack([hp + hr @ r["host_offset"], poses[r["ids"], :3]])
            bq = np.vstack(
                [
                    np.array(
                        [
                            float(quat(hr @ r["host_frame"]).GetReal()),
                            *quat(hr @ r["host_frame"]).GetImaginary(),
                        ]
                    ),
                    poses[r["ids"]][:, [6, 3, 4, 5]],
                ]
            )
            r["channels"][0].Set(Vt.Vec3fArray.FromNumpy(bp.astype(np.float32)))
            r["channels"][1].Set(
                Vt.QuatfArray(
                    [Gf.Quatf(float(q[0]), Gf.Vec3f(*map(float, q[1:]))) for q in bq]
                )
            )
            if diag:
                c = r["config"]
                anchor1 = poses[r["ids"][0], :3] + all_rot[r["ids"][0]] @ (
                    np.array([c["fixed_length"], 0, 0]) - r["centers"][1]
                )
                attachment_max = max(
                    attachment_max,
                    float(np.linalg.norm(anchor1 - (hp + hr @ r["anchor"]))),
                )
                deformed = skin(
                    r["points"], r["centers"], r["indices"], r["weights"], bp, bq
                )
                e, lengths = edges[i], rest_lengths[i]
                valid = lengths > 1e-8
                stretch_max = max(
                    stretch_max,
                    float(
                        np.max(
                            np.linalg.norm(
                                deformed[e[:, 0]] - deformed[e[:, 1]], axis=1
                            )[valid]
                            / lengths[valid]
                            - 1
                        )
                    ),
                )
        block[2] += time.perf_counter() - tick
        if diag and t < a.seconds:
            times.append(n / a.physics_hz)
            traces.append(poses)
            native_traces.append(np_native)
        tick = time.perf_counter()
        if a.gui or a.render:
            if fabric:
                fabric.update(n / a.physics_hz, 1 / a.physics_hz)
            else:
                get_physx_interface().update_transformations(False, True, False)
            world.render()
        block[3] += time.perf_counter() - tick
        if not exported:
            exported = True
            if fabric:
                get_physx_interface().update_transformations(False, True, False)
            stage.Export(str(a.run_dir / "scene.usda"))
        block[4] = time.perf_counter() - frame_start
        render_overhead = 0.8 * render_overhead + 0.2 * (block[1] + block[2] + block[3])
        if a.gui:
            time.sleep(max(0, 1 / 30 - (time.perf_counter() - frame_start)))
        frame_wall = time.perf_counter() - frame_start
        if t >= 8:
            timings.append(block.copy())
            frame_step_counts.append(frame_steps)
            frame_wall_times.append(frame_wall)
        if a.gui and time.perf_counter() - last_status_time >= 0.5:
            status_label.text = f"Leaf {state['selected']} | {1 / max(frame_wall, 1e-9):.1f} FPS | simulation {frame_steps / a.physics_hz / frame_wall:.2f}x | t={n / a.physics_hz:.1f}s"
            last_status_time = time.perf_counter()
        block.fill(0)
        frame_steps = 0
        frame_start = time.perf_counter()
    elapsed = time.perf_counter() - start
    np.savez_compressed(a.run_dir / "poses.npz", times=times, poses=traces, paths=order)
    np.savez_compressed(
        a.run_dir / "native_poses.npz",
        times=times,
        poses=native_traces,
        paths=native_view.prim_paths,
    )
    np.save(a.run_dir / "frame_timings.npy", np.asarray(timings))
    np.savez_compressed(
        a.run_dir / "frame_schedule.npz",
        steps=frame_step_counts,
        wall_seconds=frame_wall_times,
    )
    save(a.run_dir / "sleep.json", {"paths": sleep_paths, "samples": sleep_samples})
    if fabric and (a.gui or a.render):
        from fabric_validation import validate_native_fabric

        fabric.update(n / a.physics_hz, 1 / a.physics_hz)
        np_native = np.asarray(native_view.get_transforms()).copy()

        save(
            a.run_dir / "fabric_validation.json",
            validate_native_fabric(native_view.prim_paths, np_native),
        )
    from full_plant_analysis import analyze

    per_leaf = analyze(a.run_dir)
    if skin_batch:
        skin_batch.validate(poses, all_rot)
        save(
            a.run_dir / "skin_validation.json",
            {
                "maximum_surface_error_m": skin_batch.maximum_visual_error,
                "cluster_size": a.skin_batch,
            },
        )
    if a.optimized and per_leaf:
        attachment_max = max(r["attachment_max_m"] for r in per_leaf)
        stretch_max = max(r["stretch_max_fraction"] for r in per_leaf)
    samples = np.asarray(timings)
    report = {
        "status": "diagnostic",
        "finite": finite,
        "root_attachment_max_m": attachment_max,
        "stretch_max_fraction": stretch_max,
        "native_max_displacement_m": native_max,
        "seconds": n / a.physics_hz,
        "loop_seconds": elapsed,
        "visual_acceptance": "pending",
        "gui_manual": a.gui,
        "recorded_seconds": min(n / a.physics_hz, a.seconds),
        "checks": {
            "finite": finite,
            "attachment_below_0_2mm": attachment_max < 0.0002,
            "stretch_below_5percent": stretch_max < 0.05,
        },
        "performance_window": "after 8 simulated seconds",
        "timing_columns": [
            "physics",
            "reads_and_contact_reports",
            "skinning_and_diagnostics",
            "render_and_native_sync",
            "frame_work",
        ],
    }
    if len(samples):
        report.update(
            median_seconds=np.median(samples, axis=0).tolist(),
            p95_seconds=np.percentile(samples, 95, axis=0).tolist(),
            uncapped_work_fps=float(1 / samples[:, 4].mean()),
            actual_display_fps=float(len(frame_wall_times) / sum(frame_wall_times)),
            performance_realtime_factor=float(
                sum(frame_step_counts) / a.physics_hz / sum(frame_wall_times)
            ),
            median_physics_steps_per_frame=float(np.median(frame_step_counts)),
        )
    if per_leaf and not a.gui and n / a.physics_hz >= 18:
        report["recovery_world_max_m"] = max(
            r["world_recovery_error_m"] for r in per_leaf
        )
        report["recovery_petiole_max_m"] = max(
            r["petiole_recovery_error_m"] for r in per_leaf
        )
        report["residual_world_max_m"] = max(
            r["world_residual_motion_m"] for r in per_leaf
        )
        report["checks"]["recovery_below_2mm"] = report["recovery_world_max_m"] < 0.002
        report["checks"]["residual_below_0_5mm"] = (
            report["residual_world_max_m"] < 0.0005
        )
    if not all(report["checks"].values()):
        report["status"] = "failed"
    report["leaf_profile"] = a.leaf_profile
    report["contact_probe"] = bool(probe)
    report["leaf_contacts"] = a.leaf_contacts
    report["excluded_initial_overlap_pairs"] = len(
        contact_policy.get("excluded_initial_overlaps", [])
    )
    if contacts:
        evidence = contacts.result()
        save(a.run_dir / "contacts.json", evidence)
        report["contact_evidence"] = {k: v for k, v in evidence.items() if k != "pairs"}
        # Initial intersections are reported separately, not passed off as impacts.
        report["initial_contact_geometry_valid"] = (
            evidence["first_step_penetration_max_m"] < 0.0001
        )
        if n / a.physics_hz > 8:
            report["checks"]["settled_collider_penetration_below_1mm"] = (
                evidence["penetration_after_8s_max_m"] < 0.001
            )
        if not all(report["checks"].values()):
            report["status"] = "failed"
    save(a.run_dir / "report.json", report)
    print("FULL_PLANT_REPORT " + json.dumps(report), flush=True)
    # Keep the UI object alive through the simulation loop.
    del window


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--run-dir", type=Path, required=True)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--render", action="store_true")
    p.add_argument("--seconds", type=float, default=20)
    p.add_argument("--leaves", type=int, default=131)
    p.add_argument("--leaf-profile", choices=["original", "soft"], default="original")
    p.add_argument("--optimized", action="store_true")
    p.add_argument("--adaptive-render", action="store_true")
    p.add_argument("--camera", choices=["leaf", "plant"], default="leaf")
    p.add_argument("--skin-batch", type=int, default=0)
    p.add_argument("--fabric", action="store_true")
    p.add_argument("--gpu-physics", action="store_true")
    p.add_argument("--partition-trunk", action="store_true")
    p.add_argument("--partition-branches", action="store_true")
    p.add_argument("--diagnostics", choices=["full", "light", "live"], default="full")
    p.add_argument("--physics-hz", type=int, choices=[120, 240, 480], default=480)
    p.add_argument("--solver", choices=["PGS", "TGS"], default="PGS")
    p.add_argument("--physics-threads", type=int, default=8)
    p.add_argument("--iterations", type=int, default=32)
    p.add_argument("--leaf-armature", type=float, default=0)
    p.add_argument("--sleep-threshold", type=float, default=0)
    p.add_argument("--contact-probe", action="store_true")
    p.add_argument("--load-target", choices=["leaf", "branch"], default="leaf")
    p.add_argument("--exclude-initial-from", type=Path)
    p.add_argument("--leaf-contacts", choices=["off", "plant", "all"], default="off")
    a = p.parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": not a.gui,
            "width": 1280,
            "height": 720,
            "renderer": "RaytracedLighting",
            "disable_viewport_updates": not (a.gui or a.render),
            "fast_shutdown": True,
        }
    )
    code = 0
    try:
        run(app, a)
    except Exception:  # noqa: BLE001 -- preserve worker failure evidence
        traceback.print_exc()
        code = 1
    finally:
        app.close(wait_for_replicator=False, exit_code=code)
    os._exit(code)


if __name__ == "__main__":
    main()
