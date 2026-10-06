"""Isaac Sim 4.5 worker for D0. Invoke through run.py to retain logs/timeouts."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback

import numpy as np

from model import Config, make_mesh, signed_volumes, summarize


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")


def build(world, c):
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics
    from omni.physx.scripts import deformableUtils, physicsUtils
    from isaacsim.core.prims import DeformablePrim

    stage = world.stage
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    physics = world.get_physics_context()
    physics.enable_gpu_dynamics(True)
    physics.set_broadphase_type("GPU")
    physics.set_solver_type("TGS")
    physics.set_gravity(0.0)  # reset must not consume part of the loading phase
    points, tets, faces = make_mesh(c)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Strip")
    physicsUtils.setup_transform_as_scale_orient_translate(mesh)
    mesh.CreatePointsAttr().Set([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr().Set([3]*len(faces))
    mesh.CreateFaceVertexIndicesAttr().Set(faces.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr().Set("none")
    mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.12, 0.5, 0.08)])
    mesh.CreateDoubleSidedAttr().Set(True)
    vec = [Gf.Vec3f(*p) for p in points]
    if not deformableUtils.add_physx_deformable_body(stage, mesh.GetPath(),
        collision_rest_points=vec, collision_indices=tets.ravel().tolist(),
        simulation_rest_points=vec, simulation_indices=tets.ravel().tolist(),
        solver_position_iteration_count=c.iterations,
        vertex_velocity_damping=c.velocity_damping, self_collision=False,
        sleep_threshold=0.0, settling_threshold=0.0, sleep_damping=0.0):
        raise RuntimeError("Could not author deformable body")
    mat = Sdf.Path("/World/LeafMaterial")
    deformableUtils.add_deformable_body_material(stage, mat, density=c.density,
        youngs_modulus=c.young, poissons_ratio=c.poisson,
        elasticity_damping=c.elasticity_damping, damping_scale=0.0,
        dynamic_friction=0.3)
    physicsUtils.add_physics_material_to_prim(stage, mesh.GetPrim(), mat)
    # Static collar encloses the first 6 mm, selecting a region, not a single point.
    origin = np.array([c.clamp_length/2, 0, c.height])
    collar = physicsUtils.add_box(stage, "/World/Clamp",
        size=Gf.Vec3f(c.clamp_length+2e-5, c.width+0.002, c.thickness+0.004),
        position=Gf.Vec3f(*origin), color=Gf.Vec3f(0.35, 0.4, 0.5))
    UsdPhysics.CollisionAPI.Apply(collar)
    attachment = PhysxSchema.PhysxPhysicsAttachment.Define(stage, "/World/Strip/Attachment")
    attachment.CreateActor0Rel().SetTargets([mesh.GetPath()])
    attachment.CreateActor1Rel().SetTargets([collar.GetPath()])
    auto = PhysxSchema.PhysxAutoAttachmentAPI.Apply(attachment.GetPrim())
    auto.CreateDeformableVertexOverlapOffsetAttr().Set(0.0)
    auto.CreateEnableRigidSurfaceAttachmentsAttr().Set(False)
    # Explicitly suppress collar contact so it cannot fight the attachment.
    UsdPhysics.FilteredPairsAPI.Apply(mesh.GetPrim()).CreateFilteredPairsRel().AddTarget(collar.GetPath())
    lamp = UsdLux.DomeLight.Define(stage, "/World/Light")
    lamp.CreateIntensityAttr().Set(700)
    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.001, 10))
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([0.11, -0.12, 0.11]),
                    target=np.array([0.028, 0, 0.045]), camera_prim_path="/World/Camera")
    view = DeformablePrim("/World/Strip", name="strip", reset_xform_properties=False)
    world.scene.add(view)
    return view, points, tets, origin


def run(app, args, c):
    from isaacsim.core.api import World
    from pxr import PhysxSchema
    out = args.run_dir
    world = World(stage_units_in_meters=1.0, physics_dt=1/c.hz,
                  rendering_dt=1/60, backend="torch", device="cuda:0")
    view, authored, authored_tets, origin = build(world, c)
    if args.gui:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = "/World/Camera"
    world.reset()
    if not view.is_physics_handle_valid():
        raise RuntimeError("Deformable tensor handle invalid after reset")
    def read_nodes():
        return view.get_simulation_mesh_nodal_positions()[0].detach().cpu().numpy().copy()
    rest = read_nodes()
    tets = view.get_simulation_mesh_indices()[0].detach().cpu().numpy().astype(np.int32)
    # Refuse unexpected cooking changes; diagnostics must not use guessed topology.
    if rest.shape != authored.shape or not np.allclose(rest, authored, atol=1e-7, rtol=0):
        raise RuntimeError("Loaded nodal order/rest geometry differs from authored mesh")
    if tets.shape != authored_tets.shape or not np.all(signed_volumes(rest, tets) > 0):
        raise RuntimeError("Loaded tetrahedral topology is invalid or changed")
    physics = world.get_physics_context()
    from isaacsim.core.api.materials.deformable_material_view import DeformableMaterialView
    material_view = DeformableMaterialView("/World/LeafMaterial")
    material_view.initialize()
    loaded_material = dict(young=float(material_view.get_youngs_moduli().item()),
                           poisson=float(material_view.get_poissons_ratios().item()))
    if not np.isclose(loaded_material["young"], c.young) or not np.isclose(loaded_material["poisson"], c.poisson):
        raise RuntimeError(f"Material mismatch: {loaded_material}")
    physics.set_gravity(-9.81)
    scene_prim = world.stage.GetPrimAtPath("/physicsScene")
    if not scene_prim:
        scene_prim = next(p for p in world.stage.Traverse() if p.GetTypeName() == "PhysicsScene")
    scene_api = PhysxSchema.PhysxSceneAPI(scene_prim)
    effective = dict(gpu=scene_api.GetEnableGPUDynamicsAttr().Get(),
                     solver=scene_api.GetSolverTypeAttr().Get(),
                     hz=scene_api.GetTimeStepsPerSecondAttr().Get())
    if effective != dict(gpu=True, solver="TGS", hz=c.hz):
        raise RuntimeError(f"Physics configuration mismatch: {effective}")
    world.stage.GetRootLayer().Export(str(out/"scene.usda"))
    np.savez_compressed(out/"rest_mesh.npz", points=rest, tetrahedra=tets)
    version_path = Path(__import__("os").environ["ISAAC_PATH"])/"VERSION"
    write_json(out/"runtime.json", dict(python=sys.executable, cwd=str(Path.cwd()),
        isaac_version=version_path.read_text().strip(), effective=effective, loaded_material=loaded_material,
        vertices=len(rest), tetrahedra=len(tets), thickness_cells=c.nz,
        volume_m3=float(signed_volumes(rest, tets).sum()),
        expected_mass_kg=float(signed_volumes(rest, tets).sum()*c.density),
        mass_note="Integrated rest volume times authored material density; not an independent solver mass measurement.",
        scene_sha256=hashlib.sha256((out/"scene.usda").read_bytes()).hexdigest()))
    times, nodes, steps, samples, render_times = [0.0], [rest.copy()], [], [], []
    physics_start = float(world.current_time)
    physics_step_start = int(world.current_time_step_index)
    render_enabled = args.gui or args.offscreen_render
    total = round((c.gravity_seconds+c.recovery_seconds)*c.hz)
    switch = round(c.gravity_seconds*c.hz)
    started = time.perf_counter()
    rendered = 0
    print("D0: gravity ON; then gravity OFF for elastic recovery", flush=True)
    interrupted = False
    for index in range(total):
        if not app.is_running():
            interrupted = True
            break
        if not world.is_playing():
            # Stop/reset changes the experimental initial state: rerun from a fresh directory.
            interrupted = True
            break
        if index == switch:
            physics.set_gravity(0.0)
            print("D0: gravity OFF, recovery phase", flush=True)
        render = render_enabled and index % (c.hz//60) == 0
        a = time.perf_counter()
        # step(render=True) may advance MULTIPLE physics ticks at rendering_dt.
        # Always advance exactly one tick; render() explicitly disables physics.
        world.step(render=False)
        b = time.perf_counter()
        pos = read_nodes()
        d = time.perf_counter()
        simulated_time = float(world.current_time)-physics_start
        expected_time = (index+1)/c.hz
        # PhysX accumulates a float32 dt; allow microsecond drift, never a tick.
        if int(world.current_time_step_index)-physics_step_start != index+1 or not np.isclose(
            simulated_time, expected_time, rtol=0, atol=1e-5
        ):
            raise RuntimeError(f"Physics clock mismatch: {simulated_time} != {expected_time}")
        steps.append(b-a)
        samples.append(d-b)
        times.append(simulated_time)
        nodes.append(pos)
        if render:
            world.render()
            if not np.isclose(float(world.current_time)-physics_start, simulated_time, rtol=0, atol=1e-7):
                raise RuntimeError("Rendering changed the physics clock")
            render_times.append(time.perf_counter()-d)
        rendered += int(render)
        if not np.isfinite(pos).all() or np.max(np.abs(pos-rest)) > 2*c.length:
            break
        if index % c.hz == 0:
            write_json(out/"progress.json", dict(simulated_seconds=times[-1], phase="gravity" if index < switch else "recovery"))
        if args.gui:
            time.sleep(max(0.0, started+(index+1)/c.hz-time.perf_counter()))
    elapsed = time.perf_counter()-started
    nodes = np.asarray(nodes)
    tip = np.isclose(rest[:, 0], c.length, atol=1e-7)
    center = np.isclose(rest[:, 0], c.length/2, atol=c.length/c.nx/2+1e-7)
    attached = rest[:, 0] <= c.clamp_length+1e-7
    landmarks = np.stack([nodes[:, mask].mean(axis=1) for mask in (tip, center, attached)], axis=1)
    np.savez_compressed(out/"trace.npz", time_s=times, nodes_world_m=nodes,
        landmarks_world_m=landmarks, landmarks_support_m=landmarks-origin,
        landmark_names=["tip", "center", "attachment_region"],
        support_world_m=np.broadcast_to(origin, (len(times), 3)),
        support_quaternion_wxyz=np.broadcast_to([1., 0., 0., 0.], (len(times), 4)),
        step_seconds=steps, sampling_seconds=samples, render_seconds=render_times)
    report = summarize(c, times, nodes, tets, rest, steps, samples, elapsed)
    report.update(gui=args.gui, offscreen_render=args.offscreen_render,
                  physics_clock_verified=True,
                  mean_render_ms=1000*float(np.mean(render_times)) if render_times else None,
                  rendered_fps=rendered/elapsed if args.gui else None,
                  gui_interrupted=interrupted)
    if interrupted:
        report["status"] = "failed"
    write_json(out/"report.json", report)
    print(json.dumps(report, indent=2), flush=True)
    if args.gui and app.is_running():
        world.pause()
        print("D0 complete: paused at final state. Close window; rerun to repeat.", flush=True)
        while app.is_running():
            app.update()
            time.sleep(0.01)
    # A completed negative experiment is distinct from a runtime/process error.
    return 0 if report.get("checks", {}).get("complete") and not interrupted else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--offscreen-render", action="store_true")
    args = parser.parse_args()
    c = Config(**json.loads((args.run_dir/"config.json").read_text()))
    c.validate()
    write_json(args.run_dir/"config.json", asdict(c))
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": not args.gui, "width": 1280, "height": 720,
                         "active_gpu": 0, "physics_gpu": 0, "fast_shutdown": False})
    code = 1
    try:
        code = run(app, args, c)
    except Exception:
        error = traceback.format_exc()
        print(error, flush=True)
        write_json(args.run_dir/"report.json", dict(status="failed", error=error, manual_review="pending"))
    finally:
        app.close()
    return code


if __name__ == "__main__":
    # Isaac 4.5 can crash during Python GC after unloading its plugins. Finish
    # app.close() first, retain its errors/exit status, then skip interpreter GC.
    import os
    result = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(result)
