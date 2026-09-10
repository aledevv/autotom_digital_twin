"""Isaac-only calibration of native mouse force on an isolated free fruit."""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-case", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fruit", default="/World/TerminalBodies/Truss_r5_o0_g421786_tomato_08")
    args = parser.parse_args()
    if (args.output / "report.json").exists():
        parser.error("choose a fresh probe output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    sys.argv = [sys.argv[0], "--/plugins/carb.tasking.plugin/threadCount=4"]
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "fast_shutdown": False})
    def probe():
        import carb.settings
        import numpy as np
        from pxr import Gf, Usd, UsdGeom, UsdPhysics, PhysxSchema
        from isaacsim.core.api import World
        from isaacsim.core.prims import RigidPrim
        from omni.physx import get_physx_interface, get_physx_scene_query_interface
        from omni.physx.bindings._physx import PhysicsInteractionEvent as Event, SETTING_NUM_THREADS
        sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
        from exporterV2.fruit_experiments import fruit_geometry
        from exporterV2.fruit_interaction import pick_ray, unit
        source = args.source_case / "scene.usda"
        geometry = next(r for r in fruit_geometry(Usd.Stage.Open(str(source))) if r["fruit"] == args.fruit)
        radii = geometry["shapes"][0]["world_radii_m"]
        if not np.allclose(radii, radii[0], atol=1e-9, rtol=0):
            raise ValueError("free probe currently requires the source's spherical fruit")
        effective = json.loads((args.source_case / "headless-effective.json").read_text())
        body = next(b for b in effective["bodies"] if b["path"] == args.fruit)
        settings = carb.settings.get_settings()
        settings.set(SETTING_NUM_THREADS, 4)
        world = World(stage_units_in_meters=1., physics_dt=1/60, rendering_dt=1/60)
        world.get_physics_context().set_solver_type("PGS")
        stage = world.stage
        scene_prim = world.get_physics_context().get_current_physics_scene_prim()
        scene = UsdPhysics.Scene(scene_prim)
        scene.CreateGravityMagnitudeAttr(0.)
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        physx_scene.CreateEnableGPUDynamicsAttr(False)
        sphere = UsdGeom.Sphere.Define(stage, "/World/Probe")
        sphere.CreateRadiusAttr(radii[0])
        prim = sphere.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(body["mass_kg"])
        rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rigid.CreateLinearDampingAttr(0.)
        rigid.CreateAngularDampingAttr(0.)
        rigid.CreateSolverPositionIterationCountAttr(255)
        rigid.CreateSolverVelocityIterationCountAttr(0)
        stage.GetRootLayer().Export(str(args.output / "fixture.usda"))
        world.reset()
        world.set_simulation_dt(physics_dt=1/60, rendering_dt=1/60)
        view = RigidPrim(["/World/Probe"], reset_xform_properties=False)
        view.initialize()
        native = get_physx_interface()
        query = get_physx_scene_query_interface().raycast_closest
        for name, value in (("mouseInteractionEnabled", True), ("mouseGrab", True),
                            ("mouseGrabIgnoreInvisible", False), ("forceGrab", True)):
            settings.set("/physics/" + name, value)
        mass = float(np.asarray(view.get_masses()).reshape(-1)[0])
        results, arrays = [], {}
        for coefficient in (None, 1., 3., 10.):
            label = "known_force" if coefficient is None else f"native_{coefficient:g}"
            view.set_world_poses(positions=np.zeros((1, 3)), orientations=np.array([[1., 0., 0., 0.]]))
            view.set_velocities(np.zeros((1, 6)))
            world.step(render=False)
            eye, direction, hit, attempts = pick_ray([0., 0., 0.], "/World/Probe", query)
            if coefficient is not None:
                settings.set("/physics/pickingForce", coefficient)
                native.update_interaction(tuple(eye), tuple(direction), Event.MOUSE_DRAG_BEGAN)
            rows = []
            previous = np.asarray(view.get_velocities())[0, :3].copy()
            for step in range(300 if coefficient is not None else 30):
                if coefficient is None:
                    view.apply_forces_and_torques_at_pos(forces=np.array([[0., 0., -mass]], dtype=np.float32), is_global=True)
                else:
                    direction = unit(hit + np.array([0., 0., -.2 * (step + 1) / 300]) - eye)
                    native.update_interaction(tuple(eye), tuple(direction), Event.MOUSE_DRAG_CHANGED)
                world.step(render=False)
                pos = np.asarray(view.get_world_poses()[0])[0]
                velocity = np.asarray(view.get_velocities())[0, :3]
                estimate = mass * (velocity - previous) * 60
                rows.append(np.concatenate(([(step + 1)/60], pos, velocity, estimate)))
                previous = velocity.copy()
                if not np.isfinite(rows[-1]).all():
                    raise RuntimeError(f"nonfinite free-fruit probe at {label}/{step}")
            if coefficient is not None:
                native.update_interaction(tuple(eye), tuple(direction), Event.MOUSE_DRAG_ENDED)
            values = np.asarray(rows)
            arrays[label] = values
            results.append({"case": label, "mass_kg": mass, "radius_m": radii[0],
                            "peak_resultant_force_n": float(np.linalg.norm(values[:, 7:10], axis=1).max()),
                            "final_position_m": values[-1, 1:4].tolist(), "ray_attempts": attempts})
            print(json.dumps(results[-1]), flush=True)
            if coefficient is None and not np.allclose(values[:, 7:10], [0, 0, -mass], rtol=1e-4, atol=1e-7):
                raise RuntimeError("known-force calibration failed")
        np.savez_compressed(args.output / "probe.npz", **arrays,
                            columns=np.array(["time_s", "x", "y", "z", "vx", "vy", "vz", "fx_est_n", "fy_est_n", "fz_est_n"]))
        report = {"status": "passed", "source_fruit": args.fruit, "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
                  "hz": 60, "solver": "PGS", "gpu_dynamics": False, "gravity": 0,
                  "linear_damping": 0, "angular_damping": 0, "contacts": "one isolated collider, no other bodies",
                  "interpretation": "free-fruit resultant estimated from COM velocity; not the constrained plant mouse force",
                  "results": results}
        (args.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    try:
        probe()
    finally:
        # Release tensor views and world ownership before unloading PhysX.
        from isaacsim.core.api import World
        import gc
        World.clear_instance()
        gc.collect()
        app.close()


if __name__ == "__main__":
    import os
    import traceback
    exit_code = 0
    try:
        main()
    except Exception:
        traceback.print_exc()
        exit_code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    # Match isaac_app.py: avoid interpreter GC of stale Kit bindings after
    # the simulator has already unloaded its native plugins.
    os._exit(exit_code)
