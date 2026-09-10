"""Measure the archived main launcher's World/reset defaults without a GUI.

Reads a private flattened copy of the source; never saves over the input USD.
This measures runtime configuration, not desktop FPS or manual acceptance.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output / "report.json"
    if report_path.exists():
        raise ValueError("refusing to overwrite a runtime probe")
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import omni.usd
        from pxr import Usd, UsdPhysics
        from isaacsim.core.api import World
        from exporterV2.core.skinning.runtime import configure_physx_mouse_interaction

        private_path = args.output / "source-copy.usda"
        Usd.Stage.Open(str(args.source.resolve())).Flatten().Export(str(private_path))
        context = omni.usd.get_context()
        context.open_stage(str(private_path.resolve()))
        for _ in range(10):
            app.update()
        stage = context.get_stage()
        report = {"source": str(args.source.resolve()),
                  "source_sha256": hashlib.sha256(args.source.read_bytes()).hexdigest(),
                  "headless": True, "phases": [], "steps": []}

        def snapshot(label, world=None):
            scene = stage.GetPrimAtPath("/World/PhysicsScene")
            data = {"phase": label, "scene": {
                a.GetName(): a.Get() for a in scene.GetAttributes()
                if a.HasAuthoredValueOpinion() and isinstance(a.Get(), (str, bool, int, float))}}
            if world is not None:
                data.update(physics_dt_s=world.get_physics_dt(), render_dt_s=world.get_rendering_dt(),
                            device=world.device, simulation_time_s=world.current_time)
            report["phases"].append(data)
            print("[MAIN_RUNTIME]", json.dumps(data), flush=True)

        snapshot("opened_source")
        configure_physx_mouse_interaction(app)
        world = World(stage_units_in_meters=1.0)
        snapshot("constructed_world", world)
        world.reset()
        snapshot("after_reset", world)
        configure_physx_mouse_interaction(app)
        snapshot("after_mouse_configuration", world)
        for _ in range(3):
            before = world.current_time
            world.step(render=True)
            report["steps"].append({"before_s": before, "after_s": world.current_time,
                                    "advanced_s": world.current_time - before})
        report["fruit_count"] = sum(p.HasAPI(UsdPhysics.RigidBodyAPI) and
                                     "/TerminalBodies/" in str(p.GetPath()) for p in stage.Traverse())
        report["status"] = "completed"
        report_path.write_text(json.dumps(report, indent=2) + "\n")
        print("[MAIN_RUNTIME] report=", report_path, flush=True)
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    code = 1
    try:
        code = main()
    except Exception:
        import traceback
        traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
