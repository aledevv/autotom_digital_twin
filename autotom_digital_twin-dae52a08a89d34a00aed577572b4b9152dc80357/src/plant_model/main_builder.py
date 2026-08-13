"""
main_builder.py

Entry-point for the PlantBuilder pipeline:
  1. Bootstrap SimulationApp (must precede any pxr/omni imports)
  2. Load the plant snapshot for a given day (loader.py)
  3. Build the USD stage with the articulated stem (usd_exporter_builder.py)
  4. Inject PhysicsScene + PhysxArticulationAPI
  5. Save, open in Isaac Sim, run the simulation loop
"""

import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SRC_DIR      = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# Bootstrap Isaac Sim runtime — must come before any pxr/omni imports
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdPhysics, PhysxSchema, Gf
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporter_builder import build_plant_stage

def setup_physx(stage, stem_path):
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableStabilizationAttr().Set(True)

    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(stem_path))
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)

def parse_args():
    parser = argparse.ArgumentParser(description="Build and simulate the articulated plant using PlantBuilder in Isaac Sim.")
    parser.add_argument("--day",   type=int, default=1, help="Simulation day.")
    parser.add_argument("--plant", type=int, default=1,   help="Plant ID.")
    parser.add_argument("--csv",   default=None,          help="CSV path (default: resolved from day).")
    parser.add_argument("--out",   default=None,          help="Output .usda path (default: resolved from day).")
    return parser.parse_args()


def main():
    args = parse_args()

    csv_path = args.csv or os.path.join(
        PROJECT_ROOT, "data/simulation_output/dynamic_output/graphs", f"graph_day_{args.day}.csv"
    )
    out_path = args.out or os.path.join(
        PROJECT_ROOT, "output", f"day_{args.day}", f"plant_day{args.day}_builder.usda"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"[INFO] Loading snapshot day={args.day} plant={args.plant} from {csv_path}")
    snapshot = load_snapshot(csv_path, day=args.day, plant_id=args.plant)

    print("[INFO] Building plant stage with PlantBuilder...")
    stage, stem_path = build_plant_stage(snapshot, out_path)
    
    from plant_model.usd_exporter_builder import validate_usd_dimensions
    validate_usd_dimensions(stage, snapshot, stem_path)

    print("[INFO] Injecting PhysX config...")
    setup_physx(stage, stem_path)

    stage.GetRootLayer().Save()
    print(f"[OK] Stage saved with PhysX config: {out_path}")

    omni.usd.get_context().open_stage(out_path)
    print(f"[OK] Stage opened in Isaac Sim: {out_path}")

    try:
        action_registry = omni.kit.actions.core.get_action_registry()
        action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
    except Exception as e:
        print(f"[WARN] Could not set lighting mode: {e}")

    world = World(stage_units_in_meters=1.0)
    world.reset()
    print("[OK] Simulation running — close the window to exit.")

    # Only run for a short time to verify it's working in the test, but keep it running for visualization
    # We will close the window to end it
    while simulation_app.is_running():
        world.step(render=True)

    print("Simulation ended.")
    simulation_app.close()


if __name__ == "__main__":
    main()
