import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
VERSION_DIR  = os.path.dirname(SCRIPT_DIR)      
SRC_DIR      = os.path.dirname(VERSION_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# Bootstrap Isaac Sim runtime — must come before any pxr/omni imports
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdPhysics, PhysxSchema, Gf
# pyrefly: ignore [missing-import]
import omni.usd
# pyrefly: ignore [missing-import]
from isaacsim.core.api import World
# pyrefly: ignore [missing-import]
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.v2.plant_builder import PlantBuilder
from plant_model.v2.config import DEFAULT_CONFIG, GLOBAL_SCALE, PLANT_ROOT_PATH
from plant_model.v2.organ_params_loader import build_plant_from_snapshot

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

    print("[INFO] Building plant stage")
    
    usd_context = omni.usd.get_context()
    usd_context.new_stage()
    stage = usd_context.get_stage()

    builder = PlantBuilder(stage, base_path=PLANT_ROOT_PATH, scale=GLOBAL_SCALE)

    print(f"[INFO] Building plant with config: stem={DEFAULT_CONFIG.stem}, "
          f"leaf={DEFAULT_CONFIG.leaf}, fruit={DEFAULT_CONFIG.fruit}")
    build_plant_from_snapshot(snapshot, builder, DEFAULT_CONFIG)

    stage.GetRootLayer().Export(out_path)
    print(f"[OK] Stage saved: {out_path}")

    # Isaac Sim setup
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

    # Run simulation until window is closed
    while simulation_app.is_running():
        world.step(render=True)

    print("Simulation ended.")
    simulation_app.close()


if __name__ == "__main__":
    main()
