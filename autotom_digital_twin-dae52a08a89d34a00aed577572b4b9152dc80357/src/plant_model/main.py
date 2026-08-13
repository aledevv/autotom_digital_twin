import os
import sys
import argparse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SRC_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# Bootstrap Isaac Sim runtime
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema, Gf
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporter import export_plant_usd

def setup_physx(stage):
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableStabilizationAttr().Set(True)

def parse_args():
    parser = argparse.ArgumentParser(description="Export and view V1 static plant.")
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
        PROJECT_ROOT, "output", f"day_{args.day}", f"plant_day{args.day}_static.usda"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    
    print(f"[INFO] Loading snapshot day={args.day} plant={args.plant} from {csv_path}")
    snapshot = load_snapshot(csv_path, day=args.day, plant_id=args.plant)
    
    if os.path.exists(out_path):
        os.remove(out_path)
        
    print("[INFO] Exporting static USD (V1)...")
    export_plant_usd(snapshot, out_path)
    
    # Re-open the stage to inject PhysX Scene (useful if there are rigid bodies)
    stage = Usd.Stage.Open(out_path)
    if stage:
        setup_physx(stage)
        stage.GetRootLayer().Save()
        
    print(f"[OK] Static Stage saved: {out_path}")
    
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
    
    while simulation_app.is_running():
        world.step(render=True)
        
    print("Simulation ended.")
    simulation_app.close()

if __name__ == "__main__":
    main()
