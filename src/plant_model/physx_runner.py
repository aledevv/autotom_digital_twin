"""
physx_runner.py

Isaac Sim entry-point for the articulated stem pipeline (V2):
  1. Load plant snapshot from CSV
  2. Build the USD stage with the articulated stem (usd_exporter_v2)
  3. Inject PhysicsScene + PhysxArticulationAPI (physx_utils)
  4. Save the final stage and run the simulation loop

Run with:
    ~/isaacsim/python.sh physx_runner.py --csv data/.../graph_day_160.csv --day 160 --plant 1
"""

import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SRC_DIR      = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporter_v2 import build_stem_stage
from plant_model.physx_utils import apply_physx_scene_settings, apply_physx_articulation_settings


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def parse_args():
    parser = argparse.ArgumentParser(description="Build and simulate the articulated stem V2 in Isaac Sim.")
    parser.add_argument("--csv", default="data/simulation_output/dynamic_output/graphs/graph_day_160.csv",
                         help="Path to the simulation day CSV.")
    parser.add_argument("--day",   type=int, default=160, help="Simulation day.")
    parser.add_argument("--plant", type=int, default=1,   help="Plant ID.")
    parser.add_argument("--out",   default=None,          help="Output .usda path (default: output/day_<day>/plant_day<day>_stem_v2.usda).")
    return parser.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    out_path = args.out or os.path.join(
        PROJECT_ROOT, "output", f"day_{args.day}", f"plant_day{args.day}_stem_v2.usda"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    # 1. Load plant data and build the USD stage
    print(f"[INFO] Loading snapshot day={args.day} plant={args.plant} from {args.csv}")
    snapshot = load_snapshot(args.csv, day=args.day, plant_id=args.plant)

    print("[INFO] Building articulated stem stage V2...")
    stage, stem_path = build_stem_stage(snapshot, out_path)

    # 2. Inject PhysX config (only available here, inside SimulationApp)
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)

    # 3. Save the complete stage (geometry + physics)
    stage.GetRootLayer().Save()
    print(f"[OK] Stage saved with PhysX config: {out_path}")

    # 4. Open in Isaac Sim and run the simulation loop
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
