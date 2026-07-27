"""
mainV2.py

Entry-point for the V2 pipeline (articulated stem simulation):
  1. Bootstrap SimulationApp (must precede any pxr/omni imports)
  2. Load the plant snapshot for a given day (loader.py)
  3. Build the USD stage with the articulated stem (usd_exporter_v2.py)
  4. Inject PhysicsScene + PhysxArticulationAPI (physx_utils.py)
  5. Save, open in Isaac Sim, run the simulation loop

Run with:
    ~/isaacsim/python.sh <path>/mainV2.py --day 160 --plant 1
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
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporter_v2 import build_stem_stage
from plant_model.physx_utils import (
    apply_physx_scene_settings,
    apply_physx_articulation_settings,
)
from plant_model.usd_exporter_builder import export_plant_usd_builder

def parse_args():
    parser = argparse.ArgumentParser(description="Build and simulate the articulated stem V2 in Isaac Sim.")
    parser.add_argument("--day",   type=int, default=160, help="Simulation day.")
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
        PROJECT_ROOT, "output", f"day_{args.day}", f"plant_day{args.day}_stem_v2.usda"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"[INFO] Loading snapshot day={args.day} plant={args.plant} from {csv_path}")
    snapshot = load_snapshot(csv_path, day=args.day, plant_id=args.plant)

    print("[INFO] Building articulated stem stage V2...")
    stage, stem_path = build_stem_stage(snapshot, out_path)

    print("[INFO] Injecting PhysX config...")
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)

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

    export_plant_usd_builder(snapshot, output_path=out_path, validate=True)

    while simulation_app.is_running():
        world.step(render=True)

    print("Simulation ended.")
    simulation_app.close()


if __name__ == "__main__":
    main()