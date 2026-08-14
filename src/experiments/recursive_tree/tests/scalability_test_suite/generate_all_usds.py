"""
generate_all_usds.py

Generates USD files for all 160 days of the digital twin simulation in a single
headless Isaac Sim batch run. This avoids the overhead of booting Isaac Sim 160 times.

Usage:
    ~/isaacsim/python.sh src/experiments/recursive_tree/tests/scalability_test_suite/generate_all_usds.py
"""

import os
import sys
import glob
import re

# Bootstrap Isaac Sim in headless mode
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdPhysics, PhysxSchema, Gf
import omni.usd

# Add project root to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
os.path.join(PROJECT_ROOT, "src") = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", "..", "..", ".."))
if os.path.join(PROJECT_ROOT, "src") not in sys.path:
    sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from exporterV2.adapters.groimp_csv.parser import parse_csv_to_branches
from exporterV2.profiles.tomato_default import TOMATO_PROFILE
from exporterV2.core.usd import build_stage
from exporterV2.core.physics import apply_physx_scene_settings, apply_physx_articulation_settings

def generate_usd_for_day(day: int, output_usd_path: str):
    """Generate and save the USDA file for a single day."""
    print(f"Generating Day {day} -> {output_usd_path}")
    
    # 1. Parse CSV (exact same profile as production)
    branches, terminal_bodies, _ = parse_csv_to_branches(
        day=day,
        plant_id=1,
        include_terminal_bodies=True,
        profile=TOMATO_PROFILE
    )
    
    # 2. Build USD Stage
    # (omni.usd.get_context().new_stage() is called internally if stage doesn't exist,
    # but to be safe and clean, we close the existing stage if any)
    omni.usd.get_context().close_stage()
    
    stage, stem_path = build_stage(output_usd_path, branches=branches, terminal_bodies=terminal_bodies)
    
    # 3. Apply PhysX settings
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)
    
    # 4. Save to ASCII USD
    stage.GetRootLayer().Save()

def main():
    CSV_DIR = os.path.join(os.path.join(PROJECT_ROOT, "src"), "data", "simulation_output", "dynamic_output", "graphs")
    OUTPUT_DIR = os.path.join(os.path.join(PROJECT_ROOT, "src"), "output", "thesis_usda_days")
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    pattern = os.path.join(CSV_DIR, "graph_day_*.csv")
    files = sorted(glob.glob(pattern))
    
    days = []
    for f in files:
        match = re.search(r"graph_day_(\d+)\.csv", os.path.basename(f))
        if match:
            days.append(int(match.group(1)))
            
    days = sorted(days)
    print(f"Found {len(days)} days to process.")
    
    for day in days:
        output_usd = os.path.join(OUTPUT_DIR, f"tree_v2_day_{day}.usda")
        try:
            generate_usd_for_day(day, output_usd)
        except Exception as e:
            print(f"Error on day {day}: {e}")
            import traceback
            traceback.print_exc()

    print(f"\nFinished generating {len(days)} USDA files in {OUTPUT_DIR}")
    simulation_app.close()

if __name__ == "__main__":
    main()
