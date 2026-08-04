"""
main.py - exporterV2 Entry Point

Generates tree USD and loads it in Isaac Sim in one step.

Run with static config:
    ~/isaacsim/python.sh src/exporterV2/main.py
    
Run with CSV data:
    ~/isaacsim/python.sh src/exporterV2/main.py --day 1

Or use wrapper script:
    ./run_mainV2.sh [--day N]
"""

import os
import sys
import argparse

# Parse arguments BEFORE initializing SimulationApp
parser = argparse.ArgumentParser(description="exporterV2 Tree Loader")
parser.add_argument("--day", type=int, help="Load plant from CSV for specified day")
parser.add_argument("--plant-id", type=int, default=1, help="Plant ID (default: 1)")
args = parser.parse_args()

# Bootstrap Isaac Sim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Now import USD and Isaac modules
from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
import omni.kit.actions.core
from isaacsim.core.api import World

# Import our modules (need to add parent to path)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

from exporterV2.usd import build_stage, get_output_usd_path
from exporterV2.physics import apply_physx_scene_settings, apply_physx_articulation_settings
from exporterV2.tree_config import BRANCHES
# ==============================================================================
# MAIN
# ==============================================================================

def main():
    """Generate tree USD and load in Isaac Sim."""
    print("=" * 80)
    print("  exporterV2 - Tree Model Generator & Loader")
    print("=" * 80)
    
    # Determine configuration source and USD path
    if args.day is not None:
        # Load from CSV
        from exporterV2.csv_data import parse_csv_to_branches
        print(f"\n[CONFIG] Loading plant from CSV (day {args.day}, plant_id {args.plant_id})")
        branches, json_path = parse_csv_to_branches(args.day, args.plant_id)
        print(f"[CONFIG] Configuration saved: {json_path}")
        
        # Use day-specific USD path
        base_path = get_output_usd_path()
        usd_path = base_path.replace("tree_v2.usda", f"tree_v2_day_{args.day}.usda")
    else:
        # Use static config
        print(f"\n[CONFIG] Using static configuration from tree_config.py")
        branches = BRANCHES
        usd_path = get_output_usd_path()
    
    # Step 1: Generate USD
    print("\n[STEP 1/3] Generating tree USD stage...")
    total_links = sum(b["n_links"] for b in branches)
    print(f"  Configuration: {len(branches)} branches, {total_links} total links")
    
    stage, stem_path = build_stage(usd_path, branches=branches)
    
    # Step 2: Apply PhysX settings
    print("\n[STEP 2/3] Applying PhysX configuration...")
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)
    
    stage.GetRootLayer().Save()
    print(f"  ✓ Stage saved: {usd_path}")
    
    # Step 3: Load in Isaac Sim
    print("\n[STEP 3/3] Loading in Isaac Sim...")
    omni.usd.get_context().open_stage(usd_path)
    print("  ✓ Stage opened")
    
    # Set camera lighting mode
    try:
        reg = omni.kit.actions.core.get_action_registry()
        action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
            print("  ✓ Camera lighting applied")
    except Exception as e:
        print(f"  ⚠ Lighting action not available: {e}")
    
    # Initialize simulation
    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()
    
    print("\n" + "=" * 80)
    print("  ✓ Simulation running — close the window to exit")
    print("=" * 80 + "\n")
    
    # Run simulation loop
    while simulation_app.is_running():
        my_world.step(render=True)
    
    print("\n[INFO] Simulation finished.")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
        simulation_app.close()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
