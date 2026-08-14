"""
load_combination_isaacsim.py - Load baseline + combination side-by-side in Isaac Sim

Usage:
    ~/isaacsim/python.sh load_combination_isaacsim.py --combo 1

This loads:
  - Left:  Baseline (combo_0_baseline.usda)
  - Right: Selected combination (e.g. combo_1_p.usda for --combo 1)

Combinations:
  0: Baseline (same as baseline, no point loading side-by-side)
  1: P (Petiole Lock)
  2: L (Lateral Reduce)
  3: S (Stem Collapse)
  4: F (Leaf Reduce)
  5: P+L
  6: P+F
  7: Full (All techniques)
"""

import argparse
import sys
from pathlib import Path

# Isaac Sim imports
from isaacsim import SimulationApp

# Parse args BEFORE SimulationApp (headless mode check)
parser = argparse.ArgumentParser(description="Load technique combination for visual comparison")
parser.add_argument("--combo", type=int, required=True, help="Combination ID (0-7)")
parser.add_argument("--headless", action="store_true", help="Run in headless mode")
args = parser.parse_args()

# Start Isaac Sim
simulation_app = SimulationApp({"headless": args.headless})

from omni.isaac.core import World
from pxr import Usd, UsdGeom, Gf

# Combination metadata
COMBINATIONS = [
    {"id": 0, "label": "Baseline", "file": "combo_0_baseline.usda"},
    {"id": 1, "label": "P (Petiole Lock)", "file": "combo_1_p.usda"},
    {"id": 2, "label": "L (Lateral Reduce)", "file": "combo_2_l.usda"},
    {"id": 3, "label": "S (Stem Collapse)", "file": "combo_3_s.usda"},
    {"id": 4, "label": "F (Leaf Reduce)", "file": "combo_4_f.usda"},
    {"id": 5, "label": "P+L", "file": "combo_5_p_l.usda"},
    {"id": 6, "label": "P+F", "file": "combo_6_p_f.usda"},
    {"id": 7, "label": "Full Optimization", "file": "combo_7_full.usda"},
]


def main():
    # Validate combo ID
    if args.combo < 0 or args.combo > 7:
        print(f"Error: --combo must be 0-7, got {args.combo}")
        sys.exit(1)
    
    combo = COMBINATIONS[args.combo]
    
    # Setup paths
    script_dir = Path(__file__).parent.resolve()
    usd_dir = script_dir / "usd_output_combinations"
    
    baseline_path = str(usd_dir / "combo_0_baseline.usda")
    combo_path = str(usd_dir / combo["file"])
    
    if not Path(baseline_path).exists():
        print(f"Error: Baseline USD not found: {baseline_path}")
        print("Run generate_combinations_usd.py first")
        sys.exit(1)
    
    if not Path(combo_path).exists():
        print(f"Error: Combination USD not found: {combo_path}")
        print("Run generate_combinations_usd.py first")
        sys.exit(1)
    
    print("=" * 70)
    print("  Optimization Technique Comparison — Isaac Sim Loader")
    print("=" * 70)
    print(f"\nCombo {combo['id']}: {combo['label']}")
    print(f"Baseline: {baseline_path}")
    print(f"Combo:    {combo_path}")
    print()
    
    # Create world
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    
    # Load baseline on left (-1.5m X offset)
    print("Loading baseline (left)...")
    baseline_prim = stage.DefinePrim("/Baseline", "Xform")
    baseline_prim.GetReferences().AddReference(baseline_path)
    UsdGeom.Xformable(baseline_prim).AddTranslateOp().Set(Gf.Vec3d(-1.5, 0, 0))
    
    # Load combination on right (+1.5m X offset)
    print(f"Loading combo {combo['id']} (right)...")
    combo_prim = stage.DefinePrim(f"/Combo_{combo['id']}", "Xform")
    combo_prim.GetReferences().AddReference(combo_path)
    UsdGeom.Xformable(combo_prim).AddTranslateOp().Set(Gf.Vec3d(1.5, 0, 0))
    
    # Add ground plane
    print("Adding ground plane...")
    world.scene.add_default_ground_plane()
    
    # Setup camera to view both plants
    print("Setting up camera...")
    from omni.isaac.core.utils.viewports import set_camera_view
    set_camera_view(
        eye=[0, -5, 2],      # Camera position (center, back, up)
        target=[0, 0, 1],    # Look at center, 1m up
        camera_prim_path="/OmniverseKit_Persp"
    )
    
    print()
    print("✓ Scene loaded successfully")
    print()
    print("Controls:")
    print("  - Left:  Baseline plant")
    print("  - Right: Optimized plant")
    print("  - Play simulation to see physics")
    print("  - Use mouse to orbit camera")
    print()
    print("Press Ctrl+C to exit")
    print()
    
    # Reset world (prepares for simulation)
    world.reset()
    
    # Run simulation loop (if not headless)
    if not args.headless:
        while simulation_app.is_running():
            world.step(render=True)
    
    simulation_app.close()


if __name__ == "__main__":
    main()
