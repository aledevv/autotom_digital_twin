"""
load_final_test.py - Load Before/After USD Comparison in Isaac Sim

Loads baseline and optimized USD files side-by-side for visual comparison.

Usage:
    ~/isaacsim/python.sh src/exporterV2/core/optimizations/tests/visual_validation/load_final_test.py
    
Or use wrapper:
    ./load_final_test.sh
"""

import sys
from pathlib import Path

# Bootstrap Isaac Sim first
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Now import Isaac and USD modules
from pxr import Usd, UsdGeom, Gf
import omni.usd
import omni.kit.actions.core
from isaacsim.core.api import World

# Paths to USD files
SCRIPT_DIR = Path(__file__).parent.resolve()
USD_DIR = SCRIPT_DIR / "usd_output_before_after"
BASELINE_USD = USD_DIR / "day_100_baseline.usda"
OPTIMIZED_USD = USD_DIR / "day_100_optimized_budget_50.usda"

# Spacing between plants (in meters)
SPACING = 1.0


def main():
    """Load baseline and optimized side-by-side."""
    print("=" * 80)
    print("  Before/After Optimization Comparison")
    print("=" * 80)
    
    # Check files exist
    if not BASELINE_USD.exists():
        print(f"\n[ERROR] Baseline USD not found: {BASELINE_USD}")
        print("[HINT] Run generate_final_test.py first")
        simulation_app.close()
        sys.exit(1)
    
    if not OPTIMIZED_USD.exists():
        print(f"\n[ERROR] Optimized USD not found: {OPTIMIZED_USD}")
        print("[HINT] Run generate_final_test.py first")
        simulation_app.close()
        sys.exit(1)
    
    # Create a new empty stage
    print("\n[STEP 1/3] Creating comparison stage...")
    stage = Usd.Stage.CreateNew("tmp_before_after_comparison.usda")
    
    # Set up axis and scale
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Add baseline plant on the left
    print(f"\n[STEP 2/3] Loading baseline (left side)...")
    baseline_ref = stage.DefinePrim("/Baseline", "Xform")
    baseline_ref.GetReferences().AddReference(str(BASELINE_USD))
    
    # Position baseline at origin
    xform_baseline = UsdGeom.Xformable(baseline_ref)
    xform_baseline.AddTranslateOp().Set(Gf.Vec3d(-SPACING/2, 0, 0))
    
    print(f"  ✓ Loaded: {BASELINE_USD.name}")
    print(f"     Position: ({-SPACING/2:.1f}, 0, 0)")
    
    # Add optimized plant on the right
    print(f"\n[STEP 3/3] Loading optimized (right side)...")
    optimized_ref = stage.DefinePrim("/Optimized", "Xform")
    optimized_ref.GetReferences().AddReference(str(OPTIMIZED_USD))
    
    # Position optimized to the right
    xform_optimized = UsdGeom.Xformable(optimized_ref)
    xform_optimized.AddTranslateOp().Set(Gf.Vec3d(SPACING/2, 0, 0))
    
    print(f"  ✓ Loaded: {OPTIMIZED_USD.name}")
    print(f"     Position: ({SPACING/2:.1f}, 0, 0)")
    
    # Save and load the comparison stage
    stage.GetRootLayer().Save()
    print(f"\n  ✓ Comparison stage created")
    
    # Load in Isaac Sim
    omni.usd.get_context().open_stage("tmp_before_after_comparison.usda")
    print("  ✓ Stage opened in Isaac Sim")
    
    # Set camera lighting
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
    print("  Comparison loaded:")
    print("=" * 80)
    print(f"  LEFT  (Baseline):  165 joints, 136 branches")
    print(f"  RIGHT (Optimized): 121 joints, 119 branches")
    print(f"  Reduction: 44 joints (26.7%)")
    print()
    print("  Visual differences to look for:")
    print("  - Trunk: 10 links → 3 links (thicker segments)")
    print("  - Leaves: petiole+rachis merged into single segment")
    print("  - Positions preserved (same absolute heights)")
    print()
    print("  ✓ Simulation running — close window to exit")
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
