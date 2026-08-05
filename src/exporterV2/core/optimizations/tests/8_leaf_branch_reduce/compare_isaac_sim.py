"""
compare_isaac_sim.py - Isaac Sim comparison for Leaf Branch Reduction

Loads three plant models side-by-side in Isaac Sim:
1. Left (x=-2.0m): Baseline - Fully articulated leaf (petiole + 3-link rachis)
2. Center (x=0.0m): Partial - Rachis reduced to 1 link
3. Right (x=+2.0m): Merged - Petiole+rachis merged into single branch

Usage:
    cd src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce
    ~/isaacsim/python.sh compare_isaac_sim.py

Requirements:
    - Isaac Sim installed
    - USD files generated (run generate_comparison_usd.py first)
"""

import os
import sys

# Isaac Sim imports
try:
    from isaacsim import SimulationApp
except ImportError:
    print("Error: Isaac Sim not found. Run with:")
    print("  ~/isaacsim/python.sh compare_isaac_sim.py")
    sys.exit(1)

# Start simulation with specific config
simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Usd, UsdGeom, Gf


def load_plant_at_position(usd_path: str, position: Gf.Vec3d, name_suffix: str):
    """
    Load a plant USD at a specific world position.
    
    Args:
        usd_path: Path to USD file
        position: World position (x, y, z)
        name_suffix: Suffix for unique naming
    
    Returns:
        USD prim path
    """
    # Get current stage
    stage = Usd.Stage.Open(usd_path)
    if not stage:
        print(f"Error: Cannot open USD file: {usd_path}")
        return None
    
    # Add reference at world position
    prim_path = f"/World/Plant_{name_suffix}"
    prim = add_reference_to_stage(usd_path, prim_path)
    
    if prim:
        # Set position
        xformable = UsdGeom.Xformable(prim)
        xform_op = xformable.AddTranslateOp()
        xform_op.Set(position)
        
        print(f"✓ Loaded {name_suffix} at position {position}")
        return prim_path
    else:
        print(f"✗ Failed to load {name_suffix}")
        return None


def setup_comparison_scene():
    """Setup Isaac Sim scene with three plants side-by-side."""
    print("\n" + "=" * 70)
    print("  Leaf Branch Reduction - Progressive Optimization Comparison")
    print("=" * 70)
    
    # Get USD file paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_dir = os.path.join(script_dir, "usd_output")
    baseline_path = os.path.join(output_dir, "baseline.usda")
    partial_path = os.path.join(output_dir, "partial.usda")
    merged_path = os.path.join(output_dir, "leaf_merged.usda")
    
    # Check files exist
    for name, path in [("baseline", baseline_path), ("partial", partial_path), ("merged", merged_path)]:
        if not os.path.exists(path):
            print(f"\n✗ Error: {name}.usda not found at {path}")
            print("Run generate_comparison_usd.py first:")
            print("  uv run python src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce/generate_comparison_usd.py")
            return False
    
    print(f"\n[Step 1] Loading USD files...")
    print(f"  Baseline: {baseline_path}")
    print(f"  Partial:  {partial_path}")
    print(f"  Merged:   {merged_path}")
    
    # Load plants at different positions
    print(f"\n[Step 2] Positioning plants...")
    
    # Left: Baseline (fully articulated)
    baseline_prim = load_plant_at_position(
        baseline_path,
        Gf.Vec3d(-2.0, 0.0, 0.0),  # 2m to the left
        "baseline"
    )
    
    # Center: Partial (rachis reduced to 1 link)
    partial_prim = load_plant_at_position(
        partial_path,
        Gf.Vec3d(0.0, 0.0, 0.0),  # Center
        "partial"
    )
    
    # Right: Merged (petiole+rachis merged)
    merged_prim = load_plant_at_position(
        merged_path,
        Gf.Vec3d(2.0, 0.0, 0.0),  # 2m to the right
        "merged"
    )
    
    if not baseline_prim or not partial_prim or not merged_prim:
        return False
    
    # Success
    print("\n[Step 3] Scene setup complete!")
    print("\n" + "=" * 70)
    print("  Visual Comparison Guide")
    print("=" * 70)
    print("\nCamera view (3 plants side-by-side):")
    print("  - Left (x=-2.0m):   Baseline - Fully articulated leaf")
    print("  - Center (x=0.0m):  Partial - Rachis reduced to 1 link")
    print("  - Right (x=+2.0m):  Merged - Petiole+rachis merged")
    
    print("\nLeaf structure:")
    print("  Baseline:  Petiole (1 link) → Rachis (3 links) → Petiolules (3)")
    print("  Partial:   Petiole (1 link) → Rachis (1 link) → Petiolules (3)")
    print("  Merged:    Merged (1 link) → Petiolules (3)")
    
    print("\nWhat to observe:")
    print("  1. Press PLAY to start physics simulation")
    print("  2. Baseline: Leaf highly flexible (4 DOF: petiole + 3 rachis links)")
    print("  3. Partial: Leaf less flexible (2 DOF: petiole + 1 rachis link)")
    print("  4. Merged: Leaf rigid (1 DOF: single merged segment)")
    print("  5. Total length: All leaves same length (~25cm)")
    print("  6. Petiolules: Progressively more clustered (baseline→partial→merged)")
    
    print("\nExpected behavior:")
    print("  ✓ Baseline: Leaf bends naturally, petiolules distributed along rachis")
    print("  ✓ Partial: Leaf bends at petiole only, petiolules clustered at tip")
    print("  ✓ Merged: Leaf completely rigid, petiolules at base of merged segment")
    print("  ✓ All: Similar visual appearance at rest")
    print("  ✓ All: Same total leaf length (10cm petiole + 15cm rachis = 25cm)")
    
    print("\nMetrics:")
    print("  Baseline: 13 total links (5 trunk + 1 branch + 1 petiole + 3 rachis + 3 petiolules)")
    print("  Partial:  11 total links (5 trunk + 1 branch + 1 petiole + 1 rachis + 3 petiolules)")
    print("  Merged:   10 total links (5 trunk + 1 branch + 1 merged + 3 petiolules)")
    print("  ")
    print("  Savings:  Partial saves 2 links, Merged saves 3 links (vs baseline)")
    
    print("\nVisual impact ranking:")
    print("  1. Baseline → Partial: Medium impact (rachis less articulated)")
    print("  2. Partial → Merged:   High impact (leaf becomes rigid)")
    print("  3. Baseline → Merged:  Very high impact (complete loss of leaf flexibility)")
    
    print("\n" + "=" * 70)
    print("  Press PLAY in Isaac Sim to start simulation")
    print("  Use mouse to orbit camera and observe differences")
    print("  Close window to exit")
    print("=" * 70 + "\n")
    
    return True


def main():
    """Main function."""
    try:
        # Setup scene
        if not setup_comparison_scene():
            print("\n✗ Scene setup failed!")
            simulation_app.close()
            return 1
        
        # Initialize world
        my_world = World(stage_units_in_meters=1.0)
        my_world.reset()
        
        # Run simulation loop
        print("\n[Simulation running - close window to exit]\n")
        while simulation_app.is_running():
            my_world.step(render=True)
        
        print("\n[Simulation finished]")
        simulation_app.close()
        return 0
    
    except KeyboardInterrupt:
        print("\n[Interrupted by user]")
        simulation_app.close()
        return 1
    
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        return 1


if __name__ == "__main__":
    sys.exit(main())
