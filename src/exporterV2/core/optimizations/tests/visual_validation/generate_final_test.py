"""
generate_final_test.py - Generate Before/After Comparison USD Files

Creates two USD files for visual comparison in Isaac Sim:
1. Baseline (no optimization)
2. Optimized with aggressive budget

Usage:
    cd /home/alessandro/isaacsim/autotom_digital_twin
    uv run python src/exporterV2/core/optimizations/tests/visual_validation/generate_final_test.py
"""

import sys
from pathlib import Path

# Path setup (same as generate_combinations_usd.py)
script_dir = Path(__file__).parent.resolve()
src_dir = (script_dir / "../../../../../../src").resolve()
optimizations_dir = (script_dir / "../..").resolve()

sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(optimizations_dir))

from exporterV2.adapters.groimp_csv import parse_csv_to_branches
from optimizer import BudgetOptimizer
from techniques.base import count_d6_joints
from exporterV2.core.usd import build_stage

# Configuration
DAY = 100
PLANT_ID = 1
AGGRESSIVE_BUDGET = 50  # Forces all optimizations

OUTPUT_DIR = Path(__file__).parent / "usd_output_before_after"
OUTPUT_DIR.mkdir(exist_ok=True)

BASELINE_USD = OUTPUT_DIR / "day_100_baseline.usda"
OPTIMIZED_USD = OUTPUT_DIR / "day_100_optimized_budget_50.usda"


def main():
    print("=" * 80)
    print("  Before/After Optimization USD Generator")
    print("=" * 80)
    
    # Load plant from CSV
    print(f"\n[STEP 1/4] Loading plant from CSV (day {DAY})...")
    branches, _ = parse_csv_to_branches(DAY, PLANT_ID)
    original_joints = count_d6_joints(branches)
    print(f"  ✓ Loaded: {len(branches)} branches, {original_joints} joints")
    
    # Calculate lower bound
    optimizer = BudgetOptimizer()
    lower_bound = optimizer.calculate_lower_bound(branches)
    print(f"  Lower bound: {lower_bound} joints")
    print(f"  Target budget: {AGGRESSIVE_BUDGET} joints (aggressive)")
    
    # Generate baseline USD
    print(f"\n[STEP 2/4] Generating baseline USD...")
    stage_baseline, _ = build_stage(str(BASELINE_USD), branches=branches)
    stage_baseline.GetRootLayer().Save()
    print(f"  ✓ Saved: {BASELINE_USD}")
    print(f"     Joints: {original_joints}")
    
    # Optimize with aggressive budget
    print(f"\n[STEP 3/4] Applying optimization (budget={AGGRESSIVE_BUDGET})...")
    
    # Temporarily modify budget
    original_budget = optimizer.config.max_joints
    optimizer.config.max_joints = AGGRESSIVE_BUDGET
    
    try:
        optimized_branches, report = optimizer.optimize(branches)
        final_joints = count_d6_joints(optimized_branches)
        
        # Print report
        print("\n" + "-" * 80)
        print(str(report))
        print("-" * 80 + "\n")
        
        # Generate optimized USD
        print(f"[STEP 4/4] Generating optimized USD...")
        stage_optimized, _ = build_stage(str(OPTIMIZED_USD), branches=optimized_branches)
        stage_optimized.GetRootLayer().Save()
        print(f"  ✓ Saved: {OPTIMIZED_USD}")
        print(f"     Joints: {final_joints}")
        
        # Summary
        print("\n" + "=" * 80)
        print("  Comparison Summary")
        print("=" * 80)
        print(f"Baseline:  {BASELINE_USD.name}")
        print(f"           {original_joints} joints, {len(branches)} branches")
        print()
        print(f"Optimized: {OPTIMIZED_USD.name}")
        print(f"           {final_joints} joints, {len(optimized_branches)} branches")
        print(f"           Budget: {AGGRESSIVE_BUDGET}")
        print(f"           Reduction: {original_joints - final_joints} joints ({((original_joints - final_joints) / original_joints * 100):.1f}%)")
        
        if report.success:
            print(f"           Status: ✓ Within budget")
        else:
            print(f"           Status: ⚠ Over budget by {final_joints - AGGRESSIVE_BUDGET}")
        
        print("\n" + "=" * 80)
        print("  Load in Isaac Sim:")
        print("=" * 80)
        print(f"Baseline:")
        print(f"  ~/isaacsim/python.sh -m isaacsim '{BASELINE_USD}'")
        print()
        print(f"Optimized:")
        print(f"  ~/isaacsim/python.sh -m isaacsim '{OPTIMIZED_USD}'")
        print()
        print(f"Or load both side-by-side using Isaac Sim file browser")
        print("=" * 80 + "\n")
        
    except ValueError as e:
        print(f"\n[ERROR] Optimization failed: {e}")
        print(f"[HINT] Budget {AGGRESSIVE_BUDGET} is below lower bound {lower_bound}")
        return 1
    finally:
        # Restore original budget
        optimizer.config.max_joints = original_budget
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
