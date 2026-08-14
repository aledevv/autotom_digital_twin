"""
generate_final_comparison.py - Generate Before/After Comparison USD Files

Creates two USD files for visual comparison in Isaac Sim:
1. Baseline (no optimization)
2. Optimized with aggressive budget

Usage:
    cd /home/alessandro/isaacsim/autotom_digital_twin
    uv run python src/exporterV2/demos/optimization_visual_validation/generate_final_comparison.py
"""

import sys
from pathlib import Path

# Path setup
script_dir = Path(__file__).parent.resolve()
src_dir = script_dir.parents[2]
optimizations_dir = (script_dir / "../../core/optimizations").resolve()

sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(optimizations_dir))

from exporterV2.adapters.groimp_csv import parse_csv_to_branches
from optimizer import BudgetOptimizer
from techniques.base import count_d6_joints
from exporterV2.core.usd import build_stage

# Configuration
DAY = 100
PLANT_ID = 1
AGGRESSIVE_BUDGET = 50

OUTPUT_DIR = Path(__file__).parent / "usd_output_before_after"
OUTPUT_DIR.mkdir(exist_ok=True)

BASELINE_USD = OUTPUT_DIR / f"day_{DAY}_baseline.usda"
OPTIMIZED_USD = OUTPUT_DIR / f"day_{DAY}_optimized_budget_{AGGRESSIVE_BUDGET}.usda"


def categorize_branches(branches):
    """Group branches by type and count joints per category."""
    categories = {
        "trunk":     {"count": 0, "joints": 0},
        "lateral":   {"count": 0, "joints": 0},
        "petiole":   {"count": 0, "joints": 0},
        "rachis":    {"count": 0, "joints": 0},
        "petiolule": {"count": 0, "joints": 0},
        "other":     {"count": 0, "joints": 0},
    }

    for b in branches:
        bid = b.get("id", "")
        n = b.get("n_links", 1)
        
        # Only count D6 joints (exclude Fixed joints from petiolules)
        joint_type = b.get("joint_type", "d6").lower()
        if joint_type == "fixed":
            # Still count the object, but mark joints as 0 for budget
            n_budget = 0
        else:
            n_budget = n

        if bid == "trunk" or bid.startswith("trunk"):
            cat = "trunk"
        elif bid.startswith("Branch_r"):
            cat = "lateral"
        elif "_petiole" in bid and "_merged" not in bid:
            cat = "petiole"
        elif "petiolule" in bid.lower():
            cat = "petiolule"  # Check BEFORE rachis (petiolules contain "_rachis" in name!)
        elif "_rachis" in bid or "_merged" in bid:
            cat = "rachis"
        else:
            cat = "other"

        categories[cat]["count"] += 1
        categories[cat]["joints"] += n_budget  # Use budget-aware count

    return categories


def print_comparison_table(branches_before, branches_after, budget):
    """Print a detailed before/after joint breakdown table."""
    before = categorize_branches(branches_before)
    after  = categorize_branches(branches_after)

    total_before = sum(v["joints"] for v in before.values())
    total_after  = sum(v["joints"] for v in after.values())

    W = 74
    print("\n" + "=" * W)
    print("  JOINT BREAKDOWN: Before vs After Optimization")
    print("=" * W)
    print(f"  {'Category':<14} {'Objects':>8}  {'Joints':>8}  {'After':>8}  {'Delta':>8}  {'Change':>8}")
    print("  " + "-" * (W - 2))

    for cat in ["trunk", "lateral", "petiole", "rachis", "petiolule", "other"]:
        b = before[cat]
        a = after[cat]
        # Show category if it has joints before OR after (not just count > 0)
        if b["joints"] == 0 and a["joints"] == 0:
            continue
        delta = a["joints"] - b["joints"]
        pct   = (delta / b["joints"] * 100) if b["joints"] > 0 else 0
        bdelta = a["count"] - b["count"]
        br_str = f"{b['count']}" if bdelta == 0 else f"{b['count']} → {a['count']}"
        d_str  = f"{delta:+d}" if delta != 0 else "–"
        p_str  = f"{pct:+.0f}%" if delta != 0 else "–"
        
        # Special note for petiolules converted to Fixed
        if cat == "petiolule" and b["joints"] > 0 and a["joints"] == 0:
            p_str = "→Fixed"  # Indicate they were locked, not removed
        
        print(f"  {cat:<14} {br_str:>8}  {b['joints']:>8}  {a['joints']:>8}  {d_str:>8}  {p_str:>8}")

    print("  " + "-" * (W - 2))
    total_delta = total_after - total_before
    total_pct   = total_delta / total_before * 100 if total_before > 0 else 0
    print(f"  {'TOTAL':<14} {'':>8}  {total_before:>8}  {total_after:>8}  {total_delta:+8d}  {total_pct:>+7.1f}%")
    print("  " + "-" * (W - 2))
    
    # Use count_d6_joints on branches_after for accurate final count (excludes Fixed petiolules)
    from techniques.base import count_d6_joints
    actual_final_joints = count_d6_joints(branches_after)
    status = "✓ Within budget" if actual_final_joints <= budget else f"⚠ Over by {actual_final_joints - budget}"
    print(f"  Budget: {budget}  |  Final: {actual_final_joints} joints  |  {status}")
    print("=" * W + "\n")


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
    print(f"  Target budget: {AGGRESSIVE_BUDGET} joints")

    # Generate baseline USD
    print(f"\n[STEP 2/4] Generating baseline USD...")
    stage_baseline, _ = build_stage(str(BASELINE_USD), branches=branches)
    stage_baseline.GetRootLayer().Save()
    print(f"  ✓ Saved: {BASELINE_USD.name}  ({original_joints} joints)")

    # Optimize
    print(f"\n[STEP 3/4] Applying optimization (budget={AGGRESSIVE_BUDGET})...")
    original_budget = optimizer.config.max_joints
    optimizer.config.max_joints = AGGRESSIVE_BUDGET

    try:
        optimized_branches, report = optimizer.optimize(branches)
        final_joints = count_d6_joints(optimized_branches)

        print("\n" + "-" * 80)
        print(str(report))
        print("-" * 80 + "\n")

        # Generate optimized USD
        print(f"[STEP 4/4] Generating optimized USD...")
        stage_optimized, _ = build_stage(str(OPTIMIZED_USD), branches=optimized_branches)
        
        # Save optimization metadata as custom attributes on root layer
        root_prim = stage_optimized.GetPrimAtPath("/World")
        if root_prim:
            root_prim.SetCustomDataByKey("optimization:baseline_joints", original_joints)
            root_prim.SetCustomDataByKey("optimization:final_joints", final_joints)
            root_prim.SetCustomDataByKey("optimization:minimum_achievable", report.minimum_achievable)
            root_prim.SetCustomDataByKey("optimization:budget", AGGRESSIVE_BUDGET)
        
        stage_optimized.GetRootLayer().Save()
        print(f"  ✓ Saved: {OPTIMIZED_USD.name}  ({final_joints} joints)")
        print(f"  ✓ Saved metadata: minimum_achievable={report.minimum_achievable}")

        # Detailed per-category table
        print_comparison_table(branches, optimized_branches, AGGRESSIVE_BUDGET)

        # Final summary
        print("=" * 80)
        print("  Summary")
        print("=" * 80)
        print(f"  Baseline:  {original_joints} joints, {len(branches)} branches")
        print(f"  Optimized: {final_joints} joints, {len(optimized_branches)} branches")
        reduction = original_joints - final_joints
        print(f"  Reduction: {reduction} joints ({reduction / original_joints * 100:.1f}%)")
        print(f"  Status:    {'✓ Within budget' if report.success else f'⚠ Over by {final_joints - AGGRESSIVE_BUDGET}'}")
        print("=" * 80 + "\n")

    except ValueError as e:
        print(f"\n[ERROR] Optimization failed: {e}")
        print(f"[HINT] Budget {AGGRESSIVE_BUDGET} is below lower bound {lower_bound}")
        return 1
    finally:
        optimizer.config.max_joints = original_budget

    return 0


if __name__ == "__main__":
    sys.exit(main())
