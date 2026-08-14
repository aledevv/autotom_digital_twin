#!/usr/bin/env python3
"""
demo_integration.py - Demonstrate Integration Pipeline

Shows the complete CSV → optimize → report flow without Isaac Sim.

Usage:
    cd /home/alessandro/isaacsim/autotom_digital_twin
    uv run python src/exporterV2/core/optimizations/tests/demo_integration.py [--day N] [--optimize]
"""

import argparse
import sys
from pathlib import Path

# Add paths (adjust based on where script is run from)
SCRIPT_DIR = Path(__file__).parent
ROOT_DIR = SCRIPT_DIR.parent.parent.parent.parent
sys.path.insert(0, str(ROOT_DIR / "src"))

from exporterV2.adapters.groimp_csv import parse_csv_to_branches
from exporterV2.core.optimizations import BudgetOptimizer
from exporterV2.core.optimizations.techniques.base import count_d6_joints

# ANSI colors
RED = '\033[91m'
BLUE = '\033[94m'
GREEN = '\033[92m'
RESET = '\033[0m'


def main():
    parser = argparse.ArgumentParser(description="Demo optimization pipeline")
    parser.add_argument("--day", type=int, default=100, help="Day to load from CSV")
    parser.add_argument("--optimize", action="store_true", help="Apply optimization")
    args = parser.parse_args()
    
    print("=" * 80)
    print("  Optimization Pipeline Demo")
    print("=" * 80)
    
    # Step 1: Parse CSV
    print(f"\n[STEP 1/3] Loading plant from CSV (day {args.day})...")
    branches, json_path = parse_csv_to_branches(args.day, plant_id=1)
    original_joints = count_d6_joints(branches)
    print(f"  ✓ Loaded: {len(branches)} branches, {original_joints} joints")
    
    # Step 2: Optimize (optional)
    if args.optimize:
        print("\n[STEP 2/3] Applying optimization...")
        try:
            optimizer = BudgetOptimizer()
            branches, report = optimizer.optimize(branches)
            
            # Print report
            print("\n" + "=" * 80)
            print(str(report))
            print("=" * 80 + "\n")
            
            if report.success:
                print(f"{GREEN}✓ Optimization successful{RESET}")
            else:
                print(f"{RED}⚠ Over budget: {report.final_joints}/{report.budget} joints{RESET}")
                
        except ValueError as e:
            print(f"\n{RED}[ERROR] {e}{RESET}", file=sys.stderr)
            print(f"{BLUE}[HINT] Remove --optimize flag to generate unoptimized USD{RESET}", file=sys.stderr)
            return 1
    else:
        print("\n[STEP 2/3] Skipping optimization (use --optimize to enable)")
    
    # Step 3: Summary
    final_joints = count_d6_joints(branches)
    print(f"\n[STEP 3/3] Final configuration:")
    print(f"  Branches: {len(branches)}")
    print(f"  Joints: {final_joints}")
    
    if args.optimize and final_joints < original_joints:
        reduction = original_joints - final_joints
        pct = (reduction / original_joints) * 100
        print(f"  Reduction: {reduction} joints ({pct:.1f}%)")
    
    print("\n" + "=" * 80)
    print(f"  {GREEN}✓ Demo complete{RESET}")
    print("=" * 80 + "\n")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
