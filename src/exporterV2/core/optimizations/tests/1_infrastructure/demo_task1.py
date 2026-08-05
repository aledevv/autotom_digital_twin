"""
demo_task1.py - Demo Script for Task 1 (Setup Infrastructure)

Demonstrates:
- Configuration loading
- Joint calculation
- Lower bound calculation
- Basic optimization report (no techniques applied yet)

Run with: uv run python src/exporterV2/core/optimizations/tests/demo_task1.py
"""

import sys
import os

# Add optimizations directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")  # Go up two levels to optimizations/
sys.path.insert(0, optimizations_dir)

from optimizer import BudgetOptimizer


def main():
    print("=" * 70)
    print("  Task 1 Demo: Joint-Budget Optimization Infrastructure")
    print("=" * 70)
    
    # Initialize optimizer
    print("\n[Step 1] Loading configuration from budget_config.yaml...")
    optimizer = BudgetOptimizer()
    print(f"  ✓ Budget: {optimizer.config.max_joints} joints")
    print(f"  ✓ Warning threshold: {optimizer.config.warning_threshold} joints")
    print(f"  ✓ Techniques configured: {len(optimizer.config.techniques)}")
    
    # Create synthetic plant for demonstration
    print("\n[Step 2] Creating synthetic plant configuration...")
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.05,
            "height": 0.20,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "Branch_r1_o0",
            "parent": "trunk",
            "attach_link": 2,
            "n_links": 3,
            "radius": 0.02,
            "height": 0.15,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "Branch_r2_o0",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.02,
            "height": 0.15,
            "tilt": 45.0,
            "rot": 180.0,
        },
        {
            "id": "Petiole_r1_o0",
            "parent": "Branch_r1_o0",
            "attach_link": 1,
            "n_links": 2,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 30.0,
            "rot": 90.0,
        },
        {
            "id": "Petiole_r2_o0",
            "parent": "Branch_r2_o0",
            "attach_link": 1,
            "n_links": 2,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 30.0,
            "rot": 90.0,
        },
    ]
    print(f"  ✓ Created plant with {len(branches)} components")
    for b in branches:
        print(f"    - {b['id']}: {b['n_links']} links")
    
    # Calculate joints
    print("\n[Step 3] Calculating total joints...")
    total_joints = optimizer.calculate_total_joints(branches)
    print(f"  ✓ Total joints: {total_joints}")
    
    # Calculate lower bound
    print("\n[Step 4] Calculating structural lower bound...")
    lower_bound = optimizer.calculate_lower_bound(branches)
    print(f"  ✓ Lower bound: {lower_bound} joints")
    print(f"    (1 trunk + 2 laterals + 2 petioles = {lower_bound})")
    
    # Run optimization (no techniques implemented yet)
    print("\n[Step 5] Running optimization...")
    optimized, report = optimizer.optimize(branches)
    
    print("\n[Step 6] Optimization Report:")
    print(report)
    
    print("\n[Step 7] Summary:")
    print(f"  • Original configuration: {report.original_joints} joints")
    print(f"  • Budget: {report.budget} joints")
    print(f"  • Lower bound: {report.lower_bound} joints")
    print(f"  • Optimization needed: {'No' if report.original_joints <= report.budget else 'Yes'}")
    print(f"  • Techniques applied: {len(report.technique_reports)}")
    print(f"    (Note: Techniques not yet implemented - will be added in Tasks 4-8)")
    
    print("\n" + "=" * 70)
    print("  ✓ Task 1 Complete: Infrastructure setup successful!")
    print("=" * 70)
    print("\nNext Steps:")
    print("  - Task 2: Implement collision detection (sphere + AABB)")
    print("  - Task 3: Implement geometry remapping")
    print("  - Task 4-8: Implement optimization techniques")


if __name__ == "__main__":
    main()
