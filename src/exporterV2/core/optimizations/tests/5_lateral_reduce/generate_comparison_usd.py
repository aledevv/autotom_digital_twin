"""
generate_comparison_usd.py - Generate USD files for Lateral Branch Reduction comparison

Generates two USD files for visual comparison in Isaac Sim:
1. baseline.usda - Lateral branches with full segments (e.g., 3 links)
2. lateral_reduce.usda - Lateral branches reduced to minimum (1 link)

Usage:
    python generate_comparison_usd.py
    
Output:
    usd_output/baseline.usda
    usd_output/lateral_reduce.usda
"""

import sys
import os

# Add paths for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../../../..'))

from exporterV2.core.usd.stage import build_stage
from exporterV2.core.optimizations.techniques.lateral_reduce import LateralBranchReductionTechnique


def create_plant_config():
    """
    Create plant configuration with lateral branches for testing.
    
    Returns:
        List of branch configurations
    """
    branches = [
        # Trunk (main stem)
        {
            "id": "trunk",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "height": 0.08,  # 8cm per link = 40cm total
            "radius": 0.10,  # 10cm
            "tilt": 0.0,
            "rot": 0.0,
            "roll": 0.0,
        },
        # Lateral Branch 1 (3 links, small radius, low attachment)
        {
            "id": "Branch_r1_o0",
            "parent": "trunk",
            "attach_link": 2,
            "n_links": 3,
            "height": 0.10,  # 10cm per link = 30cm total
            "radius": 0.06,  # 6cm
            "tilt": 45.0,
            "rot": 0.0,
            "roll": 0.0,
        },
        # Petiole on Branch 1
        {
            "id": "Petiole_r1_o0",
            "parent": "Branch_r1_o0",
            "attach_link": 2,
            "n_links": 2,
            "height": 0.07,  # 7cm per link = 14cm total
            "radius": 0.03,  # 3cm
            "tilt": 30.0,
            "rot": 90.0,
            "roll": 0.0,
        },
        # Lateral Branch 2 (3 links, larger radius, higher attachment)
        {
            "id": "Branch_r1_o1",
            "parent": "trunk",
            "attach_link": 4,
            "n_links": 3,
            "height": 0.10,  # 10cm per link = 30cm total
            "radius": 0.05,  # 5cm
            "tilt": 45.0,
            "rot": 180.0,
            "roll": 0.0,
        },
        # Petiole on Branch 2
        {
            "id": "Petiole_r1_o1",
            "parent": "Branch_r1_o1",
            "attach_link": 2,
            "n_links": 2,
            "height": 0.07,  # 7cm per link = 14cm total
            "radius": 0.03,  # 3cm
            "tilt": 30.0,
            "rot": 90.0,
            "roll": 0.0,
        },
        # Lateral Branch 3 (2 links, smallest, middle)
        {
            "id": "Branch_r2_o0",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 2,
            "height": 0.10,  # 10cm per link = 20cm total
            "radius": 0.04,  # 4cm
            "tilt": 45.0,
            "rot": 90.0,
            "roll": 0.0,
        },
    ]
    
    return branches


def main():
    """Generate baseline and optimized USD files."""
    print("=" * 70)
    print("  Lateral Branch Reduction - USD Comparison Generator")
    print("=" * 70)
    
    # Create output directory
    output_dir = os.path.join(os.path.dirname(__file__), "usd_output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Create plant configuration
    print("\n[Step 1] Creating plant configuration...")
    branches = create_plant_config()
    
    # Count lateral branches
    lateral_count = sum(1 for b in branches if b["id"].startswith("Branch_r"))
    total_links_before = sum(b["n_links"] for b in branches)
    lateral_links_before = sum(b["n_links"] for b in branches if b["id"].startswith("Branch_r"))
    
    print(f"  ✓ Created plant with {len(branches)} branches")
    print(f"  ✓ Found {lateral_count} lateral branches ({lateral_links_before} links)")
    print(f"  ✓ Total links: {total_links_before}")
    
    # Generate baseline USD
    print("\n[Step 2] Generating baseline USD...")
    baseline_path = os.path.join(output_dir, "baseline.usda")
    try:
        stage_baseline, _ = build_stage(baseline_path, branches=branches, locked_joints=False)
        stage_baseline.Save()
        print(f"  ✓ Saved: {baseline_path}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Apply lateral branch reduction
    print("\n[Step 3] Applying lateral branch reduction...")
    technique = LateralBranchReductionTechnique(min_segments=1)
    
    if not technique.can_apply(branches):
        print("  ✗ Technique not applicable (no reducible lateral branches)")
        return 1
    
    reduction = technique.estimate_reduction(branches)
    print(f"  ✓ Estimated link reduction: {reduction}")
    
    # Apply reduction multiple times to reach minimum
    branches_reduced = branches
    total_applications = 0
    max_iterations = 10  # Safety limit
    
    while technique.can_apply(branches_reduced) and total_applications < max_iterations:
        branches_reduced, report = technique.apply(branches_reduced)
        total_applications += 1
        print(f"  ✓ Iteration {total_applications}: Reduced {report.details['branches_reduced']} branches, "
              f"removed {report.details['links_removed']} links")
    
    total_links_after = sum(b["n_links"] for b in branches_reduced)
    lateral_links_after = sum(b["n_links"] for b in branches_reduced if b["id"].startswith("Branch_r"))
    
    print(f"  ✓ Final: {lateral_count} lateral branches reduced to {lateral_links_after} links total")
    print(f"  ✓ Total links: {total_links_before} → {total_links_after} (saved {total_links_before - total_links_after})")
    
    # Generate lateral_reduce USD
    print("\n[Step 4] Generating lateral_reduce USD...")
    reduced_path = os.path.join(output_dir, "lateral_reduce.usda")
    try:
        stage_reduced, _ = build_stage(reduced_path, branches=branches_reduced, locked_joints=False)
        stage_reduced.Save()
        print(f"  ✓ Saved: {reduced_path}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Summary
    print("\n" + "=" * 70)
    print("  ✓ USD Generation Complete!")
    print("=" * 70)
    print(f"Generated files in: {output_dir}/")
    print(f"  1. baseline.usda         - {lateral_count} lateral branches with {lateral_links_before} links")
    print(f"  2. lateral_reduce.usda   - {lateral_count} lateral branches with {lateral_links_after} links")
    print(f"\nDifference: {lateral_links_before - lateral_links_after} lateral branch links removed")
    print(f"            {total_links_before - total_links_after} total links removed")
    
    # Next steps
    print("\n" + "=" * 70)
    print("  Next Steps:")
    print("=" * 70)
    print("1. Load in Isaac Sim:")
    print(f"   ~/isaacsim/python.sh -m isaacsim {baseline_path}")
    print(f"   ~/isaacsim/python.sh -m isaacsim {reduced_path}")
    print("")
    print("2. Run comparison script:")
    print(f"   ~/isaacsim/python.sh tests/5_lateral_reduce/compare_isaac_sim.py")
    print("")
    print("3. What to observe:")
    print("   - Baseline: Lateral branches have multiple articulated segments")
    print("   - Lateral Reduce: Lateral branches reduced to single rigid segments")
    print("   - Petioles: Attachment remapped to new segment positions")
    print("   - Visual appearance: Slightly less flexible but similar overall shape")
    print("   - Performance: Reduced should be more stable (fewer joints)")
    print("")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
