"""
generate_comparison_usd.py - Generate Baseline vs Thin Link Lock USD for Isaac Sim

Generates two USD files:
1. baseline.usda - Plant with all articulated joints
2. thin_link_lock.usda - Same plant with thin links locked (Fixed joints)

Run with: uv run python src/exporterV2/core/optimizations/tests/10_thin_link_lock/generate_comparison_usd.py
"""

import sys
import os

# Add project roots to path
script_dir = os.path.dirname(os.path.abspath(__file__))
exporterv2_dir = os.path.join(script_dir, "../../../../..")
sys.path.insert(0, exporterv2_dir)

from exporterV2.core.usd.stage import build_stage
from exporterV2.core.optimizations.techniques.thin_link_lock import ThinLinkLockTechnique


def create_complex_plant_config():
    """
    Create a complex plant configuration with some thin links.
    
    Structure:
    - trunk (5 links, r=0.05)
      - branch1 (3 links, r=0.02)
        - subbranch1 (2 links, r=0.01)
          - leaf_stem1 (1 link, r=0.001) -> THIN LINK!
          - leaf_stem2 (1 link, r=0.001) -> THIN LINK!
      - branch2 (4 links, r=0.015)
        - subbranch2 (2 links, r=0.005)
          - terminal_shoot (1 link, r=0.0005) -> THIN LINK!
    """
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 5,
            "height": 0.20,
            "radius": 0.05,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "Branch_1",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 3,
            "height": 0.15,
            "radius": 0.02,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "Subbranch_1",
            "parent": "Branch_1",
            "attach_link": 2,
            "n_links": 2,
            "height": 0.10,
            "radius": 0.01,
            "tilt": 30.0,
            "rot": 90.0,
        },
        # These two are thin links (pre-scale radius <= 0.001m)
        {
            "id": "LeafStem_1_1",
            "parent": "Subbranch_1",
            "attach_link": 2,
            "n_links": 1,
            "height": 0.05,
            "radius": 0.001,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "LeafStem_1_2",
            "parent": "Subbranch_1",
            "attach_link": 2,
            "n_links": 1,
            "height": 0.05,
            "radius": 0.001,
            "tilt": 45.0,
            "rot": 180.0,
        },
        {
            "id": "Branch_2",
            "parent": "trunk",
            "attach_link": 4,
            "n_links": 4,
            "height": 0.15,
            "radius": 0.015,
            "tilt": 40.0,
            "rot": 180.0,
        },
        {
            "id": "Subbranch_2",
            "parent": "Branch_2",
            "attach_link": 3,
            "n_links": 2,
            "height": 0.10,
            "radius": 0.005,
            "tilt": 30.0,
            "rot": -90.0,
        },
        # This one is a thin link (pre-scale radius < 0.001m)
        {
            "id": "TerminalShoot_2",
            "parent": "Subbranch_2",
            "attach_link": 2,
            "n_links": 1,
            "height": 0.03,
            "radius": 0.0005,
            "tilt": 0.0,
            "rot": 0.0,
        },
        # ADDING HIGHLY UNSTABLE MINI BRANCHES
        {
            "id": "Fragile_Stem_1",
            "parent": "TerminalShoot_2",
            "attach_link": 1,
            "n_links": 3,
            "height": 0.02,
            "radius": 0.0001, # extremely thin
            "tilt": 45.0,
            "rot": 45.0,
        },
        {
            "id": "Fragile_Stem_2",
            "parent": "TerminalShoot_2",
            "attach_link": 1,
            "n_links": 4, # longer chain, more joints
            "height": 0.015,
            "radius": 0.0002, # extremely thin
            "tilt": 45.0,
            "rot": -135.0,
        },
        {
            "id": "Ultra_Mini_Branch",
            "parent": "Branch_1",
            "attach_link": 3,
            "n_links": 5, # 5 articulated thin joints!
            "height": 0.01,
            "radius": 0.0005,
            "tilt": 90.0,
            "rot": 45.0,
        },
    ]
    
    return branches


def main():
    """Generate baseline and thin_link_lock USD files."""
    print("="*70)
    print("  Thin Link Lock - USD Comparison Generator")
    print("="*70)
    
    output_dir = os.path.join(script_dir, "usd_output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Create plant configuration
    print("\n[Step 1] Creating complex plant configuration...")
    branches = create_complex_plant_config()
    print(f"  ✓ Created plant with {len(branches)} branches")
    
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
    
    # Apply thin link lock technique
    print("\n[Step 3] Applying thin link lock...")
    technique = ThinLinkLockTechnique()
    
    if not technique.can_apply(branches):
        print("  ✗ Technique not applicable (no thin links found)")
        return 1
    
    # count thin links for report
    thin_links_count = sum(1 for b in branches if technique._is_target(b))
    
    reduction = technique.estimate_reduction(branches)
    print(f"  ✓ Estimated joints reduction: {reduction}")
    
    branches_locked, report = technique.apply(branches)
    print(f"  ✓ Locked {report.details['items_locked']} thin links")
    print(f"  ✓ Reduced {report.joints_saved} joints ({report.details['dof_reduced']} DOF)")
    
    # Generate thin_link_lock USD
    print("\n[Step 4] Generating thin_link_lock USD...")
    locked_path = os.path.join(output_dir, "thin_link_lock.usda")
    try:
        stage_locked, _ = build_stage(locked_path, branches=branches_locked, locked_joints=False)
        stage_locked.Save()
        print(f"  ✓ Saved: {locked_path}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Summary
    print("\n" + "="*70)
    print("  ✓ USD Generation Complete!")
    print("="*70)
    print(f"\nGenerated files in: {output_dir}/")
    print(f"  1. baseline.usda       - {thin_links_count} thin links with D6 joints (articulated)")
    print(f"  2. thin_link_lock.usda - {thin_links_count} thin links with Fixed joints (static)")
    print(f"\nDifference: {report.joints_saved} joints reduced (thin links locked)")
    
    print("\n" + "="*70)
    print("  Next Steps:")
    print("="*70)
    print("\n1. Load in Isaac Sim:")
    print(f"   ~/isaacsim/python.sh -m isaacsim {baseline_path}")
    print(f"   ~/isaacsim/python.sh -m isaacsim {locked_path}")
    print("\n2. Run comparison script:")
    print("   ~/isaacsim/python.sh tests/10_thin_link_lock/compare_isaac_sim.py")
    print("\n3. What to observe:")
    print("   - Baseline: Thin links might wobble or explode under physics forces due to small mass/inertia.")
    print("   - Thin Link Lock: Thin links are rigid, remaining stable.")
    print("   - Visual appearance: Identical")
    print("   - Performance: Thin link lock should be more stable")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
