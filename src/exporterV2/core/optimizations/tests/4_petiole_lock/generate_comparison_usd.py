"""
generate_comparison_usd.py - Generate Baseline vs Petiole Lock USD for Isaac Sim

Generates two USD files:
1. baseline.usda - Plant with articulated petiolules (D6 joints)
2. petiole_lock.usda - Same plant with locked petiolules (Fixed joints)

Run with: uv run python src/exporterV2/core/optimizations/tests/4_petiole_lock/generate_comparison_usd.py
"""

import sys
import os

# Add project roots to path
script_dir = os.path.dirname(os.path.abspath(__file__))
exporterv2_dir = os.path.join(script_dir, "../../../../..")
sys.path.insert(0, exporterv2_dir)

from exporterV2.core.usd.stage import build_stage
from exporterV2.core.optimizations.techniques.petiole_lock import PetioleLockTechnique


def create_simple_plant_config():
    """
    Create a simple plant configuration with petiolules.
    
    Structure:
    - trunk (5 links)
    - lateral branch (3 links)
      - petiole (2 links)
        - rachis (1 link)
          - petiolule 1 (1 link)
          - petiolule 2 (1 link)
          - petiolule 3 (1 link)
    """
    branches = [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 5,
            "height": 0.20,  # 20cm per link
            "radius": 0.05,  # 5cm radius
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "Branch_r1_o0",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 3,
            "height": 0.15,
            "radius": 0.03,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "Petiole_r1_o0",
            "parent": "Branch_r1_o0",
            "attach_link": 2,
            "n_links": 2,
            "height": 0.10,
            "radius": 0.015,
            "tilt": 30.0,
            "rot": 90.0,
        },
        {
            "id": "Rachis_r1_o0_l0",
            "parent": "Petiole_r1_o0",
            "attach_link": 2,
            "n_links": 1,
            "height": 0.08,
            "radius": 0.010,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "Petiolule_r1_o0_l0_lf0",
            "parent": "Rachis_r1_o0_l0",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.05,
            "radius": 0.005,
            "tilt": 45.0,
            "rot": 0.0,
        },
        {
            "id": "Petiolule_r1_o0_l0_lf1",
            "parent": "Rachis_r1_o0_l0",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.05,
            "radius": 0.005,
            "tilt": 45.0,
            "rot": 120.0,
        },
        {
            "id": "Petiolule_r1_o0_l0_lf2",
            "parent": "Rachis_r1_o0_l0",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.05,
            "radius": 0.005,
            "tilt": 45.0,
            "rot": 240.0,
        },
    ]
    
    return branches


def main():
    """Generate baseline and petiole_lock USD files."""
    print("="*70)
    print("  Petiole Lock - USD Comparison Generator")
    print("="*70)
    
    output_dir = os.path.join(script_dir, "usd_output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Create plant configuration
    print("\n[Step 1] Creating plant configuration...")
    branches = create_simple_plant_config()
    print(f"  ✓ Created plant with {len(branches)} branches")
    
    # Count petiolules
    petiolule_count = sum(1 for b in branches if b["id"].startswith("Petiolule"))
    print(f"  ✓ Found {petiolule_count} petiolules")
    
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
    
    # Apply petiole lock technique
    print("\n[Step 3] Applying petiole lock...")
    technique = PetioleLockTechnique()
    
    if not technique.can_apply(branches):
        print("  ✗ Technique not applicable (no petiolules found)")
        return 1
    
    dof_reduction = technique.estimate_reduction(branches)
    print(f"  ✓ Estimated DOF reduction: {dof_reduction}")
    
    branches_locked, report = technique.apply(branches)
    print(f"  ✓ Locked {report.details['items_locked']} petiolules")
    print(f"  ✓ Reduced {report.details['dof_reduced']} DOF")
    
    # Generate petiole_lock USD
    print("\n[Step 4] Generating petiole_lock USD...")
    locked_path = os.path.join(output_dir, "petiole_lock.usda")
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
    print(f"  1. baseline.usda      - {petiolule_count} petiolules with D6 joints (articulated)")
    print(f"  2. petiole_lock.usda  - {petiolule_count} petiolules with Fixed joints (static)")
    print(f"\nDifference: {dof_reduction} DOF reduced (petiolules locked)")
    
    print("\n" + "="*70)
    print("  Next Steps:")
    print("="*70)
    print("\n1. Load in Isaac Sim:")
    print(f"   ~/isaacsim/python.sh -m isaacsim {baseline_path}")
    print(f"   ~/isaacsim/python.sh -m isaacsim {locked_path}")
    print("\n2. Run comparison script:")
    print("   ~/isaacsim/python.sh tests/4_petiole_lock/compare_isaac_sim.py")
    print("\n3. What to observe:")
    print("   - Baseline: Petiolules oscillate/move with physics")
    print("   - Petiole Lock: Petiolules are rigid (fixed to rachis)")
    print("   - Visual appearance: Identical")
    print("   - Performance: Petiole lock should be more stable")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
