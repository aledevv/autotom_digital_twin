"""
generate_comparison_usd.py - Generate Progressive Optimization USD for Isaac Sim

Generates three USD files showing progressive leaf optimization:
1. baseline.usda - Fully articulated leaf (petiole + rachis + petiolules)
2. partial.usda - Rachis reduced to 1 link (intermediate)
3. leaf_merged.usda - Petiole+rachis merged into single branch

Structure: Trunk + 1 lateral branch + 1 leaf

Run with: uv run python src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce/generate_comparison_usd.py
"""

import sys
import os

# Add project roots to path
script_dir = os.path.dirname(os.path.abspath(__file__))
exporterv2_dir = os.path.join(script_dir, "../../../../..")
sys.path.insert(0, exporterv2_dir)

from exporterV2.core.usd.stage import build_stage


def create_baseline_plant():
    """
    Create baseline plant: fully articulated leaf.
    
    Structure:
    - Trunk (5 links × 20cm = 1.0m)
    - Lateral Branch (1 link × 30cm, at trunk link 3)
      - Petiole (1 link × 10cm)
        - Rachis (3 links × 5cm = 15cm total)
          - Petiolule 1 (at rachis link 1)
          - Petiolule 2 (at rachis link 2)
          - Petiolule 3 (at rachis link 3)
    
    Total: 5 + 1 + 1 + 3 + 3 = 13 links
    """
    branches = [
        # Trunk
        {
            "id": "trunk",
            "parent": None,
            "n_links": 5,
            "height": 0.20,  # 20cm per link
            "radius": 0.03,  # 3cm radius
            "tilt": 0.0,
            "rot": 0.0,
        },
        # Lateral branch
        {
            "id": "Branch_r3_o0",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 1,
            "height": 0.30,  # 30cm
            "radius": 0.020,  # 2cm
            "tilt": 45.0,
            "rot": 90.0,
        },
        # Leaf - Petiole
        {
            "id": "Leaf_r3_o0_petiole",
            "parent": "Branch_r3_o0",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.10,  # 10cm
            "radius": 0.015,  # 1.5cm
            "tilt": 30.0,
            "rot": 0.0,
        },
        # Leaf - Rachis (3 links for distributed petiolules)
        {
            "id": "Leaf_r3_o0_rachis",
            "parent": "Leaf_r3_o0_petiole",
            "attach_link": 1,
            "n_links": 3,
            "height": 0.05,  # 5cm per link, 15cm total
            "radius": 0.010,  # 1.0cm
            "tilt": 0.0,
            "rot": 0.0,
        },
        # Petiolules (attached to rachis at different links)
        {
            "id": "Petiolule_r3_o0_lf0",
            "parent": "Leaf_r3_o0_rachis",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.04,  # 4cm
            "radius": 0.005,  # 0.5cm
            "tilt": 60.0,
            "rot": 0.0,
        },
        {
            "id": "Petiolule_r3_o0_lf1",
            "parent": "Leaf_r3_o0_rachis",
            "attach_link": 2,
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 120.0,
        },
        {
            "id": "Petiolule_r3_o0_lf2",
            "parent": "Leaf_r3_o0_rachis",
            "attach_link": 3,
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 240.0,
        },
    ]
    
    return branches


def create_partial_plant():
    """
    Create partial optimization: rachis reduced to 1 link.
    
    Same as baseline but rachis is single link (petiolules all at same point).
    Shows intermediate optimization step.
    
    Total: 5 + 1 + 1 + 1 + 3 = 11 links (saved 2 links)
    """
    branches = [
        # Trunk
        {
            "id": "trunk",
            "parent": None,
            "n_links": 5,
            "height": 0.20,
            "radius": 0.03,
            "tilt": 0.0,
            "rot": 0.0,
        },
        # Lateral branch
        {
            "id": "Branch_r3_o0",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 1,
            "height": 0.30,
            "radius": 0.020,
            "tilt": 45.0,
            "rot": 90.0,
        },
        # Leaf - Petiole (unchanged)
        {
            "id": "Leaf_r3_o0_petiole",
            "parent": "Branch_r3_o0",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.10,
            "radius": 0.015,
            "tilt": 30.0,
            "rot": 0.0,
        },
        # Leaf - Rachis (reduced to 1 link)
        {
            "id": "Leaf_r3_o0_rachis",
            "parent": "Leaf_r3_o0_petiole",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.15,  # Same total length: 3×5cm = 15cm
            "radius": 0.010,
            "tilt": 0.0,
            "rot": 0.0,
        },
        # Petiolules (remapped to the single rachis link using attach_frac)
        {
            "id": "Petiolule_r3_o0_lf0",
            "parent": "Leaf_r3_o0_rachis",
            "attach_link": 1,
            "attach_frac": 1/3,  # Baseline was at link 1 of 3 (H = 0.333)
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 0.0,
        },
        {
            "id": "Petiolule_r3_o0_lf1",
            "parent": "Leaf_r3_o0_rachis",
            "attach_link": 1,
            "attach_frac": 2/3,  # Baseline was at link 2 of 3 (H = 0.667)
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 120.0,
        },
        {
            "id": "Petiolule_r3_o0_lf2",
            "parent": "Leaf_r3_o0_rachis",
            "attach_link": 1,
            "attach_frac": 1.0,  # Baseline was at link 3 of 3 (H = 1.0)
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 240.0,
        },
    ]
    
    return branches


def create_merged_plant():
    """
    Create fully merged plant: petiole+rachis merged.
    
    Manually constructed to demonstrate the geometry remapping fix.
    Total: 5 + 1 + 1 + 3 = 10 links (saved 3 links vs baseline)
    """
    branches = [
        # Trunk (unchanged)
        {
            "id": "trunk",
            "parent": None,
            "n_links": 5,
            "height": 0.20,
            "radius": 0.03,
            "tilt": 0.0,
            "rot": 0.0,
        },
        # Lateral Branch (unchanged)
        {
            "id": "Branch_r3_o0",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 1,
            "height": 0.30,
            "radius": 0.020,
            "tilt": 45.0,
            "rot": 90.0,
        },
        # Merged Petiole + Rachis (1 link, total length = 0.10 + 0.15 = 0.25)
        {
            "id": "Leaf_r3_o0_merged",
            "parent": "Branch_r3_o0",
            "attach_link": 1,
            "n_links": 1,
            "height": 0.25,
            "radius": 0.010, # average radius
            "tilt": 30.0, # Inherit petiole's original tilt
            "rot": 0.0,
        },
        # Petiolules remapped to the single merged link
        # Distances from base of leaf: 
        # lf0 = petiole(0.10) + rachis_link1(0.05) = 0.15m (0.15/0.25 = 0.6)
        # lf1 = petiole(0.10) + rachis_link2(0.10) = 0.20m (0.20/0.25 = 0.8)
        # lf2 = petiole(0.10) + rachis_link3(0.15) = 0.25m (0.25/0.25 = 1.0)
        {
            "id": "Petiolule_r3_o0_lf0",
            "parent": "Leaf_r3_o0_merged",
            "attach_link": 1,
            "attach_frac": 0.6,
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 0.0,
        },
        {
            "id": "Petiolule_r3_o0_lf1",
            "parent": "Leaf_r3_o0_merged",
            "attach_link": 1,
            "attach_frac": 0.8,
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 120.0,
        },
        {
            "id": "Petiolule_r3_o0_lf2",
            "parent": "Leaf_r3_o0_merged",
            "attach_link": 1,
            "attach_frac": 1.0,
            "n_links": 1,
            "height": 0.04,
            "radius": 0.005,
            "tilt": 60.0,
            "rot": 240.0,
        },
    ]
    return branches


def main():
    """Generate three USD files with progressive optimization."""
    print("="*70)
    print("  Leaf Branch Reduction - Progressive Optimization USD Generator")
    print("="*70)
    
    output_dir = os.path.join(script_dir, "usd_output")
    os.makedirs(output_dir, exist_ok=True)
    
    # Generate baseline
    print("\n[Step 1] Generating baseline USD (fully articulated)...")
    baseline_branches = create_baseline_plant()
    baseline_links = sum(b["n_links"] for b in baseline_branches)
    print(f"  ✓ Created baseline plant: {len(baseline_branches)} branches, {baseline_links} links")
    print(f"    - Trunk: 5 links")
    print(f"    - Lateral branch: 1 link")
    print(f"    - Petiole: 1 link")
    print(f"    - Rachis: 3 links (articulated)")
    print(f"    - Petiolules: 3 × 1 link")
    
    baseline_path = os.path.join(output_dir, "baseline.usda")
    try:
        stage_baseline, _ = build_stage(baseline_path, branches=baseline_branches, locked_joints=False)
        stage_baseline.Save()
        print(f"  ✓ Saved: {baseline_path}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Generate partial optimization
    print("\n[Step 2] Generating partial USD (rachis reduced)...")
    partial_branches = create_partial_plant()
    partial_links = sum(b["n_links"] for b in partial_branches)
    print(f"  ✓ Created partial plant: {len(partial_branches)} branches, {partial_links} links")
    print(f"    - Rachis: 1 link (reduced from 3)")
    print(f"    - Savings: {baseline_links - partial_links} links")
    
    partial_path = os.path.join(output_dir, "partial.usda")
    try:
        stage_partial, _ = build_stage(partial_path, branches=partial_branches, locked_joints=False)
        stage_partial.Save()
        print(f"  ✓ Saved: {partial_path}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Generate fully merged
    print("\n[Step 3] Generating merged USD (petiole+rachis merged)...")
    merged_branches = create_merged_plant()
    merged_links = sum(b["n_links"] for b in merged_branches)
    print(f"  ✓ Created merged plant: {len(merged_branches)} branches, {merged_links} links")
    print(f"    - Petiole: 1 link (merged petiole+rachis)")
    print(f"    - Rachis: removed")
    print(f"    - Savings: {baseline_links - merged_links} links vs baseline")
    
    merged_path = os.path.join(output_dir, "leaf_merged.usda")
    try:
        stage_merged, _ = build_stage(merged_path, branches=merged_branches, locked_joints=False)
        stage_merged.Save()
        print(f"  ✓ Saved: {merged_path}")
    except Exception as e:
        print(f"  ✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Summary
    print("\n" + "="*70)
    print("  ✓ USD Generation Complete!")
    print("="*70)
    print(f"\nGenerated 3 files in: {output_dir}/")
    print(f"\n1. baseline.usda     - {baseline_links} links (fully articulated)")
    print(f"   - Petiole: 1 link")
    print(f"   - Rachis: 3 links (distributed petiolules)")
    print(f"\n2. partial.usda      - {partial_links} links (rachis reduced)")
    print(f"   - Petiole: 1 link")
    print(f"   - Rachis: 1 link (petiolules clustered)")
    print(f"   - Savings: {baseline_links - partial_links} links")
    print(f"\n3. leaf_merged.usda  - {merged_links} links (fully merged)")
    print(f"   - Merged: 1 link (petiole+rachis combined)")
    print(f"   - Rachis: removed")
    print(f"   - Savings: {baseline_links - merged_links} links")
    
    print("\n" + "="*70)
    print("  Next Steps:")
    print("="*70)
    print("\n1. View individual USD files:")
    print(f"   ~/isaacsim/python.sh -m isaacsim {baseline_path}")
    print(f"   ~/isaacsim/python.sh -m isaacsim {partial_path}")
    print(f"   ~/isaacsim/python.sh -m isaacsim {merged_path}")
    print("\n2. Run comparison script (side-by-side):")
    print("   cd src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce")
    print("   ~/isaacsim/python.sh compare_isaac_sim.py")
    print("\n3. What to observe:")
    print("   - Baseline: Leaf highly articulated (petiole + 3 rachis links)")
    print("   - Partial: Rachis reduced but still separate from petiole")
    print("   - Merged: Leaf rigid (single merged segment)")
    print("   - Length preserved: All leaves same total length (25cm)")
    print("   - Visual impact: Progressive loss of flexibility")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
