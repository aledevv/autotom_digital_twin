"""
generate_combinations_usd.py - Generate USD files for technique combinations

Creates USD files for different optimization technique combinations, all compared
against a common baseline. This allows visual comparison in Isaac Sim to see the
effect of each technique or combination.

Combinations (8 total):
  0. Baseline          - No optimization
  1. P (Petiole Lock)  - Only petiolules → Fixed
  2. L (Lateral)       - Only lateral branch reduction
  3. S (Stem)          - Only stem collapse
  4. F (Leaf)          - Only leaf branch reduction (petiole+rachis merge)
  5. P+L               - Petiole lock + lateral reduce
  6. P+F               - Petiole lock + leaf reduce
  7. Full (P+L+S+F)    - All techniques applied

Usage:
    uv run python src/exporterV2/demos/optimization_visual_validation/generate_combinations_usd.py
"""

import sys
import os
from pathlib import Path
import copy

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
script_dir = Path(__file__).parent.resolve()
src_dir = script_dir.parents[2]
optimizations_dir = (script_dir / "../../core/optimizations").resolve()

sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(optimizations_dir))

from exporterV2.core.usd.stage import build_stage
from techniques.petiole_lock import PetioleLockTechnique
from techniques.lateral_reduce import LateralBranchReductionTechnique
from techniques.stem_collapse import StemCollapseTechnique
from techniques.leaf_branch_reduce import LeafBranchReductionTechnique
from techniques.base import count_d6_joints

OUTPUT_DIR = script_dir / "usd_output_combinations"
OUTPUT_DIR.mkdir(exist_ok=True)


# ---------------------------------------------------------------------------
# Common plant baseline (same as visual_validation)
# ---------------------------------------------------------------------------

def create_baseline_plant() -> list:
    """Synthetic mid-growth tomato plant. D6 joint count: 99."""
    branches = []
    
    # Trunk
    branches.append({
        "id": "trunk",
        "parent": None,
        "n_links": 10,
        "height": 0.10,
        "radius": 0.04,
        "tilt": 0.0,
        "rot": 0.0,
    })
    
    # 5 lateral branches
    for i in range(5):
        branches.append({
            "id": f"Branch_r{i+1}_o0",
            "parent": "trunk",
            "attach_link": (i + 1) * 2,
            "attach_frac": 1.0,
            "n_links": 5,
            "height": 0.08,
            "radius": 0.025,
            "tilt": 50.0,
            "rot": 72.0 * i,
        })
    
    # 8 compound leaves
    leaf_parents = [
        ("trunk", 1), ("trunk", 3), ("trunk", 5), ("trunk", 7),
        ("Branch_r1_o0", 2), ("Branch_r2_o0", 2),
        ("Branch_r3_o0", 3), ("Branch_r4_o0", 3),
    ]
    
    for idx, (parent_id, attach_link) in enumerate(leaf_parents):
        rank = idx + 1
        rot_base = 45.0 * idx
        petiole_id = f"Leaf_r{rank}_o0_petiole"
        rachis_id = f"Leaf_r{rank}_o0_rachis"
        
        branches.append({
            "id": petiole_id,
            "parent": parent_id,
            "attach_link": attach_link,
            "attach_frac": 1.0,
            "n_links": 2,
            "height": 0.05,
            "radius": 0.012,
            "tilt": 35.0,
            "rot": rot_base,
        })
        
        branches.append({
            "id": rachis_id,
            "parent": petiole_id,
            "attach_link": 2,
            "attach_frac": 1.0,
            "n_links": 3,
            "height": 0.04,
            "radius": 0.008,
            "tilt": 0.0,
            "rot": 0.0,
        })
        
        for pet in range(3):
            branches.append({
                "id": f"Petiolule_r{rank}_o0_lf{pet}",
                "parent": rachis_id,
                "attach_link": pet + 1,
                "attach_frac": 1.0,
                "n_links": 1,
                "height": 0.03,
                "radius": 0.005,
                "tilt": 60.0,
                "rot": rot_base + 120.0 * pet,
            })
    
    return branches


# ---------------------------------------------------------------------------
# Technique helpers
# ---------------------------------------------------------------------------

def apply_petiole_lock(branches: list) -> tuple:
    t = PetioleLockTechnique()
    return t.apply(branches) if t.can_apply(branches) else (branches, None)


def apply_lateral_reduce(branches: list) -> tuple:
    t = LateralBranchReductionTechnique(min_segments=1)
    result = branches
    while t.can_apply(result):
        result, _ = t.apply(result)
    return result, None


def apply_stem_collapse(branches: list) -> tuple:
    t = StemCollapseTechnique(target_segments=3)
    return t.apply(branches) if t.can_apply(branches) else (branches, None)


def apply_leaf_reduce(branches: list) -> tuple:
    t = LeafBranchReductionTechnique()
    return t.apply(branches) if t.can_apply(branches) else (branches, None)


# ---------------------------------------------------------------------------
# Combination definitions
# ---------------------------------------------------------------------------

COMBINATIONS = [
    {
        "id": 0,
        "label": "Baseline",
        "short": "baseline",
        "description": "No optimization",
        "techniques": [],
    },
    {
        "id": 1,
        "label": "Petiole Lock",
        "short": "P",
        "description": "Petiolules D6 → Fixed",
        "techniques": ["petiole_lock"],
    },
    {
        "id": 2,
        "label": "Lateral Reduce",
        "short": "L",
        "description": "Lateral branches → 1 segment",
        "techniques": ["lateral_reduce"],
    },
    {
        "id": 3,
        "label": "Stem Collapse",
        "short": "S",
        "description": "Trunk 10 → 3 segments",
        "techniques": ["stem_collapse"],
    },
    {
        "id": 4,
        "label": "Leaf Reduce",
        "short": "F",
        "description": "Petiole+rachis → single segment",
        "techniques": ["leaf_reduce"],
    },
    {
        "id": 5,
        "label": "P+L",
        "short": "P+L",
        "description": "Petiole lock + lateral reduce",
        "techniques": ["petiole_lock", "lateral_reduce"],
    },
    {
        "id": 6,
        "label": "P+F",
        "short": "P+F",
        "description": "Petiole lock + leaf reduce",
        "techniques": ["petiole_lock", "leaf_reduce"],
    },
    {
        "id": 7,
        "label": "Full Optimization",
        "short": "Full",
        "description": "All techniques (P+L+S+F)",
        "techniques": ["petiole_lock", "lateral_reduce", "stem_collapse", "leaf_reduce"],
    },
]

TECHNIQUE_FUNCS = {
    "petiole_lock": apply_petiole_lock,
    "lateral_reduce": apply_lateral_reduce,
    "stem_collapse": apply_stem_collapse,
    "leaf_reduce": apply_leaf_reduce,
}


# ---------------------------------------------------------------------------
# USD generation
# ---------------------------------------------------------------------------

def save_usd(filename: str, branches: list) -> bool:
    """Build and save a USD stage. Returns True on success."""
    path = str(OUTPUT_DIR / filename)
    try:
        stage, _ = build_stage(path, branches=branches, locked_joints=False)
        stage.Save()
        return True
    except Exception as exc:
        print(f"  ✗ FAILED {filename}: {exc}")
        import traceback
        traceback.print_exc()
        return False


def apply_combination(baseline: list, techniques: list) -> list:
    """Apply a list of techniques to baseline. Returns modified branches."""
    result = copy.deepcopy(baseline)
    for tech_name in techniques:
        result, _ = TECHNIQUE_FUNCS[tech_name](result)
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 70)
    print("  Optimization Technique Combinations — USD Generator")
    print("=" * 70)
    print(f"\nOutput directory: {OUTPUT_DIR}\n")
    
    baseline = create_baseline_plant()
    baseline_joints = count_d6_joints(baseline)
    
    print(f"Baseline plant: {len(baseline)} branches, {baseline_joints} D6 joints\n")
    
    results = []
    
    for combo in COMBINATIONS:
        cid = combo["id"]
        label = combo["label"]
        short = combo["short"]
        techs = combo["techniques"]
        
        print(f"─" * 70)
        print(f"[Combo {cid}] {label} ({short})")
        print(f"  Description: {combo['description']}")
        print(f"  Techniques:  {', '.join(techs) if techs else '(none)'}")
        
        # Apply techniques
        modified = apply_combination(baseline, techs)
        joints = count_d6_joints(modified)
        delta = joints - baseline_joints
        
        # Save USD
        filename = f"combo_{cid}_{short.lower().replace('+', '_')}.usda"
        ok = save_usd(filename, modified)
        
        # Track results
        results.append({
            "id": cid,
            "label": label,
            "short": short,
            "filename": filename,
            "branches": len(modified),
            "joints": joints,
            "delta": delta,
            "ok": ok,
        })
        
        print(f"  Branches:    {len(modified)}")
        print(f"  D6 Joints:   {joints}  (Δ {delta:+d})")
        print(f"  {'✓' if ok else '✗'} Saved {filename}")
    
    # Summary table
    print()
    print("=" * 70)
    print("  SUMMARY — Technique Combinations")
    print("=" * 70)
    print(f"{'ID':<4} {'Label':<20} {'File':<30} {'Joints':>8} {'Δ':>8}")
    print("-" * 70)
    for r in results:
        print(f"{r['id']:<4} {r['label']:<20} {r['filename']:<30} {r['joints']:>8} {r['delta']:>8}")
    print("-" * 70)
    print(f"{'':26} {'Baseline D6 joints:':<30} {baseline_joints:>8}")
    print("=" * 70)
    
    print()
    print(f"✓ Generated {len(results)} USD files in {OUTPUT_DIR}")
    print()
    print("Next steps:")
    print(f"  1. Run tests:  uv run pytest {script_dir / 'validate_combinations.py'}")
    print(f"  2. Load in Isaac Sim:  See COMBINATIONS_README.md for commands")
    print()
    
    return 0 if all(r["ok"] for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
