"""
run_visual_test.py - Visual Validation Suite for Joint-Budget Optimization

Generates 6 USD files showing the plant at each optimization stage, so you can
load them in Isaac Sim and visually verify each technique works correctly.

Stages:
  0_baseline.usda          - Original plant, no optimization
  1_petiole_lock.usda      - After locking petiolules (D6 → Fixed)
  2_lateral_reduce.usda    - After reducing lateral branch segments
  3_stem_collapse.usda     - After collapsing trunk segments
  4_leaf_branch_reduce.usda - After merging petiole+rachis
  5_fully_optimized.usda   - All techniques applied (same as stage 4 if budget met)

After running, open each USD in Isaac Sim and follow README.md for the checklist.

Usage:
    uv run python src/exporterV2/core/optimizations/tests/visual_validation/run_visual_test.py
"""

import sys
import os
from pathlib import Path

# ---------------------------------------------------------------------------
# Path setup
# visual_validation/ is at: src/exporterV2/core/optimizations/tests/visual_validation/
# "../../../../../.." from here → project root (autotom_digital_twin/)
# "../../../../../../src" → src/   (where exporterV2 lives)
# "../../.."             → optimizations/
# ---------------------------------------------------------------------------
script_dir = Path(__file__).parent.resolve()
src_dir = (script_dir / "../../../../../../src").resolve()
optimizations_dir = (script_dir / "../..").resolve()  # visual_validation/../.. → optimizations/

sys.path.insert(0, str(src_dir))
sys.path.insert(0, str(optimizations_dir))

from exporterV2.core.usd.stage import build_stage
from techniques.petiole_lock import PetioleLockTechnique
from techniques.lateral_reduce import LateralBranchReductionTechnique
from techniques.stem_collapse import StemCollapseTechnique
from techniques.leaf_branch_reduce import LeafBranchReductionTechnique
from techniques.base import count_d6_joints

OUTPUT_DIR = script_dir / "usd_output"
OUTPUT_DIR.mkdir(exist_ok=True)


# ---------------------------------------------------------------------------
# Synthetic plant: realistic mid-growth tomato (enough to be over budget)
# ---------------------------------------------------------------------------

def create_plant() -> list:
    """
    Synthetic mid-growth tomato plant.

    Structure:
      - Trunk: 10 links × 10cm = 1.0m
      - 5 lateral branches: 5 links × 8cm each
      - 8 compound leaves (each on trunk or lateral):
          petiole  2 links × 5cm
          rachis   3 links × 4cm
          3 petiolules × 1 link × 3cm

    D6 joint count:
      trunk            10
      laterals   5×5 = 25
      petioles   8×2 = 16
      rachis     8×3 = 24
      petiolules 8×3 =  24
                       ────
                        99 D6 joints
    """
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

    # 5 lateral branches distributed along trunk
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
        ("trunk", 1),
        ("trunk", 3),
        ("trunk", 5),
        ("trunk", 7),
        ("Branch_r1_o0", 2),
        ("Branch_r2_o0", 2),
        ("Branch_r3_o0", 3),
        ("Branch_r4_o0", 3),
    ]

    for idx, (parent_id, attach_link) in enumerate(leaf_parents):
        rank = idx + 1
        rot_base = 45.0 * idx

        petiole_id = f"Leaf_r{rank}_o0_petiole"
        rachis_id  = f"Leaf_r{rank}_o0_rachis"

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
# USD generation helper
# ---------------------------------------------------------------------------

def save_usd(filename: str, branches: list, stage_label: str) -> bool:
    """Build and save a USD stage. Returns True on success."""
    path = str(OUTPUT_DIR / filename)
    try:
        stage, _ = build_stage(path, branches=branches, locked_joints=False)
        stage.Save()
        print(f"  ✓ Saved {filename}  ({len(branches)} branches)")
        return True
    except Exception as exc:
        print(f"  ✗ FAILED {filename}: {exc}")
        import traceback
        traceback.print_exc()
        return False


# ---------------------------------------------------------------------------
# Diff helper
# ---------------------------------------------------------------------------

def branch_diff(before: list, after: list) -> dict:
    """Return a summary of structural changes between two branch lists."""
    before_ids = {b["id"]: b for b in before}
    after_ids  = {b["id"]: b for b in after}

    added   = [i for i in after_ids  if i not in before_ids]
    removed = [i for i in before_ids if i not in after_ids]
    changed = []
    for bid in before_ids:
        if bid not in after_ids:
            continue
        b0, b1 = before_ids[bid], after_ids[bid]
        diffs = {}
        for key in ("n_links", "height", "radius", "attach_link",
                    "attach_frac", "joint_type"):
            v0 = b0.get(key)
            v1 = b1.get(key)
            if v0 != v1:
                diffs[key] = (v0, v1)
        if diffs:
            changed.append((bid, diffs))

    return {"added": added, "removed": removed, "changed": changed}


def print_diff(diff: dict):
    """Print a human-readable diff."""
    if diff["removed"]:
        print(f"    Removed branches ({len(diff['removed'])}):")
        for bid in diff["removed"][:6]:
            print(f"      - {bid}")
        if len(diff["removed"]) > 6:
            print(f"      ... and {len(diff['removed']) - 6} more")

    if diff["added"]:
        print(f"    Added branches ({len(diff['added'])}):")
        for bid in diff["added"][:6]:
            print(f"      + {bid}")

    if diff["changed"]:
        print(f"    Modified branches ({len(diff['changed'])}):")
        for bid, diffs in diff["changed"][:8]:
            parts = []
            for key, (v0, v1) in diffs.items():
                if isinstance(v0, float) or isinstance(v1, float):
                    parts.append(f"{key}: {v0:.3f}→{v1:.3f}" if v0 is not None and v1 is not None
                                 else f"{key}: {v0}→{v1}")
                else:
                    parts.append(f"{key}: {v0}→{v1}")
            print(f"      ~ {bid}: {', '.join(parts)}")
        if len(diff["changed"]) > 8:
            print(f"      ... and {len(diff['changed']) - 8} more")

    if not any(diff.values()):
        print("    (no structural changes)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("=" * 70)
    print("  Joint-Budget Optimization — Visual Validation Suite")
    print("=" * 70)
    print(f"\nOutput directory: {OUTPUT_DIR}\n")

    # Track all stages for summary table
    stages = []  # list of (label, filename, joints, branches)
    ok = True

    # ------------------------------------------------------------------
    # Stage 0: Baseline
    # ------------------------------------------------------------------
    print("─" * 70)
    print("[Stage 0] Baseline — no optimization")
    print("─" * 70)
    plant = create_plant()
    joints0 = count_d6_joints(plant)
    ok &= save_usd("0_baseline.usda", plant, "Baseline")
    stages.append(("0 Baseline", "0_baseline.usda", joints0, plant))
    prev = plant

    # ------------------------------------------------------------------
    # Stage 1: Petiole Lock
    # ------------------------------------------------------------------
    print()
    print("─" * 70)
    print("[Stage 1] Petiole Lock — D6 petiolules → Fixed joints")
    print("─" * 70)
    t1 = PetioleLockTechnique()
    if t1.can_apply(prev):
        s1, rep1 = t1.apply(prev)
        j1 = count_d6_joints(s1)
        print(f"  Petiolules locked : {rep1.details['petiolules_locked']}")
        print(f"  DOF reduced       : {rep1.details['dof_reduced']}")
        print(f"  D6 joints         : {joints0} → {j1}  (Δ {j1 - joints0:+d})")
        d1 = branch_diff(prev, s1)
        print_diff(d1)
        ok &= save_usd("1_petiole_lock.usda", s1, "Petiole Lock")
        stages.append(("1 Petiole Lock", "1_petiole_lock.usda", j1, s1))
        prev = s1
    else:
        print("  (technique not applicable — no petiolules found)")
        stages.append(("1 Petiole Lock", "—", joints0, prev))

    # ------------------------------------------------------------------
    # Stage 2: Lateral Reduce
    # ------------------------------------------------------------------
    print()
    print("─" * 70)
    print("[Stage 2] Lateral Reduce — fewer segments per lateral branch")
    print("─" * 70)
    t2 = LateralBranchReductionTechnique(min_segments=1)
    if t2.can_apply(prev):
        s2, rep2 = t2.apply(prev)
        j2 = count_d6_joints(s2)
        print(f"  Links removed     : {rep2.joints_saved}")
        print(f"  D6 joints         : {count_d6_joints(prev)} → {j2}  (Δ {j2 - count_d6_joints(prev):+d})")
        d2 = branch_diff(prev, s2)
        print_diff(d2)
        ok &= save_usd("2_lateral_reduce.usda", s2, "Lateral Reduce")
        stages.append(("2 Lateral Reduce", "2_lateral_reduce.usda", j2, s2))
        prev = s2
    else:
        print("  (technique not applicable)")
        stages.append(("2 Lateral Reduce", "—", count_d6_joints(prev), prev))

    # ------------------------------------------------------------------
    # Stage 3: Stem Collapse
    # ------------------------------------------------------------------
    print()
    print("─" * 70)
    print("[Stage 3] Stem Collapse — trunk 10 links → 3 links")
    print("─" * 70)
    t3 = StemCollapseTechnique(target_segments=3)
    if t3.can_apply(prev):
        s3, rep3 = t3.apply(prev)
        j3 = count_d6_joints(s3)
        print(f"  Trunk links       : {rep3.details['original_links']} → {rep3.details['final_links']}")
        print(f"  Children remapped : {rep3.details['children_remapped']}")
        print(f"  Links removed     : {rep3.joints_saved}")
        print(f"  D6 joints         : {count_d6_joints(prev)} → {j3}  (Δ {j3 - count_d6_joints(prev):+d})")
        d3 = branch_diff(prev, s3)
        print_diff(d3)
        ok &= save_usd("3_stem_collapse.usda", s3, "Stem Collapse")
        stages.append(("3 Stem Collapse", "3_stem_collapse.usda", j3, s3))
        prev = s3
    else:
        print("  (technique not applicable — trunk already at target)")
        stages.append(("3 Stem Collapse", "—", count_d6_joints(prev), prev))

    # ------------------------------------------------------------------
    # Stage 4: Leaf Branch Reduce
    # ------------------------------------------------------------------
    print()
    print("─" * 70)
    print("[Stage 4] Leaf Branch Reduce — petiole+rachis → single segment")
    print("─" * 70)
    t4 = LeafBranchReductionTechnique()
    if t4.can_apply(prev):
        s4, rep4 = t4.apply(prev)
        j4 = count_d6_joints(s4)
        print(f"  Pairs merged      : {rep4.details['pairs_merged']}")
        print(f"  Links removed     : {rep4.details['links_removed']}")
        print(f"  Petiolules remapped: {rep4.details['petiolules_remapped']}")
        print(f"  D6 joints         : {count_d6_joints(prev)} → {j4}  (Δ {j4 - count_d6_joints(prev):+d})")
        d4 = branch_diff(prev, s4)
        print_diff(d4)
        ok &= save_usd("4_leaf_branch_reduce.usda", s4, "Leaf Branch Reduce")
        stages.append(("4 Leaf Reduce", "4_leaf_branch_reduce.usda", j4, s4))
        prev = s4
    else:
        print("  (technique not applicable — no petiole+rachis pairs)")
        stages.append(("4 Leaf Reduce", "—", count_d6_joints(prev), prev))

    # ------------------------------------------------------------------
    # Stage 5: Fully optimized (optimizer pipeline con budget ridotto)
    # ------------------------------------------------------------------
    print()
    print("─" * 70)
    print("[Stage 5] Fully Optimized — BudgetOptimizer pipeline (budget=50)")
    print("─" * 70)
    from optimizer import BudgetOptimizer
    import tempfile, yaml

    # Load config and force a tight budget so all techniques fire
    config_path = optimizations_dir / "budget_config.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)
    config["budget"]["max_joints"] = 50  # Force full optimization

    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as tf:
        yaml.dump(config, tf)
        temp_config = tf.name

    try:
        optimizer = BudgetOptimizer(config_path=temp_config)
        plant_fresh = create_plant()
        j_init = count_d6_joints(plant_fresh)
        print(f"  Budget            : {optimizer.config.max_joints}")
        print(f"  Initial D6 joints : {j_init}")
        try:
            s5, rep5 = optimizer.optimize(plant_fresh)
            j5 = count_d6_joints(s5)
            print(f"  Final D6 joints   : {j5}")
            print(f"  Success           : {rep5.success}")
            print(f"  Techniques used   : {[r.technique_name for r in rep5.technique_reports]}")
            ok &= save_usd("5_fully_optimized.usda", s5, "Fully Optimized")
            stages.append(("5 Fully Optimized", "5_fully_optimized.usda", j5, s5))
        except ValueError as exc:
            print(f"  Budget impossible : {exc}")
            ok &= save_usd("5_fully_optimized.usda", prev, "Best Effort")
            stages.append(("5 Best Effort", "5_fully_optimized.usda", count_d6_joints(prev), prev))
    finally:
        import os as _os
        _os.unlink(temp_config)

    # ------------------------------------------------------------------
    # Summary table
    # ------------------------------------------------------------------
    print()
    print("=" * 70)
    print("  SUMMARY — D6 Joint Count per Stage")
    print("=" * 70)
    header = f"{'Stage':<25} {'File':<30} {'D6 Joints':>10} {'Δ':>8}"
    print(header)
    print("-" * 70)
    prev_j = None
    for label, filename, joints, _ in stages:
        delta = ""
        if prev_j is not None:
            d = joints - prev_j
            delta = f"{d:+d}" if d != 0 else "—"
        bar_len = max(0, int(joints / 2))  # scale: 1 char = 2 joints
        bar = "█" * bar_len
        print(f"{label:<25} {filename:<30} {joints:>10}  {delta:>5}  {bar}")
        prev_j = joints
    print("-" * 70)
    print(f"{'Total reduction':<25} {'':30} {stages[0][2] - stages[-1][2]:>10}")
    print("=" * 70)

    # ------------------------------------------------------------------
    # IsaacSim instructions
    # ------------------------------------------------------------------
    print()
    print("=" * 70)
    print("  HOW TO VIEW IN ISAAC SIM")
    print("=" * 70)
    print()
    print("Load a single stage:")
    print()
    for _, filename, joints, _ in stages:
        if filename != "—":
            path = OUTPUT_DIR / filename
            print(f"  ~/isaacsim/python.sh -m isaacsim '{path}'")
    print()
    print("Or load all stages side by side (run separately).")
    print()
    print("See README.md for the visual checklist per technique.")
    print()

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
