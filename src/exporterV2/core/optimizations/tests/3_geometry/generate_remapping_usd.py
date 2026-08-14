"""
generate_remapping_usd.py - Generate Before/After USD for Isaac Sim remapping test

Generates two USD files to validate geometric attachment remapping visually in Isaac Sim:
  • remapping_before.usda  — trunk N_OLD=10 segments, 3 branches at L9, L4, L10
  • remapping_after.usda   — trunk collapsed to N_NEW=3, branches remapped with attach_frac

Mathematical model (p=1.0 top-of-link convention):
    H_abs   = attach_link_old / N_old                    (fraction of total height)
    V       = H_abs * N_new                              (position in new-link units)
    k_new   = floor(V) + 1  if H<1.0,  else N_new        (1-based, edge-case guarded)
    p_new   = V - floor(V)                               (fraction WITHIN new link)

The key fix: attach_frac = p_new is stored in the branch dict and used by build_stage
to set LocalPos0.z = p_new * seg_h_new  (instead of always seg_h_new + gap).
This gives exact sub-link positioning in Isaac Sim.

Expected world-Z of each branch (MUST match before ↔ after):
    Branch_A at L9  → H=0.90m  → after: L3 @ p=0.70  z=0.90m ✓
    Branch_B at L4  → H=0.40m  → after: L2 @ p=0.20  z=0.40m ✓
    Branch_C at L10 → H=1.00m  → after: L3 @ p=1.00  z=1.00m ✓  (edge: top)

Run:
    cd /home/alessandro/isaacsim/autotom_digital_twin
    uv run python src/exporterV2/core/optimizations/tests/3_geometry/generate_remapping_usd.py

Open in Isaac Sim:
    ~/isaacsim/python.sh -m isaacsim <path>/remapping_before.usda
    ~/isaacsim/python.sh -m isaacsim <path>/remapping_after.usda
"""

import math
import sys
import os

# ── path setup ────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../../.."))
sys.path.insert(0, PROJECT_ROOT)

from exporterV2.core.usd.stage import build_stage

OUTPUT_DIR = os.path.join(SCRIPT_DIR, "usd_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── remapping math ────────────────────────────────────────────────────────────

def remap(attach_link: int, n_old: int, n_new: int) -> tuple[int, float]:
    """
    Remap 1-based attach_link from n_old → n_new.

    Returns:
        (k_new, p_new)
        k_new : 1-based link index in new trunk  [1 .. n_new]
        p_new : fractional position within k_new  [0.0 .. 1.0]
                0.0 = base of link,  1.0 = top of link

    Convention: p=1.0 means branch sits at the TOP of its link (original convention).
    """
    H = attach_link / n_old          # absolute height fraction ∈ (0, 1]
    if H >= 1.0:
        return n_new, 1.0            # edge case: top of last new segment
    V = H * n_new
    k = math.floor(V) + 1           # 1-based link
    p = V - math.floor(V)           # fraction within link
    return k, p


# ── trunk parameters ──────────────────────────────────────────────────────────

N_OLD      = 10
N_NEW      = 3
SEG_H_OLD  = 0.10    # 10 cm per segment  →  total trunk = 1.0 m
TOTAL_H    = N_OLD * SEG_H_OLD
SEG_H_NEW  = TOTAL_H / N_NEW   # 33.3 cm per segment in reduced trunk

TRUNK_R    = 0.030
BRANCH_R   = 0.012
BRANCH_H   = 0.20
BRANCH_TILT = 90.0   # horizontal for easy Z-measurement in Isaac Sim

# 3 test branches — different rot angles so they spread visually
BRANCH_DEFS = [
    {"label": "Branch_A", "link_old": 9,  "rot": 0.0,   "note": "H=0.90m → L3@70%"},
    {"label": "Branch_B", "link_old": 4,  "rot": 120.0, "note": "H=0.40m → L2@20%"},
    {"label": "Branch_C", "link_old": 10, "rot": 240.0, "note": "H=1.00m → L3@100% (edge)"},
]


# ── branch list builders ──────────────────────────────────────────────────────

def make_before_branches() -> list[dict]:
    """N_OLD trunk + 3 branches at original links (no attach_frac needed = top-of-link)."""
    branches = [{
        "id": "trunk", "parent": None,
        "n_links": N_OLD, "height": SEG_H_OLD, "radius": TRUNK_R,
        "tilt": 0.0, "rot": 0.0,
    }]
    for b in BRANCH_DEFS:
        branches.append({
            "id":          b["label"],
            "parent":      "trunk",
            "attach_link": b["link_old"],   # 1-based
            # attach_frac defaults to 1.0 → top of link (original behaviour)
            "n_links":     1,
            "height":      BRANCH_H,
            "radius":      BRANCH_R,
            "tilt":        BRANCH_TILT,
            "rot":         b["rot"],
        })
    return branches


def make_after_branches() -> list[dict]:
    """
    N_NEW trunk + 3 branches remapped with exact sub-link fraction.
    attach_frac = p_new is passed so build_stage places the joint at
    LocalPos0.z = p_new * seg_h_new  (not always at the top of the link).
    """
    branches = [{
        "id": "trunk", "parent": None,
        "n_links": N_NEW, "height": SEG_H_NEW, "radius": TRUNK_R,
        "tilt": 0.0, "rot": 0.0,
    }]
    for b in BRANCH_DEFS:
        k_new, p_new = remap(b["link_old"], N_OLD, N_NEW)
        z_expected   = b["link_old"] * SEG_H_OLD
        z_check      = (k_new - 1) * SEG_H_NEW + p_new * SEG_H_NEW
        err          = abs(z_expected - z_check)

        print(f"  {b['label']:<12}  old=L{b['link_old']:>2}  H={b['link_old']/N_OLD:.2f}"
              f"  →  new=L{k_new} p={p_new*100:.1f}%"
              f"  z_expected={z_expected:.4f}  z_check={z_check:.4f}"
              f"  Δ={err:.2e}")

        branches.append({
            "id":          b["label"],
            "parent":      "trunk",
            "attach_link": k_new,     # 1-based new link
            "attach_frac": p_new,     # ← KEY FIX: sub-link fraction for build_stage
            "n_links":     1,
            "height":      BRANCH_H,
            "radius":      BRANCH_R,
            "tilt":        BRANCH_TILT,
            "rot":         b["rot"],
        })
    return branches


# ── USD generation ────────────────────────────────────────────────────────────

def main():
    print("=" * 65)
    print("  Remapping Test — USD Generator for Isaac Sim")
    print(f"  Trunk: N={N_OLD} → N={N_NEW},  total height={TOTAL_H:.2f}m")
    print("=" * 65)

    # ── BEFORE ────────────────────────────────────────────────────────────────
    print(f"\n[1/2] Generating BEFORE ({N_OLD}-link trunk)...")
    before_branches = make_before_branches()
    before_path = os.path.join(OUTPUT_DIR, "remapping_before.usda")
    try:
        stage_b, _ = build_stage(before_path, branches=before_branches,
                                  locked_joints=True, skip_limit_check=True)
        stage_b.Save()
        n = sum(b["n_links"] for b in before_branches)
        print(f"  ✓ Saved ({n} links): {before_path}")
    except Exception as exc:
        import traceback; traceback.print_exc(); return 1

    # ── AFTER ─────────────────────────────────────────────────────────────────
    print(f"\n[2/2] Generating AFTER ({N_NEW}-link trunk, attach_frac fix)...")
    after_branches = make_after_branches()
    after_path = os.path.join(OUTPUT_DIR, "remapping_after.usda")
    try:
        stage_a, _ = build_stage(after_path, branches=after_branches,
                                  locked_joints=True, skip_limit_check=True)
        stage_a.Save()
        n = sum(b["n_links"] for b in after_branches)
        print(f"  ✓ Saved ({n} links): {after_path}")
    except Exception as exc:
        import traceback; traceback.print_exc(); return 1

    # ── summary ───────────────────────────────────────────────────────────────
    print("\n" + "=" * 65)
    print("  Summary — expected world-Z (must match before ↔ after)")
    print("=" * 65)
    for b in BRANCH_DEFS:
        z   = b["link_old"] * SEG_H_OLD
        k, p = remap(b["link_old"], N_OLD, N_NEW)
        print(f"  {b['label']:<12}  Z={z:.3f}m   {b['note']}"
              f"  (attach_frac={p:.2f})")

    print(f"""
  Open in Isaac Sim:
    ~/isaacsim/python.sh -m isaacsim {before_path}
    ~/isaacsim/python.sh -m isaacsim {after_path}

  What to verify:
  • Select each branch → check world translation Z
  • Branch_A: Z must be 0.900m in BOTH files
  • Branch_B: Z must be 0.400m in BOTH files
  • Branch_C: Z must be 1.000m in BOTH files
  • Joint count: before=13, after=6
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
