"""
generate_thin_link_usd.py - Genera USD per validazione visiva Thin Link Lock

Genera due file USD in src/experiments/usd_output:
  - thin_link_before.usda : Albero con rami sottili con giunti D6 (rischio instabilità).
  - thin_link_after.usda  : Albero con rami sottili con giunti Fixed.
"""

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../"))
sys.path.insert(0, PROJECT_ROOT)

from src.exporterV2.core.usd.stage import build_stage
from src.exporterV2.core.optimizations.techniques.thin_link_lock import ThinLinkLockTechnique

OUTPUT_DIR = os.path.join(SCRIPT_DIR, "usd_output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Crea configurazione di rami
# Global scale è 1.0, MIN_LINK_RADIUS_WORLD = 0.002
def get_test_branches():
    return [
        {
            "id": "trunk",
            "parent": None,
            "n_links": 3,
            "radius": 0.015,  # 15mm (Safe)
            "height": 0.20,
            "tilt": 0.0,
            "rot": 0.0,
            "joint_type": "d6"
        },
        {
            "id": "branch_safe",
            "parent": "trunk",
            "attach_link": 2,
            "n_links": 2,
            "radius": 0.005,  # 5mm (Safe)
            "height": 0.15,
            "tilt": 45.0,
            "rot": 0.0,
            "joint_type": "d6"
        },
        {
            "id": "branch_thin",
            "parent": "trunk",
            "attach_link": 3,
            "n_links": 2,
            "radius": 0.002,  # 2mm (Unsafe for D6, Safe if Fixed)
            "height": 0.10,
            "tilt": -45.0,
            "rot": 180.0,
            "joint_type": "d6"
        }
    ]

def main():
    print("=" * 65)
    print("  Thin Link Lock Test — USD Generator for Isaac Sim")
    print("=" * 65)

    before_branches = get_test_branches()
    tech = ThinLinkLockTechnique()
    
    print("\n[1/2] Generating BEFORE (Tutti giunti D6)...")
    before_path = os.path.join(OUTPUT_DIR, "thin_link_before.usda")
    try:
        # Pass skip_limit_check=True per evitare problemi con numeri piccoli di rami
        stage_b, _ = build_stage(before_path, branches=before_branches, skip_limit_check=True)
        stage_b.Save()
        print(f"  ✓ Saved BEFORE: {before_path}")
    except Exception as exc:
        import traceback; traceback.print_exc(); return 1
        
    print("\n[2/2] Generating AFTER (Thin Link Lock applicato)...")
    after_branches, report = tech.apply(before_branches)
    print(report)
    
    after_path = os.path.join(OUTPUT_DIR, "thin_link_after.usda")
    try:
        stage_a, _ = build_stage(after_path, branches=after_branches, skip_limit_check=True)
        stage_a.Save()
        print(f"  ✓ Saved AFTER: {after_path}")
    except Exception as exc:
        import traceback; traceback.print_exc(); return 1

    print(f"""
  Open in Isaac Sim:
    ~/isaacsim/python.sh -m isaacsim {before_path}
    ~/isaacsim/python.sh -m isaacsim {after_path}
    """)
    return 0

if __name__ == "__main__":
    sys.exit(main())
