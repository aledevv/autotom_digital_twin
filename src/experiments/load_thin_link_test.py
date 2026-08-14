"""
load_thin_link_test.py - Visual validation for Thin Link Lock in Isaac Sim

Loads the BEFORE and AFTER USD files side-by-side. 
Hit Play in Isaac Sim to watch physics stability:
- LEFT (BEFORE): Thin branch (2mm clamped) with D6 joint -> highly unstable, might explode.
- RIGHT (AFTER): Thin branch with Fixed joint -> completely stable.

Run:
    ~/isaacsim/python.sh src/experiments/load_thin_link_test.py
"""

import sys
import os
from pathlib import Path

# Paths
SCRIPT_DIR = Path(__file__).parent.resolve()
USD_DIR    = SCRIPT_DIR / "usd_output"
BEFORE_USD = USD_DIR / "thin_link_before.usda"
AFTER_USD  = USD_DIR / "thin_link_after.usda"
COMPARISON_USD = USD_DIR / "thin_link_comparison.usda"
SPACING = 0.5

# Generate the base USDs if they don't exist
# I file vengono ora generati all'esterno (poiché uv e isaacsim-python hanno conflitti su SRE)
if not BEFORE_USD.exists() or not AFTER_USD.exists():
    print("[ERROR] USD files missing! Run generate_thin_link_usd.py first.")
    sys.exit(1)

# Bootstrap Isaac Sim
print("[INFO] Starting Isaac Sim...")
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, Gf
import omni.usd
import omni.kit.actions.core
import omni.kit.app

def create_comparison_stage():
    """Create comparison stage with both plants via references."""
    print("\n[STEP 1/3] Creating comparison stage...")
    stage = Usd.Stage.CreateNew(str(COMPARISON_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    print("\n[STEP 2/3] Referencing BEFORE (D6 joints)...")
    xf = UsdGeom.Xform.Define(stage, "/Before")
    xf.GetPrim().GetReferences().AddReference(str(BEFORE_USD))
    xf.AddTranslateOp().Set(Gf.Vec3d(-SPACING / 2, 0, 0))
    
    print("\n[STEP 3/3] Referencing AFTER (Thin Link Locked)...")
    xf2 = UsdGeom.Xform.Define(stage, "/After")
    xf2.GetPrim().GetReferences().AddReference(str(AFTER_USD))
    xf2.AddTranslateOp().Set(Gf.Vec3d(SPACING / 2, 0, 0))
    
    stage.GetRootLayer().Save()

def main():
    print("=" * 80)
    print("  Thin Link Lock — Physical Stability Test")
    print("=" * 80)
    
    # Build comparison stage if needed
    if not COMPARISON_USD.exists():
        create_comparison_stage()
    else:
        print(f"\n[INFO] Reusing: {COMPARISON_USD.name} (delete to regenerate)")
    
    # Open in Isaac Sim
    print(f"\n[LOADING] Opening stage...")
    omni.usd.get_context().open_stage(str(COMPARISON_USD))
    print("  ✓ Stage opened")
    
    # Wait for references to resolve
    for _ in range(20):
        omni.kit.app.get_app().update()
        
    try:
        reg = omni.kit.actions.core.get_action_registry()
        action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
    except Exception:
        pass
    
    print("\n" + "=" * 80)
    print("  Viewer Ready!")
    print("=" * 80)
    print("  LEFT  (BEFORE): Thin branch (2mm) with D6 joint -> Physically unstable.")
    print("  RIGHT (AFTER) : Thin branch (2mm) with Fixed joint -> Stable.")
    print("\n  >> Hit PLAY (Spacebar) to test physics simulation. <<")
    print("=" * 80 + "\n")
    
    # Keep viewer open
    while simulation_app.is_running():
        simulation_app.update()
    
    simulation_app.close()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        simulation_app.close()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        simulation_app.close()
