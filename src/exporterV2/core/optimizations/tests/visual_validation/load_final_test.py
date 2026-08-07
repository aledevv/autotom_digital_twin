"""
load_final_test.py - Load Before/After USD Comparison in Isaac Sim

Loads baseline and optimized USD files side-by-side for static visual comparison.
Prints a joint breakdown table after Isaac Sim completes initialization.

Usage:
    ~/isaacsim/python.sh src/exporterV2/core/optimizations/tests/visual_validation/load_final_test.py

Or use wrapper:
    ./load_final_test.sh
"""

import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
USD_DIR    = SCRIPT_DIR / "usd_output_before_after"
DAY        = 100
BUDGET     = 50
BASELINE_USD  = USD_DIR / f"day_{DAY}_baseline.usda"
OPTIMIZED_USD = USD_DIR / f"day_{DAY}_optimized_budget_{BUDGET}.usda"
COMPARISON_USD = USD_DIR / f"comparison_day_{DAY}_side_by_side.usda"
SPACING = 2.0

# Bootstrap Isaac Sim (this provides pxr module)
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Now we can import pxr
from pxr import Usd, UsdGeom, Gf
import omni.usd
import omni.kit.actions.core
import omni.kit.app


def count_by_category(usd_path, root_prim_path="/World"):
    """
    Count D6 joints per branch category from USD file.
    
    Args:
        usd_path: Path to USD file
        root_prim_path: Root prim to start traversal (e.g., "/World")
    """
    stage = Usd.Stage.Open(str(usd_path))
    cats = {"trunk": 0, "lateral": 0, "petiole": 0, "rachis": 0, "petiolule": 0, "other": 0}
    total_d6 = 0
    
    root_prim = stage.GetPrimAtPath(root_prim_path)
    if not root_prim:
        # Fallback: traverse entire stage
        root_prim = stage.GetPseudoRoot()
    
    for prim in Usd.PrimRange(root_prim):
        # Check if this is a joint (D6Joint or FixedJoint prim type)
        prim_type = prim.GetTypeName()
        if prim_type not in ("PhysicsJoint", "PhysicsFixedJoint", "PhysicsRevoluteJoint"):
            continue
        
        # Determine if it's a D6 joint (excludes Fixed joints)
        # Fixed joints have type PhysicsFixedJoint
        is_d6 = (prim_type != "PhysicsFixedJoint")
        
        # Only count D6 joints
        if not is_d6:
            continue
            
        # Extract branch ID from path
        # Path format: /World/Stem/BranchID_Link_XX/Joint
        # BranchID is embedded in the Link name before "_Link_"
        # Examples:
        #   /World/Stem/trunk_Link_02/Joint_01_02 → bid="trunk"
        #   /World/Stem/Branch_r1_o0_Link_01/AttachJoint → bid="Branch_r1_o0"
        #   /World/Stem/Leaf_r1_o0_petiole_Link_01/AttachJoint → bid="Leaf_r1_o0_petiole"
        path = str(prim.GetPath())
        parts = path.split("/")
        
        # Joint is child of Link, get Link name
        if len(parts) < 2:
            continue
        link_name = parts[-2]  # Parent of Joint
        
        # Extract branch ID from link name (everything before "_Link_")
        if "_Link_" not in link_name:
            continue
        bid = link_name.split("_Link_")[0]
        
        if not bid:
            continue
        
        # Categorize based on branch ID
        if bid in ("Stem", "trunk") or bid.startswith("trunk"):
            cat = "trunk"
        elif bid.startswith("Branch_r"):
            cat = "lateral"
        elif "_petiole" in bid and "_merged" not in bid:
            cat = "petiole"
        elif "petiolule" in bid.lower():
            cat = "petiolule"  # Check BEFORE rachis
        elif "_rachis" in bid or "_merged" in bid:
            cat = "rachis"
        else:
            cat = "other"
        
        cats[cat] += 1
        total_d6 += 1
    
    return cats, total_d6


def print_comparison_table():
    """Print joint breakdown comparing baseline vs optimized."""
    print("\n[INFO] Reading joint counts from USD files...")
    
    # Read from actual USD files
    before_cats, baseline_total = count_by_category(BASELINE_USD, "/World")
    after_cats, optimized_total = count_by_category(OPTIMIZED_USD, "/World")
    
    print(f"  Baseline:  {baseline_total} D6 joints")
    print(f"  Optimized: {optimized_total} D6 joints")
    
    # Read optimization metadata from optimized USD
    stage_opt = Usd.Stage.Open(str(OPTIMIZED_USD))
    root_prim = stage_opt.GetPrimAtPath("/World")
    minimum_achievable = None
    if root_prim:
        minimum_achievable = root_prim.GetCustomDataByKey("optimization:minimum_achievable")
    
    W = 74
    print("\n" + "=" * W)
    print("  JOINT BREAKDOWN: Before vs After Optimization")
    print("=" * W)
    print(f"  {'Category':<14} {'Joints':>10}  {'After':>10}  {'Delta':>8}  {'Change':>8}")
    print("  " + "-" * (W - 2))
    
    for cat in ["trunk", "lateral", "petiole", "rachis", "petiolule", "other"]:
        b = before_cats.get(cat, 0)
        a = after_cats.get(cat, 0)
        if b == 0 and a == 0:
            continue
        delta = a - b
        pct   = (delta / b * 100) if b > 0 else 0.0
        d_str = f"{delta:+d}" if delta != 0 else "–"
        p_str = f"{pct:+.0f}%" if delta != 0 else "–"
        
        # Special note for petiolules converted to Fixed
        if cat == "petiolule" and b > 0 and a == 0:
            p_str = "→Fixed"
        
        print(f"  {cat:<14} {b:>10}  {a:>10}  {d_str:>8}  {p_str:>8}")
    
    print("  " + "-" * (W - 2))
    delta_tot = optimized_total - baseline_total
    pct_tot   = delta_tot / baseline_total * 100 if baseline_total > 0 else 0.0
    print(f"  {'TOTAL':<14} {baseline_total:>10}  {optimized_total:>10}  {delta_tot:+8d}  {pct_tot:>+7.1f}%")
    print("  " + "-" * (W - 2))
    
    status = "✓ Within budget" if optimized_total <= BUDGET else f"⚠ Over by {optimized_total - BUDGET}"
    print(f"  Budget: {BUDGET}  |  {status}")
    
    # Show minimum achievable if available
    if minimum_achievable is not None:
        max_reduction_pct = (baseline_total - minimum_achievable) / baseline_total * 100 if baseline_total > 0 else 0.0
        print(f"  Min achievable: {minimum_achievable} (max {max_reduction_pct:.1f}% reduction)")
    
    print("=" * W)


def create_comparison_stage():
    """Create comparison stage with both plants via references."""
    print("\n[STEP 1/3] Creating comparison stage...")
    stage = Usd.Stage.CreateNew(str(COMPARISON_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    print(f"  ✓ Empty stage: {COMPARISON_USD.name}")
    
    print("\n[STEP 2/3] Referencing baseline (left)...")
    xf = UsdGeom.Xform.Define(stage, "/Baseline")
    xf.GetPrim().GetReferences().AddReference(str(BASELINE_USD))
    xf.AddTranslateOp().Set(Gf.Vec3d(-SPACING / 2, 0, 0))
    print(f"  ✓ /Baseline ← {BASELINE_USD.name}  (x={-SPACING/2:.1f}m)")
    
    print("\n[STEP 3/3] Referencing optimized (right)...")
    xf2 = UsdGeom.Xform.Define(stage, "/Optimized")
    xf2.GetPrim().GetReferences().AddReference(str(OPTIMIZED_USD))
    xf2.AddTranslateOp().Set(Gf.Vec3d(SPACING / 2, 0, 0))
    print(f"  ✓ /Optimized ← {OPTIMIZED_USD.name}  (x=+{SPACING/2:.1f}m)")
    
    stage.GetRootLayer().Save()
    print(f"\n  ✓ Comparison stage saved")


def main():
    print("=" * 80)
    print("  Before/After Optimization — Static Visual Comparison")
    print("=" * 80)
    
    # Validate files
    if not BASELINE_USD.exists():
        print(f"\n[ERROR] Baseline USD not found: {BASELINE_USD}")
        print("[HINT] Run generate_final_test.py first")
        simulation_app.close()
        sys.exit(1)
    
    if not OPTIMIZED_USD.exists():
        print(f"\n[ERROR] Optimized USD not found: {OPTIMIZED_USD}")
        print("[HINT] Run generate_final_test.py first")
        simulation_app.close()
        sys.exit(1)
    
    # Build comparison stage if needed
    if not COMPARISON_USD.exists():
        create_comparison_stage()
    else:
        print(f"\n[INFO] Reusing: {COMPARISON_USD.name}")
        print("  (delete to regenerate)")
    
    # Open in Isaac Sim
    print(f"\n[LOADING] Opening stage...")
    omni.usd.get_context().open_stage(str(COMPARISON_USD))
    print("  ✓ Stage opened")
    
    # Wait for references to resolve
    print("  ⏳ Resolving references...")
    for _ in range(20):
        omni.kit.app.get_app().update()
    
    # Apply camera lighting
    try:
        reg = omni.kit.actions.core.get_action_registry()
        action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
    except Exception:
        pass
    
    # ---- Print table HERE (after Isaac Sim startup noise) ----
    print_comparison_table()
    
    print("\n" + "=" * 80)
    print("  Viewer Ready")
    print("=" * 80)
    print(f"  LEFT  /Baseline  → {BASELINE_USD.name}")
    print(f"  RIGHT /Optimized → {OPTIMIZED_USD.name}")
    print()
    print("  Static view (no physics). Close window to exit.")
    print("=" * 80 + "\n")
    
    # Keep viewer open
    while simulation_app.is_running():
        simulation_app.update()
    
    print("[INFO] Viewer closed.")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted.")
        simulation_app.close()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
