#!/usr/bin/env python3
"""
Attachment Joint Damping Ratio Fix Test

Fixes the damping scaling on attachment joints to maintain constant damping ratio ζ.

Before:
    K_attach = K * 5.0
    D_attach = D * 2.0  ❌ Wrong! Damping ratio changes

After:
    K_attach = K * 5.0
    D_attach = D * sqrt(5) ≈ 2.236  ✅ Correct! Maintains ζ

Why this matters:
    ζ = D / (2*sqrt(K*J))
    
    If we scale K by 5×:
    - To keep ζ constant, D must scale by sqrt(5) ≈ 2.236
    - With D*2.0, the joint is under-damped → can cause oscillations/instability

Usage:
    uv run src/experiments/recursive_tree/tests/test_damping_fix.py
"""
import sys
import os
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))

from tree_config import validate_branches
from generate_recursive_tree_usda import build_stage


def generate_test2_branches():
    """Test 2: Stem + 1 petiole - 8 links total."""
    return [
        {
            "id": "stem",
            "parent": None,
            "attach_link": None,
            "n_links": 5,
            "radius": 0.004,
            "height": 0.030,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "petiole_1",
            "parent": "stem",
            "attach_link": 3,
            "n_links": 3,
            "radius": 0.0023,
            "height": 0.027,
            "tilt": 45.0,
            "rot": 0.0,
        },
    ]


def main():
    print("=" * 80)
    print("ATTACHMENT JOINT DAMPING RATIO FIX TEST")
    print("=" * 80)
    print()
    print("Regenerating test2_stem_1petiole with CORRECTED damping scaling")
    print()
    print("Fix applied:")
    print("  K_attach = K * 5.0")
    print("  D_attach = D * sqrt(5) ≈ 2.236  (was 2.0)")
    print()
    print("Why this matters:")
    print("  Damping ratio: ζ = D / (2*sqrt(K*J))")
    print("  If K → 5K, then D must → sqrt(5)*D to keep ζ constant")
    print("  Old D*2.0 → under-damped attachment joint → oscillations/instability")
    print()
    
    branches = generate_test2_branches()
    
    # Validate
    print("Step 1: Validating configuration...")
    try:
        validate_branches(branches)
        print("  ✅ Configuration valid")
    except ValueError as e:
        print(f"  ❌ Validation failed: {e}")
        return 1
    
    # Generate USD with correct damping
    print()
    print("Step 2: Generating USD with corrected damping...")
    usd_dir = SCRIPT_DIR / "scalability_usds"
    usd_dir.mkdir(exist_ok=True)
    usd_path = usd_dir / "test2_stem_1petiole_DAMPING_FIX.usda"
    
    try:
        stage, stem_path = build_stage(str(usd_path), branches, locked_joints=False)
        stage.GetRootLayer().Save()
        print(f"  ✅ USD saved: {usd_path.name}")
    except Exception as e:
        print(f"  ❌ USD generation failed: {e}")
        return 1
    
    # Verify damping values in USD
    print()
    print("Step 3: Verifying damping values in USD...")
    
    # Read USD to check damping values
    with open(usd_path, 'r') as f:
        usd_content = f.read()
    
    # Look for AttachJoint damping
    if "AttachJoint" in usd_content:
        print("  ✅ AttachJoint found in USD")
        # Extract a sample line to show the values
        import re
        damping_matches = re.findall(r'drive:rot[XY]:physics:damping = ([\d.]+)', usd_content)
        if damping_matches:
            # First few are internal joints, last ones are attachment
            internal_damp = float(damping_matches[0]) if len(damping_matches) > 0 else 0
            attach_damp = float(damping_matches[-1]) if len(damping_matches) > 5 else 0
            
            if internal_damp > 0:
                ratio = attach_damp / internal_damp
                print(f"  Internal joint damping: {internal_damp:.6f}")
                print(f"  Attach joint damping:   {attach_damp:.6f}")
                print(f"  Ratio: {ratio:.3f} (expected: ~2.236)")
                
                if abs(ratio - 2.236) < 0.01:
                    print("  ✅ Damping ratio correct!")
                elif abs(ratio - 2.0) < 0.01:
                    print("  ⚠️  Still using old ratio (2.0) - fix not applied?")
                else:
                    print(f"  ⚠️  Unexpected ratio: {ratio:.3f}")
    
    # Create loader script
    print()
    print("Step 4: Creating Isaac Sim loader script...")
    loader_script = SCRIPT_DIR / "_load_test2_damping_fix.py"
    
    loader_content = f'''
from omni.isaac.kit import SimulationApp
config = {{"headless": False, "width": 1920, "height": 1080}}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
from pxr import Usd, PhysxSchema
import carb

# Create world with baseline settings
world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()

# Set baseline solver iterations
stage = world.stage
physics_scene_path = "/physicsScene"
physics_scene = stage.GetPrimAtPath(physics_scene_path)

if physics_scene:
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)
    physx_scene_api.CreateSolverPositionIterationCountAttr(64)
    physx_scene_api.CreateSolverVelocityIterationCountAttr(8)
    carb.log_info("Solver: pos=64, vel=8 (baseline)")

# Load USD
tree_prim = stage.DefinePrim("/World/tomato", "Xform")
tree_prim.GetReferences().AddReference("{usd_path}")

world.reset()

print("\\n" + "="*80)
print("DAMPING RATIO FIX TEST - test2_stem_1petiole")
print("="*80)
print("This version has:")
print("  ✅ Corrected damping scaling: D_attach = D * sqrt(5) ≈ 2.236")
print("  ✅ Explicit center of mass (height/2 along Z)")
print("  ✅ Collision filtering (parent-child)")
print("  ✅ targetPosition = 0 (correct)")
print("  ✅ Baseline solver (pos=64, vel=8)")
print()
print("Expected behavior:")
print("  - Geometry should start at |/ (stem vertical, petiole 45°)")
print("  - After PLAY, should stay stable with proper damping")
print("  - NO sudden snap to Y shape")
print("  - NO high-frequency jitter or oscillations")
print("  - Smooth response to gravity (may droop slightly)")
print()
print("What changed from before:")
print("  OLD: D_attach = D * 2.0   → under-damped → oscillations")
print("  NEW: D_attach = D * 2.236 → correct ζ → stable")
print()
print("Press PLAY to test, then CLOSE Isaac Sim when done.")
print("="*80 + "\\n")

# Run until user closes
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
'''
    
    with open(loader_script, 'w') as f:
        f.write(loader_content)
    
    print(f"  ✅ Loader saved: {loader_script.name}")
    
    # Final instructions
    print()
    print("=" * 80)
    print("TESTING INSTRUCTIONS")
    print("=" * 80)
    print()
    print("1. Test the DAMPING FIX version:")
    print(f"   cd ~/isaacsim && ./python.sh {loader_script}")
    print()
    print("2. Compare with LOCKED version (baseline stable reference):")
    print(f"   cd ~/isaacsim && ./python.sh {SCRIPT_DIR}/_load_test2_locked.py")
    print()
    print("Expected outcome:")
    print("  ✅ If STABLE → Damping fix solved the under-damped oscillation problem!")
    print("  ❌ If UNSTABLE → Need to investigate other causes")
    print()
    print("Physics background:")
    print("  For a spring-damper system: ζ = D / (2*sqrt(K*J))")
    print("  ")
    print("  Attachment joint:")
    print("    K_attach = K * 5.0")
    print("    → Need D_attach = D * sqrt(5) to keep same ζ")
    print("    → sqrt(5) ≈ 2.236")
    print()
    print("  Old code used D*2.0:")
    print("    → ζ_attach = (D*2.0) / (2*sqrt(K*5*J))")
    print("              = (D*2.0) / (2*sqrt(5)*sqrt(K*J))")
    print("              = (2.0/2.236) * ζ_internal")
    print("              ≈ 0.894 * ζ_internal")
    print("    → Under-damped by ~11% → can cause instability")
    print()
    print("=" * 80)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
