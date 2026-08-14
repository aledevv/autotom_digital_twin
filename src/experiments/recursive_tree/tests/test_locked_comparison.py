#!/usr/bin/env python3
"""
Locked Joint Comparison Test

Generates test2 (stem + 1 petiole) with LOCKED joints (FixedJoint) instead of
flexible D6 joints. This helps isolate whether instability is caused by:
- Joint flexibility/drives → If locked is stable, problem is in joint tuning
- Geometry/collisions → If locked also has problems, issue is elsewhere

Usage:
    uv run src/experiments/recursive_tree/tests/test_locked_comparison.py
"""
import sys
import os
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))

from tree_config import validate_branches
from generate_recursive_tree_usda import build_stage_locked


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
    print("LOCKED JOINT COMPARISON TEST")
    print("=" * 80)
    print()
    print("Generating test2_stem_1petiole with LOCKED joints (FixedJoint)")
    print("This configuration has NO flexibility - all joints are rigid.")
    print()
    print("Purpose: Isolate whether instability is caused by:")
    print("  1. Joint flexibility/drives → locked should be stable")
    print("  2. Geometry/collisions → locked will also show problems")
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
    
    # Generate USD with LOCKED joints
    print()
    print("Step 2: Generating USD with locked joints...")
    usd_dir = SCRIPT_DIR / "scalability_usds"
    usd_dir.mkdir(exist_ok=True)
    usd_path = usd_dir / "test2_stem_1petiole_LOCKED.usda"
    
    try:
        stage, stem_path = build_stage_locked(str(usd_path), branches)
        stage.GetRootLayer().Save()
        print(f"  ✅ USD saved: {usd_path.name}")
    except Exception as e:
        print(f"  ❌ USD generation failed: {e}")
        return 1
    
    # Create loader script
    print()
    print("Step 3: Creating Isaac Sim loader script...")
    loader_script = SCRIPT_DIR / "_load_test2_locked.py"
    
    loader_content = f'''
from omni.isaac.kit import SimulationApp
config = {{"headless": False, "width": 1920, "height": 1080}}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
import carb

# Create world
world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()

# Load USD
stage = world.stage
tree_prim = stage.DefinePrim("/World/tomato", "Xform")
tree_prim.GetReferences().AddReference("{usd_path}")

world.reset()

print("\\n" + "="*80)
print("LOCKED JOINT TEST - test2_stem_1petiole")
print("="*80)
print("All joints are FixedJoint (completely rigid, no flexibility)")
print()
print("Expected behavior:")
print("  - Geometry should stay EXACTLY as shown (no movement at all)")
print("  - If geometry changes/moves → problem is in USD structure/physics setup")
print("  - If stable → original problem is joint flexibility/drive tuning")
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
    print("Run the locked joint test:")
    print(f"  cd ~/isaacsim && ./python.sh {loader_script}")
    print()
    print("What to observe:")
    print("  1. Before PLAY: Note the geometry (stem vertical, petiole at 45°)")
    print("  2. Press PLAY")
    print("  3. Observe if geometry changes:")
    print()
    print("If STABLE (no movement):")
    print("  → Problem is joint flexibility/drive settings")
    print("  → Need to tune stiffness/damping or increase solver iterations")
    print()
    print("If UNSTABLE (moves/collapses/jitters):")
    print("  → Problem is deeper (geometry, collision filtering, USD structure)")
    print("  → Check collision filtering, joint frames, mass distribution")
    print()
    print("Compare with flexible version:")
    print(f"  cd ~/isaacsim && ./python.sh {SCRIPT_DIR}/_load_baseline.py")
    print()
    print("=" * 80)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
