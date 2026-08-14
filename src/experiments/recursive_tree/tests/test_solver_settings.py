#!/usr/bin/env python3
"""
Physics Solver Settings Test - Find optimal iterations

Uses test2 (stem + 1 petiole) as benchmark to test different solver settings.

Current settings (TREMA):
- positionIterations: 64
- velocityIterations: 8

Test progression:
1. baseline (64/8)   - current (trema)
2. medium (128/16)   - 2× increase
3. high (256/32)     - 4× increase
4. extreme (512/64)  - 8× increase
"""
import sys
import subprocess
from pathlib import Path

SCRIPT_DIR = Path(__file__).parent.resolve()
RECURSIVE_TREE_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(RECURSIVE_TREE_DIR))
sys.path.insert(0, str(SCRIPT_DIR))

from tree_config import validate_branches
from test_scalability import test_config_geometry

def generate_test2_branches():
    """Test 2: Stem + 1 petiole - our benchmark."""
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

def create_loader_script_with_settings(usd_path: str, pos_iter: int, vel_iter: int, output_script: Path):
    """Create Isaac Sim loader script with custom solver settings."""

    script_content = f'''
from omni.isaac.kit import SimulationApp
config = {{"headless": False, "width": 1920, "height": 1080}}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
from pxr import Usd, PhysxSchema
import carb

# Create world
world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()

# Get physics scene and set solver iterations
stage = world.stage
physics_scene_path = "/physicsScene"
physics_scene = stage.GetPrimAtPath(physics_scene_path)

if physics_scene:
    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(physics_scene)

    # SET CUSTOM SOLVER ITERATIONS
    physx_scene_api.CreateSolverPositionIterationCountAttr({pos_iter})
    physx_scene_api.CreateSolverVelocityIterationCountAttr({vel_iter})

    carb.log_info(f"Solver settings: positionIterations={{pos_iter}}, velocityIterations={{vel_iter}}")
    print(f"\\n{'='*80}")
    print(f"SOLVER SETTINGS:")
    print(f"  Position iterations: {pos_iter}")
    print(f"  Velocity iterations: {vel_iter}")
    print(f"{'='*80}\\n")

# Load USD
tree_prim = stage.DefinePrim("/World/tomato", "Xform")
tree_prim.GetReferences().AddReference("{usd_path}")

world.reset()

print("\\n" + "="*80)
print("Config loaded! Press PLAY to test, then CLOSE Isaac Sim when done.")
print("="*80 + "\\n")

# Run until user closes
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
'''

    with open(output_script, 'w') as f:
        f.write(script_content)

def test_solver_settings(name: str, pos_iter: int, vel_iter: int):
    """Test specific solver settings."""
    print(f"\n{'='*80}")
    print(f"TEST: {name}")
    print(f"Position iterations: {pos_iter}, Velocity iterations: {vel_iter}")
    print(f"{'='*80}")

    # Generate USD for test2
    branches = generate_test2_branches()
    config_name = f"solver_{name}_pos{pos_iter}_vel{vel_iter}"

    # Validate
    try:
        validate_branches(branches)
    except ValueError as e:
        print(f"❌ Validation failed: {e}")
        return None

    # Generate USD
    passed, max_error, details = test_config_geometry(
        config_name, branches, "SOLVER_TEST", save_usd=True
    )

    if not passed:
        print(f"❌ USD generation failed")
        return None

    usd_path = SCRIPT_DIR / "scalability_usds" / f"{config_name}.usda"
    print(f"✅ USD: {usd_path.name}")

    # Create custom loader script
    loader_script = SCRIPT_DIR / f"_load_{name}.py"
    create_loader_script_with_settings(str(usd_path), pos_iter, vel_iter, loader_script)
    print(f"✅ Loader: {loader_script.name}")

    return str(loader_script)

def main():
    print("="*80)
    print("PHYSICS SOLVER SETTINGS TEST")
    print("="*80)
    print()
    print("Benchmark: test2_stem_1petiole (8 links)")
    print()
    print("Testing different solver iteration counts to find stable settings")
    print()

    # Test configurations
    tests = [
        ("baseline", 64, 8),     # Current (trema)
        ("medium", 128, 16),     # 2× increase
        ("high", 256, 32),       # 4× increase
        ("extreme", 512, 64),    # 8× increase
    ]

    loaders = []
    for name, pos_iter, vel_iter in tests:
        loader = test_solver_settings(name, pos_iter, vel_iter)
        if loader:
            loaders.append((name, pos_iter, vel_iter, loader))

    print("\n" + "="*80)
    print("MANUAL TESTING INSTRUCTIONS")
    print("="*80)
    print()
    print("Test each configuration manually:")
    print()

    for i, (name, pos, vel, loader) in enumerate(loaders, 1):
        print(f"{i}. {name.upper()} (pos={pos}, vel={vel}):")
        print(f"   cd ~/isaacsim && ./python.sh {loader}")
        print(f"   → Press PLAY, observe for jitter, close Isaac Sim")
        print()

    print("Goal: Find minimum iterations needed for stable simulation")
    print()
    print("Expected:")
    print("  - baseline (64/8)   → TREMA (confirmed)")
    print("  - medium (128/16)   → Still jitter?")
    print("  - high (256/32)     → Stable?")
    print("  - extreme (512/64)  → Definitely stable (but slower)")
    print()
    print("Once you find stable settings, update tree_config.py or generate_recursive_tree_usda.py")
    print()

if __name__ == "__main__":
    main()
