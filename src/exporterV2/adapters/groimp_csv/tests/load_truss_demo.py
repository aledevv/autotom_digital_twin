"""
load_truss_demo.py - Isaac Sim loader for truss demos

Loads truss USD with tomatoes and sets up simulation.

Run in Isaac Sim:
    ~/isaacsim/python.sh load_truss_demo.py [flexible|locked]
    
    flexible (default): Load flexible truss with D6 joints (realistic drooping)
    locked: Load locked truss with rigid joints (stable, no movement)
"""

import os
import sys
from pathlib import Path
from omni.isaac.kit import SimulationApp

# Start Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import carb

def main():
    # Parse command line argument
    demo_type = "flexible"  # Default to flexible
    if len(sys.argv) > 1:
        demo_type = sys.argv[1].lower()
        if demo_type not in ["flexible", "locked"]:
            print(f"✗ ERROR: Invalid demo type '{demo_type}'. Use 'flexible' or 'locked'")
            simulation_app.close()
            return
    
    print("\n" + "="*80)
    print(f"  Truss Demo - Isaac Sim Loader ({demo_type.upper()})")
    print("="*80)
    
    # Find USD file
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent.parent.parent.parent
    
    if demo_type == "flexible":
        usd_filename = "truss_flexible_demo.usda"
        demo_script = "demo_flexible_truss.py"
        joint_type = "D6 spring-damper (flexible)"
    else:
        usd_filename = "truss_complete_demo.usda"
        demo_script = "demo_complete_truss.py"
        joint_type = "Locked (rigid)"
    
    usd_path = project_root / "data" / "usd_models" / usd_filename
    
    if not usd_path.exists():
        print(f"\n✗ ERROR: USD file not found at {usd_path}")
        print(f"  Run {demo_script} first to generate the USD:")
        print(f"  uv run python src/exporterV2/adapters/groimp_csv/tests/{demo_script}")
        simulation_app.close()
        return
    
    print(f"\nDemo type: {demo_type.upper()}")
    print(f"Joint type: {joint_type}")
    print(f"Loading USD: {usd_path}")
    
    # Load stage
    open_stage(str(usd_path))
    
    # Create world with physics
    world = World()
    world.scene.add_default_ground_plane()
    
    print(f"✓ Stage loaded successfully")
    print(f"\nSimulation ready:")
    print(f"  - Trunk with truss attached at rank 3")
    print(f"  - Rachis (4 segments) drooping downward")
    print(f"  - Pedicels (6 lateral + 1 terminal) with tomatoes")
    print(f"  - 7 tomatoes of varying sizes")
    print(f"  - Joint type: {joint_type}")
    
    if demo_type == "flexible":
        print(f"\nExpected behavior (FLEXIBLE):")
        print(f"  ✓ Trunk, rachis, and pedicels should bend naturally under gravity")
        print(f"  ✓ Truss should droop with realistic physics")
        print(f"  ✓ Tomatoes remain attached (FixedJoint)")
        print(f"  ✓ Smooth spring-damper motion (no violent oscillations)")
    else:
        print(f"\nExpected behavior (LOCKED):")
        print(f"  ✓ All joints are rigid (no movement)")
        print(f"  ✓ Structure remains in initial configuration")
        print(f"  ✓ Useful for debugging geometry")
    
    print(f"\nPress PLAY button to start physics simulation")
    
    print("\n" + "="*80)
    print("  Isaac Sim Running - Close window to exit")
    print("="*80)
    
    # Keep simulation running
    while simulation_app.is_running():
        world.step(render=True)
    
    simulation_app.close()


if __name__ == "__main__":
    main()
