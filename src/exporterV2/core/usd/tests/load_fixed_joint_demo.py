"""
load_fixed_joint_demo.py - Isaac Sim loader for fixed joint demo

Loads the fixed joint demo USD and sets up simulation.

Run in Isaac Sim:
    python load_fixed_joint_demo.py
"""

import os
from pathlib import Path
from omni.isaac.kit import SimulationApp

# Start Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
import carb

def main():
    print("\n" + "="*80)
    print("  Fixed Joint Demo - Isaac Sim Loader")
    print("="*80)
    
    # Find USD file
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent.parent.parent.parent
    usd_path = project_root / "data" / "usd_models" / "fixed_joint_demo.usda"
    
    if not usd_path.exists():
        print(f"\n✗ ERROR: USD file not found at {usd_path}")
        print(f"  Run demo_fixed_joint.py first to generate the USD")
        simulation_app.close()
        return
    
    print(f"\nLoading USD: {usd_path}")
    
    # Load stage
    open_stage(str(usd_path))
    
    # Create world with physics
    world = World()
    world.scene.add_default_ground_plane()
    
    print(f"✓ Stage loaded successfully")
    print(f"\nSimulation ready:")
    print(f"  - Multiple pedicels (cylinders) with spheres at tips")
    print(f"  - Spheres attached with FixedJoint (rigid attachment)")
    print(f"  - Press PLAY button to start physics simulation")
    print(f"  - Spheres should stay rigidly attached to pedicels")
    print(f"  - Check Physics Inspector:")
    print(f"    * Each sphere should have a FixedJoint")
    print(f"    * Joint body0 = pedicel, body1 = sphere")
    print(f"    * No relative motion between sphere and pedicel")
    
    print("\n" + "="*80)
    print("  Isaac Sim Running - Close window to exit")
    print("="*80)
    
    # Keep simulation running
    while simulation_app.is_running():
        world.step(render=True)
    
    simulation_app.close()


if __name__ == "__main__":
    main()
