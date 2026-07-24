"""
test_add_branch.py

Extensively tests the `add_branch` feature of the PlantBuilder API.
Creates a main trunk and tests 8 different branch parameter combinations
(many/few segments, big/small radius, long/short length).
"""

import os
import sys

# --- bootstrap Isaac Sim (headless) so pxr becomes available -----------------
from isaacsim import SimulationApp
_sim_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom


script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(script_dir))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from src.plant_model.plant_builder import PlantBuilder

def get_output_usd_path() -> str:
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "test_add_branch.usda")

def main():
    output_path = get_output_usd_path()
    
    if os.path.exists(output_path):
        os.remove(output_path)
        
    import omni.usd
    from isaacsim.core.api import World
    
    # 1. Initialize World first (this creates the default active stage and physics scene)
    world = World(stage_units_in_meters=1.0)
    world.reset()
    
    # 2. Get the active stage
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    # 3. Build the plant directly in the active stage
    builder = PlantBuilder(stage, "/World/Stem")
    
    # 1. Create a tall trunk to attach branches to
    trunk_radius = 0.1
    trunk_length = 2.0
    root_id = builder.create_root("Trunk_00", radius=trunk_radius, length=trunk_length)
    
    # 8 Test Cases
    # Parameters to test:
    # Length: Long (0.8), Short (0.3)
    # Segments: Many (8), Few (2)
    # Radius (start/end): Big (0.06->0.04), Small (0.02->0.005)
    
    test_cases = [
        # length, segments, start_rad, end_rad, name
        (0.8, 8, 0.06, 0.04, "Long_Many_Big"),
        (0.8, 12, 0.01, 0.005, "Long_Many_Small"),
        (0.8, 2, 0.06, 0.04, "Long_Few_Big"),
        (0.8, 2, 0.005, 0.005, "Long_Few_Small"),
        (0.3, 8, 0.06, 0.04, "Short_Many_Big"),
        (0.3, 8, 0.005, 0.005, "Short_Many_Small"),
        (0.3, 2, 0.06, 0.04, "Short_Few_Big"),
        (0.3, 2, 0.005, 0.005, "Short_Few_Small"),
    ]
    
    for i, params in enumerate(test_cases):
        length, segments, start_rad, end_rad, name = params
        
        # Distribute branches along the trunk
        z_ratio = 0.1 + (i * 0.1) # 0.1 to 0.8
        rot_angle = i * 45.0      # Spiral around the trunk
        
        builder.add_branch(
            parent_id=root_id,
            base_id=f"Branch_{name}",
            total_length=length,
            start_radius=start_rad,
            end_radius=end_rad,
            num_segments=segments,
            z_offset_ratio=z_ratio,
            tilt_angle=45.0,
            rot_around_parent=rot_angle
        )
        print(f"Added test branch: {name}")

    # Save the active stage
    omni.usd.get_context().save_as_stage(output_path)
    print(f"[OK] Saved USD to {output_path}")
    
    print("👀 The generated plant should now be visible in the Isaac Sim window.")
    
    # Re-initialize World because save_as_stage invalidates the previous World instance
    World.clear_instance()
    world = World(stage_units_in_meters=1.0)
    world.reset()
    
    # Keep the window open until the user closes it
    while _sim_app.is_running():
        world.step(render=True)
        
    _sim_app.close()
    sys.exit(0)

if __name__ == "__main__":
    main()
