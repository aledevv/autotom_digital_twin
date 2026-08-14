
from omni.isaac.kit import SimulationApp
config = {"headless": False, "width": 1920, "height": 1080}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
import carb

# Create world
world = World(stage_units_in_meters=1.0, physics_dt=1/480.0, rendering_dt=1/60.0)
world.scene.add_default_ground_plane()

# Load USD
stage = world.stage
tree_prim = stage.DefinePrim("/World/tomato", "Xform")
tree_prim.GetReferences().AddReference("/home/alessandro/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/scalability_usds/test2_stem_1petiole_LOCKED.usda")

world.reset()

print("\n" + "="*80)
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
print("="*80 + "\n")

# Run until user closes
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
