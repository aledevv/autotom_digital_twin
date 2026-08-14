
from omni.isaac.kit import SimulationApp
config = {"headless": False, "width": 1920, "height": 1080}
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
    physx_scene_api.CreateSolverPositionIterationCountAttr(128)
    physx_scene_api.CreateSolverVelocityIterationCountAttr(16)

    carb.log_info(f"Solver settings: positionIterations={pos_iter}, velocityIterations={vel_iter}")
    print(f"\n================================================================================")
    print(f"SOLVER SETTINGS:")
    print(f"  Position iterations: 128")
    print(f"  Velocity iterations: 16")
    print(f"================================================================================\n")

# Load USD
tree_prim = stage.DefinePrim("/World/tomato", "Xform")
tree_prim.GetReferences().AddReference("/home/alessandro/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/scalability_usds/solver_medium_pos128_vel16.usda")

world.reset()

print("\n" + "="*80)
print("Config loaded! Press PLAY to test, then CLOSE Isaac Sim when done.")
print("="*80 + "\n")

# Run until user closes
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
