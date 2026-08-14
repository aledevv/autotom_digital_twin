
from omni.isaac.kit import SimulationApp
config = {"headless": False, "width": 1920, "height": 1080}
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
tree_prim.GetReferences().AddReference("/home/alessandro/isaacsim/autotom_digital_twin/src/experiments/recursive_tree/tests/scalability_usds/test2_stem_1petiole_DAMPING_FIX.usda")

world.reset()

print("\n" + "="*80)
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
print("="*80 + "\n")

# Run until user closes
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
