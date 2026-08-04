"""
main.py - exporterV2 Entry Point

Generates tree USD and loads it in Isaac Sim in one step.

Run with:
    ~/isaacsim/python.sh src/exporterV2/main.py
or:
    ./run_exporterV2.sh
"""

import os
import sys

# Bootstrap Isaac Sim
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Now import USD and Isaac modules
from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
import omni.kit.actions.core
from isaacsim.core.api import World

# Import our modules (need to add parent to path)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

from exporterV2.generate_tree import build_stage, get_output_usd_path
from exporterV2.tree_config import BRANCHES

USD_PATH = get_output_usd_path()


# ==============================================================================
# PHYSX CONFIGURATION
# ==============================================================================

def apply_physx_scene_settings(stage) -> None:
    """PhysicsScene tuned for stiff articulation drives."""
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(480)
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateEnableGPUDynamicsAttr().Set(True)
    physx.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    """Iteration counts for articulation with mixed stiffness levels."""
    prim = stage.GetPrimAtPath(stem_path)
    art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)



# ==============================================================================
# MAIN
# ==============================================================================

def main():
    """Generate tree USD and load in Isaac Sim."""
    print("=" * 80)
    print("  exporterV2 - Tree Model Generator & Loader")
    print("=" * 80)
    
    # Step 1: Generate USD
    print("\n[STEP 1/3] Generating tree USD stage...")
    total_links = sum(b["n_links"] for b in BRANCHES)
    print(f"  Configuration: {len(BRANCHES)} branches, {total_links} total links")
    
    stage, stem_path = build_stage(USD_PATH)
    
    # Step 2: Apply PhysX settings
    print("\n[STEP 2/3] Applying PhysX configuration...")
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)
    
    stage.GetRootLayer().Save()
    print(f"  ✓ Stage saved: {USD_PATH}")
    
    # Step 3: Load in Isaac Sim
    print("\n[STEP 3/3] Loading in Isaac Sim...")
    omni.usd.get_context().open_stage(USD_PATH)
    print("  ✓ Stage opened")
    
    # Set camera lighting mode
    try:
        reg = omni.kit.actions.core.get_action_registry()
        action = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
        if action:
            action.execute()
            print("  ✓ Camera lighting applied")
    except Exception as e:
        print(f"  ⚠ Lighting action not available: {e}")
    
    # Initialize simulation
    my_world = World(stage_units_in_meters=1.0)
    my_world.reset()
    
    print("\n" + "=" * 80)
    print("  ✓ Simulation running — close the window to exit")
    print("=" * 80 + "\n")
    
    # Run simulation loop
    while simulation_app.is_running():
        my_world.step(render=True)
    
    print("\n[INFO] Simulation finished.")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user.")
        simulation_app.close()
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        simulation_app.close()
        sys.exit(1)
