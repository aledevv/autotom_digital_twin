"""
load_subbranch_test.py

Single entry point: generates the USD stage with subbranches,
applies the PhysX configuration, and starts the simulation in Isaac Sim.
"""

import os
import sys
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
import omni.kit.actions.core

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from generate_subbranch_articulation import build_stage, get_output_usd_path

USD_PATH = get_output_usd_path()


def apply_physx_scene_settings(stage) -> None:
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx_scene_api.CreateSolverTypeAttr().Set("TGS")
    physx_scene_api.CreateTimeStepsPerSecondAttr().Set(120)
    physx_scene_api.CreateEnableCCDAttr().Set(True)
    physx_scene_api.CreateEnableStabilizationAttr().Set(True)
    physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    physx_scene_api.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    stem_prim = stage.GetPrimAtPath(stem_path)
    physx_art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    physx_art_api.CreateSolverPositionIterationCountAttr().Set(64)
    physx_art_api.CreateSolverVelocityIterationCountAttr().Set(8)
    physx_art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    physx_art_api.CreateSleepThresholdAttr().Set(0.0)


print("[INFO] Building stage via generate_subbranch_articulation...")
stage, stem_path, _ = build_stage(USD_PATH)

apply_physx_scene_settings(stage)
apply_physx_articulation_settings(stage, stem_path)

stage.GetRootLayer().Save()
print(f"[OK] Stage generated and saved with PhysX config: {USD_PATH}")

omni.usd.get_context().open_stage(USD_PATH)
print(f"[OK] Stage opened in Isaac Sim: {USD_PATH}")

try:
    action_registry = omni.kit.actions.core.get_action_registry()
    action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    if action:
        action.execute()
except Exception as e:
    print(f"[WARN] Lighting not set: {e}")

my_world = World(stage_units_in_meters=1.0)
my_world.reset()
print("[OK] Simulation started — close the window to exit.")

while simulation_app.is_running():
    my_world.step(render=True)

print("Simulation finished.")
simulation_app.close()
