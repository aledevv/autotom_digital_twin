"""
load_tomato_branch.py

Single entry point: generates the USD stage (from generate_tomato_branch_usda.py),
applies the PhysX configuration, and starts the simulation in Isaac Sim.

Run with:
~/isaacsim/python.sh src/experiments/articulation_subbranch/load_tomato_branch.py
"""

import os
import sys
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
import omni.kit.actions.core
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from generate_tomato_branch_usda import build_stage, get_output_usd_path

USD_PATH = get_output_usd_path()


def apply_physx_scene_settings(stage) -> None:
    """Creates/configures PhysicsScene with PhysX parameters suitable for stiff drives."""
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
    """Configures iteration count on the ArticulationRoot for stability with stiff drives."""
    stem_prim = stage.GetPrimAtPath(stem_path)
    physx_art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    physx_art_api.CreateSolverPositionIterationCountAttr().Set(64)
    physx_art_api.CreateSolverVelocityIterationCountAttr().Set(8)
    physx_art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    physx_art_api.CreateSleepThresholdAttr().Set(0.0)


# ---------------------------------------------------------------------------
# 1. Generate the in-memory stage using pure functions from generate_tomato_branch_usda
# ---------------------------------------------------------------------------
print("[INFO] Building stage via generate_tomato_branch_usda...")
stage, stem_path = build_stage(USD_PATH)

# ---------------------------------------------------------------------------
# 2. Inject PhysX configuration (only available here, inside SimulationApp)
# ---------------------------------------------------------------------------
apply_physx_scene_settings(stage)
apply_physx_articulation_settings(stage, stem_path)

# ---------------------------------------------------------------------------
# 3. Save the complete file (geometry + physics + PhysX) for reuse/debugging
# ---------------------------------------------------------------------------
stage.GetRootLayer().Save()
print(f"[OK] Stage generated and saved with PhysX config: {USD_PATH}")

# ---------------------------------------------------------------------------
# 4. Open the stage in the Isaac Sim context and start the simulation
# ---------------------------------------------------------------------------
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

# Inizializza l'articolazione
stem_articulation = Articulation("/World/Stem", name="stem_articulation")
my_world.scene.add(stem_articulation)

my_world.reset()
print("[OK] Simulation started — close the window to exit.")

step_counter = 0

while simulation_app.is_running():
    my_world.step(render=True)
    
    step_counter += 1
    # Leggiamo le forze ogni 60 frame (circa ogni 0.5s a 120Hz)
    if step_counter % 60 == 0:
        forces = stem_articulation.get_link_incoming_joint_force()
        if forces is not None and len(forces) > 0:
            # 'forces' restituisce un array 6D [Fx, Fy, Fz, Tx, Ty, Tz] per ogni link
            # forces[0] è solitamente la root base
            torque_base = forces[0][3:] # Momento torcente/flettente
            torque_mag = np.linalg.norm(torque_base)
            
            # Formatta l'output per essere leggibile
            tx, ty, tz = [round(float(v), 2) for v in torque_base]
            print(f"[Step {step_counter:04d}] Coppia base: [{tx}, {ty}, {tz}] Nm | Magnitudo: {torque_mag:.2f} Nm")

print("Simulation finished.")
simulation_app.close()
