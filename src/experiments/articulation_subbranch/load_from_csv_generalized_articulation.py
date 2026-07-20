"""
load_from_csv_generalized_articulation.py

Variant loader: Reads the CSV configuration (generated previously) and deterministically
builds the USD stage from that CSV without randomization, then starts Isaac Sim.

Run with:
~/isaacsim/python.sh src/experiments/articulation_subbranch/load_from_csv_generalized_articulation.py
"""

import os
import sys
import csv
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
import omni.kit.actions.core

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from generate_generalized_articulation_usda import build_stage_from_csv_data, get_output_usd_path

USD_PATH = get_output_usd_path()
CSV_PATH = USD_PATH.replace(".usda", "_config.csv")

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
# 1. Load the CSV configuration
# ---------------------------------------------------------------------------
if not os.path.exists(CSV_PATH):
    print(f"[ERROR] CSV config not found at {CSV_PATH}. Please run the standard generator first.")
    simulation_app.close()
    sys.exit(1)

print(f"[INFO] Loading configuration from {CSV_PATH}...")
csv_data = []
with open(CSV_PATH, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        csv_data.append(row)

# ---------------------------------------------------------------------------
# 2. Generate the in-memory stage from the loaded CSV
# ---------------------------------------------------------------------------
print("[INFO] Building deterministic stage from CSV...")
stage, stem_path = build_stage_from_csv_data(USD_PATH, csv_data)

# ---------------------------------------------------------------------------
# 3. Inject PhysX configuration
# ---------------------------------------------------------------------------
apply_physx_scene_settings(stage)
apply_physx_articulation_settings(stage, stem_path)

# ---------------------------------------------------------------------------
# 4. Save and Open the stage
# ---------------------------------------------------------------------------
stage.GetRootLayer().Save()
print(f"[OK] Deterministic stage generated and saved with PhysX config: {USD_PATH}")

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
