"""
load_stem_v2.py

Loader for the Phase-1 articulated stem (usd_exporterV2).
Pattern identical to load_articulation_subbranch.py:
  1. Generate stage via export_stem_articulated_usd()
  2. Inject PhysxSchema config (only possible inside SimulationApp)
  3. Save and open in Isaac Sim
  4. Run simulation loop

Run with:
  ~/isaacsim/python.sh src/plant_model/load_stem_v2.py
"""

import os
import sys

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
SRC_DIR      = os.path.dirname(SCRIPT_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
import omni.kit.actions.core

from plant_model.loader import load_snapshot
from plant_model.usd_exporterV2 import export_stem_articulated_usd

# ── Config ───────────────────────────────────────────────────────────────────
DAY       = 160
PLANT_ID  = 1
CSV_PATH  = os.path.join(PROJECT_ROOT, f"data/simulation_output/dynamic_output/graphs/graph_day_{DAY}.csv")
USD_PATH  = os.path.join(PROJECT_ROOT, f"./output/day_{DAY}/plant_day{DAY}_v2.usda")


def apply_physx_scene(stage) -> None:
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    api = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    api.CreateSolverTypeAttr().Set("TGS")
    api.CreateTimeStepsPerSecondAttr().Set(120)
    api.CreateEnableCCDAttr().Set(True)
    api.CreateEnableStabilizationAttr().Set(True)
    api.CreateEnableGPUDynamicsAttr().Set(True)
    api.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation(stage, stem_path: str) -> None:
    stem_prim = stage.GetPrimAtPath(stem_path)
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    art_api.CreateSolverPositionIterationCountAttr().Set(64)
    art_api.CreateSolverVelocityIterationCountAttr().Set(8)
    art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    art_api.CreateSleepThresholdAttr().Set(0.0)


# ── 1. Generate USD stage ─────────────────────────────────────────────────────
print(f"[INFO] Loading snapshot day={DAY} plant={PLANT_ID}...")
snapshot = load_snapshot(CSV_PATH, day=DAY, plant_id=PLANT_ID)

print("[INFO] Exporting articulated stem V2...")
export_stem_articulated_usd(snapshot, USD_PATH)

# ── 2. Inject PhysX config ────────────────────────────────────────────────────
import omni.usd as _ousd
_ousd.get_context().open_stage(USD_PATH)
stage = _ousd.get_context().get_stage()

apply_physx_scene(stage)
apply_physx_articulation(stage, "/World/Stem")

# ── 3. Save with PhysX config ─────────────────────────────────────────────────
stage.GetRootLayer().Save()
print(f"[OK] Stage saved with PhysX config: {USD_PATH}")

# ── 4. Reload and simulate ───────────────────────────────────────────────────
omni.usd.get_context().open_stage(USD_PATH)
print(f"[OK] Stage opened in Isaac Sim")

try:
    reg = omni.kit.actions.core.get_action_registry()
    act = reg.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
    if act:
        act.execute()
except Exception as e:
    print(f"[WARN] Lighting: {e}")

world = World(stage_units_in_meters=1.0)
world.reset()
print("[OK] Simulation running — close window to exit.")

while simulation_app.is_running():
    world.step(render=True)

print("Done.")
simulation_app.close()