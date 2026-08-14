import os
import sys

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from generate_cantilever_usda import build_stage, get_output_usd_path

USD_PATH = get_output_usd_path()

print("[INFO] Building stage via generate_cantilever_usda...")
stage, stem_path = build_stage(USD_PATH)

# Apply settings
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

stem_prim = stage.GetPrimAtPath(stem_path)
physx_art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
physx_art_api.CreateSolverPositionIterationCountAttr().Set(64)
physx_art_api.CreateSolverVelocityIterationCountAttr().Set(8)
physx_art_api.CreateEnabledSelfCollisionsAttr().Set(False)
physx_art_api.CreateSleepThresholdAttr().Set(0.0)

stage.GetRootLayer().Save()
omni.usd.get_context().open_stage(USD_PATH)

my_world = World(stage_units_in_meters=1.0, physics_prim_path="/World/PhysicsScene")
my_world.reset()

import numpy as np
from isaacsim.core.prims import RigidPrim

print("=====================================================")
print("✅ SIMULAZIONE AUTOMATICA AVVIATA (Cantilever Test)")
print("=====================================================")

# 1. Inizializza il prim della punta
tip_prim = RigidPrim(prim_path="/World/Stem/Trunk_10")
tip_prim.initialize()

# 2. Definisci la forza (es. 0.5 Newton diretti verso il basso -Z)
force_to_apply = np.array([0.0, 0.0, -0.5])

print("Fase 1: Assestamento gravità (2 secondi)...")
for _ in range(240):
    my_world.step(render=True)

print("Fase 2: Applicazione FORZA COSTANTE 0.5 N (3 secondi)...")
for _ in range(360):
    # FONDAMENTALE: Bisogna ri-applicare la forza ad OGNI frame PRIMA dello step!
    try:
        tip_prim.apply_forces(forces=force_to_apply)
    except:
        tip_prim.apply_forces(forces=np.array([force_to_apply]))
    my_world.step(render=True)

print("Fase 3: Rilascio forza e smorzamento (3 secondi)...")
for _ in range(360):
    # Non chiamiamo apply_forces, la forza torna spontaneamente a 0
    my_world.step(render=True)

simulation_app.close()
