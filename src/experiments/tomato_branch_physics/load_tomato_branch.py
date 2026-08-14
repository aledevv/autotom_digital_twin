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
import csv
import time

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

my_world = World(stage_units_in_meters=1.0, physics_prim_path="/World/PhysicsScene")

# Inizializza l'articolazione
stem_articulation = Articulation("/World/Stem", name="stem_articulation")
my_world.scene.add(stem_articulation)

my_world.reset()
stem_articulation.initialize()
print("[OK] Simulation started — close the window to exit.")

step_counter = 0

# --- Diagnostica una tantum dopo initialize() ---
print("[DEBUG] body_names:", stem_articulation.body_names)
print("[DEBUG] num_bodies:", stem_articulation.num_bodies)
try:
    print("[DEBUG] masses:", stem_articulation.get_body_masses())
except Exception as e:
    print("[DEBUG] get_body_masses fallito:", e)

step_counter = 0
found_nonzero = False

# Setup CSV Logging
csv_path = os.path.join(os.path.dirname(USD_PATH), "forces_log.csv")
print(f"[INFO] Logging forces to {csv_path}")
csv_file = open(csv_path, "w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["Step", "Time", "JointName", "Fx", "Fy", "Fz", "Tx", "Ty", "Tz", "F_norm"])

start_time = time.time()

while simulation_app.is_running():
    my_world.step(render=True)
    step_counter += 1

    if step_counter % 10 == 0:  # controlla più spesso per non perdere transitori
        try:
            joint_forces = stem_articulation.get_measured_joint_forces()  # (envs, links, 6)
        except Exception as e:
            print(f"[Step {step_counter:04d}] Errore lettura: {e}")
            continue

        # Scansiona TUTTI i link, non solo il link 0
        mags = np.linalg.norm(joint_forces[0, :, :3], axis=-1)  # forza per link
        nonzero_idx = np.where(mags > 1e-6)[0]

        if len(nonzero_idx) > 0:
            found_nonzero = True
            current_time = time.time() - start_time
            for idx in nonzero_idx:
                name = stem_articulation.body_names[idx] if idx < len(stem_articulation.body_names) else f"idx{idx}"
                fx, fy, fz, tx, ty, tz = joint_forces[0, idx, :]
                fnorm = mags[idx]
                
                # Scrivi su CSV
                csv_writer.writerow([step_counter, f"{current_time:.4f}", name, 
                                     f"{fx:.6f}", f"{fy:.6f}", f"{fz:.6f}", 
                                     f"{tx:.6f}", f"{ty:.6f}", f"{tz:.6f}", f"{fnorm:.6f}"])
                
                # Stampa a video
                # (arrotonda solo per il print)
                fx_r, fy_r, fz_r, tx_r, ty_r, tz_r = [round(float(v), 4) for v in joint_forces[0, idx, :]]
                print(f"[Step {step_counter:04d}] {name}: F=[{fx_r},{fy_r},{fz_r}] T=[{tx_r},{ty_r},{tz_r}]")
        elif step_counter % 300 == 0:
            # ogni 2.5s, se ancora tutto zero, stampa diagnostica extra
            print(f"[Step {step_counter:04d}] Ancora tutto zero. "
                  f"max forza assoluta rilevata: {mags.max():.6e} | "
                  f"velocità giunti: {stem_articulation.get_joint_velocities()}")

print(f"[FINE] Forze non-zero mai rilevate: {not found_nonzero}")

csv_file.close()
print(f"[OK] Log salvato in {csv_path}")

print("Simulation finished.")
simulation_app.close()
