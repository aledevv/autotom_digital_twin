import os
import sys
import numpy as np
import time
import csv

# Scegli la modalità di esecuzione:
# "CALIBRATE" -> Ricerca in automatico (headless) il YOUNG_MODULUS per la deflessione target
# "AUTO"      -> Applica la forza automaticamente per 3s e poi la rilascia (con GUI)
# "MANUAL"    -> Nessuna forza automatica, usa il mouse (Shift + Click) (con GUI)
EXECUTION_MODE = "AUTO"

TARGET_DEFLECTION_MM = 7.6
CALIBRATION_TOLERANCE_MM = 0.1
MAX_ITERATIONS = 15

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": EXECUTION_MODE == "CALIBRATE"})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim, Articulation
import omni.kit.actions.core

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import generate_cantilever_usda
from generate_cantilever_usda import build_stage, get_output_usd_path

USD_PATH = get_output_usd_path()

def print_info(msg: str):
    print(f"\033[94m[INFO]\033[0m {msg}")

def print_warning(msg: str):
    print(f"\033[93m[WARNING]\033[0m {msg}")

def print_error(msg: str):
    print(f"\033[91m[ERROR]\033[0m {msg}")

def print_success(msg: str):
    print(f"\033[92m[SUCCESS]\033[0m {msg}")


def apply_physx_scene_settings(stage) -> None:
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx_scene_api.CreateSolverTypeAttr().Set("TGS")
    physx_scene_api.CreateTimeStepsPerSecondAttr().Set(480)
    physx_scene_api.CreateEnableCCDAttr().Set(True)
    physx_scene_api.CreateEnableStabilizationAttr().Set(True)
    physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    physx_scene_api.CreateBroadphaseTypeAttr().Set("MBP")

def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    stem_prim = stage.GetPrimAtPath(stem_path)
    physx_art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    physx_art_api.CreateSolverPositionIterationCountAttr().Set(128)
    physx_art_api.CreateSolverVelocityIterationCountAttr().Set(32)
    physx_art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    physx_art_api.CreateSleepThresholdAttr().Set(0.0)

def run_simulation_test(current_E: float) -> float:
    # Close stage in Isaac Sim to remove physical constraints from memory.
    try:
        omni.usd.get_context().close_stage()
    except Exception:
        pass

    
    # Aggiorna il modulo di Young per la generazione
    generate_cantilever_usda.BioConfig.YOUNG_MODULUS = current_E
    
    print_info(f"Building stage via generate_cantilever_usda with E = {current_E:.2e} ...")
    stage, stem_path = build_stage(USD_PATH)
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)
    stage.GetRootLayer().Save()

    omni.usd.get_context().open_stage(USD_PATH)
    
    # Assicurati di non avere residui nel World se ricreato
    if World.instance() is not None:
        World.instance().clear_instance()
        
    my_world = World(stage_units_in_meters=1.0, physics_prim_path="/World/PhysicsScene")
    stem_articulation = Articulation("/World/Stem", name="stem_articulation")
    my_world.scene.add(stem_articulation)

    my_world.reset()
    stem_articulation.initialize()

    # Seleziona l'ultimo link della catena (la punta Trunk_10)
    tip_path = "/World/Stem/Trunk_10"
    tip_prim = RigidPrim(tip_path)
    tip_prim.initialize()

    # Setup CSV Logging
    csv_path = os.path.join(os.path.dirname(USD_PATH), "cantilever_log.csv")
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Step", "Time", "X_Pos_m", "Deflection_mm", "Force_N"])

    pos_initial, _ = tip_prim.get_world_poses()
    x_initial = np.squeeze(pos_initial)[0]

    print_info("Stabilizzazione gravità per 1 secondo...")
    for _ in range(120): 
        my_world.step(render=not (EXECUTION_MODE == "CALIBRATE"))

    pos_rest, _ = tip_prim.get_world_poses()
    x_rest = np.squeeze(pos_rest)[0]
    print_info(f"X Rest: {x_rest:.6f} m")

    AUTO_FORCE_DURATION = 3.0
    RELEASE_DURATION = 3.0

    force_magnitude = 0.5

    start_time = time.time()
    step = 0
    simulation_running = True
    steady_deflections = []

    if EXECUTION_MODE == "MANUAL":
        print_info("Forza automatica DISATTIVATA. Tieni premuto SHIFT e tira Trunk_10 con il mouse!")
        print_info("Chiudi la finestra per terminare la simulazione e salvare il grafico.")
    else:
        print_info(f"Applicazione forza {force_magnitude} N in corso automatica...")

    while simulation_app.is_running() and simulation_running:
        current_time = time.time() - start_time
        applied_force_val = 0.0
        
        if EXECUTION_MODE in ["AUTO", "CALIBRATE"]:
            RAMP_TIME = 1.0  # Tempo di salita della forza
            
            if current_time < AUTO_FORCE_DURATION:
                scale = min(1.0, current_time / RAMP_TIME)
                current_force_val = force_magnitude * scale
                
                force_vec = np.array([[current_force_val, 0.0, 0.0]], dtype=np.float32)
                applied_force_val = current_force_val
                
                tip_prim.apply_forces(forces=force_vec, is_global=True)
                
            elif current_time >= AUTO_FORCE_DURATION + RELEASE_DURATION:
                simulation_running = False
                continue

        my_world.step(render=not (EXECUTION_MODE == "CALIBRATE"))
        
        pos_current, _ = tip_prim.get_world_poses()
        x_current = np.squeeze(pos_current)[0]
        
        try:
            joint_forces = stem_articulation.get_measured_joint_forces()
            mags = np.linalg.norm(joint_forces[0, :, :3], axis=-1)
            max_force = np.max(mags)
        except:
            max_force = 0.0
            
        recorded_force = applied_force_val if EXECUTION_MODE in ["AUTO", "CALIBRATE"] else max_force
            
        deflection_m = abs(x_current - x_rest)
        deflection_mm = deflection_m * 1000.0
        
        if EXECUTION_MODE in ["AUTO", "CALIBRATE"] and 2.0 <= current_time <= 2.9:
            steady_deflections.append(deflection_mm)
        elif EXECUTION_MODE == "MANUAL":
            # In manuale teniamo traccia di tutto per sicurezza
            steady_deflections.append(deflection_mm)
        
        csv_writer.writerow([step, f"{current_time:.4f}", f"{x_current:.6f}", f"{deflection_mm:.4f}", f"{recorded_force:.6f}"])
        step += 1

    csv_file.close()

    if steady_deflections:
        final_deflection = sum(steady_deflections) / len(steady_deflections)
    else:
        final_deflection = 0.0
        
    return final_deflection

if EXECUTION_MODE == "CALIBRATE":
    print_info(f"\n=== INIZIO CALIBRAZIONE STIFFNESS (Target: {TARGET_DEFLECTION_MM} mm) ===")
    current_E = 1.5e8  # Punto di partenza
    
    for i in range(MAX_ITERATIONS):
        print_info(f"\n--- Iterazione {i+1} ---")
        measured_deflection = run_simulation_test(current_E)
        print_info(f"Deflessione misurata: {measured_deflection:.4f} mm")
        
        # Filtro anti-esplosione: se la misurazione sballa oltre i 500mm, ignoriamo il dato grezzo
        if measured_deflection > 500.0:
            print_info("[WARNING] Possibile instabilità rilevata. Riduco lo step.")
            measured_deflection = 200.0 
            
        error = abs(measured_deflection - TARGET_DEFLECTION_MM)
        if error <= CALIBRATION_TOLERANCE_MM:
            print_info(f"[SUCCESSO] Calibrazione completata in {i+1} iterazioni!")
            print_info(f"Il valore ideale per YOUNG_MODULUS è: {current_E:.2e} ({current_E})")
            break
            
        if measured_deflection > 0:
            ratio = measured_deflection / TARGET_DEFLECTION_MM
            
            # MAGIA VELOCE: Ora saltiamo fino a 5 volte la rigidità precedente!
            safe_ratio = max(0.2, min(ratio, 5.0)) 
            current_E = current_E * safe_ratio
        else:
            print("[ERRORE] Deflessione zero, calibrazione fallita.")
            break
            
else:
    # Esecuzione standard singola
    current_E = 1.5e8  # O il valore fisso desiderato
    measured_deflection = run_simulation_test(current_E)
    print("\033[1;36m\n--- RISULTATI BENCHMARK ---\033[0m")
    print_info(f"Deflessione MAX da Forza: {measured_deflection:.2f} mm")
    print_info(f"Atteso da Letteratura: ~{TARGET_DEFLECTION_MM} mm")
    
simulation_app.close()