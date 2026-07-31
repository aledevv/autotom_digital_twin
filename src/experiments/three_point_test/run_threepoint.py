"""
run_threepoint.py

Isaac Sim runner for the Three-Point Bending Test.

Execution modes:
  "AUTO"      — Apply a stepped force ramp to the central link, log F and δ,
                compute structural stiffness kB via linear regression, derive E.
  "CALIBRATE" — Binary-search loop that finds E such that kB_sim ≈ kB_target.

Protocol inspired by Anisimov et al. (2025):
  - Force applied in stepped increments at the beam center.
  - Structural stiffness kB [N/m] = slope of initial linear region of F-vs-δ curve.
  - E = kB × L³ / (48 × I)

Expected deflection at F=0.5 N:
  E=35 MPa  → δ ≈ 4.51 mm
  E=150 MPa → δ ≈ 1.05 mm

Usage:
    ./python.sh run_threepoint.py
"""

import os
import sys
import csv
import math
import time
import numpy as np

# ---------------------------------------------------------------------------
# Execution mode
# ---------------------------------------------------------------------------
EXECUTION_MODE = "AUTO"   # "AUTO" | "CALIBRATE"

# Target structural stiffness for CALIBRATE mode.
# Default: derived from Anisimov primary tissue range center (E = 35 MPa).
# Replace with a value measured on a real stem sample for a reliable calibration.
KB_TARGET_N_PER_M = None   # set to None → auto-compute from BioConfig below

CALIBRATION_TOLERANCE = 0.05   # accept kB within ±5 % of target
MAX_ITERATIONS = 20

# ---------------------------------------------------------------------------
# Isaac Sim bootstrap
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": EXECUTION_MODE == "CALIBRATE"})

from pxr import UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim, Articulation

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

import generate_threepoint_usda as gen
from generate_threepoint_usda import build_stage, get_output_usd_path, TrunkConfig, BioConfig
from threepoint_theory import (
    second_moment_of_area,
    structural_stiffness,
    elastic_modulus_from_stiffness,
    theoretical_deflection,
    span_diameter_ratio,
)

USD_PATH = get_output_usd_path()

# ---------------------------------------------------------------------------
# Force protocol (step-wise ramp, inspired by Anisimov et al. 2025)
# ---------------------------------------------------------------------------
FORCE_STEPS_N = [0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.45, 0.50]
STEP_HOLD_SIM_STEPS = 120   # steps at 480 Hz ≈ 0.25 s hold per force level

# Number of initial steps to use for the linear regression (kB estimation)
N_LINEAR_POINTS = len(FORCE_STEPS_N)   # use all steps (linear elastic regime assumed)

# Force applied at the center link, downward (-Z, with gravity)
FORCE_DIRECTION = np.array([0.0, 0.0, -1.0], dtype=np.float32)

# ---------------------------------------------------------------------------
# Logging helpers
# ---------------------------------------------------------------------------

def info(msg: str):    print(f"\033[94m[INFO]\033[0m {msg}")
def warn(msg: str):    print(f"\033[93m[WARN]\033[0m {msg}")
def success(msg: str): print(f"\033[92m[OK]\033[0m {msg}")
def error(msg: str):   print(f"\033[91m[ERR]\033[0m {msg}")


# ---------------------------------------------------------------------------
# PhysX scene / articulation settings
# ---------------------------------------------------------------------------

def apply_physx_scene_settings(stage) -> None:
    scene_path = "/World/PhysicsScene"
    usd_scene  = UsdPhysics.Scene.Define(stage, scene_path)
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
    prim    = stage.GetPrimAtPath(stem_path)
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    art_api.CreateSolverPositionIterationCountAttr().Set(128)
    art_api.CreateSolverVelocityIterationCountAttr().Set(32)
    art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    art_api.CreateSleepThresholdAttr().Set(0.0)


# ---------------------------------------------------------------------------
# Main simulation function
# ---------------------------------------------------------------------------

def run_simulation_test(current_E: float) -> dict:
    """
    Run one three-point bending simulation with a given Young's modulus.

    Returns a dict with:
        kB_sim   [N/m]   — measured structural stiffness (from regression)
        E_sim    [Pa]    — derived Young's modulus
        r2                — R² of the linear fit
        forces   [N]     — force steps list
        deflections [m]  — corresponding central deflections
    """
    # Close previous stage
    try:
        omni.usd.get_context().close_stage()
    except Exception:
        pass

    gen.BioConfig.YOUNG_MODULUS = current_E

    info(f"Building stage with E = {current_E:.2e} Pa ({current_E/1e6:.1f} MPa) ...")
    stage, stem_path = build_stage(USD_PATH)
    apply_physx_scene_settings(stage)
    apply_physx_articulation_settings(stage, stem_path)
    stage.GetRootLayer().Save()

    omni.usd.get_context().open_stage(USD_PATH)

    if World.instance() is not None:
        World.instance().clear_instance()

    my_world = World(stage_units_in_meters=1.0, physics_prim_path="/World/PhysicsScene")
    stem_art  = Articulation("/World/Stem", name="stem_art")
    my_world.scene.add(stem_art)
    my_world.reset()
    stem_art.initialize()

    # Central link (0-based index → 1-based name)
    center_idx  = TrunkConfig.center_link_index()    # = 10 for N=20
    center_path = f"/World/Stem/Link_{center_idx + 1:02d}"
    center_prim = RigidPrim(center_path)
    center_prim.initialize()

    # ----- Phase 1: gravity settlement (120 steps ≈ 0.25 s at 480 Hz) -----
    info("Settling under gravity ...")
    for _ in range(120):
        my_world.step(render=(EXECUTION_MODE != "CALIBRATE"))

    pos_rest, _ = center_prim.get_world_poses()
    z_rest = float(np.squeeze(pos_rest)[2])
    info(f"Central link Z at rest: {z_rest*1000:.3f} mm")

    # ----- Phase 2: stepped force ramp -----
    info("Applying stepped force ramp ...")
    csv_path   = os.path.join(os.path.dirname(USD_PATH), "threepoint_log.csv")
    csv_file   = open(csv_path, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Step", "Force_N", "Z_center_m", "Deflection_mm"])

    forces_log      = []
    deflections_log = []

    sim_step = 0
    for force_val in FORCE_STEPS_N:
        force_vec = (FORCE_DIRECTION * force_val).reshape(1, 3)

        # Hold force for STEP_HOLD_SIM_STEPS steps, then sample the last value
        for hold_i in range(STEP_HOLD_SIM_STEPS):
            center_prim.apply_forces(forces=force_vec, is_global=True)
            my_world.step(render=(EXECUTION_MODE != "CALIBRATE"))
            sim_step += 1

        # Sample after the hold has settled
        pos_cur, _ = center_prim.get_world_poses()
        z_cur      = float(np.squeeze(pos_cur)[2])
        delta_m    = abs(z_cur - z_rest)   # deflection magnitude [m]
        delta_mm   = delta_m * 1000.0

        forces_log.append(force_val)
        deflections_log.append(delta_m)

        csv_writer.writerow([sim_step, f"{force_val:.4f}", f"{z_cur:.6f}", f"{delta_mm:.4f}"])
        info(f"  F={force_val:.3f} N  →  δ={delta_mm:.3f} mm")

    csv_file.close()
    info(f"Log saved: {csv_path}")

    # ----- Phase 3: linear regression on F-vs-δ -----
    forces_arr  = np.array(forces_log)
    deltas_arr  = np.array(deflections_log)   # [m]

    n_pts = min(N_LINEAR_POINTS, len(forces_arr))
    if n_pts < 2:
        warn("Not enough data points for regression.")
        return {"kB_sim": 0.0, "E_sim": 0.0, "r2": 0.0,
                "forces": forces_log, "deflections": deflections_log}

    coeffs    = np.polyfit(deltas_arr[:n_pts], forces_arr[:n_pts], 1)
    kB_sim    = float(coeffs[0])   # [N/m]

    # Compute R²
    y_hat = np.polyval(coeffs, deltas_arr[:n_pts])
    ss_res = np.sum((forces_arr[:n_pts] - y_hat) ** 2)
    ss_tot = np.sum((forces_arr[:n_pts] - forces_arr[:n_pts].mean()) ** 2)
    r2     = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    # Derive E from kB
    I      = second_moment_of_area(TrunkConfig.RADIUS)
    L      = TrunkConfig.total_span()
    E_sim  = elastic_modulus_from_stiffness(kB_sim, L, I)

    return {
        "kB_sim":       kB_sim,
        "E_sim":        E_sim,
        "r2":           r2,
        "forces":       forces_log,
        "deflections":  deflections_log,
    }


# ==============================================================================
# Entrypoints
# ==============================================================================

def _print_geometry_summary():
    L   = TrunkConfig.total_span()
    I   = second_moment_of_area(TrunkConfig.RADIUS)
    SDR = span_diameter_ratio(L, TrunkConfig.RADIUS)
    kB_theo = structural_stiffness(BioConfig.YOUNG_MODULUS, L, I)
    delta_theo = theoretical_deflection(FORCE_STEPS_N[-1], L, BioConfig.YOUNG_MODULUS, I)

    print("\n\033[1;36m=== Three-Point Bending Test ===\033[0m")
    print(f"  N_LINKS  = {TrunkConfig.N_LINKS}")
    print(f"  Span L   = {L*100:.1f} cm")
    print(f"  Radius   = {TrunkConfig.RADIUS*1000:.1f} mm")
    print(f"  SDR      = {SDR:.1f}  {'✅' if SDR >= 20 else '⚠️  < 20'}")
    print(f"  I        = {I:.3e} m⁴")
    print(f"  E (init) = {BioConfig.YOUNG_MODULUS/1e6:.1f} MPa")
    print(f"  kB theo  = {kB_theo:.4f} N/m")
    print(f"  δ theo (F={FORCE_STEPS_N[-1]:.2f}N) = {delta_theo*1000:.3f} mm")
    print()


if EXECUTION_MODE == "AUTO":
    _print_geometry_summary()
    result = run_simulation_test(BioConfig.YOUNG_MODULUS)
    print("\n\033[1;36m--- RESULTS ---\033[0m")
    info(f"kB measured  = {result['kB_sim']:.4f} N/m")
    info(f"E derived    = {result['E_sim']/1e6:.2f} MPa")
    info(f"R²           = {result['r2']:.4f}")
    I = second_moment_of_area(TrunkConfig.RADIUS)
    L = TrunkConfig.total_span()
    kB_theo = structural_stiffness(BioConfig.YOUNG_MODULUS, L, I)
    info(f"kB theory    = {kB_theo:.4f} N/m  (for E={BioConfig.YOUNG_MODULUS/1e6:.0f} MPa)")
    ratio = result['kB_sim'] / kB_theo if kB_theo > 0 else 0
    info(f"sim/theory   = {ratio:.3f}  (1.0 = perfect match)")

elif EXECUTION_MODE == "CALIBRATE":
    I = second_moment_of_area(TrunkConfig.RADIUS)
    L = TrunkConfig.total_span()

    # Default target: kB from literature E=35 MPa
    if KB_TARGET_N_PER_M is None:
        kb_target = structural_stiffness(3.5e7, L, I)
        warn(f"KB_TARGET_N_PER_M not set. Using literature default: {kb_target:.4f} N/m (E=35 MPa)")
    else:
        kb_target = KB_TARGET_N_PER_M

    _print_geometry_summary()
    info(f"=== CALIBRATING: kB target = {kb_target:.4f} N/m ===\n")

    # Binary search on E
    E_lo, E_hi = 1e6, 5e8   # 1 MPa to 500 MPa
    current_E  = BioConfig.YOUNG_MODULUS

    for iteration in range(MAX_ITERATIONS):
        info(f"--- Iteration {iteration + 1} / {MAX_ITERATIONS}  (E = {current_E/1e6:.2f} MPa) ---")
        result = run_simulation_test(current_E)
        kB_sim = result["kB_sim"]
        info(f"kB_sim = {kB_sim:.4f} N/m  (target = {kb_target:.4f} N/m)")

        rel_error = abs(kB_sim - kb_target) / kb_target if kb_target > 0 else 1.0
        if rel_error <= CALIBRATION_TOLERANCE:
            success(f"Converged in {iteration + 1} iterations!")
            success(f"E = {current_E/1e6:.2f} MPa  (kB error = {rel_error*100:.1f} %)")
            break

        # Proportional update: kB ∝ E → scale E by ratio of targets
        if kB_sim > 0:
            ratio = kb_target / kB_sim
            safe_ratio = max(0.2, min(ratio, 5.0))
            current_E  = max(E_lo, min(E_hi, current_E * safe_ratio))
        else:
            error("kB_sim = 0. Calibration failed (possible instability).")
            break
    else:
        warn(f"Max iterations reached. Best E = {current_E/1e6:.2f} MPa")

simulation_app.close()
