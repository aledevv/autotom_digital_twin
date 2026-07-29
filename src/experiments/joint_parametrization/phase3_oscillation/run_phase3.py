"""
Phase 3 — Dynamic Oscillation Test
=====================================
Verifies that the critical-damping formula produces a consistent
settling time as the number of articulation segments N changes.

Protocol:
  1. Build a horizontal cantilever (same as Phase 1, physics stiffness formula).
  2. Let it settle under gravity for 1 second.
  3. Apply an instantaneous impulse (velocity perturbation) to the tip.
  4. Record tip Z position every sim step for 5 more seconds.
  5. Detect settling time = first time |z - z_settled| < threshold.
  6. Compare settling time across N = 2, 3, 5, 10.

Expected outcome: with the critical-damping formula (D = 2√(K·m)),
the settling time should remain approximately constant regardless of N.

Run headless:
    ~/isaacsim/python.sh src/experiments/joint_parametrization/phase3_oscillation/run_phase3.py

Or via the launcher:
    ./run_experiment.sh exp3
"""

import os
import sys
import math
import json
import subprocess

# ── Path bootstrap ────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
EXP_DIR      = os.path.dirname(SCRIPT_DIR)
EXPERIMENTS  = os.path.dirname(EXP_DIR)
SRC_DIR      = os.path.dirname(EXPERIMENTS)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# ── Experiment parameters ─────────────────────────────────────────────────────
HEADLESS      = True

L             = 0.4
RADIUS        = 0.01
E             = 1.0e6
DENSITY       = 700.0
DAMPING_RATIO = 0.7

SCALE         = 10.0
GRAVITY       = 9.81
SIM_HZ        = 120

SETTLE_STEPS  = int(SIM_HZ * 1.0)   # 1 s to settle under gravity first
IMPULSE_VZ    = -2.0 * SCALE         # downward velocity impulse on tip [sim units/s]
TRACK_STEPS   = int(SIM_HZ * 5.0)   # 5 s tracking window after impulse
SETTLE_THRESH = 0.02 * SCALE         # settling threshold [sim units] ≈ 2mm real

N_VALUES      = [2, 3, 5, 10]

RESULTS_DIR   = os.path.join(SCRIPT_DIR, "results")

# ── Isaac Sim bootstrap ───────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf
import omni.usd
from isaacsim.core.api import World

from plant_model.v2.plant_builder import PlantBuilder
from plant_model.v2.plant_builder_utils import _auto_mass, _critical_damping


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def physics_stiffness(E, r, N, L):
    I = math.pi * r**4 / 4.0
    l = L / N
    return E * I / l


def setup_physics_scene(stage):
    sc_path = "/World/PhysicsScene"
    if not stage.GetPrimAtPath(sc_path):
        sc = UsdPhysics.Scene.Define(stage, sc_path)
    else:
        sc = UsdPhysics.Scene(stage.GetPrimAtPath(sc_path))
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(GRAVITY)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(SIM_HZ)
    px.CreateEnableStabilizationAttr().Set(True)


def get_world_z(stage, seg_id):
    path = f"/World/Plant/{seg_id}"
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"Prim not found: {path}")
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return mat.ExtractTranslation()[2]


def apply_velocity_impulse(stage, seg_id, vz: float):
    """
    Apply a velocity impulse by directly setting the rigid body's
    linear velocity attribute. This is the simplest approach without
    needing force-over-time application.
    """
    path = f"/World/Plant/{seg_id}"
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"Prim not found for impulse: {path}")

    rb_api = UsdPhysics.RigidBodyAPI(prim)
    if rb_api:
        rb_api.CreateVelocityAttr().Set(Gf.Vec3f(0.0, 0.0, float(vz)))


def build_cantilever(stage, N):
    segment_len = L / N
    r = RADIUS
    stiff = physics_stiffness(E, r, N, L)
    r_sc  = r * SCALE
    l_sc  = segment_len * SCALE
    mass  = _auto_mass(r_sc, l_sc, DENSITY)
    damp  = DAMPING_RATIO * _critical_damping(stiff, mass)

    builder = PlantBuilder(stage, base_path="/World/Plant", scale=SCALE)
    root_segs = [{"order": 0, "rank": 0, "radius": r, "length": 0.02}]
    builder.add_main_stem_segments("stem", root_segs, physics=False)

    anchor_path = "/World/Plant/Internode_o0_r0"
    anchor_prim = stage.GetPrimAtPath(anchor_path)
    UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    UsdPhysics.MassAPI.Apply(anchor_prim).CreateMassAttr().Set(1000.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{anchor_path}/WorldFixedJoint")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(anchor_path)])

    trunk_id = "Internode_o0_r0"
    seg_ids = []
    for i in range(N):
        seg_id = f"Branch/Seg_{i:02d}"
        seg_ids.append(seg_id)
        if i == 0:
            builder.add_lateral_branch(
                parent_id=trunk_id, id=seg_id,
                radius=r, length=segment_len,
                z_offset_ratio=0.5, tilt_angle=90.0, rot_around_parent=0.0,
                density=DENSITY, stiffness=stiff, damping_ratio=DAMPING_RATIO,
                max_bend_angle=60.0, twist_limit=5.0, physics=True,
            )
        else:
            builder.add_internode(
                parent_id=seg_ids[i - 1], id=seg_id,
                radius=r, length=segment_len,
                density=DENSITY, stiffness=stiff, damping_ratio=DAMPING_RATIO,
                max_bend_angle=60.0, physics=True,
            )

    return anchor_path, seg_ids[-1]


def find_settling_time(trajectory: list[float], settled_z: float, threshold: float, dt: float) -> float | None:
    """
    Return the first time (seconds) where |z - settled_z| < threshold
    and stays below for at least 10 consecutive steps.
    Returns None if never settled within the tracking window.
    """
    consecutive_needed = 10
    count = 0
    for i, z in enumerate(trajectory):
        if abs(z - settled_z) < threshold:
            count += 1
            if count >= consecutive_needed:
                return (i - consecutive_needed + 1) * dt
        else:
            count = 0
    return None


def run_single_n(N: int) -> dict:
    print(f"\n{'='*60}")
    print(f"  Phase 3 — N = {N} segments")
    print(f"{'='*60}")

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    setup_physics_scene(stage)
    anchor_path, tip_id = build_cantilever(stage, N)

    world = World(stage_units_in_meters=1.0)
    world.reset()

    # Phase A: let settle under gravity
    print(f"  Settling under gravity ({SETTLE_STEPS} steps)...")
    for _ in range(SETTLE_STEPS):
        world.step(render=not HEADLESS)

    z_settled = get_world_z(stage, tip_id)
    print(f"  Settled tip Z = {z_settled:.4f} (sim units)")

    # Phase B: apply impulse
    print(f"  Applying impulse Vz={IMPULSE_VZ:.2f} ...")
    apply_velocity_impulse(stage, tip_id, IMPULSE_VZ)

    # Phase C: track oscillation
    trajectory_z = []
    trajectory_t = []
    dt = 1.0 / SIM_HZ

    for step in range(TRACK_STEPS):
        world.step(render=not HEADLESS)
        z_now = get_world_z(stage, tip_id)
        trajectory_z.append(z_now)
        trajectory_t.append(step * dt)

    # Check for explosion
    status = "OK"
    if any(math.isnan(z) or math.isinf(z) for z in trajectory_z):
        status = "EXPLODED"

    settling_time = None
    if status == "OK":
        settling_time = find_settling_time(trajectory_z, z_settled, SETTLE_THRESH, dt)

    oscillation_amplitude = max(trajectory_z) - min(trajectory_z) if status == "OK" else float("nan")

    print(f"  Status: {status}")
    print(f"  Oscillation amplitude (sim): {oscillation_amplitude:.4f}")
    if settling_time is not None:
        print(f"  Settling time: {settling_time:.2f} s")
    else:
        print("  Did not settle within tracking window.")

    result = {
        "N": N,
        "stiffness_K": round(physics_stiffness(E, RADIUS, N, L), 4),
        "settled_z_sim": round(z_settled, 4),
        "impulse_vz": IMPULSE_VZ,
        "settling_time_s": round(settling_time, 3) if settling_time is not None else None,
        "oscillation_amplitude_sim": round(oscillation_amplitude, 4) if not math.isnan(oscillation_amplitude) else None,
        "trajectory_z": [round(z, 5) for z in trajectory_z],
        "trajectory_t": [round(t, 4) for t in trajectory_t],
        "status": status,
    }
    return result


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("\n" + "="*60)
    print("  PHASE 3 — Dynamic Oscillation Test")
    print("  Settling time invariance with N")
    print("="*60)
    print(f"  L={L}m  r={RADIUS}m  E={E:.2e}Pa  rho={DENSITY}kg/m³")
    print(f"  Impulse Vz={IMPULSE_VZ}  Threshold={SETTLE_THRESH}")

    all_results = []
    for N in N_VALUES:
        res = run_single_n(N)
        all_results.append(res)

    output = {
        "experiment": "phase3_oscillation",
        "parameters": {
            "L_m": L, "radius_m": RADIUS, "E_Pa": E,
            "density_kg_m3": DENSITY, "damping_ratio": DAMPING_RATIO,
            "scale": SCALE, "sim_hz": SIM_HZ,
            "settle_steps": SETTLE_STEPS,
            "track_seconds": TRACK_STEPS / SIM_HZ,
            "impulse_vz": IMPULSE_VZ,
            "settle_threshold": SETTLE_THRESH,
        },
        "results": all_results,
    }

    out_path = os.path.join(RESULTS_DIR, "phase3_results.json")
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n[OK] Results saved to: {out_path}")

    print("\n── Summary ────────────────────────────────────────────────────────")
    print(f"  {'N':>4}  {'K':>12}  {'Ampl (sim)':>12}  {'Settle (s)':>12}  Status")
    print(f"  {'-'*4}  {'-'*12}  {'-'*12}  {'-'*12}  ------")
    for r in all_results:
        amp = f"{r['oscillation_amplitude_sim']:.4f}" if r["oscillation_amplitude_sim"] else "NaN"
        st  = f"{r['settling_time_s']:.3f}" if r["settling_time_s"] else "Not settled"
        print(f"  {r['N']:>4}  {r['stiffness_K']:>12.1f}  {amp:>12}  {st:>12}  {r['status']}")

    simulation_app.close()

    # ── Auto-generate plot ────────────────────────────────────────────────
    plot_script = os.path.join(SCRIPT_DIR, "plot_results.py")
    fig_out     = os.path.join(RESULTS_DIR, "phase3_plot.png")
    print(f"\n[INFO] Generating plot \u2192 {fig_out}")
    result = subprocess.run(
        ["uv", "run", plot_script, "--output", fig_out],
        cwd=PROJECT_ROOT,
    )
    if result.returncode == 0:
        print(f"[OK]  Plot saved to: {fig_out}")
    else:
        print(f"[WARN] Plot script exited with code {result.returncode}. "
              "Run plot_results.py manually to debug.")


if __name__ == "__main__":
    main()
