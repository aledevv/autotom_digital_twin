"""
Phase 1 — Static Deflection Test
=================================
Validates that parameterizing joint stiffness using Euler-Bernoulli beam theory
keeps the tip deflection of a horizontal cantilever branch invariant as the
number of articulation segments N changes.

Physics formula used:
    K = (E * pi * r^4 * N) / (4 * L)

Analytical Euler-Bernoulli cantilever deflection (self-weight load):
    delta = (rho * A * g * L^4) / (8 * E * I)
    where I = pi*r^4/4, A = pi*r^2

Run headless:
    ~/isaacsim/python.sh src/experiments/joint_parametrization/phase1_static_deflection/run_phase1.py

Or via the launcher:
    ./run_experiment.sh exp1
"""

import os
import sys
import math
import json
import subprocess

# ── Path bootstrap ────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
EXP_DIR      = os.path.dirname(SCRIPT_DIR)           # joint_parametrization/
EXPERIMENTS  = os.path.dirname(EXP_DIR)              # experiments/
SRC_DIR      = os.path.dirname(EXPERIMENTS)           # src/
PROJECT_ROOT = os.path.dirname(SRC_DIR)               # autotom_digital_twin/

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

# ── Experiment parameters ─────────────────────────────────────────────────────
# These are the tunable physical parameters. All in SI units.

HEADLESS      = False          # Set False to watch the simulation in the GUI

# Branch geometry (real-world, before baked scale)
L             = 0.4           # total branch length [m]
RADIUS        = 0.01          # cylinder radius [m]  (uniform, no tapering in phase 1)

# Material
E             = 50.0e6        # Young's modulus [Pa]. 50 MPa keeps E-B sag ~1.76 cm on 40 cm
                               # branch → safely inside small-deflection regime (sag < 10% L).
DENSITY       = 700.0         # density [kg/m³] (wood-like)
DAMPING_RATIO = 0.7           # fraction of critical damping

# Simulation
SCALE         = 10.0          # baked scale (10× to mitigate small-number instability in PhysX)
GRAVITY       = 9.81          # [m/s²]
SIM_HZ        = 120           # PhysX steps per second
SIM_SECONDS   = 10.0           # settle time [s]
SIM_STEPS     = int(SIM_HZ * SIM_SECONDS)

# N values to sweep
N_VALUES      = [2, 3, 5, 10, 20, 50]

RESULTS_DIR   = os.path.join(SCRIPT_DIR, "results")

# ── Isaac Sim bootstrap ───────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Gf, Sdf
import omni.usd
from isaacsim.core.api import World

from plant_model.v2.plant_builder import PlantBuilder
from plant_model.v2.plant_builder_utils import _auto_mass, _critical_damping


# ─────────────────────────────────────────────────────────────────────────────
# Analytical reference
# ─────────────────────────────────────────────────────────────────────────────

def analytical_deflection(L: float, r: float, rho: float, E: float, g: float) -> float:
    """
    Tip deflection of a uniform cantilever beam under its own self-weight.

    Euler-Bernoulli: delta = (rho * A * g * L^4) / (8 * E * I)
    I = pi*r^4/4,  A = pi*r^2
    Simplifies to: delta = (rho * g * L^4) / (2 * E * r^2)
    """
    I = math.pi * r**4 / 4.0
    A = math.pi * r**2
    w = rho * A * g          # distributed load [N/m]
    delta = (w * L**4) / (8.0 * E * I)
    return delta


# ─────────────────────────────────────────────────────────────────────────────
# Per-N simulation
# ─────────────────────────────────────────────────────────────────────────────

def physics_stiffness(E: float, r: float, N: int, L: float) -> float:
    """
    Joint torsional stiffness derived from Euler-Bernoulli:
        K = E * I / l   where  l = L/N  and  I = pi*r^4/4
    => K = (E * pi * r^4 * N) / (4 * L)
    """
    I = math.pi * r**4 / 4.0
    l = L / N
    return E * I / l


def setup_physics_scene(stage: Usd.Stage) -> None:
    """Configure gravity and PhysX solver settings."""
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


def setup_lighting(stage: Usd.Stage) -> None:
    """
    Add studio-style lighting so the scene is visible in the Isaac Sim GUI.
    Mimics the built-in "Stage" lighting preset:
      - DomeLight   : soft ambient fill (neutral white, no HDR texture)
      - DistantLight: key light from upper-right at 60° elevation
    """
    lights_path = "/World/Lights"
    if not stage.GetPrimAtPath(lights_path):
        UsdGeom.Xform.Define(stage, lights_path)

    # ── Ambient dome ──────────────────────────────────────────────────────
    dome = UsdLux.DomeLight.Define(stage, f"{lights_path}/DomeLight")
    dome.CreateIntensityAttr().Set(800.0)
    dome.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    dome.CreateExposureAttr().Set(0.0)

    # ── Key distant light (upper-right, 60° elevation) ────────────────────
    key = UsdLux.DistantLight.Define(stage, f"{lights_path}/KeyLight")
    key.CreateIntensityAttr().Set(2500.0)
    key.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.95))   # slightly warm
    key.CreateAngleAttr().Set(0.53)                          # soft shadow edge
    key_xform = UsdGeom.Xformable(key.GetPrim())
    key_xform.AddRotateXYZOp().Set(Gf.Vec3f(-60.0, 45.0, 0.0))


def build_cantilever(stage: Usd.Stage, N: int) -> tuple[str, str, float]:
    """
    Build a horizontal cantilever branch with N uniform segments.

    Returns (root_anchor_path, tip_segment_id, tip_seg_length_scaled) so the
    caller can compute the world position of the physical tip (not just the
    segment origin).

    Architecture:
      - /World/Plant/Internode_o0_r0  — heavy root prim, FixedJoint to world
      - /World/Plant/Branch/Seg_00..Seg_N-1 — horizontal articulated chain
    """
    segment_len = L / N
    r = RADIUS

    # ── Euler-Bernoulli stiffness (real-world, unscaled) ─────────────────
    stiff_unscaled = physics_stiffness(E, r, N, L)

    # ── Scale-corrected stiffness for the PhysX simulation ───────────────
    # Geometry is enlarged by SCALE. Mass grows S^3, lever-arm grows S,
    # so gravity torque per joint scales S^4. Stiffness must grow the same way.
    stiff_sim = stiff_unscaled * (SCALE ** 4)

    r_sc = r * SCALE
    l_sc = segment_len * SCALE
    mass = _auto_mass(r_sc, l_sc, DENSITY)
    damp_sim = DAMPING_RATIO * _critical_damping(stiff_sim, mass)

    print(f"  [N={N}] l={segment_len:.4f}m  K_real={stiff_unscaled:.4f}  "
          f"K_sim={stiff_sim:.2f}  mass={mass:.5f}kg  D={damp_sim:.2f}")

    builder = PlantBuilder(stage, base_path="/World/Plant", scale=SCALE)

    # ── Static root trunk (anchor) ────────────────────────────────────────
    root_segments = [{"order": 0, "rank": 0, "radius": r, "length": 0.02}]
    builder.add_main_stem_segments("stem", root_segments, physics=False)

    anchor_prim_path = "/World/Plant/Internode_o0_r0"
    anchor_prim = stage.GetPrimAtPath(anchor_prim_path)

    UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    UsdPhysics.MassAPI.Apply(anchor_prim).CreateMassAttr().Set(1000.0)

    fj = UsdPhysics.FixedJoint.Define(stage, f"{anchor_prim_path}/WorldFixedJoint")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(anchor_prim_path)])

    # Fix 3 (fixes.md): PhysxArticulationAPI forces PhysX to use the
    # reduced-coordinate Featherstone solver for the whole chain, eliminating
    # constraint stretching / numerical compliance at high N.
    PhysxSchema.PhysxArticulationAPI.Apply(anchor_prim)

    # ── Horizontal articulated chain ──────────────────────────────────────
    trunk_id = "Internode_o0_r0"
    seg_ids  = []

    for i in range(N):
        seg_id = f"Branch/Seg_{i:02d}"
        seg_ids.append(seg_id)
        if i == 0:
            builder.add_lateral_branch(
                parent_id=trunk_id,
                id=seg_id,
                radius=r,
                length=segment_len,
                z_offset_ratio=0.5,
                tilt_angle=90.0,            # horizontal cantilever
                rot_around_parent=0.0,
                density=DENSITY,
                stiffness=stiff_sim,
                damping_ratio=DAMPING_RATIO,
                max_bend_angle=60.0,
                twist_limit=5.0,
                physics=True,
            )
        else:
            builder.add_internode(
                parent_id=seg_ids[i - 1],
                id=seg_id,
                radius=r,
                length=segment_len,
                density=DENSITY,
                stiffness=stiff_sim,
                damping_ratio=DAMPING_RATIO,
                max_bend_angle=60.0,
                physics=True,
            )

    return anchor_prim_path, seg_ids[-1], l_sc


def get_tip_world_z(stage: Usd.Stage, seg_id: str, seg_length_scaled: float) -> float:
    """
    Return the world-space Z of the PHYSICAL TIP of a segment cylinder.

    mat.ExtractTranslation() gives the segment ORIGIN (base). To get the tip
    we transform the local point (0, 0, seg_length_scaled) — the top of the
    cylinder along its local Z — into world space.
    """
    path = f"/World/Plant/{seg_id}"
    prim = stage.GetPrimAtPath(path)
    if not prim:
        raise RuntimeError(f"Prim not found: {path}")
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    tip_local = Gf.Vec4d(0.0, 0.0, seg_length_scaled, 1.0)
    tip_world = mat * tip_local
    return tip_world[2]


def run_single_n(N: int) -> dict:
    """Run one simulation trial for a given N. Returns a result dict."""
    print(f"\n{'='*60}")
    print(f"  Running Phase 1  —  N = {N} segments")
    print(f"{'='*60}")

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()

    setup_physics_scene(stage)
    setup_lighting(stage)
    anchor_path, tip_id, l_sc = build_cantilever(stage, N)   # l_sc = scaled segment length

    # Fix 1 (fixes.md): measure the PHYSICAL TIP of the last cylinder, not its origin.
    tip_z_initial = get_tip_world_z(stage, tip_id, l_sc)
    print(f"  Tip initial Z = {tip_z_initial:.4f}  (tip of Seg_{N-1}, scaled)")

    world = World(stage_units_in_meters=1.0)
    world.reset()

    for step in range(SIM_STEPS):
        world.step(render=not HEADLESS)

    tip_z_final   = get_tip_world_z(stage, tip_id, l_sc)
    deflection_sim = tip_z_initial - tip_z_final    # positive = downward sag in sim units
    deflection_m   = deflection_sim / SCALE          # back to real-world metres

    # ── Analytical references ─────────────────────────────────────────────
    # Continuous Euler-Bernoulli (N→∞ limit)
    delta_eb = analytical_deflection(L, RADIUS, DENSITY, E, GRAVITY)

    # Discrete-chain theoretical prediction: δ_N = δ_EB · (1 + 1/N)²
    # This is the mathematically correct expectation for a lumped N-segment model.
    delta_discrete = delta_eb * (1.0 + 1.0 / N) ** 2

    status = "OK"
    if math.isnan(tip_z_final) or math.isinf(tip_z_final):
        status = "EXPLODED"
        deflection_m   = float("nan")
        delta_discrete = float("nan")

    # Error vs the discrete target (the fair comparison for a finite-N chain)
    if not math.isnan(deflection_m) and delta_discrete > 0:
        error_vs_discrete = 100.0 * abs(deflection_m - delta_discrete) / delta_discrete
    else:
        error_vs_discrete = float("nan")

    result = {
        "N": N,
        "segment_length_m":       round(L / N, 6),
        "stiffness_K_real":       round(physics_stiffness(E, RADIUS, N, L), 6),
        "stiffness_K_sim":        round(physics_stiffness(E, RADIUS, N, L) * SCALE**4, 4),
        "tip_deflection_sim_m":   round(deflection_m, 6) if not math.isnan(deflection_m) else None,
        "analytical_eb_m":        round(delta_eb, 6),
        "analytical_discrete_m":  round(delta_discrete, 6) if not math.isnan(delta_discrete) else None,
        "error_vs_discrete_pct":  round(error_vs_discrete, 2) if not math.isnan(error_vs_discrete) else None,
        "status": status,
    }

    print(f"  Tip final Z        = {tip_z_final:.4f}")
    print(f"  Deflection (sim)   = {deflection_m*100:.3f} cm")
    print(f"  Deflection (E-B ∞) = {delta_eb*100:.3f} cm")
    print(f"  Deflection (disc.) = {delta_discrete*100:.3f} cm  [= E-B·(1+1/N)²]")
    print(f"  Error vs discrete  = {error_vs_discrete:.1f}%   Status: {status}")

    return result


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("\n" + "="*60)
    print("  PHASE 1 — Static Deflection Test")
    print("  Euler-Bernoulli Joint Parametrization")
    print("="*60)
    print(f"  L={L}m  r={RADIUS}m  E={E:.2e}Pa  rho={DENSITY}kg/m³")
    print(f"  SCALE={SCALE}  SIM_STEPS={SIM_STEPS} ({SIM_SECONDS}s @ {SIM_HZ}Hz)")
    delta_ref = analytical_deflection(L, RADIUS, DENSITY, E, GRAVITY)
    print(f"  Analytical Euler-Bernoulli deflection: {delta_ref*100:.3f} cm")

    all_results = []
    for N in N_VALUES:
        res = run_single_n(N)
        all_results.append(res)

    # ── Save results ─────────────────────────────────────────────────────
    delta_ref = analytical_deflection(L, RADIUS, DENSITY, E, GRAVITY)
    output = {
        "experiment": "phase1_static_deflection",
        "parameters": {
            "L_m": L, "radius_m": RADIUS, "E_Pa": E,
            "density_kg_m3": DENSITY, "damping_ratio": DAMPING_RATIO,
            "scale": SCALE, "sim_hz": SIM_HZ, "sim_seconds": SIM_SECONDS,
        },
        "analytical_eb_m": round(delta_ref, 6),
        "results": all_results,
    }

    out_path = os.path.join(RESULTS_DIR, "phase1_results.json")
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)

    print(f"\n[OK] Results saved to: {out_path}")
    print("\n── Summary ────────────────────────────────────────────────────────────")
    print(f"  {'N':>4}  {'K_real':>10}  {'K_sim':>12}  {'Meas(cm)':>10}  "
          f"{'Disc(cm)':>10}  {'EB(cm)':>8}  {'Err%':>7}  Status")
    print(f"  {'-'*4}  {'-'*10}  {'-'*12}  {'-'*10}  {'-'*10}  {'-'*8}  {'-'*7}  ------")
    for r in all_results:
        meas = f"{r['tip_deflection_sim_m']*100:.3f}"  if r['tip_deflection_sim_m'] is not None else "NaN"
        disc = f"{r['analytical_discrete_m']*100:.3f}" if r['analytical_discrete_m'] is not None else "NaN"
        eb   = f"{r['analytical_eb_m']*100:.3f}"
        err  = f"{r['error_vs_discrete_pct']:.1f}"    if r['error_vs_discrete_pct'] is not None else "NaN"
        k_r  = f"{r['stiffness_K_real']:.4f}"
        k_s  = f"{r['stiffness_K_sim']:.1f}"
        print(f"  {r['N']:>4}  {k_r:>10}  {k_s:>12}  {meas:>10}  {disc:>10}  {eb:>8}  {err:>7}  {r['status']}")

    simulation_app.close()

    # ── Auto-generate plot ────────────────────────────────────────────────
    plot_script = os.path.join(SCRIPT_DIR, "plot_results.py")
    fig_out     = os.path.join(RESULTS_DIR, "phase1_plot.png")
    print(f"\n[INFO] Generating plot → {fig_out}")
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
