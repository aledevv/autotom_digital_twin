"""
Phase 2 — Tapering Test
========================
Compares two stiffness-tapering strategies on a horizontal cantilever
branch whose radius decreases linearly from base to tip:

  Strategy A (linear):  K_i = K_base + t * (K_tip - K_base)
                        where t = i/(N-1) and K values are hand-tuned scalars.

  Strategy B (physics): K_i = E * pi * r_i^4 / (4 * l_i)
                        where r_i is the local radius at segment i (r⁴ law).

Both branches have the same total length, same base and tip radii.
We measure tip deflection for each strategy and compare to the
Euler-Bernoulli tapered-beam analytical formula (numerical integration).

Run headless:
    ~/isaacsim/python.sh src/experiments/joint_parametrization/phase2_tapering/run_phase2.py

Or via the launcher:
    ./run_experiment.sh exp2
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

L             = 0.4           # total branch length [m]
R_BASE        = 0.014         # base radius [m]
R_TIP         = 0.005         # tip radius [m]

E             = 1.0e6         # Young's modulus [Pa]
DENSITY       = 700.0         # [kg/m³]
DAMPING_RATIO = 0.7

# Linear strategy hand-tuned scalars (matched so that the base K equals the
# physics-derived K at r=R_BASE, and tip K at r=R_TIP with N=5)
N_SEGMENTS    = 5             # fixed N for both strategies (fair comparison)

SCALE         = 10.0
GRAVITY       = 9.81
SIM_HZ        = 120
SIM_SECONDS   = 3.0
SIM_STEPS     = int(SIM_HZ * SIM_SECONDS)

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

def physics_stiffness_at(E: float, r: float, l: float) -> float:
    """K = E * I / l  (Euler-Bernoulli, I = pi*r^4/4)."""
    I = math.pi * r**4 / 4.0
    return E * I / l


def radius_at(i: int, N: int) -> float:
    """Linearly tapered radius from R_BASE (i=0) to R_TIP (i=N-1)."""
    t = i / max(N - 1, 1)
    return R_BASE + t * (R_TIP - R_BASE)


def analytical_tapered_deflection_numerical(L, r_base, r_tip, rho, E, g, n_elements=200):
    """
    Numerical integration of Euler-Bernoulli for a tapered cantilever
    under self-weight using the conjugate beam method (virtual work).

    We discretise the beam into n_elements strips and integrate:
        delta = integral_0^L [ M(x) * m(x) / (E*I(x)) ] dx
    where M(x) is the bending moment from self-weight,
          m(x) is the virtual moment from a unit tip load.

    For self-weight loading:
        M(x) = integral_x^L  rho*A(xi)*g*(xi-x) dxi
    For unit virtual tip load:
        m(x) = (L - x)

    We compute M(x) numerically via trapz integration.
    """
    xs   = [i * L / n_elements for i in range(n_elements + 1)]
    r_fn = lambda x: r_base + (r_tip - r_base) * (x / L)
    A_fn = lambda x: math.pi * r_fn(x)**2
    I_fn = lambda x: math.pi * r_fn(x)**4 / 4.0

    # Distributed load w(x) = rho * A(x) * g
    w = [rho * A_fn(x) * g for x in xs]

    # Bending moment at each x due to self-weight (integrate from x to L)
    M = []
    for i, x in enumerate(xs):
        # trapz from i to n_elements
        integrand = [w[j] * (xs[j] - x) for j in range(i, n_elements + 1)]
        dx = L / n_elements
        M.append(sum(integrand) * dx)

    # Virtual moment from unit tip load: m(x) = L - x
    m = [L - x for x in xs]

    # Integrate M*m / (E*I)
    integrand_delta = [M[i] * m[i] / (E * I_fn(xs[i])) for i in range(n_elements + 1)]
    dx = L / n_elements
    delta = sum(integrand_delta) * dx
    return delta


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


def build_tapered_cantilever(stage, strategy: str, N: int) -> tuple[str, str]:
    """
    Build a tapered cantilever.
    strategy: "linear" or "physics"
    Returns (anchor_path, tip_seg_id).
    """
    segment_len = L / N
    builder = PlantBuilder(stage, base_path="/World/Plant", scale=SCALE)

    # Root anchor
    root_segs = [{"order": 0, "rank": 0, "radius": R_BASE, "length": 0.02}]
    builder.add_main_stem_segments("stem", root_segs, physics=False)

    anchor_path = "/World/Plant/Internode_o0_r0"
    anchor_prim = stage.GetPrimAtPath(anchor_path)
    UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    UsdPhysics.MassAPI.Apply(anchor_prim).CreateMassAttr().Set(1000.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{anchor_path}/WorldFixedJoint")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(anchor_path)])

    trunk_id = "Internode_o0_r0"

    # Physics-strategy: precompute base K to scale the linear strategy equally
    k_base_physics = physics_stiffness_at(E, R_BASE, segment_len)
    k_tip_physics  = physics_stiffness_at(E, R_TIP,  segment_len)

    seg_ids = []
    for i in range(N):
        r_i = radius_at(i, N)
        seg_id = f"Branch/Seg_{i:02d}"
        seg_ids.append(seg_id)

        # Choose stiffness by strategy
        if strategy == "physics":
            k_i = physics_stiffness_at(E, r_i, segment_len)
        else:  # linear
            t = i / max(N - 1, 1)
            k_i = k_base_physics + t * (k_tip_physics - k_base_physics)

        r_sc   = r_i * SCALE
        l_sc   = segment_len * SCALE
        mass   = _auto_mass(r_sc, l_sc, DENSITY)
        damp   = DAMPING_RATIO * _critical_damping(k_i, mass)

        if i == 0:
            builder.add_lateral_branch(
                parent_id=trunk_id, id=seg_id,
                radius=r_i, length=segment_len,
                z_offset_ratio=0.5, tilt_angle=90.0, rot_around_parent=0.0,
                density=DENSITY, stiffness=k_i, damping_ratio=DAMPING_RATIO,
                max_bend_angle=60.0, twist_limit=5.0, physics=True,
            )
        else:
            builder.add_internode(
                parent_id=seg_ids[i - 1], id=seg_id,
                radius=r_i, length=segment_len,
                density=DENSITY, stiffness=k_i, damping_ratio=DAMPING_RATIO,
                max_bend_angle=60.0, physics=True,
            )

    return anchor_path, seg_ids[-1]


def run_single_strategy(strategy: str) -> dict:
    print(f"\n{'='*60}")
    print(f"  Phase 2 — Strategy: {strategy.upper()}  N={N_SEGMENTS}")
    print(f"{'='*60}")

    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    setup_physics_scene(stage)
    anchor_path, tip_id = build_tapered_cantilever(stage, strategy, N_SEGMENTS)

    tip_z_initial = get_world_z(stage, tip_id)

    world = World(stage_units_in_meters=1.0)
    world.reset()
    for _ in range(SIM_STEPS):
        world.step(render=not HEADLESS)

    tip_z_final = get_world_z(stage, tip_id)
    deflection_m = (tip_z_initial - tip_z_final) / SCALE

    delta_ref = analytical_tapered_deflection_numerical(L, R_BASE, R_TIP, DENSITY, E, GRAVITY)

    if delta_ref > 0:
        error_pct = 100.0 * abs(deflection_m - delta_ref) / delta_ref
    else:
        error_pct = float("nan")

    status = "OK"
    if math.isnan(tip_z_final) or math.isinf(tip_z_final):
        status = "EXPLODED"
        deflection_m = float("nan")
        error_pct    = float("nan")

    result = {
        "strategy": strategy,
        "N": N_SEGMENTS,
        "tip_deflection_sim_m": round(deflection_m, 6) if not math.isnan(deflection_m) else None,
        "analytical_tapered_deflection_m": round(delta_ref, 6),
        "error_pct": round(error_pct, 2) if not math.isnan(error_pct) else None,
        "status": status,
    }

    print(f"  Deflection (sim) = {deflection_m*100:.3f} cm"
          if not math.isnan(deflection_m) else "  Deflection: EXPLODED")
    print(f"  Deflection (E-B) = {delta_ref*100:.3f} cm")
    print(f"  Error            = {error_pct:.1f}%  Status: {status}"
          if not math.isnan(error_pct) else f"  Error: NaN  Status: {status}")
    return result


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)

    print("\n" + "="*60)
    print("  PHASE 2 — Tapering Test")
    print("  Linear vs. r⁴ (physics-derived) stiffness tapering")
    print("="*60)
    print(f"  L={L}m  R_base={R_BASE}m  R_tip={R_TIP}m  E={E:.2e}Pa  N={N_SEGMENTS}")

    results = []
    for strategy in ("linear", "physics"):
        results.append(run_single_strategy(strategy))

    delta_ref = analytical_tapered_deflection_numerical(L, R_BASE, R_TIP, DENSITY, E, GRAVITY)
    output = {
        "experiment": "phase2_tapering",
        "parameters": {
            "L_m": L, "R_base_m": R_BASE, "R_tip_m": R_TIP,
            "E_Pa": E, "density_kg_m3": DENSITY, "damping_ratio": DAMPING_RATIO,
            "N_segments": N_SEGMENTS, "scale": SCALE,
            "sim_hz": SIM_HZ, "sim_seconds": SIM_SECONDS,
        },
        "analytical_tapered_deflection_m": round(delta_ref, 6),
        "results": results,
    }

    out_path = os.path.join(RESULTS_DIR, "phase2_results.json")
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n[OK] Results saved to: {out_path}")

    print("\n── Summary ────────────────────────────────────────────────")
    print(f"  {'Strategy':>10}  {'Measured (cm)':>14}  {'E-B (cm)':>10}  {'Error%':>8}  Status")
    print(f"  {'-'*10}  {'-'*14}  {'-'*10}  {'-'*8}  ------")
    for r in results:
        meas = f"{r['tip_deflection_sim_m']*100:.3f}" if r["tip_deflection_sim_m"] else "NaN"
        eb   = f"{r['analytical_tapered_deflection_m']*100:.3f}"
        err  = f"{r['error_pct']:.1f}" if r["error_pct"] else "NaN"
        print(f"  {r['strategy']:>10}  {meas:>14}  {eb:>10}  {err:>8}  {r['status']}")

    simulation_app.close()

    # ── Auto-generate plot ────────────────────────────────────────────────
    plot_script = os.path.join(SCRIPT_DIR, "plot_results.py")
    fig_out     = os.path.join(RESULTS_DIR, "phase2_plot.png")
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
