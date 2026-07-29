"""
run_phase1_v2.py — Phase 1 Static Deflection Test (Version 2)
==============================================================
Validates that parameterising joint stiffness via Euler-Bernoulli beam theory
keeps the tip deflection of a horizontal cantilever branch invariant as the
number of articulation segments N changes.

Differences from phase1_static_deflection/run_phase1.py:
  - Zero dependency on PlantBuilder or any plant_model.* module.
  - Every USD prim, joint, drive, and physics API is created inline and visible.
  - Math helpers (mass, damping, stiffness, quaternion) are defined at the top
    of this file — no hidden abstractions.
  - Two simulation passes per N:
      Pass 0 (zero-gravity sanity): tip must NOT move.
      Pass 1 (gravity sweep): measure final tip deflection.
  - Per-step diagnostic logging every LOG_EVERY steps.
  - Early-convergence detection: breaks the step loop once the tip has settled.
  - GUI mode on by default so you can watch the branch droop.

Physics recap:
    I  = π r⁴ / 4
    K  = E · I / l      (l = L / N, one segment length)
    K_sim = K · SCALE⁴  (geometry inflated by SCALE to mitigate PhysX small-number issues)
    mass  = max(π r_sc² · l_sc · ρ,  MASS_FLOOR)
    damp  = DAMPING_RATIO · 2√(K_sim · mass)

Expected discrete-chain deflection:
    δ_N = δ_EB · (1 + 1/N)²,   δ_EB = ρ·g·L⁴ / (2·E·r²)

Run:
    ~/isaacsim/python.sh src/experiments/joint_parametrization/phase1_version2/run_phase1_v2.py

Or via the launcher:
    ./run_experiment.sh exp1v2
"""

import os
import sys
import math
import json
import subprocess

# ── Path bootstrap ─────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
EXP_DIR      = os.path.dirname(SCRIPT_DIR)
EXPERIMENTS  = os.path.dirname(EXP_DIR)
SRC_DIR      = os.path.dirname(EXPERIMENTS)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

# ── Experiment parameters (SI units throughout) ────────────────────────────────
HEADLESS      = True     # set False to watch in the GUI

L             = 0.4      # total branch length [m]
RADIUS        = 0.01     # cylinder radius [m]  (uniform — no tapering in phase 1)

E             = 50.0e6   # Young's modulus [Pa].  50 MPa → δ_EB ≈ 1.76 cm → inside
                          # small-deflection regime (δ < 10% L = 4 cm).
DENSITY       = 700.0    # [kg/m³]
DAMPING_RATIO = 0.7      # fraction of critical damping (underdamped, settles cleanly)

SCALE         = 10.0     # geometry scale factor (10× to avoid PhysX small-number issues)
GRAVITY       = 9.81     # [m/s²]
SIM_HZ        = 120      # PhysX substeps per second
SIM_SECONDS   = 10.0     # settle time per trial
SIM_STEPS     = int(SIM_HZ * SIM_SECONDS)

# Logging: print a diagnostics line every this many steps (every 0.5 s of sim time)
LOG_EVERY     = SIM_HZ // 2

# Early convergence: stop the step loop if the last 3 log readings differ by less
# than this threshold in sim units (sub-micrometre in real-world terms)
CONVERGENCE_TOL = 1e-5 * SCALE

# Explosion guard: if |tip_Z| exceeds this in sim units, abort immediately
EXPLOSION_LIMIT = 20.0 * SCALE

# Zero-gravity sanity tolerance: tip must not drift more than this [sim units]
SANITY_TOL    = 1e-4 * SCALE

# Segment counts to sweep
N_VALUES      = [2, 5]   # validation run — extend to [2,3,5,10,20,50] after sanity confirmed

# Mass floor for very thin/short segments [kg]
MASS_FLOOR    = 0.005

RESULTS_DIR   = os.path.join(SCRIPT_DIR, "results")


# ── Isaac Sim bootstrap ────────────────────────────────────────────────────────
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Gf, Sdf
import omni.usd
from isaacsim.core.api import World


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 1 — Inline math helpers  (no plant_model.* imports)
# ═════════════════════════════════════════════════════════════════════════════

def analytical_deflection(L: float, r: float, rho: float, E: float, g: float) -> float:
    """
    Tip deflection of a uniform cantilever beam under its own self-weight.
    Euler-Bernoulli:  δ = (ρ·A·g·L⁴) / (8·E·I)
    I = πr⁴/4,  A = πr² → simplifies to δ = (ρ·g·L⁴) / (2·E·r²)
    """
    I = math.pi * r**4 / 4.0
    A = math.pi * r**2
    w = rho * A * g          # distributed load [N/m]
    return (w * L**4) / (8.0 * E * I)


def physics_stiffness(E: float, r: float, N: int, L: float) -> float:
    """
    Torsional spring stiffness for one segment of a discretised cantilever.
    Derived from Euler-Bernoulli: K = E·I / l, l = L/N, I = πr⁴/4.
    Returns the real-world (unscaled) value in N·m/rad.
    """
    I = math.pi * r**4 / 4.0
    l = L / N
    return E * I / l


def auto_mass(radius: float, length: float, density: float) -> float:
    """
    Mass of a cylinder segment: volume × density, with a minimum floor.
    radius and length must already be in scaled (sim) units.
    """
    volume = math.pi * radius**2 * length
    return max(volume * density, MASS_FLOOR)


def critical_damping(stiffness: float, mass: float) -> float:
    """Critical damping coefficient: 2√(K·m)."""
    return 2.0 * math.sqrt(stiffness * mass)


def quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    """Convert Gf.Quatd → Gf.Quatf without relying on implicit constructor."""
    im = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()),
                    float(im[0]), float(im[1]), float(im[2]))


def configure_drives(
    joint_prim: Usd.Prim,
    stiff: float,
    damp: float,
    bend_limit: float,
    lock_twist: bool,
    twist_limit: float = 5.0,
) -> None:
    """
    Configure a USD joint prim for a torsional spring:
      - translational DOFs: locked (low > high is PhysX convention for locked)
      - rotX, rotY: spring drive with ±bend_limit [deg]
      - rotZ: locked if lock_twist=True, else ±twist_limit spring drive
    """
    # Lock all translational DOFs
    for ax in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(joint_prim, ax)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    # Bending DOFs (rotX, rotY) — spring drive
    for ax in ("rotX", "rotY"):
        lim = UsdPhysics.LimitAPI.Apply(joint_prim, ax)
        lim.CreateLowAttr().Set(-bend_limit)
        lim.CreateHighAttr().Set(bend_limit)
        drv = UsdPhysics.DriveAPI.Apply(joint_prim, ax)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiff)
        drv.CreateDampingAttr().Set(damp)
        drv.CreateTargetPositionAttr().Set(0.0)

    # Twist DOF (rotZ)
    lim_z = UsdPhysics.LimitAPI.Apply(joint_prim, "rotZ")
    if lock_twist:
        lim_z.CreateLowAttr().Set(1.0)
        lim_z.CreateHighAttr().Set(-1.0)
    else:
        lim_z.CreateLowAttr().Set(-twist_limit)
        lim_z.CreateHighAttr().Set(twist_limit)
        drv_z = UsdPhysics.DriveAPI.Apply(joint_prim, "rotZ")
        drv_z.CreateTypeAttr().Set("force")
        drv_z.CreateStiffnessAttr().Set(stiff)
        drv_z.CreateDampingAttr().Set(damp)
        drv_z.CreateTargetPositionAttr().Set(0.0)


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 2 — Scene helpers
# ═════════════════════════════════════════════════════════════════════════════

def setup_physics_scene(stage: Usd.Stage, gravity_magnitude: float = GRAVITY) -> None:
    """
    Create (or reconfigure) the PhysX scene with TGS solver.
    gravity_magnitude=0.0 is used for the zero-gravity sanity pass.
    """
    sc_path = "/World/PhysicsScene"
    if stage.GetPrimAtPath(sc_path):
        sc = UsdPhysics.Scene(stage.GetPrimAtPath(sc_path))
    else:
        sc = UsdPhysics.Scene.Define(stage, sc_path)

    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(gravity_magnitude)

    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(SIM_HZ)
    px.CreateEnableStabilizationAttr().Set(True)


def setup_lighting(stage: Usd.Stage) -> None:
    """
    Studio-style lighting so the scene is visible in the GUI.
      - DomeLight  : soft ambient fill (neutral white)
      - DistantLight: key light from upper-right at 60° elevation
    """
    lights_path = "/World/Lights"
    if not stage.GetPrimAtPath(lights_path):
        UsdGeom.Xform.Define(stage, lights_path)

    dome = UsdLux.DomeLight.Define(stage, f"{lights_path}/DomeLight")
    dome.CreateIntensityAttr().Set(800.0)
    dome.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    dome.CreateExposureAttr().Set(0.0)

    key = UsdLux.DistantLight.Define(stage, f"{lights_path}/KeyLight")
    key.CreateIntensityAttr().Set(2500.0)
    key.CreateColorAttr().Set(Gf.Vec3f(1.0, 0.98, 0.95))
    key.CreateAngleAttr().Set(0.53)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-60.0, 45.0, 0.0))


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 3 — Raw cantilever builder  (zero PlantBuilder dependency)
# ═════════════════════════════════════════════════════════════════════════════

# Green-ish color for the branch cylinders
_BRANCH_COLOR = Gf.Vec3f(0.35, 0.62, 0.20)
_ANCHOR_COLOR = Gf.Vec3f(0.55, 0.35, 0.15)

IDENTITY_QUATF = Gf.Quatf(1.0, 0.0, 0.0, 0.0)


def _define_cylinder(stage: Usd.Stage, path: str,
                      radius: float, height: float,
                      color: Gf.Vec3f) -> Usd.Prim:
    """
    Create a UsdGeom.Cylinder centered at local (0, 0, height/2) along the Z axis.
    The parent Xform controls world placement; the cylinder sits "above" the origin
    so the joint pivot is at the segment base, not its centre.
    """
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
    cyl.GetDisplayColorAttr().Set([color])
    return cyl.GetPrim()


def build_cantilever_raw(stage: Usd.Stage, N: int) -> tuple[str, list[str], float]:
    """
    Build a horizontal cantilever from scratch using only raw USD/PhysX APIs.
    No PlantBuilder.  Every prim, joint, and drive is created explicitly here.

    Layout (scaled units):
      /World/Plant                   — root Xform (not a rigid body)
      /World/Plant/Anchor            — 1000 kg static anchor, FixedJoint to world,
                                       PhysxArticulationAPI (Featherstone solver)
      /World/Plant/Seg_00 … Seg_N-1  — articulated chain, oriented horizontally

    The first segment (Seg_00) is tilted 90° around the X-axis so its local +Z
    axis points in the world +Y direction (horizontal).  Each subsequent segment
    is tip-to-tip with its predecessor — same orientation.

    Returns
    -------
    (anchor_path, seg_paths, seg_length_scaled)
      seg_length_scaled  : scaled height of one cylinder (used for tip measurement)
    """
    root_path = "/World/Plant"
    if not stage.GetPrimAtPath(root_path):
        UsdGeom.Xform.Define(stage, root_path)

    segment_len = L / N             # real-world segment length [m]
    r_sc  = RADIUS * SCALE          # scaled radius
    l_sc  = segment_len * SCALE     # scaled segment length

    # ── Euler-Bernoulli stiffness ──────────────────────────────────────────
    K_real = physics_stiffness(E, RADIUS, N, L)
    # Torque per joint scales S⁴ when geometry is enlarged by S → K must scale S⁴
    K_sim  = K_real * (SCALE ** 4)
    mass   = auto_mass(r_sc, l_sc, DENSITY)
    damp   = DAMPING_RATIO * critical_damping(K_sim, mass)

    print(f"  [build N={N}]  l={segment_len:.4f}m  K_real={K_real:.4f}  "
          f"K_sim={K_sim:.2f}  mass={mass:.5f}kg  damp={damp:.2f}")

    # ── Anchor ────────────────────────────────────────────────────────────
    anchor_path = f"{root_path}/Anchor"
    anchor_xform = UsdGeom.Xform.Define(stage, anchor_path)
    anchor_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    anchor_xform.AddOrientOp().Set(IDENTITY_QUATF)

    # Small visual marker for the anchor
    _define_cylinder(stage, f"{anchor_path}/Cylinder",
                     radius=r_sc * 1.5, height=r_sc * 2.0, color=_ANCHOR_COLOR)

    anchor_prim = anchor_xform.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(anchor_prim)
    UsdPhysics.MassAPI.Apply(anchor_prim).CreateMassAttr().Set(1000.0)

    # Fix the anchor to the world frame
    fj = UsdPhysics.FixedJoint.Define(stage, f"{anchor_path}/WorldFixedJoint")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(anchor_path)])

    # Force Featherstone reduced-coordinate solver for the whole chain.
    # Without this, maximal-coordinate constraints stretch at high N values.
    PhysxSchema.PhysxArticulationAPI.Apply(anchor_prim)

    # ── Articulated chain ─────────────────────────────────────────────────
    # The first segment is tilted 90° around X so it points horizontally (+Y world).
    # rot_tilt: local +Z → world +Y after this rotation
    tilt_rot  = Gf.Rotation(Gf.Vec3d(1, 0, 0), -90.0)   # −90° around X
    orient_qf = quatd_to_quatf(tilt_rot.GetQuat())

    seg_paths: list[str] = []
    prev_path  = anchor_path
    prev_world_pos = Gf.Vec3d(0.0, 0.0, 0.0)  # anchor is at origin
    prev_height_sc = 0.0                         # anchor contributes no chain length

    for i in range(N):
        seg_path = f"{root_path}/Seg_{i:02d}"
        seg_paths.append(seg_path)

        # World position of this segment's origin = previous segment's tip
        # (tip = origin + local-Z-axis × l_sc, transformed by orientation)
        if i == 0:
            # Seg_00 origin: at world origin (on top of the anchor)
            world_pos = Gf.Vec3d(0.0, 0.0, 0.0)
            # Joint pivot on the anchor: at the anchor's "top" (0, 0, 0 — anchor has no height)
            local_pos0 = Gf.Vec3f(0.0, 0.0, 0.0)
            # The branch starts horizontal: tilt −90° around X
            joint_rot0 = orient_qf
        else:
            # Each subsequent segment's origin = previous segment's tip in world space.
            # Since all segments share the same orientation (horizontal), the tip
            # advances by l_sc along the local +Z axis → which is world +Y.
            world_pos = prev_world_pos + Gf.Vec3d(0.0, l_sc, 0.0)
            # Joint pivot on the previous segment: at its tip (local Z = l_sc)
            local_pos0 = Gf.Vec3f(0.0, 0.0, float(l_sc))
            joint_rot0 = IDENTITY_QUATF   # no extra rotation — same direction

        seg_xform = UsdGeom.Xform.Define(stage, seg_path)
        seg_xform.AddTranslateOp().Set(world_pos)
        seg_xform.AddOrientOp().Set(orient_qf)

        _define_cylinder(stage, f"{seg_path}/Cylinder",
                         radius=r_sc, height=l_sc, color=_BRANCH_COLOR)

        seg_prim = seg_xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(seg_prim)
        UsdPhysics.MassAPI.Apply(seg_prim).CreateMassAttr().Set(mass)

        # Collision on the cylinder mesh
        cyl_prim = stage.GetPrimAtPath(f"{seg_path}/Cylinder")
        UsdPhysics.CollisionAPI.Apply(cyl_prim)

        # Joint connecting this segment to the previous one
        jnt = UsdPhysics.Joint.Define(stage, f"{seg_path}/Joint")
        jnt.CreateBody0Rel().SetTargets([Sdf.Path(prev_path)])
        jnt.CreateBody1Rel().SetTargets([Sdf.Path(seg_path)])
        jnt.CreateLocalPos0Attr().Set(local_pos0)
        jnt.CreateLocalRot0Attr().Set(joint_rot0)
        jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)

        configure_drives(
            jnt.GetPrim(),
            stiff=K_sim,
            damp=damp,
            bend_limit=60.0,
            lock_twist=False,
            twist_limit=5.0,
        )

        prev_path      = seg_path
        prev_world_pos = world_pos

    return anchor_path, seg_paths, l_sc


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 4 — Tip measurement
# ═════════════════════════════════════════════════════════════════════════════

def get_tip_world_z(stage: Usd.Stage, seg_path: str, seg_length_scaled: float) -> float:
    """
    Return the world-space Z coordinate of the PHYSICAL TIP of a segment.

    Why not ExtractTranslation()?
    ─────────────────────────────
    mat.ExtractTranslation() returns the Xform *origin* (the segment base).
    For a chain of N segments the last segment's base is at position (N-1)/N
    along the branch — not at the tip.  As N increases the origin gets closer
    to the true tip, which is why the original v1 experiment showed deflection
    increasing monotonically with N: the measurement point was moving, not
    the physics.

    Correct approach: transform the local-frame tip point (0, 0, l_sc) into
    world space using the full 4×4 local-to-world matrix.
    """
    prim = stage.GetPrimAtPath(seg_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found or invalid: {seg_path}")
    mat = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    tip_local = Gf.Vec4d(0.0, 0.0, seg_length_scaled, 1.0)
    tip_world = mat * tip_local
    return float(tip_world[2])


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 5 — Zero-gravity sanity pass
# ═════════════════════════════════════════════════════════════════════════════

def run_sanity_pass(N: int) -> tuple[bool, float]:
    """
    Build the cantilever with gravity=0, step 120 frames, measure tip drift.

    Returns (passed: bool, drift_sim: float).
    The stage is torn down after this call; the caller must create a fresh
    stage for the gravity sweep.
    """
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    setup_physics_scene(stage, gravity_magnitude=0.0)
    setup_lighting(stage)

    _, seg_paths, l_sc = build_cantilever_raw(stage, N)
    tip_path = seg_paths[-1]

    # Measure BEFORE world.reset() to see the raw USD-placed position
    tip_z_usd = get_tip_world_z(stage, tip_path, l_sc)

    # KEY FIX: set_defaults=False prevents World/PhysicsContext from overwriting
    # the gravity=0 we already set in setup_physics_scene() with its default -9.81.
    world = World(stage_units_in_meters=1.0, set_defaults=False)
    world.reset()

    tip_z_initial = get_tip_world_z(stage, tip_path, l_sc)
    tip_jump = tip_z_initial - tip_z_usd
    print(f"  [SANITY N={N:>2}]  tip_Z_usd={tip_z_usd:.6f}  "
          f"after_reset={tip_z_initial:.6f}  jump={tip_jump:.2e}")

    for step in range(120):
        world.step(render=False)   # always headless for the sanity pass

    tip_z_final = get_tip_world_z(stage, tip_path, l_sc)
    drift = abs(tip_z_final - tip_z_initial)
    passed = drift < SANITY_TOL

    status = "PASS" if passed else "FAIL"
    print(f"  [SANITY N={N:>2}]  initial_Z={tip_z_initial:.6f}  "
          f"final_Z={tip_z_final:.6f}  drift={drift:.2e}  → {status}")
    return passed, drift


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 6 — Single-N gravity sweep
# ═════════════════════════════════════════════════════════════════════════════

def run_gravity_sweep(N: int) -> dict:
    """
    Build the cantilever with full gravity, step up to SIM_STEPS frames,
    log tip Z every LOG_EVERY steps, detect early convergence, and return
    a result dictionary.

    The step loop also watches for numerical explosion so we can abort early
    and mark the result rather than waiting the full 10 s.
    """
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    setup_physics_scene(stage, gravity_magnitude=GRAVITY)
    setup_lighting(stage)

    _, seg_paths, l_sc = build_cantilever_raw(stage, N)
    tip_path = seg_paths[-1]

    world = World(stage_units_in_meters=1.0, set_defaults=False)
    world.reset()

    tip_z_initial = get_tip_world_z(stage, tip_path, l_sc)
    print(f"  [N={N}] gravity sweep start — tip_Z_initial = {tip_z_initial:.6f}")

    # Pre-compute reference values for the live log
    delta_eb       = analytical_deflection(L, RADIUS, DENSITY, E, GRAVITY)
    delta_discrete = delta_eb * (1.0 + 1.0 / N) ** 2
    target_sim     = delta_discrete * SCALE   # expected drop in sim units

    # Per-step tracking
    recent_z: list[float] = []       # last 3 logged Z readings (convergence check)
    step_converged: int | None = None
    status = "OK"
    converge_log: list[dict] = []    # full trace: [{step, t_s, tip_z, delta_cm}]

    for step in range(SIM_STEPS):
        world.step(render=not HEADLESS)

        # ── Explosion guard ───────────────────────────────────────────────
        if step % 10 == 0:
            tip_z = get_tip_world_z(stage, tip_path, l_sc)
            if math.isnan(tip_z) or math.isinf(tip_z) or abs(tip_z) > EXPLOSION_LIMIT:
                print(f"  [N={N}] !! EXPLODED at step {step} (tip_Z={tip_z:.4f})")
                status = "EXPLODED"
                break

        # ── Periodic diagnostic log ───────────────────────────────────────
        if step % LOG_EVERY == 0:
            tip_z    = get_tip_world_z(stage, tip_path, l_sc)
            t_s      = step / SIM_HZ
            delta_cm = (tip_z_initial - tip_z) / SCALE * 100.0   # real-world cm
            converge_log.append({"step": step, "t_s": round(t_s, 3),
                                  "tip_z": round(tip_z, 6), "delta_cm": round(delta_cm, 4)})
            print(f"  [N={N:>2} | t={t_s:5.2f}s | step={step:>5}/{SIM_STEPS}]  "
                  f"tip_Z={tip_z:+.4f}  Δ={delta_cm:+6.3f}cm  "
                  f"(target≈{delta_discrete*100:.3f}cm)")

            # ── Early convergence detection ───────────────────────────────
            recent_z.append(tip_z)
            if len(recent_z) > 3:
                recent_z.pop(0)
            if (len(recent_z) == 3
                    and max(recent_z) - min(recent_z) < CONVERGENCE_TOL
                    and step > LOG_EVERY * 3):   # don't trigger in the first few steps
                step_converged = step
                t_conv = step / SIM_HZ
                print(f"  [N={N}] ✓ CONVERGED early at t={t_conv:.2f}s (step {step})")
                break

    tip_z_final   = get_tip_world_z(stage, tip_path, l_sc)
    deflection_sim = tip_z_initial - tip_z_final
    deflection_m   = deflection_sim / SCALE

    if status == "EXPLODED" or math.isnan(tip_z_final) or math.isinf(tip_z_final):
        status = "EXPLODED"
        deflection_m = float("nan")

    # Error vs the discrete-chain theoretical target (the fair comparison)
    if not math.isnan(deflection_m) and delta_discrete > 0:
        err_vs_disc = 100.0 * (deflection_m - delta_discrete) / delta_discrete
    else:
        err_vs_disc = float("nan")

    print(f"  [N={N}] FINAL  tip_Z={tip_z_final:.6f}  "
          f"Δ_sim={deflection_m*100:.3f}cm  "
          f"target={delta_discrete*100:.3f}cm  "
          f"err={err_vs_disc:+.1f}%  status={status}")

    return {
        "N":                      N,
        "segment_length_m":       round(L / N, 6),
        "stiffness_K_real":       round(physics_stiffness(E, RADIUS, N, L), 6),
        "stiffness_K_sim":        round(physics_stiffness(E, RADIUS, N, L) * SCALE**4, 4),
        "tip_deflection_sim_m":   round(deflection_m, 6) if not math.isnan(deflection_m) else None,
        "analytical_eb_m":        round(delta_eb, 6),
        "analytical_discrete_m":  round(delta_discrete, 6),
        "error_vs_discrete_pct":  round(err_vs_disc, 2) if not math.isnan(err_vs_disc) else None,
        "converged_early":        step_converged is not None,
        "converged_at_step":      step_converged,
        "status":                 status,
        "convergence_trace":      converge_log,
    }


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 7 — Single-N orchestrator  (sanity + sweep)
# ═════════════════════════════════════════════════════════════════════════════

def run_single_n(N: int) -> dict:
    """
    Full pipeline for one value of N:
      1. Zero-gravity sanity pass  → verifies joint wiring is correct
      2. Gravity sweep             → measures tip deflection

    If the sanity pass fails, the gravity sweep is skipped and the result is
    marked status="SANITY_FAIL" so it stands out in the summary table.
    """
    print(f"\n{'='*60}")
    print(f"  Phase 1 v2  —  N = {N} segment{'s' if N > 1 else ''}")
    print(f"{'='*60}")

    # ── Pass 0: zero-gravity sanity ───────────────────────────────────────
    print(f"\n  ── Pass 0: zero-gravity sanity ──────────────────────────")
    sanity_ok, sanity_drift = run_sanity_pass(N)

    if not sanity_ok:
        print(f"  [N={N}] ✗ SANITY FAILED — skipping gravity sweep")
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E, GRAVITY)
        return {
            "N":                      N,
            "segment_length_m":       round(L / N, 6),
            "stiffness_K_real":       round(physics_stiffness(E, RADIUS, N, L), 6),
            "stiffness_K_sim":        round(physics_stiffness(E, RADIUS, N, L) * SCALE**4, 4),
            "tip_deflection_sim_m":   None,
            "analytical_eb_m":        round(delta_eb, 6),
            "analytical_discrete_m":  round(delta_eb * (1.0 + 1.0/N)**2, 6),
            "error_vs_discrete_pct":  None,
            "sanity_drift_sim":       round(sanity_drift, 8),
            "sanity_passed":          False,
            "converged_early":        False,
            "converged_at_step":      None,
            "status":                 "SANITY_FAIL",
            "convergence_trace":      [],
        }

    # ── Pass 1: gravity sweep ─────────────────────────────────────────────
    print(f"\n  ── Pass 1: gravity sweep ────────────────────────────────")
    result = run_gravity_sweep(N)
    result["sanity_drift_sim"] = round(sanity_drift, 8)
    result["sanity_passed"]    = True
    return result


# ═════════════════════════════════════════════════════════════════════════════
# SECTION 8 — Main
# ═════════════════════════════════════════════════════════════════════════════

def main() -> None:
    os.makedirs(RESULTS_DIR, exist_ok=True)

    # ── Run the standalone math tests first (in-process, no subprocess) ──
    # We import the test module directly so we don't re-launch Isaac Sim.
    # sys.executable inside Isaac Sim's Python is not a plain python3 binary.
    import importlib.util, unittest
    test_script = os.path.join(SCRIPT_DIR, "test_phase1_v2.py")
    print("\n" + "="*60)
    print("  Running standalone math tests before simulation …")
    print("="*60)
    spec = importlib.util.spec_from_file_location("test_phase1_v2", test_script)
    test_mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(test_mod)
    loader = unittest.TestLoader()
    suite  = loader.loadTestsFromModule(test_mod)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    if not result.wasSuccessful():
        print("\n[ERROR] Math tests FAILED — fix test_phase1_v2.py before running "
              "the simulation.  Aborting.")
        simulation_app.close()
        sys.exit(1)
    print("[OK] All math tests passed.\n")

    # ── Print experiment header ───────────────────────────────────────────
    delta_ref = analytical_deflection(L, RADIUS, DENSITY, E, GRAVITY)
    print("="*60)
    print("  PHASE 1 v2 — Static Deflection Test  (raw USD articulation)")
    print("="*60)
    print(f"  L={L}m  r={RADIUS}m  E={E:.2e}Pa  ρ={DENSITY}kg/m³")
    print(f"  SCALE={SCALE}  SIM_HZ={SIM_HZ}  SIM_SECONDS={SIM_SECONDS}")
    print(f"  Analytical E-B deflection (N→∞): {delta_ref*100:.4f} cm")
    print(f"  N sweep: {N_VALUES}")
    print()

    # ── Main sweep ────────────────────────────────────────────────────────
    all_results: list[dict] = []
    for N in N_VALUES:
        res = run_single_n(N)
        all_results.append(res)

    # ── Save JSON ─────────────────────────────────────────────────────────
    output = {
        "experiment":   "phase1_version2",
        "parameters": {
            "L_m":            L,
            "radius_m":       RADIUS,
            "E_Pa":           E,
            "density_kg_m3":  DENSITY,
            "damping_ratio":  DAMPING_RATIO,
            "scale":          SCALE,
            "sim_hz":         SIM_HZ,
            "sim_seconds":    SIM_SECONDS,
        },
        "analytical_eb_m": round(delta_ref, 6),
        "results":         all_results,
    }
    out_path = os.path.join(RESULTS_DIR, "phase1_v2_results.json")
    with open(out_path, "w") as f:
        json.dump(output, f, indent=2)
    print(f"\n[OK] Results saved → {out_path}")

    # ── Summary table ─────────────────────────────────────────────────────
    print("\n── Summary ────────────────────────────────────────────────────────────")
    hdr = (f"  {'N':>4}  {'K_real':>10}  {'K_sim':>12}  {'Meas(cm)':>10}  "
           f"{'Disc(cm)':>10}  {'EB(cm)':>8}  {'Err%':>7}  {'Sanity':>7}  Status")
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))
    for r in all_results:
        meas  = f"{r['tip_deflection_sim_m']*100:.3f}"  if r['tip_deflection_sim_m'] is not None else "  NaN "
        disc  = f"{r['analytical_discrete_m']*100:.3f}"
        eb    = f"{r['analytical_eb_m']*100:.3f}"
        err   = f"{r['error_vs_discrete_pct']:+.1f}"    if r['error_vs_discrete_pct'] is not None else "  NaN "
        k_r   = f"{r['stiffness_K_real']:.4f}"
        k_s   = f"{r['stiffness_K_sim']:.1f}"
        sane  = "PASS" if r.get("sanity_passed") else "FAIL"
        print(f"  {r['N']:>4}  {k_r:>10}  {k_s:>12}  {meas:>10}  "
              f"{disc:>10}  {eb:>8}  {err:>7}  {sane:>7}  {r['status']}")

    simulation_app.close()

    # ── Generate plot ─────────────────────────────────────────────────────
    plot_script = os.path.join(SCRIPT_DIR, "plot_results_v2.py")
    if os.path.exists(plot_script):
        fig_out = os.path.join(RESULTS_DIR, "phase1_v2_plot.png")
        print(f"\n[INFO] Generating plot → {fig_out}")
        result = subprocess.run(
            ["uv", "run", plot_script, "--output", fig_out],
            cwd=PROJECT_ROOT,
        )
        if result.returncode == 0:
            print(f"[OK]  Plot saved → {fig_out}")
        else:
            print(f"[WARN] Plot script exited with code {result.returncode}. "
                  "Run plot_results_v2.py manually to debug.")
    else:
        print(f"\n[INFO] plot_results_v2.py not yet present — skipping plot generation.")


if __name__ == "__main__":
    import traceback
    try:
        main()
    except Exception as exc:
        traceback.print_exc()
        print(f"\n[FATAL] {exc}")
        simulation_app.close()
        sys.exit(1)
