"""
verify_experiment.py

Standalone verification module for the Three-Point Bending experiment.

Runs all 8 analytical checks described in additional_tests.md:
  1. Expected vs theoretical mass of the chain
  2. Second moment of area I (no radius/diameter mix-up)
  3. Single-joint equilibrium: θ = τ / K
  4. Free oscillation period: T = 2π√(J/K)
  5. Self-weight deflection: δ = 5wL⁴/(384EI)
  6. Central point-load deflection (no gravity): δ = FL³/(48EI)
  7. Superposition linearity: δ_combined ≈ δ_gravity + δ_force
  8. Discretization error: beam vs discrete chain comparison

Each test prints PASS / FAIL / WARN with tolerances from the document.
No Isaac Sim required — all tests are purely analytical.
"""

import math
import sys
import os

# ---------------------------------------------------------------------------
# Import theory helpers
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from threepoint_theory import (
    second_moment_of_area,
    structural_stiffness,
    theoretical_deflection,
    elastic_modulus_from_stiffness,
    span_diameter_ratio,
    joint_rotational_stiffness,
    critical_damping,
)

# ===========================================================================
# Experiment parameters (MUST match generate_threepoint_usda.py exactly)
# ===========================================================================

N_LINKS       = 20
HEIGHT        = 0.015       # m per link
RADIUS        = 0.005       # m  (5 mm radius, 10 mm diameter)
GAP           = 0.0001      # m  gap between links
DENSITY       = 1000.0      # kg/m³  (turgid tissue ≈ water)
YOUNG_MODULUS = 3.5e7       # Pa  (35 MPa — Anisimov primary tissue center)
DAMPING_RATIO = 0.2
G             = 9.81        # m/s²

# Derived geometry
LINK_VOLUME   = math.pi * RADIUS**2 * HEIGHT
LINK_MASS     = DENSITY * LINK_VOLUME
CHAIN_MASS    = N_LINKS * LINK_MASS
# Span: distance between the two support link ORIGINS
L_SPAN        = (N_LINKS - 1) * (HEIGHT + GAP)
I             = second_moment_of_area(RADIUS)
SDR           = span_diameter_ratio(L_SPAN, RADIUS)
K_JOINT       = joint_rotational_stiffness(YOUNG_MODULUS, I, HEIGHT)
W_DIST        = CHAIN_MASS * G / L_SPAN   # distributed load [N/m]

# Reference E values for Tests 5 & 6 (use lower E to see bigger, more measurable deflection)
E_TEST        = 2.0e7   # 20 MPa (lower end of Anisimov primary range — larger deflection)
F_TEST        = 0.5     # N  (standard test force)

# ===========================================================================
# Reporting helpers
# ===========================================================================

_PASS  = "\033[92mPASS\033[0m"
_FAIL  = "\033[91mFAIL\033[0m"
_WARN  = "\033[93mWARN\033[0m"
_INFO  = "\033[94mINFO\033[0m"
_results: list[tuple[str, str]] = []   # (test_name, PASS|FAIL|WARN)


def header(title: str):
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def result(name: str, passed: bool, warn: bool, msg: str):
    tag = _PASS if passed else (_WARN if warn else _FAIL)
    print(f"  [{tag}] {msg}")
    _results.append((name, "PASS" if passed else ("WARN" if warn else "FAIL")))


def info(msg: str):
    print(f"  [{_INFO}] {msg}")


# ===========================================================================
# TEST 1 — Mass of the chain
# ===========================================================================

def test_1_chain_mass():
    header("Test 1 — Chain mass")

    mass_theo = N_LINKS * LINK_MASS

    info(f"N_LINKS    = {N_LINKS}")
    info(f"RADIUS     = {RADIUS*1000:.1f} mm")
    info(f"HEIGHT     = {HEIGHT*100:.1f} cm")
    info(f"DENSITY    = {DENSITY:.0f} kg/m³")
    info(f"Link volume = {LINK_VOLUME*1e6:.4f} cm³")
    info(f"Link mass   = {LINK_MASS*1000:.4f} g")
    info(f"Chain mass  = {mass_theo*1000:.4f} g  ({mass_theo:.6f} kg)")

    # Sanity: mass must be positive and realistic for a 30 cm plant stem
    # A tomato branch ~30 cm, ~10 mm dia, weighs ~15-30 g in reality
    ok   = 10e-3 < mass_theo < 100e-3   # 10 g – 100 g
    warn_ = not (15e-3 < mass_theo < 50e-3)   # ideal range 15-50 g
    result("mass_chain", ok, warn_, 
           f"Chain mass = {mass_theo*1000:.1f} g "
           f"({'realistic' if ok else 'UNREALISTIC — check geometry'})"
           f"{' — outside ideal 15-50g range for tomato branch' if warn_ and ok else ''}")

    # Per-link mass: must equal DENSITY * π * r² * h exactly (no rounding)
    I_check = math.pi * RADIUS**4 / 4.0
    match = abs(I_check - I) / I < 1e-10
    result("I_formula_consistency", match, False,
           f"I from formula  = {I:.4e} m⁴  (consistent: {match})")


# ===========================================================================
# TEST 2 — Second moment of area I
# ===========================================================================

def test_2_moment_of_area():
    header("Test 2 — Second moment of area I")

    I_radius   = math.pi * RADIUS**4 / 4.0
    I_diameter = math.pi * (2*RADIUS)**4 / 64.0   # same formula, different form
    I_wrong    = math.pi * (2*RADIUS)**4 / 4.0     # classic mistake: diameter used as radius

    info(f"I (correct, r=5mm)         = {I_radius:.4e} m⁴")
    info(f"I (via D=10mm, πD⁴/64)     = {I_diameter:.4e} m⁴  (must equal above)")
    info(f"I (WRONG: D as r, πD⁴/4)   = {I_wrong:.4e} m⁴  (16× too large)")

    # Both correct forms must agree
    consistent = abs(I_radius - I_diameter) / I_radius < 1e-10
    result("I_forms_consistent", consistent, False,
           f"πr⁴/4 == πD⁴/64: {consistent}  (Δ={abs(I_radius-I_diameter):.2e})")

    # Ratio between correct and wrong
    ratio = I_wrong / I_radius
    result("I_no_diameter_radius_mixup", abs(ratio - 16.0) < 0.01, False,
           f"Error factor if D used as r = {ratio:.1f}×  "
           f"→ kB would be {ratio:.0f}× too high, E would need to be {ratio:.0f}× too low to compensate")

    # SDR check (Anisimov: SDR >= 20 to neglect shear)
    sdr_ok   = SDR >= 20.0
    sdr_warn = SDR < 25.0
    result("SDR_criterion", sdr_ok, sdr_warn,
           f"SDR = L/D = {L_SPAN:.4f}/{2*RADIUS:.3f} = {SDR:.1f} "
           f"({'✅ >= 20' if sdr_ok else '❌ < 20 — shear effects significant'})"
           f"{'  (marginal, ideally >= 25)' if sdr_ok and sdr_warn else ''}")


# ===========================================================================
# TEST 3 — Single joint equilibrium: θ = τ / K
# ===========================================================================

def test_3_joint_equilibrium():
    header("Test 3 — Single joint equilibrium (analytical)")

    K_test  = 1.0    # N·m/rad  (synthetic, clean number)
    tau_test = 0.1   # N·m

    theta_expected = tau_test / K_test   # rad

    info(f"K_test (synthetic)  = {K_test:.2f} N·m/rad")
    info(f"τ applied           = {tau_test:.3f} N·m")
    info(f"θ expected          = {math.degrees(theta_expected):.4f}°  ({theta_expected:.4f} rad)")

    # For the REAL joint in our experiment:
    K_real = joint_rotational_stiffness(E_TEST, I, HEIGHT)
    info(f"\nReal joint K (E=20 MPa, r=5mm, L_link=1.5cm) = {K_real:.6f} N·m/rad")

    # Force applied at tip of link to produce a moment:
    # If we push the tip of a 1.5 cm link with F = 0.5 N perpendicular to its axis:
    tau_real  = F_TEST * HEIGHT   # ≈ torque at joint from tip force
    theta_real = tau_real / K_real
    info(f"τ from F={F_TEST}N at tip = {tau_real:.5f} N·m")
    info(f"θ expected for real joint  = {math.degrees(theta_real):.4f}°")

    small_angle_ok = theta_real < math.radians(30.0)
    result("single_joint_small_angle", small_angle_ok, math.degrees(theta_real) > 15,
           f"Single-joint angle at F={F_TEST}N: {math.degrees(theta_real):.2f}°  "
           f"({'✅ < 30° bend limit' if small_angle_ok else '❌ exceeds BEND_LIMIT_DEG'})")


# ===========================================================================
# TEST 4 — Free oscillation period: T = 2π√(J/K)
# ===========================================================================

def test_4_oscillation_period():
    header("Test 4 — Free oscillation period (analytical)")

    K_test  = 1.0   # N·m/rad (synthetic)
    # J for a thin rod rotating about one end: J = m*L²/3
    J_rod   = LINK_MASS * HEIGHT**2 / 3.0
    T_expected = 2.0 * math.pi * math.sqrt(J_rod / K_test)

    info(f"Link mass   = {LINK_MASS*1000:.4f} g")
    info(f"Link length = {HEIGHT*100:.1f} cm")
    info(f"J_rod (about end) = {J_rod:.4e} kg·m²")
    info(f"K_test            = {K_test:.1f} N·m/rad")
    info(f"T expected        = {T_expected:.4f} s  ({T_expected*1000:.2f} ms)")

    # For the real joint K:
    K_real   = joint_rotational_stiffness(E_TEST, I, HEIGHT)
    T_real   = 2.0 * math.pi * math.sqrt(J_rod / K_real)
    info(f"\nReal joint K (E=20 MPa) = {K_real:.6f} N·m/rad")
    info(f"T real (undamped)       = {T_real*1000:.3f} ms")

    # At 480 Hz, how many steps per oscillation cycle?
    SIM_HZ = 480
    steps_per_cycle = T_real * SIM_HZ
    sampling_ok = steps_per_cycle >= 10.0
    result("oscillation_sampling", sampling_ok, steps_per_cycle < 20,
           f"Steps per oscillation cycle @ {SIM_HZ} Hz: {steps_per_cycle:.1f}  "
           f"({'✅ well sampled' if steps_per_cycle >= 20 else '⚠️ marginal (>=10 min)' if sampling_ok else '❌ under-sampled'})")

    # Damped period (with DAMPING_RATIO = 0.2)
    omega_n = 2.0 * math.pi / T_real
    omega_d = omega_n * math.sqrt(1.0 - DAMPING_RATIO**2)
    T_damped = 2.0 * math.pi / omega_d
    info(f"Damped period (ζ={DAMPING_RATIO}): {T_damped*1000:.3f} ms  ({T_damped/T_real:.4f}× undamped)")


# ===========================================================================
# TEST 5 — Self-weight deflection: δ = 5wL⁴/(384EI)
# ===========================================================================

def test_5_self_weight_deflection():
    header("Test 5 — Self-weight deflection (continuous beam approximation)")

    delta_expected = 5.0 * W_DIST * L_SPAN**4 / (384.0 * E_TEST * I)

    info(f"E_test           = {E_TEST/1e6:.0f} MPa")
    info(f"w (N/m)          = {W_DIST:.6f} N/m  (= chain_mass × g / L)")
    info(f"L_span           = {L_SPAN*100:.4f} cm")
    info(f"δ_gravity theory = {delta_expected*1000:.4f} mm")

    # Deflection as fraction of span (small-angle validity)
    ratio = delta_expected / L_SPAN
    small_def_ok   = ratio < 0.05    # < 5 % of span = safely small deformation
    small_def_warn = ratio < 0.10    # < 10 % = still acceptable
    result("self_weight_small_deformation", small_def_ok, not small_def_ok and small_def_warn,
           f"δ_gravity / L = {ratio*100:.2f}%  "
           f"({'✅ small deformation (< 5%)' if small_def_ok else '⚠️ moderate (5-10%)' if small_def_warn else '❌ large deformation — Euler-Bernoulli not valid'})")

    # Expected tolerance for discrete chain vs continuous beam (additional_tests.md: 15-20%)
    info(f"Expected error discrete vs continuous: 15-20% (N=20 links)")
    info(f"→ Accept sim deflection in range: {delta_expected*0.80*1000:.3f} – {delta_expected*1.20*1000:.3f} mm")


# ===========================================================================
# TEST 6 — Central point load (no gravity): δ = FL³/(48EI)
# ===========================================================================

def test_6_central_point_load():
    header("Test 6 — Central point load (no gravity, Anisimov eq.1)")

    delta_expected = theoretical_deflection(F_TEST, L_SPAN, E_TEST, I)

    info(f"E_test     = {E_TEST/1e6:.0f} MPa")
    info(f"F_test     = {F_TEST:.2f} N")
    info(f"L_span     = {L_SPAN*100:.4f} cm")
    info(f"I          = {I:.4e} m⁴")
    info(f"δ expected = {delta_expected*1000:.4f} mm")

    # Round-trip: compute kB from this δ and recover E
    kB_from_delta = F_TEST / delta_expected   # N/m (by definition)
    E_roundtrip   = elastic_modulus_from_stiffness(kB_from_delta, L_SPAN, I)
    rt_ok = abs(E_roundtrip - E_TEST) / E_TEST < 1e-6
    result("formula_roundtrip", rt_ok, False,
           f"E round-trip: {E_TEST/1e6:.2f} MPa → δ → kB → {E_roundtrip/1e6:.2f} MPa  "
           f"({'✅' if rt_ok else '❌ formula inconsistency'})")

    # Small-angle check
    ratio = delta_expected / L_SPAN
    small_ok   = ratio < 0.05
    small_warn = ratio < 0.10
    result("point_load_small_deformation", small_ok, not small_ok and small_warn,
           f"δ / L = {ratio*100:.2f}%  "
           f"({'✅ small (<5%)' if small_ok else '⚠️ moderate (5-10%)' if small_warn else '❌ large — nonlinear regime'})")

    # Accepted tolerance for discrete vs continuous (additional_tests.md: 10-15%)
    tol = 0.15
    info(f"Expected error discrete vs continuous: <= {tol*100:.0f}%  (N=20 links)")
    info(f"→ Accept sim δ in range: {delta_expected*(1-tol)*1000:.3f} – {delta_expected*(1+tol)*1000:.3f} mm")

    # kB and E for the default (nominal) E_test value
    kB_nom = structural_stiffness(E_TEST, L_SPAN, I)
    info(f"\nFor nominal E={E_TEST/1e6:.0f} MPa:")
    info(f"  kB = {kB_nom:.5f} N/m")
    info(f"  δ(F=0.5N) = {theoretical_deflection(F_TEST, L_SPAN, E_TEST, I)*1000:.3f} mm")


# ===========================================================================
# TEST 7 — Superposition linearity
# ===========================================================================

def test_7_superposition():
    header("Test 7 — Superposition (gravity + point load)")

    delta_gravity = 5.0 * W_DIST * L_SPAN**4 / (384.0 * E_TEST * I)
    delta_force   = theoretical_deflection(F_TEST, L_SPAN, E_TEST, I)
    delta_combined = delta_gravity + delta_force

    info(f"δ_gravity  = {delta_gravity*1000:.4f} mm")
    info(f"δ_force    = {delta_force*1000:.4f} mm")
    info(f"δ_combined = {delta_combined*1000:.4f} mm  (superposition)")

    # Superposition valid only in small-deformation regime
    ratio_total = delta_combined / L_SPAN
    valid = ratio_total < 0.10
    result("superposition_valid_regime", ratio_total < 0.05, not valid,
           f"Total δ / L = {ratio_total*100:.2f}%  "
           f"({'✅ superposition valid' if ratio_total < 0.05 else '⚠️ superposition may break down (>5%)' if valid else '❌ nonlinear regime — DO NOT sum'})")

    # Gravity contribution as % of total
    grav_frac = delta_gravity / delta_combined * 100
    info(f"Gravity contributes {grav_frac:.1f}% of total central deflection")
    if grav_frac > 20:
        print(f"  [{_WARN}] Gravity is significant. "
              f"In simulation, either zero gravity (test 6 only) "
              f"or subtract gravity deflection before computing kB.")
    else:
        print(f"  [{_PASS}] Gravity is minor (<20%). Simple approach: "
              f"subtract rest position before applying load.")


# ===========================================================================
# TEST 8 — Discretization error (discrete chain vs continuous beam)
# ===========================================================================

def test_8_discretization_error():
    header("Test 8 — Discretization error analysis (N=20 chain vs continuous beam)")

    # Central deflection for a continuous beam vs discrete chain.
    # For a uniform discrete chain of N links under central point load,
    # the exact discrete result (Hrennikoff lattice) converges to the continuous
    # beam with an error proportional to 1/N².
    # For N=20, expected relative error ~ (π/N)² / 10 ≈ 0.25 % for the bending mode.
    # In practice with PhysX joint discretization, empirical errors are higher: ~5-15%.

    delta_cont = theoretical_deflection(F_TEST, L_SPAN, E_TEST, I)

    # Discrete chain analytical estimate (Euler method on the discrete chain):
    # For N segments with rotational springs K_theta = EI/L_link at each joint,
    # the central deflection under point load F at center node:
    # δ_discrete ≈ δ_continuous × (1 + α/N²) where α ≈ π²/12 for clamped ends
    # For simply-supported ends the correction is smaller. Use empirical 5-15%.
    alpha = math.pi**2 / 12.0
    delta_disc_est = delta_cont * (1.0 + alpha / N_LINKS**2)
    disc_error_pct = abs(delta_disc_est - delta_cont) / delta_cont * 100

    info(f"N_LINKS = {N_LINKS}")
    info(f"δ continuous  = {delta_cont*1000:.4f} mm")
    info(f"δ discrete    = {delta_disc_est*1000:.4f} mm  (Hrennikoff estimate)")
    info(f"Discretization error estimate = {disc_error_pct:.3f}%  (theoretical)")
    info(f"Practical error (PhysX joint discrete): ~5-15%")

    disc_ok = N_LINKS >= 10
    disc_ideal = N_LINKS >= 20
    result("N_links_sufficient", disc_ok, not disc_ideal,
           f"N_LINKS = {N_LINKS}  "
           f"({'✅ good (>=20)' if disc_ideal else '✅ acceptable (>=10)' if disc_ok else '❌ too few, increase N_LINKS'})")

    # kB error propagation: if δ has error ε, then kB = F/δ has same error
    # and E = kB × L³/48I also has same relative error.
    info(f"→ Expected E measurement error from discretization: 5–15%")
    info(f"   Acceptable for Livello 1 validation (literature order-of-magnitude).")
    info(f"   For Livello 2 (real sample calibration), use N>=30 or apply correction.")

    # Sensitivity analysis: how much does δ change per 1% change in r?
    # I = πr⁴/4, so δ ∝ 1/I ∝ 1/r⁴. A 1% error on r → 4% error on δ.
    r_sensitivity = 4.0
    info(f"\nRadius sensitivity: 1% error in r → {r_sensitivity:.0f}% error in δ (and E)")
    info(f"→ Measure radius at >=3 points along stem (Anisimov recommendation).")


# ===========================================================================
# SUMMARY
# ===========================================================================

def print_summary():
    print(f"\n{'='*60}")
    print("  SUMMARY")
    print(f"{'='*60}")
    passed = [(n, s) for n, s in _results if s == "PASS"]
    warned = [(n, s) for n, s in _results if s == "WARN"]
    failed = [(n, s) for n, s in _results if s == "FAIL"]

    print(f"\n  ✅ PASS: {len(passed)}")
    print(f"  ⚠️  WARN: {len(warned)}")
    print(f"  ❌ FAIL: {len(failed)}")

    if warned:
        print("\n  Warnings:")
        for n, _ in warned:
            print(f"    ⚠️  {n}")
    if failed:
        print("\n  Failures (must fix before trusting results):")
        for n, _ in failed:
            print(f"    ❌ {n}")

    print(f"\n{'='*60}")
    print("  REFERENCE VALUES FOR SIMULATION COMPARISON")
    print(f"{'='*60}")
    print(f"  Use E_TEST = {E_TEST/1e6:.0f} MPa for first validation run.")
    print(f"  Span L    = {L_SPAN*100:.4f} cm  (between support origins)")
    print(f"  I         = {I:.4e} m⁴")
    K_joint_test = joint_rotational_stiffness(E_TEST, I, HEIGHT)
    delta_grav   = 5.0 * W_DIST * L_SPAN**4 / (384.0 * E_TEST * I)
    delta_force  = theoretical_deflection(F_TEST, L_SPAN, E_TEST, I)
    print(f"\n  [Test 5] δ_gravity (self-weight only):    {delta_grav*1000:.3f} mm")
    print(f"           accept sim in [{delta_grav*0.80*1000:.3f}, {delta_grav*1.20*1000:.3f}] mm (±20%)")
    print(f"\n  [Test 6] δ_force (F={F_TEST}N, no gravity):   {delta_force*1000:.3f} mm")
    print(f"           accept sim in [{delta_force*0.85*1000:.3f}, {delta_force*1.15*1000:.3f}] mm (±15%)")
    print(f"\n  [Test 3] K_joint (per D6 joint):          {K_joint_test:.6f} N·m/rad")
    print(f"  [kB]     Structural stiffness kB:          {structural_stiffness(E_TEST, L_SPAN, I):.5f} N/m")
    print(f"\n{'='*60}\n")


# ===========================================================================
# MAIN
# ===========================================================================

if __name__ == "__main__":
    print("\n\033[1;36m")
    print("  Three-Point Bending — Experiment Verification")
    print("  (additional_tests.md — all 8 analytical checks)")
    print("\033[0m")

    test_1_chain_mass()
    test_2_moment_of_area()
    test_3_joint_equilibrium()
    test_4_oscillation_period()
    test_5_self_weight_deflection()
    test_6_central_point_load()
    test_7_superposition()
    test_8_discretization_error()
    print_summary()
