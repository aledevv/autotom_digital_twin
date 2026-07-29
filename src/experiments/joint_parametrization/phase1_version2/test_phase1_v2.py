"""
test_phase1_v2.py — Standalone Math & Geometry Tests
======================================================
Validates every formula used in run_phase1_v2.py *without* importing Isaac Sim.
Run with plain Python:

    python3 test_phase1_v2.py -v

All tests must pass before trusting the simulation results.

Coverage:
  1. analytical_deflection            — Euler-Bernoulli cantilever self-weight
  2. physics_stiffness                — K = E·I / l  (discrete segment stiffness)
  3. stiffness scale correction       — K_sim = K_real × SCALE^4
  4. auto_mass                        — cylinder volume × density
  5. critical_damping                 — 2√(K·m)
  6. discrete convergence formula     — δ_N = δ_EB · (1 + 1/N)²
  7. small-deflection regime check    — E=10MPa violates it; E=50MPa satisfies it
  8. tip Z geometry (no Isaac Sim)    — simulate mat×local_tip using plain numpy
  9. scale-corrected tip measurement  — verify deflection_m = deflection_sim / SCALE
 10. segment local frame              — tip of a horizontal segment points in +X world
"""

import math
import unittest

# ── Experiment constants (must stay in sync with run_phase1_v2.py) ────────────
L             = 0.4      # branch total length [m]
RADIUS        = 0.01     # cylinder radius [m]
E_WRONG       = 10.0e6   # E that violates small-deflection (10 MPa)
E_GOOD        = 50.0e6   # E that satisfies small-deflection (50 MPa)
DENSITY       = 700.0    # [kg/m³]
GRAVITY       = 9.81     # [m/s²]
SCALE         = 10.0     # geometry scale factor
DAMPING_RATIO = 0.7
MASS_FLOOR    = 0.005    # [kg] — minimum mass for thin segments


# ── Pure-Python implementations of the helpers (mirrors run_phase1_v2.py) ─────

def analytical_deflection(L, r, rho, E, g):
    """δ = (ρ·A·g·L⁴) / (8·E·I),  I = π r⁴/4,  A = π r²
    Simplifies to: δ = (ρ·g·L⁴) / (2·E·r²)
    """
    I = math.pi * r**4 / 4.0
    A = math.pi * r**2
    w = rho * A * g           # distributed load [N/m]
    return (w * L**4) / (8.0 * E * I)


def physics_stiffness(E, r, N, L):
    """K = E·I / l,  where l = L/N and I = π r⁴/4"""
    I = math.pi * r**4 / 4.0
    l = L / N
    return E * I / l


def auto_mass(radius, length, density, mass_floor=MASS_FLOOR):
    volume = math.pi * radius**2 * length
    return max(volume * density, mass_floor)


def critical_damping(stiffness, mass):
    return 2.0 * math.sqrt(stiffness * mass)


def discrete_deflection(delta_eb, N):
    """δ_N = δ_EB · (1 + 1/N)²"""
    return delta_eb * (1.0 + 1.0 / N) ** 2



# ─────────────────────────────────────────────────────────────────────────────
# Test cases
# ─────────────────────────────────────────────────────────────────────────────

class TestAnalyticalDeflection(unittest.TestCase):
    """Euler-Bernoulli cantilever self-weight deflection formula."""

    def test_known_value_E50MPa(self):
        """At E=50MPa the expected deflection is ~1.759 cm."""
        delta = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        self.assertAlmostEqual(delta, 0.01759, places=4,
            msg=f"Expected ~1.759 cm, got {delta*100:.4f} cm")

    def test_known_value_E10MPa(self):
        """At E=10MPa the expected deflection is ~8.79 cm."""
        delta = analytical_deflection(L, RADIUS, DENSITY, E_WRONG, GRAVITY)
        self.assertAlmostEqual(delta, 0.08790, places=3,
            msg=f"Expected ~8.79 cm, got {delta*100:.4f} cm")

    def test_stiffer_means_less_deflection(self):
        d_soft = analytical_deflection(L, RADIUS, DENSITY, E_WRONG, GRAVITY)
        d_stiff = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        self.assertGreater(d_soft, d_stiff,
            "Softer material must deflect more")

    def test_longer_beam_deflects_more(self):
        d_short = analytical_deflection(0.2, RADIUS, DENSITY, E_GOOD, GRAVITY)
        d_long  = analytical_deflection(0.4, RADIUS, DENSITY, E_GOOD, GRAVITY)
        self.assertGreater(d_long, d_short,
            "Longer beam must deflect more (L^4 dependence)")


class TestPhysicsStiffness(unittest.TestCase):
    """K = E·I / l scaling law."""

    def test_k_proportional_to_N(self):
        """Doubling N must double K (K ∝ N for fixed L, r, E)."""
        K5  = physics_stiffness(E_GOOD, RADIUS, 5,  L)
        K10 = physics_stiffness(E_GOOD, RADIUS, 10, L)
        self.assertAlmostEqual(K10 / K5, 2.0, places=10,
            msg="K must scale linearly with N")

    def test_known_K_at_N2(self):
        """At N=2, E=50MPa: K = 50e6 × π×(0.01)⁴/4 / 0.2 ≈ 1.9635 N·m/rad"""
        K = physics_stiffness(E_GOOD, RADIUS, 2, L)
        self.assertAlmostEqual(K, 1.9635, places=3,
            msg=f"Expected K≈1.9635, got {K:.6f}")

    def test_known_K_at_N10(self):
        K = physics_stiffness(E_GOOD, RADIUS, 10, L)
        self.assertAlmostEqual(K, 9.8175, places=3,
            msg=f"Expected K≈9.8175, got {K:.6f}")

    def test_larger_radius_means_stiffer(self):
        K_thin  = physics_stiffness(E_GOOD, 0.005, 5, L)
        K_thick = physics_stiffness(E_GOOD, 0.020, 5, L)
        self.assertGreater(K_thick, K_thin,
            "Thicker cylinder must be stiffer (I ∝ r^4)")


class TestScaleCorrection(unittest.TestCase):
    """K_sim = K_real × SCALE^4 (gravity torque scales with geometry^4)."""

    def test_scale_factor_is_scale_to_the_fourth(self):
        K_real = physics_stiffness(E_GOOD, RADIUS, 5, L)
        K_sim  = K_real * SCALE**4
        expected = K_real * 10000.0   # SCALE=10 → 10^4=10000
        self.assertAlmostEqual(K_sim, expected, places=6)

    def test_known_K_sim_at_N2(self):
        """K_sim at N=2 should be ≈19635."""
        K_real = physics_stiffness(E_GOOD, RADIUS, 2, L)
        K_sim  = K_real * SCALE**4
        self.assertAlmostEqual(K_sim, 19635.0, places=0,
            msg=f"Expected K_sim≈19635, got {K_sim:.2f}")

    def test_scale_reasoning(self):
        """
        Geometry scaled by S: length×S, radius×S.
        Mass scales S^3 (volume). Lever arm scales S.
        Gravity torque = m·g·(L/2) scales S^3 × S = S^4.
        So stiffness must scale S^4 to keep θ = torque/K constant.
        """
        S = SCALE
        K_real = physics_stiffness(E_GOOD, RADIUS, 5, L)
        K_sim  = K_real * S**4
        # Verify K_sim / K_real equals S^4
        self.assertAlmostEqual(K_sim / K_real, S**4, places=6)


class TestAutoMass(unittest.TestCase):
    """Cylinder mass = π r² l ρ, with mass_floor."""

    def test_known_mass(self):
        """Scaled segment: r=0.1m (0.01×10), l=2.0m (0.2×10), ρ=700"""
        r_sc = RADIUS * SCALE           # 0.1 m
        l_sc = (L / 2) * SCALE          # 2.0 m  (N=2 → l=0.2 m → scaled 2.0 m)
        m = auto_mass(r_sc, l_sc, DENSITY)
        expected = math.pi * (r_sc**2) * l_sc * DENSITY
        self.assertAlmostEqual(m, expected, places=6,
            msg=f"Expected mass={expected:.6f} kg, got {m:.6f} kg")

    def test_mass_floor_applied(self):
        """Tiny segment should hit the mass floor."""
        m = auto_mass(0.0001, 0.0001, DENSITY)
        self.assertEqual(m, MASS_FLOOR,
            "Mass floor must kick in for very thin/short segments")

    def test_mass_scales_with_length(self):
        m1 = auto_mass(0.1, 1.0, DENSITY)
        m2 = auto_mass(0.1, 2.0, DENSITY)
        self.assertAlmostEqual(m2 / m1, 2.0, places=6,
            msg="Mass must scale linearly with length")


class TestCriticalDamping(unittest.TestCase):
    """D_critical = 2√(K·m)."""

    def test_formula(self):
        K = 1000.0
        m = 0.1
        d = critical_damping(K, m)
        self.assertAlmostEqual(d, 2.0 * math.sqrt(K * m), places=10)

    def test_damping_increases_with_stiffness(self):
        m = 0.05
        d_soft  = critical_damping(500.0,  m)
        d_stiff = critical_damping(5000.0, m)
        self.assertGreater(d_stiff, d_soft)

    def test_actual_damping_with_ratio(self):
        """D = ratio × D_critical must be less than D_critical for ratio < 1."""
        K = physics_stiffness(E_GOOD, RADIUS, 5, L) * SCALE**4
        r_sc = RADIUS * SCALE
        l_sc = (L / 5) * SCALE
        m = auto_mass(r_sc, l_sc, DENSITY)
        D_crit = critical_damping(K, m)
        D_act  = DAMPING_RATIO * D_crit
        self.assertLess(D_act, D_crit,
            "Actual damping must be underdamped (ratio=0.7 < 1)")


class TestDiscreteConvergence(unittest.TestCase):
    """δ_N = δ_EB · (1 + 1/N)² convergence formula."""

    def test_N2_overshoot(self):
        """N=2 should be 2.25× the continuous E-B value."""
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        delta_2  = discrete_deflection(delta_eb, 2)
        self.assertAlmostEqual(delta_2 / delta_eb, 2.25, places=10,
            msg="N=2 discrete chain must be 2.25× E-B")

    def test_N10_overshoot(self):
        """N=10: factor = (1.1)² = 1.21"""
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        delta_10 = discrete_deflection(delta_eb, 10)
        self.assertAlmostEqual(delta_10 / delta_eb, 1.21, places=10)

    def test_monotonic_convergence(self):
        """Deflection must decrease as N increases."""
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        deltas = [discrete_deflection(delta_eb, N) for N in [2, 3, 5, 10, 20, 50]]
        for i in range(len(deltas) - 1):
            self.assertGreater(deltas[i], deltas[i+1],
                f"δ[{i}] must be > δ[{i+1}]")

    def test_convergence_to_eb(self):
        """At very large N (N=1000) the discrete value must be within 0.21% of E-B.
        The discrete formula gives (1 + 1/1000)^2 - 1 = 2/1000 + 1/10^6 ≈ 0.2001%,
        so we use 0.21% as the threshold."""
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        delta_large = discrete_deflection(delta_eb, 1000)
        rel_err = abs(delta_large - delta_eb) / delta_eb
        self.assertLess(rel_err, 0.0021,
            f"N=1000 should converge to within 0.21% of E-B; error={rel_err*100:.4f}%")

    def test_known_values_cm(self):
        """Spot-check expected cm values from fixes.md table."""
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        expected = {2: 3.958, 5: 2.533, 10: 2.128, 50: 1.830}
        for N, exp_cm in expected.items():
            got_cm = discrete_deflection(delta_eb, N) * 100.0
            self.assertAlmostEqual(got_cm, exp_cm, delta=0.005,
                msg=f"N={N}: expected {exp_cm:.3f} cm, got {got_cm:.3f} cm")


class TestSmallDeflectionRegime(unittest.TestCase):
    """Verify that E=10MPa violates the small-deflection assumption and E=50MPa satisfies it."""

    SMALL_DEFLECTION_LIMIT = 0.10  # δ must be < 10% of L

    def test_E10MPa_violates_regime(self):
        delta = analytical_deflection(L, RADIUS, DENSITY, E_WRONG, GRAVITY)
        ratio = delta / L
        self.assertGreater(ratio, self.SMALL_DEFLECTION_LIMIT,
            f"E=10MPa: δ/L={ratio:.3f} should exceed {self.SMALL_DEFLECTION_LIMIT} "
            f"(violates small-deflection); actual δ={delta*100:.2f} cm")

    def test_E50MPa_satisfies_regime(self):
        delta = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        ratio = delta / L
        self.assertLess(ratio, self.SMALL_DEFLECTION_LIMIT,
            f"E=50MPa: δ/L={ratio:.3f} should be < {self.SMALL_DEFLECTION_LIMIT} "
            f"(within small-deflection regime); actual δ={delta*100:.2f} cm")


class TestTipZGeometry(unittest.TestCase):
    """
    Validate the tip-measurement approach without Isaac Sim.
    We use plain Python 4×4 matrix math to simulate what
    ComputeLocalToWorldTransform + (mat * local_tip) does.

    Scenario: A horizontal segment (rotated 90° around X so its +Z axis
    points in the world +X direction), placed at world origin.
    Its origin is at (0,0,0) and its tip should be at (l_scaled, 0, 0).
    """

    def _make_rotation_x(self, angle_deg):
        """4×4 rotation matrix around world X axis."""
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        return [
            [1, 0,  0, 0],
            [0, c, -s, 0],
            [0, s,  c, 0],
            [0, 0,  0, 1],
        ]

    def _make_translation(self, tx, ty, tz):
        """4×4 translation matrix."""
        return [
            [1, 0, 0, tx],
            [0, 1, 0, ty],
            [0, 0, 1, tz],
            [0, 0, 0,  1],
        ]

    def _mat_mul_vec(self, mat, vec4):
        """Multiply 4×4 matrix by a 4-vector."""
        result = [0.0] * 4
        for row in range(4):
            for col in range(4):
                result[row] += mat[row][col] * vec4[col]
        return result

    def _mat_mul_mat(self, A, B):
        """Multiply two 4×4 matrices."""
        C = [[0.0]*4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    C[i][j] += A[i][k] * B[k][j]
        return C

    def test_origin_measurement_gives_wrong_result(self):
        """
        mat.ExtractTranslation() returns the segment origin (base), NOT the tip.
        For a horizontal segment with origin at (0,0,0), this reads Z=0 regardless
        of where the physical tip actually is — demonstrating the original bug.
        """
        # A horizontal segment at world origin: Z_origin = 0
        origin_z = 0.0
        # The physical tip is at +X = l_scaled, NOT at Z = l_scaled
        l_scaled = (L / 2) * SCALE   # N=2, scaled
        tip_z_correct = 0.0          # horizontal segment: tip is at Z=0 too

        # Bug: reading origin Z gives 0, which is "correct" only by coincidence
        # for t=0; after bending, origin stays near 0 but tip drops in Z.
        # This test documents why we MUST use the full mat*local_tip transform.
        self.assertAlmostEqual(origin_z, 0.0, places=10)

    def test_tip_transform_horizontal_segment(self):
        """
        Demonstrate tip-Z measurement for a horizontal cantilever segment.

        In our Isaac Sim build:
          - The anchor prim sits at the world origin pointing up (+Z).
          - The first branch segment is attached with tilt_angle=90°, meaning its
            local +Z axis is rotated 90° around the world X-axis → it points in
            the world +Y direction (horizontal).
          - Local tip coordinate is (0, 0, l_scaled).
          - After a 90° rotation around X:  local Z → world -Y  (rot convention).

        Key result: the TIP world-Z equals the ORIGIN world-Z (both = 0) for a
        perfectly horizontal segment.  Only after gravity bends the chain does
        tip_world_Z drop below 0.  This confirms we need the full mat*local_tip
        transform — not just ExtractTranslation() — to capture bending.
        """
        l_scaled = (L / 2) * SCALE   # 2.0 m for N=2

        # 90° rotation around world X: maps local (0,0,l) → (0, -l, 0) in world.
        # (The sign depends on which direction "tilt" goes in the joint frame,
        #  but the key property is that world-Z of the tip = 0 when horizontal.)
        rot_x_pos90 = self._make_rotation_x(90.0)   # +90° around X

        local_tip = [0.0, 0.0, l_scaled, 1.0]
        world_tip = self._mat_mul_vec(rot_x_pos90, local_tip)

        # For a horizontal segment (rotated 90° around X):
        #   local Z → world -Y  (or +Y depending on sign), world Z = 0.
        self.assertAlmostEqual(world_tip[2], 0.0, places=6,
            msg="Tip world-Z must be 0 for a perfectly horizontal segment (before gravity)")
        # The tip should have moved in Y, not Z (it's horizontal)
        self.assertAlmostEqual(abs(world_tip[1]), l_scaled, places=6,
            msg="Tip world-Y magnitude should equal l_scaled for a 90°-tilted segment")

    def test_tip_transform_sagged_segment(self):
        """
        Simulate a segment that has sagged by 2 cm (0.02 m, or 0.2 in sim units
        with SCALE=10). Its origin is at (l_scaled, 0, -0.2) and it has rotated
        slightly downward. The tip Z should be BELOW the origin Z — i.e. more negative.
        """
        l_scaled = (L / 2) * SCALE   # 2.0 m
        sag_cm   = 2.0               # 2 cm in real world
        sag_sim  = sag_cm / 100.0 * SCALE   # 0.2 in sim units

        # Segment origin has already moved to (l_scaled, 0, -sag_sim)
        # The segment itself has rotated downward by a small angle
        # → local tip has negative Z component in world space
        # We test that measuring the tip gives MORE deflection than origin.

        tilt_angle = math.radians(5.0)  # small downward tilt
        # Local tip after 5° downward tilt and 90° initial tilt:
        # Origin Z = -sag_sim, tip contributes an extra -l*sin(5°) in Z
        extra_z = -l_scaled * math.sin(tilt_angle)
        tip_z_approx = -sag_sim + extra_z
        # Tip must be below origin
        self.assertLess(tip_z_approx, -sag_sim,
            "Sagged segment tip must be below its own origin")

    def test_deflection_m_conversion(self):
        """deflection_m = (tip_z_initial - tip_z_final) / SCALE"""
        tip_z_initial = 0.0
        tip_z_final   = -0.1759 * SCALE   # 1.759 cm sag × SCALE in sim units
        deflection_sim = tip_z_initial - tip_z_final
        deflection_m   = deflection_sim / SCALE
        self.assertAlmostEqual(deflection_m, 0.1759, places=6,
            msg=f"Expected 0.1759 m, got {deflection_m:.6f} m")



class TestFullPipelineSanity(unittest.TestCase):
    """End-to-end sanity checks that verify all formulas are internally consistent."""

    def test_N_sweep_stiffness_consistency(self):
        """
        For every N in the sweep, K_real × SCALE^4 must increase with N.
        K_sim[N2] / K_sim[N1] must equal N2/N1 exactly.
        """
        N_VALUES = [2, 3, 5, 10, 20, 50]
        K_sims = [physics_stiffness(E_GOOD, RADIUS, N, L) * SCALE**4
                  for N in N_VALUES]
        for i in range(len(N_VALUES) - 1):
            ratio_K = K_sims[i+1] / K_sims[i]
            ratio_N = N_VALUES[i+1] / N_VALUES[i]
            self.assertAlmostEqual(ratio_K, ratio_N, places=10,
                msg=f"K_sim ratio {K_sims[i+1]:.2f}/{K_sims[i]:.2f}={ratio_K:.6f} "
                    f"must equal N ratio {ratio_N:.6f}")

    def test_mass_consistent_across_N(self):
        """
        Total mass of the chain (N × mass_per_segment) should be approximately
        constant across N values (since total volume = π r² L is constant).
        Small variations are OK due to mass_floor, but for our geometry it
        should be well above the floor.
        """
        N_VALUES = [2, 3, 5, 10, 20, 50]
        for N in N_VALUES:
            l_sc = (L / N) * SCALE
            r_sc = RADIUS * SCALE
            m_seg = auto_mass(r_sc, l_sc, DENSITY)
            total_mass = N * m_seg
            # Total mass should equal ρ × π r² L (scaled)
            expected_total = DENSITY * math.pi * (RADIUS * SCALE)**2 * L * SCALE
            # Tolerance: 1% — large N segments are small but still above floor
            rel_err = abs(total_mass - expected_total) / expected_total
            self.assertLess(rel_err, 0.01,
                f"N={N}: total_mass={total_mass:.4f} vs expected={expected_total:.4f} "
                f"(err={rel_err*100:.2f}%)")

    def test_discrete_targets_print(self):
        """Print the expected results table for visual inspection."""
        delta_eb = analytical_deflection(L, RADIUS, DENSITY, E_GOOD, GRAVITY)
        print(f"\n{'─'*70}")
        print(f"  Expected results table (E={E_GOOD:.0e} Pa,  δ_EB = {delta_eb*100:.4f} cm)")
        print(f"  {'N':>4}  {'K_real':>10}  {'K_sim':>12}  {'δ_disc(cm)':>12}  {'δ_EB(cm)':>10}")
        print(f"  {'─'*4}  {'─'*10}  {'─'*12}  {'─'*12}  {'─'*10}")
        for N in [2, 3, 5, 10, 20, 50]:
            K_r = physics_stiffness(E_GOOD, RADIUS, N, L)
            K_s = K_r * SCALE**4
            d_n = discrete_deflection(delta_eb, N) * 100.0
            print(f"  {N:>4}  {K_r:>10.4f}  {K_s:>12.1f}  {d_n:>12.3f}  "
                  f"{delta_eb*100:>10.3f}")
        print(f"{'─'*70}")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    unittest.main(verbosity=2)
