"""
threepoint_theory.py

Standalone physics formulas for the Three-Point Bending Test.
No Isaac Sim dependency — importable from any analysis script.

References:
  [1] Anisimov et al. (2025), Methods and Protocols 8(2), 32.
      DOI: 10.3390/mps8020032
  [2] Shtein et al. (2020), Plants 9(6), 678.
      DOI: 10.3390/plants9060678
"""

import math


def second_moment_of_area(radius: float, inner_radius: float = 0.0) -> float:
    """
    I = π(R⁴ - r⁴)/4  [m⁴]
    For a solid circular cross-section (inner_radius=0): I = πR⁴/4.
    Equivalent to π(D⁴ - d⁴)/64 from Anisimov eq.(2).
    """
    return math.pi * (radius**4 - inner_radius**4) / 4.0


def theoretical_deflection(F: float, L: float, E: float, I: float) -> float:
    """
    Central deflection of a simply-supported beam under a central point load.
    δ = FL³ / (48·E·I)  [m]   — Anisimov eq.(1), Shtein eq.(4).

    Args:
        F: applied force [N]
        L: span between supports [m]
        E: Young's modulus [Pa]
        I: second moment of area [m⁴]
    Returns:
        deflection [m]
    """
    return (F * L**3) / (48.0 * E * I)


def elastic_modulus_from_deflection(F: float, L: float, delta: float, I: float) -> float:
    """
    E = FL³ / (48·δ·I)  [Pa]   — Anisimov eq.(1) rearranged.

    Args:
        F:     applied force [N]
        L:     span [m]
        delta: measured central deflection [m]
        I:     second moment of area [m⁴]
    Returns:
        Young's modulus [Pa]
    """
    return (F * L**3) / (48.0 * delta * I)


def structural_stiffness(E: float, L: float, I: float) -> float:
    """
    kB = 48·E·I / L³  [N/m]
    Structural (flexural) stiffness — slope of the F-vs-δ curve.
    """
    return (48.0 * E * I) / L**3


def elastic_modulus_from_stiffness(kB: float, L: float, I: float) -> float:
    """
    E = kB·L³ / (48·I)  [Pa]   — Shtein eq.(4).
    Preferred calibration target: use kB (slope) rather than a single (F, δ) point.
    """
    return (kB * L**3) / (48.0 * I)


def span_diameter_ratio(span: float, radius: float) -> float:
    """
    SDR = L / D = L / (2·R).
    Must be ≥ 20 to neglect shear effects (Anisimov, citing Fok & Smart 1993).
    """
    return span / (2.0 * radius)


def joint_rotational_stiffness(E: float, I: float, link_length: float) -> float:
    """
    K_θ = E·I / L_link  [N·m/rad]
    Drive stiffness for each D6 joint in the discrete chain approximation.
    Applied as 'acceleration' drive in PhysX to avoid mass-scaling issues.
    """
    return (E * I) / link_length


def critical_damping(K_theta: float, mass: float, ratio: float = 0.2) -> float:
    """
    D = 2 · ratio · √(K_θ · mass)  [N·m·s/rad]
    Under-critical damping (ratio < 1) to allow oscillation to settle.
    """
    return 2.0 * ratio * math.sqrt(K_theta * mass)


# ==============================================================================
# Self-test (run as standalone script to verify formulas)
# ==============================================================================

if __name__ == "__main__":
    print("=" * 60)
    print("Three-Point Bending Theory — Self-Test")
    print("=" * 60)

    R = 0.005          # 5 mm radius
    L = 0.30           # 30 cm span
    E_primary = 3.5e7  # 35 MPa (Anisimov: primary tissue range 20-50 MPa)
    E_mature  = 1.5e8  # 150 MPa (Shah et al.: mature stem with sclerenchyma)

    I = second_moment_of_area(R)
    SDR = span_diameter_ratio(L, R)

    print(f"\nGeometry:")
    print(f"  Radius R     = {R*1000:.1f} mm")
    print(f"  Span L       = {L*100:.0f} cm")
    print(f"  I            = {I:.3e} m⁴")
    print(f"  SDR = L/D    = {SDR:.1f}  {'✅ >= 20' if SDR >= 20 else '⚠️  < 20 (shear effects significant)'}")

    for label, E in [("Primary tissue (35 MPa)", E_primary), ("Mature stem (150 MPa)", E_mature)]:
        kB = structural_stiffness(E, L, I)
        delta_05 = theoretical_deflection(0.5, L, E, I)
        E_roundtrip = elastic_modulus_from_stiffness(kB, L, I)
        print(f"\n{label}:")
        print(f"  E input      = {E/1e6:.1f} MPa")
        print(f"  kB           = {kB:.4f} N/m")
        print(f"  δ at F=0.5N  = {delta_05*1000:.3f} mm")
        print(f"  E round-trip = {E_roundtrip/1e6:.1f} MPa  {'✅' if abs(E_roundtrip - E) < 1 else '❌'}")

    print("\n" + "=" * 60)
