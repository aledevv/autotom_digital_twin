"""
droop_theory.py

Theoretical calculation of branch tip deflection under gravity using Euler-Bernoulli
beam theory for a cantilever with uniformly distributed load.

Formula:
    δ_tip = (q × L⁴) / (8 × E × I)

where:
    q = load per unit length = ρ × g × A = ρ × g × π × r² [N/m]
    L = total beam length [m]
    E = Young's modulus [Pa]
    I = second moment of area = π × r⁴ / 4 [m⁴]
    ρ = density [kg/m³]
    g = gravity [m/s²]

For tilted branches, only the component perpendicular to the beam axis causes bending:
    δ_effective = δ_tip × sin(tilt_angle)

Run standalone:
    uv run src/experiments/recursive_tree/droop_theory.py
"""

import math
import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(SCRIPT_DIR))

from tree_config import GLOBAL_SCALE, BioConfig, BRANCHES, scaled


GRAVITY = 9.81  # m/s²


def calculate_cantilever_deflection(
    n_links: int,
    height_per_link: float,
    radius: float,
    E: float,
    tilt_deg: float,
) -> dict:
    """
    Calculate the tip deflection of a cantilever beam under its own weight.

    Args:
        n_links         : number of segments (for total length calculation)
        height_per_link : height of each segment [m, world units]
        radius          : beam radius [m, world units]
        E               : Young's modulus [Pa]
        tilt_deg        : tilt angle from vertical [degrees]

    Returns:
        dict with keys:
            L            : total length [m]
            I            : second moment of area [m⁴]
            q            : distributed load [N/m]
            delta_tip_horizontal : tip deflection if beam were horizontal [mm]
            delta_effective      : actual deflection given tilt [mm]
            tilt_factor          : sin(tilt) — fraction of horizontal droop
    """
    # Total length
    L = n_links * height_per_link  # [m]

    # Second moment of area
    I = (math.pi * (radius ** 4)) / 4.0  # [m⁴]

    # Cross-sectional area
    A = math.pi * (radius ** 2)  # [m²]

    # Distributed load (weight per unit length)
    q = BioConfig.PLANT_DENSITY * GRAVITY * A  # [N/m]

    # Tip deflection for horizontal cantilever
    delta_tip_m = (q * (L ** 4)) / (8.0 * E * I)  # [m]
    delta_tip_mm = delta_tip_m * 1000.0  # [mm]

    # Correction for tilt (only perpendicular component causes bending)
    tilt_rad = math.radians(tilt_deg)
    tilt_factor = abs(math.sin(tilt_rad))
    delta_effective_mm = delta_tip_mm * tilt_factor  # [mm]

    return {
        "L": L,
        "I": I,
        "q": q,
        "delta_tip_horizontal": delta_tip_mm,
        "delta_effective": delta_effective_mm,
        "tilt_factor": tilt_factor,
    }


def print_droop_summary(branches=None):
    """Print theoretical droop for all branches in the config."""
    if branches is None:
        branches = BRANCHES

    E = BioConfig.YOUNG_MODULUS

    print()
    print("=" * 80)
    print(f"  Theoretical Branch Droop  |  E = {E:.2e} Pa  |  ρ = {BioConfig.PLANT_DENSITY} kg/m³")
    print("=" * 80)
    print(f"  {'Branch':<12} {'L(m)':>6} {'r(m)':>7} {'Tilt(°)':>8} "
          f"{'δ_horiz(mm)':>12} {'δ_eff(mm)':>11} {'sin(θ)':>7}")
    print("-" * 80)

    for b in branches:
        bid         = b["id"]
        n_links     = b["n_links"]
        h_raw       = b["height"]
        r_raw       = b["radius"]
        tilt        = b.get("tilt", 0.0)

        # Scale to world units
        h_world = scaled(h_raw)
        r_world = scaled(r_raw)

        result = calculate_cantilever_deflection(n_links, h_world, r_world, E, tilt)

        L              = result["L"]
        delta_horiz    = result["delta_tip_horizontal"]
        delta_eff      = result["delta_effective"]
        tilt_fac       = result["tilt_factor"]

        print(f"  {bid:<12} {L:>6.2f} {r_world:>7.3f} {tilt:>8.1f} "
              f"{delta_horiz:>12.2f} {delta_eff:>11.2f} {tilt_fac:>7.3f}")

    print("-" * 80)
    print("  δ_horiz = tip deflection if branch were horizontal (90° tilt)")
    print("  δ_eff   = actual deflection = δ_horiz × sin(tilt)")
    print("  sin(θ)  = tilt factor (0 = vertical, 1 = horizontal)")
    print()


if __name__ == "__main__":
    print_droop_summary()
