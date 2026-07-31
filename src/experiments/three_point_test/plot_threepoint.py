"""
plot_threepoint.py

Plotting tool for Three-Point Bending Test results.

Reads 'threepoint_log.csv' and produces a two-panel figure:
  Panel 1 (top)    — Deflection [mm] vs Time steps  (transient response)
  Panel 2 (bottom) — Force [N] vs Deflection [mm]   (F-δ curve + kB fit)

The F-δ panel also shows:
  - The measured data points (one per force step)
  - A linear fit line with annotated slope kB [N/m]
  - The theoretical F-δ line from Anisimov eq.(1) for two reference E values

Usage:
    python plot_threepoint.py
"""

import os
import sys
import math
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ---------------------------------------------------------------------------
# Import theory helpers (works without Isaac Sim)
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from threepoint_theory import (
    second_moment_of_area,
    structural_stiffness,
    elastic_modulus_from_stiffness,
    theoretical_deflection,
    span_diameter_ratio,
)

# ---------------------------------------------------------------------------
# Geometry (must match generate_threepoint_usda.py)
# ---------------------------------------------------------------------------
N_LINKS = 20
HEIGHT  = 0.015   # m
GAP     = 0.0001  # m
RADIUS  = 0.005   # m
L       = (N_LINKS - 1) * (HEIGHT + GAP)   # effective span between support origins
I       = second_moment_of_area(RADIUS)
SDR     = span_diameter_ratio(L, RADIUS)

# Reference E values for theoretical lines
E_REF_PRIMARY = 3.5e7   # 35 MPa  — Anisimov primary tissue
E_REF_MATURE  = 1.5e8   # 150 MPa — Shah et al. mature stem

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

def find_csv() -> str:
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(SCRIPT_DIR)))
    return os.path.join(project_root, "data", "usd_models", "threepoint_log.csv")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    csv_path = find_csv()
    if not os.path.exists(csv_path):
        print(f"[ERR] CSV not found: {csv_path}")
        print("      Run run_threepoint.py first.")
        return

    df = pd.read_csv(csv_path)
    if df.empty:
        print("[ERR] CSV is empty. Run the simulation first.")
        return

    forces_n    = df["Force_N"].values
    deflect_mm  = df["Deflection_mm"].values
    deflect_m   = deflect_mm / 1000.0
    steps       = df["Step"].values

    # Keep only the last sample per force level (end of each hold period)
    # since the log records every sim step, we sample at force transitions
    # by finding where force changes.
    force_transitions = np.where(np.diff(forces_n, prepend=np.nan) != 0)[0]
    # Actually: since the CSV logs all steps, find the LAST occurrence of each force level
    unique_forces = np.unique(forces_n)
    sampled_forces = []
    sampled_deltas = []
    for f in unique_forces:
        mask = forces_n == f
        sampled_forces.append(f)
        sampled_deltas.append(deflect_mm[mask][-1])   # last (settled) sample

    sampled_forces = np.array(sampled_forces)
    sampled_deltas = np.array(sampled_deltas)
    sampled_deltas_m = sampled_deltas / 1000.0

    # ----- Linear regression (kB) -----
    if len(sampled_deltas_m) >= 2 and sampled_deltas_m.max() > 0:
        coeffs = np.polyfit(sampled_deltas_m, sampled_forces, 1)
        kB_fit = float(coeffs[0])   # N/m
        E_fit  = elastic_modulus_from_stiffness(kB_fit, L, I)

        y_hat  = np.polyval(coeffs, sampled_deltas_m)
        ss_res = np.sum((sampled_forces - y_hat) ** 2)
        ss_tot = np.sum((sampled_forces - sampled_forces.mean()) ** 2)
        r2     = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    else:
        kB_fit = 0.0
        E_fit  = 0.0
        r2     = 0.0

    # ----- Theoretical lines -----
    delta_range_m  = np.linspace(0, sampled_deltas_m.max() * 1.2 if sampled_deltas_m.max() > 0 else 0.01, 100)
    delta_range_mm = delta_range_m * 1000.0
    f_theo_primary = kB_fit * delta_range_m if kB_fit > 0 else np.zeros_like(delta_range_m)

    max_f = sampled_forces.max() if len(sampled_forces) > 0 else 0.5
    f_line = np.linspace(0, max_f * 1.2, 100)
    d_primary_mm = np.array([theoretical_deflection(f, L, E_REF_PRIMARY, I) * 1000 for f in f_line])
    d_mature_mm  = np.array([theoretical_deflection(f, L, E_REF_MATURE,  I) * 1000 for f in f_line])

    # ==================================================================
    # Figure
    # ==================================================================
    fig = plt.figure(figsize=(13, 10))
    fig.suptitle(
        f"Three-Point Bending Test  |  L={L*100:.0f} cm, r={RADIUS*1000:.0f} mm, SDR={SDR:.0f}\n"
        f"[Anisimov et al. 2025 / Shtein et al. 2020]",
        fontsize=13, fontweight="bold"
    )
    gs = gridspec.GridSpec(2, 1, figure=fig, hspace=0.45)

    # ------------------------------------------------------------------
    # Panel 1: Deflection vs Simulation Step (transient response)
    # ------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(steps, deflect_mm, color="#2E86AB", linewidth=1.5, label="δ center (sim)")
    ax1.set_xlabel("Simulation Step", fontsize=11)
    ax1.set_ylabel("Central Deflection δ [mm]", fontsize=11, color="#2E86AB")
    ax1.tick_params(axis="y", labelcolor="#2E86AB")
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.set_title("Transient response — deflection over time", fontsize=11)

    ax1b = ax1.twinx()
    ax1b.plot(steps, forces_n, color="#E07A5F", linewidth=1.2, alpha=0.8, linestyle="--", label="F applied")
    ax1b.set_ylabel("Applied Force F [N]", fontsize=11, color="#E07A5F")
    ax1b.tick_params(axis="y", labelcolor="#E07A5F")

    lines1, labs1 = ax1.get_legend_handles_labels()
    lines2, labs2 = ax1b.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labs1 + labs2, loc="upper left", fontsize=9)

    # ------------------------------------------------------------------
    # Panel 2: F vs δ  (structural stiffness curve)
    # ------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[1])

    # Theoretical reference lines
    ax2.plot(d_primary_mm, f_line, color="#3BB273", linestyle="--", linewidth=1.5,
             label=f"Theory E=35 MPa (Anisimov primary)")
    ax2.plot(d_mature_mm,  f_line, color="#F4A261", linestyle="--", linewidth=1.5,
             label=f"Theory E=150 MPa (Shah mature)")

    # Measured data points
    ax2.scatter(sampled_deltas, sampled_forces, color="#2E86AB", zorder=5,
                s=60, label="Simulated (settled)")

    # Linear fit line
    if kB_fit > 0:
        fit_d_mm = delta_range_mm
        fit_f    = np.polyval(coeffs, delta_range_m)
        ax2.plot(fit_d_mm, fit_f, color="#E63946", linewidth=2.0,
                 label=f"Linear fit: kB={kB_fit:.3f} N/m  →  E={E_fit/1e6:.1f} MPa  (R²={r2:.3f})")

    ax2.set_xlabel("Central Deflection δ [mm]", fontsize=11)
    ax2.set_ylabel("Applied Force F [N]", fontsize=11)
    ax2.set_title("F–δ curve  (slope = structural stiffness kB)", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=9, loc="upper left")

    # Annotation box
    if kB_fit > 0:
        kB_theo_primary = structural_stiffness(E_REF_PRIMARY, L, I)
        kB_theo_mature  = structural_stiffness(E_REF_MATURE,  L, I)
        txt = (
            f"kB measured = {kB_fit:.4f} N/m\n"
            f"E derived   = {E_fit/1e6:.1f} MPa\n"
            f"R²          = {r2:.4f}\n"
            f"─────────────────────\n"
            f"kB(35 MPa)  = {kB_theo_primary:.4f} N/m\n"
            f"kB(150 MPa) = {kB_theo_mature:.4f} N/m"
        )
        ax2.text(0.97, 0.05, txt, transform=ax2.transAxes, fontsize=8,
                 verticalalignment="bottom", horizontalalignment="right",
                 bbox=dict(boxstyle="round,pad=0.4", facecolor="lightyellow", alpha=0.8))

    # ------------------------------------------------------------------
    # Save
    # ------------------------------------------------------------------
    save_path = os.path.join(SCRIPT_DIR, "threepoint_results.png")
    fig.savefig(save_path, dpi=200, bbox_inches="tight")
    print(f"[OK] Plot saved: {save_path}")
    plt.show()


if __name__ == "__main__":
    main()
