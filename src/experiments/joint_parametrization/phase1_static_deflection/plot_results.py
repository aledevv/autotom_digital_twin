"""
plot_results.py — Phase 1 Static Deflection Test
=================================================
Offline plotting script (no Isaac Sim needed).
Reads results/phase1_results.json and produces a publication-quality
bar chart comparing measured tip deflection to the Euler-Bernoulli
analytical prediction across different articulation counts N.

Run with any Python that has matplotlib + numpy:
    python3 plot_results.py
    python3 plot_results.py --input results/phase1_results.json --output figures/phase1.png
"""

import argparse
import json
import math
import os
import sys

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    import numpy as np
except ImportError:
    print("ERROR: matplotlib and numpy are required. Install with:\n"
          "  pip install matplotlib numpy")
    sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def parse_args():
    p = argparse.ArgumentParser(description="Plot Phase 1 results.")
    p.add_argument("--input",  default=os.path.join(SCRIPT_DIR, "results", "phase1_results.json"),
                   help="Path to phase1_results.json")
    p.add_argument("--output", default=os.path.join(SCRIPT_DIR, "results", "phase1_plot.png"),
                   help="Save figure to this path (default: results/phase1_plot.png)")
    p.add_argument("--show", action="store_true",
                   help="Open an interactive matplotlib window after saving.")
    return p.parse_args()


def load_results(path: str) -> dict:
    if not os.path.exists(path):
        print(f"ERROR: Results file not found: {path}")
        print("Run run_phase1.py first to generate results.")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


def make_figure(data: dict, out_path: str, show: bool = False) -> None:
    results  = data["results"]
    params   = data["parameters"]
    eb_ref   = data["analytical_deflection_m"] * 100.0   # convert to cm

    N_vals     = [r["N"] for r in results]
    meas_cm    = [r["tip_deflection_sim_m"] * 100.0 for r in results]
    errors_pct = [r["error_pct"] for r in results]
    statuses   = [r["status"] for r in results]

    # ── Style ─────────────────────────────────────────────────────────────
    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), facecolor="#f8f9fa")
    fig.suptitle(
        "Phase 1 — Static Deflection Test\n"
        f"Euler-Bernoulli Joint Parametrization  |  "
        f"L={params['L_m']}m  r={params['radius_m']}m  "
        f"E={params['E_Pa']:.1e}Pa  ρ={params['density_kg_m3']}kg/m³",
        fontsize=12, fontweight="bold", y=1.01
    )

    bar_colors = ["#3a86ff" if s == "OK" else "#ff006e" for s in statuses]

    # ── Left: Deflection comparison ───────────────────────────────────────
    ax1 = axes[0]
    x = np.arange(len(N_vals))
    bars = ax1.bar(x, meas_cm, color=bar_colors, width=0.55,
                   label="Simulated (PhysX)", zorder=3, edgecolor="white", linewidth=0.8)

    ax1.axhline(eb_ref, color="#ff6b35", linewidth=2.0, linestyle="--",
                label=f"Analytical E-B  ({eb_ref:.3f} cm)", zorder=4)

    # Annotate each bar with its value
    for bar, val in zip(bars, meas_cm):
        if not math.isnan(val):
            ax1.text(bar.get_x() + bar.get_width() / 2.0, val + 0.001 * abs(val) + 0.002,
                     f"{val:.3f}", ha="center", va="bottom", fontsize=9, color="#333")

    ax1.set_xticks(x)
    ax1.set_xticklabels([f"N={n}" for n in N_vals])
    ax1.set_xlabel("Number of segments (N)", fontsize=11)
    ax1.set_ylabel("Tip deflection (cm)", fontsize=11)
    ax1.set_title("Tip Deflection vs. N", fontsize=12)
    ax1.legend(fontsize=9)
    ax1.set_facecolor("#ffffff")

    # ── Right: Relative error vs. Euler-Bernoulli ─────────────────────────
    ax2 = axes[1]
    valid_mask = [not math.isnan(e) for e in errors_pct]
    x_valid    = [x[i] for i, v in enumerate(valid_mask) if v]
    err_valid  = [errors_pct[i] for i, v in enumerate(valid_mask) if v]
    col_valid  = [bar_colors[i] for i, v in enumerate(valid_mask) if v]

    ax2.bar(x_valid, err_valid, color=col_valid, width=0.55,
            zorder=3, edgecolor="white", linewidth=0.8)
    ax2.axhline(0, color="black", linewidth=0.8, zorder=4)
    ax2.axhline(20, color="#ffc300", linewidth=1.2, linestyle=":",
                label="±20% threshold", zorder=4)
    ax2.axhline(-20, color="#ffc300", linewidth=1.2, linestyle=":", zorder=4)

    for xi, err in zip(x_valid, err_valid):
        ax2.text(xi, err + 0.3 if err >= 0 else err - 1.5,
                 f"{err:.1f}%", ha="center", va="bottom", fontsize=9, color="#333")

    ax2.set_xticks(list(np.arange(len(N_vals))))
    ax2.set_xticklabels([f"N={n}" for n in N_vals])
    ax2.set_xlabel("Number of segments (N)", fontsize=11)
    ax2.set_ylabel("Relative error vs. E-B analytical (%)", fontsize=11)
    ax2.set_title("Convergence to Analytical Solution", fontsize=12)
    ax2.legend(fontsize=9)
    ax2.set_facecolor("#ffffff")

    # ── Legend patch for exploded ─────────────────────────────────────────
    if any(s != "OK" for s in statuses):
        exploded_patch = mpatches.Patch(color="#ff006e", label="Exploded / NaN")
        for ax in axes:
            ax.legend(handles=ax.get_legend().legend_handles + [exploded_patch], fontsize=9)

    # ── Stiffness annotation table (below) ────────────────────────────────
    table_data = [
        [f"N={r['N']}",
         f"{r['stiffness_K_real']:.4f}",
         f"{r['stiffness_K_sim']:.1f}",
         f"{r['tip_deflection_sim_m']*100:.3f} cm",
         f"{r['error_pct']:.1f}%" if not math.isnan(r['error_pct']) else "NaN",
         r["status"]]
        for r in results
    ]
    col_labels = ["N", "K_real (E-B)", "K_sim (×SCALE⁴)", "Measured sag", "Error vs E-B", "Status"]

    fig.subplots_adjust(bottom=0.22)
    table_ax = fig.add_axes([0.1, 0.0, 0.8, 0.17])
    table_ax.axis("off")
    tbl = table_ax.table(
        cellText=table_data, colLabels=col_labels,
        loc="center", cellLoc="center"
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(9)
    tbl.scale(1.0, 1.3)

    plt.tight_layout(rect=[0, 0.15, 1, 1])

    os.makedirs(os.path.dirname(out_path) if os.path.dirname(out_path) else ".", exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"[OK] Figure saved to: {out_path}")

    if show:
        plt.show()
    plt.close(fig)


def main():
    args = parse_args()
    data = load_results(args.input)
    make_figure(data, args.output, show=args.show)


if __name__ == "__main__":
    main()
