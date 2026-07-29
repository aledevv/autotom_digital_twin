"""
plot_results.py — Phase 2 Tapering Test
========================================
Offline plotting script. Reads results/phase2_results.json and produces
a side-by-side bar chart comparing the two tapering strategies (linear vs
physics-derived r⁴) against the numerical Euler-Bernoulli reference.

Run:
    uv run src/experiments/joint_parametrization/phase2_tapering/plot_results.py
    uv run ... --output figures/phase2.png
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
    p = argparse.ArgumentParser(description="Plot Phase 2 results.")
    p.add_argument("--input",  default=os.path.join(SCRIPT_DIR, "results", "phase2_results.json"))
    p.add_argument("--output", default=os.path.join(SCRIPT_DIR, "results", "phase2_plot.png"),
                   help="Output figure path (default: results/phase2_plot.png)")
    p.add_argument("--show", action="store_true", help="Open interactive window after saving.")
    return p.parse_args()


def load_results(path: str) -> dict:
    if not os.path.exists(path):
        print(f"ERROR: Results file not found: {path}")
        print("Run run_phase2.py first to generate results.")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


def make_figure(data: dict, out_path: str, show: bool = False) -> None:
    results  = data["results"]
    params   = data["parameters"]
    eb_ref   = data["analytical_tapered_deflection_m"] * 100.0   # cm

    strategies  = [r["strategy"] for r in results]
    meas_cm     = [(r["tip_deflection_sim_m"] or float("nan")) * 100.0 for r in results]
    errors_pct  = [r["error_pct"] or float("nan") for r in results]
    statuses    = [r["status"] for r in results]

    palette = {"linear": "#3a86ff", "physics": "#06d6a0"}
    bar_colors = [palette.get(s, "#aaa") for s in strategies]

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(1, 2, figsize=(11, 5), facecolor="#f8f9fa")
    fig.suptitle(
        "Phase 2 — Tapering Test\n"
        f"Linear vs. r⁴ Stiffness Tapering  |  "
        f"N={params['N_segments']}  L={params['L_m']}m  "
        f"R_base={params['R_base_m']}m → R_tip={params['R_tip_m']}m  "
        f"E={params['E_Pa']:.1e}Pa",
        fontsize=11, fontweight="bold", y=1.02
    )

    x = np.arange(len(strategies))

    # ── Left: deflection comparison ───────────────────────────────────────
    ax1 = axes[0]
    bars = ax1.bar(x, meas_cm, color=bar_colors, width=0.45,
                   edgecolor="white", linewidth=0.8, zorder=3,
                   label=[s.capitalize() for s in strategies])
    ax1.axhline(eb_ref, color="#ff6b35", linewidth=2.0, linestyle="--",
                label=f"Numerical E-B  ({eb_ref:.3f} cm)", zorder=4)

    for bar, val in zip(bars, meas_cm):
        if not math.isnan(val):
            ax1.text(bar.get_x() + bar.get_width() / 2.0, val + 0.01 * abs(val) + 0.02,
                     f"{val:.3f} cm", ha="center", va="bottom", fontsize=9, color="#222")

    # Custom legend for bars
    legend_handles = [
        mpatches.Patch(color=palette["linear"], label="Linear tapering"),
        mpatches.Patch(color=palette["physics"], label="Physics r⁴ tapering"),
        plt.Line2D([0], [0], color="#ff6b35", linewidth=2, linestyle="--",
                   label=f"Numerical E-B ({eb_ref:.3f} cm)"),
    ]
    ax1.legend(handles=legend_handles, fontsize=9)
    ax1.set_xticks(x)
    ax1.set_xticklabels([s.capitalize() for s in strategies])
    ax1.set_xlabel("Tapering strategy", fontsize=11)
    ax1.set_ylabel("Tip deflection (cm)", fontsize=11)
    ax1.set_title("Measured Tip Deflection", fontsize=12)
    ax1.set_facecolor("#ffffff")

    # ── Right: error vs. analytical ───────────────────────────────────────
    ax2 = axes[1]
    valid = [not math.isnan(e) for e in errors_pct]
    x_v   = [x[i] for i, v in enumerate(valid) if v]
    err_v = [errors_pct[i] for i, v in enumerate(valid) if v]
    col_v = [bar_colors[i] for i, v in enumerate(valid) if v]

    ax2.bar(x_v, err_v, color=col_v, width=0.45,
            edgecolor="white", linewidth=0.8, zorder=3)
    ax2.axhline(0,  color="black",   linewidth=0.8, zorder=4)
    ax2.axhline(20, color="#ffc300", linewidth=1.2, linestyle=":", label="±20% threshold", zorder=4)
    ax2.axhline(-20, color="#ffc300", linewidth=1.2, linestyle=":", zorder=4)

    for xi, err in zip(x_v, err_v):
        ax2.text(xi, err + 0.3 if err >= 0 else err - 1.5,
                 f"{err:.1f}%", ha="center", va="bottom", fontsize=9, color="#222")

    ax2.set_xticks(list(np.arange(len(strategies))))
    ax2.set_xticklabels([s.capitalize() for s in strategies])
    ax2.set_xlabel("Tapering strategy", fontsize=11)
    ax2.set_ylabel("Relative error vs. numerical E-B (%)", fontsize=11)
    ax2.set_title("Error vs. Analytical Reference", fontsize=12)
    ax2.legend(fontsize=9)
    ax2.set_facecolor("#ffffff")

    # ── Summary table ─────────────────────────────────────────────────────
    table_data = [
        [r["strategy"].capitalize(),
         f"{(r['tip_deflection_sim_m'] or 0)*100:.3f} cm" if r["tip_deflection_sim_m"] else "NaN",
         f"{r['analytical_tapered_deflection_m']*100:.3f} cm",
         f"{r['error_pct']:.1f}%" if r["error_pct"] else "NaN",
         r["status"]]
        for r in results
    ]
    col_labels = ["Strategy", "Measured sag", "E-B (numerical)", "Error", "Status"]

    fig.subplots_adjust(bottom=0.22)
    table_ax = fig.add_axes([0.1, 0.0, 0.8, 0.15])
    table_ax.axis("off")
    tbl = table_ax.table(cellText=table_data, colLabels=col_labels,
                         loc="center", cellLoc="center")
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
