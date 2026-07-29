"""
plot_results.py — Phase 3 Dynamic Oscillation Test
====================================================
Offline plotting script. Reads results/phase3_results.json and produces:
  - Left: oscillation trajectories (Z vs time) for each N, overlaid
  - Right: bar chart of settling time vs N

Run:
    uv run src/experiments/joint_parametrization/phase3_oscillation/plot_results.py
    uv run ... --output figures/phase3.png
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

# Color palette per N value
N_COLORS = {2: "#3a86ff", 3: "#06d6a0", 5: "#ffbe0b", 10: "#fb5607"}


def parse_args():
    p = argparse.ArgumentParser(description="Plot Phase 3 results.")
    p.add_argument("--input",  default=os.path.join(SCRIPT_DIR, "results", "phase3_results.json"))
    p.add_argument("--output", default=os.path.join(SCRIPT_DIR, "results", "phase3_plot.png"),
                   help="Output figure path (default: results/phase3_plot.png)")
    p.add_argument("--show", action="store_true", help="Open interactive window after saving.")
    return p.parse_args()


def load_results(path: str) -> dict:
    if not os.path.exists(path):
        print(f"ERROR: Results file not found: {path}")
        print("Run run_phase3.py first to generate results.")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


def make_figure(data: dict, out_path: str, show: bool = False) -> None:
    results  = data["results"]
    params   = data["parameters"]

    plt.style.use("seaborn-v0_8-whitegrid")
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), facecolor="#f8f9fa")
    fig.suptitle(
        "Phase 3 — Dynamic Oscillation Test\n"
        f"Settling-Time Invariance with N  |  "
        f"L={params['L_m']}m  r={params['radius_m']}m  E={params['E_Pa']:.1e}Pa  "
        f"ζ={params['damping_ratio']}",
        fontsize=11, fontweight="bold", y=1.02
    )

    # ── Left: oscillation trajectories ───────────────────────────────────
    ax1 = axes[0]
    for r in results:
        if r["status"] != "OK":
            continue
        N       = r["N"]
        color   = N_COLORS.get(N, "#888")
        t_arr   = r["trajectory_t"]
        z_arr   = r["trajectory_z"]
        z_ref   = r["settled_z_sim"]
        # Convert sim-units displacement to cm in real units
        scale   = params["scale"]
        disp_cm = [(z - z_ref) / scale * 100.0 for z in z_arr]

        ax1.plot(t_arr, disp_cm, color=color, linewidth=1.4, label=f"N={N}", alpha=0.9)

        # Mark settling time
        st = r["settling_time_s"]
        if st is not None:
            ax1.axvline(st, color=color, linewidth=0.8, linestyle=":", alpha=0.6)

    threshold_cm = params["settle_threshold"] / params["scale"] * 100.0
    ax1.axhline( threshold_cm, color="#aaa", linewidth=1.0, linestyle="--", label=f"±{threshold_cm:.2f} cm threshold")
    ax1.axhline(-threshold_cm, color="#aaa", linewidth=1.0, linestyle="--")
    ax1.axhline(0, color="black", linewidth=0.8)

    ax1.set_xlabel("Time after impulse (s)", fontsize=11)
    ax1.set_ylabel("Tip displacement from settled (cm)", fontsize=11)
    ax1.set_title("Oscillation Trajectories", fontsize=12)
    ax1.legend(fontsize=9)
    ax1.set_facecolor("#ffffff")

    # ── Right: settling time bar chart ───────────────────────────────────
    ax2 = axes[1]
    N_vals = [r["N"] for r in results]
    st_vals = [r["settling_time_s"] if r["settling_time_s"] is not None else 0.0 for r in results]
    bar_colors = [N_COLORS.get(r["N"], "#888") for r in results]
    hatches = ["" if r["settling_time_s"] is not None else "///" for r in results]

    x = np.arange(len(N_vals))
    bars = ax2.bar(x, st_vals, color=bar_colors, width=0.55,
                   edgecolor="white", linewidth=0.8, zorder=3)
    for bar, hatch, r in zip(bars, hatches, results):
        bar.set_hatch(hatch)
        val = r["settling_time_s"]
        label = f"{val:.2f}s" if val is not None else "Not settled"
        ax2.text(bar.get_x() + bar.get_width() / 2.0,
                 bar.get_height() + 0.01 * max(st_vals + [0.1]),
                 label, ha="center", va="bottom", fontsize=9, color="#222")

    ax2.set_xticks(x)
    ax2.set_xticklabels([f"N={n}" for n in N_vals])
    ax2.set_xlabel("Number of segments (N)", fontsize=11)
    ax2.set_ylabel("Settling time (s)", fontsize=11)
    ax2.set_title("Settling Time vs. N", fontsize=12)
    ax2.set_facecolor("#ffffff")

    # ── Summary table ─────────────────────────────────────────────────────
    table_data = [
        [f"N={r['N']}",
         f"{r['stiffness_K']:.2f}",
         f"{r['oscillation_amplitude_sim'] / params['scale'] * 100:.3f} cm"
         if r["oscillation_amplitude_sim"] else "NaN",
         f"{r['settling_time_s']:.2f} s" if r["settling_time_s"] else "Not settled",
         r["status"]]
        for r in results
    ]
    col_labels = ["N", "K (joint stiffness)", "Osc. amplitude (cm)", "Settling time", "Status"]

    fig.subplots_adjust(bottom=0.22)
    table_ax = fig.add_axes([0.05, 0.0, 0.9, 0.15])
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
