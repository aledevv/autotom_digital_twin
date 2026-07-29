"""
plot_results_v2.py — Phase 1 v2 Static Deflection Test
=======================================================
Offline plotting script — no Isaac Sim needed.
Reads results/phase1_v2_results.json and produces a 3-panel figure:

  Panel 1  Deflection comparison bar chart
           Bars = PhysX measured values
           Dashed orange  = continuous E-B analytical (N→∞ limit)
           Dashed blue    = discrete-chain target δ_N = δ_EB·(1+1/N)²

  Panel 2  Error vs. discrete target (%)
           The discrete target is the fair reference for a finite-N chain.
           ±15% threshold lines are drawn.

  Panel 3  Per-step convergence traces
           Tip deflection (cm) vs. simulation time (s) for each N.
           Shows settling speed; useful to spot oscillation or explosion.

  Bottom table  Summary rows with K_real, K_sim, measured sag, discrete
                target, error, sanity status.

Run with any Python that has matplotlib + numpy:
    python3 plot_results_v2.py
    python3 plot_results_v2.py --input results/phase1_v2_results.json \\
                               --output results/phase1_v2_plot.png --show
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
    print("ERROR: matplotlib and numpy are required.\n"
          "  pip install matplotlib numpy")
    sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Colour palette
C_BARS     = "#3a86ff"   # OK bars
C_FAIL     = "#ff006e"   # EXPLODED / SANITY_FAIL bars
C_EB       = "#ff6b35"   # continuous E-B reference line
C_DISC     = "#0077b6"   # discrete-target reference line
C_THRESH   = "#ffc300"   # ±15% threshold


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot Phase 1 v2 results.")
    p.add_argument(
        "--input",
        default=os.path.join(SCRIPT_DIR, "results", "phase1_v2_results.json"),
        help="Path to phase1_v2_results.json",
    )
    p.add_argument(
        "--output",
        default=os.path.join(SCRIPT_DIR, "results", "phase1_v2_plot.png"),
        help="Output PNG path",
    )
    p.add_argument("--show", action="store_true",
                   help="Open interactive matplotlib window after saving.")
    return p.parse_args()


def load_results(path: str) -> dict:
    if not os.path.exists(path):
        print(f"ERROR: Results file not found: {path}")
        print("Run run_phase1_v2.py first to generate results.")
        sys.exit(1)
    with open(path) as f:
        return json.load(f)


def _safe_cm(val_m) -> float | None:
    """Convert metres → cm, returning None when the value is None or NaN."""
    if val_m is None:
        return None
    v = float(val_m)
    return None if math.isnan(v) else v * 100.0


def make_figure(data: dict, out_path: str, show: bool = False) -> None:
    results = data["results"]
    params  = data["parameters"]

    eb_ref_cm   = _safe_cm(data["analytical_eb_m"])   # continuous E-B (N→∞)

    N_vals      = [r["N"] for r in results]
    meas_cm     = [_safe_cm(r.get("tip_deflection_sim_m")) for r in results]
    disc_cm     = [_safe_cm(r.get("analytical_discrete_m")) for r in results]
    errors_pct  = [r.get("error_vs_discrete_pct") for r in results]
    statuses    = [r.get("status", "OK") for r in results]
    sanity_ok   = [r.get("sanity_passed", True) for r in results]

    bar_colors  = [C_BARS if s == "OK" else C_FAIL for s in statuses]
    x           = np.arange(len(N_vals))

    # ── Figure layout: 3 panels + table strip ─────────────────────────────
    plt.style.use("seaborn-v0_8-whitegrid")
    fig = plt.figure(figsize=(18, 6), facecolor="#f8f9fa")

    # Reserve bottom 18% for the table
    gs = fig.add_gridspec(1, 3, left=0.06, right=0.98, top=0.88,
                          bottom=0.22, wspace=0.35)
    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    ax3 = fig.add_subplot(gs[2])

    title = (
        "Phase 1 v2 — Static Deflection Test  (raw USD articulation, no PlantBuilder)\n"
        f"L={params['L_m']}m  r={params['radius_m']}m  "
        f"E={params['E_Pa']:.1e}Pa  ρ={params['density_kg_m3']}kg/m³  "
        f"SCALE={params['scale']}  {params['sim_hz']}Hz  {params['sim_seconds']}s"
    )
    fig.suptitle(title, fontsize=11, fontweight="bold", y=0.97)

    # ═══════════════════════════════════════════════════════════════════════
    # Panel 1 — Deflection comparison
    # ═══════════════════════════════════════════════════════════════════════
    meas_plot = [v if v is not None else 0.0 for v in meas_cm]
    bars = ax1.bar(x, meas_plot, color=bar_colors, width=0.55,
                   label="PhysX measured", zorder=3, edgecolor="white", linewidth=0.8)

    # E-B continuous reference (N→∞)
    if eb_ref_cm is not None:
        ax1.axhline(eb_ref_cm, color=C_EB, linewidth=2.0, linestyle="--",
                    label=f"E-B analytical  ({eb_ref_cm:.3f} cm)", zorder=4)

    # Discrete-chain targets as a step/scatter overlay
    disc_valid = [(xi, d) for xi, d in zip(x, disc_cm) if d is not None]
    if disc_valid:
        xd, yd = zip(*disc_valid)
        ax1.plot(xd, yd, color=C_DISC, linewidth=1.5, linestyle=":",
                 marker="D", markersize=5, label="Discrete target δ_N", zorder=5)

    # Annotate bars with measured values
    for bar, val, status in zip(bars, meas_cm, statuses):
        label_val = val if val is not None else 0.0
        txt = f"{label_val:.3f}" if status == "OK" else status[:3]
        ax1.text(bar.get_x() + bar.get_width() / 2.0,
                 bar.get_height() + 0.04,
                 txt, ha="center", va="bottom", fontsize=8, color="#222")

    ax1.set_xticks(x)
    ax1.set_xticklabels([f"N={n}" for n in N_vals], fontsize=9)
    ax1.set_xlabel("Number of segments (N)", fontsize=10)
    ax1.set_ylabel("Tip deflection (cm)", fontsize=10)
    ax1.set_title("Tip Deflection vs. N", fontsize=11, fontweight="bold")
    ax1.legend(fontsize=8, loc="upper right")
    ax1.set_facecolor("#ffffff")

    # ═══════════════════════════════════════════════════════════════════════
    # Panel 2 — Error vs. discrete target
    # ═══════════════════════════════════════════════════════════════════════
    err_plot   = [e if e is not None else 0.0 for e in errors_pct]
    err_colors = [bar_colors[i] if errors_pct[i] is not None else "#cccccc"
                  for i in range(len(errors_pct))]

    ax2.bar(x, err_plot, color=err_colors, width=0.55,
            zorder=3, edgecolor="white", linewidth=0.8)
    ax2.axhline(0,   color="black",   linewidth=0.8, zorder=4)
    ax2.axhline( 15, color=C_THRESH, linewidth=1.2, linestyle=":",
                label="±15% threshold", zorder=4)
    ax2.axhline(-15, color=C_THRESH, linewidth=1.2, linestyle=":", zorder=4)

    for xi, err in zip(x, errors_pct):
        if err is not None:
            yoff = 0.4 if err >= 0 else -2.0
            ax2.text(xi, err + yoff, f"{err:+.1f}%",
                     ha="center", va="bottom", fontsize=8, color="#222")

    ax2.set_xticks(x)
    ax2.set_xticklabels([f"N={n}" for n in N_vals], fontsize=9)
    ax2.set_xlabel("Number of segments (N)", fontsize=10)
    ax2.set_ylabel("Error vs. discrete target (%)", fontsize=10)
    ax2.set_title("Convergence to Discrete Target δ_N", fontsize=11, fontweight="bold")
    ax2.legend(fontsize=8)
    ax2.set_facecolor("#ffffff")

    # ─── Subtitle explaining the discrete reference ───────────────────────
    ax2.text(0.5, -0.18,
             "Reference: δ_N = δ_EB·(1+1/N)²  (expected overshoot for a finite chain)",
             ha="center", va="top", transform=ax2.transAxes,
             fontsize=7.5, color="#555", style="italic")


    # ═══════════════════════════════════════════════════════════════════════
    # Panel 3 — Per-step convergence traces
    # ═══════════════════════════════════════════════════════════════════════
    cmap   = plt.get_cmap("tab10")
    traced = False

    for idx, r in enumerate(results):
        trace = r.get("convergence_trace", [])
        if not trace:
            continue
        traced = True
        t_vals  = [pt["t_s"] for pt in trace]
        dz_vals = [pt["delta_cm"] for pt in trace]

        color = cmap(idx % 10)
        label = f"N={r['N']}"
        if r.get("converged_early"):
            label += " [conv]"
        elif r.get("status") != "OK":
            label += f" [{r['status'][:3]}]"

        ax3.plot(t_vals, dz_vals, color=color, linewidth=1.6,
                 marker="o", markersize=3, label=label, zorder=3)

        # Draw the discrete target as a horizontal dashed line in the same colour
        if disc_cm[idx] is not None:
            ax3.axhline(disc_cm[idx], color=color, linewidth=0.8,
                        linestyle="--", alpha=0.5)

    if not traced:
        ax3.text(0.5, 0.5, "No convergence trace data\n(run the simulation first)",
                 ha="center", va="center", transform=ax3.transAxes,
                 fontsize=10, color="#888")

    # E-B continuous line across the full time axis
    if eb_ref_cm is not None:
        ax3.axhline(eb_ref_cm, color=C_EB, linewidth=1.5, linestyle="--",
                    alpha=0.7, label=f"E-B ({eb_ref_cm:.3f} cm)", zorder=4)

    ax3.set_xlabel("Simulation time (s)", fontsize=10)
    ax3.set_ylabel("Tip deflection (cm)", fontsize=10)
    ax3.set_title("Settling Traces — Tip Deflection over Time", fontsize=11,
                  fontweight="bold")
    if traced:
        ax3.legend(fontsize=8, loc="lower right")
    ax3.set_facecolor("#ffffff")

    # ─── Annotate ✓ = converged early ────────────────────────────────────
    ax3.text(0.02, 0.98,
             "Dashed lines = discrete targets per N\n[conv] = converged early",
             ha="left", va="top", transform=ax3.transAxes,
             fontsize=7.5, color="#555", style="italic")

    # ─── Legend patches for non-OK statuses ──────────────────────────────
    if any(s != "OK" for s in statuses):
        fail_patch = mpatches.Patch(color=C_FAIL, label="EXPLODED / SANITY_FAIL")
        for ax in (ax1, ax2):
            handles, labels = ax.get_legend_handles_labels()
            ax.legend(handles=handles + [fail_patch],
                      labels=labels + ["EXPLODED / SANITY_FAIL"],
                      fontsize=8, loc="upper right")

    # ═══════════════════════════════════════════════════════════════════════
    # Bottom table
    # ═══════════════════════════════════════════════════════════════════════
    table_rows = []
    for r in results:
        meas_v = _safe_cm(r.get("tip_deflection_sim_m"))
        disc_v = _safe_cm(r.get("analytical_discrete_m"))
        err_v  = r.get("error_vs_discrete_pct")
        sane   = "OK" if r.get("sanity_passed", True) else "FAIL"
        conv   = "yes" if r.get("converged_early", False) else "no"

        table_rows.append([
            f"N={r['N']}",
            f"{r['stiffness_K_real']:.4f}",
            f"{r['stiffness_K_sim']:.0f}",
            f"{meas_v:.3f}" if meas_v is not None else "NaN",
            f"{disc_v:.3f}" if disc_v is not None else "NaN",
            f"{eb_ref_cm:.3f}" if eb_ref_cm is not None else "NaN",
            f"{err_v:+.1f}%" if err_v is not None else "NaN",
            sane,
            conv,
            r.get("status", "?"),
        ])

    col_labels = [
        "N", "K_real (N·m/rad)", "K_sim (×10⁴)",
        "Meas (cm)", "Discrete tgt", "E-B (cm)",
        "Err% vs disc", "Sanity", "Early conv.", "Status",
    ]

    table_ax = fig.add_axes([0.03, 0.0, 0.94, 0.18])
    table_ax.axis("off")
    tbl = table_ax.table(
        cellText=table_rows,
        colLabels=col_labels,
        loc="center",
        cellLoc="center",
    )
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(8)
    tbl.scale(1.0, 1.25)

    # Colour header row
    for col_idx in range(len(col_labels)):
        tbl[(0, col_idx)].set_facecolor("#2b2d42")
        tbl[(0, col_idx)].set_text_props(color="white", fontweight="bold")

    # Colour data rows: green for OK, red for failures
    for row_idx, r in enumerate(results, start=1):
        row_color = "#e8f5e9" if r.get("status") == "OK" else "#fce4ec"
        for col_idx in range(len(col_labels)):
            tbl[(row_idx, col_idx)].set_facecolor(row_color)

    # ── Save ──────────────────────────────────────────────────────────────
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"[OK] Figure saved → {out_path}")

    if show:
        plt.show()
    plt.close(fig)


def main() -> None:
    args = parse_args()
    data = load_results(args.input)
    make_figure(data, args.output, show=args.show)


if __name__ == "__main__":
    main()
