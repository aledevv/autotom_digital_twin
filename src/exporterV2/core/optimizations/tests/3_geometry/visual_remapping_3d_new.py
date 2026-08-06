"""
visual_remapping_3d_new.py - 3D Comparison of Branch Attachment Remapping

Shows BEFORE and AFTER stem reduction side-by-side in a true 3D matplotlib plot.
Both trunks have the same total height; branches are drawn as cylinders; horizontal
planes (like the old "riquadri") cross both columns at the same absolute height,
proving geometric preservation.

Mathematical model (p=1.0, top-of-link convention):
    H_abs = attach_link / N_old          (fraction of total height)
    V     = H_abs * N_new
    k_new = floor(V) + 1  if H<1.0,  else N_new   (1-based, edge-case guarded)
    p_new = V - floor(V)               (fraction within new link)

Example shown (user's example A): N=10 → N=3
  Branch A at L9  → L3 @ 70%     H=0.90
  Branch B at L4  → L2 @ 20%     H=0.40
  Branch C at L10 → L3 @ 100%    H=1.00  (edge-case: clamped to top)

Run with:
    cd /home/alessandro/isaacsim/autotom_digital_twin
    uv run python src/exporterV2/core/optimizations/tests/3_geometry/visual_remapping_3d_new.py
"""

import math
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── remapping math ────────────────────────────────────────────────────────────

def remap(attach_link: int, n_old: int, n_new: int) -> tuple[int, float]:
    """Remap 1-based attach_link from n_old → n_new.  Returns (k_new, p_new)."""
    H = attach_link / n_old
    if H >= 1.0:
        return n_new, 1.0
    V = H * n_new
    k = math.floor(V) + 1
    p = V - math.floor(V)
    return k, p


# ── 3-D drawing helpers ───────────────────────────────────────────────────────

N_THETA = 24   # cylinder resolution


def _cylinder_surface(base, top, radius):
    """Return (X, Y, Z) arrays for a vertical cylinder from base to top z."""
    theta = np.linspace(0, 2 * np.pi, N_THETA)
    zs    = np.array([base, top])
    T, Z  = np.meshgrid(theta, zs)
    X = radius * np.cos(T)
    Y = radius * np.sin(T)
    return X, Y, Z


def draw_trunk(ax, x_offset, n_links, total_h, trunk_r, seg_colors):
    """Draw a segmented trunk at x_offset (trunk axis = Z)."""
    link_h = total_h / n_links
    for i in range(n_links):
        z0 = i * link_h
        z1 = z0 + link_h * 0.97          # tiny visual gap
        X, Y, Z = _cylinder_surface(z0, z1, trunk_r)
        ax.plot_surface(
            X + x_offset, Y, Z,
            color=seg_colors[i % len(seg_colors)],
            alpha=0.55, linewidth=0, edgecolor="none",
        )
        # Link label
        ax.text(x_offset, 0, z0 + link_h / 2,
                f"L{i+1}", ha="center", va="center",
                fontsize=7, color="k", fontweight="bold", zorder=10)


def draw_branch_arrow(ax, x_trunk, z_attach, color, length=0.35, label=""):
    """Draw a horizontal arrow + label representing a branch."""
    x_end = x_trunk + length
    ax.quiver(x_trunk, 0, z_attach,
              length, 0, 0,
              color=color, linewidth=2.5, arrow_length_ratio=0.18)
    ax.text(x_end + 0.03, 0, z_attach + 0.02,
            label, color=color, fontsize=8, fontweight="bold")
    # dot at attachment
    ax.scatter([x_trunk], [0], [z_attach],
               c=color, s=60, edgecolors="white", linewidths=1.2, zorder=9)


def draw_height_plane(ax, z_abs, x_min, x_max, color, alpha=0.10):
    """Draw a transparent horizontal square at height z_abs."""
    y_r = 0.6
    verts = [
        [(x_min, -y_r, z_abs), (x_max, -y_r, z_abs),
         (x_max,  y_r, z_abs), (x_min,  y_r, z_abs)]
    ]
    poly = Poly3DCollection(verts, alpha=alpha, facecolor=color, edgecolor=color,
                            linewidth=1.0, linestyle="--")
    ax.add_collection3d(poly)


# ── main scenario ─────────────────────────────────────────────────────────────

def main():
    # ── parameters ───────────────────────────────────────────────────────────
    N_OLD    = 10
    N_NEW    = 3
    TOTAL_H  = 1.0          # metres (normalised)
    TRUNK_R  = 0.06
    X_OLD    = 0.0          # x-position of old trunk centre
    X_NEW    = 2.2          # x-position of new trunk centre
    X_PLANE_MIN = X_OLD - 0.15
    X_PLANE_MAX = X_NEW + 0.45

    branches = [
        {"label": "Branch A", "link_old": 9,  "color": "#e74c3c"},   # red
        {"label": "Branch B", "link_old": 4,  "color": "#2980b9"},   # blue
        {"label": "Branch C", "link_old": 10, "color": "#27ae60"},   # green (edge)
    ]

    # segment colours
    old_colors = ["#aec6e8", "#c8dbf0"]
    new_colors = ["#a8d5a2", "#c5e8bf"]

    # ── figure ────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(13, 8))
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_box_aspect([2.8, 1, 1.2])

    # ── draw trunks ───────────────────────────────────────────────────────────
    draw_trunk(ax, X_OLD, N_OLD, TOTAL_H, TRUNK_R, old_colors)
    draw_trunk(ax, X_NEW, N_NEW, TOTAL_H, TRUNK_R, new_colors)

    link_h_old = TOTAL_H / N_OLD
    link_h_new = TOTAL_H / N_NEW

    # ── for each branch: compute positions, draw plane + arrows ───────────────
    print(f"\n{'='*60}")
    print(f"  Remapping  N={N_OLD} → N={N_NEW}")
    print(f"{'='*60}")
    print(f"  {'Branch':<10} {'Old link':>8} {'H':>6} {'New link':>10} {'p_new':>7}  {'y_old':>7} {'y_new':>7}")
    print(f"  {'-'*60}")

    for b in branches:
        link_old = b["link_old"]
        color    = b["color"]
        lbl      = b["label"]

        # absolute height (top of link, p=1.0)
        z_old = link_old * link_h_old

        # remap
        k_new, p_new = remap(link_old, N_OLD, N_NEW)
        z_new = (k_new - 1) * link_h_new + p_new * link_h_new

        err = abs(z_old - z_new)
        status = "✓" if err < 1e-9 else f"Δ={err:.2e}"

        print(f"  {lbl:<10} L{link_old:>2}       {link_old/N_OLD:.2f}   "
              f"L{k_new} @{p_new*100:5.1f}%   {z_old:.4f}   {z_new:.4f}  {status}")

        # height plane spanning both trunks
        draw_height_plane(ax, z_old, X_PLANE_MIN, X_PLANE_MAX, color, alpha=0.12)

        # old trunk branch
        draw_branch_arrow(ax, X_OLD + TRUNK_R, z_old, color,
                          label=f"{lbl}: L{link_old}")
        # new trunk branch
        p_pct = f"{p_new*100:.0f}%"
        draw_branch_arrow(ax, X_NEW + TRUNK_R, z_new, color,
                          label=f"{lbl}: L{k_new} @ {p_pct}")

    print(f"{'='*60}\n")

    # ── column headers ────────────────────────────────────────────────────────
    ax.text(X_OLD, 0, TOTAL_H + 0.07,
            f"BEFORE\n({N_OLD} segments)", ha="center", va="bottom",
            fontsize=10, fontweight="bold", color="#1a5276")
    ax.text(X_NEW, 0, TOTAL_H + 0.07,
            f"AFTER\n({N_NEW} segments)", ha="center", va="bottom",
            fontsize=10, fontweight="bold", color="#145a32")

    # ── axes & labels ─────────────────────────────────────────────────────────
    ax.set_xlabel("", labelpad=0)
    ax.set_ylabel("", labelpad=0)
    ax.set_zlabel("Height (m)", fontsize=9, labelpad=6)

    ax.set_xticks([])
    ax.set_yticks([])
    z_ticks = np.linspace(0, TOTAL_H, N_OLD + 1)
    ax.set_zticks(z_ticks)
    ax.set_zticklabels([f"{z:.1f}" for z in z_ticks], fontsize=7)

    ax.set_xlim(X_PLANE_MIN, X_PLANE_MAX + 0.5)
    ax.set_ylim(-0.8, 0.8)
    ax.set_zlim(-0.05, TOTAL_H + 0.15)

    ax.view_init(elev=18, azim=-55)
    ax.grid(False)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    # ── legend ────────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor="#aec6e8", edgecolor="gray", label=f"Old trunk segment (N={N_OLD})"),
        mpatches.Patch(facecolor="#a8d5a2", edgecolor="gray", label=f"New trunk segment (N={N_NEW})"),
        mpatches.Patch(facecolor="#e74c3c", alpha=0.3, label="Branch A  L9 → L3 @ 70%"),
        mpatches.Patch(facecolor="#2980b9", alpha=0.3, label="Branch B  L4 → L2 @ 20%"),
        mpatches.Patch(facecolor="#27ae60", alpha=0.3, label="Branch C  L10 → L3 @ 100% (edge)"),
    ]
    ax.legend(handles=legend_handles, loc="upper left", fontsize=8,
              framealpha=0.9, bbox_to_anchor=(0.01, 0.99))

    fig.suptitle(
        f"3D Branch Attachment Remapping  —  N={N_OLD} → N={N_NEW}\n"
        "Transparent planes confirm each branch sits at the same absolute height",
        fontsize=12, fontweight="bold",
    )
    plt.tight_layout()

    # save
    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_png = os.path.join(script_dir, "remapping_3d_new.png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"[OK] Saved PNG: {out_png}")

    plt.show()


if __name__ == "__main__":
    main()
