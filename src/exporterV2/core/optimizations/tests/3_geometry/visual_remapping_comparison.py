"""
visual_remapping_comparison.py - Before/After Remapping Comparison

Shows BOTH the original and remapped trunks in the SAME figure,
with horizontal height lines spanning both columns to prove that
branches sit at exactly the same absolute height before and after.

Mathematical model (p=1.0, top-of-link convention):
    H_abs = attach_link / N_orig          (fraction of total height, 0..1)
    V     = H_abs * N_new
    k_new = floor(V) + 1                  (1-based, clamped to N_new)
    p_new = V - floor(V)                  (fractional offset within new link)

    Special case: H_abs >= 1.0 → k_new = N_new, p_new = 1.0

Two user-provided examples are shown together:
    Example A: N=10 → N=3, branch at link 9  → segment 3 @ 70%
    Example B: N=10 → N=5, branch at link 4  → segment 2 @ 75% (approx.)

Run with:
    cd /home/alessandro/isaacsim/autotom_digital_twin
    uv run python src/exporterV2/core/optimizations/tests/3_geometry/visual_remapping_comparison.py
"""

import math
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch

# ── imports from optimizations (best-effort) ─────────────────────────────────
script_dir = os.path.dirname(os.path.abspath(__file__))
optimizations_dir = os.path.join(script_dir, "../..")
sys.path.insert(0, optimizations_dir)


# ── Core remapping math (self-contained, p=1.0 convention) ───────────────────

def absolute_height(attach_link: int, n_links: int, link_height: float = 1.0) -> float:
    """
    Absolute height of the attachment point (top of link, p=1.0).
    H_abs = attach_link * link_height
    """
    return attach_link * link_height


def remap(attach_link_1based: int, n_old: int, n_new: int) -> tuple[int, float]:
    """
    Remap 1-based attach_link from n_old to n_new segments.
    Returns (new_attach_link_1based, p_new) where p_new ∈ [0, 1].

    Convention: branch attaches at TOP of its link (p_orig = 1.0).
    H = attach_link / n_old
    V = H * n_new
    k_new = floor(V) + 1   (clamped to n_new)
    p_new = V - floor(V)   (== 0.0 means exactly at link boundary)

    Edge case: attach_link == n_old → H = 1.0 → top of last new segment.
    """
    H = attach_link_1based / n_old          # absolute height fraction ∈ (0, 1]
    if H >= 1.0:
        return n_new, 1.0
    V = H * n_new
    k_new = math.floor(V) + 1              # 1-based
    p_new = V - math.floor(V)             # fractional position within new link
    return k_new, p_new


# ── Drawing helpers ───────────────────────────────────────────────────────────

# Column positions (x-centre of each trunk)
COL_OLD = 0.0
COL_NEW = 3.5

TRUNK_W  = 0.55    # visual trunk half-width
BRANCH_L = 1.4     # horizontal branch length drawn
BRANCH_H = 0.04    # branch visual half-height


def draw_trunk(ax, x_center, n_links, total_h, link_colors, label):
    """Draw a segmented trunk column."""
    link_h = total_h / n_links
    for i in range(n_links):
        y0 = i * link_h
        y1 = y0 + link_h
        col = link_colors[i % len(link_colors)]
        rect = FancyBboxPatch(
            (x_center - TRUNK_W / 2, y0),
            TRUNK_W, link_h - 0.005,          # tiny gap between links
            boxstyle="round,pad=0.005",
            linewidth=1.5,
            edgecolor="#555",
            facecolor=col,
            alpha=0.55,
            zorder=2,
        )
        ax.add_patch(rect)
        # Link index label
        ax.text(
            x_center, y0 + link_h / 2,
            f"L{i+1}",
            ha="center", va="center",
            fontsize=8, fontweight="bold", color="#333",
            zorder=3,
        )
    # Trunk header
    ax.text(
        x_center, total_h + 0.07,
        label,
        ha="center", va="bottom",
        fontsize=10, fontweight="bold", color="#222",
    )


def draw_branch(ax, x_center, y_attach, color, label, side="right"):
    """Draw a horizontal branch at y_attach."""
    dx = BRANCH_L if side == "right" else -BRANCH_L
    ax.annotate(
        "",
        xy=(x_center + dx, y_attach),
        xytext=(x_center + TRUNK_W / 2 * (1 if side == "right" else -1), y_attach),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=2.5),
        zorder=4,
    )
    offset_y = 0.055
    ax.text(
        x_center + dx * 1.03,
        y_attach + offset_y,
        label,
        ha="left" if side == "right" else "right",
        va="bottom",
        fontsize=8.5,
        color=color,
        fontweight="bold",
        zorder=5,
    )
    # Dot at junction
    ax.plot(
        x_center + TRUNK_W / 2 * (1 if side == "right" else -1),
        y_attach,
        "o",
        color=color,
        ms=8,
        markeredgecolor="white",
        markeredgewidth=1.5,
        zorder=5,
    )


def draw_height_band(ax, y_abs, total_h, color, alpha=0.12):
    """Draw a horizontal highlight band at y_abs spanning both columns."""
    ax.axhspan(y_abs - 0.012, y_abs + 0.012, color=color, alpha=alpha, zorder=0)
    ax.axhline(y_abs, color=color, linewidth=1.4, linestyle="--", alpha=0.75, zorder=1)


def draw_height_label(ax, y_abs, H_frac, color, x_pos):
    """Annotate the absolute height fraction on the right margin."""
    ax.text(
        x_pos, y_abs,
        f"  H={H_frac:.0%}",
        ha="left", va="center",
        fontsize=8, color=color, style="italic",
    )


# ── Per-scenario plot ─────────────────────────────────────────────────────────

def draw_scenario(ax, title, n_old, n_new, branches, total_h=1.0):
    """
    Draw one scenario on ax.
    branches: list of dict {label, attach_link_old, color}
    """
    link_h_old = total_h / n_old
    link_h_new = total_h / n_new

    # Trunk segment colors (alternating greys)
    trunk_colors_old = ["#b5c9e0", "#d4e4f3"]
    trunk_colors_new = ["#b5d4bb", "#d0edda"]

    draw_trunk(ax, COL_OLD, n_old, total_h, trunk_colors_old,
               f"BEFORE  ({n_old} segments\n@ {link_h_old:.3f}m each)")
    draw_trunk(ax, COL_NEW, n_new, total_h, trunk_colors_new,
               f"AFTER  ({n_new} segments\n@ {link_h_new:.3f}m each)")

    for b in branches:
        link_old  = b["attach_link_old"]
        color     = b["color"]
        label_base = b["label"]

        # Absolute height of attachment (top of link, p=1.0)
        y_abs = link_old * link_h_old          # H_abs * total_h

        # Remap
        k_new, p_new = remap(link_old, n_old, n_new)
        y_new = (k_new - 1) * link_h_new + p_new * link_h_new

        H_frac = link_old / n_old

        # Height band spanning both trunks
        draw_height_band(ax, y_abs, total_h, color)

        # Labels
        label_old = f"{label_base}\nL{link_old} (top)"
        label_new = (
            f"{label_base}\nL{k_new} @ {p_new*100:.0f}%"
            if p_new > 0.001
            else f"{label_base}\nL{k_new} (top)"
        )

        # Old branch (right side of old trunk)
        draw_branch(ax, COL_OLD, y_abs, color, label_old, side="right")
        # New branch (right side of new trunk)
        draw_branch(ax, COL_NEW, y_new, color, label_new, side="right")

        # Height fraction annotation on the far right
        draw_height_label(ax, y_abs, H_frac, color, COL_NEW + TRUNK_W / 2 + BRANCH_L + 0.1)

        # Error check (should be ~0)
        err = abs(y_abs - y_new)
        status = "✓" if err < 1e-9 else f"Δ={err:.4f}"
        print(f"  {label_base:12s}  old=L{link_old}  H={H_frac:.2f}  "
              f"new=L{k_new}@{p_new*100:.1f}%  "
              f"y_old={y_abs:.4f}  y_new={y_new:.4f}  {status}")

    # Axes
    margin = 0.18
    ax.set_xlim(COL_OLD - TRUNK_W / 2 - 0.2, COL_NEW + TRUNK_W / 2 + BRANCH_L + 0.85)
    ax.set_ylim(-margin, total_h + margin)
    ax.set_yticks([i * link_h_old for i in range(n_old + 1)])
    ax.set_yticklabels([f"{i*link_h_old:.2f}m" for i in range(n_old + 1)], fontsize=8)
    ax.yaxis.set_tick_params(length=3)
    ax.set_xticks([COL_OLD, COL_NEW])
    ax.set_xticklabels(["Before", "After"], fontsize=10)
    ax.set_title(title, fontsize=12, fontweight="bold", pad=14)
    ax.spines[["top", "right", "bottom"]].set_visible(False)
    ax.grid(axis="y", alpha=0.25, linestyle=":")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 65)
    print("  Attachment Remapping - Side-by-Side Comparison")
    print("  Convention: branch attaches at TOP of link (p = 1.0)")
    print("=" * 65)

    # ── Scenario definitions ──────────────────────────────────────────────────

    scenarios = [
        {
            "title": "Example A  —  N=10 → N=3   (user's example)",
            "n_old": 10,
            "n_new": 3,
            "total_h": 1.0,
            "branches": [
                {"label": "Branch A", "attach_link_old": 9,  "color": "#c0392b"},
                {"label": "Branch B", "attach_link_old": 4,  "color": "#2980b9"},
                {"label": "Branch C", "attach_link_old": 10, "color": "#27ae60"},  # edge case: top
            ],
        },
        {
            "title": "Example B  —  N=10 → N=5",
            "n_old": 10,
            "n_new": 5,
            "total_h": 1.0,
            "branches": [
                {"label": "Branch A", "attach_link_old": 4,  "color": "#8e44ad"},
                {"label": "Branch B", "attach_link_old": 7,  "color": "#e67e22"},
                {"label": "Branch C", "attach_link_old": 10, "color": "#16a085"},  # edge case: top
            ],
        },
    ]

    n_scenarios = len(scenarios)
    fig, axes = plt.subplots(1, n_scenarios, figsize=(10 * n_scenarios, 9))
    if n_scenarios == 1:
        axes = [axes]

    fig.suptitle(
        "Branch Attachment Remapping — Same Absolute Height Before & After\n"
        "Dashed lines + shaded bands confirm geometric preservation",
        fontsize=13,
        fontweight="bold",
        y=1.01,
    )

    for ax, sc in zip(axes, scenarios):
        print(f"\n{'─'*65}")
        print(f"  {sc['title']}")
        print(f"{'─'*65}")
        draw_scenario(
            ax,
            title=sc["title"],
            n_old=sc["n_old"],
            n_new=sc["n_new"],
            branches=sc["branches"],
            total_h=sc["total_h"],
        )

    # Shared legend
    legend_elements = [
        mpatches.Patch(facecolor="#b5c9e0", edgecolor="#555", label="Old trunk segment"),
        mpatches.Patch(facecolor="#b5d4bb", edgecolor="#555", label="New trunk segment"),
        plt.Line2D([0], [0], color="gray", lw=1.4, ls="--", label="Constant absolute height"),
    ]
    fig.legend(
        handles=legend_elements,
        loc="lower center",
        ncol=3,
        fontsize=9,
        frameon=True,
        bbox_to_anchor=(0.5, -0.04),
    )

    plt.tight_layout()

    # Save PNG next to script
    out_png = os.path.join(script_dir, "remapping_comparison.png")
    plt.savefig(out_png, dpi=150, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_png}")

    plt.show()

    print("\n" + "=" * 65)
    print("  Key check: y_old ≈ y_new for every branch  (Δ should be 0)")
    print("  Edge case H=1.0 → clamped to last segment  (no out-of-range)")
    print("=" * 65)


if __name__ == "__main__":
    main()
