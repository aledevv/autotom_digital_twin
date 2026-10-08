"""Presentation figures for the complexity study (static PNG, light surface).

    uv run python src/experiments/complexity_study/make_figures.py \
        --push push_all.json --fidelity fidelity_compare.json \
        --skeleton ref=ref.usda --skeleton b120=b120.usda ... --out figures/

Palette: the dataviz skill's validated categorical slots 1-4, assigned in
fixed order; the reference is neutral ink. Two slots are below 3:1 contrast
on the light surface; the relief is the always-present legend plus the
labelled points of the cost/fidelity figure.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SERIES = {  # entity -> colour, fixed for every figure
    "ref": INK,
    "b40_load": "#2a78d6",
    "b40_keep": "#eb6834",
    "b40_series": "#1baf7a",
    "g206_255bodies": "#eda100",
}
NAMES = {
    "ref": "reference (206 D6, 216 bodies)",
    "b120_load": "120 D6, load",
    "b60_load": "60 D6, load",
    "b40_load": "40 D6, load",
    "b40_keep": "40 D6, keep (legacy rigid)",
    "b40_series": "40 D6, series",
    "g206_255bodies": "206 D6, 255 bodies",
}
PROBES = {"lateral_tip": "Lateral branch tip", "truss_tip": "Truss rachis tip", "leaf_blade": "Leaf blade"}


def _style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "xtick.color": INK_2, "ytick.color": INK_2,
        "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
        "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 11, "axes.titlesize": 13, "axes.titleweight": "bold", "axes.titlecolor": INK,
        "lines.linewidth": 2.0, "legend.frameon": False,
    })


def push_trajectories(push: dict, out: Path) -> Path:
    results = push["results"]
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), sharex=True)
    for ax, (probe, title) in zip(axes, PROBES.items()):
        ax.axvspan(0, push["push_duration_s"], color=GRID, alpha=0.7, lw=0)
        ax.text(push["push_duration_s"] + 0.04, 0.03, "push", transform=ax.get_xaxis_transform(),
                ha="left", va="bottom", color=INK_2, fontsize=9)
        ax.axhline(0, color=INK_2, lw=0.8)
        for stage, colour in SERIES.items():
            row = next((r for r in results if r["stage"] == stage and r["probe"] == probe), None)
            if row is None:
                continue
            t, y = row["trajectory"]["t"], row["trajectory"]["along_mm"]
            ax.plot(t, y, color=colour, lw=2.6 if stage == "ref" else 2.0, zorder=3 if stage == "ref" else 2,
                    label=NAMES[stage])
        force = next(r["force_n"] for r in results if r["probe"] == probe)
        ax.set_title(f"{title}  (F = {force:.2f} N)", loc="left")
        ax.set_xlabel("time since push start [s]")
    axes[0].set_ylabel("displacement along push [mm]")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=5, bbox_to_anchor=(0.5, -0.02))
    fig.suptitle("Lateral push equal to the downstream weight: response of optimized plants vs reference",
                 x=0.01, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.07, 1, 0.95))
    path = out / "push_trajectories.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def cost_vs_fidelity(fidelity: list[dict], push: dict, out: Path) -> Path:
    rows = {r["variant"]: r for r in fidelity}
    ref_wall = rows["ref"]["wall_s"]
    ref_peak = {r["probe"]: r["peak_mm"] for r in push["results"] if r["stage"] == "ref"}
    fig, (left, right) = plt.subplots(1, 2, figsize=(13, 4.8))
    for variant, row in rows.items():
        if variant == "ref":
            continue
        speed = ref_wall / row["wall_s"]
        dynamic = np.mean([
            abs(r["peak_mm"] - ref_peak[r["probe"]]) / ref_peak[r["probe"]] * 100
            for r in push["results"] if r["stage"] == variant
        ])
        colour = SERIES.get(variant, "#2a78d6")
        marker = "o" if variant.endswith("load") else "s"
        for ax, value in ((left, row["mean_mm"]), (right, dynamic)):
            ax.scatter(speed, value, s=90, color=colour, marker=marker, edgecolor=SURFACE, linewidth=2, zorder=3)
            above = variant == "g206_255bodies"
            ax.annotate(NAMES[variant], (speed, value), xytext=(0, 10) if above else (8, 4),
                        textcoords="offset points", ha="center" if above else "left",
                        fontsize=9, color=INK)
    # The load-policy budget sweep as a connected path.
    sweep = [v for v in ("b120_load", "b60_load", "b40_load") if v in rows]
    xs = [ref_wall / rows[v]["wall_s"] for v in sweep]
    left.plot(xs, [rows[v]["mean_mm"] for v in sweep], color="#2a78d6", lw=1.2, alpha=0.6, zorder=2)
    right.plot(xs, [np.mean([abs(r["peak_mm"] - ref_peak[r["probe"]]) / ref_peak[r["probe"]] * 100
                             for r in push["results"] if r["stage"] == v]) for v in sweep],
               color="#2a78d6", lw=1.2, alpha=0.6, zorder=2)
    left.set_title("Static fidelity (gravity settle)", loc="left")
    left.set_ylabel("mean organ position error [mm]")
    right.set_title("Dynamic fidelity (lateral push)", loc="left")
    right.set_ylabel("mean |peak - reference peak| [% of reference]")
    for ax in (left, right):
        ax.set_xlabel("physics speed-up vs reference [x]  (480 Hz, RTX 4080 Laptop)")
        ax.set_ylim(bottom=0)
        ax.set_xlim(0.6, max(xs + [2.8]) + 0.4)
    fig.suptitle("Cost vs fidelity of the joint-budget optimizer (day 160)", x=0.01, ha="left",
                 fontsize=14, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    path = out / "cost_vs_fidelity.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def _skeleton(usd: Path):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.Open(str(usd))
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    segments = []
    for index, body in enumerate(bodies):
        for prim in Usd.PrimRange(body):
            if prim.IsA(UsdGeom.Capsule):
                capsule = UsdGeom.Capsule(prim)
                half = 0.5 * capsule.GetHeightAttr().Get() + capsule.GetRadiusAttr().Get()
                world = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
                a = world.Transform(Gf.Vec3d(0, 0, -half))
                b = world.Transform(Gf.Vec3d(0, 0, half))
                segments.append((index, (a[0], a[2]), (b[0], b[2])))
    d6 = []
    for prim in stage.Traverse():
        if prim.GetTypeName() == "PhysicsJoint":
            target = prim.GetRelationship("physics:body1").GetTargets()[0]
            world = UsdGeom.Xformable(stage.GetPrimAtPath(target)).ComputeLocalToWorldTransform(0)
            p = world.Transform(Gf.Vec3d(prim.GetAttribute("physics:localPos1").Get()))
            d6.append((p[0], p[2]))
    return segments, np.asarray(d6), len(bodies)


def complexity_skeletons(stages: list[tuple[str, Path]], out: Path) -> Path:
    fig, axes = plt.subplots(1, len(stages), figsize=(4.2 * len(stages), 6.2), sharey=True)
    tones = ("#8a8984", "#c9c8c2")
    for ax, (label, usd) in zip(np.atleast_1d(axes), stages):
        segments, d6, bodies = _skeleton(usd)
        for index, a, b in segments:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=tones[index % 2], lw=2.2, solid_capstyle="round")
        if len(d6):
            ax.scatter(d6[:, 0], d6[:, 1], s=26, color="#2a78d6", edgecolor=SURFACE, linewidth=1.2, zorder=3)
        ax.set_title(f"{len(d6)} D6 joints | {bodies} bodies", loc="center")
        ax.text(0.5, -0.04, NAMES.get(label, label), transform=ax.transAxes, ha="center", va="top",
                color=INK_2, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(False)
        ax.set_xticks([])
        ax.set_yticks([])
        for spine in ax.spines.values():
            spine.set_visible(False)
    fig.suptitle("Same plant, fewer joints: blue = flexible D6 joint, grey tones = rigid bodies",
                 x=0.01, ha="left", fontsize=14, fontweight="bold", color=INK)
    fig.tight_layout(rect=(0, 0.02, 1, 0.94))
    path = out / "complexity_skeletons.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--push", type=Path, required=True)
    parser.add_argument("--fidelity", type=Path, required=True)
    parser.add_argument("--skeleton", action="append", default=[], help="LABEL=USD")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    _style()
    push = json.loads(args.push.read_text())
    fidelity = json.loads(args.fidelity.read_text())
    paths = [push_trajectories(push, args.out), cost_vs_fidelity(fidelity, push, args.out)]
    if args.skeleton:
        stages = [(item.partition("=")[0], Path(item.partition("=")[2])) for item in args.skeleton]
        paths.append(complexity_skeletons(stages, args.out))
    for path in paths:
        print(path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
