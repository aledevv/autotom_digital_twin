"""Plot recorded scaling costs; never infer a capacity beyond measured counts."""

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("campaign", type=Path)
    args = parser.parse_args()
    rows = json.loads((args.campaign / "scaling.json").read_text())
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 4), layout="constrained")
    counts = [r["count"] for r in rows]
    for render, label in ((False, "Headless"), (True, "Rendering 60 Hz")):
        runs = [
            [r["timing"] for r in row["repetitions"] if r["render"] == render]
            for row in rows
        ]
        for ax, key, scale in (
            (axes[0], "realtime_factor", 1.0),
            (axes[1], "frame_work", 1000.0),
        ):
            vals = [
                [
                    r[key] if key == "realtime_factor" else r[key]["p95_seconds"]
                    for r in group
                ]
                for group in runs
            ]
            avg = np.array([np.mean(v) for v in vals]) * scale
            low = np.array([min(v) for v in vals]) * scale
            high = np.array([max(v) for v in vals]) * scale
            ax.errorbar(
                counts,
                avg,
                yerr=[avg - low, high - avg],
                marker="o",
                capsize=4,
                label=label,
            )
    components = ("physics", "read", "skinning", "diagnostics", "render")
    bottom = np.zeros(len(counts))
    for key in components:
        values = []
        for row in rows:
            ts = [r["timing"] for r in row["repetitions"] if r["render"]]
            values.append(
                np.mean(
                    [
                        t["components"][key]["total_seconds"]
                        / t["frame_work"]["count"]
                        * 1000
                        for t in ts
                    ]
                )
            )
        axes[2].bar(np.arange(len(counts)), values, bottom=bottom, label=key)
        bottom += values
    total_mean = []
    for row in rows:
        ts = [r["timing"] for r in row["repetitions"] if r["render"]]
        total_mean.append(
            np.mean(
                [
                    t["frame_work"]["total_seconds"] / t["frame_work"]["count"] * 1000
                    for t in ts
                ]
            )
        )
    axes[2].bar(
        np.arange(len(counts)),
        np.maximum(0.0, np.asarray(total_mean) - bottom),
        bottom=bottom,
        label="other control/overhead",
    )
    axes[0].axhline(1, color="black", linestyle="--", linewidth=1)
    axes[1].axhline(1000 / 60, color="black", linestyle="--", linewidth=1)
    axes[0].set(
        ylabel="Tempo simulato / tempo reale",
        title="Velocità: media e intervallo di 3 prove",
    )
    axes[1].set(
        ylabel="p95 lavoro per frame [ms]", title="Due passi fisici + un rendering"
    )
    axes[2].set(
        xticks=np.arange(len(counts)),
        xticklabels=counts,
        ylabel="Costo medio per frame [ms]",
        title="Componenti misurate, con rendering",
    )
    for ax in axes:
        ax.set_xlabel("Numero di lamine")
        ax.grid(axis="y", alpha=0.25)
        ax.legend(fontsize=8)
    axes[0].set_xticks(counts)
    axes[1].set_xticks(counts)
    axes[2].legend(loc="upper left", fontsize=8)
    fig.savefig(args.campaign / "scaling.png", dpi=160)


if __name__ == "__main__":
    main()
