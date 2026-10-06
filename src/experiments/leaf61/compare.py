"""Compare completed benches without converting failed trials into accepted candidates."""

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rows = []
    datasets = []
    for directory in args.runs:
        report = json.loads((directory / "report.json").read_text())
        runtime = json.loads((directory / "runtime.json").read_text())
        config = runtime["config"]
        rows.append(
            {
                "run": str(directory),
                "model": runtime["model"],
                "shape": runtime["shape"],
                "hz": config["hz"],
                "report": report,
                "runtime": runtime,
            }
        )
        data = np.load(directory / "trace.npz")
        rest = np.load(directory / "rest_mesh.npz")
        datasets.append((config, data, rest))
    baseline = rows[0]["runtime"]
    shared = (
        "length",
        "width",
        "fixed_length",
        "thickness",
        "density",
        "radius",
        "depth",
        "settle",
        "recovery",
    )
    comparable = all(
        row["shape"] == baseline["shape"]
        and all(
            row["runtime"]["config"][key] == baseline["config"][key] for key in shared
        )
        for row in rows
    )
    summary = {
        "same_fixture_and_protocol": comparable,
        "runs": rows,
        "visual_acceptance": "pending",
    }
    (args.output / "comparison.json").write_text(json.dumps(summary, indent=2) + "\n")
    lines = [
        "# Confronto foglie 6.1",
        "",
        f"Fixture e protocollo comuni: {comparable}. Revisione visiva: da eseguire.",
        "",
        "| Run | Stato | Sag mm | Deriva mm | Recupero peggiore mm | Velocità / tempo reale |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        r = row["report"]
        m = r.get("metrics", {})
        cycles = r.get("cycles", [])
        recovery = max((v["recovery_m"] for v in cycles), default=None)
        values = [m.get("sag_m"), m.get("attachment_drift_m"), recovery]
        formatted = ["—" if v is None else f"{1000 * v:.3f}" for v in values]
        lines.append(
            f"| {Path(row['run']).name} | {r['status']} | "
            + " | ".join(formatted)
            + f" | {r['timing']['instrumented_realtime_factor']:.2f} |"
        )
    (args.output / "comparison.md").write_text("\n".join(lines) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
    for row, (c, data, rest) in zip(rows, datasets):
        times = data["times"]
        points = data["points"]
        p = rest["points"]
        tip = p[:, 0] > p[:, 0].max() - 0.0031
        label = Path(row["run"]).name
        axes[0].plot(
            times,
            (p[tip, 2].mean() - points[:, tip, 2].mean(axis=1)) * 1000,
            label=label,
        )
        frame = points[np.argmin(abs(times - (1 + c["settle"])))]
        # Choose the nearest-to-centre vertex at each longitudinal station.
        stations = np.unique(np.round(p[:, 0], 6))
        ids = [
            np.flatnonzero(abs(p[:, 0] - x) < 1e-6)[
                np.argmin(abs(p[abs(p[:, 0] - x) < 1e-6, 1]))
            ]
            for x in stations
        ]
        axes[1].plot(
            frame[ids, 0] * 1000, (frame[ids, 2] - c["height"]) * 1000, label=label
        )
    axes[0].set(xlabel="Tempo simulato [s]", ylabel="Cedimento medio punta [mm]")
    axes[1].set(
        xlabel="X [mm]",
        ylabel="Z rispetto alla base [mm]",
        title="Equilibrio sotto gravità",
    )
    axes[1].set_aspect("equal", adjustable="datalim")
    for ax in axes:
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)
    fig.savefig(args.output / "comparison.png", dpi=160)


if __name__ == "__main__":
    main()
