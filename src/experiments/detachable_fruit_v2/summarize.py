"""Create a local comparison table and static scientific plot from case reports."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rows, plot_cases = [], []
    for directory in args.run_dirs:
        path = directory / "report.json"
        if not path.exists():
            rows.append({"case": directory.name, "status": "not_run"})
            continue
        report = json.loads(path.read_text())
        config = report.get("config", {})
        effective_path = directory / "headless-effective.json"
        effective = json.loads(effective_path.read_text()) if effective_path.exists() else {}
        actual_solver = effective.get("scene", {}).get("physxScene:solverType")
        configuration_valid = actual_solver == config.get("solver")
        row = {"case": directory.name, "status": report["status"],
               "simulated_seconds": report.get("simulated_seconds"),
               "wall_seconds": report.get("wall_seconds"),
               "breaks": len(report.get("events", [])), "body_count": report.get("body_count"),
               "fruit_count": report.get("fruit_count"), "tail": report.get("tail", {}),
               "errors": report.get("errors", []), "solver": config.get("solver"),
               "gpu": config.get("gpu"), "scenario": config.get("scenario"),
               "actual_solver": actual_solver, "configuration_valid": configuration_valid,
               "first_break_s": next((e["time_s"] for e in report.get("events", []) if e["kind"] == "joint_break"), None),
               "completed": report.get("simulated_seconds", 0) >= config.get("duration", 20) - .5 / config.get("hz", 480),
               "max_attachment_error_m": report.get("max_attachment_error_m")}
        if not configuration_valid:
            row["status"] = "invalid_configuration"
        recheck = directory / "pose-velocity-check.json"
        if recheck.exists():
            corrected = json.loads(recheck.read_text())
            row["tail"] = dict(row["tail"])
            for key in ("kinematic_linear_speed_mps", "kinematic_angular_speed_radps"):
                row["tail"][key] = corrected[key]
            row["kinematic_source"] = recheck.name
            row["errors_note"] = "Errors retain the original runtime report; kinematic metrics use the independent recheck."
        rows.append(row)
        metrics = directory / "headless-metrics.npz"
        if metrics.exists() and configuration_valid:
            plot_cases.append((directory.name, np.load(metrics)["samples"], config.get("hz", 480)))
    (args.output / "comparison.json").write_text(json.dumps(rows, indent=2) + "\n")
    lines = ["# ExporterV2 fruit comparison", "", "Headless engineering gates; GUI acceptance remains pending.",
             "Tail speeds are shown only for completed runs; interrupted screens describe startup, not settling.", "",
             "| Case | Status | Simulated s | First break s | Breaks | Tail v (m/s) | Tail w (rad/s) | Max attachment gap (m) |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    def number(v):
        return "—" if v is None else f"{v:.5g}"
    for r in rows:
        tail = r.get("tail", {}) if r.get("completed") else {}
        lines.append(f"| {r['case']} | {r['status']} | {number(r.get('simulated_seconds'))} | {number(r.get('first_break_s'))} | {r.get('breaks', '—')} | {number(tail.get('linear_speed_mps'))} | {number(tail.get('angular_speed_radps'))} | {number(r.get('max_attachment_error_m'))} |")
    (args.output / "comparison.md").write_text("\n".join(lines) + "\n")
    if plot_cases:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        figure, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, layout="constrained")
        for name, samples, hz in plot_cases:
            if samples.size:
                # Per-time-bin maxima retain fast spikes while limiting plot size.
                width = max(1, round(.05 * hz))
                blocks = [samples[i:i + width] for i in range(0, len(samples), width)]
                t = [b[-1, 0] for b in blocks]
                for ax, col in zip(axes, (1, 2, 3)):
                    ax.plot(t, [max(1e-10, b[:, col].max()) for b in blocks], label=name, linewidth=1)
        for ax, label, limit in zip(axes, ("Max linear speed (m/s)", "Max angular speed (rad/s)", "Attached joint gap (m)"), (.005, .05, .0005)):
            ax.set_yscale("log")
            ax.axhline(limit, color="black", linestyle="--", linewidth=.8)
            ax.set_ylabel(label)
            ax.grid(alpha=.25)
        axes[-1].set_xlabel("Simulated time (s)")
        axes[0].legend(fontsize=8, ncol=2)
        rates = ", ".join(str(hz) for hz in sorted({hz for _, _, hz in plot_cases}))
        figure.suptitle(f"ExporterV2 — day 160 PlantState; {rates} Hz\nPer-step measurements, 50 ms maxima; dashed lines are engineering thresholds")
        figure.savefig(args.output / "comparison.png", dpi=150)
        plt.close(figure)
    print(args.output / "comparison.md")


if __name__ == "__main__":
    main()
