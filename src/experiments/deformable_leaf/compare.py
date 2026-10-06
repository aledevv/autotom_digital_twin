"""Compare matched D0 runs at 60/120/240 Hz; creates JSON and scientific PNG."""
import argparse
import json
from pathlib import Path

import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("runs", type=Path, nargs=3)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    records = []
    reference = None
    sources = None
    for run in a.runs:
        config = json.loads((run/"config.json").read_text())
        hz = config.pop("hz")
        launch = json.loads((run/"launch.json").read_text())
        source = launch["source_sha256"]
        if reference is not None and (config != reference or source != sources):
            raise ValueError("Cases must differ only in physics Hz, with identical implementation")
        reference, sources = config, source
        report = json.loads((run/"report.json").read_text())
        if (not report.get("process_ok") or not report.get("checks", {}).get("complete")
                or report.get("gui") or report.get("offscreen_render")):
            raise ValueError(f"Expected a completed headless run without process errors: {run}")
        records.append((hz, run, report))
    records.sort()
    if [r[0] for r in records] != [60, 120, 240]:
        raise ValueError("Need exactly 60, 120, 240 Hz")
    a.output.mkdir(parents=True, exist_ok=False)
    high_sag = records[-1][2]["tip_sag_m"]
    deltas = {str(hz): abs(r["tip_sag_m"]-high_sag)/max(abs(high_sag), 1e-8)
              for hz, _, r in records}
    result = dict(cases=[dict(hz=hz, run=str(run.resolve()), report=r) for hz, run, r in records],
                  sag_relative_difference_from_240hz=deltas,
                  sensitivity_screen_passed=max(deltas.values()) <= 0.10,
                  all_individual_screens_passed=all(r["status"] == "passed" for _, _, r in records),
                  relative_tolerance=0.10, manual_review="pending",
                  note="Agreement in this fixture is a numerical screening check, not mesh convergence or biological validation.")
    (a.output/"comparison.json").write_text(json.dumps(result, indent=2)+"\n")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True, constrained_layout=True)
    for hz, run, _ in records:
        with np.load(run/"trace.npz") as trace:
            tip = trace["landmarks_support_m"][:, 0, 2]
            axes[0].plot(trace["time_s"], (tip[0]-tip)*1000, label=f"{hz} Hz")
            rest = np.load(run/"rest_mesh.npz")["points"]
            c = json.loads((run/"config.json").read_text())
            mask = rest[:, 0] <= c["clamp_length"]+1e-7
            drift = np.linalg.norm(trace["nodes_world_m"][:, mask]-rest[mask], axis=-1).max(axis=1)
            axes[1].plot(trace["time_s"], drift*1000, label=f"{hz} Hz")
    for axis in axes:
        axis.axvline(reference["gravity_seconds"], color="black", linestyle="--", label="Gravity off")
        axis.grid(alpha=0.25)
        axis.legend()
    axes[0].set_ylabel("Tip downward displacement [mm]")
    axes[1].set_ylabel("Maximum clamp drift [mm]")
    axes[1].set_xlabel("Simulated time [s]")
    fig.suptitle("D0 — isolated volumetric strip, TGS/GPU")
    fig.savefig(a.output/"comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps({k: v for k, v in result.items() if k != "cases"}, indent=2))


if __name__ == "__main__":
    main()
