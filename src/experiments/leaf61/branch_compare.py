"""Compare recorded branch scaling runs against a single-branch trajectory."""

import argparse
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("runs", nargs="+", type=Path)
    a = parser.parse_args()
    cfg = json.loads((a.reference / "config.json").read_text())
    ref = np.load(a.reference / "trace-000.npz")
    keep = [i for i, p in enumerate(cfg["pose_paths"]) if not p.endswith("/Tomato")]
    rows = []
    for folder in a.runs:
        config = json.loads((folder / "config.json").read_text())
        report = json.loads((folder / "report-000.json").read_text())
        trace = np.load(folder / "trace-000.npz")
        if not np.array_equal(trace["times"], ref["times"]):
            raise ValueError(f"Unmatched sample times: {folder}")
        differences = []
        for i, shift in enumerate(config["branch_shifts_m"]):
            prefix = "/World" if i == 0 else f"/World/Replicas/B{i:03d}"
            indices = [
                config["pose_paths"].index(
                    prefix + cfg["pose_paths"][j][len("/World") :]
                )
                for j in keep
            ]
            delta = trace["poses"][:, indices, :3] - shift - ref["poses"][:, keep, :3]
            differences.append(float(np.linalg.norm(delta, axis=-1).max()))
        rows.append(
            {
                "run": str(folder),
                "branches": config["branches"],
                "leaves": 3 * config["branches"],
                "all_checks_passed": all(report["checks"].values()),
                "performance": report["performance"],
                "branch_leaf_max_position_difference_m": differences,
                "trajectory_within_0_1mm": max(differences) < 0.0001,
            }
        )
    a.output.write_text(
        json.dumps(
            {"reference": str(a.reference), "sample_hz": 10, "rows": rows}, indent=2
        )
    )
    for r in rows:
        print(
            r["branches"],
            r["performance"]["rendered_fps"],
            r["performance"]["work_p95_ms"],
            max(r["branch_leaf_max_position_difference_m"]),
        )


if __name__ == "__main__":
    main()
