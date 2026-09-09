"""Recompute independent tail velocities from saved poses, without simulation."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))
from exporterV2.fruit_diagnostics import multiply, rotate


def recheck(directory):
    report = json.loads((directory / "report.json").read_text())
    if report["status"] == "running":
        raise ValueError(f"incomplete runtime report: {directory}")
    hz = report["runtime_physics_hz"]
    files = sorted(directory.glob("headless-trace-*.npz"))[-3:]
    states, paths = [], None
    for path in files:
        with np.load(path) as data:
            if paths is not None and paths != data["paths"].tolist():
                raise ValueError("trace body order changed")
            paths = data["paths"].tolist()
            states.append(data["state"])
    state = np.concatenate(states)[-(10 * hz + 1):].astype(float)
    effective = json.loads((directory / "headless-effective.json").read_text())
    properties = {r["path"]: r for r in effective["bodies"]}
    com = np.array([properties[p]["com"] for p in paths]).reshape(len(paths), 3)
    q = state[:, :, 3:7]
    q /= np.linalg.norm(q, axis=-1, keepdims=True)
    world_com = state[:, :, :3] + rotate(q, com)
    linear = np.linalg.norm(np.diff(world_com, axis=0), axis=-1) * hz
    inverse = q[:-1].copy()
    inverse[:, :, 1:] *= -1
    delta = multiply(q[1:], inverse)
    angular = 2 * np.arctan2(np.linalg.norm(delta[:, :, 1:], axis=-1), np.abs(delta[:, :, 0])) * hz
    detached = {e.get("fruit") for e in report.get("events", [])}
    for e in report.get("events", []):
        detached.update(r["fruit"] for r in effective["attachments"] if r["joint"] == e["joint"])
    live = np.array([p not in detached for p in paths])
    linear[:, ~live] = angular[:, ~live] = 0
    worst = int(linear.max(axis=0).argmax())
    result = {
        "method": "COM and quaternion differences between adjacent saved physics steps",
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "trace_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        "sample_intervals": len(linear), "hz": hz,
        "kinematic_linear_speed_mps": float(linear.max()),
        "kinematic_angular_speed_radps": float(angular.max()),
        "worst_linear_body": paths[worst],
        "per_body": [{"path": p, "included": bool(live[i]), "linear_mps": float(linear[:, i].max()),
                      "angular_radps": float(angular[:, i].max())} for i, p in enumerate(paths)]}
    (directory / "pose-velocity-check.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(directory.name, result["kinematic_linear_speed_mps"], result["kinematic_angular_speed_radps"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    args = parser.parse_args()
    for directory in args.run_dirs:
        recheck(directory)


if __name__ == "__main__":
    main()
