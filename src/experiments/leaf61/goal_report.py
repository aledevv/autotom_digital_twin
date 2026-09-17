"""Summarize measured optimization runs without altering their evidence."""

import argparse
import json
from pathlib import Path


def collect(root):
    rows = []
    for directory in sorted(root.glob("goal-*")):
        report_path = directory / "report.json"
        if not report_path.exists():
            continue
        report = json.loads(report_path.read_text())
        runtime_path = directory / "runtime.json"
        runtime = json.loads(runtime_path.read_text()) if runtime_path.exists() else {}
        timing = report.get("timing", {})
        fps = timing.get("rendered_fps_unpaced")
        p95 = timing.get("frame_work", {}).get("p95_seconds")
        numeric = (
            report.get("status") == "passed"
            and report.get("process_ok", False)
            and report.get("geometry_full_rate", False)
        )
        display = bool(
            numeric
            and not timing.get("paced", True)
            and fps
            and fps > 20
            and p95
            and p95 < 0.05
        )
        rows.append(
            {
                "run": directory.name,
                "leaves": len(report.get("leaves", [])),
                "solver": runtime.get("solver"),
                "iterations": runtime.get("config", {}).get("iterations"),
                "physics_hz": timing.get("physics_hz"),
                "render_hz": timing.get("render_hz"),
                "stress_all": runtime.get("stress_all", False),
                "sleep_threshold": runtime.get("sleep_threshold", 0),
                "skin_writes": report.get("skin_writes", "usd"),
                "fps": fps,
                "p95_ms": p95 * 1000 if p95 else None,
                "realtime_factor": timing.get("realtime_factor"),
                "numerical_pass": numeric,
                "display_20fps_pass": display,
                "display_and_realtime_pass": display
                and timing.get("realtime_factor", 0) >= 1,
                "strict_cadence_pass": display
                and timing.get("realtime_factor", 0) >= 1
                and p95 <= 1 / timing["render_hz"],
                "report": str(report_path.resolve()),
            }
        )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    rows = collect(args.root)
    result = {
        "scope": "Individual executions, not a repeated-run promotion. Display FPS and real time are separate.",
        "runs": rows,
    }
    for name in (
        "display_20fps_pass",
        "display_and_realtime_pass",
        "strict_cadence_pass",
    ):
        result["maximum_single_trial_" + name] = max(
            (r["leaves"] for r in rows if r[name]), default=0
        )
    (args.root / "goal-summary.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({k: v for k, v in result.items() if k != "runs"}, indent=2))


if __name__ == "__main__":
    main()
