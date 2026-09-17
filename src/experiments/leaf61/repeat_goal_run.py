"""Repeat a completed offscreen goal case, requiring exact traces before reusing analysis."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("reference", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--repeats", type=int, default=2)
    args = parser.parse_args()
    reference = args.reference.resolve()
    original = json.loads((reference / "report.json").read_text())
    if (
        original["status"] != "passed"
        or not original.get("process_ok")
        or not original.get("geometry_full_rate")
    ):
        parser.error("Reference must have completed full numerical verification")
    if args.repeats < 1:
        parser.error("Need at least one repeat")
    args.output.mkdir(parents=True, exist_ok=False)
    command = json.loads((reference / "launch.json").read_text())["command"][2:]
    if "--gui" in command:
        parser.error("Reference must be offscreen")
    if "--analysis-reference" in command:
        i = command.index("--analysis-reference")
        del command[i : i + 2]
    command += [
        "--config",
        str(reference / "config.json"),
        "--mesh",
        str(reference / "input_mesh.npz"),
        "--analysis-reference",
        str(reference),
    ]
    if (reference / "input_canopy.usda").exists():
        command += ["--canopy-usd", str(reference / "input_canopy.usda")]
    cases = [(reference, original)]
    for number in range(2, args.repeats + 2):
        destination = args.output.resolve() / f"repeat{number}"
        invocation = list(command)
        invocation[invocation.index("--run-dir") + 1] = str(destination)
        with (args.output / f"repeat{number}.log").open("w") as log:
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "src/experiments/leaf61/run.py"),
                    *invocation,
                ],
                cwd=ROOT,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )
        path = destination / "report.json"
        report = json.loads(path.read_text()) if path.exists() else {"status": "failed"}
        cases.append((destination, report))
        print(
            json.dumps(
                {
                    "run": str(destination),
                    "returncode": result.returncode,
                    "status": report["status"],
                }
            ),
            flush=True,
        )
        if result.returncode:
            break
    rows = []
    for directory, report in cases:
        timing = report.get("timing", {})
        fps, p95 = (
            timing.get("rendered_fps_unpaced", 0),
            timing.get("frame_work", {}).get("p95_seconds", float("inf")),
        )
        numeric = (
            report["status"] == "passed"
            and report.get("process_ok", False)
            and report.get("geometry_full_rate", False)
        )
        display = bool(
            numeric and not timing.get("paced", True) and fps > 20 and p95 < 0.05
        )
        rows.append(
            {
                "run": str(directory),
                "numeric": numeric,
                "fps": fps,
                "p95_ms": p95 * 1000,
                "rtf": timing.get("realtime_factor", 0),
                "display_pass": display,
                "realtime_pass": display and timing.get("realtime_factor", 0) >= 1,
            }
        )
    summary = {
        "cases": rows,
        "all_requested_completed": len(rows) == args.repeats + 1,
        "display_confirmed": len(rows) >= 3 and all(r["display_pass"] for r in rows),
        "realtime_confirmed": len(rows) >= 3 and all(r["realtime_pass"] for r in rows),
    }
    (args.output / "confirmation.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return (
        0
        if summary["all_requested_completed"] and all(r["numeric"] for r in rows)
        else 1
    )


if __name__ == "__main__":
    sys.exit(main())
