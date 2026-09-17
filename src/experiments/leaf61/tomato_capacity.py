"""Sequential full-diagnostic capacity sweep; no concurrent Isaac processes."""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def assess(report):
    t = report["timing"]
    return (
        report["status"] == "passed"
        and report.get("process_ok", False)
        and t["rendered_fps_unpaced"] > 20
        and t["frame_work"]["p95_seconds"] < 0.05
    )


def command(count, output, render_hz):
    return [
        sys.executable,
        str(ROOT / "src/experiments/leaf61/run.py"),
        "--model",
        "skinning",
        "--shape",
        "real",
        "--mesh",
        str(ROOT / "artifacts/leaf61/real-seed42.npz"),
        "--canopy-usd",
        str(
            ROOT / "artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda"
        ),
        "--layout",
        "canopy-drop",
        "--drop-test",
        "scale",
        "--leaves",
        str(count),
        "--scenario",
        "press",
        "--hz",
        "480",
        "--ball-mass",
        "0.02",
        "--ball-radius",
        "0.02",
        "--drop-height",
        "0.06",
        "--render",
        "--render-hz",
        str(render_hz),
        "--drop-runtime",
        "optimized",
        "--timeout",
        "1800",
        "--run-dir",
        str(output),
    ]


def write_summary(directory):
    rows = json.loads((directory / "results.json").read_text())
    values = []
    for row in rows:
        t = row["report"]["timing"]
        frames = t["frame_work"]["count"]
        entry = {
            "run": Path(row["run"]).name,
            "leaves": row["count"],
            "fps": t["rendered_fps_unpaced"],
            "realtime_factor": t["realtime_factor"],
            "p95_ms": t["frame_work"]["p95_seconds"] * 1000,
            "impact_p95_ms": t["first_second_after_release"]["p95_seconds"] * 1000,
            "numerical_passed": row["report"]["status"] == "passed",
            "passed_20fps": row["passed_20fps"],
        }
        for key, component in t["components"].items():
            entry[key + "_mean_ms_per_frame"] = (
                component["total_seconds"] / frames * 1000
            )
        values.append(entry)
    with (directory / "costs.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(values[0]))
        writer.writeheader()
        writer.writerows(values)
    lines = [
        "# Capacità: foglie indipendenti nella pianta",
        "",
        f"Fisica 480 Hz, rendering {rows[0]['report']['timing']['render_hz']} Hz di tempo simulato, 1280×720. FPS senza pacing; RTF separato. Una sola foglia colpita. Tutte le pose e i controlli geometrici completi conservati.",
        "",
        "| Prova | Foglie | FPS | RTF | p95 ms | p95 urto ms | Sopra 20 FPS |",
        "|---|---:|---:|---:|---:|---:|---|",
    ]
    for row in values:
        lines.append(
            f"| {row['run']} | {row['leaves']} | {row['fps']:.2f} | {row['realtime_factor']:.3f} | {row['p95_ms']:.2f} | {row['impact_p95_ms']:.2f} | {row['passed_20fps']} |"
        )
    lines += [
        "",
        "La soglia FPS usa media >20 e p95 <50 ms, oltre ai controlli fisici. Non garantisce ogni singolo frame. `target_20fps_passed` nei report di questa campagna include anche RTF ≥1; la decisione FPS separata è in results.json. GUI e collisioni simultanee su molte foglie non sono convalidate da questa misura.",
    ]
    (directory / "REPORT.md").write_text("\n".join(lines) + "\n")


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--counts", type=int, nargs="+", default=[1, 5, 10, 20, 40, 80, 120])
    p.add_argument("--render-hz", type=int, choices=[30, 60], default=30)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    rows = []
    candidate = None

    def run(count, label):
        path = a.output / label
        print("RUN", label, flush=True)
        with (a.output / f"{label}.log").open("w") as log:
            result = subprocess.run(
                command(count, path, a.render_hz),
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )
        report = json.loads((path / "report.json").read_text())
        passed = result.returncode == 0 and assess(report)
        rows.append(
            {
                "count": count,
                "run": str(path),
                "passed_20fps": passed,
                "report": report,
            }
        )
        (a.output / "results.json").write_text(json.dumps(rows, indent=2) + "\n")
        print(
            "DONE",
            label,
            "passed",
            passed,
            "RTF",
            report["timing"]["realtime_factor"],
            "FPS",
            report["timing"]["rendered_fps_unpaced"],
            flush=True,
        )
        return passed

    for count in a.counts:
        if not run(count, f"n{count}-1"):
            break
        candidate = count
    confirmed = candidate is not None
    if candidate is not None:
        for repeat in (2, 3):
            confirmed = run(candidate, f"n{candidate}-{repeat}") and confirmed
    (a.output / "decision.json").write_text(
        json.dumps(
            {
                "largest_candidate": candidate,
                "confirmed_three_runs": confirmed,
                "scope": "480 Hz physics, independent leaf collisions filtered; 20 FPS floor; realtime factor reported separately; GUI review separate",
            },
            indent=2,
        )
        + "\n"
    )

    write_summary(a.output)


if __name__ == "__main__":
    main()
