"""Paired baseline/frame-synchronized USD measurements with exact physical-trajectory regression."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def regression(reference, candidate):
    result = {}
    with (
        np.load(reference / "trace.npz") as old,
        np.load(candidate / "trace.npz") as new,
    ):
        for key in ("times", "rigid_poses", "ball", "ball_velocities", "contacts"):
            a, b = old[key], new[key]
            identical = bool(a.shape == b.shape and np.array_equal(a, b))
            result[key] = {
                "identical": identical,
                "max_abs_difference": float(np.max(np.abs(a - b)))
                if a.shape == b.shape
                else None,
            }
    result["passed"] = all(v["identical"] for v in result.values())
    return result


def summarize(out):
    rows = json.loads((out / "comparison.json").read_text())
    text = [
        "# Pomodoro: ottimizzazione senza modificare la fisica",
        "",
        "480 Hz, 20 g, raggio 20 mm, altezza 60 mm. Stessa scena, camera, diagnostica, rendering 60 Hz e finestra t=9–17 s. Tre confronti accoppiati baseline/optimized, in sequenza.",
        "",
        "| Percorso | RTF min–max | p95 peggiore per frame | p95 peggiore nel primo secondo |",
        "|---|---:|---:|---:|",
    ]
    for mode in ("baseline", "optimized"):
        runs = [r["report"]["timing"] for r in rows if r["mode"] == mode]
        text.append(
            f"| {mode} | {min(r['realtime_factor'] for r in runs):.3f}–{max(r['realtime_factor'] for r in runs):.3f} | {max(r['frame_work']['p95_seconds'] for r in runs) * 1000:.3f} ms | {max(r['first_second_after_release']['p95_seconds'] for r in runs) * 1000:.3f} ms |"
        )
    valid = [r for r in rows if r["mode"] == "optimized"]
    decision = {
        "repetitions": len(valid),
        "all_regressions_identical": all(r["regression"]["passed"] for r in rows),
        "all_numerical_passed": all(r["report"]["status"] == "passed" for r in rows),
        "optimized_performance_passed": len(valid) == 3
        and all(
            r["report"]["timing"]["performance_passed"]
            and r["report"]["timing"]["first_second_after_release"]["p95_seconds"]
            <= 1 / 60
            for r in valid
        ),
        "gui_review": "pending",
        "scope": "One dynamic leaf; no extrapolation to multiple leaves",
    }
    (out / "decision.json").write_text(json.dumps(decision, indent=2) + "\n")
    text += [
        "",
        "Pose, velocità e contatti devono coincidere esattamente con il riferimento approvato. La posizione renderizzata del pomodoro viene controllata durante il primo secondo. Il p95 per frame comprende tutto il lavoro di otto passi fisici e un rendering; la revisione GUI resta separata.",
    ]
    (out / "REPORT.md").write_text("\n".join(text) + "\n")
    return decision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--reference",
        type=Path,
        default=ROOT / "artifacts/leaf61/tomato-drop-20g-480-render",
    )
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    rows = []
    expected_fixture = None
    for repeat in range(1, 4):
        for mode in ("baseline", "optimized"):
            directory = out / f"{mode}-{repeat}"
            cmd = [
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
                    ROOT
                    / "artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda"
                ),
                "--layout",
                "canopy-drop",
                "--leaves",
                "1",
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
                "--drop-runtime",
                mode,
                "--run-dir",
                str(directory),
            ]
            print("RUN", directory.name, flush=True)
            with (out / (directory.name + ".log")).open("w") as log:
                proc = subprocess.run(
                    cmd, stdout=log, stderr=subprocess.STDOUT, check=False
                )
            report = json.loads((directory / "report.json").read_text())
            match = (
                regression(args.reference, directory)
                if (directory / "trace.npz").exists()
                else {"passed": False}
            )
            row = {
                "mode": mode,
                "repeat": repeat,
                "report": report,
                "regression": match,
            }
            rows.append(row)
            (out / "comparison.json").write_text(json.dumps(rows, indent=2) + "\n")
            if proc.returncode or report["status"] != "passed" or not match["passed"]:
                raise RuntimeError(
                    f"Numerical, rendering or trajectory gate failed: {directory}"
                )
            runtime = json.loads((directory / "runtime.json").read_text())
            fixture = {
                key: runtime[key]
                for key in (
                    "config",
                    "ball_spawn_m",
                    "canopy",
                    "leaf_offsets_m",
                    "leaf_yaws_rad",
                    "friction",
                )
            }
            if expected_fixture is None:
                expected_fixture = fixture
            elif fixture != expected_fixture:
                raise RuntimeError("Physical fixture changed between runs")
            print(
                "DONE",
                directory.name,
                "RTF",
                round(report["timing"]["realtime_factor"], 3),
                "p95_ms",
                round(report["timing"]["frame_work"]["p95_seconds"] * 1000, 3),
                flush=True,
            )
    print(json.dumps(summarize(out), indent=2), flush=True)


if __name__ == "__main__":
    main()
