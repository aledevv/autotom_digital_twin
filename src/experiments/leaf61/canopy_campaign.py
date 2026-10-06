"""First full-plant increment: static, one and five dynamic laminae, sequentially."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gui-benchmark", action="store_true")
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    rows = []
    geometry = None
    for count in (0, 1, 5):
        modes = [("full-headless", "full", [])] + [
            (f"light-render-{i}", "light", ["--render"]) for i in range(3)
        ]
        if args.gui_benchmark and count == 5:
            modes.append(("gui", "light", ["--gui", "--gui-benchmark"]))
        for mode, diagnostics, flags in modes:
            directory = out / f"n{count}-{mode}"
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
                "canopy",
                "--leaves",
                str(count),
                "--scenario",
                "press",
                "--hz",
                "120",
                "--diagnostics",
                diagnostics,
                "--run-dir",
                str(directory),
                *flags,
            ]
            print("RUN", directory.name, flush=True)
            with (out / f"{directory.name}.log").open("w") as log:
                result = subprocess.run(
                    cmd, stdout=log, stderr=subprocess.STDOUT, check=False
                )
            report_path = directory / "report.json"
            report = (
                json.loads(report_path.read_text())
                if report_path.exists()
                else {"status": "failed"}
            )
            row = {
                "count": count,
                "mode": mode,
                "path": str(directory),
                "report": report,
            }
            rows.append(row)
            (out / "comparison.json").write_text(json.dumps(rows, indent=2) + "\n")
            if result.returncode or report["status"] != "passed":
                raise RuntimeError(f"Numerical/runtime gate failed: {directory}")
            runtime = json.loads((directory / "runtime.json").read_text())
            fingerprint = {
                key: runtime["canopy"][key]
                for key in (
                    "source_sha256",
                    "selected",
                    "camera_eye",
                    "camera_target",
                    "all_placements",
                )
            }
            if geometry is None:
                geometry = fingerprint
            elif geometry != fingerprint:
                raise RuntimeError("Fixture or camera changed between counts")
            print(
                "DONE",
                directory.name,
                "RTF",
                round(report["timing"]["realtime_factor"], 3),
                "p95_ms",
                round(report["timing"]["frame_work"]["p95_seconds"] * 1000, 3),
                flush=True,
            )
    summarize(out)


def summarize(out):
    rows = json.loads((out / "comparison.json").read_text())
    text = [
        "# Pianta completa: primo incremento 0 / 1 / 5",
        "",
        "131 lamine visibili; cinque sostituzioni identiche fra tutti i casi. Background statico senza collisioni. CPU/PGS, fisica 120 Hz, rendering 60 Hz richiesto, 1280×720. Numerica full headless; tre ripetizioni light con rendering per numero. Finestra t=9–22 s.",
        "",
        "| Dinamiche | RTF rendering, min–max | p95 peggiore, ms | Budget superato in tutte le ripetizioni |",
        "|---:|---:|---:|---|",
    ]
    for count in (0, 1, 5):
        runs = [
            r["report"]["timing"]
            for r in rows
            if r["count"] == count and r["mode"].startswith("light-render")
        ]
        rtfs = [r["realtime_factor"] for r in runs]
        text.append(
            f"| {count} | {min(rtfs):.3f}–{max(rtfs):.3f} | {max(r['frame_work']['p95_seconds'] for r in runs) * 1000:.3f} | {'sì' if all(r['performance_passed'] for r in runs) else 'no'} |"
        )
    text += [
        "",
        "Le prestazioni includono letture, skinning e diagnostica; i componenti e la memoria sono separati nei report. Il costo GPU è l'occupazione del dispositivo, non esclusiva del processo. Non è una prova con tutte le foglie fisiche, rami dinamici o collisioni con il background. Revisione visiva umana ancora richiesta.",
    ]
    (out / "REPORT.md").write_text("\n".join(text) + "\n")
    print("REPORT", out / "REPORT.md", flush=True)


if __name__ == "__main__":
    main()
