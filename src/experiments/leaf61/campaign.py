"""Sequential bounded campaign; stops on runtime errors, never hides failed criteria."""

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--real-mesh", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    records = []

    def run(config, name, scenario="cycle", model="surface", extra=()):
        cfg = args.output / (name + ".json")
        cfg.write_text(json.dumps(config, indent=2) + "\n")
        directory = args.output / name
        command = [
            sys.executable,
            str(HERE / "run.py"),
            "--model",
            model,
            "--scenario",
            scenario,
            "--config",
            str(cfg),
            "--run-dir",
            str(directory),
            "--timeout",
            "900",
            *extra,
        ]
        subprocess.run(command, cwd=ROOT, check=False)
        report = json.loads((directory / "report.json").read_text())
        records.append(
            {"name": name, "model": model, "config": config, "report": report}
        )
        (args.output / "summary.json").write_text(json.dumps(records, indent=2) + "\n")
        if not report.get("process_ok"):
            raise RuntimeError(
                f"Runtime failure in {directory}; inspect log before continuing"
            )
        return report

    def eligible(report):
        return (
            report["status"] == "passed"
            and report["timing"]["instrumented_realtime_factor"] >= 1
        )

    # Five physical configurations; bend, solver convergence, then material elasticity.
    configs = [
        {"bend": 5e6},
        {"bend": 5e8},
        {"bend": 5e10},
        {"bend": 5e8, "iterations": 256},
        {"bend": 5e8, "iterations": 256, "young": 5e7},
    ]
    surface_candidate = None
    distinct = 0
    for config in configs:
        distinct += 1
        report = run(config, f"surface-{distinct:02}-rest", "rest")
        if not all(report["checks"].values()):
            continue
        report = run(config, f"surface-{distinct:02}-cycle")
        if eligible(report):
            surface_candidate = config
            break
        if report["status"] == "passed":
            # Physically useful reference, but too slow: don't spend more tuning trials.
            break
        # On first gravity-qualified fixture, vary only damping, keeping mass/thickness fixed.
        for damping in (0.05, 0.1, 0.3):
            distinct += 1
            variant = {**config, "damping": damping, "bend_damping": damping}
            report = run(variant, f"surface-{distinct:02}-cycle")
            if eligible(report):
                surface_candidate = variant
                break
        break
    assert distinct <= 12
    skin_candidate = None
    for i, stiffness in enumerate((0.024, 0.016, 0.048)):
        config = {"joint_stiffness": stiffness}
        report = run(config, f"skinning-{i + 1:02}", model="skinning")
        if eligible(report):
            skin_candidate = config
            break
    candidate = (
        ("surface", surface_candidate)
        if surface_candidate is not None
        else (("skinning", skin_candidate) if skin_candidate is not None else None)
    )
    if candidate is None:
        (args.output / "decision.json").write_text(
            json.dumps({"status": "no_accepted_candidate"}, indent=2) + "\n"
        )
        return 1
    model, config = candidate
    validation = []
    for hz in (60, 240):
        report = run({**config, "hz": hz}, f"{model}-{hz}hz", model=model)
        validation.append({"case": f"{hz}hz", "passed": report["status"] == "passed"})
    if model == "surface":
        report = run({**config, "nx": 60, "ny": 28}, "surface-fine", model=model)
        validation.append({"case": "fine_mesh", "passed": report["status"] == "passed"})
    report = run(
        config,
        f"{model}-real",
        model=model,
        extra=("--shape", "real", "--mesh", str(args.real_mesh.resolve())),
    )
    validation.append({"case": "real_contour", "passed": report["status"] == "passed"})
    report = run(config, f"{model}-render", model=model, extra=("--render",))
    validation.append(
        {"case": "offscreen_render", "passed": report["status"] == "passed"}
    )
    decision = {
        "model": model,
        "config": config,
        "validation": validation,
        "visual_acceptance": "pending",
        "status": "numeric_candidate"
        if all(v["passed"] for v in validation)
        else "validation_failed",
    }
    (args.output / "decision.json").write_text(json.dumps(decision, indent=2) + "\n")
    return 0 if decision["status"] == "numeric_candidate" else 1


if __name__ == "__main__":
    sys.exit(main())
