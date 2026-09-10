"""Prepare or run one versioned ExporterV2 fruit experiment (use project uv)."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--source-usd", type=Path)
    parser.add_argument("--input", type=Path, default=ROOT / "data/plant_states/plant_state_day_160.json")
    parser.add_argument("--scenario", choices=("full", "no-fruit", "single", "truss", "trusses"), default="full")
    parser.add_argument("--fruit", help="Exact source USD fruit path; default highest mass ratio")
    parser.add_argument("--no-collisions", action="store_true")
    parser.add_argument("--unbreakable", action="store_true")
    parser.add_argument("--attachment", choices=("external", "internal"), default="external")
    parser.add_argument("--solver", choices=("TGS", "PGS"), default="TGS")
    parser.add_argument("--cpu", action="store_true")
    parser.add_argument("--hz", type=int, choices=(60, 120, 240, 480, 960), default=480)
    parser.add_argument("--art-position", type=int, choices=range(1, 256), default=32)
    parser.add_argument("--art-velocity", type=int, choices=range(0, 256), default=4)
    parser.add_argument("--fruit-position", type=int, choices=range(1, 256), default=32)
    parser.add_argument("--fruit-velocity", type=int, choices=range(0, 256), default=1)
    parser.add_argument("--stiffness-scale", type=float, choices=(1, .5, .25), default=1)
    damping = parser.add_mutually_exclusive_group()
    damping.add_argument("--damping-scale", type=float, choices=(1, 2, 4, 7),
                         help="Multiplier of the baseline truss damping ratio (4)")
    damping.add_argument("--damping-ratio", type=float, choices=(1, 2, 4, 7),
                         help="Absolute ratio override for reproducing the initial diagnostic cases")
    parser.add_argument("--duration", type=float, default=20)
    parser.add_argument("--acceptance", choices=("strict", "functional"), default="strict",
                        help="Functional keeps numerical tolerances advisory; GUI requires 20 FPS and user review.")
    parser.add_argument("--force-target", choices=("min", "median", "max"))
    parser.add_argument("--force-start", type=float, default=30)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args()
    if args.damping_ratio is None:
        args.damping_scale = args.damping_scale or 1.0
        args.damping_ratio = 4.0 * args.damping_scale
    else:
        args.damping_scale = args.damping_ratio / 4.0
    if args.duration <= 0 or (args.force_target and (args.force_start < 10 or args.duration < args.force_start + 15)):
        parser.error("duration must be positive; force tests need >=10s settling and >=15s after ramp start")
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    if (run_dir / "config.json").exists():
        parser.error("run directory already contains an experiment; choose a fresh directory")
    source = args.source_usd.resolve() if args.source_usd else run_dir / "source.usda"
    input_path = args.input.resolve()
    if not args.source_usd:
        command = [sys.executable, "-m", "exporterV2", "--day", "160", "--input", str(input_path),
                   "--output", str(source), "--debug-profile", "full", "--allow-experimental-fruit-physics",
                   "--physics-preset", "flexible", "--physics-hz", "480", "--leaf-shape-seed", "42", "--generate-only"]
        with (run_dir / "generation.log").open("w") as log:
            subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=True)
    from pxr import Usd
    from exporterV2.fruit_experiments import prepare_stage
    source_manifest = source.with_suffix(".manifest.json")
    manifest = json.loads(source_manifest.read_text())
    metadata, physics = manifest["metadata"], manifest["physics"]["truss_profile"]
    input_day = json.loads(input_path.read_text())["metadata"]["simulation_time"]
    if metadata["day"] != input_day or metadata["day"] != 160 or metadata["debug_profile"] != "full":
        parser.error("source manifest must describe the full day-160 input")
    if any(physics[key] != 4 for key in ("rachis_damping_ratio", "pedicel_damping_ratio")):
        parser.error("source must use the baseline truss damping ratio 4")
    config = {k: (str(v.resolve()) if isinstance(v, Path) else v) for k, v in vars(args).items()}
    config.update(run_dir=str(run_dir), input=str(input_path), input_sha256=sha(input_path),
                  source_usd=str(source), source_sha256=sha(source), collisions=not args.no_collisions,
                  breakable=not args.unbreakable, gpu=not args.cpu,
                  baseline_commit="ac8793ea683998eab8d7318d6e30182535ce5543",
                  current_commit=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip())
    config["source_manifest_sha256"] = sha(source_manifest)
    config["source_metadata"] = {k: v for k, v in metadata.items() if k != "leaf_shapes"}
    config["source_truss_physics"] = physics
    config["implementation_sha256"] = {str(p.relative_to(ROOT)): sha(p) for p in [
        Path(__file__).resolve(), ROOT / "src/exporterV2/fruit_experiments.py",
        ROOT / "src/exporterV2/fruit_diagnostics.py", ROOT / "src/exporterV2/isaac_app.py"]}
    isaac_root = Path(os.environ.get("ISAACSIM_DIR", str(Path.home() / "isaacsim")))
    config["isaac_version"] = (isaac_root / "VERSION").read_text().strip()
    original = Usd.Stage.Open(str(source))
    stage = Usd.Stage.Open(original.Flatten())
    report = prepare_stage(stage, config)
    (run_dir / "audit.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    if report["errors"]:
        raise RuntimeError("USD sanity audit failed: " + "; ".join(report["errors"][:5]))
    output = run_dir / "scene.usda"
    stage.GetRootLayer().Export(str(output))
    config["scene_sha256"] = sha(output)
    (run_dir / "config.json").write_text(json.dumps(config, indent=2, allow_nan=False) + "\n")
    print(f"[PREPARED] {run_dir} bodies={report['body_count']} fruits={len(report['attachments'])}", flush=True)
    if args.prepare_only:
        return 0
    command = [str(isaac_root / "python.sh"), str(ROOT / "src/exporterV2/isaac_app.py"),
               "--usd", str(output), "--physics-preset", "flexible", "--physics-hz", str(args.hz),
               "--interactive-physics-hz", str(args.hz), "--duration", str(args.duration),
               "--fruit-experiment", str(run_dir / "config.json")]
    if not args.gui:
        command.append("--headless")
    with (run_dir / "isaac.log").open("w") as log:
        result = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
    print(f"[FINISHED] exit={result.returncode} log={run_dir / 'isaac.log'}", flush=True)
    return result.returncode


if __name__ == "__main__":
    raise SystemExit(main())
