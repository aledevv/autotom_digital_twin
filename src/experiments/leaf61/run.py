"""System-Python supervisor for the isolated Isaac Sim 6.1 bench."""

import argparse
import hashlib
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
ISAAC = Path.home() / "isaacsim-6.1/python.sh"


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", choices=["surface", "skinning"], default="surface")
    p.add_argument(
        "--scenario", choices=["smoke", "rest", "press", "cycle"], default="cycle"
    )
    p.add_argument("--shape", choices=["rectangle", "real"], default="rectangle")
    p.add_argument("--mesh", type=Path, help="NPZ with points/faces for --shape real")
    p.add_argument("--hz", type=int, choices=[60, 120, 240, 480, 960])
    p.add_argument("--config", type=Path)
    p.add_argument("--run-dir", type=Path)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--render", action="store_true")
    p.add_argument("--timeout", type=float, default=600)
    p.add_argument("--gui-benchmark", action="store_true")
    p.add_argument("--leaves", type=int)
    p.add_argument(
        "--layout",
        choices=["plant", "contact-pair", "drop-stack", "canopy", "canopy-drop"],
        default="plant",
    )
    p.add_argument("--diagnostics", choices=["full", "light"], default="full")
    p.add_argument("--pair-collisions", choices=["on", "off"], default="on")
    p.add_argument(
        "--ball-mass", type=float, default=0.005, help="Dynamic ball mass in kg"
    )
    p.add_argument(
        "--drop-height",
        type=float,
        default=0.06,
        help="Initial ball-surface clearance in metres",
    )
    p.add_argument("--leaf-gap", type=float, default=0.06)
    p.add_argument("--lower-offset", type=float, default=0.075)
    p.add_argument("--lower-y", type=float, default=0.0)
    p.add_argument("--ball-x", type=float, default=0.055)
    p.add_argument("--playback-speed", type=float, default=1.0)
    p.add_argument(
        "--drop-runtime",
        choices=["baseline", "optimized", "batched", "visual"],
        default="baseline",
    )
    p.add_argument("--ball-radius", type=float, help="Dynamic sphere radius in metres")
    p.add_argument("--canopy-usd", type=Path)
    p.add_argument(
        "--drop-test", choices=["single", "scale", "cascade"], default="single"
    )
    p.add_argument("--render-hz", type=int, choices=[20, 30, 60], default=60)
    p.add_argument("--cascade-layout", type=Path)
    p.add_argument("--analysis-reference", type=Path)
    p.add_argument("--geometry-method", choices=["exact", "bounded"], default="exact")
    p.add_argument("--plant-copies", type=int, default=1)
    p.add_argument("--collision-filter", choices=["pairs", "groups"], default="pairs")
    p.add_argument("--visual-tolerance", type=float, default=0.0)
    p.add_argument("--skin-batch-size", type=int, default=0)
    p.add_argument("--sleep-threshold", type=float, default=0.0)
    p.add_argument("--stress-all", action="store_true")
    p.add_argument("--rain", action="store_true")
    p.add_argument("--storm", action="store_true")
    p.add_argument("--storm-rate", type=float, default=200.0)
    p.add_argument("--storm-gain", type=float, default=1.0)
    p.add_argument("--storm-load-hz", type=int, choices=[60, 120, 480], default=120)
    p.add_argument("--rain-count", type=int, default=24)
    p.add_argument("--rain-seed", type=int, default=42)
    p.add_argument("--rain-waves", default="")
    p.add_argument(
        "--canopy-selection", choices=["nearby", "primary"], default="nearby"
    )
    p.add_argument("--skin-writes", choices=["usd", "sdf"], default="usd")
    p.add_argument("--solver", choices=["PGS", "TGS"], default="PGS")
    p.add_argument("--recording", choices=["full", "first-trial"], default="full")
    p.add_argument("--tail-seconds", type=float, default=0.0)
    p.add_argument("--physics-threads", type=int, choices=range(33), default=0)
    a = p.parse_args()
    if a.storm:
        if (
            a.rain
            or a.stress_all
            or a.layout != "canopy-drop"
            or a.drop_runtime != "visual"
            or a.plant_copies != 1
        ):
            p.error(
                "Storm requires a single canopy with visual runtime, without tomato rain or stress-all"
            )
        if (
            not math.isfinite(a.storm_rate)
            or not 0 <= a.storm_rate <= 1000
            or not math.isfinite(a.storm_gain)
            or not 1 <= a.storm_gain <= 20
        ):
            p.error("Storm rate must be 0..1000 mm/h and gain 1..20")
    if a.rain_waves:
        try:
            wave_sizes = [int(v) for v in a.rain_waves.split(",")]
        except ValueError:
            p.error("Rain waves must be comma-separated integers")
        if (
            not a.rain
            or not wave_sizes
            or any(v < 1 or v > 12 for v in wave_sizes)
            or sum(wave_sizes) > 100
        ):
            p.error(
                "Rain waves require --rain, 1..12 per wave and at most 100 tomatoes"
            )
        a.rain_count = sum(wave_sizes)
    if (
        not math.isfinite(a.tail_seconds)
        or a.tail_seconds < 0
        or (a.tail_seconds and a.recording != "first-trial")
    ):
        p.error("A nonnegative tail requires first-trial recording")
    if a.rain and (
        a.layout != "canopy-drop"
        or a.drop_runtime != "visual"
        or a.collision_filter != "groups"
        or a.stress_all
        or a.plant_copies != 1
        or not 1 <= a.rain_count <= 100
    ):
        p.error(
            "Rain requires one canopy, visual runtime, collision groups, 1..100 tomatoes and no stress-all"
        )
    if a.canopy_selection != "nearby" and (
        a.layout != "canopy-drop" or a.drop_test != "scale"
    ):
        p.error("Primary selection requires canopy scale mode")
    if a.skin_writes == "sdf" and (a.drop_runtime != "visual" or a.skin_batch_size):
        p.error("Sdf skin writes require visual runtime without merged skins")
    if a.solver != "PGS" and a.layout not in ("drop-stack", "canopy-drop"):
        p.error("Solver selection is supported by drop layouts only")
    if a.stress_all and a.drop_test != "scale":
        p.error("--stress-all requires scale mode")
    if not math.isfinite(a.sleep_threshold) or not 0 <= a.sleep_threshold <= 0.001:
        p.error("Sleep threshold must be finite and in [0, 0.001]")
    if a.skin_batch_size < 0 or (a.skin_batch_size and a.drop_runtime != "visual"):
        p.error("Skin batches require visual runtime and nonnegative size")
    if not math.isfinite(a.visual_tolerance) or not 0 <= a.visual_tolerance <= 1e-5:
        p.error("Visual tolerance must be finite and between 0 and 10 micrometres")
    if a.plant_copies < 1 or (a.plant_copies != 1 and a.drop_test != "scale"):
        p.error("Plant copies require scale mode and a positive count")
    if a.leaves is not None and a.leaves < 0:
        p.error("--leaves must be nonnegative")
    if a.render_hz != 60 and a.layout not in ("drop-stack", "canopy-drop"):
        p.error("--render-hz is supported by drop layouts only")
    if a.layout == "canopy-drop":
        if not a.leaves or a.leaves < 1 or not a.canopy_usd:
            p.error("Tomato drop requires positive --leaves and --canopy-usd")
    elif a.layout == "canopy":
        if a.leaves not in (0, 1, 5) or not a.canopy_usd:
            p.error("Canopy stage 1 requires --leaves 0|1|5 and --canopy-usd")
    elif a.leaves == 0 or a.canopy_usd is not None:
        p.error("--leaves 0 and --canopy-usd require --layout canopy")
    if a.drop_test == "scale" and a.layout != "canopy-drop":
        p.error("Scale requires canopy-drop")
    if a.drop_test == "single" and a.layout == "canopy-drop" and a.leaves != 1:
        p.error("Use --drop-test scale for multiple canopy leaves")
    if a.drop_test == "cascade" and (
        a.layout != "drop-stack" or not a.leaves or not 2 <= a.leaves <= 8
    ):
        p.error("Cascade requires drop-stack with 2..8 leaves")
    if a.layout == "plant" and a.leaves not in (None, 1, 5, 10, 20):
        p.error("Plant pressure fixture supports 1/5/10/20 leaves")
    if a.cascade_layout and a.drop_test != "cascade":
        p.error("--cascade-layout requires --drop-test cascade")
    if a.gui_benchmark and (not a.gui or a.leaves is None):
        p.error("--gui-benchmark requires --gui and --leaves")
    if a.layout in ("contact-pair", "drop-stack") and a.leaves is None:
        p.error(
            "--layout contact-pair requires --leaves (the pair always contains two leaves)"
        )
    if a.layout not in ("drop-stack", "canopy-drop") and a.hz in (480, 960):
        p.error("480/960 Hz are reserved for drop layouts")
    if a.leaves is not None and (a.model != "skinning" or a.shape != "real"):
        p.error("Multi-leaf bench requires --model skinning --shape real")
    if a.leaves is not None and a.scenario not in ("press", "cycle"):
        p.error("Multi-leaf smoke supports --scenario press|cycle")
    if (
        a.leaves is not None
        and a.layout not in ("drop-stack", "canopy-drop")
        and a.hz not in (None, 120)
    ):
        p.error("Multi-leaf smoke is fixed at 120 Hz")
    if not math.isfinite(a.timeout) or a.timeout <= 0:
        p.error("--timeout must be finite and positive")
    if a.shape == "real" and not a.mesh:
        p.error("--shape real requires --mesh")
    out = (
        a.run_dir
        or ROOT
        / "artifacts/leaf61"
        / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    ).resolve()
    out.mkdir(parents=True, exist_ok=False)
    config = json.loads(a.config.read_text()) if a.config else {}
    if a.hz:
        config["hz"] = a.hz
    if a.ball_radius is not None:
        if not math.isfinite(a.ball_radius) or a.ball_radius <= 0:
            p.error("--ball-radius must be finite and positive")
        config["radius"] = a.ball_radius
    (out / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    if a.canopy_usd:
        shutil.copyfile(a.canopy_usd, out / "input_canopy.usda")
    if a.mesh:
        shutil.copyfile(a.mesh, out / "input_mesh.npz")
        if a.mesh.with_suffix(".json").exists():
            shutil.copyfile(a.mesh.with_suffix(".json"), out / "input_mesh.json")
    if a.cascade_layout:
        shutil.copyfile(a.cascade_layout, out / "input_layout.json")
    worker = Path(__file__).with_name("scene.py")
    cmd = [
        str(ISAAC),
        str(worker),
        "--run-dir",
        str(out),
        "--model",
        a.model,
        "--scenario",
        a.scenario,
        "--shape",
        a.shape,
    ]
    if a.leaves is not None:
        cmd += [
            "--leaves",
            str(a.leaves),
            "--layout",
            a.layout,
            "--diagnostics",
            a.diagnostics,
            "--pair-collisions",
            a.pair_collisions,
        ]
    if a.layout in ("drop-stack", "canopy-drop"):
        cmd += [
            "--drop-runtime",
            a.drop_runtime,
            "--drop-test",
            a.drop_test,
            "--render-hz",
            str(a.render_hz),
        ]
        if a.cascade_layout:
            cmd += ["--cascade-layout", str(out / "input_layout.json")]
        for name in (
            "ball_mass",
            "drop_height",
            "leaf_gap",
            "lower_offset",
            "lower_y",
            "ball_x",
            "playback_speed",
        ):
            value = getattr(a, name)
            if not math.isfinite(value):
                p.error(f"Non-finite {name}")
            cmd += ["--" + name.replace("_", "-"), str(value)]
    cmd += [
        "--geometry-method",
        a.geometry_method,
        "--plant-copies",
        str(a.plant_copies),
        "--collision-filter",
        a.collision_filter,
    ]
    if a.analysis_reference:
        cmd += ["--analysis-reference", str(a.analysis_reference.resolve())]
    cmd += [
        "--visual-tolerance",
        str(a.visual_tolerance),
        "--skin-batch-size",
        str(a.skin_batch_size),
    ]
    cmd += [
        "--sleep-threshold",
        str(a.sleep_threshold),
        "--physics-threads",
        str(a.physics_threads),
    ]
    cmd += [
        "--recording",
        a.recording,
        "--tail-seconds",
        str(a.tail_seconds),
        "--solver",
        a.solver,
    ]
    cmd += ["--skin-writes", a.skin_writes, "--canopy-selection", a.canopy_selection]
    cmd += [
        "--rain-count",
        str(a.rain_count),
        "--rain-seed",
        str(a.rain_seed),
        "--rain-waves",
        a.rain_waves,
    ]
    cmd += [
        "--storm-rate",
        str(a.storm_rate),
        "--storm-gain",
        str(a.storm_gain),
        "--storm-load-hz",
        str(a.storm_load_hz),
    ]
    for flag in ("gui", "render", "gui_benchmark", "stress_all", "rain", "storm"):
        if getattr(a, flag):
            cmd.append("--" + flag.replace("_", "-"))
    env = dict(os.environ)
    for name in (
        "PYTHONHOME",
        "PYTHONEXE",
        "VIRTUAL_ENV",
        "CONDA_PREFIX",
        "CONDA_DEFAULT_ENV",
    ):
        env.pop(name, None)
    (out / "sources").mkdir()
    for source in worker.parent.glob("*.py"):
        shutil.copyfile(source, out / "sources" / source.name)
    cmd[1] = str(out / "sources" / "scene.py")
    provenance = {
        str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
        for f in worker.parent.glob("*.py")
    }
    (out / "launch.json").write_text(
        json.dumps(
            {
                "command": cmd,
                "cwd": str(ROOT),
                "source_sha256": provenance,
                "canopy_source": str(a.canopy_usd.resolve()) if a.canopy_usd else None,
                "input_canopy_sha256": hashlib.sha256(
                    a.canopy_usd.read_bytes()
                ).hexdigest()
                if a.canopy_usd
                else None,
                "input_mesh_sha256": hashlib.sha256(a.mesh.read_bytes()).hexdigest()
                if a.mesh
                else None,
            },
            indent=2,
        )
        + "\n"
    )
    print("RUN_DIR=" + str(out), flush=True)
    start = time.monotonic()
    with (out / "isaac.log").open("w") as log:
        proc = subprocess.Popen(
            cmd,
            cwd=ROOT,
            env=env,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            code = proc.wait(
                timeout=None if a.gui and not a.gui_benchmark else a.timeout
            )
        except (KeyboardInterrupt, subprocess.TimeoutExpired) as exc:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            code = 130 if isinstance(exc, KeyboardInterrupt) else 124
    errors = [
        s
        for s in (out / "isaac.log").read_text(errors="replace").splitlines()
        if "[Error]" in s
        and any(x in s.lower() for x in ("physx", "physics.tensors", "cuda"))
    ]
    process = {
        "returncode": code,
        "physics_errors": errors,
        "wall_seconds": time.monotonic() - start,
    }
    (out / "process.json").write_text(json.dumps(process, indent=2) + "\n")
    rp = out / "report.json"
    report = (
        json.loads(rp.read_text())
        if rp.exists()
        else {"status": "failed", "reason": "Missing worker report"}
    )
    report["process_ok"] = code == 0 and not errors
    if not report["process_ok"]:
        report["status"] = "failed"
    rp.write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            {
                "report": str(rp),
                "status": report["status"],
                "process_ok": report.get("process_ok"),
                "checks": report.get("checks"),
                "timing": report.get("timing"),
                "leaf_count": len(report.get("leaves", [])),
                "reason": report.get("reason"),
            },
            indent=2,
        )
    )
    return 0 if report["status"] in ("passed", "diagnostic", "manual") else 1


if __name__ == "__main__":
    sys.exit(main())
