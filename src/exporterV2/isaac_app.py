"""Open and exercise an already generated canonical ExporterV2 stage in Isaac Sim."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
from typing import Callable


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--physics-preset", choices=("locked", "flexible"), required=True)
    parser.add_argument("--physics-hz", type=int, choices=(480, 960), default=480)
    parser.add_argument("--report", type=Path)
    args, kit_args = parser.parse_known_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    # Kit also owns --usd; prevent a bootstrap open followed by a second open.
    sys.argv = [sys.argv[0], *kit_args]
    return args


def _open_stage_and_wait(
    context,
    app,
    usd_path: Path,
    is_stage_loading: Callable[[], bool],
) -> Path | None:
    context.open_stage(str(usd_path))
    while is_stage_loading():
        app.update()
    app.update()
    stage = context.get_stage()
    if stage is None or not stage.GetRootLayer().realPath:
        return None
    return Path(stage.GetRootLayer().realPath).resolve()


def _entity_paths(stage) -> tuple[list[str], str]:
    paths = []
    root_path = ""
    for prim in stage.Traverse():
        attribute = prim.GetAttribute("autotom:entityKind")
        kind = attribute.Get() if attribute else None
        if kind in {"physical_link", "terminal_body"}:
            path = str(prim.GetPath())
            paths.append(path)
            if kind == "physical_link" and prim.GetChild("RootFixedJoint"):
                root_path = path
    paths.sort()
    if not paths or not root_path:
        raise RuntimeError("stage has no canonical physical bodies or fixed root")
    return paths, root_path


def _write_report(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _run_stability(stage, world, args: argparse.Namespace) -> dict:
    import numpy as np
    from isaacsim.core.prims import RigidPrim

    paths, root_path = _entity_paths(stage)
    bodies = RigidPrim(
        paths,
        name="canonical_v2_stability_bodies",
        reset_xform_properties=False,
        prepare_contact_sensors=False,
    )
    bodies.initialize()
    resolved_paths = list(bodies.prim_paths)
    if set(resolved_paths) != set(paths):
        raise RuntimeError("batch rigid-body view does not cover the authored bodies")

    def positions_by_path():
        positions, _ = bodies.get_world_poses()
        values = np.asarray(positions, dtype=np.float64).reshape(-1, 3)
        return dict(zip(resolved_paths, values, strict=True))

    initial = positions_by_path()
    latest = dict(initial)
    initial_extent = max(float(np.linalg.norm(value)) for value in initial.values())
    explosion_limit = max(5.0, initial_extent * 10.0 + 1.0)
    dt = 1.0 / args.physics_hz
    steps = max(1, int(math.ceil(args.duration * args.physics_hz)))
    # A batched full-body sample once per simulated second is sufficient for
    # explosion/detachment checks and avoids making Python telemetry dominate
    # mature 200+ link plants.
    sample_interval = max(1, int(round(1.0 * args.physics_hz)))
    samples = []
    errors = []
    max_displacement = 0.0
    max_root_displacement = 0.0

    for step in range(steps):
        world.step(render=False)
        if step % sample_interval and step != steps - 1:
            continue
        current = {}
        for path, value in positions_by_path().items():
            if not np.isfinite(value).all():
                errors.append(f"non-finite body pose: {path}")
                continue
            if float(np.linalg.norm(value)) > explosion_limit:
                errors.append(f"body exceeded explosion limit: {path}")
            current[path] = value
            max_displacement = max(
                max_displacement, float(np.linalg.norm(value - initial[path]))
            )
        if root_path in current:
            max_root_displacement = max(
                max_root_displacement,
                float(np.linalg.norm(current[root_path] - initial[root_path])),
            )
        if len(current) != len(paths):
            errors.append("one or more rigid bodies became unreadable")
        elapsed = min((step + 1) * dt, args.duration)
        if samples:
            sample_dt = max(elapsed - samples[-1][0], dt)
            max_speed = max(
                float(np.linalg.norm(current[path] - latest[path]) / sample_dt)
                for path in current.keys() & latest.keys()
            )
        else:
            max_speed = 0.0
        samples.append((elapsed, max_speed))
        latest = current
        print(
            f"[SIM] {elapsed:.1f}/{args.duration:.1f}s "
            f"bodies={len(current)} max_displacement={max_displacement:.6g}m",
            flush=True,
        )
        if errors:
            break

    tail_start = args.duration * 0.8
    tail_speeds = [speed for time, speed in samples if time >= tail_start]
    tail_max_speed = max(tail_speeds, default=0.0)
    root_drift_limit = 1e-3
    locked_projection_limit = 1e-2
    if max_root_displacement > root_drift_limit:
        errors.append(
            f"fixed root drifted by {max_root_displacement:.6g} m"
        )
    if args.physics_preset == "locked" and max_displacement > locked_projection_limit:
        errors.append(
            f"locked structure moved by {max_displacement:.6g} m"
        )
    if args.duration >= 30.0 and tail_max_speed > 0.05:
        errors.append(
            f"persistent oscillation: tail speed {tail_max_speed:.6g} m/s"
        )
    return {
        "schema_version": "exporter_v2_stability/1.0",
        "status": "passed" if not errors else "failed",
        "usd": str(args.usd),
        "physics_preset": args.physics_preset,
        "physics_hz": args.physics_hz,
        "duration_seconds": args.duration,
        "steps_requested": steps,
        "samples": len(samples),
        "rigid_body_count": len(paths),
        "max_displacement_m": max_displacement,
        "max_root_displacement_m": max_root_displacement,
        "tail_max_speed_mps": tail_max_speed,
        "explosion_limit_m": explosion_limit,
        "root_drift_limit_m": root_drift_limit,
        "locked_initial_projection_limit_m": locked_projection_limit,
        "errors": errors,
    }


def main() -> int:
    args = _arguments()
    usd_path = args.usd.expanduser().resolve()
    args.usd = usd_path
    if not usd_path.is_file():
        print(f"[ERROR] USD stage does not exist: {usd_path}", file=sys.stderr)
        return 2

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import is_stage_loading

        context = omni.usd.get_context()
        opened_path = _open_stage_and_wait(context, app, usd_path, is_stage_loading)
        if opened_path != usd_path:
            raise RuntimeError(f"opened {opened_path!s} instead of {usd_path}")
        stage = context.get_stage()
        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / args.physics_hz,
            rendering_dt=1.0 / 60.0,
        )
        world.reset()
        print(f"[OK] Isaac Sim opened canonical V2 stage: {usd_path}", flush=True)
        if args.headless:
            report = _run_stability(stage, world, args)
            report_path = (
                args.report.expanduser().resolve()
                if args.report is not None
                else usd_path.with_suffix(".stability.json")
            )
            _write_report(report_path, report)
            print(
                f"[{'OK' if report['status'] == 'passed' else 'ERROR'}] "
                f"stability={report['status']} report={report_path}",
                flush=True,
            )
            return 0 if report["status"] == "passed" else 1
        while app.app.is_running() and not app.is_exiting():
            world.step(render=True)
        return 0
    except KeyboardInterrupt:
        print("[INFO] Isaac Sim interrupted by user.", flush=True)
        return 0
    except Exception as exc:
        import traceback

        print(f"[ERROR] Isaac Sim V2 loader failed: {exc}", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        app.close()


if __name__ == "__main__":
    exit_code = main()
    # Kit can normalize SystemExit to zero during shutdown.  A headless
    # stability failure must remain observable by shell/CI callers.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(exit_code)
