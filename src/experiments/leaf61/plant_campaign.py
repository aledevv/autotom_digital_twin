"""Bounded sequential campaign, system Python. No concurrent Isaac processes."""

import argparse
import ast
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RUN = ROOT / "src/experiments/leaf61/run.py"


def read(path):
    return json.loads(path.read_text())


def physics_fingerprint(directory):
    """Only reuse headless runs when authoring, numerics and stepping are identical.

    Camera configuration and post-run reporting are intentionally excluded.
    """
    trees = {
        name: ast.parse((directory / name).read_text())
        for name in ("model.py", "plant_model.py", "scene.py", "plant_scene.py")
    }
    build_node = next(
        n
        for n in trees["scene.py"].body
        if isinstance(n, ast.FunctionDef) and n.name == "build"
    )
    run_node = next(
        n
        for n in trees["plant_scene.py"].body
        if isinstance(n, ast.FunctionDef) and n.name == "run"
    )
    authoring = []
    for node in run_node.body:
        if (
            isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Call)
            and isinstance(node.value.func, ast.Name)
            and node.value.func.id == "set_camera_view"
        ):
            break
        authoring.append(node)
    reads = [
        n
        for n in run_node.body
        if isinstance(n, ast.FunctionDef) and n.name in ("read", "points")
    ]
    stepping = [n for n in run_node.body if isinstance(n, ast.While)]
    protocol = [
        n
        for n in run_node.body
        if isinstance(n, ast.Assign)
        and any(
            isinstance(t, ast.Name)
            and t.id
            in (
                "total_cycles",
                "begin",
                "limit",
                "active",
                "cycles",
                "number",
                "manual",
                "release",
                "parking",
                "state",
            )
            for t in n.targets
        )
    ]
    nodes = [
        trees["model.py"],
        trees["plant_model.py"],
        build_node,
        *authoring,
        *reads,
        *protocol,
        *stepping,
    ]
    return hashlib.sha256(
        "\n".join(ast.dump(n, include_attributes=False) for n in nodes).encode()
    ).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--baseline",
        type=Path,
        default=ROOT / "artifacts/leaf61/skinning-real-final/trace.npz",
    )
    parser.add_argument("--regression-run", type=Path)
    parser.add_argument("--gui-benchmark", action="store_true")
    parser.add_argument(
        "--reuse-headless-from",
        type=Path,
        help="Reuse successful headless evidence only if physical source AST, config and mesh match",
    )
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)

    reused = {}

    def run(name, count, diagnostics="light", render=False, scenario="press", extra=()):
        folder = out / name
        if (
            args.reuse_headless_from
            and not render
            and not extra
            and scenario == "press"
        ):
            cached = args.reuse_headless_from.resolve() / name
            if (cached / "report.json").exists():
                old = read(cached / "report.json")
                if old.get("process_ok") and old.get("status") == "passed":
                    from dataclasses import asdict

                    from model import Config

                    same = (
                        physics_fingerprint(cached / "sources")
                        == physics_fingerprint(RUN.parent)
                        and read(cached / "config.json") == asdict(Config())
                        and old.get("count") == count
                        and old.get("diagnostics") == diagnostics
                        and old["timing"]["rendered_frames"] == 0
                        and hashlib.sha256(
                            (cached / "input_mesh.npz").read_bytes()
                        ).digest()
                        == hashlib.sha256(
                            (ROOT / "artifacts/leaf61/real-seed42.npz").read_bytes()
                        ).digest()
                    )
                    if same:
                        reused[name] = {
                            "run": str(cached),
                            "physics_fingerprint": physics_fingerprint(
                                cached / "sources"
                            ),
                        }
                        (out / "reused_headless.json").write_text(
                            json.dumps(reused, indent=2) + "\n"
                        )
                        print(name, "reused validated headless evidence", flush=True)
                        return old, cached
        cmd = [
            sys.executable,
            str(RUN),
            "--model",
            "skinning",
            "--shape",
            "real",
            "--mesh",
            str(ROOT / "artifacts/leaf61/real-seed42.npz"),
            "--leaves",
            str(count),
            "--diagnostics",
            diagnostics,
            "--scenario",
            scenario,
            "--run-dir",
            str(folder),
            "--timeout",
            "900",
        ]
        if render:
            cmd.append("--render")
        cmd += list(extra)
        with (out / (name + ".supervisor.log")).open("w") as stream:
            subprocess.run(
                cmd, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, check=False
            )
        report = read(folder / "report.json")
        if not report.get("process_ok"):
            raise RuntimeError(
                f"Runtime failure in {folder}; inspect log before continuing"
            )
        print(
            name,
            report["status"],
            report.get("timing", {}).get("realtime_factor"),
            flush=True,
        )
        return report, folder

    regression = args.regression_run
    if regression is None:
        _, regression = run("regression", 1, "full")
    regreport = read(regression / "report.json")
    # NumPy is only needed for small regression/control comparisons; use known system environment.
    import numpy as np

    old = np.load(args.baseline)
    new = np.load(regression / "regression_trace.npz")
    same_time = old["times"].shape == new["times"].shape and np.allclose(
        old["times"], new["times"], atol=1e-8
    )
    difference = (
        float(np.max(np.linalg.norm(old["points"] - new["points"], axis=-1)))
        if same_time
        else None
    )
    gate = {
        "run": str(regression),
        "baseline": str(args.baseline),
        "same_times": same_time,
        "max_difference_m": difference,
        "passed": bool(
            regreport["status"] == "passed"
            and regreport.get("process_ok")
            and same_time
            and difference < 0.0001
        ),
    }
    (out / "regression.json").write_text(json.dumps(gate, indent=2) + "\n")
    if not gate["passed"]:
        raise RuntimeError("Single-leaf regression failed; campaign blocked")
    rows = []
    for count in (1, 5, 10, 20):
        full, folder = run(f"n{count:02d}-full", count, "full")
        reps = []
        for render in (False, True):
            for rep in range(3):
                result, runpath = run(
                    f"n{count:02d}-{'render' if render else 'headless'}-{rep + 1}",
                    count,
                    render=render,
                )
                reps.append(
                    {
                        "render": render,
                        "run": str(runpath),
                        "status": result["status"],
                        "timing": result["timing"],
                    }
                )
        passed = (
            full["status"] == "passed"
            and all(r["status"] == "passed" for r in reps)
            and all(r["timing"]["performance_passed"] for r in reps if r["render"])
        )
        rows.append(
            {
                "count": count,
                "full_run": str(folder),
                "full_status": full["status"],
                "repetitions": reps,
                "promoted": passed,
            }
        )
        (out / "scaling.json").write_text(json.dumps(rows, indent=2) + "\n")
    qualified = [r["count"] for r in rows if r["promoted"]]
    candidate = max(qualified) if qualified else None
    confirmation = None
    if candidate is not None:
        result, path = run("candidate-six-cycles", candidate, "full", True, "cycle")
        confirmation = {"run": str(path), "status": result["status"]}
        if result["status"] != "passed":
            candidate = None
    gui = None
    if candidate is not None and args.gui_benchmark:
        result, path = run(
            "candidate-gui", candidate, extra=("--gui", "--gui-benchmark")
        )
        gui = {"run": str(path), "status": result["status"], "timing": result["timing"]}
    off, offpath = run(
        "pair-off",
        1,
        "full",
        extra=("--layout", "contact-pair", "--pair-collisions", "off"),
    )
    on, onpath = run(
        "pair-on",
        1,
        "full",
        extra=("--layout", "contact-pair", "--pair-collisions", "on"),
    )
    # Reconstruct only the lower leaf from compact poses, comparing complete trajectories.
    from model import Config, skin, skin_setup

    mesh = np.load(onpath / "rest_mesh.npz")
    p = mesh["points"]
    centers, indices, weights = skin_setup(p, Config(**read(onpath / "config.json")))
    a = np.load(offpath / "trace.npz")
    b = np.load(onpath / "trace.npz")
    displacement = []
    if a["rigid_poses"].shape != b["rigid_poses"].shape or not np.allclose(
        a["times"], b["times"]
    ):
        raise RuntimeError("Pair control timelines differ")
    for t, x, y in zip(a["times"], a["rigid_poses"][:, 1], b["rigid_poses"][:, 1]):
        if 9 <= t <= 14:
            displacement.append(
                float(
                    np.linalg.norm(
                        skin(p, centers, indices, weights, x[:, :3], x[:, 3:])
                        - skin(p, centers, indices, weights, y[:, :3], y[:, 3:]),
                        axis=1,
                    ).max()
                )
            )
    transferred = max(displacement, default=0.0)
    pair = {
        "off": str(offpath),
        "on": str(onpath),
        "lower_difference_m": transferred,
        "passed": bool(
            off["status"] == "passed"
            and on["status"] == "passed"
            and transferred > 0.0001
        ),
    }
    gui_count = (
        candidate
        if gui and gui["status"] == "passed" and gui["timing"]["performance_passed"]
        else None
    )
    decision = {
        "maximum_verified_gui_count": gui_count,
        "maximum_verified_offscreen_count": candidate,
        "scaling": rows,
        "confirmation": confirmation,
        "gui": gui,
        "pair": pair,
        "visual_acceptance": "pending",
        "no_extrapolation": True,
    }
    (out / "decision.json").write_text(json.dumps(decision, indent=2) + "\n")
    lines = [
        "# Fixed-plant smoke results",
        "",
        f"Maximum verified offscreen count: {candidate}; GUI count: {gui_count}.",
        "",
        "| Lamine | Full checks | Render RTF min | Render frame p95 max (ms) | Promoted |",
        "|---:|---|---:|---:|---|",
    ]
    for row in rows:
        rr = [r for r in row["repetitions"] if r["render"]]
        lines.append(
            f"| {row['count']} | {row['full_status']} | {min(r['timing']['realtime_factor'] for r in rr):.3f} | {1000 * max(r['timing']['frame_work']['p95_seconds'] for r in rr):.3f} | {row['promoted']} |"
        )
    lines += [
        "",
        f"Pair contact comparison passed: {pair['passed']}; lower displacement difference: {transferred * 1000:.3f} mm.",
        "",
        "GUI measurement and human visual acceptance are separate. See decision.json and individual reports.",
    ]
    (out / "REPORT.md").write_text("\n".join(lines) + "\n")
    print(
        json.dumps({"maximum_verified_offscreen_count": candidate, "pair": pair}),
        flush=True,
    )


if __name__ == "__main__":
    main()
