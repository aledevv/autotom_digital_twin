"""Snapshot and launch the isolated coupled branch test using Isaac Sim 6.1."""

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--fixed", action="store_true")
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--tomato", action="store_true")
    parser.add_argument(
        "--runtime", choices=["baseline", "optimized"], default="optimized"
    )
    parser.add_argument("--branches", type=int, choices=[1, 2, 5, 10], default=1)
    a = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    out = (
        a.run_dir
        or root
        / "artifacts/leaf61"
        / ("branch-" + datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S"))
    ).resolve()
    out.mkdir(parents=True, exist_ok=False)
    sources = out / "sources"
    sources.mkdir()
    for source in Path(__file__).parent.glob("*.py"):
        shutil.copy2(source, sources / source.name)
    shutil.copy2(root / "artifacts/leaf61/real-seed42.npz", out / "input_mesh.npz")
    cmd = [
        str(Path.home() / "isaacsim-6.1/python.sh"),
        str(sources / "branch_scene.py"),
        "--run-dir",
        str(out),
    ]
    cmd += ["--runtime", a.runtime, "--branches", str(a.branches)]
    if not a.headless:
        cmd += ["--gui"]
    if a.fixed:
        cmd += ["--fixed"]
    if a.tomato:
        cmd += ["--tomato"]
    (out / "launch.json").write_text(
        json.dumps(
            {
                "command": cmd,
                "source_sha256": {
                    p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sources.glob("*.py")
                },
            },
            indent=2,
        )
    )
    isaac = Path.home() / "isaacsim-6.1"
    try:
        gpu = subprocess.run(
            ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"],
            capture_output=True,
            text=True,
            check=False,
        )
        gpu_info = gpu.stdout.strip() if gpu.returncode == 0 else gpu.stderr.strip()
    except OSError as exc:
        gpu_info = str(exc)
    (out / "environment.json").write_text(
        json.dumps(
            {
                "isaac_version": (isaac / "VERSION").read_text().strip(),
                "physx_extension_candidates": [
                    p.name for p in (isaac / "extscache").glob("omni.physx-*")
                ],
                "gpu_and_driver": gpu_info,
            },
            indent=2,
        )
    )
    print("RUN_DIR=" + str(out), flush=True)
    with (out / "isaac.log").open("w") as log:
        result = subprocess.run(
            cmd, cwd=root, stdout=log, stderr=subprocess.STDOUT, check=False
        )
    (out / "process.json").write_text(json.dumps({"returncode": result.returncode}))
    raise SystemExit(result.returncode)


if __name__ == "__main__":
    main()
