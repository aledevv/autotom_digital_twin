"""Run prepared headless cases sequentially, one Isaac process per fresh scene."""
from __future__ import annotations

import argparse
import json
import os
import signal
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--timeout", type=float,
                        help="Wall seconds per case; default max(3600, 60 * simulated seconds)")
    args = parser.parse_args()
    if args.timeout is not None and args.timeout <= 0:
        parser.error("timeout must be positive")
    isaac = Path(os.environ.get("ISAACSIM_DIR", str(Path.home() / "isaacsim"))) / "python.sh"
    for directory in args.run_dirs:
        directory = directory.resolve()
        config = json.loads((directory / "config.json").read_text())
        if (directory / "report.json").exists():
            raise ValueError(f"report already exists: {directory}; prepare a new case")
        command = [str(isaac), str(ROOT / "src/exporterV2/isaac_app.py"), "--usd", str(directory / "scene.usda"),
                   "--physics-preset", "flexible", "--physics-hz", str(config["hz"]), "--duration", str(config["duration"]),
                   "--headless", "--fruit-experiment", str(directory / "config.json")]
        print(f"[START] {directory.name}", flush=True)
        start = time.monotonic()
        with (directory / "isaac.log").open("w") as log:
            process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                exit_code = process.wait(timeout=args.timeout or max(3600, 60 * config["duration"]))
            except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                if isinstance(exc, KeyboardInterrupt):
                    raise
                exit_code = 124
        report = json.loads((directory / "report.json").read_text()) if (directory / "report.json").exists() else {}
        status = report.get("status", "runtime_error")
        if status == "running":
            status = "runtime_error"
        summary = {"case": directory.name, "exit_code": exit_code, "wall_seconds": time.monotonic() - start,
                   "status": status, "errors": report.get("errors", [])}
        (directory / "process.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
