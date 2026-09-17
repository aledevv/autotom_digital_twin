"""Supervised D0 launcher. Uses system Python; Isaac runs in its own interpreter."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--hz", type=int, choices=(60, 120, 240), default=120)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--offscreen-render", action="store_true", help="Exercise GUI render cadence without opening a window")
    p.add_argument("--run-dir", type=Path)
    p.add_argument("--config", type=Path, help="JSON overrides of model.Config")
    p.add_argument("--timeout", type=float, default=240, help="Headless wall timeout, including startup")
    a = p.parse_args()
    if a.timeout <= 0:
        p.error("timeout must be positive")
    out = a.run_dir or ROOT / "artifacts/deformable_leaf" / (
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")+f"-d0-{a.hz}hz"+("-gui" if a.gui else ""))
    out = out.resolve()
    out.mkdir(parents=True, exist_ok=False)
    overrides = json.loads(a.config.read_text()) if a.config else {}
    overrides["hz"] = a.hz
    (out / "config.json").write_text(json.dumps(overrides, indent=2)+"\n")
    isaac = Path(os.environ.get("ISAAC_PYTHON", str(Path.home()/"isaacsim/python.sh")))
    cmd = [str(isaac), str(Path(__file__).with_name("scene.py")), "--run-dir", str(out)]
    if a.gui:
        cmd.append("--gui")
    if a.offscreen_render:
        cmd.append("--offscreen-render")
    provenance = {f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                  for f in Path(__file__).parent.glob("*.py")}
    (out / "launch.json").write_text(json.dumps(dict(command=cmd, cwd=str(ROOT),
        launcher_python=sys.executable, source_sha256=provenance), indent=2)+"\n")
    env = dict(os.environ)
    for key in ("PYTHONHOME", "PYTHONEXE", "VIRTUAL_ENV", "CONDA_PREFIX"):
        env.pop(key, None)
    print(f"RUN_DIR={out}", flush=True)
    started = time.monotonic()
    timed_out = False
    with (out/"isaac.log").open("w") as log:
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log,
                                stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = proc.wait(timeout=None if a.gui else a.timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            import signal
            timed_out = isinstance(exc, subprocess.TimeoutExpired)
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
            code = 124 if timed_out else 130
    log_text = (out/"isaac.log").read_text(errors="replace")
    physics_errors = [line for line in log_text.splitlines()
                      if "[Error]" in line and any(s in line.lower() for s in ("physx", "cuda", "physics.tensors"))]
    (out/"process.json").write_text(json.dumps(dict(returncode=code, timed_out=timed_out,
        wall_seconds=time.monotonic()-started, physics_errors=physics_errors), indent=2)+"\n")
    report_path = out/"report.json"
    report = json.loads(report_path.read_text()) if report_path.exists() else {"status": "failed", "reason": "No runtime report"}
    report["process_ok"] = code == 0 and not physics_errors
    if not report["process_ok"]:
        report["status"] = "failed"
    report_path.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
