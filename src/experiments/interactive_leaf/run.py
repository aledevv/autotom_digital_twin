"""Isolated interactive leaf launcher (system Python supervises Isaac Python)."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gui", action="store_true")
    p.add_argument("--gui-test", action="store_true", help="Exercise GUI callbacks for a bounded contact cycle")
    p.add_argument("--render", action="store_true", help="Offscreen render and snapshots")
    p.add_argument("--hz", type=int, choices=(60, 120, 240))
    p.add_argument("--scenario", choices=("rest", "press", "drop", "cycle"), default="cycle")
    p.add_argument("--duration", type=float, default=20)
    p.add_argument("--config", type=Path)
    p.add_argument("--run-dir", type=Path)
    p.add_argument("--timeout", type=float, default=240)
    a = p.parse_args()
    if a.gui_test:
        a.gui = True
    if not math.isfinite(a.duration) or not math.isfinite(a.timeout) or a.duration < 20 or a.timeout <= 0:
        p.error("duration must be >= 20 s; timeout must be positive")
    out = (a.run_dir or ROOT/"artifacts/interactive_leaf"/(
        datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")+("-gui" if a.gui else "-test"))).resolve()
    out.mkdir(parents=True, exist_ok=False)
    config = json.loads(a.config.read_text()) if a.config else {}
    if a.hz is not None:
        config["hz"] = a.hz
    (out/"config.json").write_text(json.dumps(config, indent=2)+"\n")
    worker = Path(__file__).with_name("scene.py")
    isaac = os.environ.get("ISAAC_PYTHON", str(Path.home()/"isaacsim/python.sh"))
    cmd = [isaac, str(worker), "--run-dir", str(out), "--scenario", a.scenario,
           "--duration", str(a.duration)]
    if a.gui:
        cmd.append("--gui")
    if a.render:
        cmd.append("--render")
    if a.gui_test:
        cmd.append("--gui-test")
    provenance = {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in worker.parent.glob("*.py")}
    (out/"launch.json").write_text(json.dumps(dict(command=cmd, cwd=str(ROOT),
        source_sha256=provenance), indent=2)+"\n")
    env = dict(os.environ)
    for name in ("PYTHONHOME", "PYTHONEXE", "VIRTUAL_ENV", "CONDA_PREFIX"):
        env.pop(name, None)
    print(f"RUN_DIR={out}", flush=True)
    start = time.monotonic()
    with (out/"isaac.log").open("w") as log:
        process = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            code = process.wait(timeout=None if a.gui and not a.gui_test else a.timeout)
        except (subprocess.TimeoutExpired, KeyboardInterrupt) as exc:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            code = 124 if isinstance(exc, subprocess.TimeoutExpired) else 130
    errors = [s for s in (out/"isaac.log").read_text(errors="replace").splitlines()
              if "[Error]" in s and any(x in s.lower() for x in ("physx", "physics.tensors", "cuda"))]
    (out/"process.json").write_text(json.dumps(dict(returncode=code, physics_errors=errors,
        wall_seconds=time.monotonic()-start), indent=2)+"\n")
    rp = out/"report.json"
    report = json.loads(rp.read_text()) if rp.exists() else dict(status="failed", reason="Missing report")
    report["process_ok"] = code == 0 and not errors
    if not report["process_ok"]:
        report["status"] = "failed"
    rp.write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] in ("passed", "manual") else 1


if __name__ == "__main__":
    sys.exit(main())
