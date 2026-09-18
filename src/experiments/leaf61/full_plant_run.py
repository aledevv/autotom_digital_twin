"""Reproducible, isolated launcher for the native plant with flexible blades."""

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--render", action="store_true")
    p.add_argument("--seconds", type=float, default=20)
    p.add_argument("--leaves", type=int, default=131)
    p.add_argument("--leaf-profile", choices=["original", "soft"], default="original")
    p.add_argument("--optimized", action="store_true")
    p.add_argument("--adaptive-render", action="store_true")
    p.add_argument("--camera", choices=["leaf", "plant"], default="leaf")
    p.add_argument("--skin-batch", type=int, default=0)
    p.add_argument("--fabric", action="store_true")
    p.add_argument("--gpu-physics", action="store_true")
    p.add_argument("--partition-trunk", action="store_true")
    p.add_argument("--partition-branches", action="store_true")
    p.add_argument("--diagnostics", choices=["full", "light", "live"], default="full")
    p.add_argument("--physics-hz", type=int, choices=[120, 240, 480], default=480)
    p.add_argument("--solver", choices=["PGS", "TGS"], default="PGS")
    p.add_argument("--physics-threads", type=int, default=8)
    p.add_argument("--iterations", type=int, default=32)
    p.add_argument("--leaf-armature", type=float, default=0)
    p.add_argument("--sleep-threshold", type=float, default=0)
    p.add_argument("--contact-probe", action="store_true")
    p.add_argument("--load-target", choices=["leaf", "branch"], default="leaf")
    p.add_argument("--exclude-initial-from", type=Path)
    p.add_argument("--leaf-contacts", choices=["off", "plant", "all"], default="off")
    p.add_argument("--run-dir", type=Path)
    a = p.parse_args()
    if not 1 <= a.leaves <= 131 or a.seconds <= 0:
        p.error("Use 1..131 leaves and a positive duration")
    root = Path(__file__).resolve().parents[3]
    source = (
        root / "artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda"
    )
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != "e21c37aae8d16339c69d5ed24258c11a6b6dc6c6c3339b9e3e34504086b4579a":
        raise RuntimeError("Native source differs from the verified main fixture")
    out = (
        a.run_dir
        or root
        / "artifacts/leaf61"
        / ("full-plant-" + datetime.now(timezone.utc).strftime("%Y%m%d-%H%M%S"))
    ).resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out / "sources").mkdir()
    for f in Path(__file__).parent.glob("*.py"):
        shutil.copy2(f, out / "sources" / f.name)
    shutil.copy2(source, out / "input.usda")
    cmd = [
        str(Path.home() / "isaacsim-6.1/python.sh"),
        str(out / "sources/full_plant_scene.py"),
        "--run-dir",
        str(out),
        "--seconds",
        str(a.seconds),
        "--leaves",
        str(a.leaves),
        "--leaf-profile",
        a.leaf_profile,
        "--leaf-contacts",
        a.leaf_contacts,
    ]
    cmd += [
        "--physics-hz",
        str(a.physics_hz),
        "--solver",
        a.solver,
        "--physics-threads",
        str(a.physics_threads),
        "--iterations",
        str(a.iterations),
        "--sleep-threshold",
        str(a.sleep_threshold),
    ]
    cmd += ["--skin-batch", str(a.skin_batch), "--leaf-armature", str(a.leaf_armature)]
    cmd += ["--diagnostics", a.diagnostics]
    if a.partition_branches:
        cmd.append("--partition-branches")
    if a.partition_trunk:
        cmd.append("--partition-trunk")
    if a.gpu_physics:
        cmd.append("--gpu-physics")
    if a.fabric:
        cmd.append("--fabric")
    cmd += ["--camera", a.camera]
    if a.adaptive_render:
        cmd.append("--adaptive-render")
    if a.optimized:
        cmd.append("--optimized")
    cmd += ["--load-target", a.load_target]
    if a.contact_probe:
        cmd.append("--contact-probe")
    if a.exclude_initial_from:
        if a.leaf_contacts == "off":
            p.error("Initial-overlap exclusions require contacts enabled")
        evidence_dir = a.exclude_initial_from.resolve()
        launch = json.loads((evidence_dir / "launch.json").read_text())
        runtime = json.loads((evidence_dir / "runtime.json").read_text())
        if (
            launch["source_sha256"] != digest
            or runtime["leaves"] != a.leaves
            or runtime.get("leaf_profile", "original") != a.leaf_profile
            or runtime.get("leaf_profile_revision", 1)
            != (3 if a.leaf_profile == "soft" else 1)
        ):
            raise RuntimeError(
                "Initial contact evidence must match source and leaf count"
            )
        shutil.copy2(
            evidence_dir / "contacts.json", out / "initial_contact_evidence.json"
        )
        cmd += ["--exclude-initial-from", str(out / "initial_contact_evidence.json")]
    if not a.headless:
        cmd.append("--gui")
    if a.render:
        cmd.append("--render")
    (out / "launch.json").write_text(
        json.dumps(
            {
                "command": cmd,
                "source": str(source),
                "source_sha256": digest,
                "code_sha256": {
                    f.name: hashlib.sha256(f.read_bytes()).hexdigest()
                    for f in (out / "sources").glob("*.py")
                },
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
