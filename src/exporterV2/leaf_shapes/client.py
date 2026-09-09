"""Prepare all normalized blades before USD authoring, using one isolated worker."""
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import threading

from .config import LeafShapeConfig, load_leaf_shape_config


def leaflet_seed(global_seed, plant_id, structural_id, role):
    if role not in ("left", "right", "terminal"):
        raise ValueError(f"invalid leaflet role: {role!r}")
    payload = json.dumps(["autotom-leaf-seed-v1", global_seed, plant_id, structural_id, role], separators=(",", ":"), ensure_ascii=True)
    return int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big") & ((1 << 63)-1)


@dataclass
class PreparedLeafShapes:
    config: LeafShapeConfig
    shapes: dict
    runtime: dict

    def for_leaf(self, structural_id):
        if structural_id not in self.shapes:
            raise ValueError(f"leaf shape not prepared for {structural_id}; no fallback allowed")
        return self.shapes[structural_id]

    def manifest(self):
        records = []
        for key, shape in sorted(self.shapes.items()):
            records.append({k: v for k, v in shape.items() if k not in ("vertices", "triangles", "contour_xy")})
        # Wall time is diagnostic only: preserve byte-reproducible manifests/USD.
        runtime = {k: v for k, v in self.runtime.items() if k != "generation_seconds"}
        return {"config": asdict(self.config), "runtime": runtime, "leaves": records}


def _stop_worker(process):
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            process.wait()
            return
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def prepare_leaf_shapes(leaves, *, plant_id, config=None):
    config = config if config is not None else load_leaf_shape_config()
    records = []
    seen = set()
    for structural_id, role in leaves:
        if structural_id in seen:
            raise ValueError(f"duplicate leaflet identity: {structural_id}")
        seen.add(structural_id)
        records.append({"id": structural_id, "role": role, "seed": leaflet_seed(config.seed, plant_id, structural_id, role)})
    records.sort(key=lambda record: record["id"])
    if config.backend == "legacy" or not records:
        return PreparedLeafShapes(config, {r["id"]: {**r, "backend": "legacy"} for r in records}, {})
    uv = shutil.which("uv")
    if uv is None:
        raise ValueError("Realistic leaf generation requires uv in PATH")
    worker_dir = Path(__file__).resolve().parent
    env = os.environ.copy()
    for key in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV"):
        env.pop(key, None)
    env.setdefault("UV_CACHE_DIR", "/tmp/uv-cache")
    with tempfile.TemporaryDirectory(prefix="autotom-leaf-shapes-") as folder:
        request_path, result_path = Path(folder)/"request.json", Path(folder)/"result.json"
        request_path.write_text(json.dumps({"backend": config.backend, "generator": config.generator, "leaves": records}))
        command = [uv, "run", "--no-config", "--project", str(worker_dir/"runtime"), "--locked", "--python", "3.12", "python", str(worker_dir/"worker.py"), str(request_path), str(result_path)]
        print(f"[leaf shapes] {config.backend}: {len(records)} individual leaflets, global seed={config.seed}", flush=True)
        try:
            process = subprocess.Popen(command, env=env, start_new_session=True)
        except OSError as exc:
            raise ValueError(f"cannot start leaf shape worker: {exc}") from exc
        # SIGTERM must also stop uv and its child; otherwise interrupted exports orphan workers.
        main_thread = threading.current_thread() is threading.main_thread()
        previous_term = signal.getsignal(signal.SIGTERM)
        def terminate(_signum, _frame):
            raise KeyboardInterrupt("leaf generation terminated")
        if main_thread:
            signal.signal(signal.SIGTERM, terminate)
        try:
            status = process.wait(timeout=config.timeout_seconds)
        except subprocess.TimeoutExpired as exc:
            _stop_worker(process)
            raise ValueError(f"{config.backend} leaf generation timed out after {config.timeout_seconds}s") from exc
        except BaseException:
            _stop_worker(process)
            raise
        finally:
            if main_thread:
                signal.signal(signal.SIGTERM, previous_term)
        if status != 0 or not result_path.is_file():
            raise ValueError(f"{config.backend} leaf generation failed (exit {status}); see worker diagnostics above")
        result = json.loads(result_path.read_text())
    if result.get("schema") != 1 or set(result.get("shapes", {})) != seen:
        raise ValueError("leaf worker returned an incomplete or unsupported result")
    for record in records:
        shape = result["shapes"][record["id"]]
        if any(shape[k] != v for k, v in record.items()) or shape["backend"] != config.backend:
            raise ValueError(f"leaf worker identity mismatch: {record['id']}")
    print(f"[leaf shapes] completed in {result['runtime']['generation_seconds']:.2f}s", flush=True)
    return PreparedLeafShapes(config, result["shapes"], result["runtime"])


def author_shape_manifest(stage, prepared):
    """Persist full provenance without storing the contour twice in USD."""
    from pxr import Sdf
    root = stage.GetDefaultPrim()
    root.CreateAttribute("autotom:leafShapes", Sdf.ValueTypeNames.String, custom=True).Set(
        json.dumps(prepared.manifest(), sort_keys=True, allow_nan=False)
    )
