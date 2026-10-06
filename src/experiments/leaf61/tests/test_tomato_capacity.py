"""The FPS floor must not hide invalid physics or an uneven frame budget."""

import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "tomato_capacity", Path(__file__).parents[1] / "tomato_capacity.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def report(fps=25, p95=0.04, status="passed"):
    return {
        "status": status,
        "process_ok": True,
        "timing": {
            "rendered_fps_unpaced": fps,
            "frame_work": {"p95_seconds": p95},
            "realtime_factor": 0.8,
        },
    }


def test_fps_is_separate_from_simulation_speed():
    assert module.assess(report())
    assert not module.assess(report(fps=20))


def test_frame_tail_and_physics_are_acceptance_gates():
    assert not module.assess(report(p95=0.051))
    assert not module.assess(report(status="failed"))
    r = report()
    r["process_ok"] = False
    assert not module.assess(r)
