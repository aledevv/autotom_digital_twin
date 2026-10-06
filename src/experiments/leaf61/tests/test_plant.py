"""Physical invariants affected by sharing the world and transforming leaves."""

import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


def load(name, filename):
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).parents[1] / filename
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


m = load("leaf61_plant_test_model", "model.py")
previous = sys.modules.get("model")
try:
    sys.modules["model"] = m
    pm = load("leaf61_plant_test_helpers", "plant_model.py")
finally:
    if previous is None:
        sys.modules.pop("model", None)
    else:
        sys.modules["model"] = previous
Config, rectangle, rotation, skin, skin_setup = (
    m.Config,
    m.rectangle,
    m.rotation,
    m.skin,
    m.skin_setup,
)
layout, local_poses, verify_layout, visual_crossing, yaw_matrix = (
    pm.layout,
    pm.local_poses,
    pm.verify_layout,
    pm.visual_crossing,
    pm.yaw_matrix,
)


def test_nested_layout_and_clearance():
    p, _ = rectangle(Config())
    for n in (1, 5, 10, 20):
        placements = layout(n)
        assert len(verify_layout(p, placements, 0.0005)) == n
        assert np.allclose([x[0] for x in placements], [x[0] for x in layout(20)[:n]])
    with pytest.raises(ValueError, match="overlap"):
        verify_layout(p, [(np.zeros(3), 0.0)] * 2, 0.0005)


def test_world_pose_inverse_keeps_skin_in_local_coordinates():
    c = Config()
    p, _ = rectangle(c)
    centers, indices, weights = skin_setup(p, c)
    angle = 0.73
    offset = np.array([0.2, -0.3, 0.12])
    q = np.tile([np.cos(angle / 2), 0, 0, np.sin(angle / 2)], (4, 1))
    wp = centers @ yaw_matrix(angle).T + offset
    lp, lq = local_poses(wp, q, offset, angle)
    assert np.allclose(lp, centers)
    assert np.allclose(rotation(lq), np.eye(3))
    assert np.allclose(skin(p, centers, indices, weights, lp, lq), p)


def test_pair_initial_gap_and_crossing():
    p, f = rectangle(Config(nx=4, ny=4))
    upper, lower = layout(1, "contact-pair")
    low = p + lower[0]
    assert visual_crossing(p, low, f) == 0
    assert visual_crossing(p - 0.004, low, f) == pytest.approx(0.001)
    assert upper[0][2] - lower[0][2] == pytest.approx(0.003)


def test_reuse_fingerprint_rejects_changed_physics(tmp_path):
    import shutil

    campaign = load("leaf61_plant_test_campaign", "plant_campaign.py")
    source = Path(__file__).parents[1]
    for name in ("model.py", "plant_model.py", "scene.py", "plant_scene.py"):
        shutil.copyfile(source / name, tmp_path / name)
    baseline = campaign.physics_fingerprint(tmp_path)
    model = tmp_path / "model.py"
    model.write_text(
        model.read_text().replace(
            "joint_stiffness: float = 0.024", "joint_stiffness: float = 0.023"
        )
    )
    assert baseline != campaign.physics_fingerprint(tmp_path)
    shutil.copyfile(source / "model.py", model)
    scene = tmp_path / "plant_scene.py"
    scene.write_text(
        scene.read_text().replace(
            "camera.CreateFocalLengthAttr().Set(35.0)",
            "camera.CreateFocalLengthAttr().Set(34.0)",
        )
    )
    assert baseline == campaign.physics_fingerprint(tmp_path)
