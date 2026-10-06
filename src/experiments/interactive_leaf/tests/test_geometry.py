from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from experiments.interactive_leaf.geometry import Config, geometry, rotations, skin, press_fraction
from experiments.interactive_leaf.evaluate import evaluate


def test_skin_rest_and_rigid_motion():
    c = Config()
    p, f, centers, indices, weights = geometry(c)
    q = np.tile([1., 0, 0, 0], (c.links, 1))
    assert np.allclose(weights.sum(axis=1), 1)
    assert (weights >= 0).all()
    assert np.allclose(skin(p, centers, indices, weights, centers, q), p)
    q = np.tile([np.cos(.2), 0, np.sin(.2), 0], (c.links, 1))
    r = rotations(q)[0]
    translation = np.array([.1, .2, -.05])
    result = skin(p, centers, indices, weights, centers@r.T+translation, q)
    assert np.allclose(result, p@r.T+translation)
    normals = np.cross(p[f[:, 1]]-p[f[:, 0]], p[f[:, 2]]-p[f[:, 0]])
    assert (normals[:, 2] > 0).all()


def test_bending_preserves_attachment_and_continuity():
    c = Config()
    p, f, centers, indices, weights = geometry(c)
    q = np.tile([1., 0, 0, 0], (c.links, 1))
    pos = centers.copy()
    ds = c.length/c.links
    hinge = np.array([c.petiole_length, 0., c.height])
    for i in range(c.links):
        angle = .08*(i+1)
        q[i] = [np.cos(angle/2), 0, np.sin(angle/2), 0]
        direction = rotations(q[i:i+1])[0]@np.array([1., 0, 0])
        pos[i] = hinge+direction*ds/2
        hinge += direction*ds
    result = skin(p, centers, indices, weights, pos, q)
    assert np.linalg.norm(result[:9].mean(axis=0)-p[:9].mean(axis=0)) < 1e-10
    assert result[-9:, 2].mean() < c.height-.01
    # No cracks: same vertices shared by all adjacent faces; no substantial stretch.
    old = np.linalg.norm(p[f[:, 1]]-p[f[:, 0]], axis=1)
    new = np.linalg.norm(result[f[:, 1]]-result[f[:, 0]], axis=1)
    assert (new/old).max() < 1.06


def test_contact_profile_is_slow_and_releases():
    t = np.linspace(0, 20, 2401)
    value = np.array([press_fraction(x) for x in t])
    assert value.min() == 0 and value.max() == 1
    assert (value[t > 7] == 0).all()
    assert abs(np.diff(value)).max() < .01


@pytest.mark.parametrize("override", [dict(bend_stiffness=0), dict(ball_mass=-1),
    dict(hz=10), dict(links=1), dict(press_depth=1), dict(target_y=1), dict(target_y=float("nan"))])
def test_config_rejects_invalid_cases(override):
    with pytest.raises(ValueError):
        replace(Config(), **override).validate()


def test_rigid_noncontact_leaf_cannot_pass():
    c = Config()
    t = np.arange(1, 2401)/120
    data = np.zeros((len(t), 15))
    data[:, 0:2] = t[:, None]
    data[:, 5] = c.height
    report = evaluate(data, c, "cycle", 20)
    assert report["status"] == "failed"
    assert not report["checks"]["press_contact"]
    assert not report["checks"]["local_bending"]
    assert not report["checks"]["drop_contact"]


def test_later_cycle_cannot_hide_failed_contact_or_residual_motion():
    c = Config()
    t = np.arange(1, 7201)/120
    phase = t % 20
    data = np.zeros((len(t), 15))
    data[:, 0] = t
    data[:, 1] = phase
    data[:, 5] = c.height
    press = (phase > 4) & (phase < 5)
    drop = (phase > 9) & (phase < 12)
    data[press | drop, 8] = -.01
    data[press | drop, 10] = .02
    assert evaluate(data, c, "cycle", 60)["status"] == "passed"
    data[(t > 20) & (t < 40) & press, 10] = 0
    result = evaluate(data, c, "cycle", 60)
    assert not result["checks"]["all_cycles"]
    assert not result["cycles"][1]["checks"]["press"]
