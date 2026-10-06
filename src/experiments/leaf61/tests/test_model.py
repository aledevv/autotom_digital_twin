import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location(
    "leaf61_model", Path(__file__).parents[1] / "model.py"
)
m = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = m
spec.loader.exec_module(m)


def test_mesh_scale_area_and_normals():
    c = m.Config()
    p, f = m.rectangle(c)
    assert p.shape == (465, 3) and f.shape == (840, 3)
    assert m.area(p, f).sum() == pytest.approx(0.09 * 0.04)
    assert np.all(np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]])[:, 2] > 0)
    assert np.count_nonzero(p[:, 0] <= 0.006 + 1e-9) == 45


def test_contact_triangle_interior_and_miss():
    p = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], float)
    f = np.array([[0, 1, 2]])
    assert m.sphere_gap(p, f, np.array([0.2, 0.2, 0.1]), 0.1) == pytest.approx(0)
    assert m.sphere_gap(p, f, np.array([0.2, 0.2, 0.05]), 0.1) == pytest.approx(-0.05)
    assert m.first_contact_height(p, f, 0.2, 0.2, 0.1) == pytest.approx(0.1)
    with pytest.raises(ValueError):
        m.first_contact_height(p, f, 2, 2, 0.1)


def test_skin_fixed_region_and_bind_pose():
    c = m.Config()
    p, _f = m.rectangle(c)
    centers, ids, w = m.skin_setup(p, c)
    q = np.tile([1.0, 0, 0, 0], (4, 1))
    np.testing.assert_allclose(m.skin(p, centers, ids, w, centers, q), p, atol=1e-16)
    moved = centers.copy()
    moved[1:, 2] -= 0.01
    result = m.skin(p, centers, ids, w, moved, q)
    fixed = p[:, 0] <= 0.006 + 1e-9
    np.testing.assert_allclose(result[fixed], p[fixed], atol=1e-16)


def test_invalid_and_false_pass_guards():
    with pytest.raises(ValueError):
        m.Config(young=float("nan")).validate()
    with pytest.raises(ValueError):
        m.Config(nx=1).validate()
    c = m.Config()
    p, f = m.rectangle(c)
    report = m.evaluate(
        c, p, f, np.array([p, p]), np.array([0, 9]), np.zeros((2, 3)), []
    )
    assert (
        report["status"] == "failed"
        and not report["checks"]["cycles"]
        and not report["checks"]["sag"]
    )
    bad = p.copy()
    bad[0] = np.nan
    assert not m.evaluate(
        c, p, f, np.array([p, bad]), np.array([0, 9]), np.zeros((2, 3)), []
    )["checks"]["finite"]


def test_material_target_follows_sag_and_preserves_side():
    c = m.Config()
    p, f = m.rectangle(c)
    moved = p.copy()
    moved[:, 0] -= 0.012
    moved[:, 2] -= 0.02
    target = m.material_target(p, moved, f, 0.0606, 0.01)
    np.testing.assert_allclose(target, [0.0486, 0.01, 0.13], atol=1e-8)
    with pytest.raises(ValueError):
        m.material_target(p, moved, f, 0.06, 0.1)


@pytest.mark.parametrize("failure", ["detach", "stretch", "collapse", "incomplete"])
def test_physical_failure_gates(failure):
    c = m.Config()
    p, f = m.rectangle(c)
    deformed = p.copy()
    if failure == "detach":
        deformed[:, 2] += 0.001
    if failure == "stretch":
        deformed[:, 0] *= 1.1
    if failure == "collapse":
        deformed[:, 1] = 0
    report = m.evaluate(
        c,
        p,
        f,
        np.array([p, deformed]),
        np.array([0, 9]),
        np.zeros((2, 3)),
        [],
        complete=failure != "incomplete",
    )
    gate = {
        "detach": "attachment",
        "stretch": "stretch",
        "collapse": "noncollapsed",
        "incomplete": "complete",
    }[failure]
    assert not report["checks"][gate]
    assert report["status"] == "failed"


def test_skin_collision_gap_is_not_hidden():
    c = m.Config()
    p, _ = m.rectangle(c)
    centers, _, _ = m.skin_setup(p, c)
    q = np.tile([1.0, 0, 0, 0], (4, 1))
    assert m.collider_mismatch(c, p, centers, q) == pytest.approx(0.001, abs=1e-7)
    floated = p.copy()
    floated[:, 2] += 0.01
    assert m.collider_mismatch(c, floated, centers, q) > 0.009


def test_convex_strip_and_prism_distances():
    c = m.Config()
    p, f = m.rectangle(c)
    poly = m.convex_strip(p, f, 0.006, 0.034, np.array([0.02, 0, 0.15]))
    assert len(poly) == 4
    pp, ff = m.prism_mesh(poly, 0.0005)
    assert len(pp) == 8 and len(ff) == 12
    # Closed consistently wound triangular prism has the expected positive signed volume.
    volume = (
        np.sum(
            np.einsum("ij,ij->i", pp[ff[:, 0]], np.cross(pp[ff[:, 1]], pp[ff[:, 2]]))
        )
        / 6
    )
    assert volume == pytest.approx(0.028 * 0.04 * 0.0005)
    distances = m.prism_distance(
        np.array([[0, 0, 0], [0, 0, 0.00125], [0.024, 0, 0]]), poly, 0.0005
    )
    np.testing.assert_allclose(distances, [-0.00025, 0.001, 0.01], atol=1e-10)


def test_root_hinge_does_not_create_artificial_stretch():
    c = m.Config()
    p, f = m.rectangle(c)
    centers, ids, w = m.skin_setup(p, c)
    angle = 0.4
    q = np.tile([np.cos(angle / 2), 0, np.sin(angle / 2), 0], (4, 1))
    q[0] = [1, 0, 0, 0]
    r = m.rotation(q)
    anchor = np.array([c.fixed_length, 0, c.height])
    positions = centers.copy()
    positions[1:] = np.einsum("bij,bj->bi", r[1:], centers[1:] - anchor) + anchor
    actual = m.skin(p, centers, ids, w, positions, q)
    e = m.edges(f)
    np.testing.assert_allclose(
        np.linalg.norm(actual[e[:, 1]] - actual[e[:, 0]], axis=1),
        np.linalg.norm(p[e[:, 1]] - p[e[:, 0]], axis=1),
        atol=1e-14,
    )
