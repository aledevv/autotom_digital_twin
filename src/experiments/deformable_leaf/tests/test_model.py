"""Geometry invariants and negative controls for the D0 screening gates."""
from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from experiments.deformable_leaf.model import Config, make_mesh, signed_volumes, summarize


def test_closed_conforming_volume_and_outward_surface():
    c = Config(nx=6, ny=4, nz=2)
    points, tets, faces = make_mesh(c)
    volumes = signed_volumes(points, tets)
    assert (volumes > 0).all()
    assert volumes.sum() == pytest.approx(c.length*c.width*c.thickness)
    surface_volume = np.einsum("ij,ij->i", points[faces[:, 0]],
        np.cross(points[faces[:, 1]], points[faces[:, 2]])).sum()/6
    assert surface_volume == pytest.approx(volumes.sum())
    edges = Counter(tuple(sorted(pair)) for face in faces
                    for pair in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])))
    assert set(edges.values()) == {2}
    tet_faces = Counter(tuple(sorted(np.delete(tet, k))) for tet in tets for k in range(4))
    assert set(tet_faces.values()) == {1, 2}
    assert sum(n == 1 for n in tet_faces.values()) == len(faces)


@pytest.mark.parametrize("overrides", [dict(nz=1), dict(thickness=0), dict(hz=0),
    dict(young=float("nan")), dict(poisson=0.5), dict(clamp_length=0.07),
    dict(velocity_damping=-1), dict(recovery_seconds=0.1)])
def test_invalid_config(overrides):
    with pytest.raises(ValueError):
        replace(Config(), **overrides).validate()


def trace():
    c = Config(nx=10, ny=4, nz=2)
    rest, tets, _ = make_mesh(c)
    times = np.array([0., 7.1, 7.5, 8., 8.5, 15.1, 16.])
    nodes = np.broadcast_to(rest, (len(times), *rest.shape)).copy()
    profile = np.maximum(rest[:, 0]-c.clamp_length, 0)**2/(c.length-c.clamp_length)**2
    nodes[1:4, :, 2] -= 0.004*profile
    return c, times, nodes, tets, rest


def evaluate(data):
    return summarize(*data, [0.001], [0.001], 1.0)


def test_sag_and_recovery_pass():
    report = evaluate(trace())
    assert report["status"] == "passed"
    assert report["tip_sag_m"] == pytest.approx(0.004)
    json.dumps(report, allow_nan=False)


@pytest.mark.parametrize("failure", ["attachment", "inversion", "nan", "no_sag", "no_recovery"])
def test_failed_physics_is_detected(failure):
    data = trace()
    c, times, nodes, tets, rest = data
    if failure == "attachment":
        nodes[:, :, 0] += 0.002
    elif failure == "inversion":
        nodes[2, :, 2] = 2*c.height-nodes[2, :, 2]
    elif failure == "nan":
        nodes[2, 0, 0] = np.nan
    elif failure == "no_sag":
        nodes[:] = rest
    else:
        nodes[-2:] = nodes[2]
    assert evaluate(data)["status"] == "failed"


def test_interrupted_trace_is_finite_json_and_fails():
    c, times, nodes, tets, rest = trace()
    report = evaluate((c, times[:2], nodes[:2], tets, rest))
    assert not report["checks"]["complete"]
    assert report["status"] == "failed"
    json.dumps(report, allow_nan=False)
