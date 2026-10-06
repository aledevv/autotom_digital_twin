"""Validate conservative certificates against explicit perturbed surface meshes."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from geometry_bounds import edge_error_bound, metric_bounds, skin_error_bound

    from model import Config, area, edges, rectangle, skin, skin_setup
finally:
    sys.path.pop(0)


def test_bounds_cover_explicit_mesh_at_every_time():
    c = Config(nx=4, ny=4)
    p, f = rectangle(c)
    centers, indices, weights = skin_setup(p, c)
    reference = np.zeros((11, 4, 7))
    reference[:, :, :3] = centers
    reference[:, :, 3] = 1
    actual = reference.copy()
    rng = np.random.default_rng(31)
    actual[:, :, :3] += rng.normal(size=(11, 4, 3)) * 1e-6
    actual[:, :, 3:] += rng.normal(size=(11, 4, 4)) * 1e-5
    error = skin_error_bound(actual, reference, p, centers)
    trace = np.array(
        [
            skin(p, centers, indices, weights, pose[:, :3], pose[:, 3:])
            for pose in actual
        ]
    )
    assert np.all(np.linalg.norm(trace - p, axis=2).max(axis=1) <= error)
    exact_reference = {
        "leaf": 0,
        "max_deflection_from_equilibrium_m": 0,
        "recovery_last_second_m": 0,
        "residual_motion_last_second_m": 0,
        "gravity_sag_m": 0,
        "attachment_drift_m": 0,
        "max_edge_extension": 0,
        "min_area_ratio": 1,
    }
    times = np.linspace(0, 2, 11)
    edge_error = edge_error_bound(actual, reference, p, centers, indices, weights, f)
    bounds = metric_bounds(exact_reference, error, times, 2, p, f, edge_error)
    e = edges(f)
    difference = (trace[:, e[:, 1]] - trace[:, e[:, 0]]) - (p[e[:, 1]] - p[e[:, 0]])
    assert np.all(
        np.max(
            np.linalg.norm(difference, axis=2)
            / np.linalg.norm(p[e[:, 1]] - p[e[:, 0]], axis=1),
            axis=1,
        )
        <= edge_error
    )
    stretch = (
        np.linalg.norm(trace[:, e[:, 1]] - trace[:, e[:, 0]], axis=2)
        / np.linalg.norm(p[e[:, 1]] - p[e[:, 0]], axis=1)
    ).max() - 1
    minimum_area = min((area(v, f) / area(p, f)).min() for v in trace)
    assert stretch <= bounds["max_edge_extension"]
    assert minimum_area >= bounds["min_area_ratio"]
    assert (
        np.linalg.norm(trace - trace[2], axis=2).max()
        <= bounds["max_deflection_from_equilibrium_m"]
    )
    tail = trace[times >= 1]
    assert (
        np.linalg.norm(np.ptp(tail, axis=0), axis=1).max()
        <= bounds["residual_motion_last_second_m"]
    )


def test_convex_hull_layout_accepts_disjoint_rotated_leaves_but_rejects_overlap():
    import pytest
    from plant_model import verify_layout_hulls, yaw_matrix

    narrow = np.array(
        [[0, -0.005, 0.15], [0.09, -0.005, 0.15], [0.09, 0.005, 0.15], [0, 0.005, 0.15]]
    )
    yaw = np.pi / 4
    side = np.array([0, 0.014, 0]) @ yaw_matrix(yaw).T
    verify_layout_hulls(narrow, [(np.zeros(3), yaw), (side, yaw)], 0.0005)
    with pytest.raises(ValueError, match="overlap"):
        verify_layout_hulls(narrow, [(np.zeros(3), yaw), (side / 2, yaw)], 0.0005)
    with pytest.raises(ValueError, match="clearance"):
        verify_layout_hulls(narrow, [(np.zeros(3), yaw), (side * 0.75, yaw)], 0.0005)
