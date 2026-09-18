"""Curved collider slicing preserves native geometry and mass integration."""

import itertools
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from soft_leaf import setup as soft_setup
    from soft_leaf import strip_mesh

    from model import Config, area, rectangle, skin
finally:
    sys.path.pop(0)


def test_curved_slices_partition_area_without_flattening():
    c = Config(height=0)
    points, faces = rectangle(c)
    points[:, 2] = 0.008 * np.sin(np.pi * points[:, 0] / c.length)
    centers, _, _ = soft_setup(points, c)
    boundaries = np.r_[0, np.linspace(c.fixed_length, c.length, 8)]
    areas = []
    for i, (lo, hi) in enumerate(itertools.pairwise(boundaries)):
        vertices, triangles, value = strip_mesh(
            points, faces, lo, hi, centers[i], c.thickness
        )
        assert np.isfinite(vertices).all()
        assert triangles.max() < len(vertices)
        assert np.ptp(vertices[:, 2]) > c.thickness
        areas.append(value)
    np.testing.assert_allclose(sum(areas), area(points, faces).sum(), rtol=1e-10)


def test_seven_segment_bind_preserves_curvature_and_base_hinge():
    c = Config(height=0, fixed_length=0)
    points, _ = rectangle(c)
    points[:, 2] = -0.009 * (points[:, 0] / c.length) ** 2
    centers, indices, weights = soft_setup(points, c)
    assert len(centers) == 8
    assert np.all(indices[points[:, 0] <= centers[1, 0], 0] == 1)
    np.testing.assert_allclose(weights.sum(axis=1), 1)
    np.testing.assert_allclose(
        skin(points, centers, indices, weights, centers, np.tile([1, 0, 0, 0], (8, 1))),
        points,
        atol=1e-12,
    )


def test_base_point_stays_attached_when_entire_lamina_rotates():
    c = Config(height=0, fixed_length=0)
    points, _ = rectangle(c)
    centers, indices, weights = soft_setup(points, c)
    theta = 0.7
    r = np.array(
        [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ]
    )
    result = skin(
        points,
        centers,
        indices,
        weights,
        centers @ r.T,
        np.tile([np.cos(theta / 2), 0, np.sin(theta / 2), 0], (8, 1)),
    )
    np.testing.assert_allclose(result, points @ r.T, atol=1e-12)
    base = np.argmin(np.linalg.norm(points, axis=1))
    np.testing.assert_allclose(result[base], 0, atol=1e-12)
