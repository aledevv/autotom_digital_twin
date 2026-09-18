"""Preserve arbitrary native leaf frames, not only horizontal yaw fixtures."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from full_plant_skin import NativeSkinBatch, multiply_xyzw

    from model import rotation
finally:
    sys.path.pop(0)


def test_xyzw_product_preserves_arbitrary_rotations():
    rng = np.random.default_rng(42)
    a, b = rng.normal(size=(2, 20, 4))
    a /= np.linalg.norm(a, axis=1)[:, None]
    b /= np.linalg.norm(b, axis=1)[:, None]
    q = multiply_xyzw(a, b)
    np.testing.assert_allclose(
        rotation(q[:, [3, 0, 1, 2]]),
        rotation(a[:, [3, 0, 1, 2]]) @ rotation(b[:, [3, 0, 1, 2]]),
        atol=1e-14,
    )


def test_batch_uses_each_leafs_host_and_its_own_links():
    batch = NativeSkinBatch.__new__(NativeSkinBatch)
    batch.ids = np.array([[1, 2], [4, 5]])
    batch.hosts = np.array([0, 3])
    batch.offsets = np.array([[0.01, 0.02, 0.03], [-0.02, 0.01, 0.04]])
    batch.frames = np.array([[0, 0, 0, 1], [0, 1, 0, 0]])
    poses = np.zeros((6, 7))
    poses[:, :3] = np.arange(18).reshape(6, 3) / 10
    poses[:, 6] = 1
    poses[3, 3:] = [0, 0, np.sqrt(0.5), np.sqrt(0.5)]
    r = rotation(poses[:, [6, 3, 4, 5]])
    actual = batch.poses(poses, r)
    np.testing.assert_allclose(actual[:, 1:], poses[batch.ids], atol=1e-7)
    np.testing.assert_allclose(
        actual[:, 0, :3],
        poses[batch.hosts, :3] + np.einsum("lij,lj->li", r[batch.hosts], batch.offsets),
        atol=1e-7,
    )
    expected = r[batch.hosts] @ rotation(batch.frames[:, [3, 0, 1, 2]])
    np.testing.assert_allclose(
        rotation(actual[:, 0, [6, 3, 4, 5]]), expected, atol=2e-7
    )
