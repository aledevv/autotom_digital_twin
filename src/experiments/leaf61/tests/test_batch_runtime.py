"""Batch conversion must preserve the scalar path on either NumPy promotion policy."""

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT))
try:
    from batch_runtime import LocalPoseBatch
    from plant_model import local_poses
finally:
    sys.path.pop(0)


def test_batch_matches_scalar_for_float32_and_float64():
    rng = np.random.default_rng(41)
    offsets = rng.normal(size=(120, 3))
    yaws = rng.uniform(-np.pi, np.pi, 120)
    converter = LocalPoseBatch(offsets, yaws)
    for dtype in (np.float32, np.float64):
        p = rng.normal(size=(480, 3)).astype(dtype)
        q = rng.normal(size=(480, 4)).astype(dtype)
        p_copy, q_copy = p.copy(), q.copy()
        actual = converter(p, q)
        expected = np.empty_like(actual)
        for i in range(120):
            expected[i, :, :3], expected[i, :, 3:] = local_poses(
                p[4 * i : 4 * i + 4], q[4 * i : 4 * i + 4], offsets[i], yaws[i]
            )
        np.testing.assert_array_equal(actual, expected)
        np.testing.assert_array_equal(p, p_copy)
        np.testing.assert_array_equal(q, q_copy)
