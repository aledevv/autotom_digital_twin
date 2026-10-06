"""Check moving attachment frames, including translated and rotated branches."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from branch_layout import attachment_error
    from shared_stem import attachment_frames
finally:
    sys.path.pop(0)


def test_attachment_error_uses_rotating_branch_frame_and_world_anchor():
    shifts = np.array([[0.0, 0, 0], [0.3, 0.38, 0]])
    poses = np.array(
        [
            [0, 0.14, 0.2, 0, 0, 0, 1],
            [0.3, 0.38, 0.34, np.sqrt(0.5), 0, 0, np.sqrt(0.5)],
        ]
    )
    local = np.tile([[0, -0.08, 0], [0, 0, 0], [0, 0.08, 0]], (2, 1)).astype(float)
    petioles = np.array(
        [
            [0, 0.06, 0.2],
            [0, 0.14, 0.2],
            [0, 0.22, 0.2],
            [0.3, 0.38, 0.26],
            [0.3, 0.38, 0.34],
            [0.3, 0.38, 0.42],
        ]
    )
    assert attachment_error(poses, petioles, shifts, local) < 1e-12
    petioles[4, 0] += 0.0003
    assert np.isclose(attachment_error(poses, petioles, shifts, local), 0.0003)
    petioles[4, 0] -= 0.0003
    poses[1, 0] += 0.001
    petioles[3:, 0] += 0.001
    assert np.isclose(attachment_error(poses, petioles, shifts, local), 0.001)


def test_shared_stem_frames_rotate_and_detect_base_drift():
    roots_local = np.array([[0, 0, -0.05], [0, 0, 0.07], [0, 0, 0.19]])
    stem = np.array([0, -0.25, 0, np.sqrt(0.5), 0, 0, np.sqrt(0.5)])
    roots, error = attachment_frames(stem, roots_local)
    np.testing.assert_allclose(
        roots, [[0, -0.2, 0], [0, -0.32, 0], [0, -0.44, 0]], atol=1e-14
    )
    assert error < 1e-14
    stem[0] += 0.001
    shifted, error = attachment_frames(stem, roots_local)
    np.testing.assert_allclose(shifted, roots + [0.001, 0, 0], atol=1e-14)
    assert np.isclose(error, 0.001)


def test_attachment_check_accepts_moving_parent_roots():
    roots = np.array([[0.1, 0.2, 0.3]])
    poses = np.array([[0.1, 0.34, 0.3, 0, 0, 0, 1]])
    local = np.array([[0, -0.08, 0], [0, 0, 0], [0, 0.08, 0]])
    petioles = poses[:, :3] + local
    assert attachment_error(poses, petioles, np.zeros((1, 3)), local, roots) < 1e-14
    assert attachment_error(poses, petioles, np.zeros((1, 3)), local) > 0.1
