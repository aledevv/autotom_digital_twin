"""Check moving attachment frames, including translated and rotated branches."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from branch_layout import attachment_error
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
