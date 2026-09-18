"""A moving branch must not be mistaken for deformation of its blade."""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1]))
try:
    from full_plant_analysis import surfaces

    from model import Config, skin_setup
finally:
    sys.path.pop(0)


def test_native_host_rotation_preserves_curved_surface_and_relative_measurement():
    c = Config(height=0)
    points = np.array(
        [
            [0, 0, 0],
            [0.003, 0.001, -0.0002],
            [0.02, -0.01, 0.002],
            [0.07, 0.01, -0.004],
            [0.09, 0, -0.01],
        ]
    )
    centers, indices, weights = skin_setup(points, c)
    offset = np.array([0.12, -0.04, 0.20])
    frame = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0.0]])
    # The blade frame is a pi rotation about (1,0,1). The host then turns 90 degrees about Z and translates.
    q = np.array([np.sqrt(0.5), 0, np.sqrt(0.5), 0])  # xyzw
    poses = np.zeros((2, 4, 7))
    poses[:, 0, 6] = 1
    poses[:, 1:, 3:] = q
    poses[0, 1:, :3] = centers[1:] @ frame.T + offset
    poses[1] = poses[0]
    rz = np.array([[0.0, -1, 0], [1, 0, 0], [0, 0, 1]])
    poses[1, :, :3] = poses[0, :, :3] @ rz.T + [1, 2, 3]
    poses[1, 0, 3:] = [0, 0, np.sqrt(0.5), np.sqrt(0.5)]
    poses[1, 1:, 3:] = [0.5, 0.5, 0.5, -0.5]
    row = {
        "ids": [1, 2, 3],
        "host_id": 0,
        "host_offset": centers[0] @ frame.T + offset,
        "host_frame": frame,
    }
    world, local = surfaces(row, (points, centers, indices, weights), poses)
    expected = points @ frame.T + offset
    np.testing.assert_allclose(world[0], expected, atol=1e-12)
    np.testing.assert_allclose(world[1], expected @ rz.T + [1, 2, 3], atol=1e-12)
    np.testing.assert_allclose(local[0], local[1], atol=1e-12)
