"""Vectorized leaf pose conversion; no physics approximation."""

import numpy as np
from plant_model import yaw_matrix


class LocalPoseBatch:
    def __init__(self, offsets, yaws):
        self.offsets = np.asarray(offsets)[:, None, :]
        self.matrices = np.asarray([yaw_matrix(yaw) for yaw in yaws])
        self.co = np.cos(np.asarray(yaws) / 2)[:, None]
        self.si = np.sin(np.asarray(yaws) / 2)[:, None]
        self.co32, self.si32 = (
            self.co.astype((np.float64(1) * np.ones(1, dtype=np.float32)).dtype),
            self.si.astype((np.float64(1) * np.ones(1, dtype=np.float32)).dtype),
        )

    def __call__(self, positions, quaternions):
        count = len(self.offsets)
        output = np.empty((count, 4, 7))
        output[:, :, :3] = (
            positions.reshape(count, 4, 3) - self.offsets
        ) @ self.matrices
        w, x, y, z = np.moveaxis(quaternions.reshape(count, 4, 4), -1, 0)
        co, si = (
            (self.co32, self.si32)
            if quaternions.dtype == np.float32
            else (self.co, self.si)
        )
        output[:, :, 3] = co * w + si * z
        output[:, :, 4] = co * x + si * y
        output[:, :, 5] = co * y - si * x
        output[:, :, 6] = co * z - si * w
        return output
