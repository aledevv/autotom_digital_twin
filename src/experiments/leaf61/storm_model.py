"""Reduced rain momentum model, SI units; no droplets are rigid bodies."""

import numpy as np
from plant_model import yaw_matrix

from model import area, rotation


def rain_envelope(seconds):
    if seconds <= 0 or seconds >= 20:
        return 0.0
    return min(1.0, seconds / 8.0)


class RainImpulses:
    def __init__(
        self, points, faces, centers, offsets, yaws, fixed_length, length, seed=42
    ):
        self.points, self.faces, self.centers = points, faces, centers
        self.offsets = offsets
        self.yaw_rot = np.array([yaw_matrix(y) for y in yaws])
        self.count = len(offsets)
        self.rng = np.random.default_rng(seed)
        self.drop_diameter_m, self.speed_m_s = 0.003, 7.0
        self.mass_kg = 1000 * np.pi * self.drop_diameter_m**3 / 6
        tri_area = area(points, faces)
        x = points[faces].mean(1)[:, 0]
        ds = (length - fixed_length) / 3
        group = np.clip(((x - fixed_length) / ds).astype(int), 0, 2)
        self.triangles, self.cdfs, self.areas = [], [], []
        for j in range(3):
            ids = np.flatnonzero((group == j) & (x >= fixed_length))
            self.triangles.append(faces[ids])
            self.areas.append(float(tri_area[ids].sum()))
            self.cdfs.append(np.cumsum(tri_area[ids]) / tri_area[ids].sum())
        self.areas = np.array(self.areas)
        self.hits = np.zeros(self.count, dtype=np.int64)
        self.total_drops = 0
        self.expected_drops = 0.0
        self.impulse_ns = 0.0
        self.elapsed_rain_s = 0.0

    def sample(self, poses, rate_mm_h, dt, gain=1.0):
        """Poisson impacts from rainfall volume flux and projected surface area.

        Each leaf receives its own unoccluded rain column: a simultaneous-load
        test, not a water-conserving canopy interception/hydrodynamics model.
        Momentum transfer is full vertical stopping, no rebound or retained water.
        """
        local_rot = rotation(poses[:, 1:, 3:].reshape(-1, 4)).reshape(
            self.count, 3, 3, 3
        )
        world_rot = self.yaw_rot[:, None] @ local_rot
        projected = self.areas[None] * np.abs(world_rot[:, :, 2, 2])
        lam = rate_mm_h * 0.001 / 3600 * 1000 * projected * dt / self.mass_kg
        numbers = self.rng.poisson(lam)
        self.expected_drops += float(lam.sum())
        self.elapsed_rain_s += dt if rate_mm_h > 0 else 0
        self.hits += numbers.sum(axis=1)
        self.total_drops += int(numbers.sum())
        strength = self.mass_kg * self.speed_m_s * gain / dt
        forces = np.zeros((self.count, 3, 3), dtype=np.float32)
        torques = np.zeros_like(forces)
        forces[:, :, 2] = -numbers * strength
        for j in range(3):
            leaf = np.repeat(np.arange(self.count), numbers[:, j])
            if not len(leaf):
                continue
            tri = self.triangles[j][
                np.searchsorted(self.cdfs[j], self.rng.random(len(leaf)))
            ]
            u = np.sqrt(self.rng.random(len(leaf)))
            v = self.rng.random(len(leaf))
            weights = np.column_stack((1 - u, u * (1 - v), u * v))
            hit = np.einsum("ni,nij->nj", weights, self.points[tri])
            arm = np.einsum("nij,nj->ni", world_rot[leaf, j], hit - self.centers[j + 1])
            f = np.zeros_like(arm)
            f[:, 2] = -strength
            np.add.at(torques[:, j], leaf, np.cross(arm, f))
        self.impulse_ns += int(numbers.sum()) * self.mass_kg * self.speed_m_s * gain
        return forces, torques, np.flatnonzero(numbers.sum(axis=1))
