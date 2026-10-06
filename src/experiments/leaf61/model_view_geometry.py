"""Surface-aligned pivots and connected poses for the illustrative leaf viewer."""

import numpy as np


def surface_pivots(points, faces, xs):
    """Intersect the triangulated rest surface with the longitudinal centreline."""
    triangles = points[faces]
    a = triangles[:, 0, :2]
    b = triangles[:, 1, :2] - a
    c = triangles[:, 2, :2] - a
    det = b[:, 0] * c[:, 1] - b[:, 1] * c[:, 0]
    valid = abs(det) > 1e-14
    result = []
    for x in xs:
        d = [x, 0] - a
        u, v = np.full(len(faces), -99.0), np.full(len(faces), -99.0)
        u[valid] = (d[valid, 0] * c[valid, 1] - d[valid, 1] * c[valid, 0]) / det[valid]
        v[valid] = (b[valid, 0] * d[valid, 1] - b[valid, 1] * d[valid, 0]) / det[valid]
        hit = np.flatnonzero((u >= -1e-8) & (v >= -1e-8) & (u + v <= 1 + 1e-8))
        if not len(hit):
            raise ValueError(f"No leaf surface at centreline x={x}")
        k = hit[0]
        z = (
            triangles[k, 0, 2] * (1 - u[k] - v[k])
            + triangles[k, 1, 2] * u[k]
            + triangles[k, 2, 2] * v[k]
        )
        result.append([x, 0, z])
    return np.asarray(result)


def connected_poses(rotations, centers, pivots):
    """Retarget orientations to new pivots; both bodies share every joint anchor."""
    positions = np.empty(rotations.shape[:-2] + (3,))
    for j in range(len(pivots)):
        anchor = (
            pivots[j]
            if j == 0
            else positions[..., j - 1, :]
            + np.einsum(
                "...ij,j->...i", rotations[..., j - 1, :, :], pivots[j] - centers[j]
            )
        )
        positions[..., j, :] = anchor - np.einsum(
            "...ij,j->...i", rotations[..., j, :, :], pivots[j] - centers[j + 1]
        )
    return positions
