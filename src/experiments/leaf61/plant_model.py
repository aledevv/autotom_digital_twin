"""Deterministic multi-leaf fixture and offline metrics (no Isaac imports)."""

import numpy as np

from model import rotation


def layout(count, kind="plant"):
    if kind == "contact-pair":
        # Same orientation; 4 mm lateral shift avoids identical coincident borders.
        return [(np.zeros(3), 0.0), (np.array([0.0, 0.004, -0.003]), 0.0)]
    if count not in (1, 5, 10, 20):
        raise ValueError("Expected 1, 5, 10 or 20 leaves")
    # Interleaved branches, 80 mm between neighbouring lamina centers.
    return [
        (np.array([0.0, (i // 3) * 0.08, (i % 3) * 0.12]), 0.0) for i in range(count)
    ]


def yaw_matrix(angle):
    co, si = np.cos(angle), np.sin(angle)
    return np.array([[co, -si, 0.0], [si, co, 0.0], [0.0, 0.0, 1.0]])


def local_poses(positions, quaternions, offset, angle):
    """World wxyz poses -> instance-local wxyz poses, including parent yaw."""
    positions = (positions - offset) @ yaw_matrix(angle)
    w, x, y, z = quaternions.T
    co, si = np.cos(angle / 2), np.sin(angle / 2)
    q = np.column_stack(
        (co * w + si * z, co * x + si * y, co * y - si * x, co * z - si * w)
    )
    return positions, q


def summary(values):
    a = np.asarray(values, dtype=float)
    return {
        "count": len(a),
        "total_seconds": float(a.sum()),
        "median_seconds": float(np.median(a)) if len(a) else None,
        "p95_seconds": float(np.percentile(a, 95)) if len(a) else None,
    }


def box_overlap(amin, amax, bmin, bmax, clearance=0.0):
    return bool(np.all(np.minimum(amax, bmax) - np.maximum(amin, bmin) > -clearance))


def verify_layout(points, placements, thickness):
    """Conservative AABB gate: no initial lamina overlap in independent layout."""
    bounds = []
    for offset, yaw in placements:
        p = points @ yaw_matrix(yaw).T + offset
        low, high = p.min(0), p.max(0)
        low[2] -= thickness / 2
        high[2] += thickness / 2
        for a, b in bounds:
            if box_overlap(low, high, a, b, 0.001):
                raise ValueError("Initial lamina bounds overlap")
        bounds.append((low, high))
    return bounds


def visual_crossing(upper, lower, faces):
    """Vertical surface ordering at vertices, for near-horizontal pair diagnostics."""
    tri = lower[faces]
    worst = 0.0
    # Chunked vertex/triangle barycentric queries; avoids a dense all-run array.
    a, u, v = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    safe = np.where(abs(det) > 1e-15, det, 1.0)
    for pts in np.array_split(upper, max(1, len(upper) // 64)):
        q = pts[:, None, :2] - a[None, :, :2]
        s = (q[:, :, 0] * v[:, 1] - q[:, :, 1] * v[:, 0]) / safe
        t = (u[:, 0] * q[:, :, 1] - u[:, 1] * q[:, :, 0]) / safe
        inside = (s >= 0) & (t >= 0) & (s + t <= 1) & (abs(det) > 1e-15)
        heights = a[:, 2] + s * u[:, 2] + t * v[:, 2]
        if inside.any():
            worst = max(
                worst,
                float(np.max(np.where(inside, heights - pts[:, None, 2], -np.inf))),
            )
    return worst


def collider_vertices(polygons, positions, quaternions, thickness):
    from model import prism_mesh

    return [
        prism_mesh(poly, thickness)[0] @ rot.T + pos
        for poly, pos, rot in zip(
            polygons[1:], positions[1:], rotation(quaternions[1:])
        )
    ]


def convex_hull_xy(points):
    """CCW hull without duplicate endpoints, independent of SciPy."""
    points = sorted(set(map(tuple, np.asarray(points)[:, :2])))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    parts = []
    for sequence in (points, points[::-1]):
        hull = []
        for point in sequence:
            while len(hull) >= 2 and cross(hull[-2], hull[-1], point) <= 0:
                hull.pop()
            hull.append(point)
        parts.extend(hull[:-1])
    if len(parts) < 3:
        raise ValueError("Degenerate leaf hull")
    return np.asarray(parts)


def verify_layout_hulls(points, placements, thickness):
    """Conservative convex-prism SAT gate with the same 1 mm clearance."""
    hull = convex_hull_xy(points)
    bounds = []
    footprints = []
    for offset, yaw in placements:
        footprint = hull @ yaw_matrix(yaw)[:2, :2].T + offset[:2]
        low = np.r_[footprint.min(0), points[:, 2].min() + offset[2] - thickness / 2]
        high = np.r_[footprint.max(0), points[:, 2].max() + offset[2] + thickness / 2]
        for previous, (a, b) in zip(footprints, bounds):
            if not box_overlap(low, high, a, b, 0.001):
                continue
            separated = False
            for polygon in (footprint, previous):
                edges = np.roll(polygon, -1, axis=0) - polygon
                axes = np.column_stack((-edges[:, 1], edges[:, 0]))
                axes /= np.linalg.norm(axes, axis=1)[:, None]
                pa, pb = footprint @ axes.T, previous @ axes.T
                if np.any(
                    (pa.max(0) + 0.001 < pb.min(0)) | (pb.max(0) + 0.001 < pa.min(0))
                ):
                    separated = True
                    break
            if not separated:
                raise ValueError(
                    "Initial lamina convex hulls overlap or lack clearance"
                )
        bounds.append((low, high))
        footprints.append(footprint)
    return bounds
