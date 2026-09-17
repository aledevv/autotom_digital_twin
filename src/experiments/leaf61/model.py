"""Shared SI fixture, contact geometry and acceptance; independent of Isaac."""

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class Config:
    hz: int = 120
    nx: int = 30
    ny: int = 14
    length: float = 0.09
    width: float = 0.04
    fixed_length: float = 0.006
    height: float = 0.15
    thickness: float = 0.0005
    density: float = 1000.0
    young: float = 5e6
    poisson: float = 0.3
    bend: float = 5e6
    damping: float = 0.01
    bend_damping: float = 0.01
    iterations: int = 64
    contact_offset: float = 0.0005
    rest_offset: float = 0.00025
    joint_stiffness: float = 0.024
    damping_ratio: float = 1.0
    radius: float = 0.01
    depth: float = 0.01
    settle: float = 8.0
    recovery: float = 8.0
    seed: int = 42

    def validate(self):
        for k, v in asdict(self).items():
            if not np.isfinite(v):
                raise ValueError(f"Non-finite {k}")
        if self.hz not in (60, 120, 240, 480, 960):
            raise ValueError("hz must be 60, 120, 240, 480 or 960")
        for k in ("nx", "ny", "iterations", "seed"):
            if not isinstance(getattr(self, k), int):
                raise TypeError(f"{k} must be an integer")
        for k in (
            "length",
            "width",
            "fixed_length",
            "height",
            "thickness",
            "density",
            "young",
            "bend",
            "joint_stiffness",
            "radius",
            "depth",
            "settle",
            "recovery",
            "damping_ratio",
        ):
            if getattr(self, k) <= 0:
                raise ValueError(f"{k} must be positive")
        if (
            self.nx < 4
            or self.ny < 4
            or self.iterations < 1
            or not 0 <= self.poisson < 0.5
        ):
            raise ValueError("Invalid mesh, iterations or Poisson ratio")
        if (
            not 0 <= self.rest_offset < self.contact_offset
            or min(self.damping, self.bend_damping) < 0
        ):
            raise ValueError("Invalid collision offsets or damping")
        if (
            not self.fixed_length < self.length / 3
            or self.recovery < 2
            or self.settle < 2
        ):
            raise ValueError("Invalid fixed region or observation window")


def rectangle(c):
    p = np.array(
        [
            [x, y, c.height]
            for x in np.linspace(0, c.length, c.nx + 1)
            for y in np.linspace(-c.width / 2, c.width / 2, c.ny + 1)
        ]
    )
    f = []
    for i in range(c.nx):
        for j in range(c.ny):
            a = i * (c.ny + 1) + j
            b = a + c.ny + 1
            f.extend(((a, b, a + 1), (a + 1, b, b + 1)))
    return p, np.asarray(f, dtype=np.int32)


def edges(f):
    return np.unique(
        np.sort(np.concatenate((f[:, [0, 1]], f[:, [1, 2]], f[:, [2, 0]])), axis=1),
        axis=0,
    )


def area(p, f):
    return (
        np.linalg.norm(
            np.cross(p[f[:, 1]] - p[f[:, 0]], p[f[:, 2]] - p[f[:, 0]]), axis=1
        )
        / 2
    )


def sphere_gap(p, f, center, radius):
    """Exact distance to triangle interiors and edges; no vertex-only contact proxy."""
    tri = p[f]
    a, b, d = tri[:, 0], tri[:, 1], tri[:, 2]
    u = b - a
    v = d - a
    w = center - a
    n = np.cross(u, v)
    nn = np.sum(n * n, axis=1)
    projected = center - n * (np.sum(w * n, axis=1) / np.maximum(nn, 1e-30))[:, None]
    q = projected - a
    uu = np.sum(u * u, 1)
    uv = np.sum(u * v, 1)
    vv = np.sum(v * v, 1)
    qu = np.sum(q * u, 1)
    qv = np.sum(q * v, 1)
    den = np.maximum(uu * vv - uv * uv, 1e-30)
    s = (qu * vv - qv * uv) / den
    t = (qv * uu - qu * uv) / den
    inside = (s >= 0) & (t >= 0) & (s + t <= 1) & (nn > 1e-24)
    dist = np.where(inside, np.linalg.norm(projected - center, axis=1), np.inf)
    for x, y in ((a, b), (b, d), (d, a)):
        e = y - x
        k = np.clip(
            np.sum((center - x) * e, 1) / np.maximum(np.sum(e * e, 1), 1e-30), 0, 1
        )
        dist = np.minimum(dist, np.linalg.norm(x + k[:, None] * e - center, axis=1))
    return float(dist.min() - radius)


def first_contact_height(p, f, x, y, radius):
    """First geometric sphere contact while descending from above the full surface."""
    top = float(p[:, 2].max() + radius + 0.02)
    previous = top
    for z in np.linspace(top, float(p[:, 2].min() - radius), 500):
        if sphere_gap(p, f, np.array([x, y, z]), radius) <= 0:
            lo, hi = z, previous
            for _ in range(35):
                mid = (lo + hi) / 2
                if sphere_gap(p, f, np.array([x, y, mid]), radius) <= 0:
                    lo = mid
                else:
                    hi = mid
            return (lo + hi) / 2
        previous = z
    raise ValueError("Probe misses the settled mesh; cannot validate contact")


def ramp(u):
    u = np.clip(u, 0, 1)
    return float(u * u * (3 - 2 * u))


def rotation(q):
    q = np.asarray(q)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q.T
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    ).transpose(2, 0, 1)


def skin_setup(p, c):
    ds = (c.length - c.fixed_length) / 3
    centers = np.array(
        [[(min(float(p[:, 0].min()), 0.0) + c.fixed_length) / 2, 0, c.height]]
        + [[c.fixed_length + (i + 0.5) * ds, 0, c.height] for i in range(3)]
    )
    # The root hinge transforms its anchor exactly: blending the support
    # through the first link would spuriously stretch the blade.
    u = np.interp(p[:, 0], centers[1:, 0], np.arange(1, 4))
    u[p[:, 0] <= c.fixed_length + 1e-9] = 0.0
    low = np.floor(u).astype(int)
    high = np.minimum(low + 1, 3)
    frac = u - low
    return centers, np.column_stack((low, high)), np.column_stack((1 - frac, frac))


def skin(p, centers, indices, weights, positions, quaternions):
    r = rotation(quaternions)
    local = p[:, None, :] - centers[indices]
    return np.sum(
        (np.einsum("vkij,vkj->vki", r[indices], local) + positions[indices])
        * weights[:, :, None],
        axis=1,
    )


def evaluate(c, rest, f, trace, times, probe, cycles, complete=True):
    trace = np.asarray(trace)
    times = np.asarray(times)
    finite = bool(np.isfinite(trace).all())
    if not finite:
        return {"status": "failed", "checks": {"finite": False}, "cycles": []}
    e = edges(f)
    initial = np.linalg.norm(rest[e[:, 1]] - rest[e[:, 0]], axis=1)
    ratios = np.linalg.norm(trace[:, e[:, 1]] - trace[:, e[:, 0]], axis=2) / initial
    fixed = rest[:, 0] <= c.fixed_length + 1e-7
    drift = float(np.linalg.norm(trace[:, fixed] - rest[None, fixed], axis=2).max())
    min_area = min(float((area(p, f) / area(rest, f)).min()) for p in trace)
    eq = trace[np.argmin(abs(times - (1 + c.settle)))]
    tip = rest[:, 0] > rest[:, 0].max() - 0.0031
    sag = float(np.mean(rest[tip, 2] - eq[tip, 2]))
    metrics = {
        "attachment_drift_m": drift,
        "max_edge_extension": float(ratios.max() - 1),
        "min_area_ratio": min_area,
        "sag_m": sag,
    }
    checks = {
        "complete": bool(complete),
        "finite": finite,
        "attachment": drift < 0.0002,
        "stretch": bool(ratios.max() < 1.05),
        "noncollapsed": min_area > 0.05,
        "sag": 0.00001 < sag < 0.3 * (c.length - c.fixed_length),
    }
    reports = []
    for cycle in cycles:
        end = cycle["end"]
        mask = (times > end - 1 - 1e-8) & (times <= end + 1e-8)
        baseline = trace[cycle["baseline_index"]]
        recovery = (
            float(np.linalg.norm(trace[mask] - baseline, axis=2).max())
            if mask.any()
            else float("inf")
        )
        oscillation = (
            float(np.linalg.norm(np.ptp(trace[mask], axis=0), axis=1).max())
            if mask.any()
            else float("inf")
        )
        ids = np.flatnonzero((times >= cycle["start"]) & (times <= cycle["start"] + 5))
        gaps = [sphere_gap(trace[i], f, probe[i], c.radius) for i in ids]
        deformation = max(
            float(np.linalg.norm(trace[i] - baseline, axis=1).max()) for i in ids
        )
        result = {
            "location": cycle["location"],
            "recovery_m": recovery,
            "residual_motion_m": oscillation,
            "min_sphere_gap_m": min(gaps),
            "deformation_m": deformation,
        }
        result["passed"] = bool(
            recovery < 0.002
            and oscillation < 0.0005
            and min(gaps) < 0.001
            and min(gaps) > -0.001
            and deformation > 0.001
        )
        reports.append(result)
    checks["cycles"] = bool(reports) and all(r["passed"] for r in reports)
    return {
        "status": "passed" if all(checks.values()) else "failed",
        "checks": checks,
        "metrics": metrics,
        "cycles": reports,
        "visual_acceptance": "pending",
        "biological_calibration": False,
    }


def collider_mismatch(c, points, positions, quaternions, polygons=None):
    """Largest visual-vertex distance to the nearest rigid collider boundary."""
    half = np.array(
        [[c.fixed_length / 2 + 0.00005, c.width / 2 + 0.001, 0.001]]
        + [[(c.length - c.fixed_length) / 6, c.width / 2, c.thickness / 2]] * 3
    )
    r = rotation(quaternions)
    local = np.einsum("bji,vbj->vbi", r, points[:, None] - positions[None])
    if polygons is not None:
        distances = [
            abs(prism_distance(local[:, i], poly, 0.002 if i == 0 else c.thickness))
            for i, poly in enumerate(polygons)
        ]
        return float(np.min(distances, axis=0).max())
    q = np.abs(local) - half[None]
    signed = np.linalg.norm(np.maximum(q, 0), axis=2) + np.minimum(q.max(axis=2), 0)
    return float(np.min(np.abs(signed), axis=1).max())


def material_target(rest, current, faces, x, y):
    """Follow the chosen material point after sag, keeping off-center probes comparable."""
    tri = rest[faces, :2]
    u = tri[:, 1] - tri[:, 0]
    v = tri[:, 2] - tri[:, 0]
    q = np.array([x, y]) - tri[:, 0]
    det = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
    valid = np.abs(det) > 1e-15
    safe = np.where(valid, det, 1.0)
    s = (q[:, 0] * v[:, 1] - q[:, 1] * v[:, 0]) / safe
    t = (u[:, 0] * q[:, 1] - u[:, 1] * q[:, 0]) / safe
    found = np.flatnonzero(valid & (s >= -1e-6) & (t >= -1e-6) & (s + t <= 1 + 1e-6))
    if not len(found):
        raise ValueError(
            "Requested material contact point lies outside the leaf contour"
        )
    i = found[0]
    return np.array([1 - s[i] - t[i], s[i], t[i]]) @ current[faces[i]]


def convex_strip(points, faces, lo, hi, center):
    """Convex 2-D collider footprint of a clipped strip, in its rigid frame."""
    vertices = []
    for face in faces:
        poly = list(points[face, :2])
        for bound, sign in ((lo, 1), (hi, -1)):
            clipped = []
            if not poly:
                break
            for a, b in zip(poly, poly[1:] + poly[:1]):
                ain = sign * (a[0] - bound) >= -1e-12
                bin_ = sign * (b[0] - bound) >= -1e-12
                if ain:
                    clipped.append(a)
                if ain != bin_:
                    clipped.append(a + (b - a) * ((bound - a[0]) / (b[0] - a[0])))
            poly = clipped
        vertices.extend(poly)
    pts = sorted(set(map(tuple, np.round(vertices, 12))))
    if len(pts) < 3:
        raise ValueError("Empty collider strip")

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    upper = []
    for pt in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], pt) <= 0:
            lower.pop()
        lower.append(pt)
    for pt in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], pt) <= 0:
            upper.pop()
        upper.append(pt)
    return np.asarray(lower[:-1] + upper[:-1]) - center[:2]


def prism_mesh(polygon, thickness):
    n = len(polygon)
    p = np.concatenate(
        (
            np.column_stack((polygon, np.full(n, -thickness / 2))),
            np.column_stack((polygon, np.full(n, thickness / 2))),
        )
    )
    faces = []
    for i in range(1, n - 1):
        faces.extend(((0, i + 1, i), (n, n + i, n + i + 1)))
    for i in range(n):
        j = (i + 1) % n
        faces.extend(((i, j, j + n), (i, j + n, i + n)))
    return p, np.asarray(faces)


def prism_distance(points, polygon, thickness):
    """Signed Euclidean distance to a convex extruded polygon."""
    a = polygon
    b = np.roll(polygon, -1, axis=0)
    e = b - a
    delta = points[:, None, :2] - a[None]
    u = np.clip(np.sum(delta * e[None], axis=2) / np.sum(e * e, axis=1), 0, 1)
    dist = np.linalg.norm(delta - u[:, :, None] * e[None], axis=2).min(axis=1)
    inside = np.all(
        e[None, :, 0] * delta[:, :, 1] - e[None, :, 1] * delta[:, :, 0] >= -1e-12,
        axis=1,
    )
    q = np.column_stack(
        (np.where(inside, -dist, dist), abs(points[:, 2]) - thickness / 2)
    )
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)
