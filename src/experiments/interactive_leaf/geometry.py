"""Small inextensible leaf skeleton and continuous linear-blend skin, SI units."""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Config:
    hz: int = 120
    links: int = 5
    length: float = 0.08
    width: float = 0.036
    thickness: float = 0.0008
    height: float = 0.16
    petiole_length: float = 0.022
    leaf_mass: float = 0.0006
    petiole_mass: float = 0.0015
    bend_stiffness: float = 0.024  # N m / rad, converted at USD boundary
    support_stiffness: float = 0.08
    damping_ratio: float = 0.9
    probe_radius: float = 0.008
    ball_radius: float = 0.006
    ball_mass: float = 0.001
    press_depth: float = 0.018
    target_fraction: float = 0.7
    target_y: float = 0.0

    def validate(self):
        if self.hz not in (60, 120, 240) or self.links not in range(3, 10):
            raise ValueError("Use 60/120/240 Hz and 3–9 links")
        if not 0 < self.damping_ratio <= 2 or not 0.2 < self.target_fraction < 0.95:
            raise ValueError("Invalid damping or contact position")
        for name in ("length", "width", "thickness", "height", "petiole_length", "leaf_mass",
                     "petiole_mass", "bend_stiffness", "support_stiffness", "probe_radius",
                     "ball_radius", "ball_mass", "press_depth"):
            v = getattr(self, name)
            if not np.isfinite(v) or v <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not np.isfinite(self.target_y) or abs(self.target_y) >= self.width/3 or self.press_depth > self.length/2:
            raise ValueError("Contact outside the fixture's supported envelope")


def half_width(s, c):
    s = np.asarray(s)
    return c.width/2 * np.maximum(np.sin(np.pi*s), 0)**0.7 * (1+0.045*np.sin(12*np.pi*s))


def geometry(c):
    c.validate()
    points, faces = [], []
    stations, across = 49, 9
    # Distinct narrow endpoints avoid degenerate triangles while retaining a tip.
    for i, s in enumerate(np.linspace(0, 1, stations)):
        width = max(float(half_width(s, c)), 0.00015)
        for v in np.linspace(-1, 1, across):
            points.append([c.petiole_length+c.length*s, width*v,
                           c.height+0.00065*np.sin(np.pi*s)*(1-v*v)])
    for i in range(stations-1):
        for j in range(across-1):
            a = i*across+j
            faces.extend([(a, a+across, a+1), (a+1, a+across, a+across+1)])
    points = np.asarray(points)
    centers = np.column_stack((c.petiole_length+(np.arange(c.links)+0.5)*c.length/c.links,
                               np.zeros(c.links), np.full(c.links, c.height)))
    u = np.clip((points[:, 0]-centers[0, 0])/(c.length/c.links), 0, c.links-1)
    low = np.floor(u).astype(int)
    high = np.minimum(low+1, c.links-1)
    fraction = u-low
    # Smoothstep blends preserve continuity around rigid-link boundaries.
    fraction = fraction*fraction*(3-2*fraction)
    indices = np.column_stack((low, high))
    weights = np.column_stack((1-fraction, fraction))
    return points, np.asarray(faces), centers, indices, weights


def rotations(quaternions):
    q = np.asarray(quaternions, dtype=float)
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    w, x, y, z = q.T
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]).transpose(2, 0, 1)


def skin(points, bind_centers, indices, weights, positions, quaternions):
    r = rotations(quaternions)
    local = points[:, None, :]-bind_centers[indices]
    transformed = np.einsum("vkij,vkj->vki", r[indices], local)+positions[indices]
    return np.sum(transformed*weights[:, :, None], axis=1)


def smooth_ramp(t, start, end):
    u = np.clip((t-start)/(end-start), 0, 1)
    return float(u*u*(3-2*u))


def press_fraction(t):
    if t < 4:
        return smooth_ramp(t, 2, 4)
    if t < 5:
        return 1.0
    return 1-smooth_ramp(t, 5, 7)
