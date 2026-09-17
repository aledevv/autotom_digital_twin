"""D0 geometry and diagnostics, independent of Isaac and USD (SI units)."""
from dataclasses import asdict, dataclass
from itertools import permutations

import numpy as np


@dataclass(frozen=True)
class Config:
    hz: int = 120
    length: float = 0.06
    width: float = 0.02
    thickness: float = 0.002
    nx: int = 30
    ny: int = 10
    nz: int = 2
    clamp_length: float = 0.006
    height: float = 0.05
    density: float = 1000.0
    young: float = 5e6
    poisson: float = 0.3
    velocity_damping: float = 2.0
    elasticity_damping: float = 0.005
    iterations: int = 128
    gravity_seconds: float = 8.0
    recovery_seconds: float = 8.0
    # Numerical screening thresholds, not biological acceptance criteria.
    attachment_tolerance: float = 0.0002
    recovery_tolerance: float = 0.001
    settling_span_tolerance: float = 0.0005

    def validate(self):
        values = asdict(self)
        if not all(np.isfinite(v) for v in values.values()):
            raise ValueError("Configuration must be finite")
        if any(values[k] <= 0 for k in values if k not in {
            "poisson", "velocity_damping", "elasticity_damping"
        }):
            raise ValueError("Dimensions, counts, durations and tolerances must be positive")
        if not 0 <= self.poisson < 0.49:
            raise ValueError("Use 0 <= Poisson ratio < 0.49 for this feasibility fixture")
        if min(self.velocity_damping, self.elasticity_damping) < 0:
            raise ValueError("Damping must be nonnegative")
        if not 0 < self.clamp_length < self.length:
            raise ValueError("Clamp must leave a free strip")
        for name in ("nx", "ny", "nz", "hz", "iterations"):
            if int(values[name]) != values[name]:
                raise ValueError(f"{name} must be an integer")
        if self.nz < 2:
            raise ValueError("D0 requires at least two cells through thickness")
        if min(self.gravity_seconds, self.recovery_seconds) < 2:
            raise ValueError("Each phase needs at least 2 seconds for diagnostics")
        if self.nx * self.ny * self.nz > 200_000:
            raise ValueError("Mesh exceeds isolated-fixture budget")


def signed_volumes(points, tets):
    p = np.asarray(points, dtype=np.float64)[tets]
    return np.einsum("ij,ij->i", np.cross(p[:, 1]-p[:, 0], p[:, 2]-p[:, 0]),
                     p[:, 3]-p[:, 0]) / 6.0


def make_mesh(c):
    """Conforming six-tet subdivision around each cell's 000–111 diagonal."""
    c.validate()
    axes = (np.linspace(0, c.length, c.nx+1),
            np.linspace(-c.width/2, c.width/2, c.ny+1),
            np.linspace(c.height-c.thickness/2, c.height+c.thickness/2, c.nz+1))
    points = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 3)
    shape = (c.nx+1, c.ny+1, c.nz+1)
    tets = []
    for cell in np.ndindex(c.nx, c.ny, c.nz):
        for order in permutations(range(3)):
            vertex = np.array(cell)
            ids = [np.ravel_multi_index(vertex, shape)]
            for axis in order:
                vertex = vertex.copy()
                vertex[axis] += 1
                ids.append(np.ravel_multi_index(vertex, shape))
            tets.append(ids)
    tets = np.asarray(tets, dtype=np.int32)
    negative = signed_volumes(points, tets) < 0
    tets[negative] = tets[negative][:, [0, 2, 1, 3]]
    faces = {}
    for a, b, d, e in tets:
        for face in ((b, d, e), (a, e, d), (a, b, e), (a, d, b)):
            key = tuple(sorted(face))
            if key in faces:
                del faces[key]
            else:
                faces[key] = face
    return points, tets, np.asarray(list(faces.values()), dtype=np.int32)


def summarize(c, times, nodes, tets, rest, step_seconds, sample_seconds, elapsed):
    """Use the loaded/rest topology; reject NaNs rather than hiding them."""
    times, nodes = np.asarray(times), np.asarray(nodes)
    finite = bool(np.isfinite(nodes).all())
    if not finite:
        return {"status": "failed", "checks": {"finite": False}}
    tip = np.isclose(rest[:, 0], c.length, atol=1e-7)
    center = np.isclose(rest[:, 0], c.length/2, atol=c.length/c.nx/2+1e-7)
    clamp = rest[:, 0] <= c.clamp_length+1e-7
    tip_xyz = nodes[:, tip].mean(axis=1)
    center_xyz = nodes[:, center].mean(axis=1)
    drift = np.linalg.norm(nodes[:, clamp]-rest[clamp], axis=-1).max()
    rest_v = signed_volumes(rest, tets)
    ratios = np.array([signed_volumes(p, tets)/rest_v for p in nodes])
    loaded = (times > c.gravity_seconds-1) & (times <= c.gravity_seconds+1e-8)
    recovered = times > c.gravity_seconds+c.recovery_seconds-1
    complete = bool(loaded.any() and recovered.any() and
                    times[-1] >= c.gravity_seconds+c.recovery_seconds-1/c.hz-1e-8)
    equilibrium = tip_xyz[loaded].mean(axis=0) if loaded.any() else tip_xyz[-1]
    tip_rest = rest[tip].mean(axis=0)
    recovery = float(np.max(np.linalg.norm(nodes[recovered]-rest, axis=-1))) if recovered.any() else float("inf")
    settling_span = float(np.ptp(tip_xyz[loaded], axis=0).max()) if loaded.any() else float("inf")
    max_displacement = float(np.linalg.norm(nodes-rest, axis=-1).max())
    # Windowed amplitude is diagnostic; the pass gate uses final settling/recovery.
    amplitudes = []
    for sec in range(int(np.floor(times[-1]))):
        window = (times >= sec) & (times < sec+1)
        if window.any():
            amplitudes.append(float(np.ptp(tip_xyz[window, 2])))
    checks = dict(finite=finite, complete=complete,
                  positive_tetrahedra=bool(ratios.min() > 0),
                  bounded_motion=max_displacement < 2*c.length,
                  attachment=bool(drift < c.attachment_tolerance),
                  measurable_sag=bool(tip_rest[2]-equilibrium[2] > 1e-5),
                  settled=settling_span < c.settling_span_tolerance,
                  elastic_recovery=recovery < c.recovery_tolerance)
    return dict(status="passed" if all(checks.values()) else "failed", checks=checks,
                tip_sag_m=float(tip_rest[2]-equilibrium[2]),
                max_attachment_drift_m=float(drift), min_volume_ratio=float(ratios.min()),
                max_displacement_m=max_displacement, recovery_error_m=recovery if np.isfinite(recovery) else None,
                loaded_tip_span_m=settling_span if np.isfinite(settling_span) else None, tip_z_span_by_second_m=amplitudes,
                simulated_seconds=float(times[-1]), simulation_wall_seconds=elapsed,
                instrumented_real_time_factor=float(times[-1]/elapsed),
                mean_step_ms=1000*float(np.mean(step_seconds)) if len(step_seconds) else None,
                mean_sampling_ms=1000*float(np.mean(sample_seconds)) if len(sample_seconds) else None,
                timing_note="Step includes CPU submission/GPU work as synchronized by sampling; not a pure GPU kernel benchmark.",
                manual_review="pending", center_final_world_m=center_xyz[-1].tolist())
