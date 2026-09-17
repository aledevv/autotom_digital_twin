"""Conservative full-time bounds for nearly identical skinned leaf trajectories.

For normalized nonnegative skin weights, vertex error is bounded by the largest
bone translation error plus ||R-Rref||_F times its maximum vertex radius.
These bounds skip no time samples and never replace a physical measurement.
"""

import numpy as np

from model import area, edges, rotation


def skin_error_bound(poses, reference, points, centers):
    bound = np.zeros(len(poses))
    for j in range(4):
        radius = np.linalg.norm(points - centers[j], axis=1).max()
        delta_r = rotation(poses[:, j, 3:]) - rotation(reference[:, j, 3:])
        error = np.linalg.norm(poses[:, j, :3] - reference[:, j, :3], axis=1)
        error += np.linalg.norm(delta_r, axis=(1, 2)) * radius
        bound = np.maximum(bound, error)
    return bound + 1e-12


def edge_error_bound(poses, reference, points, centers, indices, weights, faces):
    """Bound edge-vector error / rest length using LBS coefficient differences."""
    edge = edges(faces)
    length = np.linalg.norm(points[edge[:, 1]] - points[edge[:, 0]], axis=1)
    dense = np.zeros((len(points), 4))
    for column in range(indices.shape[1]):
        np.add.at(
            dense, (np.arange(len(points)), indices[:, column]), weights[:, column]
        )
    bound = np.zeros(len(poses))
    for j in range(4):
        weighted = dense[:, j, None] * (points - centers[j])
        coefficient = np.max(
            np.linalg.norm(weighted[edge[:, 1]] - weighted[edge[:, 0]], axis=1) / length
        )
        weight_change = np.max(
            abs(dense[edge[:, 1], j] - dense[edge[:, 0], j]) / length
        )
        dr = rotation(poses[:, j, 3:]) - rotation(reference[:, j, 3:])
        dt = poses[:, j, :3] - reference[:, j, :3]
        bound += coefficient * np.linalg.norm(
            dr, axis=(1, 2)
        ) + weight_change * np.linalg.norm(dt, axis=1)
    return bound + 1e-9


def metric_bounds(
    reference, error, times, baseline_index, points, faces, edge_error=None
):
    result = dict(reference)
    maximum = float(error.max())
    initial, baseline = float(error[0]), float(error[baseline_index])
    tail = float(error[times >= times[-1] - 1].max())
    edge = edges(faces)
    minimum_edge = np.linalg.norm(points[edge[:, 1]] - points[edge[:, 0]], axis=1).min()
    tri = points[faces]
    uv = np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1) + np.linalg.norm(
        tri[:, 2] - tri[:, 0], axis=1
    )
    area_error = (
        maximum * uv * (1 + reference["max_edge_extension"]) + 2 * maximum**2
    ) / area(points, faces)
    extension_error = 2 * maximum / minimum_edge
    if edge_error is not None:
        extension_error = min(extension_error, float(np.max(edge_error)))
        u = np.linalg.norm(tri[:, 1] - tri[:, 0], axis=1)
        v = np.linalg.norm(tri[:, 2] - tri[:, 0], axis=1)
        tighter = (
            u
            * v
            * (
                2 * extension_error * (1 + reference["max_edge_extension"])
                + extension_error**2
            )
            / (2 * area(points, faces))
        )
        area_error = np.minimum(area_error, tighter)
    result.update(
        max_deflection_from_equilibrium_m=reference["max_deflection_from_equilibrium_m"]
        + maximum
        + baseline,
        deflection_lower_bound_m=max(
            0, reference["max_deflection_from_equilibrium_m"] - maximum - baseline
        ),
        recovery_last_second_m=reference["recovery_last_second_m"] + tail + baseline,
        residual_motion_last_second_m=reference["residual_motion_last_second_m"]
        + 2 * np.sqrt(3) * tail,
        gravity_sag_m=reference["gravity_sag_m"] + initial + baseline,
        gravity_sag_lower_bound_m=reference["gravity_sag_m"] - initial - baseline,
        gravity_sag_upper_bound_m=reference["gravity_sag_m"] + initial + baseline,
        attachment_drift_m=reference["attachment_drift_m"] + initial + maximum,
        max_edge_extension=reference["max_edge_extension"] + extension_error,
        min_area_ratio=reference["min_area_ratio"] - float(area_error.max()),
        metrics_kind="conservative_full_rate_bounds",
        reference_leaf=reference["leaf"],
        maximum_vertex_error_bound_m=maximum,
    )
    return result
