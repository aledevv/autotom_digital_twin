"""Offline measurements in both world and moving native-petiole frames."""

import json
from pathlib import Path

import numpy as np

from model import rotation


def surfaces(row, mesh, poses):
    """Reconstruct original curved meshes, including a host-driven basal bone."""
    ids = row["ids"]
    h = row["host_id"]
    r = rotation(poses[..., [6, 3, 4, 5]].reshape(-1, 4)).reshape(
        *poses.shape[:2], 3, 3
    )
    hr = r[:, h]
    bp = np.concatenate(
        [
            (poses[:, h, :3] + np.einsum("tij,j->ti", hr, row["host_offset"]))[:, None],
            poses[:, ids, :3],
        ],
        axis=1,
    )
    br = np.concatenate(
        [(hr @ np.array(row["host_frame"]))[:, None], r[:, ids]], axis=1
    )
    points, centers, indices, weights = mesh
    local = points[:, None] - centers[indices]
    world = np.sum(
        (np.einsum("tvkij,vkj->tvki", br[:, indices], local) + bp[:, indices])
        * weights[None, :, :, None],
        axis=2,
    )
    host_local = np.einsum("tji,tvj->tvi", hr, world - poses[:, h, None, :3])
    return world, host_local


def analyze(run_dir):
    run_dir = Path(run_dir)
    rows = json.loads((run_dir / "layout.json").read_text())
    data = np.load(run_dir / "poses.npz")
    mesh_data = np.load(run_dir / "meshes.npz")
    times, poses = data["times"], data["poses"]
    if len(times) == 0:
        return []
    baseline = int(np.argmin(abs(times - 8)))
    results = []
    for i, row in enumerate(rows):
        mesh = [
            mesh_data[f"{key}_{i}"]
            for key in ("points", "centers", "indices", "weights")
        ]
        world, local = surfaces(row, mesh, poses)
        result = {"leaf": i, "source": row["source"], "mass_kg": row["mass_kg"]}
        points, centers, _indices, _weights = mesh
        faces = mesh_data[f"faces_{i}"]
        edges = np.unique(
            np.sort(
                np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]),
                axis=1,
            ),
            axis=0,
        )
        rest = np.linalg.norm(points[edges[:, 0]] - points[edges[:, 1]], axis=1)
        valid = rest > 1e-8
        lengths = np.linalg.norm(world[:, edges[:, 0]] - world[:, edges[:, 1]], axis=-1)
        result["stretch_max_fraction"] = float(
            np.max(lengths[:, valid] / rest[valid] - 1)
        )
        first = poses[:, row["ids"][0]]
        host = poses[:, row["host_id"]]
        first_r = rotation(first[:, [6, 3, 4, 5]])
        host_r = rotation(host[:, [6, 3, 4, 5]])
        delta = np.array([row["config"]["fixed_length"], 0, 0]) - centers[1]
        anchor1 = first[:, :3] + np.einsum("tij,j->ti", first_r, delta)
        anchor0 = host[:, :3] + np.einsum("tij,j->ti", host_r, row["anchor"])
        result["attachment_max_m"] = float(
            np.linalg.norm(anchor1 - anchor0, axis=1).max()
        )
        if times[-1] >= 18:
            final = times >= times[-1] - 1
            for name, points in [("world", world), ("petiole", local)]:
                result[name + "_peak_displacement_after_8s_m"] = float(
                    np.linalg.norm(points[baseline:] - points[baseline], axis=-1).max()
                )
                result[name + "_recovery_error_m"] = float(
                    np.linalg.norm(
                        points[final].mean(axis=0) - points[baseline], axis=-1
                    ).max()
                )
                result[name + "_residual_motion_m"] = float(
                    np.linalg.norm(np.ptp(points[final], axis=0), axis=-1).max()
                )
        results.append(result)
    (run_dir / "per_leaf.json").write_text(json.dumps(results, indent=2) + "\n")
    return results


if __name__ == "__main__":
    import sys

    analyze(sys.argv[1])
