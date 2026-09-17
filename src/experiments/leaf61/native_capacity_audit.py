"""Read-only native canopy coverage audit, run with a Python providing OpenUSD."""

import argparse
import json
from pathlib import Path

import numpy as np


def audit(source, runtime):
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(source))
    cache = UsdGeom.XformCache()
    selected = {
        row["path"].replace("/World/Canopy", "/World", 1)
        for row in runtime["canopy"]["selected"][: len(runtime["leaf_offsets_m"])]
    }
    rows = []
    for prim in stage.Traverse():
        if prim.GetName() != "LeafBlade":
            continue
        mesh = UsdGeom.Mesh(prim)
        if set(mesh.GetFaceVertexCountsAttr().Get()) != {3}:
            raise ValueError(f"Nontriangle native mesh: {prim.GetPath()}")
        transform = cache.GetLocalToWorldTransform(prim)
        points = np.array(
            [transform.Transform(Gf.Vec3d(v)) for v in mesh.GetPointsAttr().Get()]
        )
        faces = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        base, tip = [
            np.array(
                transform.Transform(
                    Gf.Vec3d(prim.GetAttribute("autotom:leafShape" + endpoint).Get())
                )
            )
            for endpoint in ("Base", "Tip")
        ]
        vector = tip - base
        length = float(np.linalg.norm(vector))
        area = float(
            np.linalg.norm(
                np.cross(
                    points[faces[:, 1]] - points[faces[:, 0]],
                    points[faces[:, 2]] - points[faces[:, 0]],
                ),
                axis=1,
            ).sum()
            / 2
        )
        path = str(prim.GetPath())
        rows.append(
            {
                "path": path,
                "represented_by_dynamic_candidate": path in selected,
                "primary_branch": "/LatLeaf_" not in path,
                "area_m2": area,
                "length_m": length,
                "inclination_deg": float(np.degrees(np.arcsin(vector[2] / length))),
                "planarity_rms_m": float(
                    np.linalg.svd(points - points.mean(0), compute_uv=False)[-1]
                    / np.sqrt(len(points))
                ),
                "vertices": len(points),
                "triangles": len(faces),
            }
        )
    rows.sort(key=lambda row: (-row["area_m2"], row["path"]))
    primary = [r for r in rows if r["primary_branch"]]
    return {
        "source": str(source.resolve()),
        "native_leaf_count": len(rows),
        "represented_count": sum(r["represented_by_dynamic_candidate"] for r in rows),
        "represented_area_fraction": sum(
            r["area_m2"] for r in rows if r["represented_by_dynamic_candidate"]
        )
        / sum(r["area_m2"] for r in rows),
        "largest_20_represented": sum(
            r["represented_by_dynamic_candidate"] for r in rows[:20]
        ),
        "primary_branch_leaves": len(primary),
        "primary_branch_represented": sum(
            r["represented_by_dynamic_candidate"] for r in primary
        ),
        "scope": "Coverage only. Candidate meshes and horizontal poses differ from native geometry; no native physics acceptance.",
        "leaves_by_native_area": rows,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("runtime", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Output already exists")
    result = audit(args.source, json.loads(args.runtime.read_text()))
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(
        json.dumps(
            {k: v for k, v in result.items() if k != "leaves_by_native_area"}, indent=2
        )
    )


if __name__ == "__main__":
    main()
