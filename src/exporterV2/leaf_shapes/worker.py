"""Batch shape generation in Python 3.12. Input/output JSON, diagnostics on stderr."""
from pathlib import Path
import hashlib
import json
import platform
import sys
import time

import numpy as np
import triangle

# The sole bootstrap for the external source package. Do not install its unrelated pipeline dependencies.
SOURCE = Path(__file__).resolve().parents[3] / "external/real_leaves/src"
sys.path.insert(0, str(SOURCE))
from tomato_leaf_generator.shape.factory import create_leaf_shape_generator


def triangulate_contour(contour):
    # LeafShape2D is immutable; Triangle's Cython binding requires a writable buffer.
    contour = np.array(contour, dtype=np.float64, copy=True)
    n = len(contour)
    if contour.shape != (n, 2) or n < 3 or not np.isfinite(contour).all():
        raise ValueError("invalid canonical contour")
    indices = np.arange(n)
    segments = np.column_stack((indices, np.roll(indices, -1)))
    result = triangle.triangulate({"vertices": contour, "segments": segments}, "p")
    if "vertices" not in result or "triangles" not in result:
        raise ValueError("Triangle did not return vertices and triangles")
    vertices = np.asarray(result["vertices"], dtype=np.float64)
    faces = np.asarray(result["triangles"], dtype=np.int64)
    if vertices.ndim != 2 or vertices.shape[1] != 2 or not np.isfinite(vertices).all():
        raise ValueError("invalid triangulation vertices")
    if faces.ndim != 2 or faces.shape[1] != 3 or not len(faces):
        raise ValueError("invalid triangle array")
    if faces.min() < 0 or faces.max() >= len(vertices):
        raise ValueError("triangle index out of bounds")
    if len(vertices) != n or not np.array_equal(vertices, contour):
        raise ValueError("triangulation altered the boundary or inserted vertices")
    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    signed = (b[:, 0]-a[:, 0])*(c[:, 1]-a[:, 1]) - (b[:, 1]-a[:, 1])*(c[:, 0]-a[:, 0])
    if np.any(signed <= 0):
        raise ValueError("degenerate or reversed triangle")
    polygon_area = 0.5 * np.sum(contour[:, 0]*np.roll(contour[:, 1], -1) - contour[:, 1]*np.roll(contour[:, 0], -1))
    if not np.isclose(signed.sum()*0.5, polygon_area, rtol=1e-9, atol=1e-12):
        raise ValueError("triangulation does not cover the polygon")
    edges = {tuple(sorted((int(face[i]), int(face[(i+1)%3])))) for face in faces for i in range(3)}
    if any(tuple(sorted(map(int, edge))) not in edges for edge in segments):
        raise ValueError("triangulation lost a boundary segment")
    return vertices, faces


def run(request):
    started = time.monotonic()
    generator = create_leaf_shape_generator(config=request["generator"], root=SOURCE / "tomato_leaf_generator/resources", method=request["backend"])
    shapes = {}
    for index, item in enumerate(request["leaves"]):
        try:
            leaf = generator.generate(seed=item["seed"], role=item["role"])
            vertices, faces = triangulate_contour(leaf.contour_xy)
            shapes[item["id"]] = {
                **item, "backend": request["backend"],
                "contour_xy": leaf.contour_xy.tolist(),
                "vertices": vertices.tolist(), "triangles": faces.tolist(),
                "base": leaf.biological_base_xy.tolist(), "tip": leaf.biological_tip_xy.tolist(),
                "provenance": dict(leaf.provenance),
                "contour_sha256": hashlib.sha256(np.asarray(leaf.contour_xy, dtype="<f8").tobytes()).hexdigest(),
            }
        except Exception as exc:
            raise ValueError(f"{request['backend']} leaflet {item['id']} role={item['role']} seed={item['seed']}: {exc}") from exc
        if index == 0 or (index+1) % 25 == 0 or index+1 == len(request["leaves"]):
            print(f"[leaf shapes] {index+1}/{len(request['leaves'])}", file=sys.stderr, flush=True)
    return {"schema": 1, "shapes": shapes, "runtime": {
        "python": platform.python_version(), "numpy": np.__version__, "triangle": triangle.__version__,
        "generation_seconds": time.monotonic()-started,
    }}


if __name__ == "__main__":
    try:
        result = run(json.loads(Path(sys.argv[1]).read_text()))
        Path(sys.argv[2]).write_text(json.dumps(result, allow_nan=False))
    except Exception as exc:
        print(f"[leaf shapes ERROR] {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
