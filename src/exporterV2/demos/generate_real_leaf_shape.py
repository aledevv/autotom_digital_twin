from pathlib import Path

import numpy as np
import triangle as tr

from pxr import Usd, UsdGeom, Gf

from tomato_leaf_generator.shape.gaussian_efd import GaussianEFDShapeGenerator


LEAF_LENGTH = 0.06  # 6 cm
SEED = 42
ROLE = "left"


def main():
    # -------------------------------------------------------------------------
    # Paths
    # -------------------------------------------------------------------------

    repo_root = Path(__file__).resolve().parents[3]

    model_path = (
        repo_root
        / "external"
        / "real_leaves"
        / "src"
        / "tomato_leaf_generator"
        / "resources"
        / "gaussian_efd_d16"
    )

    output_usd = Path(__file__).resolve().parent / "generated_leaflet.usda"

    print("[1] Loading Gaussian leaf generator")
    print("    Model:", model_path)

    if not model_path.exists():
        raise FileNotFoundError(model_path)

    # -------------------------------------------------------------------------
    # Generate canonical 2D leaflet
    # -------------------------------------------------------------------------

    generator = GaussianEFDShapeGenerator(model_path)

    leaf = generator.generate(
        seed=SEED,
        role=ROLE,
    )

    contour = leaf.contour_xy.copy()

    print("[2] Leaf generated")
    print("    Role:", ROLE)
    print("    Contour:", contour.shape)
    print("    H60:", leaf.features_h60.shape)

    # -------------------------------------------------------------------------
    # Scale canonical shape to physical size
    #
    # Canonical base ≈ (0, 0)
    # Canonical tip  ≈ (0, 1)
    # -------------------------------------------------------------------------

    scaled = contour * LEAF_LENGTH

    print("[3] Scaled leaflet")
    print("    Length target:", LEAF_LENGTH, "m")
    print("    Bounds min:", scaled.min(axis=0))
    print("    Bounds max:", scaled.max(axis=0))

    # -------------------------------------------------------------------------
    # Triangulate the closed 2D polygon
    # -------------------------------------------------------------------------

    indices = np.arange(len(scaled))

    segments = np.column_stack([
        indices,
        np.roll(indices, -1),
    ])

    triangulation_input = {
        "vertices": scaled,
        "segments": segments,
    }

    result = tr.triangulate(triangulation_input, "p")

    if "vertices" not in result or "triangles" not in result:
        raise RuntimeError("Triangle failed to produce a valid triangulation")

    vertices_2d = result["vertices"]
    triangles = result["triangles"]

    print("[4] Triangulation complete")
    print("    Vertices:", vertices_2d.shape)
    print("    Triangles:", triangles.shape)

    # -------------------------------------------------------------------------
    # Convert 2D polygon into a flat 3D mesh
    # x -> leaflet lateral direction
    # y -> base-to-tip direction
    # z -> initially zero
    # -------------------------------------------------------------------------

    points = [
        Gf.Vec3f(float(x), float(y), 0.0)
        for x, y in vertices_2d
    ]

    face_vertex_counts = [3] * len(triangles)
    face_vertex_indices = triangles.flatten().tolist()

    # -------------------------------------------------------------------------
    # Author USD
    # -------------------------------------------------------------------------

    print("[5] Creating USD mesh")

    stage = Usd.Stage.CreateNew(str(output_usd))

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    UsdGeom.Xform.Define(stage, "/World")

    mesh = UsdGeom.Mesh.Define(stage, "/World/Leaf")

    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)

    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)

    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

    stage.GetRootLayer().Save()

    print("[OK] USD saved:")
    print("    ", output_usd)


if __name__ == "__main__":
    main()