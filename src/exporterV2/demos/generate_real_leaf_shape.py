from pathlib import Path

import numpy as np
import triangle as tr

from pxr import Usd, UsdGeom, Gf

from tomato_leaf_generator.shape.factory import create_leaf_shape_generator

ROLE = "right"
SEED = 42

LEAF_LENGTH = 0.06

ARCH_LIFT = 0.004
TIP_SAG = 0.008
TIP_SAG_EXPONENT = 1.85

FOLD_DEPTH = 0.003
FOLD_EXPONENT = 0.8

GEN_MODE = "gaussian" # "i3/gaussian"

def main():
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

    output_usd = (
        Path(__file__).resolve().parent
        / "generated_leaflet_3d.usda"
    )

    # ------------------------------------------------------------------
    # Generate 2D leaflet
    # ------------------------------------------------------------------
    
    repo_root = Path(__file__).resolve().parents[3]
    
    resources_root = (
        repo_root
        / "external"
        / "real_leaves"
        / "src"
        / "tomato_leaf_generator"
        / "resources"
    )

    if GEN_MODE == "i3":
        shape_config = {
                "backend": "i3",
                "bank": "shape_bank.npz",
                "neighbours": 5,
                "dirichlet_alpha": 1.0,
                "maximum_raw_weight": 0.95,
                "maximum_expansion": 0.1,
                "variation_seed_offset": 90000,
                "maximum_weight_draws": 100,
                "maximum_geometry_attempts": 100,
            }
    elif GEN_MODE == "gaussian":
        shape_config = {
            # Gaussian
            "model": "gaussian_efd_d16",
    
            # I3
            "bank": "shape_bank.npz",
            "neighbours": 5,
            "dirichlet_alpha": 1.0,
            "maximum_raw_weight": 0.95,
            "maximum_expansion": 0.1,
            "variation_seed_offset": 90000,
            "maximum_weight_draws": 100,
            "maximum_geometry_attempts": 100,
        }
    else:
        print("Unknown generation mode, exiting")
        exit(1)

    

    generator = create_leaf_shape_generator(
        config=shape_config,
        root=resources_root,
        method=GEN_MODE,
    )

    leaf = generator.generate(
        seed=SEED,
        role=ROLE,
    )
    

    contour = leaf.contour_xy.copy()

    print("[1] Generated contour:", contour.shape)

    # ------------------------------------------------------------------
    # Scale to physical size
    # ------------------------------------------------------------------

    scaled = contour * LEAF_LENGTH

    # ------------------------------------------------------------------
    # Triangulate
    # ------------------------------------------------------------------

    indices = np.arange(len(scaled))

    segments = np.column_stack([
        indices,
        np.roll(indices, -1),
    ])

    result = tr.triangulate(
        {
            "vertices": scaled,
            "segments": segments,
        },
        "p",
    )

    vertices_2d = result["vertices"]
    triangles = result["triangles"]

    print("[2] Vertices:", vertices_2d.shape)
    print("[3] Triangles:", triangles.shape)

    # ------------------------------------------------------------------
    # Static 3D deformation
    # ------------------------------------------------------------------

    x = vertices_2d[:, 0]
    y = vertices_2d[:, 1]

    t = np.clip(
        y / LEAF_LENGTH,
        0.0,
        1.0,
    )

    # Longitudinal arch
    z_arch = ARCH_LIFT * 4.0 * t * (1.0 - t)

    # Tip sag
    z_sag = TIP_SAG * t**TIP_SAG_EXPONENT

    # Approximate central fold
    max_half_width = np.max(np.abs(x))

    if max_half_width > 1e-10:
        lateral = np.clip(
            np.abs(x) / max_half_width,
            0.0,
            1.0,
        )
    else:
        lateral = np.zeros_like(x)

    fold_profile = np.sin(np.pi * t) ** FOLD_EXPONENT

    z_fold = FOLD_DEPTH * lateral * fold_profile

    z = z_arch - z_sag - z_fold

    vertices_3d = np.column_stack([
        x,
        y,
        z,
    ])

    print(
        "[4] Z range:",
        vertices_3d[:, 2].min(),
        vertices_3d[:, 2].max(),
    )

    # ------------------------------------------------------------------
    # Create USD
    # ------------------------------------------------------------------

    stage = Usd.Stage.CreateNew(str(output_usd))

    UsdGeom.SetStageUpAxis(
        stage,
        UsdGeom.Tokens.z,
    )
    UsdGeom.SetStageMetersPerUnit(
        stage,
        1.0,
    )

    world = UsdGeom.Xform.Define(
        stage,
        "/World",
    )

    mesh = UsdGeom.Mesh.Define(
        stage,
        "/World/Leaf",
    )

    points = [
        Gf.Vec3f(
            float(px),
            float(py),
            float(pz),
        )
        for px, py, pz in vertices_3d
    ]

    mesh.GetPointsAttr().Set(points)

    mesh.GetFaceVertexCountsAttr().Set(
        [3] * len(triangles)
    )

    mesh.GetFaceVertexIndicesAttr().Set(
        triangles.flatten().tolist()
    )

    mesh.CreateOrientationAttr().Set(
        UsdGeom.Tokens.rightHanded
    )

    mesh.CreateSubdivisionSchemeAttr().Set(
        UsdGeom.Tokens.none
    )

    mesh.CreateDoubleSidedAttr().Set(True)

    mesh.CreateDisplayColorAttr().Set([
        Gf.Vec3f(0.15, 0.45, 0.10)
    ])

    stage.SetDefaultPrim(world.GetPrim())

    stage.GetRootLayer().Save()

    print("[OK] Saved:", output_usd)


if __name__ == "__main__":
    main()