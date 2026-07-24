"""
geometry_utils.py

Shared geometry/material helpers that are genuinely identical between the
v1 pipeline (usd_exporter.py / usd_helpers.py) and the v2 pipeline
(plant_builder.py / usd_exporter_builder.py).

Only logic that was verified to be byte-for-byte equivalent between both
pipelines lives here. Anything with subtle differences (e.g. v1's leaf mesh
uses n_side=8 while v2's uses n_side=7 and different thickness handling) is
intentionally NOT unified — see REFACTOR_SPEC.md Task 1 for the rationale.
Duplicating a couple of small, clearly-marked functions is preferable to
risking a silent change in v1's numeric output.
"""

import math
import numpy as np
from pxr import UsdGeom, UsdShade, Gf, Sdf


# ─────────────────────────────────────────────────────────────────────────────
# Phyllotaxis / azimuth
# ─────────────────────────────────────────────────────────────────────────────

def phyllotaxis_azimuth_deg(rank: int, phyllotaxis_deg: float) -> float:
    """Cumulative phyllotaxis azimuth (rank * phyllotaxis_deg, wrapped to [0, 360)).

    Used by both v1 (usd_helpers._make_leaf, usd_exporter fruit truss azimuth)
    and v2 (usd_exporter_builder leaf/branch azimuth) as the fallback when the
    CSV does not provide an explicit orientation.
    """
    return (rank * phyllotaxis_deg) % 360.0


def resolve_azimuth_deg(explicit_deg: float, rank: int, phyllotaxis_deg: float,
                          epsilon: float = 1e-3) -> float:
    """If the CSV provides an explicit orientation (abs > epsilon), use it.
    Otherwise fall back to cumulative phyllotaxis (rank * phyllotaxis_deg).

    This is the shared decision logic used verbatim by v1's `_make_leaf` and
    v2's `attach_leaves` / `attach_lateral_branches` azimuth resolution.
    """
    if abs(explicit_deg) > epsilon:
        return explicit_deg
    return phyllotaxis_azimuth_deg(rank, phyllotaxis_deg)


# ─────────────────────────────────────────────────────────────────────────────
# world_base_z stacking (internode base-height accumulation)
# ─────────────────────────────────────────────────────────────────────────────

def compute_world_base_z(node, internode_type) -> float:
    """Recursively computes (and caches on the node as `.world_base_z`) the
    unscaled world Z coordinate of an internode's base, by stacking parent
    internode lengths.

    `internode_type` is passed in (rather than imported) to avoid a circular
    dependency between geometry_utils.py and models.py — both v1's
    `get_base_z` (usd_exporter.py) and v2's `_compute_world_base_z`
    (usd_exporter_builder.py) implement exactly this recursion.
    """
    if node is None or not isinstance(node, internode_type):
        return 0.0
    if hasattr(node, "world_base_z"):
        return node.world_base_z
    parent = node.parent
    if parent is not None and isinstance(parent, internode_type):
        z = compute_world_base_z(parent, internode_type) + parent.length
    else:
        z = 0.0
    node.world_base_z = z
    return z


# ─────────────────────────────────────────────────────────────────────────────
# Material creation/binding helpers
# ─────────────────────────────────────────────────────────────────────────────

def make_material(stage, path: str, color: tuple, roughness: float = 0.6, metallic: float = 0.0):
    """Creates a UsdPreviewSurface material with RGB color (0-1).

    Identical to v1's usd_helpers._make_material. v2 currently sets material
    color via UsdGeom DisplayColor directly instead of UsdShade materials, so
    it does not consume this helper today, but it is kept here as the single
    shared implementation for any future v2 code path that needs real
    UsdShade materials (e.g. matching v1's render-quality look).
    """
    mat = UsdShade.Material.Define(stage, path)

    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)

    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return mat


def bind_material(prim, mat):
    """Identical to v1's usd_helpers._bind_material."""
    if mat is None:
        return
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


# ─────────────────────────────────────────────────────────────────────────────
# Quaternion / 4x4-matrix conversion helpers
# ─────────────────────────────────────────────────────────────────────────────

def translate_matrix(tx: float, ty: float, tz: float) -> np.ndarray:
    """Identical to v1's usd_helpers._translate."""
    m = np.eye(4, dtype=float)
    m[0, 3] = tx
    m[1, 3] = ty
    m[2, 3] = tz
    return m


def mat_to_gf(m: np.ndarray) -> Gf.Matrix4d:
    """Identical to v1's usd_helpers._mat_to_gf.
    Gf.Matrix4d is column-major — transpose before flatten.
    """
    return Gf.Matrix4d(*m.T.flatten().tolist())


def set_transform(prim, mat: np.ndarray):
    """Identical to v1's usd_helpers._set_transform."""
    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(mat_to_gf(mat))


def align_z_to(dx: float, dy: float, dz: float,
               cx: float, cy: float, cz: float) -> np.ndarray:
    """Identical to v1's usd_helpers._align_z_to.

    Build a 4x4 matrix that places the origin at (cx,cy,cz)
    and rotates Z-axis to point along (dx,dy,dz).
    Uses Rodrigues rotation from (0,0,1) to (dx,dy,dz).

    v2 uses a completely different rotation representation (Gf.Rotation /
    quaternions composed directly, see plant_builder.py) so it does not
    consume this helper — it is shared here only for v1's own internal use
    (usd_helpers.py re-exports it unchanged for backward compatibility).
    """
    z = np.array([dx, dy, dz], dtype=float)
    norm = np.linalg.norm(z)
    if norm < 1e-9:
        z = np.array([0.0, 0.0, 1.0])
    else:
        z /= norm

    up = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(z, up)) > 0.999:
        up = np.array([1.0, 0.0, 0.0])

    x = np.cross(up, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)

    m = np.eye(4, dtype=float)
    m[:3, 0] = x
    m[:3, 1] = y
    m[:3, 2] = z
    m[0, 3] = cx
    m[1, 3] = cy
    m[2, 3] = cz
    return m
