"""
usd_exporter.py  —  Plant USD Exporter V1
==========================================
Converts a PlantSnapshot (Python graph from CSV) into a USD file
ready for import in IsaacSim.

Design decisions (V1):
  - Internode  → UsdGeomCylinder  (length, radius = widthm/2)
  - Leaf       → UsdGeomCylinder (petiole) + UsdGeomMesh flat quad (each blade)
  - Fruits     → UsdGeomCylinder (pedicel) + UsdGeomSphere per fruit
  - Root       → UsdGeomSphere (tiny logical marker, r=0.005 m)

Coordinate system:
  - IsaacSim uses Z-up  →  the stem grows along +Z
  - Leaf angle (anglePetiole) is elevation from horizontal plane (XY)
  - CCW orientation is azimuth around Z

Known Issues addressed:
  [1] Z-up enforced via UsdGeom.SetStageUpAxis
  [2] Meters enforced via UsdGeom.SetStageMetersPerUnit(stage, 1.0)
  [3] UsdGeomCylinder used for internodes (analytical collider, no convex hull issue)
  [5] World-space transforms computed via matrix stack (numpy)
"""

import math
import numpy as np
from pxr import Usd, UsdGeom, Gf, Sdf

# ── internal imports (same package) ──────────────────────────────────────────
from .models import (
    PlantSnapshot, OrganNode, InternodeNode,
    LeafNode, FruitsNode, RootNode,
)

# ── constants from groIMP (Info.md) ──────────────────────────────────────────
INTERNODE_TRUSS_LENGTH   = 0.012   # m  — pedicel length for fruit truss
INTERNODE_TRUSS_DIAMETER = 0.0015  # m  — pedicel radius * 2
INTERNODE_TRUSS_ANGLE    = 9.0     # deg — truss insertion angle (from vertical)
PHYLLOTAXIS_ANGLE        = 137.5   # deg — golden angle between successive leaves
ROOT_SPHERE_RADIUS       = 0.005   # m  — visual marker for root
PHYLLOTAXIS_ANGLE        = 137.5   # aureus angle


# ─────────────────────────────────────────────────────────────────────────────
# Helpers: 4×4 matrix math (pure numpy, no pxr dependency in the math layer)
# ─────────────────────────────────────────────────────────────────────────────

def _identity() -> np.ndarray:
    return np.eye(4, dtype=float)


def _translate(tx: float, ty: float, tz: float) -> np.ndarray:
    """Translation matrix."""
    m = _identity()
    m[0, 3] = tx
    m[1, 3] = ty
    m[2, 3] = tz
    return m


def _rot_x(deg: float) -> np.ndarray:
    """Rotation around X axis."""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    m = _identity()
    m[1, 1] = c;  m[1, 2] = -s
    m[2, 1] = s;  m[2, 2] =  c
    return m


def _rot_z(deg: float) -> np.ndarray:
    """Rotation around Z axis."""
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    m = _identity()
    m[0, 0] = c;  m[0, 1] = -s
    m[1, 0] = s;  m[1, 1] =  c
    return m


def _mat_to_gf(m: np.ndarray) -> Gf.Matrix4d:
    """Convert a 4×4 numpy matrix to a Gf.Matrix4d (row-major)."""
    return Gf.Matrix4d(*m.T.flatten().tolist())


def _set_transform(xform_prim, world_mat: np.ndarray):
    """
    Assign a world-space 4×4 matrix to a USD Xform prim.
    We use a single xformOp:transform so that the matrix is stored directly
    without being decomposed into translate/rotate/scale — this avoids
    gimbal lock issues and is the safest approach for procedural geometry.
    """
    xformable = UsdGeom.Xformable(xform_prim)
    # Clear any previously set ops
    xformable.ClearXformOpOrder()
    op = xformable.AddTransformOp()
    op.Set(_mat_to_gf(world_mat))


# ─────────────────────────────────────────────────────────────────────────────
# USD prim creation helpers
# ─────────────────────────────────────────────────────────────────────────────

def _make_cylinder(stage, path: str, height: float, radius: float,
                   world_mat: np.ndarray):
    """
    Create a UsdGeomCylinder at *path*.

    Important: UsdGeomCylinder's default axis is 'Y', but we want it along Z
    (the direction the parent xform is already pointing after our transform).
    We set axis='Z' so the cylinder grows along +Z in its local frame.

    height and radius in meters.
    """
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetHeightAttr().Set(max(height, 1e-4))   # avoid zero-height degenerate
    cyl.GetRadiusAttr().Set(max(radius, 5e-5))   # avoid zero-radius degenerate
    cyl.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(cyl, world_mat)
    return cyl


def _make_sphere(stage, path: str, radius: float, world_mat: np.ndarray):
    sph = UsdGeom.Sphere.Define(stage, path)
    sph.GetRadiusAttr().Set(max(radius, 1e-4))
    _set_transform(sph, world_mat)
    return sph


def _make_flat_quad(stage, path: str, width: float, height: float,
                    world_mat: np.ndarray):
    """
    A single flat quad mesh representing one leaflet blade.
    Vertices lie in the XY plane (Z=0 in local space), centred at origin.

    The mesh is a simple 4-vertex quad, triangulated into 2 triangles.
    This satisfies the IsaacSim requirement of triangulated meshes (no quads).
    """
    hw = width  / 2.0
    hh = height / 2.0
    points = [
        Gf.Vec3f(-hw, -hh, 0),
        Gf.Vec3f( hw, -hh, 0),
        Gf.Vec3f( hw,  hh, 0),
        Gf.Vec3f(-hw,  hh, 0),
    ]
    # Two triangles: (0,1,2) and (0,2,3)
    face_vertex_counts = [3, 3]
    face_vertex_indices = [0, 1, 2,  0, 2, 3]

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set(face_vertex_counts)
    mesh.GetFaceVertexIndicesAttr().Set(face_vertex_indices)
    mesh.GetSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)  # no subdivision
    _set_transform(mesh, world_mat)
    return mesh


# ─────────────────────────────────────────────────────────────────────────────
# Pose computation:  where does each organ "start" in world space?
# ─────────────────────────────────────────────────────────────────────────────

def _build_world_transforms(snapshot: PlantSnapshot) -> dict:
    """
    Walk the organ tree (DFS from roots) and compute a 4×4 world-space matrix
    for the *base* of each organ (i.e. the point where it attaches to its parent).

    Convention (Z-up, stem grows along +Z):
      - Internode: placed at parent tip, extends along +Z by `length`.
        Its "tip" is at base + (0, 0, length).
      - Leaf:      placed at parent tip, rotated by azimuth (ccworientation)
        around Z then tilted by anglePetiole from horizontal.
      - Fruits:    placed at parent tip, tilted by trussAngle from vertical.
      - Root:      placed at world origin.

    Returns a dict  OrganKey → (world_matrix_4x4, tip_matrix_4x4)
    """
    transforms = {}   # OrganKey → (base_mat, tip_mat)

    # Find the main-stem root internode (parentrank == -1, order == 0)
    root_nodes = [n for n in snapshot.organs if n.parent_rank == -1]

    def _visit(node: OrganNode, parent_tip_mat: np.ndarray):
        """Recursively assign transforms, DFS."""

        if isinstance(node, RootNode):
            # Root: place a tiny sphere at world origin (below z=0)
            base_mat = _translate(0, 0, -0.02)  # 2 cm below ground
            tip_mat  = base_mat.copy()
            transforms[node.key] = (base_mat, tip_mat)

        elif isinstance(node, InternodeNode):
            # The cylinder is centred at its midpoint in USD, so we translate
            # the base to parent tip and then shift up by half the length so
            # the cylinder centre is at base + length/2.
            L = max(node.length, 1e-4)
            # base_mat = parent_tip
            base_mat = parent_tip_mat.copy()
            # The cylinder will be placed at the midpoint:
            mid_mat = parent_tip_mat @ _translate(0, 0, L / 2)
            tip_mat = parent_tip_mat @ _translate(0, 0, L)
            transforms[node.key] = (mid_mat, tip_mat)

        elif isinstance(node, LeafNode):
            # Start at parent tip, rotate azimuth around Z (ccworientation),
            # then tilt upward from horizontal by (90 - anglePetiole) around X.
            # anglePetiole=0° → horizontal, 90° → straight up.
            az   = node.ccw_orientation   # azimuth, degrees
            elev = node.angle_petiole     # elevation from horizontal, degrees
            L    = max(node.length_petiole, 1e-4)

            rot = parent_tip_mat @ _rot_z(az) @ _rot_x(elev - 90.0)
            base_mat = rot.copy()
            mid_mat  = rot @ _translate(0, 0, L / 2)
            tip_mat  = rot @ _translate(0, 0, L)
            transforms[node.key] = (mid_mat, tip_mat)

        elif isinstance(node, FruitsNode):
            # Place pedicel tilted by trussAngle from vertical (+Z axis),
            # then each fruit at the pedicel tip.
            az   = (node.key.rank * PHYLLOTAXIS_ANGLE) % 360
            tilt = INTERNODE_TRUSS_ANGLE   # degrees from vertical
            L    = INTERNODE_TRUSS_LENGTH

            rot = parent_tip_mat @ _rot_z(az) @ _rot_x(tilt)
            base_mat = rot.copy()
            mid_mat  = rot @ _translate(0, 0, L / 2)
            tip_mat  = rot @ _translate(0, 0, L)
            transforms[node.key] = (mid_mat, tip_mat)

        else:
            # Fallback for any unknown organ
            transforms[node.key] = (parent_tip_mat.copy(), parent_tip_mat.copy())
            tip_mat = parent_tip_mat

        # Recurse into children, passing the tip as the new parent origin
        _, tip = transforms[node.key]
        for child in node.children:
            _visit(child, tip)

    # Start DFS from root nodes
    origin = _identity()
    for root_node in root_nodes:
        if isinstance(root_node, RootNode):
            _visit(root_node, origin)          # Root: starts from origin, tip remains at z=-0.02
        elif isinstance(root_node, InternodeNode):
            _visit(root_node, origin)          # First internode with z=0

    return transforms


# ─────────────────────────────────────────────────────────────────────────────
# Main exporter
# ─────────────────────────────────────────────────────────────────────────────

def export_plant_usd(snapshot: PlantSnapshot, output_path: str) -> None:
    """
    Export a PlantSnapshot to a USD file at *output_path*.

    Parameters
    ----------
    snapshot    : PlantSnapshot object (from loader.py)
    output_path : e.g. "plant_day10.usda"   (.usda = ASCII, .usdc = binary)
    """

    # ── 1. Create stage ──────────────────────────────────────────────────────
    stage = Usd.Stage.CreateNew(output_path)

    # Fix [1]: enforce Z-up (IsaacSim default)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # Fix [2]: enforce meters (GroIMP already uses meters)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # ── 2. Create root Xform for this plant ──────────────────────────────────
    plant_path = f"/World/Plant_{snapshot.plant_id}"
    plant_xform = UsdGeom.Xform.Define(stage, plant_path)

    # Create group Xforms for readability in IsaacSim stage browser
    stem_path    = f"{plant_path}/Stem"
    leaves_path  = f"{plant_path}/Leaves"
    trusses_path = f"{plant_path}/Trusses"
    roots_path   = f"{plant_path}/Roots"
    for p in [stem_path, leaves_path, trusses_path, roots_path]:
        UsdGeom.Xform.Define(stage, p)

    # ── 3. Compute all world transforms (Fix [5]) ─────────────────────────────
    transforms = _build_world_transforms(snapshot)

    # ── 4. Emit geometry for each organ ──────────────────────────────────────
    for node in snapshot.organs:
        if node.key not in transforms:
            continue  # skip any organ the DFS didn't reach

        mid_mat, tip_mat = transforms[node.key]
        k = node.key
        organ_id = f"r{k.rank}_o{k.order}"  # e.g. r3_o0

        # ── Internode ────────────────────────────────────────────────────────
        if isinstance(node, InternodeNode):
            # Fix [3]: use native UsdGeomCylinder → analytical PhysX collider,
            # no convex hull issues on thin/long cylinders.
            path = f"{stem_path}/Internode_{organ_id}"
            _make_cylinder(
                stage, path,
                height=max(node.length, 1e-4),
                radius=max(node.width_m / 2, 5e-5),
                world_mat=mid_mat,
            )

        # ── Leaf ─────────────────────────────────────────────────────────────
        elif isinstance(node, LeafNode):
            leaf_group = f"{leaves_path}/Leaf_{organ_id}"
            UsdGeom.Xform.Define(stage, leaf_group)

            # Petiole cylinder
            petiole_path = f"{leaf_group}/Petiole"
            _make_cylinder(
                stage, petiole_path,
                height=max(node.length_petiole, 1e-4),
                radius=max(node.diameter_petiole / 2, 2e-4),
                world_mat=mid_mat,
            )

            # Leaflet blades — one flat quad per blade
            # Blade size: approximate square with total area divided equally
            if node.blades_nr > 0 and node.area_blades_total > 0:
                blade_area = node.area_blades_total / node.blades_nr
                blade_side = math.sqrt(blade_area)  # side of square approximation
            else:
                blade_side = 0.02  # fallback 2 cm

            for bi in range(node.blades_nr):
                # Fan blades along the rachis: distribute azimuth evenly
                blade_az = node.ccw_orientation + bi * (180.0 / max(node.blades_nr, 1))
                # Blades attach at the leaf tip, spread around the rachis direction
                blade_rot = tip_mat @ _rot_z(blade_az) @ _rot_x(-30.0)

                blade_path = f"{leaf_group}/Blade_{bi}"
                _make_flat_quad(
                    stage, blade_path,
                    width=blade_side,
                    height=blade_side,
                    world_mat=blade_rot,
                )

        # ── Fruits (truss) ────────────────────────────────────────────────────
        elif isinstance(node, FruitsNode):
            truss_group = f"{trusses_path}/Truss_{organ_id}"
            UsdGeom.Xform.Define(stage, truss_group)

            # Pedicel
            pedicel_path = f"{truss_group}/Pedicel"
            _make_cylinder(
                stage, pedicel_path,
                height=INTERNODE_TRUSS_LENGTH,
                radius=INTERNODE_TRUSS_DIAMETER / 2,
                world_mat=mid_mat,
            )

            # Individual fruits along the truss
            radii = node.fruit_radii if node.fruit_radii else [0.015] * node.fruit_nr
            angle_step = 30.0  # degrees between fruits along the truss
            for fi, r in enumerate(radii[:node.fruit_nr]):
                fruit_offset = tip_mat @ _translate(0, 0, fi * (2 * r + 0.005))
                fruit_path = f"{truss_group}/Fruit_{fi}"
                _make_sphere(stage, fruit_path, radius=max(r, 1e-3),
                             world_mat=fruit_offset)

        # ── Root ─────────────────────────────────────────────────────────────
        elif isinstance(node, RootNode):
            root_path = f"{roots_path}/Root_{organ_id}"
            base_mat, _ = transforms[node.key]
            _make_sphere(stage, root_path, radius=ROOT_SPHERE_RADIUS,
                         world_mat=base_mat)

    # ── 5. Save stage ─────────────────────────────────────────────────────────
    stage.GetRootLayer().Save()
    print(f"[USD Exporter] Saved: {output_path}")
    print(f"  Plant ID   : {snapshot.plant_id}")
    print(f"  Day        : {snapshot.day}")
    print(f"  Organs     : {len(snapshot.organs)}")
    print(f"  Up axis    : Z  (IsaacSim compatible)")
    print(f"  Units      : meters")
