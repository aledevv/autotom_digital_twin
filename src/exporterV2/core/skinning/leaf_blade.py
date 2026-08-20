"""Realistic 3D leaf blade generation for vegetative structures.

This implements the Merlice cultivar mathematical leaf width profile (Coussement et al. 2017)
and applies longitudinal midrib folding and static gravity sag.
"""

import math
from typing import Iterable
from pxr import Gf
from ..tree_config import PlantColors
from ..usd.materials import get_or_create_tomato_leaf_material
from .mesh import author_plain_mesh


LEAF_STATIONS = 18
LEAF_LENGTH_FRACTION = 5.35     # leaf_length / petiolule_length (0.075 / 0.014)
LEAF_HALF_WIDTH_FRACTION = 0.33 # leaf_half_width / leaf_length (0.025 / 0.075)

LEAF_LONGITUDINAL_FOLD_FRACTION = 0.064  # fold_depth / leaf_length (0.0048 / 0.075)
LEAF_FOLD_EXPONENT = 0.78
LEAF_ARCH_LIFT_FRACTION = 0.08  # arch_lift / leaf_length (0.0060 / 0.075)
LEAF_TIP_SAG_FRACTION = 0.133   # tip_sag / leaf_length (0.0100 / 0.075)
LEAF_TIP_SAG_EXPONENT = 1.85


def _normalized(vector: Gf.Vec3d) -> Gf.Vec3d:
    vector = Gf.Vec3d(vector)
    if vector.GetLength() <= 1e-10:
        raise ValueError("Cannot normalize a zero-length vector")
    vector.Normalize()
    return vector


def _merlice_leaflet_width(t: float) -> float:
    """Normalized leaflet width based on Coussement et al. (2017) Merlice model."""
    pos_norm = 1.0 - t  # 1.0 at root/base, 0.0 at tip
    max_w = 0.6         # Position of maximum width
    k1 = 2.0            # Curvature towards tip
    k2 = 2.2            # Curvature towards base

    if pos_norm <= max_w:
        return 1.0 - ((max_w - pos_norm) / max_w) ** k1
    else:
        return 1.0 - ((pos_norm - max_w) / (1.0 - max_w)) ** k2


def author_leaf_blade(
    stage,
    path: str,
    root: Gf.Vec3d,
    forward: Gf.Vec3d,
    *,
    length: float,
    half_width: float,
    fold_depth: float,
    arch_lift: float,
    tip_sag: float,
    color: tuple,
    world_to_link: Gf.Matrix4d,
) -> None:
    """2D blade with longitudinal midrib fold and one gentle gravity arch."""
    forward = _normalized(forward)
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    side = Gf.Cross(world_up, forward)
    if side.GetLength() <= 1e-8:
        side = Gf.Cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)
    sheet_normal = _normalized(Gf.Cross(forward, side))

    points = []
    for index in range(LEAF_STATIONS):
        t = index / float(LEAF_STATIONS - 1)
        width_profile = _merlice_leaflet_width(t)
        width = half_width * width_profile

        arch_offset = arch_lift * (4.0 * t * (1.0 - t))
        gravity_offset = tip_sag * t**LEAF_TIP_SAG_EXPONENT
        center = root + forward * (length * t) + world_up * (arch_offset - gravity_offset)

        edge_drop = fold_depth * math.sin(math.pi * t) ** LEAF_FOLD_EXPONENT

        p1 = center + side * width - sheet_normal * edge_drop
        p2 = center
        p3 = center - side * width - sheet_normal * edge_drop

        points.extend((
            Gf.Vec3f(*world_to_link.Transform(p1)),
            Gf.Vec3f(*world_to_link.Transform(p2)),
            Gf.Vec3f(*world_to_link.Transform(p3)),
        ))

    counts, indices = [], []
    for station in range(LEAF_STATIONS - 1):
        a, b = station * 3, (station + 1) * 3
        counts.extend((3, 3, 3, 3))
        indices.extend((
            a, a + 1, b + 1,
            a, b + 1, b,
            a + 1, a + 2, b + 2,
            a + 1, b + 2, b + 1
        ))

    material = get_or_create_tomato_leaf_material(stage)
    author_plain_mesh(
        stage,
        path,
        points,
        counts,
        indices,
        color,
        material=material,
    )


def author_petiolule_leaf_blades(stage, visual_axes: Iterable) -> None:
    """Find all petiolules and author a realistic leaf blade at their tip."""
    # Import locally to avoid circular dependencies
    from .mesh import _is_petiolule_axis, link_rest_world

    count = 0
    for axis in visual_axes:
        if not _is_petiolule_axis(axis):
            continue

        # The petiolule tip is at the end of the visual axis
        root = axis.start + axis.axis * axis.total_length
        forward = axis.axis

        # Calculate dynamic size based on petiolule length, but capped to realistic biological limits
        petiolule_length = axis.total_length
        leaf_length = min(0.09, max(0.04, petiolule_length * LEAF_LENGTH_FRACTION))

        half_width = leaf_length * LEAF_HALF_WIDTH_FRACTION
        fold_depth = leaf_length * LEAF_LONGITUDINAL_FOLD_FRACTION
        arch_lift = leaf_length * LEAF_ARCH_LIFT_FRACTION
        tip_sag = leaf_length * LEAF_TIP_SAG_FRACTION

        path = f"{axis.link_paths[-1]}/LeafBlade"

        world_to_link = link_rest_world(axis, -1).GetInverse()

        author_leaf_blade(
            stage,
            path,
            root,
            forward,
            length=leaf_length,
            half_width=half_width,
            fold_depth=fold_depth,
            arch_lift=arch_lift,
            tip_sag=tip_sag,
            color=PlantColors.LEAF_BLADE,
            world_to_link=world_to_link,
        )
        count += 1

    return count
