"""Alternative smooth visual authoring modes for the vegetative backend.

The validated ``skinned`` mode lives in :mod:`mesh`.  This module contains
non-UsdSkel alternatives that deliberately reuse the same visual radius profile,
centerline sampling, taper and junction shaping so performance can be compared
without changing the plant's botanical geometry.
"""

import math

from pxr import Gf, UsdGeom, Vt

from .mesh import (
    _axis_color,
    _smoothstep,
    _visual_radius,
    build_axis_sample_arcs,
    build_axis_tube_data,
    build_parallel_transport_frames,
)
from .model import VisualAxisData


# Segmented realtime mode. Each physical link owns one rigid visual mesh.
_SEGMENT_TONGUE_MAX_M = 0.006
_SEGMENT_TONGUE_FRACTION = 0.18
_SEGMENT_TONGUE_START_SCALE = 0.90
_SEGMENT_TONGUE_END_SCALE = 0.75

# Optional terminal taper used only by ``segmented-fork`` structural hosts.
# The normal segmented mode remains unchanged when terminal_tip_scale is 1.0.
_TERMINAL_TAPER_MAX_M = 0.022
_TERMINAL_TAPER_LINK_FRACTION = 0.42

# A centered terminal petiole is the real continuation of a lateral branch in
# segmented-fork mode.  Its first visual segment therefore extends a few mm
# backwards into the parent terminal mesh.  This hides the open angled corner at
# the fork without moving the rigid body or joint.  The deepest ring is slightly
# narrower so the overlap reads as a rounded organic insertion, not a hard tube
# crossing through the parent.
_CENTERED_FORK_ROOT_OVERLAP_MAX_M = 0.006
_CENTERED_FORK_ROOT_OVERLAP_LINK_FRACTION = 0.16
_CENTERED_FORK_ROOT_START_SCALE = 0.72


def _author_plain_mesh(
    stage,
    mesh_path: str,
    axis: VisualAxisData,
    points,
    face_counts,
    face_indices,
) -> None:
    """Author a non-skinned mesh with the same display material as the tube."""
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(*_axis_color(axis))
    ]))


def _link_rest_world(axis: VisualAxisData, link_index: int) -> Gf.Matrix4d:
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(
        Gf.Rotation(Gf.Quatd(axis.link_orientations[link_index])),
        axis.link_bases[link_index],
    )
    return matrix


def author_static_visual_axis(stage, axis: VisualAxisData) -> None:
    """Author the exact smooth tube as a static world-space mesh, with no UsdSkel."""
    UsdGeom.Xform.Define(stage, axis.visual_root_path)
    points, face_counts, face_indices, _, _ = build_axis_tube_data(axis)
    _author_plain_mesh(
        stage,
        f"{axis.visual_root_path}/StaticMesh",
        axis,
        points,
        face_counts,
        face_indices,
    )


def author_rigid_visual_axis(stage, axis: VisualAxisData) -> None:
    """Attach a one-bone tube directly below its single PhysX rigid link."""
    if len(axis.link_paths) != 1:
        raise ValueError(
            f"Rigid visual axis '{axis.axis_id}' must have exactly one physics link"
        )

    world_points, face_counts, face_indices, _, _ = build_axis_tube_data(axis)
    world_to_link = _link_rest_world(axis, 0).GetInverse()
    local_points = [
        Gf.Vec3f(*world_to_link.Transform(Gf.Vec3d(*point)))
        for point in world_points
    ]

    _author_plain_mesh(
        stage,
        f"{axis.link_paths[0]}/VisualMesh",
        axis,
        local_points,
        face_counts,
        face_indices,
    )


def _segment_overlap(axis: VisualAxisData, link_index: int) -> float:
    """Return the forward visual tongue length at one internal physical joint."""
    if link_index >= len(axis.link_paths) - 1:
        return 0.0
    current_length = axis.bone_lengths[link_index]
    next_length = axis.bone_lengths[link_index + 1]
    return min(
        _SEGMENT_TONGUE_MAX_M,
        current_length * _SEGMENT_TONGUE_FRACTION,
        next_length * _SEGMENT_TONGUE_FRACTION,
    )


def _centered_fork_root_overlap(axis: VisualAxisData, link_index: int) -> float:
    """Return visual-only root penetration for a centered terminal leaf branch."""
    if link_index != 0:
        return 0.0
    if not bool(axis.definition.get("_terminal_fork_centered", False)):
        return 0.0

    link_length = axis.bone_lengths[0]
    return min(
        _CENTERED_FORK_ROOT_OVERLAP_MAX_M,
        link_length * _CENTERED_FORK_ROOT_OVERLAP_LINK_FRACTION,
    )


def _terminal_taper_start(
    axis: VisualAxisData,
    link_index: int,
    terminal_tip_scale: float,
):
    if terminal_tip_scale >= 0.999999:
        return None
    if link_index != len(axis.link_paths) - 1:
        return None

    link_length = axis.bone_lengths[link_index]
    taper_length = min(
        _TERMINAL_TAPER_MAX_M,
        link_length * _TERMINAL_TAPER_LINK_FRACTION,
    )
    if taper_length <= 1e-8:
        return None
    return max(axis.bone_starts[link_index], axis.total_length - taper_length)


def _segment_sample_arcs(
    axis: VisualAxisData,
    visual_start: float,
    core_start: float,
    core_end: float,
    tongue_end: float,
    *,
    terminal_taper_start=None,
):
    """Reuse global smooth-mesh samples and add local overlap/tip samples."""
    eps = 1e-10
    arcs = {
        arc
        for arc in build_axis_sample_arcs(axis)
        if visual_start - eps <= arc <= tongue_end + eps
    }
    arcs.add(visual_start)
    arcs.add(core_start)
    arcs.add(core_end)
    arcs.add(tongue_end)

    if visual_start < core_start - eps:
        root_overlap = core_start - visual_start
        arcs.add(visual_start + root_overlap * 0.25)
        arcs.add(visual_start + root_overlap * 0.50)
        arcs.add(visual_start + root_overlap * 0.75)

    if tongue_end > core_end + eps:
        overlap = tongue_end - core_end
        arcs.add(core_end + overlap / 3.0)
        arcs.add(core_end + 2.0 * overlap / 3.0)

    if terminal_taper_start is not None:
        taper_length = max(core_end - terminal_taper_start, 0.0)
        arcs.add(terminal_taper_start)
        arcs.add(terminal_taper_start + taper_length * 0.25)
        arcs.add(terminal_taper_start + taper_length * 0.50)
        arcs.add(terminal_taper_start + taper_length * 0.75)
        arcs.add(core_end)

    return sorted(arcs)


def _root_overlap_radius_scale(
    arc: float,
    visual_start: float,
    core_start: float,
) -> float:
    """Round the buried petiole root while it blends back to normal at arc=0."""
    if visual_start >= core_start - 1e-10 or arc >= core_start:
        return 1.0
    q = _smoothstep(
        (arc - visual_start) / max(core_start - visual_start, 1e-8)
    )
    return _CENTERED_FORK_ROOT_START_SCALE + (
        1.0 - _CENTERED_FORK_ROOT_START_SCALE
    ) * q


def _tongue_radius_scale(arc: float, core_end: float, tongue_end: float) -> float:
    """Taper the hidden overlap so it nests inside the following rigid segment."""
    if tongue_end <= core_end + 1e-10 or arc <= core_end + 1e-10:
        return 1.0

    q = (arc - core_end) / max(tongue_end - core_end, 1e-8)
    q = _smoothstep(q)
    return (
        _SEGMENT_TONGUE_START_SCALE
        + (_SEGMENT_TONGUE_END_SCALE - _SEGMENT_TONGUE_START_SCALE) * q
    )


def _terminal_radius_scale(
    arc: float,
    core_end: float,
    taper_start,
    terminal_tip_scale: float,
) -> float:
    if taper_start is None or arc <= taper_start:
        return 1.0
    q = _smoothstep(
        (arc - taper_start) / max(core_end - taper_start, 1e-8)
    )
    return 1.0 + (terminal_tip_scale - 1.0) * q


def _build_segmented_link_mesh(
    axis: VisualAxisData,
    link_index: int,
    *,
    terminal_tip_scale: float = 1.0,
):
    """Build one rigid piece of the organic tube in the link's local frame."""
    radial_segments = axis.profile.radial_segments
    if radial_segments < 3:
        raise ValueError(
            f"Visual axis '{axis.axis_id}' radial_segments must be at least 3"
        )
    if not 0.1 <= terminal_tip_scale <= 1.0:
        raise ValueError(
            f"terminal_tip_scale must be in [0.1, 1.0], got {terminal_tip_scale}"
        )

    core_start = axis.bone_starts[link_index]
    core_end = min(
        axis.total_length,
        core_start + axis.bone_lengths[link_index],
    )

    root_overlap = _centered_fork_root_overlap(axis, link_index)
    visual_start = core_start - root_overlap

    overlap = _segment_overlap(axis, link_index)
    tongue_end = min(axis.total_length, core_end + overlap)
    taper_start = _terminal_taper_start(
        axis,
        link_index,
        terminal_tip_scale,
    )
    arcs = _segment_sample_arcs(
        axis,
        visual_start,
        core_start,
        core_end,
        tongue_end,
        terminal_taper_start=taper_start,
    )

    normals, binormals = build_parallel_transport_frames(axis, len(arcs))
    world_to_link = _link_rest_world(axis, link_index).GetInverse()
    points = []

    for arc, normal, binormal in zip(arcs, normals, binormals):
        center = axis.start + axis.axis * arc
        radius = _visual_radius(axis, arc)
        radius *= _root_overlap_radius_scale(
            arc,
            visual_start,
            core_start,
        )
        radius *= _tongue_radius_scale(arc, core_end, tongue_end)
        radius *= _terminal_radius_scale(
            arc,
            core_end,
            taper_start,
            terminal_tip_scale,
        )

        for radial in range(radial_segments):
            theta = 2.0 * math.pi * radial / radial_segments
            world_point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            local_point = world_to_link.Transform(world_point)
            points.append(Gf.Vec3f(*local_point))

    face_counts = []
    face_indices = []
    for ring in range(len(arcs) - 1):
        row0 = ring * radial_segments
        row1 = (ring + 1) * radial_segments
        for radial in range(radial_segments):
            next_radial = (radial + 1) % radial_segments
            face_counts.extend((3, 3))
            face_indices.extend((
                row0 + radial,
                row1 + radial,
                row1 + next_radial,
                row0 + radial,
                row1 + next_radial,
                row0 + next_radial,
            ))

    return points, face_counts, face_indices, overlap


def author_segmented_visual_axis(
    stage,
    axis: VisualAxisData,
    *,
    terminal_tip_scale: float = 1.0,
) -> dict:
    """Author one organic rigid mesh per PhysX link, with no UsdSkel.

    ``terminal_tip_scale`` is normally 1.0.  The segmented-fork mode may pass a
    smaller value for an eligible structural host.  Centered terminal petioles
    also receive a small visual-only backwards root overlap so their angled
    insertion into a lateral branch has no exposed corner/gap.
    """
    segment_count = 0
    tongue_count = 0

    for link_index, link_path in enumerate(axis.link_paths):
        points, face_counts, face_indices, overlap = _build_segmented_link_mesh(
            axis,
            link_index,
            terminal_tip_scale=terminal_tip_scale,
        )
        _author_plain_mesh(
            stage,
            f"{link_path}/OrganicVisual_{link_index + 1:02d}",
            axis,
            points,
            face_counts,
            face_indices,
        )
        segment_count += 1
        if overlap > 0.0:
            tongue_count += 1

    return {
        "segments": segment_count,
        "tongues": tongue_count,
    }
