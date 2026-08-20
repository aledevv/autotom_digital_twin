"""Centered visual bridge for the real organ arm of a terminal fork.

The existing petiole/truss remains physically untouched.  This short rigid mesh
starts on the *centerline* of the terminal structural branch and points toward
the real organ, visually filling the open socket caused by the legacy radial
attachment offset.  It is visual-only and parented to the terminal parent link.
"""

import math

from pxr import Gf, UsdGeom, Vt

from ..tree_config import PlantColors, scaled
from .mesh import _axis_color, _smoothstep, _visual_radius
from .model import VisualAxisData


BRIDGE_LENGTH_MIN_M = 0.010
BRIDGE_LENGTH_MAX_M = 0.020
BRIDGE_LENGTH_RADIUS_SCALE = 2.8
BRIDGE_ROOT_SCALE = 0.60
BRIDGE_TIP_SCALE = 0.42
RADIAL_SEGMENTS_MIN = 10
CURVE_SAMPLES = 7


def _normalized(vector):
    vector = Gf.Vec3d(vector)
    if vector.GetLength() <= 1e-10:
        raise ValueError("Cannot normalize zero-length fork bridge vector")
    vector.Normalize()
    return vector


def _child_axis(parent_member, child_def):
    tilt = float(child_def.get("tilt", 0.0))
    rot = float(child_def.get("rot", 0.0))
    roll = float(child_def.get("roll", 0.0))

    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot)
    rot_tilt = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt)
    rot_roll = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), roll)
    branch_in_parent = rot_roll * rot_tilt * rot_z
    parent_rotation = Gf.Rotation(Gf.Quatd(parent_member.orientation))
    combined = branch_in_parent * parent_rotation
    return _normalized(combined.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0)))


def _transport_frames(tangents):
    first = tangents[0]
    ref = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(Gf.Dot(first, ref)) > 0.92:
        ref = Gf.Vec3d(1.0, 0.0, 0.0)
    normal = _normalized(Gf.Cross(ref, first))
    binormal = _normalized(Gf.Cross(first, normal))
    return [normal] * len(tangents), [binormal] * len(tangents)


def _link_rest_world(axis: VisualAxisData):
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(
        Gf.Rotation(Gf.Quatd(axis.link_orientations[-1])),
        axis.link_bases[-1],
    )
    return matrix


def author_centered_existing_arm_bridge(stage, axis: VisualAxisData, child_def: dict):
    """Fill the central fork arm toward the existing real organ."""
    parent = axis.members[-1]
    child_axis = _child_axis(parent, child_def)
    junction = axis.start + axis.axis * axis.total_length
    parent_radius = _visual_radius(axis, axis.total_length)
    child_radius = scaled(float(child_def.get("radius", parent_radius * 0.45)))

    bridge_length = max(
        BRIDGE_LENGTH_MIN_M,
        min(BRIDGE_LENGTH_MAX_M, parent_radius * BRIDGE_LENGTH_RADIUS_SCALE),
    )

    # Start slightly inside the now-tapered parent tip, exactly on its centerline.
    p0 = junction - _normalized(axis.axis) * min(0.004, bridge_length * 0.25)
    p2 = junction + child_axis * bridge_length
    p1 = junction + child_axis * (bridge_length * 0.46)

    centers = []
    tangents = []
    for index in range(CURVE_SAMPLES):
        t = index / float(CURVE_SAMPLES - 1)
        u = 1.0 - t
        center = p0 * (u * u) + p1 * (2.0 * u * t) + p2 * (t * t)
        tangent = (p1 - p0) * (2.0 * u) + (p2 - p1) * (2.0 * t)
        centers.append(center)
        tangents.append(_normalized(tangent))

    normals, binormals = _transport_frames(tangents)
    radial_segments = max(RADIAL_SEGMENTS_MIN, axis.profile.radial_segments)
    world_to_link = _link_rest_world(axis).GetInverse()

    root_radius = parent_radius * BRIDGE_ROOT_SCALE
    tip_radius = max(child_radius * 1.08, parent_radius * BRIDGE_TIP_SCALE)

    points = []
    for ring_index, (center, normal, binormal) in enumerate(
        zip(centers, normals, binormals)
    ):
        t = ring_index / float(CURVE_SAMPLES - 1)
        q = _smoothstep(t)
        radius = root_radius + (tip_radius - root_radius) * q
        for radial in range(radial_segments):
            theta = 2.0 * math.pi * radial / radial_segments
            world_point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*world_to_link.Transform(world_point)))

    face_counts = []
    face_indices = []
    for ring in range(CURVE_SAMPLES - 1):
        row0 = ring * radial_segments
        row1 = (ring + 1) * radial_segments
        for radial in range(radial_segments):
            nxt = (radial + 1) % radial_segments
            face_counts.extend((3, 3))
            face_indices.extend((
                row0 + radial,
                row1 + radial,
                row1 + nxt,
                row0 + radial,
                row1 + nxt,
                row0 + nxt,
            ))

    mesh = UsdGeom.Mesh.Define(
        stage,
        f"{axis.link_paths[-1]}/TerminalForkExistingArmBridge",
    )
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(*_axis_color(axis))
    ]))
