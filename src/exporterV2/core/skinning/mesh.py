"""Metric continuous tube mesh and UsdSkel authoring for visual axes."""

import bisect
import math
from typing import Iterable, List

from pxr import Gf, Sdf, UsdGeom, UsdSkel, Vt

from ..tree_config import PlantColors
from .model import VisualAxisData
from .schema import (
    ANIMATION_REL,
    BRANCH_ID_ATTR,
    PHYSICS_LINKS_REL,
    SCHEMA_VERSION,
    SCHEMA_VERSION_ATTR,
    VISUAL_AXIS_ID_ATTR,
)


_ARC_PRECISION = 12


def _pose_matrix(position: Gf.Vec3d, orientation: Gf.Quatf) -> Gf.Matrix4d:
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(Gf.Rotation(Gf.Quatd(orientation)), position)
    return matrix


def _quatf_from_matrix(matrix: Gf.Matrix4d) -> Gf.Quatf:
    quat = matrix.ExtractRotationQuat()
    return Gf.Quatf(float(quat.GetReal()), Gf.Vec3f(*quat.GetImaginary()))


def _decompose(transforms):
    translations = []
    rotations = []
    for matrix in transforms:
        translations.append(Gf.Vec3f(*matrix.ExtractTranslation()))
        rotations.append(_quatf_from_matrix(matrix))
    return translations, rotations


def _axis_color(axis: VisualAxisData):
    branch = axis.definition
    branch_id = " ".join(
        [branch["id"], *(segment.source_id for segment in axis.visual_segments)]
    ).lower()
    kind = branch.get("kind", "").lower()
    if kind in ("petiole", "rachis") or "petiole" in branch_id or "rachis" in branch_id:
        return PlantColors.PETIOLE
    if kind == "petiolule" or "petiolule" in branch_id:
        return PlantColors.PETIOLULE
    return PlantColors.STEM


def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


def _gaussian(value: float, center: float, sigma: float) -> float:
    delta = (value - center) / max(sigma, 1e-8)
    return math.exp(-0.5 * delta * delta)


def _canonical_arc(value: float, total_length: float) -> float:
    return round(max(0.0, min(total_length, value)), _ARC_PRECISION)


def _radius_transition_half_width(
    axis: VisualAxisData,
    previous,
    current,
) -> float:
    profile = axis.profile

    return min(
        profile.radius_transition_half_width_m,
        previous.length * profile.radius_transition_max_fraction,
        current.length * profile.radius_transition_max_fraction,
    )


def build_axis_sample_arcs(axis: VisualAxisData) -> List[float]:
    """Build optimizer-independent axial samples in world-space meters."""
    spacing = axis.profile.axial_spacing_m
    if spacing <= 0.0:
        raise ValueError(
            f"Visual axis '{axis.axis_id}' axial_spacing_m must be positive"
        )

    arcs = {0.0, _canonical_arc(axis.total_length, axis.total_length)}
    regular_count = int(math.floor(axis.total_length / spacing))
    arcs.update(
        _canonical_arc(index * spacing, axis.total_length)
        for index in range(1, regular_count + 1)
    )
    arcs.update(
        _canonical_arc(segment.end_arc, axis.total_length)
        for segment in axis.visual_segments[:-1]
    )
    # Add local samples around every botanical radius boundary.
    # This keeps the global mesh resolution unchanged while giving
    # radius transitions enough geometry to look smooth.
    for previous, current in zip(
        axis.visual_segments,
        axis.visual_segments[1:],
    ):
        boundary = previous.end_arc

        half_width = _radius_transition_half_width(
            axis,
            previous,
            current,
        )

        sample_count = max(
            3,
            axis.profile.radius_transition_samples,
        )

        for index in range(sample_count):
            t = (
                index
                / float(sample_count - 1)
            )

            transition_arc = (
                boundary
                - half_width
                + 2.0
                * half_width
                * t
            )

            arcs.add(
                _canonical_arc(
                    transition_arc,
                    axis.total_length,
                )
            )
    arcs.update(
        _canonical_arc(arc, axis.total_length)
        for arc in axis.attachment_arcs
    )
    return sorted(arcs)


def _profile_radius(axis: VisualAxisData, arc: float) -> float:
    segments = axis.visual_segments
    radius = segments[-1].radius
    for segment in segments:
        if arc <= segment.end_arc + 1e-12:
            radius = segment.radius
            break

    for previous, current in zip(
        segments,
        segments[1:],
    ):
        boundary = previous.end_arc

        half_width = _radius_transition_half_width(
            axis,
            previous,
            current,
        )

        if (
            half_width > 0.0
            and boundary - half_width
            <= arc
            <= boundary + half_width
        ):
            blend = _smoothstep(
                (
                    arc
                    - (boundary - half_width)
                )
                / (2.0 * half_width)
            )

            return (
                previous.radius
                + (
                    current.radius
                    - previous.radius
                )
                * blend
            )
    return radius


def _visual_radius(axis: VisualAxisData, arc: float) -> float:
    radius = _profile_radius(axis, arc)
    profile = axis.profile
    bulge_sigma = max(axis.total_length * profile.junction_bulge_sigma, 1e-8)
    for center in axis.attachment_arcs:
        radius *= 1.0 + profile.junction_bulge_amplitude * _gaussian(
            arc,
            center,
            bulge_sigma,
        )

    if axis.parent_radius is not None:
        flare_length = min(axis.total_length, max(0.042, axis.parent_radius * 3.2))
        if arc <= flare_length:
            q = arc / max(flare_length, 1e-8)
            fade = 1.0 - _smoothstep(q)
            root_target = max(radius, axis.parent_radius * profile.root_parent_fraction)
            radius += (root_target - radius) * fade
            radius *= (
                1.0
                + profile.root_shoulder_amplitude
                * _gaussian(q, 0.42, 0.22)
                * fade
            )
    return radius


def _skin_weights(axis: VisualAxisData, arc: float):
    profile = axis.profile
    for child in range(1, len(axis.bone_starts)):
        center = axis.bone_starts[child]
        half_width = min(
            profile.skin_blend_half_width_m,
            axis.bone_lengths[child - 1] * 0.25,
            axis.bone_lengths[child] * 0.25,
        )
        if half_width > 0.0 and center - half_width <= arc <= center + half_width:
            blend = (arc - (center - half_width)) / (2.0 * half_width)
            return child - 1, child, 1.0 - blend, blend

    bone = bisect.bisect_right(axis.bone_starts, arc) - 1
    bone = max(0, min(bone, len(axis.bone_starts) - 1))
    return bone, bone, 1.0, 0.0


def build_straight_centerline(axis: VisualAxisData, arcs: Iterable[float]):
    """Sample the exact straight rest pose without procedural curvature."""
    arc_list = list(arcs)
    return arc_list, [axis.start + axis.axis * arc for arc in arc_list]


def build_parallel_transport_frames(axis: VisualAxisData, ring_count: int):
    """Return the constant transport frame implied by a straight axis."""
    rotation = Gf.Rotation(Gf.Quatd(axis.orientation))
    normal = Gf.Vec3d(
        rotation.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
    ).GetNormalized()
    binormal = Gf.Vec3d(
        rotation.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
    ).GetNormalized()
    return [normal] * ring_count, [binormal] * ring_count


def build_axis_tube_data(axis: VisualAxisData):
    """Return mesh topology and skin weights for one continuous visual axis."""
    radial_segments = axis.profile.radial_segments
    if radial_segments < 3:
        raise ValueError(
            f"Visual axis '{axis.axis_id}' radial_segments must be at least 3"
        )
    arcs = build_axis_sample_arcs(axis)
    arcs, centers = build_straight_centerline(axis, arcs)
    normals, binormals = build_parallel_transport_frames(axis, len(arcs))
    points = []
    joint_indices = []
    joint_weights = []

    for arc, center, normal, binormal in zip(arcs, centers, normals, binormals):
        radius = _visual_radius(axis, arc)
        bone0, bone1, weight0, weight1 = _skin_weights(axis, arc)
        for radial in range(radial_segments):
            theta = 2.0 * math.pi * radial / radial_segments
            point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*point))
            joint_indices.extend((bone0, bone1))
            joint_weights.extend((weight0, weight1))

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
    return points, face_counts, face_indices, joint_indices, joint_weights


def _joint_names(count: int):
    names = ["Bone0"]
    for index in range(1, count):
        names.append(f"{names[-1]}/Bone{index}")
    return names


def author_visual_axis(stage, axis: VisualAxisData) -> None:
    """Author one smooth tube, skeleton, animation, and discovery relations."""
    UsdGeom.Xform.Define(stage, axis.visual_root_path)
    skel_root = UsdSkel.Root.Define(stage, axis.skel_root_path)
    skeleton = UsdSkel.Skeleton.Define(stage, axis.skeleton_path)
    animation = UsdSkel.Animation.Define(stage, axis.animation_path)

    names = _joint_names(len(axis.link_paths))
    bind = [
        _pose_matrix(base, orientation)
        for base, orientation in zip(axis.link_bases, axis.link_orientations)
    ]
    rest = [Gf.Matrix4d(bind[0])]
    for index in range(1, len(bind)):
        rest.append(bind[index] * bind[index - 1].GetInverse())
    translations, rotations = _decompose(rest)

    skeleton.CreateJointsAttr().Set(Vt.TokenArray(names))
    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest))
    animation.CreateJointsAttr().Set(Vt.TokenArray(names))
    animation.CreateTranslationsAttr().Set(Vt.Vec3fArray(translations))
    animation.CreateRotationsAttr().Set(Vt.QuatfArray(rotations))
    animation.CreateScalesAttr().Set(Vt.Vec3hArray([
        Gf.Vec3h(1.0, 1.0, 1.0) for _ in axis.link_paths
    ]))
    UsdSkel.BindingAPI.Apply(skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets([
        animation.GetPrim().GetPath()
    ])

    mesh = UsdGeom.Mesh.Define(stage, axis.mesh_path)
    points, face_counts, face_indices, joint_indices, joint_weights = (
        build_axis_tube_data(axis)
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

    binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    binding.CreateSkeletonRel().SetTargets([skeleton.GetPrim().GetPath()])
    binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))
    index_primvar = binding.CreateJointIndicesPrimvar(constant=False, elementSize=2)
    index_primvar.SetInterpolation(UsdGeom.Tokens.vertex)
    index_primvar.Set(Vt.IntArray(joint_indices))
    weight_primvar = binding.CreateJointWeightsPrimvar(constant=False, elementSize=2)
    weight_primvar.SetInterpolation(UsdGeom.Tokens.vertex)
    weight_primvar.Set(Vt.FloatArray(joint_weights))

    root_prim = skel_root.GetPrim()
    root_prim.CreateRelationship(PHYSICS_LINKS_REL, custom=True).SetTargets([
        Sdf.Path(path) for path in axis.link_paths
    ])
    root_prim.CreateRelationship(ANIMATION_REL, custom=True).SetTargets([
        animation.GetPrim().GetPath()
    ])
    root_prim.CreateAttribute(
        VISUAL_AXIS_ID_ATTR,
        Sdf.ValueTypeNames.String,
        custom=True,
    ).Set(axis.axis_id)
    root_prim.CreateAttribute(
        BRANCH_ID_ATTR,
        Sdf.ValueTypeNames.String,
        custom=True,
    ).Set(axis.members[0].branch_id)
    root_prim.CreateAttribute(
        SCHEMA_VERSION_ATTR,
        Sdf.ValueTypeNames.Int,
        custom=True,
    ).Set(SCHEMA_VERSION)
