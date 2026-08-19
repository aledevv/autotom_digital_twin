"""Shared-Skeleton diagnostic for ExporterV2 vegetative visual axes.

This module keeps the exact same smooth meshes and per-vertex skin weights as the
validated per-axis skinning backend, but binds every visual axis to one shared
UsdSkel.Skeleton and one shared UsdSkel.Animation.  Each visual axis becomes an
independent root chain inside the global Skeleton, so the deformation is
mathematically equivalent to the existing per-axis setup while reducing the
number of SkelRoot/Skeleton/Animation objects and per-frame animation writes.
"""

from pxr import Gf, Sdf, UsdGeom, UsdSkel, Vt

from .mesh import (
    _axis_color,
    _decompose,
    _pose_matrix,
    build_axis_tube_data,
)
from .schema import (
    ANIMATION_REL,
    BRANCH_ID_ATTR,
    PHYSICS_LINKS_REL,
    SCHEMA_VERSION,
    SCHEMA_VERSION_ATTR,
    VISUAL_AXIS_ID_ATTR,
)


GLOBAL_SKEL_ROOT_PATH = "/World/PlantVisual/GlobalSkelRoot"
GLOBAL_SKELETON_PATH = f"{GLOBAL_SKEL_ROOT_PATH}/Skeleton"
GLOBAL_ANIMATION_PATH = f"{GLOBAL_SKEL_ROOT_PATH}/SkelAnim"
GLOBAL_MESHES_PATH = f"{GLOBAL_SKEL_ROOT_PATH}/Meshes"


def _global_joint_names(axis_index: int, bone_count: int):
    """Return one independent root chain for an axis inside the shared Skeleton."""
    root = f"Axis{axis_index:04d}_Bone0"
    names = [root]
    for bone_index in range(1, bone_count):
        names.append(f"{names[-1]}/Bone{bone_index}")
    return names


def _author_bound_mesh(
    stage,
    axis,
    axis_index: int,
    skeleton_path: str,
    joint_index_offset: int,
):
    """Author one existing smooth axis mesh bound into the global joint order."""
    points, face_counts, face_indices, joint_indices, joint_weights = (
        build_axis_tube_data(axis)
    )

    mesh_path = f"{GLOBAL_MESHES_PATH}/Axis_{axis_index:04d}"
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

    # build_axis_tube_data() returns indices local to this visual axis.
    # With no per-mesh skel:joints remapping, UsdSkel interprets the indices in
    # the bound Skeleton's global joint order, so offset them into that order.
    global_joint_indices = [
        int(index) + joint_index_offset
        for index in joint_indices
    ]

    binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    binding.CreateSkeletonRel().SetTargets([Sdf.Path(skeleton_path)])
    binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))

    index_primvar = binding.CreateJointIndicesPrimvar(
        constant=False,
        elementSize=2,
    )
    index_primvar.SetInterpolation(UsdGeom.Tokens.vertex)
    index_primvar.Set(Vt.IntArray(global_joint_indices))

    weight_primvar = binding.CreateJointWeightsPrimvar(
        constant=False,
        elementSize=2,
    )
    weight_primvar.SetInterpolation(UsdGeom.Tokens.vertex)
    weight_primvar.Set(Vt.FloatArray(joint_weights))


def author_global_visual_axes(stage, visual_axes) -> dict:
    """Author all visual axes using one shared Skeleton and SkelAnimation.

    Every axis remains an independent joint chain.  This intentionally does not
    change the physical branch graph or the mesh geometry; it only batches the
    UsdSkel representation.
    """
    visual_axes = list(visual_axes)
    if not visual_axes:
        return {
            "axes": 0,
            "bones": 0,
            "meshes": 0,
        }

    skel_root = UsdSkel.Root.Define(stage, GLOBAL_SKEL_ROOT_PATH)
    skeleton = UsdSkel.Skeleton.Define(stage, GLOBAL_SKELETON_PATH)
    animation = UsdSkel.Animation.Define(stage, GLOBAL_ANIMATION_PATH)
    UsdGeom.Xform.Define(stage, GLOBAL_MESHES_PATH)

    joint_names = []
    bind_transforms = []
    rest_transforms = []
    all_link_paths = []
    axis_offsets = []

    joint_offset = 0
    for axis_index, axis in enumerate(visual_axes):
        bone_count = len(axis.link_paths)
        if bone_count < 1:
            raise ValueError(
                f"Visual axis '{axis.axis_id}' has no physics links for global skinning"
            )

        axis_offsets.append(joint_offset)
        joint_names.extend(_global_joint_names(axis_index, bone_count))
        all_link_paths.extend(axis.link_paths)

        axis_bind = [
            _pose_matrix(base, orientation)
            for base, orientation in zip(
                axis.link_bases,
                axis.link_orientations,
            )
        ]
        bind_transforms.extend(axis_bind)

        # Each axis is an independent root chain in the shared Skeleton.  The
        # first bone therefore remains in SkelRoot space (world space here),
        # while subsequent bones are local to the preceding bone exactly as in
        # the original per-axis authoring.
        axis_rest = [Gf.Matrix4d(axis_bind[0])]
        for bone_index in range(1, bone_count):
            axis_rest.append(
                axis_bind[bone_index]
                * axis_bind[bone_index - 1].GetInverse()
            )
        rest_transforms.extend(axis_rest)
        joint_offset += bone_count

    topology = UsdSkel.Topology(Vt.TokenArray(joint_names))
    valid, reason = topology.Validate()
    if not valid:
        raise ValueError(f"Invalid global skeleton topology: {reason}")

    translations, rotations = _decompose(rest_transforms)

    skeleton.CreateJointsAttr().Set(Vt.TokenArray(joint_names))
    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind_transforms))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest_transforms))

    animation.CreateJointsAttr().Set(Vt.TokenArray(joint_names))
    animation.CreateTranslationsAttr().Set(Vt.Vec3fArray(translations))
    animation.CreateRotationsAttr().Set(Vt.QuatfArray(rotations))
    animation.CreateScalesAttr().Set(Vt.Vec3hArray([
        Gf.Vec3h(1.0, 1.0, 1.0)
        for _ in joint_names
    ]))

    UsdSkel.BindingAPI.Apply(
        skeleton.GetPrim()
    ).CreateAnimationSourceRel().SetTargets([
        animation.GetPrim().GetPath()
    ])

    for axis_index, (axis, offset) in enumerate(zip(visual_axes, axis_offsets)):
        _author_bound_mesh(
            stage,
            axis,
            axis_index,
            GLOBAL_SKELETON_PATH,
            offset,
        )

    root_prim = skel_root.GetPrim()
    root_prim.CreateRelationship(
        PHYSICS_LINKS_REL,
        custom=True,
    ).SetTargets([
        Sdf.Path(path)
        for path in all_link_paths
    ])
    root_prim.CreateRelationship(
        ANIMATION_REL,
        custom=True,
    ).SetTargets([
        animation.GetPrim().GetPath()
    ])
    root_prim.CreateAttribute(
        VISUAL_AXIS_ID_ATTR,
        Sdf.ValueTypeNames.String,
        custom=True,
    ).Set("__global_plant_skeleton__")
    root_prim.CreateAttribute(
        BRANCH_ID_ATTR,
        Sdf.ValueTypeNames.String,
        custom=True,
    ).Set("__global_plant_skeleton__")
    root_prim.CreateAttribute(
        SCHEMA_VERSION_ATTR,
        Sdf.ValueTypeNames.Int,
        custom=True,
    ).Set(SCHEMA_VERSION)

    return {
        "axes": len(visual_axes),
        "bones": len(joint_names),
        "meshes": len(visual_axes),
    }
