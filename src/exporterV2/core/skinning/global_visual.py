"""Shared-Skeleton diagnostic for ExporterV2 vegetative visual axes.

This mode keeps the exact validated smooth meshes and skin weights, but batches
all vegetative physics links into one plant-wide UsdSkel.Skeleton and one
UsdSkel.Animation. The global Skeleton follows the real rigid-link parent graph,
so branch attachments remain proper skeleton parents rather than independent
per-axis roots.
"""

from pxr import Gf, Sdf, UsdGeom, UsdSkel, Vt

from .mesh import _axis_color, _decompose, _pose_matrix, build_axis_tube_data
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
GLOBAL_PARENT_INDICES_ATTR = "autotom:skinning:parentIndices"
GLOBAL_MODE_ATTR = "autotom:skinning:global"


def _build_global_link_graph(resolved):
    resolved = list(resolved)
    by_id = {branch.branch_id: branch for branch in resolved}

    link_paths = []
    link_bases = []
    link_orientations = []
    path_to_index = {}

    for branch in resolved:
        for path, base in zip(branch.link_paths, branch.link_bases):
            path_to_index[path] = len(link_paths)
            link_paths.append(path)
            link_bases.append(base)
            link_orientations.append(branch.orientation)

    parent_indices = []
    for branch in resolved:
        for local_index, _ in enumerate(branch.link_paths):
            if local_index > 0:
                parent_indices.append(
                    path_to_index[branch.link_paths[local_index - 1]]
                )
                continue

            if branch.parent_id is None:
                parent_indices.append(-1)
                continue

            parent_branch = by_id[branch.parent_id]
            parent_path = parent_branch.link_paths[branch.parent_link_index]
            parent_indices.append(path_to_index[parent_path])

    joint_names = []
    for index, parent_index in enumerate(parent_indices):
        token = f"Bone{index:04d}"
        if parent_index >= 0:
            token = f"{joint_names[parent_index]}/{token}"
        joint_names.append(token)

    bind_transforms = [
        _pose_matrix(base, orientation)
        for base, orientation in zip(link_bases, link_orientations)
    ]

    rest_transforms = []
    for index, parent_index in enumerate(parent_indices):
        if parent_index < 0:
            rest_transforms.append(Gf.Matrix4d(bind_transforms[index]))
        else:
            rest_transforms.append(
                bind_transforms[index]
                * bind_transforms[parent_index].GetInverse()
            )

    return (
        link_paths,
        parent_indices,
        joint_names,
        bind_transforms,
        rest_transforms,
        path_to_index,
    )


def _author_bound_mesh(stage, axis, axis_index, path_to_index):
    points, face_counts, face_indices, joint_indices, joint_weights = (
        build_axis_tube_data(axis)
    )

    mesh = UsdGeom.Mesh.Define(
        stage,
        f"{GLOBAL_MESHES_PATH}/Axis_{axis_index:04d}",
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

    axis_global_indices = [path_to_index[path] for path in axis.link_paths]
    global_joint_indices = [
        axis_global_indices[int(index)]
        for index in joint_indices
    ]

    binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    binding.CreateSkeletonRel().SetTargets([Sdf.Path(GLOBAL_SKELETON_PATH)])
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


def author_global_visual_axes(stage, visual_axes, resolved) -> dict:
    visual_axes = list(visual_axes)
    resolved = list(resolved)
    if not visual_axes:
        return {"axes": 0, "bones": 0, "meshes": 0}

    (
        link_paths,
        parent_indices,
        joint_names,
        bind_transforms,
        rest_transforms,
        path_to_index,
    ) = _build_global_link_graph(resolved)

    topology = UsdSkel.Topology(Vt.TokenArray(joint_names))
    valid, reason = topology.Validate()
    if not valid:
        raise ValueError(f"Invalid global skeleton topology: {reason}")

    skel_root = UsdSkel.Root.Define(stage, GLOBAL_SKEL_ROOT_PATH)
    skeleton = UsdSkel.Skeleton.Define(stage, GLOBAL_SKELETON_PATH)
    animation = UsdSkel.Animation.Define(stage, GLOBAL_ANIMATION_PATH)
    UsdGeom.Xform.Define(stage, GLOBAL_MESHES_PATH)

    translations, rotations = _decompose(rest_transforms)

    skeleton.CreateJointsAttr().Set(Vt.TokenArray(joint_names))
    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind_transforms))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest_transforms))

    animation.CreateJointsAttr().Set(Vt.TokenArray(joint_names))
    animation.CreateTranslationsAttr().Set(Vt.Vec3fArray(translations))
    animation.CreateRotationsAttr().Set(Vt.QuatfArray(rotations))
    animation.CreateScalesAttr().Set(Vt.Vec3hArray([
        Gf.Vec3h(1.0, 1.0, 1.0) for _ in joint_names
    ]))

    UsdSkel.BindingAPI.Apply(
        skeleton.GetPrim()
    ).CreateAnimationSourceRel().SetTargets([
        animation.GetPrim().GetPath()
    ])

    for axis_index, axis in enumerate(visual_axes):
        _author_bound_mesh(stage, axis, axis_index, path_to_index)

    root_prim = skel_root.GetPrim()
    root_prim.CreateRelationship(PHYSICS_LINKS_REL, custom=True).SetTargets([
        Sdf.Path(path) for path in link_paths
    ])
    root_prim.CreateRelationship(ANIMATION_REL, custom=True).SetTargets([
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
        GLOBAL_PARENT_INDICES_ATTR,
        Sdf.ValueTypeNames.IntArray,
        custom=True,
    ).Set(Vt.IntArray(parent_indices))
    root_prim.CreateAttribute(
        GLOBAL_MODE_ATTR,
        Sdf.ValueTypeNames.Bool,
        custom=True,
    ).Set(True)
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
