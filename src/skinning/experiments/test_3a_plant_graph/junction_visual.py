"""
junction_visual.py — visual-only junction blending for Test 2C-C.

IMPORTANT:
This module changes only the skinned VISUAL mesh radius.

It does NOT modify:
    - rigid body dimensions
    - capsule collider dimensions
    - masses
    - D6 joint frames
    - D6 stiffness / damping
    - PhysX topology

The goal is to visually hide the hard intersection between two independently
skinned branch meshes.
"""

from dataclasses import dataclass
import math

from pxr import Gf, UsdGeom, UsdSkel, Vt

import branch_core_fixed as core


@dataclass(frozen=True)
class VisualBulge:
    center_fraction: float
    amplitude: float
    sigma_fraction: float


@dataclass(frozen=True)
class RootFlare:
    parent_radius: float
    flare_length: float

    # Root radius target relative to parent radius.
    root_parent_fraction: float = 0.92

    # Small shoulder just outside the parent surface.
    shoulder_amplitude: float = 0.18
    shoulder_center_fraction: float = 0.42
    shoulder_sigma_fraction: float = 0.22


def _gaussian(
    x,
    center,
    sigma,
):
    sigma = max(
        float(sigma),
        1e-8,
    )

    d = (
        (float(x) - float(center))
        / sigma
    )

    return math.exp(
        -0.5 * d * d
    )


def visual_radius_for_arc(
    branch,
    s,
    bulges=(),
    root_flare=None,
):
    """
    Visual-only radius.

    Starts from the validated physical/organic radius profile and adds:
        1. optional local bulges on the parent
        2. optional child root flare near s=0
    """
    spec = branch.spec
    centerline = branch.centerline

    radius = core.radius_for_arc(
        spec,
        centerline,
        s,
    )

    total = float(
        centerline[
            "total_length"
        ]
    )

    if total > 1e-10:
        u = (
            float(s)
            / total
        )

        for bulge in bulges:
            radius *= (
                1.0
                + bulge.amplitude
                * _gaussian(
                    u,
                    bulge.center_fraction,
                    bulge.sigma_fraction,
                )
            )

    if (
        root_flare is not None
        and s <= root_flare.flare_length
    ):
        flare_length = max(
            root_flare.flare_length,
            1e-8,
        )

        q = max(
            0.0,
            min(
                1.0,
                float(s)
                / flare_length,
            ),
        )

        # 1 at root, smoothly tends to 0 at the end of the flare.
        fade = (
            1.0
            - core.smoothstep01(q)
        )

        normal_root_radius = (
            core.radius_for_arc(
                spec,
                centerline,
                0.0,
            )
        )

        target_root_radius = max(
            normal_root_radius,
            root_flare.parent_radius
            * root_flare.root_parent_fraction,
        )

        root_extra = (
            target_root_radius
            - normal_root_radius
        )

        radius += (
            root_extra
            * fade
        )

        # Organic shoulder just outside the stem surface.
        radius *= (
            1.0
            + root_flare.shoulder_amplitude
            * _gaussian(
                q,
                root_flare.shoulder_center_fraction,
                root_flare.shoulder_sigma_fraction,
            )
            * fade
        )

    return radius


def build_visual_tube_data(
    branch,
    bulges=(),
    root_flare=None,
):
    spec = branch.spec
    centerline = branch.centerline
    physics = branch.physics
    normals = branch.normals
    binormals = branch.binormals

    points = []
    joint_indices = []
    joint_weights = []

    for ring, center in enumerate(
        centerline["positions"]
    ):
        normal = normals[ring]
        binormal = binormals[ring]

        s = float(
            centerline[
                "arc"
            ][ring]
        )

        radius = visual_radius_for_arc(
            branch,
            s,
            bulges=bulges,
            root_flare=root_flare,
        )

        (
            bone0,
            bone1,
            weight0,
            weight1,
        ) = core.skin_weights_for_arc(
            spec,
            centerline,
            physics,
            s,
        )

        for k in range(
            spec.radial_segments
        ):
            theta = (
                2.0
                * math.pi
                * k
                / spec.radial_segments
            )

            radial = (
                normal
                * (
                    math.cos(theta)
                    * radius
                )
                + binormal
                * (
                    math.sin(theta)
                    * radius
                )
            )

            point = (
                center
                + radial
            )

            points.append(
                Gf.Vec3f(
                    float(point[0]),
                    float(point[1]),
                    float(point[2]),
                )
            )

            joint_indices.extend(
                [bone0, bone1]
            )

            joint_weights.extend(
                [weight0, weight1]
            )

    face_counts = []
    face_indices = []

    ring_count = len(
        centerline[
            "positions"
        ]
    )

    for ring in range(
        ring_count - 1
    ):
        row0 = (
            ring
            * spec.radial_segments
        )

        row1 = (
            (ring + 1)
            * spec.radial_segments
        )

        for k in range(
            spec.radial_segments
        ):
            next_k = (
                (k + 1)
                % spec.radial_segments
            )

            v00 = row0 + k
            v01 = row0 + next_k
            v10 = row1 + k
            v11 = row1 + next_k

            face_counts.extend(
                [3, 3]
            )

            face_indices.extend([
                v00,
                v10,
                v11,
                v00,
                v11,
                v01,
            ])

    return (
        points,
        face_counts,
        face_indices,
        joint_indices,
        joint_weights,
    )


def build_branch_visual(
    stage,
    branch,
    color,
    bulges=(),
    root_flare=None,
):
    """
    Same UsdSkel construction as branch_core_fixed.build_branch_visual(),
    but with a visual-only radius modifier.
    """
    UsdGeom.Xform.Define(
        stage,
        branch.visual_root_path,
    )

    UsdSkel.Root.Define(
        stage,
        branch.skel_root_path,
    )

    skeleton = (
        UsdSkel.Skeleton.Define(
            stage,
            branch.skeleton_path,
        )
    )

    animation = (
        UsdSkel.Animation.Define(
            stage,
            branch.animation_path,
        )
    )

    names = core.joint_names(
        branch.spec.physics_links
    )

    bind = branch.physics["bind"]

    rest = core.rest_local_transforms(
        branch.physics
    )

    skeleton.CreateJointsAttr().Set(
        Vt.TokenArray(names)
    )

    skeleton.CreateBindTransformsAttr().Set(
        Vt.Matrix4dArray(bind)
    )

    skeleton.CreateRestTransformsAttr().Set(
        Vt.Matrix4dArray(rest)
    )

    animation.CreateJointsAttr().Set(
        Vt.TokenArray(names)
    )

    translations = []
    rotations = []

    for matrix in rest:
        t = matrix.ExtractTranslation()
        q = matrix.ExtractRotationQuat()
        qi = q.GetImaginary()

        translations.append(
            Gf.Vec3f(
                float(t[0]),
                float(t[1]),
                float(t[2]),
            )
        )

        rotations.append(
            Gf.Quatf(
                float(q.GetReal()),
                Gf.Vec3f(
                    float(qi[0]),
                    float(qi[1]),
                    float(qi[2]),
                ),
            )
        )

    animation.CreateTranslationsAttr().Set(
        Vt.Vec3fArray(
            translations
        )
    )

    animation.CreateRotationsAttr().Set(
        Vt.QuatfArray(
            rotations
        )
    )

    animation.CreateScalesAttr().Set(
        Vt.Vec3hArray([
            Gf.Vec3h(
                1.0,
                1.0,
                1.0,
            )
            for _ in range(
                branch.spec.physics_links
            )
        ])
    )

    skeleton_binding = (
        UsdSkel.BindingAPI.Apply(
            skeleton.GetPrim()
        )
    )

    skeleton_binding.CreateAnimationSourceRel().SetTargets(
        [
            animation
            .GetPrim()
            .GetPath()
        ]
    )

    mesh = (
        UsdGeom.Mesh.Define(
            stage,
            branch.mesh_path,
        )
    )

    (
        points,
        face_counts,
        face_indices,
        joint_indices,
        joint_weights,
    ) = build_visual_tube_data(
        branch,
        bulges=bulges,
        root_flare=root_flare,
    )

    mesh.CreatePointsAttr().Set(
        Vt.Vec3fArray(points)
    )

    mesh.CreateFaceVertexCountsAttr().Set(
        Vt.IntArray(face_counts)
    )

    mesh.CreateFaceVertexIndicesAttr().Set(
        Vt.IntArray(face_indices)
    )

    mesh.CreateSubdivisionSchemeAttr().Set(
        UsdGeom.Tokens.none
    )

    mesh.CreateOrientationAttr().Set(
        UsdGeom.Tokens.rightHanded
    )

    mesh.CreateDoubleSidedAttr().Set(
        True
    )

    mesh.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray([
            Gf.Vec3f(
                float(color[0]),
                float(color[1]),
                float(color[2]),
            )
        ])
    )

    mesh_binding = (
        UsdSkel.BindingAPI.Apply(
            mesh.GetPrim()
        )
    )

    mesh_binding.CreateSkeletonRel().SetTargets(
        [
            skeleton
            .GetPrim()
            .GetPath()
        ]
    )

    mesh_binding.CreateGeomBindTransformAttr().Set(
        Gf.Matrix4d(1.0)
    )

    indices = (
        mesh_binding
        .CreateJointIndicesPrimvar(
            constant=False,
            elementSize=2,
        )
    )

    indices.SetInterpolation(
        UsdGeom.Tokens.vertex
    )

    indices.Set(
        Vt.IntArray(
            joint_indices
        )
    )

    weights = (
        mesh_binding
        .CreateJointWeightsPrimvar(
            constant=False,
            elementSize=2,
        )
    )

    weights.SetInterpolation(
        UsdGeom.Tokens.vertex
    )

    weights.Set(
        Vt.FloatArray(
            joint_weights
        )
    )

    return branch.animation_path
