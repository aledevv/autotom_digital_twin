"""
branch_core.py — reusable branch building blocks for Test 2C+

Derived from the validated Test 2B-B pipeline.

A branch is still represented by three independent resolutions:
    - visual resolution: dense smooth centerline + skinned mesh
    - articulation resolution: rigid links / bones / D6
    - collision resolution: compound capsule shapes

This module intentionally contains no plant topology. The topology
(main stem, lateral branches, junctions) is authored by the test generator.
"""

from dataclasses import dataclass, field
import math

from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdSkel, Vt


# ============================================================================
# SPECS
# ============================================================================

@dataclass(frozen=True)
class RadiusProfile:
    base_radius: float = 0.014
    tip_radius: float = 0.008
    taper_start: float = 0.04
    taper_end: float = 0.96

    swell_fractions: tuple[float, ...] = ()
    swell_amplitude: float = 0.0
    swell_sigma_fraction: float = 0.035

    micro_variation_amplitude: float = 0.0
    micro_variation_cycles: float = 2.0


@dataclass(frozen=True)
class BranchSpec:
    control_points: tuple[tuple[float, float, float], ...]

    physics_links: int = 4
    samples_per_control_segment: int = 18
    radial_segments: int = 14

    radius: RadiusProfile = field(default_factory=RadiusProfile)

    linear_density_kg_per_m: float = 0.25

    collider_radius_scale: float = 0.90
    colliders_per_link: int = 2

    # Desired TOTAL axial capsule length as fraction of the centerline
    # sub-interval represented by each capsule.
    collider_length_scale: float = 0.92

    joint_stiffness: float = 0.0
    joint_damping: float = 0.05
    bend_limit_deg: float = 65.0

    skin_blend_fraction: float = 0.32

    show_physics_colliders: bool = True


@dataclass
class BranchData:
    name: str
    spec: BranchSpec
    centerline: dict
    physics: dict
    normals: list
    binormals: list

    physics_root_path: str
    visual_root_path: str

    link_paths: list[str]
    skel_root_path: str
    skeleton_path: str
    animation_path: str
    mesh_path: str


# ============================================================================
# BASIC MATH
# ============================================================================

def normalize(v):
    v = Gf.Vec3d(v)
    length = float(v.GetLength())

    if length < 1e-10:
        raise ValueError("Cannot normalize zero-length vector.")

    return v / length


def clamp(x, lo, hi):
    return max(lo, min(hi, float(x)))


def smoothstep01(u):
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def rotation_from_tangent(tangent):
    """
    Every rigid link / bone / capsule uses local +Z as longitudinal axis.
    """
    return Gf.Rotation(
        Gf.Vec3d(0.0, 0.0, 1.0),
        normalize(tangent),
    )


def quatf_from_rotation(rotation):
    q = rotation.GetQuat()
    qi = q.GetImaginary()

    return Gf.Quatf(
        float(q.GetReal()),
        Gf.Vec3f(
            float(qi[0]),
            float(qi[1]),
            float(qi[2]),
        ),
    )


def quatf_from_matrix(matrix):
    q = matrix.ExtractRotationQuat()
    qi = q.GetImaginary()

    return Gf.Quatf(
        float(q.GetReal()),
        Gf.Vec3f(
            float(qi[0]),
            float(qi[1]),
            float(qi[2]),
        ),
    )


def pose_matrix(position, rotation):
    matrix = Gf.Matrix4d(1.0)
    matrix.SetTransform(
        rotation,
        position,
    )
    return matrix


def local_frame_from_world(
    frame_world,
    body_world,
):
    """
    Gf/OpenUSD row-vector convention.
    """
    return (
        frame_world
        * body_world.GetInverse()
    )


# ============================================================================
# SMOOTH HERMITE CENTERLINE
# ============================================================================

def _control_points(spec):
    if len(spec.control_points) < 2:
        raise ValueError(
            "At least two control points are required."
        )

    return [
        Gf.Vec3d(*point)
        for point in spec.control_points
    ]


def _control_tangents(points):
    result = []

    for i in range(len(points)):
        if i == 0:
            tangent = (
                points[1]
                - points[0]
            )
        elif i == len(points) - 1:
            tangent = (
                points[-1]
                - points[-2]
            )
        else:
            tangent = 0.5 * (
                points[i + 1]
                - points[i - 1]
            )

        result.append(
            Gf.Vec3d(tangent)
        )

    return result


def _hermite_point(
    p0,
    p1,
    m0,
    m1,
    u,
):
    u2 = u * u
    u3 = u2 * u

    h00 = (
        2.0 * u3
        - 3.0 * u2
        + 1.0
    )
    h10 = (
        u3
        - 2.0 * u2
        + u
    )
    h01 = (
        -2.0 * u3
        + 3.0 * u2
    )
    h11 = (
        u3
        - u2
    )

    return Gf.Vec3d(
        p0 * h00
        + m0 * h10
        + p1 * h01
        + m1 * h11
    )


def _hermite_derivative(
    p0,
    p1,
    m0,
    m1,
    u,
):
    u2 = u * u

    return Gf.Vec3d(
        p0 * (
            6.0 * u2
            - 6.0 * u
        )
        + m0 * (
            3.0 * u2
            - 4.0 * u
            + 1.0
        )
        + p1 * (
            -6.0 * u2
            + 6.0 * u
        )
        + m1 * (
            3.0 * u2
            - 2.0 * u
        )
    )


def build_smooth_centerline(spec):
    controls = _control_points(spec)
    derivatives = _control_tangents(
        controls
    )

    positions = []
    tangents = []
    control_sample_indices = [
        None
    ] * len(controls)

    for segment in range(
        len(controls) - 1
    ):
        p0 = controls[segment]
        p1 = controls[segment + 1]

        m0 = derivatives[segment]
        m1 = derivatives[segment + 1]

        for j in range(
            spec.samples_per_control_segment
        ):
            u = (
                j
                / float(
                    spec.samples_per_control_segment
                )
            )

            if j == 0:
                control_sample_indices[
                    segment
                ] = len(positions)

            point = _hermite_point(
                p0,
                p1,
                m0,
                m1,
                u,
            )

            derivative = (
                _hermite_derivative(
                    p0,
                    p1,
                    m0,
                    m1,
                    u,
                )
            )

            positions.append(
                point
            )
            tangents.append(
                normalize(derivative)
            )

    positions.append(
        Gf.Vec3d(
            controls[-1]
        )
    )
    tangents.append(
        normalize(
            derivatives[-1]
        )
    )

    control_sample_indices[-1] = (
        len(positions) - 1
    )

    arc = [0.0]

    for i in range(
        1,
        len(positions),
    ):
        arc.append(
            arc[-1]
            + float(
                (
                    positions[i]
                    - positions[i - 1]
                ).GetLength()
            )
        )

    return {
        "controls": controls,
        "positions": positions,
        "tangents": tangents,
        "arc": arc,
        "control_arc": [
            arc[index]
            for index
            in control_sample_indices
        ],
        "total_length": float(
            arc[-1]
        ),
    }


def point_at_arc(
    centerline,
    s,
):
    positions = centerline[
        "positions"
    ]
    arc = centerline["arc"]
    total = float(arc[-1])

    s = clamp(
        s,
        0.0,
        total,
    )

    if s <= 0.0:
        return Gf.Vec3d(
            positions[0]
        )

    if s >= total:
        return Gf.Vec3d(
            positions[-1]
        )

    for i in range(
        len(arc) - 1
    ):
        if (
            arc[i]
            <= s
            <= arc[i + 1]
        ):
            ds = (
                arc[i + 1]
                - arc[i]
            )

            if ds <= 1e-12:
                return Gf.Vec3d(
                    positions[i]
                )

            u = (
                (s - arc[i])
                / ds
            )

            return Gf.Vec3d(
                positions[i]
                + (
                    positions[i + 1]
                    - positions[i]
                ) * u
            )

    return Gf.Vec3d(
        positions[-1]
    )


def tangent_at_arc(
    centerline,
    s,
):
    """
    Linear interpolation of neighboring sampled tangents followed by normalize.
    """
    arc = centerline["arc"]
    tangents = centerline[
        "tangents"
    ]
    total = float(arc[-1])

    s = clamp(
        s,
        0.0,
        total,
    )

    if s <= 0.0:
        return Gf.Vec3d(
            tangents[0]
        )

    if s >= total:
        return Gf.Vec3d(
            tangents[-1]
        )

    for i in range(
        len(arc) - 1
    ):
        if (
            arc[i]
            <= s
            <= arc[i + 1]
        ):
            ds = (
                arc[i + 1]
                - arc[i]
            )

            if ds <= 1e-12:
                return Gf.Vec3d(
                    tangents[i]
                )

            u = (
                (s - arc[i])
                / ds
            )

            return normalize(
                tangents[i]
                * (1.0 - u)
                + tangents[i + 1]
                * u
            )

    return Gf.Vec3d(
        tangents[-1]
    )


# ============================================================================
# PHYSICS / BONE DISCRETIZATION
# ============================================================================

def build_physics_discretization(
    spec,
    centerline,
):
    if spec.physics_links < 1:
        raise ValueError(
            "physics_links must be >= 1."
        )

    total = float(
        centerline[
            "total_length"
        ]
    )

    node_arc = [
        total
        * i
        / spec.physics_links
        for i in range(
            spec.physics_links + 1
        )
    ]

    nodes = [
        point_at_arc(
            centerline,
            s,
        )
        for s in node_arc
    ]

    origins = []
    tangents = []
    lengths = []
    rotations = []
    bind = []

    for i in range(
        spec.physics_links
    ):
        p0 = nodes[i]
        p1 = nodes[i + 1]

        chord = (
            p1 - p0
        )

        length = float(
            chord.GetLength()
        )

        if length <= 1e-8:
            raise ValueError(
                f"Generated physics link "
                f"{i} has zero length."
            )

        tangent = (
            chord / length
        )

        rotation = (
            rotation_from_tangent(
                tangent
            )
        )

        origins.append(
            Gf.Vec3d(p0)
        )
        tangents.append(
            Gf.Vec3d(tangent)
        )
        lengths.append(
            length
        )
        rotations.append(
            rotation
        )
        bind.append(
            pose_matrix(
                p0,
                rotation,
            )
        )

    return {
        "node_arc": node_arc,
        "nodes": nodes,
        "origins": origins,
        "tangents": tangents,
        "lengths": lengths,
        "rotations": rotations,
        "bind": bind,
    }


def physics_link_for_arc(
    physics,
    s,
):
    """
    Return the physical link whose centerline interval contains arc s.
    """
    node_arc = physics[
        "node_arc"
    ]

    s = clamp(
        s,
        node_arc[0],
        node_arc[-1],
    )

    for i in range(
        len(node_arc) - 1
    ):
        if (
            node_arc[i]
            <= s
            <= node_arc[i + 1]
        ):
            return i

    return (
        len(node_arc)
        - 2
    )


# ============================================================================
# PARALLEL TRANSPORT
# ============================================================================

def _initial_normal(tangent):
    preferred = Gf.Vec3d(
        0.0,
        0.0,
        1.0,
    )

    if abs(
        float(
            Gf.Dot(
                preferred,
                tangent,
            )
        )
    ) > 0.95:
        preferred = Gf.Vec3d(
            1.0,
            0.0,
            0.0,
        )

    normal = (
        preferred
        - tangent
        * Gf.Dot(
            preferred,
            tangent,
        )
    )

    return normalize(normal)


def _rodrigues(
    vector,
    axis,
    angle,
):
    axis = normalize(axis)

    c = math.cos(angle)
    s = math.sin(angle)

    return Gf.Vec3d(
        vector * c
        + Gf.Cross(
            axis,
            vector,
        ) * s
        + axis
        * (
            Gf.Dot(
                axis,
                vector,
            )
            * (1.0 - c)
        )
    )


def _transport_normal(
    previous_tangent,
    current_tangent,
    previous_normal,
):
    previous_tangent = normalize(
        previous_tangent
    )
    current_tangent = normalize(
        current_tangent
    )

    axis = Gf.Cross(
        previous_tangent,
        current_tangent,
    )

    sin_angle = float(
        axis.GetLength()
    )

    cos_angle = clamp(
        float(
            Gf.Dot(
                previous_tangent,
                current_tangent,
            )
        ),
        -1.0,
        1.0,
    )

    if sin_angle < 1e-9:
        normal = Gf.Vec3d(
            previous_normal
        )
    else:
        angle = math.atan2(
            sin_angle,
            cos_angle,
        )

        normal = _rodrigues(
            previous_normal,
            axis / sin_angle,
            angle,
        )

    normal = (
        normal
        - current_tangent
        * Gf.Dot(
            normal,
            current_tangent,
        )
    )

    if normal.GetLength() < 1e-9:
        return _initial_normal(
            current_tangent
        )

    return normalize(normal)


def build_transport_frames(
    centerline,
):
    tangents = centerline[
        "tangents"
    ]

    normals = [
        _initial_normal(
            tangents[0]
        )
    ]

    for i in range(
        1,
        len(tangents),
    ):
        normals.append(
            _transport_normal(
                tangents[i - 1],
                tangents[i],
                normals[-1],
            )
        )

    binormals = [
        normalize(
            Gf.Cross(
                tangent,
                normal,
            )
        )
        for tangent, normal
        in zip(
            tangents,
            normals,
        )
    ]

    return (
        normals,
        binormals,
    )


# ============================================================================
# RADIUS PROFILE
# ============================================================================

def taper_radius(
    spec,
    centerline,
    s,
):
    profile = spec.radius

    total = float(
        centerline[
            "total_length"
        ]
    )

    if total <= 1e-12:
        return (
            profile.base_radius
        )

    u_global = clamp(
        s / total,
        0.0,
        1.0,
    )

    if (
        u_global
        <= profile.taper_start
    ):
        return (
            profile.base_radius
        )

    if (
        u_global
        >= profile.taper_end
    ):
        return (
            profile.tip_radius
        )

    denominator = (
        profile.taper_end
        - profile.taper_start
    )

    if abs(denominator) < 1e-12:
        return (
            profile.tip_radius
        )

    u = (
        (
            u_global
            - profile.taper_start
        )
        / denominator
    )

    blend = smoothstep01(u)

    return (
        profile.base_radius
        + (
            profile.tip_radius
            - profile.base_radius
        ) * blend
    )


def radius_for_arc(
    spec,
    centerline,
    s,
):
    profile = spec.radius
    total = float(
        centerline[
            "total_length"
        ]
    )

    base = taper_radius(
        spec,
        centerline,
        s,
    )

    if total <= 1e-12:
        return base

    swell_factor = 1.0

    sigma = max(
        total
        * profile.swell_sigma_fraction,
        1e-6,
    )

    for fraction in (
        profile.swell_fractions
    ):
        center = (
            total
            * clamp(
                fraction,
                0.0,
                1.0,
            )
        )

        x = (
            (s - center)
            / sigma
        )

        swell_factor += (
            profile.swell_amplitude
            * math.exp(
                -0.5 * x * x
            )
        )

    u = clamp(
        s / total,
        0.0,
        1.0,
    )

    envelope = (
        math.sin(
            math.pi * u
        ) ** 2
    )

    phase1 = (
        2.0
        * math.pi
        * profile.micro_variation_cycles
        * u
    )

    phase2 = (
        2.0
        * math.pi
        * (
            profile.micro_variation_cycles
            * 0.47
        )
        * u
        + 0.8
    )

    signal = (
        0.70
        * math.sin(phase1)
        + 0.30
        * math.sin(phase2)
    )

    micro = (
        1.0
        + profile.micro_variation_amplitude
        * envelope
        * signal
    )

    return (
        base
        * swell_factor
        * micro
    )


# ============================================================================
# SKINNING
# ============================================================================

def skin_weights_for_arc(
    spec,
    centerline,
    physics,
    s,
):
    node_arc = physics[
        "node_arc"
    ]

    mean_arc_link = (
        centerline[
            "total_length"
        ]
        / spec.physics_links
    )

    half_width = (
        mean_arc_link
        * spec.skin_blend_fraction
        * 0.5
    )

    for child in range(
        1,
        spec.physics_links,
    ):
        center = (
            node_arc[child]
        )

        lo = (
            center
            - half_width
        )
        hi = (
            center
            + half_width
        )

        if lo <= s <= hi:
            u = clamp(
                (s - lo)
                / (hi - lo),
                0.0,
                1.0,
            )

            return (
                child - 1,
                child,
                1.0 - u,
                u,
            )

    bone = 0

    for i in range(
        1,
        spec.physics_links,
    ):
        if s >= node_arc[i]:
            bone = i
        else:
            break

    return (
        bone,
        bone,
        1.0,
        0.0,
    )


def build_tube_data(
    spec,
    centerline,
    physics,
    normals,
    binormals,
):
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

        radius = radius_for_arc(
            spec,
            centerline,
            s,
        )

        (
            bone0,
            bone1,
            weight0,
            weight1,
        ) = skin_weights_for_arc(
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


def joint_names(count):
    names = ["Bone0"]

    for i in range(
        1,
        count,
    ):
        names.append(
            names[-1]
            + f"/Bone{i}"
        )

    return names


def rest_local_transforms(
    physics,
):
    bind = physics["bind"]

    result = [
        Gf.Matrix4d(
            bind[0]
        )
    ]

    for i in range(
        1,
        len(bind),
    ):
        result.append(
            bind[i]
            * bind[i - 1].GetInverse()
        )

    return result


# ============================================================================
# COLLISION FILTERING
# ============================================================================

def add_collision_filter(
    stage,
    body_a_path,
    body_b_path,
):
    """
    Same principle used by exporterV2/core/usd/collision.py.

    Filtering is authored on the rigid-body prim and therefore propagates
    to all compound capsule shapes belonging to that body.
    """
    prim_a = stage.GetPrimAtPath(
        body_a_path
    )
    prim_b = stage.GetPrimAtPath(
        body_b_path
    )

    if (
        prim_a
        and prim_a.IsValid()
        and prim_b
        and prim_b.IsValid()
    ):
        filtered = (
            UsdPhysics
            .FilteredPairsAPI
            .Apply(
                prim_a
            )
        )

        filtered.GetFilteredPairsRel().AddTarget(
            Sdf.Path(
                body_b_path
            )
        )


def add_bidirectional_collision_filter(
    stage,
    body_a_path,
    body_b_path,
):
    # One direction is normally sufficient for FilteredPairsAPI,
    # but authoring both directions makes the diagnostic explicit.
    add_collision_filter(
        stage,
        body_a_path,
        body_b_path,
    )

    add_collision_filter(
        stage,
        body_b_path,
        body_a_path,
    )


# ============================================================================
# PHYSX HELPERS
# ============================================================================

def apply_physx_scene(stage):
    path = "/World/PhysicsScene"

    scene = (
        UsdPhysics.Scene.Define(
            stage,
            path,
        )
    )

    scene.CreateGravityDirectionAttr().Set(
        Gf.Vec3f(
            0.0,
            0.0,
            -1.0,
        )
    )
    scene.CreateGravityMagnitudeAttr().Set(
        9.81
    )

    physx = (
        PhysxSchema
        .PhysxSceneAPI
        .Apply(
            scene.GetPrim()
        )
    )

    physx.CreateSolverTypeAttr().Set(
        "TGS"
    )
    physx.CreateTimeStepsPerSecondAttr().Set(
        120
    )
    physx.CreateEnableCCDAttr().Set(
        True
    )
    physx.CreateEnableStabilizationAttr().Set(
        True
    )
    physx.CreateEnableGPUDynamicsAttr().Set(
        True
    )
    physx.CreateBroadphaseTypeAttr().Set(
        "MBP"
    )

    return path


def configure_d6(
    joint,
    spec,
    stiffness=None,
    damping=None,
    bend_limit_deg=None,
):
    if stiffness is None:
        stiffness = (
            spec.joint_stiffness
        )

    if damping is None:
        damping = (
            spec.joint_damping
        )

    if bend_limit_deg is None:
        bend_limit_deg = (
            spec.bend_limit_deg
        )

    for axis in (
        "transX",
        "transY",
        "transZ",
    ):
        limit = (
            UsdPhysics
            .LimitAPI
            .Apply(
                joint.GetPrim(),
                axis,
            )
        )

        limit.CreateLowAttr().Set(
            1.0
        )
        limit.CreateHighAttr().Set(
            -1.0
        )

    for axis in (
        "rotX",
        "rotY",
    ):
        limit = (
            UsdPhysics
            .LimitAPI
            .Apply(
                joint.GetPrim(),
                axis,
            )
        )

        limit.CreateLowAttr().Set(
            -bend_limit_deg
        )
        limit.CreateHighAttr().Set(
            bend_limit_deg
        )

        drive = (
            UsdPhysics
            .DriveAPI
            .Apply(
                joint.GetPrim(),
                axis,
            )
        )

        drive.CreateTypeAttr().Set(
            "force"
        )
        drive.CreateStiffnessAttr().Set(
            stiffness
        )
        drive.CreateDampingAttr().Set(
            damping
        )
        drive.CreateTargetPositionAttr().Set(
            0.0
        )

    twist = (
        UsdPhysics
        .LimitAPI
        .Apply(
            joint.GetPrim(),
            "rotZ",
        )
    )

    twist.CreateLowAttr().Set(
        1.0
    )
    twist.CreateHighAttr().Set(
        -1.0
    )


def create_compound_capsule_link(
    stage,
    branch,
    index,
):
    spec = branch.spec
    centerline = branch.centerline
    physics = branch.physics

    path = (
        branch.link_paths[index]
    )

    xform = (
        UsdGeom.Xform.Define(
            stage,
            path,
        )
    )

    origin = (
        physics[
            "origins"
        ][index]
    )

    rotation = (
        physics[
            "rotations"
        ][index]
    )

    length = (
        physics[
            "lengths"
        ][index]
    )

    xform.AddTranslateOp().Set(
        origin
    )
    xform.AddOrientOp().Set(
        quatf_from_rotation(
            rotation
        )
    )

    if (
        spec.colliders_per_link
        < 1
    ):
        raise ValueError(
            "colliders_per_link must be >= 1."
        )

    link_bind_world = (
        physics[
            "bind"
        ][index]
    )

    link_s0 = float(
        physics[
            "node_arc"
        ][index]
    )

    link_s1 = float(
        physics[
            "node_arc"
        ][index + 1]
    )

    for collider_index in range(
        spec.colliders_per_link
    ):
        u0 = (
            collider_index
            / float(
                spec.colliders_per_link
            )
        )

        u1 = (
            (collider_index + 1)
            / float(
                spec.colliders_per_link
            )
        )

        s0 = (
            link_s0
            + (
                link_s1
                - link_s0
            ) * u0
        )

        s1 = (
            link_s0
            + (
                link_s1
                - link_s0
            ) * u1
        )

        sm = 0.5 * (
            s0 + s1
        )

        p0 = point_at_arc(
            centerline,
            s0,
        )

        p1 = point_at_arc(
            centerline,
            s1,
        )

        chord = (
            p1 - p0
        )

        chord_length = float(
            chord.GetLength()
        )

        if chord_length <= 1e-8:
            raise ValueError(
                f"{branch.name}: "
                f"zero-length collider "
                f"{index}:{collider_index}"
            )

        tangent = (
            chord
            / chord_length
        )

        collider_rotation = (
            rotation_from_tangent(
                tangent
            )
        )

        collider_mid = (
            0.5
            * (
                p0 + p1
            )
        )

        desired_total_length = (
            chord_length
            * spec.collider_length_scale
        )

        collider_radius = (
            radius_for_arc(
                spec,
                centerline,
                sm,
            )
            * spec.collider_radius_scale
        )

        capsule_spine_height = (
            desired_total_length
            - 2.0
            * collider_radius
        )

        if (
            capsule_spine_height
            <= 1e-5
        ):
            raise ValueError(
                f"{branch.name}: capsule "
                f"{index}:{collider_index} "
                f"does not fit. "
                f"total={desired_total_length:.5f}, "
                f"diameter={2.0 * collider_radius:.5f}. "
                f"Reduce colliders_per_link or radius."
            )

        collider_world = (
            pose_matrix(
                collider_mid,
                collider_rotation,
            )
        )

        collider_local = (
            local_frame_from_world(
                collider_world,
                link_bind_world,
            )
        )

        local_translation = (
            collider_local
            .ExtractTranslation()
        )

        local_rotation = (
            quatf_from_matrix(
                collider_local
            )
        )

        collider_path = (
            f"{path}/"
            f"Collider_"
            f"{collider_index + 1:02d}"
        )

        capsule = (
            UsdGeom.Capsule.Define(
                stage,
                collider_path,
            )
        )

        capsule.CreateHeightAttr().Set(
            capsule_spine_height
        )
        capsule.CreateRadiusAttr().Set(
            collider_radius
        )
        capsule.CreateAxisAttr().Set(
            "Z"
        )

        half_total = (
            0.5
            * desired_total_length
        )

        capsule.CreateExtentAttr().Set(
            Vt.Vec3fArray([
                Gf.Vec3f(
                    -collider_radius,
                    -collider_radius,
                    -half_total,
                ),
                Gf.Vec3f(
                    collider_radius,
                    collider_radius,
                    half_total,
                ),
            ])
        )

        capsule_xform = (
            UsdGeom.Xformable(
                capsule.GetPrim()
            )
        )

        capsule_xform.AddTranslateOp().Set(
            Gf.Vec3d(
                float(
                    local_translation[0]
                ),
                float(
                    local_translation[1]
                ),
                float(
                    local_translation[2]
                ),
            )
        )

        capsule_xform.AddOrientOp().Set(
            local_rotation
        )

        if (
            not spec
            .show_physics_colliders
        ):
            UsdGeom.Imageable(
                capsule.GetPrim()
            ).MakeInvisible()

        UsdPhysics.CollisionAPI.Apply(
            capsule.GetPrim()
        )

    rigid_body = (
        UsdPhysics
        .RigidBodyAPI
        .Apply(
            xform.GetPrim()
        )
    )

    rigid_body.CreateRigidBodyEnabledAttr().Set(
        True
    )

    mass_value = max(
        spec.linear_density_kg_per_m
        * length,
        1e-5,
    )

    mass_api = (
        UsdPhysics
        .MassAPI
        .Apply(
            xform.GetPrim()
        )
    )

    mass_api.CreateMassAttr().Set(
        mass_value
    )

    return path


def create_internal_joint(
    stage,
    branch,
    parent_index,
    child_index,
):
    parent_path = (
        branch.link_paths[
            parent_index
        ]
    )

    child_path = (
        branch.link_paths[
            child_index
        ]
    )

    joint_path = (
        f"{child_path}/"
        f"Joint_"
        f"{parent_index + 1:02d}_"
        f"{child_index + 1:02d}"
    )

    joint = (
        UsdPhysics.Joint.Define(
            stage,
            joint_path,
        )
    )

    joint.CreateBody0Rel().SetTargets(
        [Sdf.Path(parent_path)]
    )
    joint.CreateBody1Rel().SetTargets(
        [Sdf.Path(child_path)]
    )

    parent_length = (
        branch.physics[
            "lengths"
        ][parent_index]
    )

    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(
            0.0,
            0.0,
            float(parent_length),
        )
    )

    joint.CreateLocalPos1Attr().Set(
        Gf.Vec3f(
            0.0,
            0.0,
            0.0,
        )
    )

    joint.CreateLocalRot0Attr().Set(
        Gf.Quatf(
            1.0,
            0.0,
            0.0,
            0.0,
        )
    )

    parent_world = (
        branch.physics[
            "bind"
        ][parent_index]
    )

    child_world = (
        branch.physics[
            "bind"
        ][child_index]
    )

    child_joint_local = (
        parent_world
        * child_world.GetInverse()
    )

    joint.CreateLocalRot1Attr().Set(
        quatf_from_matrix(
            child_joint_local
        )
    )

    configure_d6(
        joint,
        branch.spec,
    )

    # Match exporterV2 behavior: the two bodies connected by an internal
    # joint must not fight each other through their collision shapes.
    add_bidirectional_collision_filter(
        stage,
        parent_path,
        child_path,
    )

    return (
        joint
        .GetPrim()
        .GetPath()
        .pathString
    )


def create_world_anchor(
    stage,
    link_path,
):
    joint = (
        UsdPhysics.FixedJoint.Define(
            stage,
            f"{link_path}/RootFixedJoint",
        )
    )

    joint.CreateBody1Rel().SetTargets(
        [Sdf.Path(link_path)]
    )

    return (
        joint.GetPrim()
        .GetPath()
        .pathString
    )


def create_junction_joint(
    stage,
    parent_branch,
    parent_link_index,
    child_branch,
    attachment_world,
    stiffness=None,
    damping=None,
    bend_limit_deg=None,
):
    """
    ExporterV2-style branch attachment.

    The joint frame is aligned with CHILD LINK 0 in the authored rest pose:

        local frame on parent = child rest frame expressed in parent coordinates
        local frame on child  = identity

    This is important for D6 semantics:
        - rotZ lock is around the lateral branch longitudinal axis
        - rotX / rotY are the branch-local bending axes

    The previous 2C-A implementation aligned the common frame to the parent
    stem instead. The two frames coincided at rest, but the D6 axes were wrong
    for a strongly tilted lateral branch.
    """
    parent_path = (
        parent_branch
        .link_paths[
            parent_link_index
        ]
    )

    child_path = (
        child_branch
        .link_paths[0]
    )

    joint_path = (
        f"{child_path}/"
        f"JunctionTo_"
        f"{parent_branch.name}_"
        f"{parent_link_index + 1:02d}"
    )

    joint = (
        UsdPhysics.Joint.Define(
            stage,
            joint_path,
        )
    )

    joint.CreateBody0Rel().SetTargets(
        [Sdf.Path(parent_path)]
    )
    joint.CreateBody1Rel().SetTargets(
        [Sdf.Path(child_path)]
    )

    parent_world = (
        parent_branch
        .physics[
            "bind"
        ][parent_link_index]
    )

    child_world = (
        child_branch
        .physics[
            "bind"
        ][0]
    )

    # Child Link 0 starts exactly at attachment_world.
    # Use CHILD orientation as the common joint-world orientation.
    child_rotation = (
        child_branch
        .physics[
            "rotations"
        ][0]
    )

    joint_world = (
        pose_matrix(
            Gf.Vec3d(
                attachment_world
            ),
            child_rotation,
        )
    )

    # Parent receives the child rest frame expressed locally.
    local0 = (
        local_frame_from_world(
            joint_world,
            parent_world,
        )
    )

    # Because joint_world is exactly child Link 0's authored rest frame,
    # this should be approximately identity + zero translation.
    local1 = (
        local_frame_from_world(
            joint_world,
            child_world,
        )
    )

    p0 = (
        local0
        .ExtractTranslation()
    )
    p1 = (
        local1
        .ExtractTranslation()
    )

    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(
            float(p0[0]),
            float(p0[1]),
            float(p0[2]),
        )
    )

    joint.CreateLocalPos1Attr().Set(
        Gf.Vec3f(
            float(p1[0]),
            float(p1[1]),
            float(p1[2]),
        )
    )

    joint.CreateLocalRot0Attr().Set(
        quatf_from_matrix(
            local0
        )
    )

    joint.CreateLocalRot1Attr().Set(
        quatf_from_matrix(
            local1
        )
    )

    configure_d6(
        joint,
        child_branch.spec,
        stiffness=stiffness,
        damping=damping,
        bend_limit_deg=bend_limit_deg,
    )

    # Match exporterV2 attachment collision filtering.
    #
    # The visual branch intentionally starts inside/at the parent volume,
    # therefore physical collision between the attached bodies would create
    # an impossible depenetration constraint at simulation start.
    filter_indices = {
        parent_link_index,
    }

    if parent_link_index > 0:
        filter_indices.add(
            parent_link_index - 1
        )

    if (
        parent_link_index + 1
        < len(
            parent_branch.link_paths
        )
    ):
        filter_indices.add(
            parent_link_index + 1
        )

    for index in sorted(
        filter_indices
    ):
        add_bidirectional_collision_filter(
            stage,
            child_path,
            parent_branch
            .link_paths[index],
        )

    return {
        "path": (
            joint
            .GetPrim()
            .GetPath()
            .pathString
        ),
        "parent_local": local0,
        "child_local": local1,
        "filtered_parent_indices": sorted(
            filter_indices
        ),
    }


# ============================================================================
# BRANCH CREATION
# ============================================================================

def make_branch_data(
    name,
    spec,
    physics_parent_path="/World/PlantPhysics",
    visual_parent_path="/World/PlantVisual",
):
    centerline = (
        build_smooth_centerline(
            spec
        )
    )

    physics = (
        build_physics_discretization(
            spec,
            centerline,
        )
    )

    (
        normals,
        binormals,
    ) = build_transport_frames(
        centerline
    )

    physics_root_path = (
        f"{physics_parent_path}/"
        f"{name}"
    )

    visual_root_path = (
        f"{visual_parent_path}/"
        f"{name}"
    )

    link_paths = [
        (
            f"{physics_root_path}/"
            f"Link_{i + 1:02d}"
        )
        for i in range(
            spec.physics_links
        )
    ]

    skel_root_path = (
        f"{visual_root_path}/"
        f"SkelRoot"
    )

    skeleton_path = (
        f"{skel_root_path}/"
        f"Skeleton"
    )

    animation_path = (
        f"{skel_root_path}/"
        f"SkelAnim"
    )

    mesh_path = (
        f"{skel_root_path}/"
        f"BranchMesh"
    )

    return BranchData(
        name=name,
        spec=spec,
        centerline=centerline,
        physics=physics,
        normals=normals,
        binormals=binormals,
        physics_root_path=physics_root_path,
        visual_root_path=visual_root_path,
        link_paths=link_paths,
        skel_root_path=skel_root_path,
        skeleton_path=skeleton_path,
        animation_path=animation_path,
        mesh_path=mesh_path,
    )


def build_branch_physics(
    stage,
    branch,
):
    UsdGeom.Xform.Define(
        stage,
        branch.physics_root_path,
    )

    for index in range(
        branch.spec.physics_links
    ):
        create_compound_capsule_link(
            stage,
            branch,
            index,
        )

        if index > 0:
            create_internal_joint(
                stage,
                branch,
                index - 1,
                index,
            )

    return (
        list(
            branch.link_paths
        )
    )


def build_branch_visual(
    stage,
    branch,
    color,
):
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

    names = joint_names(
        branch.spec.physics_links
    )

    bind = (
        branch.physics["bind"]
    )

    rest = (
        rest_local_transforms(
            branch.physics
        )
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
        t = (
            matrix
            .ExtractTranslation()
        )

        q = (
            matrix
            .ExtractRotationQuat()
        )

        qi = (
            q.GetImaginary()
        )

        translations.append(
            Gf.Vec3f(
                float(t[0]),
                float(t[1]),
                float(t[2]),
            )
        )

        rotations.append(
            Gf.Quatf(
                float(
                    q.GetReal()
                ),
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
    ) = build_tube_data(
        branch.spec,
        branch.centerline,
        branch.physics,
        branch.normals,
        branch.binormals,
    )

    mesh.CreatePointsAttr().Set(
        Vt.Vec3fArray(
            points
        )
    )

    mesh.CreateFaceVertexCountsAttr().Set(
        Vt.IntArray(
            face_counts
        )
    )

    mesh.CreateFaceVertexIndicesAttr().Set(
        Vt.IntArray(
            face_indices
        )
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

    return (
        branch.animation_path
    )


def validate_branch_connectivity(
    branch,
    tolerance=1e-7,
):
    max_error = 0.0

    for i in range(
        branch.spec.physics_links - 1
    ):
        parent_end = (
            branch.physics[
                "origins"
            ][i]
            + branch.physics[
                "tangents"
            ][i]
            * branch.physics[
                "lengths"
            ][i]
        )

        child_origin = (
            branch.physics[
                "origins"
            ][i + 1]
        )

        error = float(
            (
                parent_end
                - child_origin
            ).GetLength()
        )

        max_error = max(
            max_error,
            error,
        )

    if max_error > tolerance:
        raise RuntimeError(
            f"{branch.name}: "
            f"link connectivity error "
            f"{max_error}"
        )

    return max_error
