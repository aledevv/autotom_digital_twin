"""
generate_parametric_branch.py — Test 2A

Standalone parametric branch generator.

Input: BranchSpec
  - arbitrary 3D control points
  - number of physical links / bones
  - visual sampling resolution
  - radius profile
  - PhysX parameters

Automatically generates:
  - smooth Hermite centerline
  - arc-length parameterization
  - parallel-transport frames
  - tapered / swollen radius profile
  - continuous skinned mesh
  - rigid links sampled from the same centerline
  - D6 joints
  - skeleton bind/rest transforms
  - skin weights

Runtime bridge: run_parametric_branch.py
"""

from dataclasses import dataclass, field
import math
import os
from pathlib import Path

from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdSkel, Vt


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_USD = str(OUTPUT_DIR / "test_2b_compound_collision.usda")


# ============================================================================
# USER-FACING SPEC
# ============================================================================

@dataclass(frozen=True)
class RadiusProfile:
    base_radius: float = 0.016
    tip_radius: float = 0.008
    taper_start: float = 0.04
    taper_end: float = 0.96

    # Visual/botanical swellings are independent from the D6 positions.
    swell_fractions: tuple[float, ...] = (0.32, 0.66)
    swell_amplitude: float = 0.18
    swell_sigma_fraction: float = 0.035

    micro_variation_amplitude: float = 0.025
    micro_variation_cycles: float = 2.3


@dataclass(frozen=True)
class BranchSpec:
    control_points: tuple[tuple[float, float, float], ...]

    # Physics resolution.
    physics_links: int = 6

    # Visual resolution, independent from physics resolution.
    samples_per_control_segment: int = 24
    radial_segments: int = 16

    radius: RadiusProfile = field(default_factory=RadiusProfile)

    linear_density_kg_per_m: float = 0.20
    collider_radius_scale: float = 0.90
    colliders_per_link: int = 3
    collider_length_scale: float = 0.82

    joint_stiffness: float = 0.0
    joint_damping: float = 0.035
    bend_limit_deg: float = 90.0

    # Total blend-zone width is this fraction of the mean physics-link arc length.
    skin_blend_fraction: float = 0.32

    show_physics_colliders: bool = True


# This is the only object that should need changing for Test 2A.
BRANCH_SPEC = BranchSpec(
    control_points=(
        (0.000, 0.000, 0.120),
        (0.010, 0.090, 0.145),
        (0.045, 0.185, 0.185),
        (0.095, 0.270, 0.175),
        (0.135, 0.345, 0.135),
    ),
    physics_links=6,
    samples_per_control_segment=24,
    radial_segments=16,
    radius=RadiusProfile(
        base_radius=0.016,
        tip_radius=0.008,
        swell_fractions=(0.32, 0.66),
        swell_amplitude=0.18,
        swell_sigma_fraction=0.035,
        micro_variation_amplitude=0.025,
        micro_variation_cycles=2.3,
    ),
    linear_density_kg_per_m=0.20,
    collider_radius_scale=0.90,
    colliders_per_link=3,
    collider_length_scale=0.82,
    joint_stiffness=0.0,
    joint_damping=0.035,
    bend_limit_deg=65.0,
    skin_blend_fraction=0.32,
    show_physics_colliders=True,
)


# ============================================================================
# BASIC MATH
# ============================================================================

def normalize(v: Gf.Vec3d) -> Gf.Vec3d:
    v = Gf.Vec3d(v)
    length = float(v.GetLength())
    if length < 1e-10:
        raise ValueError("Cannot normalize zero-length vector")
    return v / length


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(x)))


def smoothstep01(u: float) -> float:
    u = clamp(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def rotation_from_tangent(tangent: Gf.Vec3d) -> Gf.Rotation:
    # Every rigid link / bone uses local +Z as longitudinal axis.
    return Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), normalize(tangent))


def quatf_from_rotation(rot: Gf.Rotation) -> Gf.Quatf:
    q = rot.GetQuat()
    qi = q.GetImaginary()
    return Gf.Quatf(
        float(q.GetReal()),
        Gf.Vec3f(float(qi[0]), float(qi[1]), float(qi[2])),
    )


def quatf_from_matrix(m: Gf.Matrix4d) -> Gf.Quatf:
    q = m.ExtractRotationQuat()
    qi = q.GetImaginary()
    return Gf.Quatf(
        float(q.GetReal()),
        Gf.Vec3f(float(qi[0]), float(qi[1]), float(qi[2])),
    )


def pose_matrix(position: Gf.Vec3d, rotation: Gf.Rotation) -> Gf.Matrix4d:
    m = Gf.Matrix4d(1.0)
    m.SetTransform(rotation, position)
    return m


# ============================================================================
# SMOOTH CENTERLINE
# ============================================================================

def _control_points(spec: BranchSpec) -> list[Gf.Vec3d]:
    if len(spec.control_points) < 2:
        raise ValueError("At least 2 control points are required")
    return [Gf.Vec3d(*p) for p in spec.control_points]


def _control_tangents(points: list[Gf.Vec3d]) -> list[Gf.Vec3d]:
    result = []
    for i in range(len(points)):
        if i == 0:
            m = points[1] - points[0]
        elif i == len(points) - 1:
            m = points[-1] - points[-2]
        else:
            m = 0.5 * (points[i + 1] - points[i - 1])
        result.append(Gf.Vec3d(m))
    return result


def _hermite_point(p0, p1, m0, m1, u: float) -> Gf.Vec3d:
    u2 = u * u
    u3 = u2 * u
    h00 = 2.0 * u3 - 3.0 * u2 + 1.0
    h10 = u3 - 2.0 * u2 + u
    h01 = -2.0 * u3 + 3.0 * u2
    h11 = u3 - u2
    return Gf.Vec3d(p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11)


def _hermite_derivative(p0, p1, m0, m1, u: float) -> Gf.Vec3d:
    u2 = u * u
    return Gf.Vec3d(
        p0 * (6.0 * u2 - 6.0 * u)
        + m0 * (3.0 * u2 - 4.0 * u + 1.0)
        + p1 * (-6.0 * u2 + 6.0 * u)
        + m1 * (3.0 * u2 - 2.0 * u)
    )


def build_smooth_centerline(spec: BranchSpec) -> dict:
    controls = _control_points(spec)
    derivatives = _control_tangents(controls)

    positions = []
    tangents = []
    control_sample_indices = [None] * len(controls)

    for seg in range(len(controls) - 1):
        p0, p1 = controls[seg], controls[seg + 1]
        m0, m1 = derivatives[seg], derivatives[seg + 1]

        for j in range(spec.samples_per_control_segment):
            u = j / float(spec.samples_per_control_segment)
            if j == 0:
                control_sample_indices[seg] = len(positions)

            p = _hermite_point(p0, p1, m0, m1, u)
            d = _hermite_derivative(p0, p1, m0, m1, u)

            positions.append(p)
            tangents.append(normalize(d))

    positions.append(Gf.Vec3d(controls[-1]))
    tangents.append(normalize(derivatives[-1]))
    control_sample_indices[-1] = len(positions) - 1

    arc = [0.0]
    for i in range(1, len(positions)):
        arc.append(arc[-1] + float((positions[i] - positions[i - 1]).GetLength()))

    return {
        "controls": controls,
        "positions": positions,
        "tangents": tangents,
        "arc": arc,
        "control_arc": [arc[i] for i in control_sample_indices],
        "total_length": float(arc[-1]),
    }


# ============================================================================
# ARC-LENGTH SAMPLING
# ============================================================================

def point_at_arc(centerline: dict, s: float) -> Gf.Vec3d:
    positions = centerline["positions"]
    arc = centerline["arc"]
    total = float(arc[-1])

    s = clamp(s, 0.0, total)
    if s <= 0.0:
        return Gf.Vec3d(positions[0])
    if s >= total:
        return Gf.Vec3d(positions[-1])

    for i in range(len(arc) - 1):
        if arc[i] <= s <= arc[i + 1]:
            ds = arc[i + 1] - arc[i]
            if ds <= 1e-12:
                return Gf.Vec3d(positions[i])
            u = (s - arc[i]) / ds
            return Gf.Vec3d(positions[i] + (positions[i + 1] - positions[i]) * u)

    return Gf.Vec3d(positions[-1])


# ============================================================================
# AUTOMATIC PHYSICS / BONE DISCRETIZATION
# ============================================================================

def build_physics_discretization(spec: BranchSpec, centerline: dict) -> dict:
    if spec.physics_links < 1:
        raise ValueError("physics_links must be >= 1")

    total = float(centerline["total_length"])
    node_arc = [total * i / spec.physics_links for i in range(spec.physics_links + 1)]
    nodes = [point_at_arc(centerline, s) for s in node_arc]

    origins = []
    tangents = []
    lengths = []
    rotations = []
    bind = []

    # IMPORTANT: physics links are straight CHORDS between samples of the smooth curve.
    # Therefore the end of parent i coincides exactly with origin of child i+1.
    for i in range(spec.physics_links):
        p0, p1 = nodes[i], nodes[i + 1]
        chord = p1 - p0
        length = float(chord.GetLength())
        if length <= 1e-8:
            raise ValueError(f"Generated physics link {i} has zero length")

        tangent = chord / length
        rotation = rotation_from_tangent(tangent)

        origins.append(Gf.Vec3d(p0))
        tangents.append(Gf.Vec3d(tangent))
        lengths.append(length)
        rotations.append(rotation)
        bind.append(pose_matrix(p0, rotation))

    return {
        "node_arc": node_arc,
        "nodes": nodes,
        "origins": origins,
        "tangents": tangents,
        "lengths": lengths,
        "rotations": rotations,
        "bind": bind,
    }


# ============================================================================
# PARALLEL TRANSPORT
# ============================================================================

def _initial_normal(tangent: Gf.Vec3d) -> Gf.Vec3d:
    preferred = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(float(Gf.Dot(preferred, tangent))) > 0.95:
        preferred = Gf.Vec3d(1.0, 0.0, 0.0)
    n = preferred - tangent * Gf.Dot(preferred, tangent)
    return normalize(n)


def _rodrigues(v, axis, angle) -> Gf.Vec3d:
    axis = normalize(axis)
    c, s = math.cos(angle), math.sin(angle)
    return Gf.Vec3d(
        v * c
        + Gf.Cross(axis, v) * s
        + axis * (Gf.Dot(axis, v) * (1.0 - c))
    )


def _transport_normal(prev_t, curr_t, prev_n) -> Gf.Vec3d:
    prev_t = normalize(prev_t)
    curr_t = normalize(curr_t)

    axis = Gf.Cross(prev_t, curr_t)
    sin_angle = float(axis.GetLength())
    cos_angle = clamp(float(Gf.Dot(prev_t, curr_t)), -1.0, 1.0)

    if sin_angle < 1e-9:
        n = Gf.Vec3d(prev_n)
    else:
        angle = math.atan2(sin_angle, cos_angle)
        n = _rodrigues(prev_n, axis / sin_angle, angle)

    n = n - curr_t * Gf.Dot(n, curr_t)
    if n.GetLength() < 1e-9:
        return _initial_normal(curr_t)
    return normalize(n)


def build_transport_frames(centerline: dict):
    tangents = centerline["tangents"]
    normals = [_initial_normal(tangents[0])]

    for i in range(1, len(tangents)):
        normals.append(_transport_normal(tangents[i - 1], tangents[i], normals[-1]))

    binormals = [normalize(Gf.Cross(t, n)) for t, n in zip(tangents, normals)]
    return normals, binormals


# ============================================================================
# RADIUS PROFILE
# ============================================================================

def taper_radius(spec: BranchSpec, centerline: dict, s: float) -> float:
    p = spec.radius
    total = float(centerline["total_length"])
    if total <= 1e-12:
        return p.base_radius

    u_global = clamp(s / total, 0.0, 1.0)
    if u_global <= p.taper_start:
        return p.base_radius
    if u_global >= p.taper_end:
        return p.tip_radius

    u = (u_global - p.taper_start) / (p.taper_end - p.taper_start)
    blend = smoothstep01(u)
    return p.base_radius + (p.tip_radius - p.base_radius) * blend


def radius_for_arc(spec: BranchSpec, centerline: dict, s: float) -> float:
    p = spec.radius
    total = float(centerline["total_length"])
    base = taper_radius(spec, centerline, s)
    if total <= 1e-12:
        return base

    swell_factor = 1.0
    sigma = max(total * p.swell_sigma_fraction, 1e-6)
    for fraction in p.swell_fractions:
        center = total * clamp(fraction, 0.0, 1.0)
        x = (s - center) / sigma
        swell_factor += p.swell_amplitude * math.exp(-0.5 * x * x)

    u = clamp(s / total, 0.0, 1.0)
    envelope = math.sin(math.pi * u) ** 2
    phase1 = 2.0 * math.pi * p.micro_variation_cycles * u
    phase2 = 2.0 * math.pi * (p.micro_variation_cycles * 0.47) * u + 0.8
    signal = 0.70 * math.sin(phase1) + 0.30 * math.sin(phase2)
    micro = 1.0 + p.micro_variation_amplitude * envelope * signal

    return base * swell_factor * micro


# ============================================================================
# SKIN WEIGHTS
# ============================================================================

def skin_weights_for_arc(spec: BranchSpec, centerline: dict, physics: dict, s: float):
    node_arc = physics["node_arc"]
    mean_arc_link = centerline["total_length"] / spec.physics_links
    half_width = mean_arc_link * spec.skin_blend_fraction * 0.5

    for child in range(1, spec.physics_links):
        center = node_arc[child]
        lo, hi = center - half_width, center + half_width
        if lo <= s <= hi:
            u = clamp((s - lo) / (hi - lo), 0.0, 1.0)
            return child - 1, child, 1.0 - u, u

    bone = 0
    for i in range(1, spec.physics_links):
        if s >= node_arc[i]:
            bone = i
        else:
            break

    return bone, bone, 1.0, 0.0


# ============================================================================
# VISUAL MESH
# ============================================================================

def build_tube_data(spec, centerline, physics, normals, binormals):
    points = []
    joint_indices = []
    joint_weights = []

    for ring, center in enumerate(centerline["positions"]):
        normal = normals[ring]
        binormal = binormals[ring]
        s = float(centerline["arc"][ring])
        radius = radius_for_arc(spec, centerline, s)
        b0, b1, w0, w1 = skin_weights_for_arc(spec, centerline, physics, s)

        for k in range(spec.radial_segments):
            theta = 2.0 * math.pi * k / spec.radial_segments
            radial = (
                normal * (math.cos(theta) * radius)
                + binormal * (math.sin(theta) * radius)
            )
            p = center + radial
            points.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
            joint_indices.extend([b0, b1])
            joint_weights.extend([w0, w1])

    face_counts = []
    face_indices = []
    for ring in range(len(centerline["positions"]) - 1):
        row0 = ring * spec.radial_segments
        row1 = (ring + 1) * spec.radial_segments
        for k in range(spec.radial_segments):
            kn = (k + 1) % spec.radial_segments
            v00, v01 = row0 + k, row0 + kn
            v10, v11 = row1 + k, row1 + kn
            face_counts.extend([3, 3])
            face_indices.extend([v00, v10, v11, v00, v11, v01])

    return points, face_counts, face_indices, joint_indices, joint_weights


# ============================================================================
# SKELETON
# ============================================================================

def joint_names(count: int) -> list[str]:
    names = ["Bone0"]
    for i in range(1, count):
        names.append(names[-1] + f"/Bone{i}")
    return names


def rest_local_transforms(physics: dict | None = None):
    if physics is None:
        physics = PHYSICS

    bind = physics["bind"]
    out = [Gf.Matrix4d(bind[0])]
    for i in range(1, len(bind)):
        # Gf/OpenUSD row-vector convention.
        out.append(bind[i] * bind[i - 1].GetInverse())
    return out


# ============================================================================
# PHYSX
# ============================================================================

def apply_physx_scene(stage) -> str:
    scene_path = "/World/PhysicsScene"
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(120)
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateEnableGPUDynamicsAttr().Set(True)
    physx.CreateBroadphaseTypeAttr().Set("MBP")
    return scene_path


def create_rigid_link(stage, spec, centerline, physics, stem_path, index) -> str:
    """
    Create ONE rigid body / bone link, but attach multiple collision shapes
    sampled from the original smooth centerline interval belonging to it.

    This is the key Test 2B-A change:
        articulation resolution != collision resolution
    """
    path = f"{stem_path}/Branch_Link_{index + 1:02d}"
    xf = UsdGeom.Xform.Define(stage, path)

    origin = physics["origins"][index]
    rotation = physics["rotations"][index]
    length = physics["lengths"][index]

    xf.AddTranslateOp().Set(origin)
    xf.AddOrientOp().Set(quatf_from_rotation(rotation))

    if spec.colliders_per_link < 1:
        raise ValueError("colliders_per_link must be >= 1")

    # The rigid body's authored world/rest transform.
    link_bind_world = physics["bind"][index]

    # Arc-length interval owned by this rigid link.
    link_s0 = float(physics["node_arc"][index])
    link_s1 = float(physics["node_arc"][index + 1])

    for collider_i in range(spec.colliders_per_link):
        u0 = collider_i / float(spec.colliders_per_link)
        u1 = (collider_i + 1) / float(spec.colliders_per_link)

        s0 = link_s0 + (link_s1 - link_s0) * u0
        s1 = link_s0 + (link_s1 - link_s0) * u1
        sm = 0.5 * (s0 + s1)

        p0 = point_at_arc(centerline, s0)
        p1 = point_at_arc(centerline, s1)

        chord = p1 - p0
        chord_length = float(chord.GetLength())

        if chord_length <= 1e-8:
            raise ValueError(
                f"Generated collider {index}:{collider_i} has zero length"
            )

        collider_tangent = chord / chord_length
        collider_rotation = rotation_from_tangent(collider_tangent)
        collider_mid = 0.5 * (p0 + p1)

        # Deliberately leave a small gap between adjacent collision shapes.
        collider_length = chord_length * spec.collider_length_scale

        collider_radius = (
            radius_for_arc(spec, centerline, sm)
            * spec.collider_radius_scale
        )

        # Desired collider transform in world/rest space.
        collider_world = pose_matrix(
            collider_mid,
            collider_rotation,
        )

        # Convert desired world pose to the rigid-link local frame.
        # Gf/OpenUSD row-vector convention.
        collider_local = (
            collider_world
            * link_bind_world.GetInverse()
        )

        local_t = collider_local.ExtractTranslation()
        local_q = quatf_from_matrix(collider_local)

        collider_path = (
            f"{path}/Collider_{collider_i + 1:02d}"
        )

        collider = UsdGeom.Cylinder.Define(
            stage,
            collider_path,
        )

        collider.CreateHeightAttr().Set(
            collider_length
        )
        collider.CreateRadiusAttr().Set(
            collider_radius
        )
        collider.CreateAxisAttr().Set(
            "Z"
        )

        collider_xf = UsdGeom.Xformable(
            collider.GetPrim()
        )

        collider_xf.AddTranslateOp().Set(
            Gf.Vec3d(
                float(local_t[0]),
                float(local_t[1]),
                float(local_t[2]),
            )
        )
        collider_xf.AddOrientOp().Set(
            local_q
        )

        if not spec.show_physics_colliders:
            UsdGeom.Imageable(
                collider.GetPrim()
            ).MakeInvisible()

        UsdPhysics.CollisionAPI.Apply(
            collider.GetPrim()
        )

    # Still exactly ONE rigid body for the complete compound collision proxy.
    UsdPhysics.RigidBodyAPI.Apply(
        xf.GetPrim()
    ).CreateRigidBodyEnabledAttr().Set(
        True
    )

    mass_value = max(
        spec.linear_density_kg_per_m * length,
        1e-5,
    )

    UsdPhysics.MassAPI.Apply(
        xf.GetPrim()
    ).CreateMassAttr().Set(
        mass_value
    )

    return path


def anchor_root(stage, link_path: str) -> None:
    joint = UsdPhysics.FixedJoint.Define(stage, f"{link_path}/RootFixedJoint")
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def configure_d6(joint, spec: BranchSpec) -> None:
    for axis in ("transX", "transY", "transZ"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)

    for axis in ("rotX", "rotY"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(-spec.bend_limit_deg)
        limit.CreateHighAttr().Set(spec.bend_limit_deg)

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(spec.joint_stiffness)
        drive.CreateDampingAttr().Set(spec.joint_damping)
        drive.CreateTargetPositionAttr().Set(0.0)

    limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    limit.CreateLowAttr().Set(1.0)
    limit.CreateHighAttr().Set(-1.0)


def create_internal_joint(stage, spec, physics, parent_path, child_path, parent_i, child_i):
    joint = UsdPhysics.Joint.Define(
        stage,
        f"{child_path}/Joint_{parent_i + 1:02d}_{child_i + 1:02d}",
    )
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])

    parent_length = physics["lengths"][parent_i]
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, float(parent_length)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    parent_world = physics["bind"][parent_i]
    child_world = physics["bind"][child_i]
    child_joint_local = parent_world * child_world.GetInverse()
    joint.CreateLocalRot1Attr().Set(quatf_from_matrix(child_joint_local))

    configure_d6(joint, spec)


def build_physics(stage, spec, centerline, physics):
    stem_path = "/World/Stem"
    stem = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem.GetPrim())

    paths = []
    for i in range(spec.physics_links):
        path = create_rigid_link(stage, spec, centerline, physics, stem_path, i)
        if i == 0:
            anchor_root(stage, path)
        else:
            create_internal_joint(stage, spec, physics, paths[-1], path, i - 1, i)
        paths.append(path)

    art = PhysxSchema.PhysxArticulationAPI.Apply(stem.GetPrim())
    art.CreateSolverPositionIterationCountAttr().Set(32)
    art.CreateSolverVelocityIterationCountAttr().Set(1)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)
    return paths


# ============================================================================
# USDSKEL / VISUAL
# ============================================================================

def build_visual(stage, spec, centerline, physics, normals, binormals) -> str:
    UsdGeom.Xform.Define(stage, "/World/StemVisual")
    UsdSkel.Root.Define(stage, "/World/StemVisual/SkelRoot")

    skeleton = UsdSkel.Skeleton.Define(stage, "/World/StemVisual/SkelRoot/Skeleton")
    animation = UsdSkel.Animation.Define(stage, "/World/StemVisual/SkelRoot/SkelAnim")

    names = joint_names(spec.physics_links)
    bind = physics["bind"]
    rest = rest_local_transforms(physics)

    skeleton.CreateJointsAttr().Set(Vt.TokenArray(names))
    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest))

    animation.CreateJointsAttr().Set(Vt.TokenArray(names))

    translations = []
    rotations = []
    for m in rest:
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        qi = q.GetImaginary()
        translations.append(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
        rotations.append(
            Gf.Quatf(
                float(q.GetReal()),
                Gf.Vec3f(float(qi[0]), float(qi[1]), float(qi[2])),
            )
        )

    animation.CreateTranslationsAttr().Set(Vt.Vec3fArray(translations))
    animation.CreateRotationsAttr().Set(Vt.QuatfArray(rotations))
    animation.CreateScalesAttr().Set(
        Vt.Vec3hArray([Gf.Vec3h(1.0, 1.0, 1.0) for _ in range(spec.physics_links)])
    )

    skel_binding = UsdSkel.BindingAPI.Apply(skeleton.GetPrim())
    skel_binding.CreateAnimationSourceRel().SetTargets([animation.GetPrim().GetPath()])

    mesh = UsdGeom.Mesh.Define(stage, "/World/StemVisual/SkelRoot/BranchMesh")
    pts, fc, fi, ji, jw = build_tube_data(spec, centerline, physics, normals, binormals)

    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.28, 0.56, 0.19)]))

    mesh_binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    mesh_binding.CreateSkeletonRel().SetTargets([skeleton.GetPrim().GetPath()])
    mesh_binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))

    indices = mesh_binding.CreateJointIndicesPrimvar(constant=False, elementSize=2)
    indices.SetInterpolation(UsdGeom.Tokens.vertex)
    indices.Set(Vt.IntArray(ji))

    weights = mesh_binding.CreateJointWeightsPrimvar(constant=False, elementSize=2)
    weights.SetInterpolation(UsdGeom.Tokens.vertex)
    weights.Set(Vt.FloatArray(jw))

    return animation.GetPrim().GetPath().pathString


# ============================================================================
# GROUND
# ============================================================================

def build_ground(stage):
    ground = UsdGeom.Mesh.Define(stage, "/World/Ground")
    s = 0.8
    ground.CreatePointsAttr().Set(
        Vt.Vec3fArray([
            Gf.Vec3f(-s, -s, 0.0),
            Gf.Vec3f(s, -s, 0.0),
            Gf.Vec3f(s, s, 0.0),
            Gf.Vec3f(-s, s, 0.0),
        ])
    )
    ground.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    ground.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    ground.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    ground.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.25, 0.22, 0.18)]))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())


# ============================================================================
# DERIVED DATA
# ============================================================================

CENTERLINE = build_smooth_centerline(BRANCH_SPEC)
PHYSICS = build_physics_discretization(BRANCH_SPEC, CENTERLINE)
FRAME_NORMALS, FRAME_BINORMALS = build_transport_frames(CENTERLINE)

NUM_LINKS = BRANCH_SPEC.physics_links
NUM_BONES = NUM_LINKS
LINK_PATHS = [f"/World/Stem/Branch_Link_{i + 1:02d}" for i in range(NUM_LINKS)]


def validate_generated_branch():
    max_error = 0.0
    for i in range(BRANCH_SPEC.physics_links - 1):
        parent_end = (
            PHYSICS["origins"][i]
            + PHYSICS["tangents"][i] * PHYSICS["lengths"][i]
        )
        error = float((parent_end - PHYSICS["origins"][i + 1]).GetLength())
        max_error = max(max_error, error)

    if max_error > 1e-7:
        raise RuntimeError(f"Generated link connectivity error: {max_error}")


# ============================================================================
# BUILD STAGE
# ============================================================================

def build_stage(output_path: str = OUTPUT_USD) -> str:
    validate_generated_branch()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    apply_physx_scene(stage)
    links = build_physics(stage, BRANCH_SPEC, CENTERLINE, PHYSICS)
    anim_path = build_visual(
        stage,
        BRANCH_SPEC,
        CENTERLINE,
        PHYSICS,
        FRAME_NORMALS,
        FRAME_BINORMALS,
    )
    build_ground(stage)
    stage.Save()

    print("=" * 80)
    print("TEST 2B-A — COMPOUND COLLISION PROXY")
    print("=" * 80)
    print(f"[OK] {output_path}")
    print()
    print("BranchSpec input:")
    print(f"  control points        : {len(BRANCH_SPEC.control_points)}")
    print(f"  physics links / bones : {BRANCH_SPEC.physics_links}")
    print(f"  radial segments       : {BRANCH_SPEC.radial_segments}")
    print()
    print("Automatically generated:")
    print(f"  smooth rings          : {len(CENTERLINE['positions'])}")
    print(f"  centerline length     : {CENTERLINE['total_length']:.4f} m")
    print(f"  rigid links           : {len(links)}")
    print(f"  D6 joints             : {max(len(links) - 1, 0)}")
    print(f"  colliders / link      : {BRANCH_SPEC.colliders_per_link}")
    print(f"  collision shapes      : {len(links) * BRANCH_SPEC.colliders_per_link}")
    print(f"  visual vertices       : {len(CENTERLINE['positions']) * BRANCH_SPEC.radial_segments}")
    print()
    print("Physics discretization:")
    for i in range(NUM_LINKS):
        p = PHYSICS["origins"][i]
        t = PHYSICS["tangents"][i]
        L = PHYSICS["lengths"][i]
        print(
            f"  Link{i:02d}: L={L:.4f} m  "
            f"P=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})  "
            f"T=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})"
        )
    print()
    print("Radius profile:")
    total = CENTERLINE["total_length"]
    for frac in (0.0, 0.25, 0.50, 0.75, 1.0):
        r = radius_for_arc(BRANCH_SPEC, CENTERLINE, total * frac)
        print(f"  {frac:>4.0%}: {r * 1000.0:.2f} mm")
    print()
    print(f"SkelAnimation: {anim_path}")
    print()
    print("Main Test 2B-A check:")
    print("  articulation stays coarse while collision resolution increases.")
    print("  Expected here: 6 rigid links / 6 bones / 5 D6 / 18 collider shapes.")
    print("=" * 80)
    return output_path


if __name__ == "__main__":
    build_stage()
