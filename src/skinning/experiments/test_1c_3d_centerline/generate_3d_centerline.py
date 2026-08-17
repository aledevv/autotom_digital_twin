"""
generate_3d_centerline.py — Test 1C

True 3D rest centerline + smooth sweep + taper + PhysX + UsdSkel.

Validated pieces reused:
- Test 0F: PhysX articulation, D6 rest frames, Skeleton bind/rest logic.
- Test 1A: smooth Hermite centerline + parallel-transport frames.
- Test 1B: smooth taper.

New in 1C: the rest centerline itself varies in X, Y and Z.
"""

import importlib.util
import math
import os
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent

candidates = list(EXPERIMENTS_DIR.glob("test_0f*/generate_curved_centerline.py"))
if not candidates:
    raise ImportError(f"Cannot find Test 0F under {EXPERIMENTS_DIR}")

TEST_0F_PATH = candidates[0]
spec = importlib.util.spec_from_file_location("test_0f_generate_curved_centerline", TEST_0F_PATH)
if spec is None or spec.loader is None:
    raise ImportError(f"Could not load Test 0F from {TEST_0F_PATH}")
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
print(f"[DEBUG] Test 0F imported from: {TEST_0F_PATH}")

OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_USD = str(OUTPUT_DIR / "test_1c_true_3d_centerline.usda")

NUM_LINKS = base.NUM_LINKS
NUM_BONES = base.NUM_BONES
LINK_HEIGHT = base.LINK_HEIGHT
GAP = base.GAP
BASE_Z = base.BASE_Z

SAMPLES_PER_SEGMENT = 18
RADIAL_SEGMENTS = 16
BLEND_HALF_WIDTH = LINK_HEIGHT * 0.18

REFERENCE_RADIUS = float(base.TUBE_RADIUS)
BASE_RADIUS = REFERENCE_RADIUS * 1.20
TIP_RADIUS = REFERENCE_RADIUS * 0.70
TAPER_START = 0.05
TAPER_END = 0.95

# yaw: 0° = +Y, positive -> +X
# pitch: positive -> +Z, negative -> -Z
REST_YAW_DEG = [0.0, 25.0, 45.0]
REST_PITCH_DEG = [15.0, 25.0, -10.0]


def normalize(v):
    v = Gf.Vec3d(v)
    length = float(v.GetLength())
    if length < 1e-10:
        raise ValueError("zero-length vector")
    return v / length


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def tangent_from_yaw_pitch(yaw_deg, pitch_deg):
    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    return normalize(Gf.Vec3d(
        math.sin(yaw) * math.cos(pitch),
        math.cos(yaw) * math.cos(pitch),
        math.sin(pitch),
    ))


def build_3d_centerline():
    if len(REST_YAW_DEG) != NUM_LINKS or len(REST_PITCH_DEG) != NUM_LINKS:
        raise ValueError("3D rest angle array size mismatch")

    tangents = [
        tangent_from_yaw_pitch(yaw, pitch)
        for yaw, pitch in zip(REST_YAW_DEG, REST_PITCH_DEG)
    ]
    rotations = [base.rotation_from_tangent(t) for t in tangents]

    step = LINK_HEIGHT + GAP
    origins = [Gf.Vec3d(0.0, 0.0, BASE_Z)]
    for i in range(1, NUM_LINKS):
        origins.append(origins[-1] + tangents[i - 1] * step)

    bind = [base.pose_matrix(origins[i], rotations[i]) for i in range(NUM_LINKS)]
    tip = origins[-1] + tangents[-1] * LINK_HEIGHT
    nodes = list(origins) + [tip]

    arc = [0.0]
    for i in range(1, len(nodes)):
        arc.append(arc[-1] + float((nodes[i] - nodes[i - 1]).GetLength()))

    return dict(
        tangents=tangents,
        rotations=rotations,
        origins=origins,
        bind=bind,
        nodes=nodes,
        arc=arc,
    )


CENTERLINE = build_3d_centerline()
# All validated 0F physics/skeleton functions now use this 3D source of truth.
base.CENTERLINE = CENTERLINE

CONTROL_POINTS = [Gf.Vec3d(p) for p in CENTERLINE["nodes"]]


def control_tangents(points):
    out = []
    for i in range(len(points)):
        if i == 0:
            m = points[1] - points[0]
        elif i == len(points) - 1:
            m = points[-1] - points[-2]
        else:
            m = 0.5 * (points[i + 1] - points[i - 1])
        out.append(Gf.Vec3d(m))
    return out


CONTROL_TANGENTS = control_tangents(CONTROL_POINTS)


def hermite_point(p0, p1, m0, m1, u):
    u2, u3 = u * u, u * u * u
    return (
        p0 * (2*u3 - 3*u2 + 1)
        + m0 * (u3 - 2*u2 + u)
        + p1 * (-2*u3 + 3*u2)
        + m1 * (u3 - u2)
    )


def hermite_derivative(p0, p1, m0, m1, u):
    u2 = u * u
    return (
        p0 * (6*u2 - 6*u)
        + m0 * (3*u2 - 4*u + 1)
        + p1 * (-6*u2 + 6*u)
        + m1 * (3*u2 - 2*u)
    )


def sample_centerline():
    positions, tangents = [], []
    control_sample_indices = [None] * len(CONTROL_POINTS)

    for seg in range(len(CONTROL_POINTS) - 1):
        p0, p1 = CONTROL_POINTS[seg], CONTROL_POINTS[seg + 1]
        m0, m1 = CONTROL_TANGENTS[seg], CONTROL_TANGENTS[seg + 1]
        for j in range(SAMPLES_PER_SEGMENT):
            u = j / float(SAMPLES_PER_SEGMENT)
            if j == 0:
                control_sample_indices[seg] = len(positions)
            positions.append(Gf.Vec3d(hermite_point(p0, p1, m0, m1, u)))
            tangents.append(normalize(hermite_derivative(p0, p1, m0, m1, u)))

    positions.append(Gf.Vec3d(CONTROL_POINTS[-1]))
    tangents.append(normalize(CONTROL_TANGENTS[-1]))
    control_sample_indices[-1] = len(positions) - 1

    arc = [0.0]
    for i in range(1, len(positions)):
        arc.append(arc[-1] + float((positions[i] - positions[i - 1]).GetLength()))

    return dict(
        positions=positions,
        tangents=tangents,
        arc=arc,
        control_arc=[arc[i] for i in control_sample_indices],
    )


SMOOTH = sample_centerline()


def initial_normal(tangent):
    preferred = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(float(Gf.Dot(preferred, tangent))) > 0.95:
        preferred = Gf.Vec3d(1.0, 0.0, 0.0)
    return normalize(preferred - tangent * Gf.Dot(preferred, tangent))


def rodrigues(v, axis, angle):
    axis = normalize(axis)
    c, s = math.cos(angle), math.sin(angle)
    return v*c + Gf.Cross(axis, v)*s + axis*(Gf.Dot(axis, v)*(1.0-c))


def transport_normal(prev_t, curr_t, prev_n):
    prev_t, curr_t = normalize(prev_t), normalize(curr_t)
    axis = Gf.Cross(prev_t, curr_t)
    sin_angle = float(axis.GetLength())
    cos_angle = clamp(float(Gf.Dot(prev_t, curr_t)), -1.0, 1.0)

    if sin_angle < 1e-9:
        n = Gf.Vec3d(prev_n)
    else:
        n = rodrigues(prev_n, axis / sin_angle, math.atan2(sin_angle, cos_angle))

    n = n - curr_t * Gf.Dot(n, curr_t)
    return initial_normal(curr_t) if n.GetLength() < 1e-9 else normalize(n)


def build_transport_frames():
    tangents = SMOOTH["tangents"]
    normals = [initial_normal(tangents[0])]
    for i in range(1, len(tangents)):
        normals.append(transport_normal(tangents[i - 1], tangents[i], normals[-1]))
    binormals = [normalize(Gf.Cross(t, n)) for t, n in zip(tangents, normals)]
    return normals, binormals


FRAME_NORMALS, FRAME_BINORMALS = build_transport_frames()


def weights_for_arc(s):
    bone_arc = SMOOTH["control_arc"][:NUM_BONES]
    for child in range(1, NUM_BONES):
        joint_s = bone_arc[child]
        lo, hi = joint_s - BLEND_HALF_WIDTH, joint_s + BLEND_HALF_WIDTH
        if lo <= s <= hi:
            u = clamp((s - lo) / (hi - lo), 0.0, 1.0)
            return child - 1, child, 1.0 - u, u

    bone = 0
    for i in range(1, NUM_BONES):
        if s >= bone_arc[i]:
            bone = i
        else:
            break
    return bone, bone, 1.0, 0.0


def smoothstep01(u):
    u = clamp(float(u), 0.0, 1.0)
    return u*u*(3.0 - 2.0*u)


def radius_for_arc(s):
    total = float(SMOOTH["arc"][-1])
    if total <= 1e-12:
        return BASE_RADIUS
    u = clamp(float(s) / total, 0.0, 1.0)
    if u <= TAPER_START:
        return BASE_RADIUS
    if u >= TAPER_END:
        return TIP_RADIUS
    local_u = (u - TAPER_START) / (TAPER_END - TAPER_START)
    blend = smoothstep01(local_u)
    return BASE_RADIUS + (TIP_RADIUS - BASE_RADIUS) * blend


def build_3d_tube_data():
    points, joint_indices, joint_weights = [], [], []

    for ring, center in enumerate(SMOOTH["positions"]):
        n = FRAME_NORMALS[ring]
        b = FRAME_BINORMALS[ring]
        s = float(SMOOTH["arc"][ring])
        radius = radius_for_arc(s)
        bone0, bone1, w0, w1 = weights_for_arc(s)

        for k in range(RADIAL_SEGMENTS):
            theta = 2.0 * math.pi * k / RADIAL_SEGMENTS
            radial = n*(math.cos(theta)*radius) + b*(math.sin(theta)*radius)
            p = center + radial
            points.append(Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
            joint_indices.extend([bone0, bone1])
            joint_weights.extend([w0, w1])

    face_counts, face_indices = [], []
    for ring in range(len(SMOOTH["positions"]) - 1):
        row0, row1 = ring * RADIAL_SEGMENTS, (ring + 1) * RADIAL_SEGMENTS
        for k in range(RADIAL_SEGMENTS):
            kn = (k + 1) % RADIAL_SEGMENTS
            v00, v01 = row0 + k, row0 + kn
            v10, v11 = row1 + k, row1 + kn
            face_counts.extend([3, 3])
            face_indices.extend([v00, v10, v11, v00, v11, v01])

    return points, face_counts, face_indices, joint_indices, joint_weights


# Only visual mesh generation is replaced; physics/skeleton infrastructure remains 0F.
base.build_tube_data = build_3d_tube_data
rest_local_transforms = base.rest_local_transforms


def build_stage(output_path=OUTPUT_USD):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    base.apply_physx_scene(stage)
    base.build_physics(stage)
    base.build_visual(stage)
    base.build_ground(stage)
    stage.Save()

    print("=" * 76)
    print("TEST 1C — True 3D Centerline")
    print(f"[OK] {output_path}")
    print()
    for i in range(NUM_LINKS):
        t = CENTERLINE["tangents"][i]
        p = CENTERLINE["origins"][i]
        print(
            f"  Link{i}: yaw={REST_YAW_DEG[i]:+.1f}°, pitch={REST_PITCH_DEG[i]:+.1f}°  "
            f"T=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})  "
            f"P=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})"
        )
    tip = CENTERLINE["nodes"][-1]
    print(f"  Tip=({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f})")
    print(f"  Visual rings={len(SMOOTH['positions'])}")
    print(f"  Radius base/tip={BASE_RADIUS*1000:.1f}/{TIP_RADIUS*1000:.1f} mm")
    print("=" * 76)
    return output_path


if __name__ == "__main__":
    build_stage()
