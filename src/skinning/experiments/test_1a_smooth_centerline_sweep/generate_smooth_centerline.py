"""
generate_smooth_centerline.py — Test 1A

Requires the already validated Test 0F file in the same directory:
    generate_curved_centerline.py

Test purpose
------------
Keep PhysX, D6 frames, Skeleton and bind/rest pose EXACTLY as Test 0F.
Replace ONLY the visual tube generator:

    0F: polyline sweep
    1A: smooth Hermite centerline + many rings + parallel-transport frames

This isolates the geometry change from the validated physics/skinning bridge.
"""
import math
import os
import sys
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent

matches = list(
    EXPERIMENTS_DIR.glob(
        "*/generate_curved_centerline.py"
    )
)

if not matches:
    raise ImportError(
        "Cannot find generate_curved_centerline.py "
        f"under {EXPERIMENTS_DIR}"
    )

TEST_0F_DIR = matches[0].parent

if str(TEST_0F_DIR) not in sys.path:
    sys.path.insert(0, str(TEST_0F_DIR))

import generate_curved_centerline as base

print(f"[DEBUG] Test 0F imported from: {base.__file__}")
print(f"[DEBUG] CENTERLINE keys: {base.CENTERLINE.keys()}")


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
OUTPUT_USD = os.path.join(
    OUTPUT_DIR,
    "test_1a_smooth_centerline.usda",
)

# Re-export values used by run_smooth_bridge.py
NUM_LINKS = base.NUM_LINKS
NUM_BONES = base.NUM_BONES

# More visual detail than physics detail.
SAMPLES_PER_SEGMENT = 18
RADIAL_SEGMENTS = 16

# Keep radius constant in this test.
TUBE_RADIUS = base.TUBE_RADIUS

# Blend around each physical bone origin, measured on smooth arc length.
BLEND_HALF_WIDTH = base.LINK_HEIGHT * 0.18


def normalize(v):
    v = Gf.Vec3d(v)
    length = float(v.GetLength())
    if length < 1e-10:
        raise ValueError("zero-length vector")
    return v / length


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ---------------------------------------------------------------------------
# 1. SMOOTH CENTERLINE
# ---------------------------------------------------------------------------

# Use the exact polyline nodes from the validated 0F.
# In the real 0F implementation:
#   nodes = [Bone0 origin, Bone1 origin, Bone2 origin, final tip]
CONTROL_POINTS = [
    Gf.Vec3d(p)
    for p in base.CENTERLINE["nodes"]
]


def control_tangents(points):
    """
    Catmull-Rom/cardinal-style derivatives.

    The resulting Hermite curve passes exactly through every control point
    while sharing derivatives between adjacent pieces (C1 continuity).
    """
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


CONTROL_TANGENTS = control_tangents(CONTROL_POINTS)


def hermite_point(p0, p1, m0, m1, u):
    u2 = u * u
    u3 = u2 * u

    h00 = 2 * u3 - 3 * u2 + 1
    h10 = u3 - 2 * u2 + u
    h01 = -2 * u3 + 3 * u2
    h11 = u3 - u2

    return (
        p0 * h00
        + m0 * h10
        + p1 * h01
        + m1 * h11
    )


def hermite_derivative(p0, p1, m0, m1, u):
    u2 = u * u

    return (
        p0 * (6 * u2 - 6 * u)
        + m0 * (3 * u2 - 4 * u + 1)
        + p1 * (-6 * u2 + 6 * u)
        + m1 * (3 * u2 - 2 * u)
    )


def sample_centerline():
    positions = []
    tangents = []

    # Index in the sample array corresponding exactly to each control point.
    control_sample_indices = [None] * len(CONTROL_POINTS)

    for seg in range(len(CONTROL_POINTS) - 1):
        p0 = CONTROL_POINTS[seg]
        p1 = CONTROL_POINTS[seg + 1]
        m0 = CONTROL_TANGENTS[seg]
        m1 = CONTROL_TANGENTS[seg + 1]

        for j in range(SAMPLES_PER_SEGMENT):
            u = j / float(SAMPLES_PER_SEGMENT)

            if j == 0:
                control_sample_indices[seg] = len(positions)

            p = hermite_point(p0, p1, m0, m1, u)
            d = hermite_derivative(p0, p1, m0, m1, u)

            positions.append(Gf.Vec3d(p))
            tangents.append(normalize(d))

    # Exact final endpoint.
    positions.append(Gf.Vec3d(CONTROL_POINTS[-1]))
    tangents.append(normalize(CONTROL_TANGENTS[-1]))
    control_sample_indices[-1] = len(positions) - 1

    # Arc-length parameterization.
    arc = [0.0]

    for i in range(1, len(positions)):
        arc.append(
            arc[-1]
            + float((positions[i] - positions[i - 1]).GetLength())
        )

    control_arc = [
        arc[index]
        for index in control_sample_indices
    ]

    return {
        "positions": positions,
        "tangents": tangents,
        "arc": arc,
        "control_arc": control_arc,
    }


SMOOTH = sample_centerline()


# ---------------------------------------------------------------------------
# 2. PARALLEL-TRANSPORT FRAME
# ---------------------------------------------------------------------------

def initial_normal(tangent):
    preferred = Gf.Vec3d(0.0, 0.0, 1.0)

    # Fallback if tangent is almost vertical.
    if abs(float(Gf.Dot(preferred, tangent))) > 0.95:
        preferred = Gf.Vec3d(1.0, 0.0, 0.0)

    n = preferred - tangent * Gf.Dot(preferred, tangent)
    return normalize(n)


def rodrigues(v, axis, angle):
    axis = normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)

    return (
        v * c
        + Gf.Cross(axis, v) * s
        + axis * (Gf.Dot(axis, v) * (1.0 - c))
    )


def transport_normal(prev_t, current_t, prev_n):
    """
    Apply the minimum rotation that takes prev_t onto current_t.

    Unlike repeatedly building a frame from a global 'up' vector, parallel
    transport avoids arbitrary axial twist along a 3D centerline.
    """
    prev_t = normalize(prev_t)
    current_t = normalize(current_t)

    axis = Gf.Cross(prev_t, current_t)
    sin_angle = float(axis.GetLength())
    cos_angle = clamp(float(Gf.Dot(prev_t, current_t)), -1.0, 1.0)

    if sin_angle < 1e-9:
        n = Gf.Vec3d(prev_n)
    else:
        angle = math.atan2(sin_angle, cos_angle)
        n = rodrigues(
            prev_n,
            axis / sin_angle,
            angle,
        )

    # Numerical cleanup.
    n = n - current_t * Gf.Dot(n, current_t)

    if n.GetLength() < 1e-9:
        return initial_normal(current_t)

    return normalize(n)


def make_frames():
    tangents = SMOOTH["tangents"]

    normals = [initial_normal(tangents[0])]

    for i in range(1, len(tangents)):
        normals.append(
            transport_normal(
                tangents[i - 1],
                tangents[i],
                normals[-1],
            )
        )

    binormals = [
        normalize(Gf.Cross(t, n))
        for t, n in zip(tangents, normals)
    ]

    return normals, binormals


FRAME_NORMALS, FRAME_BINORMALS = make_frames()


# ---------------------------------------------------------------------------
# 3. SKIN WEIGHTS ALONG SMOOTH ARC LENGTH
# ---------------------------------------------------------------------------

def weights_for_arc(s):
    # P0..P(NUM_BONES-1) are physical bone origins.
    bone_arc = SMOOTH["control_arc"][:NUM_BONES]

    for child in range(1, NUM_BONES):
        joint_s = bone_arc[child]
        lo = joint_s - BLEND_HALF_WIDTH
        hi = joint_s + BLEND_HALF_WIDTH

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


# ---------------------------------------------------------------------------
# 4. THE ONLY REPLACEMENT: SMOOTH TUBE
# ---------------------------------------------------------------------------

def build_smooth_tube_mesh_data():
    points = []
    joint_indices = []
    joint_weights = []

    for ring, center in enumerate(SMOOTH["positions"]):
        n = FRAME_NORMALS[ring]
        b = FRAME_BINORMALS[ring]
        s = SMOOTH["arc"][ring]

        bone0, bone1, w0, w1 = weights_for_arc(s)

        for k in range(RADIAL_SEGMENTS):
            theta = 2.0 * math.pi * k / RADIAL_SEGMENTS

            radial = (
                n * (math.cos(theta) * TUBE_RADIUS)
                + b * (math.sin(theta) * TUBE_RADIUS)
            )

            p = center + radial

            points.append(
                Gf.Vec3f(float(p[0]), float(p[1]), float(p[2]))
            )
            joint_indices.extend([bone0, bone1])
            joint_weights.extend([w0, w1])

    face_counts = []
    face_indices = []

    for ring in range(len(SMOOTH["positions"]) - 1):
        row0 = ring * RADIAL_SEGMENTS
        row1 = (ring + 1) * RADIAL_SEGMENTS

        for k in range(RADIAL_SEGMENTS):
            kn = (k + 1) % RADIAL_SEGMENTS

            v00 = row0 + k
            v01 = row0 + kn
            v10 = row1 + k
            v11 = row1 + kn

            face_counts.extend([3, 3])
            face_indices.extend([
                v00, v10, v11,
                v00, v11, v01,
            ])

    return (
        points,
        face_counts,
        face_indices,
        joint_indices,
        joint_weights,
    )


# Test 1A's key isolation trick:
# base.build_visual() is already validated.
# It performs a module-global lookup of build_tube_data(), so replace
# ONLY that function inside the validated 0F generator.
base.build_tube_data = build_smooth_tube_mesh_data


# Re-export the validated skeleton-rest function for runtime checks.
rest_local_transforms = base.rest_local_transforms


def build_stage(output_path=OUTPUT_USD):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    stage = Usd.Stage.CreateNew(output_path)

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # ALL of these functions come unchanged from validated Test 0F.
    base.apply_physx_scene(stage)
    base.build_physics(stage)
    base.build_visual(stage)
    base.build_ground(stage)

    stage.Save()

    print("=" * 72)
    print("TEST 1A — Smooth Centerline Sweep")
    print("=" * 72)
    print(f"[OK] {output_path}")
    print()
    print(f"Physics links / bones : {NUM_LINKS}")
    print(f"Visual centerline rings: {len(SMOOTH['positions'])}")
    print(f"Vertices              : {len(SMOOTH['positions']) * RADIAL_SEGMENTS}")
    print(f"Smooth arc length     : {SMOOTH['arc'][-1]:.4f} m")
    print()
    print("Unchanged from 0F:")
    print("  PhysX articulation")
    print("  D6 configuration")
    print("  rigid-link rest poses")
    print("  Skeleton bind/rest poses")
    print()
    print("New in 1A:")
    print("  cubic Hermite centerline")
    print("  many visual rings")
    print("  parallel-transport frames")
    print("  weights measured on smooth arc length")
    print("=" * 72)

    return output_path


if __name__ == "__main__":
    build_stage()
