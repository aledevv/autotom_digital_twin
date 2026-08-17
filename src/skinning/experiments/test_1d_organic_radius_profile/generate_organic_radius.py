"""
generate_organic_radius.py — Test 1D

Organic radius profile on top of the validated Test 1C.

Validated and unchanged:
    - true 3D rest centerline
    - cubic Hermite smoothing
    - parallel-transport frames
    - skin weights
    - PhysX articulation
    - D6 frames
    - Skeleton / bind / rest transforms

New:
    radius(s) =
        global taper
        + local node swellings
        + very small deterministic longitudinal variation

The resulting mesh is still circular at each ring.
Non-circular cross sections are intentionally deferred to a later test.
"""

import importlib.util
import math
import os
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics


# ===========================================================================
# LOAD VALIDATED TEST 1C
# ===========================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent

candidates = list(
    EXPERIMENTS_DIR.glob(
        "test_1c*/generate_3d_centerline.py"
    )
)

if not candidates:
    raise ImportError(
        "Cannot find Test 1C generator under "
        f"{EXPERIMENTS_DIR}"
    )

TEST_1C_PATH = candidates[0]

spec = importlib.util.spec_from_file_location(
    "test_1c_generate_3d_centerline",
    TEST_1C_PATH,
)

if spec is None or spec.loader is None:
    raise ImportError(
        f"Could not load Test 1C from {TEST_1C_PATH}"
    )

spatial = importlib.util.module_from_spec(spec)
spec.loader.exec_module(spatial)

print(f"[DEBUG] Test 1C imported from: {TEST_1C_PATH}")


# ===========================================================================
# OUTPUT / RE-EXPORTS
# ===========================================================================

OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_USD = str(
    OUTPUT_DIR / "test_1d_organic_radius.usda"
)

NUM_LINKS = spatial.NUM_LINKS
NUM_BONES = spatial.NUM_BONES

SMOOTH = spatial.SMOOTH
FRAME_NORMALS = spatial.FRAME_NORMALS
FRAME_BINORMALS = spatial.FRAME_BINORMALS
RADIAL_SEGMENTS = spatial.RADIAL_SEGMENTS

REFERENCE_RADIUS = float(spatial.REFERENCE_RADIUS)

# Keep the same global taper range already validated in 1C.
BASE_RADIUS = float(spatial.BASE_RADIUS)
TIP_RADIUS = float(spatial.TIP_RADIUS)
TAPER_START = float(spatial.TAPER_START)
TAPER_END = float(spatial.TAPER_END)


# ===========================================================================
# ORGANIC PROFILE PARAMETERS
# ===========================================================================

# Swelling centered around internal physical nodes.
# 0.12 = +12% local radius at the center of each node.
NODE_SWELL_AMPLITUDE = 0.25

# Width of each swelling along centerline arc length.
NODE_SWELL_SIGMA = spatial.LINK_HEIGHT * 0.20

# Very subtle deterministic longitudinal radius modulation.
# Kept deliberately small so it removes the "perfect CAD tube" feeling
# without producing an obviously wavy stem.
MICRO_VARIATION_AMPLITUDE = 0.05

# Number of low-frequency oscillations across the complete branch.
MICRO_VARIATION_CYCLES = 2.25


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smoothstep01(u):
    u = clamp(float(u), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


# ===========================================================================
# 1. BASE TAPER — same concept as Test 1B/1C
# ===========================================================================

def taper_radius(s):
    total = float(SMOOTH["arc"][-1])

    if total <= 1e-12:
        return BASE_RADIUS

    u_global = clamp(
        float(s) / total,
        0.0,
        1.0,
    )

    if u_global <= TAPER_START:
        return BASE_RADIUS

    if u_global >= TAPER_END:
        return TIP_RADIUS

    u = (
        (u_global - TAPER_START)
        / (TAPER_END - TAPER_START)
    )

    blend = smoothstep01(u)

    return (
        BASE_RADIUS
        + (TIP_RADIUS - BASE_RADIUS) * blend
    )


# ===========================================================================
# 2. LOCAL NODE SWELLINGS
# ===========================================================================

# P0..P(NUM_BONES-1) correspond to physical bone origins.
# We intentionally skip P0 (root/base) and swell only internal nodes.
NODE_ARC_POSITIONS = (
    SMOOTH["control_arc"][1:NUM_BONES]
)


def gaussian_bump(s, center, sigma):
    x = (float(s) - float(center)) / float(sigma)
    return math.exp(-0.5 * x * x)


def node_swell_factor(s):
    """
    Returns a multiplicative radius factor.

    Example:
        1.00 away from nodes
        ~1.12 at a node center
    """
    bump_sum = 0.0

    for node_s in NODE_ARC_POSITIONS:
        bump_sum += gaussian_bump(
            s,
            node_s,
            NODE_SWELL_SIGMA,
        )

    return (
        1.0
        + NODE_SWELL_AMPLITUDE * bump_sum
    )


# ===========================================================================
# 3. VERY LIGHT DETERMINISTIC LONGITUDINAL VARIATION
# ===========================================================================

def micro_variation_factor(s):
    """
    Deterministic, smooth and fixed.

    An envelope makes the variation go to zero at both ends so the
    base/tip radii remain controlled by the global taper.
    """
    total = float(SMOOTH["arc"][-1])

    if total <= 1e-12:
        return 1.0

    u = clamp(
        float(s) / total,
        0.0,
        1.0,
    )

    # Zero at base and tip, strongest around middle.
    envelope = math.sin(math.pi * u) ** 2

    phase1 = (
        2.0
        * math.pi
        * MICRO_VARIATION_CYCLES
        * u
    )

    phase2 = (
        2.0
        * math.pi
        * (MICRO_VARIATION_CYCLES * 0.47)
        * u
        + 0.8
    )

    signal = (
        0.70 * math.sin(phase1)
        + 0.30 * math.sin(phase2)
    )

    return (
        1.0
        + MICRO_VARIATION_AMPLITUDE
        * envelope
        * signal
    )


# ===========================================================================
# 4. FINAL ORGANIC RADIUS FUNCTION
# ===========================================================================

def radius_for_arc(s):
    return (
        taper_radius(s)
        * node_swell_factor(s)
        * micro_variation_factor(s)
    )


# ===========================================================================
# 5. BUILD MESH — same 3D centerline and transport frames as Test 1C
# ===========================================================================

def build_organic_tube_data():
    points = []
    joint_indices = []
    joint_weights = []

    for ring, center in enumerate(
        SMOOTH["positions"]
    ):
        normal = FRAME_NORMALS[ring]
        binormal = FRAME_BINORMALS[ring]
        s = float(SMOOTH["arc"][ring])

        radius = radius_for_arc(s)

        bone0, bone1, w0, w1 = (
            spatial.weights_for_arc(s)
        )

        for k in range(
            RADIAL_SEGMENTS
        ):
            theta = (
                2.0
                * math.pi
                * k
                / RADIAL_SEGMENTS
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

            p = center + radial

            points.append(
                Gf.Vec3f(
                    float(p[0]),
                    float(p[1]),
                    float(p[2]),
                )
            )

            joint_indices.extend(
                [bone0, bone1]
            )
            joint_weights.extend(
                [w0, w1]
            )

    face_counts = []
    face_indices = []

    for ring in range(
        len(SMOOTH["positions"]) - 1
    ):
        row0 = (
            ring
            * RADIAL_SEGMENTS
        )
        row1 = (
            (ring + 1)
            * RADIAL_SEGMENTS
        )

        for k in range(
            RADIAL_SEGMENTS
        ):
            kn = (
                (k + 1)
                % RADIAL_SEGMENTS
            )

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


# ===========================================================================
# PATCH ONLY VISUAL RADIUS / TUBE DATA
# ===========================================================================

# Test 1C already drove the validated Test 0F physics/skeleton with the
# correct 3D CENTERLINE. We leave that intact and replace only its mesh data.
spatial.base.build_tube_data = (
    build_organic_tube_data
)

# Runtime check uses the same validated rest transforms.
rest_local_transforms = (
    spatial.rest_local_transforms
)


# ===========================================================================
# STAGE
# ===========================================================================

def build_stage(output_path=OUTPUT_USD):
    os.makedirs(
        os.path.dirname(output_path),
        exist_ok=True,
    )

    stage = Usd.Stage.CreateNew(
        output_path
    )

    UsdGeom.SetStageUpAxis(
        stage,
        UsdGeom.Tokens.z,
    )
    UsdGeom.SetStageMetersPerUnit(
        stage,
        1.0,
    )
    UsdPhysics.SetStageKilogramsPerUnit(
        stage,
        1.0,
    )

    world = UsdGeom.Xform.Define(
        stage,
        "/World",
    )
    stage.SetDefaultPrim(
        world.GetPrim()
    )

    # Exactly the validated Test 1C physical/skeletal setup.
    spatial.base.apply_physx_scene(stage)
    spatial.base.build_physics(stage)
    spatial.base.build_visual(stage)
    spatial.base.build_ground(stage)

    stage.Save()

    total = float(
        SMOOTH["arc"][-1]
    )

    print("=" * 78)
    print("TEST 1D — Organic Radius Profile")
    print("=" * 78)
    print(f"[OK] {output_path}")
    print()
    print("UNCHANGED from 1C:")
    print("  true 3D centerline")
    print("  smooth Hermite sweep")
    print("  parallel transport")
    print("  PhysX / D6")
    print("  Skeleton / skinning")
    print()
    print("NEW radius model:")
    print("  global taper")
    print(
        f"  node swelling: +"
        f"{NODE_SWELL_AMPLITUDE * 100:.1f}%"
    )
    print(
        f"  micro variation: ±~"
        f"{MICRO_VARIATION_AMPLITUDE * 100:.1f}%"
    )
    print()
    print("Radius samples:")
    for frac in (
        0.0,
        0.20,
        0.40,
        0.60,
        0.80,
        1.0,
    ):
        s = total * frac
        r = radius_for_arc(s)

        print(
            f"  {frac:>4.0%}: "
            f"{r * 1000.0:.2f} mm"
        )

    print()
    print("Internal node positions:")
    for i, node_s in enumerate(
        NODE_ARC_POSITIONS,
        start=1,
    ):
        r = radius_for_arc(node_s)

        print(
            f"  node {i}: "
            f"s={node_s:.4f} m, "
            f"r={r * 1000.0:.2f} mm"
        )

    print("=" * 78)

    return output_path


if __name__ == "__main__":
    build_stage()
