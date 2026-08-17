"""
generate_taper_centerline.py — Test 1B

Variable-radius / taper test built on top of the validated Test 1A.

Only ONE conceptual component changes:
    radius(s)

Everything else is reused from Test 1A:
    - smooth Hermite centerline
    - parallel-transport frames
    - skin weights
    - Skeleton
    - PhysX articulation
    - D6 rest frames
    - PhysX -> Skeleton bridge convention

Expected visual result:
    thicker base -> progressively thinner tip

This test intentionally leaves the physics collider radius unchanged.
That lets us isolate visual tapering from physics changes.
"""

import importlib.util
import os
from pathlib import Path

from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


# ===========================================================================
# LOAD VALIDATED TEST 1A ROBUSTLY
# ===========================================================================

SCRIPT_DIR = Path(__file__).resolve().parent
EXPERIMENTS_DIR = SCRIPT_DIR.parent

candidates = list(
    EXPERIMENTS_DIR.glob(
        "test_1a*/generate_smooth_centerline.py"
    )
)

# Also accept the filename I previously gave you before renaming.
if not candidates:
    candidates = list(
        EXPERIMENTS_DIR.glob(
            "test_1a*/generate_smooth_centerline_fixed.py"
        )
    )

if not candidates:
    raise ImportError(
        "Cannot find Test 1A generator under "
        f"{EXPERIMENTS_DIR}. Expected something like "
        "test_1a_smooth_centerline_sweep/generate_smooth_centerline.py"
    )

TEST_1A_PATH = candidates[0]

spec = importlib.util.spec_from_file_location(
    "test_1a_generate_smooth_centerline",
    TEST_1A_PATH,
)

if spec is None or spec.loader is None:
    raise ImportError(
        f"Could not load Test 1A module from {TEST_1A_PATH}"
    )

smooth = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smooth)

print(f"[DEBUG] Test 1A imported from: {TEST_1A_PATH}")


# ===========================================================================
# OUTPUT / RE-EXPORTS
# ===========================================================================

OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_USD = str(
    OUTPUT_DIR / "test_1b_taper_centerline.usda"
)

NUM_LINKS = smooth.NUM_LINKS
NUM_BONES = smooth.NUM_BONES

RADIAL_SEGMENTS = smooth.RADIAL_SEGMENTS

# Reuse the already validated smooth centerline and transport frames.
SMOOTH = smooth.SMOOTH
FRAME_NORMALS = smooth.FRAME_NORMALS
FRAME_BINORMALS = smooth.FRAME_BINORMALS


# ===========================================================================
# TEST 1B: VARIABLE RADIUS
# ===========================================================================

# Existing Test 1A tube radius is 12 mm.
REFERENCE_RADIUS = float(smooth.TUBE_RADIUS)

# Make the taper visible enough to evaluate, without making it extreme.
BASE_RADIUS = REFERENCE_RADIUS * 1.20
TIP_RADIUS = REFERENCE_RADIUS * 0.70

# Keep the extreme ends slightly calmer.
TAPER_START = 0.05
TAPER_END = 0.95


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def smoothstep01(u):
    """
    Cubic smoothstep:
        f(0) = 0
        f(1) = 1
        f'(0) = f'(1) = 0

    This avoids an abrupt radius derivative at the beginning/end
    of the taper region.
    """
    u = clamp(float(u), 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def radius_for_arc(s):
    """
    Radius as a function of distance along the smooth centerline.

    s = 0       -> BASE_RADIUS
    s = total   -> TIP_RADIUS
    """
    total = float(SMOOTH["arc"][-1])

    if total <= 1e-12:
        return BASE_RADIUS

    u_global = clamp(float(s) / total, 0.0, 1.0)

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
# TAPERED TUBE
# ===========================================================================

def build_tapered_tube_mesh_data():
    points = []
    joint_indices = []
    joint_weights = []

    for ring, center in enumerate(SMOOTH["positions"]):
        n = FRAME_NORMALS[ring]
        b = FRAME_BINORMALS[ring]
        s = float(SMOOTH["arc"][ring])

        radius = radius_for_arc(s)

        bone0, bone1, w0, w1 = smooth.weights_for_arc(s)

        for k in range(RADIAL_SEGMENTS):
            theta = (
                2.0
                * 3.141592653589793
                * k
                / RADIAL_SEGMENTS
            )

            import math

            radial = (
                n * (math.cos(theta) * radius)
                + b * (math.sin(theta) * radius)
            )

            p = center + radial

            points.append(
                Gf.Vec3f(
                    float(p[0]),
                    float(p[1]),
                    float(p[2]),
                )
            )

            joint_indices.extend([bone0, bone1])
            joint_weights.extend([w0, w1])

    face_counts = []
    face_indices = []

    ring_count = len(SMOOTH["positions"])

    for ring in range(ring_count - 1):
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


# ===========================================================================
# PATCH ONLY THE VISUAL GEOMETRY GENERATOR
# ===========================================================================

# Test 1A itself already patches the validated Test 0F module.
# Here we replace ONLY that final tube-data function once more.
smooth.base.build_tube_data = build_tapered_tube_mesh_data

# Runtime test needs the same validated skeleton rest transforms.
rest_local_transforms = smooth.rest_local_transforms


# ===========================================================================
# STAGE
# ===========================================================================

def build_stage(output_path=OUTPUT_USD):
    os.makedirs(
        os.path.dirname(output_path),
        exist_ok=True,
    )

    stage = Usd.Stage.CreateNew(output_path)

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

    # Same validated 0F/1A infrastructure.
    smooth.base.apply_physx_scene(stage)
    smooth.base.build_physics(stage)
    smooth.base.build_visual(stage)
    smooth.base.build_ground(stage)

    stage.Save()

    total = float(SMOOTH["arc"][-1])

    print("=" * 74)
    print("TEST 1B — Variable Radius / Taper")
    print("=" * 74)
    print(f"[OK] {output_path}")
    print()
    print(f"Physics links / bones : {NUM_LINKS}")
    print(f"Visual rings          : {len(SMOOTH['positions'])}")
    print(f"Reference radius      : {REFERENCE_RADIUS * 1000.0:.2f} mm")
    print(f"Base radius           : {BASE_RADIUS * 1000.0:.2f} mm")
    print(f"Tip radius            : {TIP_RADIUS * 1000.0:.2f} mm")
    print()
    print("Radius samples:")
    for frac in (0.0, 0.25, 0.50, 0.75, 1.0):
        r = radius_for_arc(total * frac)
        print(
            f"  {frac:>4.0%} arc -> {r * 1000.0:.2f} mm"
        )
    print()
    print("UNCHANGED:")
    print("  smooth centerline")
    print("  parallel-transport frames")
    print("  skin weights")
    print("  Skeleton")
    print("  PhysX")
    print("  D6 joints")
    print()
    print("CHANGED:")
    print("  ring radius = radius_for_arc(s)")
    print()
    print("NOTE:")
    print("  physics collider radius is intentionally still constant")
    print("  in this isolated visual test.")
    print("=" * 74)

    return output_path


if __name__ == "__main__":
    build_stage()
