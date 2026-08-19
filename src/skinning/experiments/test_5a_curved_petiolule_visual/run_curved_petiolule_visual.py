"""Visual-only test for plausible randomized tomato petiolules and leaf blades.

The test stays isolated from exporterV2.  It keeps the validated lightweight
representation (short rigid petiolule + 2D blade) but adds controlled biological
variation:

- petiolule length and thickness can vary both above and below the nominal size;
- petiolule tilt is randomized per leaf, including slightly downward cases;
- blade length/width/fold vary asymmetrically so some leaves can be noticeably
  finer without creating equally extreme oversized leaves;
- the blade starts almost tangent-continuous with its petiolule;
- gravity shaping is coupled to petiolule tilt: an upward petiolule needs a
  stronger longitudinal bend to bring the distal blade back down, while a
  downward petiolule receives a milder extra bend;
- every variation is deterministic for a given organ path;
- no physics, joints, UsdSkel, or runtime deformation are used.
"""

import hashlib
import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1500,
    "height": 900,
})

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, Vt


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_USD = os.path.join(OUTPUT_DIR, "curved_petiolule_visual.usda")


# =============================================================================
# VISUAL PARAMETERS
# =============================================================================

RADIAL_SEGMENTS = 14
CURVE_SAMPLES = 14
LEAF_STATIONS = 18

RACHIS_LENGTH_M = 0.36
RACHIS_RADIUS_M = 0.0060

LEAF_LENGTH_M = 0.075
LEAF_HALF_WIDTH_M = 0.025
PETIOLULE_LENGTH_M = 0.014
PETIOLULE_ROOT_RADIUS_M = 0.0024
PETIOLULE_TIP_RADIUS_M = 0.00135

LEAF_LONGITUDINAL_FOLD_M = 0.0048
LEAF_FOLD_EXPONENT = 0.78
LEAF_ARCH_LIFT_M = 0.0060
LEAF_TIP_SAG_M = 0.0100
LEAF_TIP_SAG_EXPONENT = 1.85

# Asymmetric ranges: allow clearly smaller/finer organs, while positive growth
# stays more moderate.  This matches the request to avoid all leaves being the
# same size without making the largest samples implausibly dominant.
PETIOLULE_LENGTH_SCALE_RANGE = (0.84, 1.20)
PETIOLULE_RADIUS_SCALE_RANGE = (0.80, 1.12)
LEAF_LENGTH_SCALE_RANGE = (0.86, 1.26)
LEAF_WIDTH_SCALE_RANGE = (0.72, 1.20)
LEAF_FOLD_SCALE_RANGE = (0.78, 1.28)
LEAF_ARCH_RANDOM_RANGE = (0.86, 1.16)
LEAF_SAG_RANDOM_RANGE = (0.84, 1.18)

# Petiolules are mostly upward-oriented in the reference, but a few can be close
# to horizontal or slightly downward.  The blade pose is adjusted from this same
# tilt rather than independently randomized into an incompatible configuration.
PETIOLULE_TILT_RANGE_DEG = (-5.0, 25.0)
LEAF_AZIMUTH_VARIATION_DEG = 9.0

PAIR_Y = (-0.105, 0.0, 0.105)

STEM_COLOR = Gf.Vec3f(0.58, 0.78, 0.38)
PETIOLULE_COLOR = Gf.Vec3f(0.55, 0.76, 0.34)
LEAF_COLOR = Gf.Vec3f(0.31, 0.67, 0.20)
CURRENT_COLOR = Gf.Vec3f(0.68, 0.83, 0.47)


# =============================================================================
# GEOMETRY HELPERS
# =============================================================================


def _length(vector):
    return math.sqrt(float(Gf.Dot(vector, vector)))


def _normalized(vector):
    vector = Gf.Vec3d(vector)
    length = _length(vector)
    if length <= 1e-12:
        raise ValueError("Cannot normalize a zero-length vector")
    return vector / length


def _smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def _stable_unit(key, salt):
    """Stable pseudo-random scalar in [0, 1] for one organ/property."""
    payload = f"{key}|{salt}|test-5a-v6".encode("utf-8")
    digest = hashlib.blake2b(payload, digest_size=8).digest()
    integer = int.from_bytes(digest, byteorder="big", signed=False)
    return integer / float((1 << 64) - 1)


def _stable_range(key, salt, low, high):
    return low + (high - low) * _stable_unit(key, salt)


def _stable_signed(key, salt):
    return _stable_unit(key, salt) * 2.0 - 1.0


def _lerp(a, b, t):
    return a + (b - a) * max(0.0, min(1.0, t))


def _rotate_about_up(direction, angle_deg):
    rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), angle_deg)
    return _normalized(rotation.TransformDir(direction))


def _tilted_direction(outward, tilt_deg):
    radians = math.radians(tilt_deg)
    return _normalized(
        outward * math.cos(radians)
        + Gf.Vec3d(0.0, 0.0, 1.0) * math.sin(radians)
    )


def _cubic_point(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (
        p0 * (u ** 3)
        + p1 * (3.0 * u * u * t)
        + p2 * (3.0 * u * t * t)
        + p3 * (t ** 3)
    )


def _cubic_tangent(p0, p1, p2, p3, t):
    u = 1.0 - t
    tangent = (
        (p1 - p0) * (3.0 * u * u)
        + (p2 - p1) * (6.0 * u * t)
        + (p3 - p2) * (3.0 * t * t)
    )
    return _normalized(tangent)


def _sample_cubic(p0, p1, p2, p3, count=CURVE_SAMPLES):
    centers = []
    tangents = []
    for index in range(count):
        t = index / float(count - 1)
        centers.append(_cubic_point(p0, p1, p2, p3, t))
        tangents.append(_cubic_tangent(p0, p1, p2, p3, t))
    return centers, tangents


def _sample_line(p0, p1, count=CURVE_SAMPLES):
    tangent = _normalized(p1 - p0)
    centers = []
    tangents = []
    for index in range(count):
        t = index / float(count - 1)
        centers.append(p0 * (1.0 - t) + p1 * t)
        tangents.append(tangent)
    return centers, tangents


def _transport_frames(tangents):
    first = tangents[0]
    reference = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(float(Gf.Dot(first, reference))) > 0.92:
        reference = Gf.Vec3d(0.0, 1.0, 0.0)

    normal = _normalized(Gf.Cross(reference, first))
    binormal = _normalized(Gf.Cross(first, normal))
    normals = [normal]
    binormals = [binormal]
    previous_tangent = first
    previous_normal = normal

    for tangent in tangents[1:]:
        axis = Gf.Cross(previous_tangent, tangent)
        if _length(axis) > 1e-10:
            axis = _normalized(axis)
            cosine = max(-1.0, min(1.0, float(Gf.Dot(previous_tangent, tangent))))
            rotation = Gf.Rotation(axis, math.degrees(math.acos(cosine)))
            normal = _normalized(rotation.TransformDir(previous_normal))
        else:
            normal = previous_normal

        binormal = _normalized(Gf.Cross(tangent, normal))
        normal = _normalized(Gf.Cross(binormal, tangent))
        normals.append(normal)
        binormals.append(binormal)
        previous_tangent = tangent
        previous_normal = normal

    return normals, binormals


def _taper_radii(start_radius, end_radius, count):
    result = []
    for index in range(count):
        t = index / float(max(count - 1, 1))
        q = _smoothstep(t)
        result.append(start_radius + (end_radius - start_radius) * q)
    return result


def _tube_data(centers, tangents, radii, *, cap_start=True, cap_end=True):
    normals, binormals = _transport_frames(tangents)
    points = []

    for center, normal, binormal, radius in zip(centers, normals, binormals, radii):
        for radial in range(RADIAL_SEGMENTS):
            theta = 2.0 * math.pi * radial / RADIAL_SEGMENTS
            point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*point))

    face_counts = []
    face_indices = []
    for ring in range(len(centers) - 1):
        row0 = ring * RADIAL_SEGMENTS
        row1 = (ring + 1) * RADIAL_SEGMENTS
        for radial in range(RADIAL_SEGMENTS):
            next_radial = (radial + 1) % RADIAL_SEGMENTS
            face_counts.extend((3, 3))
            face_indices.extend((
                row0 + radial,
                row1 + radial,
                row1 + next_radial,
                row0 + radial,
                row1 + next_radial,
                row0 + next_radial,
            ))

    if cap_start:
        center_index = len(points)
        points.append(Gf.Vec3f(*centers[0]))
        for radial in range(RADIAL_SEGMENTS):
            next_radial = (radial + 1) % RADIAL_SEGMENTS
            face_counts.append(3)
            face_indices.extend((center_index, next_radial, radial))

    if cap_end:
        center_index = len(points)
        points.append(Gf.Vec3f(*centers[-1]))
        row = (len(centers) - 1) * RADIAL_SEGMENTS
        for radial in range(RADIAL_SEGMENTS):
            next_radial = (radial + 1) % RADIAL_SEGMENTS
            face_counts.append(3)
            face_indices.extend((center_index, row + radial, row + next_radial))

    return points, face_counts, face_indices


def _author_mesh(stage, path, data, color):
    points, face_counts, face_indices = data
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([color]))
    return mesh


def _author_leaf_blade(
    stage,
    path,
    root,
    forward,
    *,
    length,
    half_width,
    fold_depth,
    arch_lift,
    tip_sag,
    color,
):
    """Create a 2D blade with longitudinal fold plus one smooth gravity arch."""
    forward = _normalized(forward)
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    side = Gf.Cross(world_up, forward)
    if _length(side) <= 1e-8:
        side = Gf.Cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)
    sheet_normal = _normalized(Gf.Cross(forward, side))

    points = []
    for index in range(LEAF_STATIONS):
        t = index / float(LEAF_STATIONS - 1)

        width_profile = math.sin(math.pi * t) ** 0.78
        width_profile *= 1.13 - 0.27 * t
        width = half_width * width_profile

        arch_offset = arch_lift * (4.0 * t * (1.0 - t))
        gravity_offset = tip_sag * (t ** LEAF_TIP_SAG_EXPONENT)
        center = (
            root
            + forward * (length * t)
            + world_up * (arch_offset - gravity_offset)
        )

        fold_profile = math.sin(math.pi * t) ** LEAF_FOLD_EXPONENT
        edge_drop = fold_depth * fold_profile

        left = center + side * width - sheet_normal * edge_drop
        midrib = center
        right = center - side * width - sheet_normal * edge_drop
        points.extend((Gf.Vec3f(*left), Gf.Vec3f(*midrib), Gf.Vec3f(*right)))

    face_counts = []
    face_indices = []
    for station in range(LEAF_STATIONS - 1):
        row0 = station * 3
        row1 = (station + 1) * 3

        face_counts.extend((3, 3))
        face_indices.extend((
            row0 + 0,
            row0 + 1,
            row1 + 1,
            row0 + 0,
            row1 + 1,
            row1 + 0,
        ))
        face_counts.extend((3, 3))
        face_indices.extend((
            row0 + 1,
            row0 + 2,
            row1 + 2,
            row0 + 1,
            row1 + 2,
            row1 + 1,
        ))

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([color]))
    return mesh


# =============================================================================
# SPECIMENS
# =============================================================================


def _author_rachis(stage, path, x, color):
    start = Gf.Vec3d(x, -RACHIS_LENGTH_M / 2.0, 0.055)
    end = Gf.Vec3d(x, RACHIS_LENGTH_M / 2.0, 0.055)
    centers, tangents = _sample_line(start, end)
    radii = [RACHIS_RADIUS_M] * len(centers)
    _author_mesh(stage, path, _tube_data(centers, tangents, radii), color)


def _author_straight_pair(stage, root_path, x, y):
    """Baseline: short straight petiolule and completely flat 2D blade."""
    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        root = Gf.Vec3d(x, y, 0.055)
        outward = Gf.Vec3d(sign, 0.0, 0.0)
        tip = root + outward * PETIOLULE_LENGTH_M
        centers, tangents = _sample_line(root, tip)
        radii = _taper_radii(
            PETIOLULE_ROOT_RADIUS_M,
            PETIOLULE_TIP_RADIUS_M,
            len(centers),
        )
        _author_mesh(
            stage,
            f"{root_path}/{side_name}Petiolule",
            _tube_data(centers, tangents, radii),
            CURRENT_COLOR,
        )
        _author_leaf_blade(
            stage,
            f"{root_path}/{side_name}Leaf",
            tip,
            outward,
            length=LEAF_LENGTH_M,
            half_width=LEAF_HALF_WIDTH_M,
            fold_depth=0.0,
            arch_lift=0.0,
            tip_sag=0.0,
            color=LEAF_COLOR,
        )


def _author_proposed_pair(stage, root_path, x, y):
    """Per-leaf variation with petiolule tilt and gravity-aware blade pose."""
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)

    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        key = f"{root_path}/{side_name}"
        root = Gf.Vec3d(x, y, 0.055)

        base_outward = Gf.Vec3d(sign, 0.0, 0.0)
        azimuth = LEAF_AZIMUTH_VARIATION_DEG * _stable_signed(key, "azimuth")
        outward = _rotate_about_up(base_outward, azimuth)

        petiolule_length = PETIOLULE_LENGTH_M * _stable_range(
            key, "petiolule_length", *PETIOLULE_LENGTH_SCALE_RANGE
        )
        radius_scale = _stable_range(
            key, "petiolule_radius", *PETIOLULE_RADIUS_SCALE_RANGE
        )
        tilt_deg = _stable_range(key, "petiolule_tilt", *PETIOLULE_TILT_RANGE_DEG)
        petiolule_direction = _tilted_direction(outward, tilt_deg)

        leaf_length = LEAF_LENGTH_M * _stable_range(
            key, "leaf_length", *LEAF_LENGTH_SCALE_RANGE
        )
        leaf_half_width = LEAF_HALF_WIDTH_M * _stable_range(
            key, "leaf_width", *LEAF_WIDTH_SCALE_RANGE
        )
        fold_depth = LEAF_LONGITUDINAL_FOLD_M * _stable_range(
            key, "fold", *LEAF_FOLD_SCALE_RANGE
        )

        # Couple gravity shape to the petiolule tilt.  A more upward support
        # means the blade must bend more around its short axis to settle under
        # gravity; a downward support already points toward gravity and therefore
        # receives a milder additional arch/sag.
        tilt01 = (
            (tilt_deg - PETIOLULE_TILT_RANGE_DEG[0])
            / (PETIOLULE_TILT_RANGE_DEG[1] - PETIOLULE_TILT_RANGE_DEG[0])
        )
        pose_arch_scale = _lerp(0.48, 1.22, tilt01)
        pose_sag_scale = _lerp(0.62, 1.38, tilt01)
        arch_lift = (
            LEAF_ARCH_LIFT_M
            * pose_arch_scale
            * _stable_range(key, "arch", *LEAF_ARCH_RANDOM_RANGE)
        )
        tip_sag = (
            LEAF_TIP_SAG_M
            * pose_sag_scale
            * _stable_range(key, "sag", *LEAF_SAG_RANDOM_RANGE)
        )

        # Short petiolule: almost straight in its own randomized tilt direction,
        # with only a tiny curvature perturbation for organic variation.
        curve_bias = 0.00055 * _stable_signed(key, "petiolule_curve")
        p0 = root
        p1 = (
            root
            + petiolule_direction * (petiolule_length * 0.33)
            + world_up * (curve_bias * 0.35)
        )
        p2 = (
            root
            + petiolule_direction * (petiolule_length * 0.68)
            + world_up * curve_bias
        )
        p3 = root + petiolule_direction * petiolule_length

        centers, tangents = _sample_cubic(p0, p1, p2, p3)
        radii = _taper_radii(
            PETIOLULE_ROOT_RADIUS_M * radius_scale,
            PETIOLULE_TIP_RADIUS_M * radius_scale,
            len(centers),
        )
        _author_mesh(
            stage,
            f"{root_path}/{side_name}Petiolule",
            _tube_data(centers, tangents, radii),
            PETIOLULE_COLOR,
        )

        # Begin essentially on the actual petiolule tangent.  The centerline
        # deformation above then performs the gravity-dependent longitudinal bend
        # rather than forcing every leaf into the same world-space pose.
        distal_forward = _normalized(tangents[-1] * 0.88 + outward * 0.12)
        _author_leaf_blade(
            stage,
            f"{root_path}/{side_name}Leaf",
            centers[-1],
            distal_forward,
            length=leaf_length,
            half_width=leaf_half_width,
            fold_depth=fold_depth,
            arch_lift=arch_lift,
            tip_sag=tip_sag,
            color=LEAF_COLOR,
        )

        print(
            f"[LEAF] {key} | tilt={tilt_deg:+.1f}deg | "
            f"petioleL={petiolule_length / PETIOLULE_LENGTH_M:.2f}x | "
            f"width={leaf_half_width / LEAF_HALF_WIDTH_M:.2f}x | "
            f"arch={arch_lift / LEAF_ARCH_LIFT_M:.2f}x | "
            f"sag={tip_sag / LEAF_TIP_SAG_M:.2f}x"
        )


def _author_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")

    current_x = -0.28
    UsdGeom.Xform.Define(stage, "/World/Current")
    _author_rachis(stage, "/World/Current/Rachis", current_x, CURRENT_COLOR)
    _author_straight_pair(stage, "/World/Current/Pair", current_x, 0.0)

    proposed_x = 0.24
    UsdGeom.Xform.Define(stage, "/World/Proposed")
    _author_rachis(stage, "/World/Proposed/Rachis", proposed_x, STEM_COLOR)
    for index, y in enumerate(PAIR_Y):
        _author_proposed_pair(
            stage,
            f"/World/Proposed/Pair_{index + 1:02d}",
            proposed_x,
            y,
        )

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr().Set(650.0)

    distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    distant.CreateIntensityAttr().Set(1800.0)
    distant.CreateAngleAttr().Set(0.8)
    xform = UsdGeom.Xformable(distant.GetPrim())
    xform.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 25.0))

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr().Set(52.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    eye = Gf.Vec3d(0.78, -1.15, 0.48)
    target = Gf.Vec3d(0.0, 0.0, 0.06)
    view = Gf.Matrix4d(1.0)
    view.SetLookAt(eye, target, Gf.Vec3d(0.0, 0.0, 1.0))
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(view.GetInverse())


def main():
    stage = Usd.Stage.CreateNew(OUTPUT_USD)
    _author_scene(stage)
    stage.GetRootLayer().Save()

    print("=" * 76)
    print("TEST 5A v6 - TILT-AWARE RANDOMIZED LEAF REST SHAPES")
    print("=" * 76)
    print(f"USD: {OUTPUT_USD}")
    print("LEFT  : short straight petiolule + flat 2D blade")
    print("RIGHT : asymmetric size variation + randomized petiolule tilt")
    print("Leaf gravity bend is coupled to petiolule tilt for plausible poses")
    print("Randomness is stable: identical organ path -> identical generated shape")
    print("No physics / no joints / no UsdSkel / no runtime deformation")
    print("=" * 76)

    omni.usd.get_context().open_stage(OUTPUT_USD)
    simulation_app.update()

    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            viewport.set_active_camera("/World/Camera")
    except Exception as exc:
        print(f"[INFO] Could not set test camera automatically: {exc}")

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
