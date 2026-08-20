"""Visual-only test for short tomato petiolules and organically varied leaf blades.

The proposed specimen keeps one important structural constraint while allowing
visible variation in the leaf organs:
- petiolules are short and remain almost horizontal, with only a very small tilt
  spread so leaves emerge at nearly the same height;
- petiolule thickness/length and blade size can still vary noticeably;
- leaf azimuth, width, longitudinal fold, arch and sag retain the broader organic
  variation used in the previous version;
- blades keep the longitudinal midrib fold;
- each blade follows one gentle rise and then a gravity-like distal sag;
- no physics, joints, UsdSkel or runtime deformation are used.
"""

import hashlib
import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1500, "height": 900})

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, Vt


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_USD = os.path.join(OUTPUT_DIR, "curved_petiolule_visual.usda")

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

# Restore the broader organic variation from the previous version. Smaller/finer
# organs are allowed somewhat more strongly than oversized ones where useful.
PETIOLULE_LENGTH_SCALE_RANGE = (0.84, 1.20)
PETIOLULE_RADIUS_SCALE_RANGE = (0.80, 1.12)
LEAF_LENGTH_SCALE_RANGE = (0.86, 1.26)
LEAF_WIDTH_SCALE_RANGE = (0.72, 1.20)
LEAF_FOLD_SCALE_RANGE = (0.78, 1.28)
LEAF_ARCH_SCALE_RANGE = (0.86, 1.16)
LEAF_SAG_SCALE_RANGE = (0.84, 1.18)

# The vertical inclination is the constrained variable: tomato leaflets usually
# emerge with very similar petiolule elevation, so only a few degrees of tilt are
# allowed. Azimuthal variation can remain broader because it does not make the
# leaflets originate at visibly different heights.
PETIOLULE_TILT_RANGE_DEG = (-2.0, 4.0)
LEAF_AZIMUTH_VARIATION_DEG = 9.0

PAIR_Y = (-0.105, 0.0, 0.105)

STEM_COLOR = Gf.Vec3f(0.58, 0.78, 0.38)
PETIOLULE_COLOR = Gf.Vec3f(0.55, 0.76, 0.34)
LEAF_COLOR = Gf.Vec3f(0.31, 0.67, 0.20)
CURRENT_COLOR = Gf.Vec3f(0.68, 0.83, 0.47)


def _length(v):
    return math.sqrt(float(Gf.Dot(v, v)))


def _normalized(v):
    v = Gf.Vec3d(v)
    length = _length(v)
    if length <= 1e-12:
        raise ValueError("Cannot normalize a zero-length vector")
    return v / length


def _stable_unit(key, salt):
    payload = f"{key}|{salt}|test-5a-v7".encode("utf-8")
    digest = hashlib.blake2b(payload, digest_size=8).digest()
    return int.from_bytes(digest, "big") / float((1 << 64) - 1)


def _stable_range(key, salt, low, high):
    return low + (high - low) * _stable_unit(key, salt)


def _stable_signed(key, salt):
    return 2.0 * _stable_unit(key, salt) - 1.0


def _rotate_about_up(direction, angle_deg):
    rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), angle_deg)
    return _normalized(rotation.TransformDir(direction))


def _tilted_direction(outward, tilt_deg):
    angle = math.radians(tilt_deg)
    return _normalized(
        outward * math.cos(angle)
        + Gf.Vec3d(0.0, 0.0, 1.0) * math.sin(angle)
    )


def _cubic_point(p0, p1, p2, p3, t):
    u = 1.0 - t
    return p0 * u**3 + p1 * (3.0 * u * u * t) + p2 * (3.0 * u * t * t) + p3 * t**3


def _cubic_tangent(p0, p1, p2, p3, t):
    u = 1.0 - t
    return _normalized(
        (p1 - p0) * (3.0 * u * u)
        + (p2 - p1) * (6.0 * u * t)
        + (p3 - p2) * (3.0 * t * t)
    )


def _sample_cubic(p0, p1, p2, p3, count=CURVE_SAMPLES):
    centers, tangents = [], []
    for index in range(count):
        t = index / float(count - 1)
        centers.append(_cubic_point(p0, p1, p2, p3, t))
        tangents.append(_cubic_tangent(p0, p1, p2, p3, t))
    return centers, tangents


def _sample_line(p0, p1, count=CURVE_SAMPLES):
    tangent = _normalized(p1 - p0)
    centers = []
    for index in range(count):
        t = index / float(count - 1)
        centers.append(p0 * (1.0 - t) + p1 * t)
    return centers, [tangent] * count


def _transport_frames(tangents):
    first = tangents[0]
    reference = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(float(Gf.Dot(first, reference))) > 0.92:
        reference = Gf.Vec3d(0.0, 1.0, 0.0)

    normal = _normalized(Gf.Cross(reference, first))
    binormal = _normalized(Gf.Cross(first, normal))
    normals, binormals = [normal], [binormal]
    previous_tangent, previous_normal = first, normal

    for tangent in tangents[1:]:
        axis = Gf.Cross(previous_tangent, tangent)
        if _length(axis) > 1e-10:
            axis = _normalized(axis)
            cosine = max(-1.0, min(1.0, float(Gf.Dot(previous_tangent, tangent))))
            normal = _normalized(
                Gf.Rotation(axis, math.degrees(math.acos(cosine))).TransformDir(previous_normal)
            )
        else:
            normal = previous_normal
        binormal = _normalized(Gf.Cross(tangent, normal))
        normal = _normalized(Gf.Cross(binormal, tangent))
        normals.append(normal)
        binormals.append(binormal)
        previous_tangent, previous_normal = tangent, normal

    return normals, binormals


def _taper_radii(start_radius, end_radius, count):
    result = []
    for index in range(count):
        t = index / float(max(count - 1, 1))
        q = t * t * (3.0 - 2.0 * t)
        result.append(start_radius + (end_radius - start_radius) * q)
    return result


def _tube_data(centers, tangents, radii):
    normals, binormals = _transport_frames(tangents)
    points = []
    for center, normal, binormal, radius in zip(centers, normals, binormals, radii):
        for radial in range(RADIAL_SEGMENTS):
            theta = 2.0 * math.pi * radial / RADIAL_SEGMENTS
            point = center + radius * (normal * math.cos(theta) + binormal * math.sin(theta))
            points.append(Gf.Vec3f(*point))

    counts, indices = [], []
    for ring in range(len(centers) - 1):
        row0, row1 = ring * RADIAL_SEGMENTS, (ring + 1) * RADIAL_SEGMENTS
        for radial in range(RADIAL_SEGMENTS):
            nxt = (radial + 1) % RADIAL_SEGMENTS
            counts.extend((3, 3))
            indices.extend((row0 + radial, row1 + radial, row1 + nxt,
                            row0 + radial, row1 + nxt, row0 + nxt))
    return points, counts, indices


def _author_mesh(stage, path, data, color):
    points, counts, indices = data
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([color]))


def _author_leaf_blade(stage, path, root, forward, *, length, half_width,
                       fold_depth, arch_lift, tip_sag, color):
    """2D blade with longitudinal midrib fold and one gentle gravity arch."""
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
        gravity_offset = tip_sag * t**LEAF_TIP_SAG_EXPONENT
        center = root + forward * (length * t) + world_up * (arch_offset - gravity_offset)

        edge_drop = fold_depth * math.sin(math.pi * t) ** LEAF_FOLD_EXPONENT
        points.extend((
            Gf.Vec3f(*(center + side * width - sheet_normal * edge_drop)),
            Gf.Vec3f(*center),
            Gf.Vec3f(*(center - side * width - sheet_normal * edge_drop)),
        ))

    counts, indices = [], []
    for station in range(LEAF_STATIONS - 1):
        a, b = station * 3, (station + 1) * 3
        counts.extend((3, 3, 3, 3))
        indices.extend((a, a + 1, b + 1, a, b + 1, b,
                        a + 1, a + 2, b + 2, a + 1, b + 2, b + 1))

    _author_mesh(stage, path, (points, counts, indices), color)


def _author_rachis(stage, path, x, color):
    p0 = Gf.Vec3d(x, -RACHIS_LENGTH_M / 2.0, 0.055)
    p1 = Gf.Vec3d(x, RACHIS_LENGTH_M / 2.0, 0.055)
    centers, tangents = _sample_line(p0, p1)
    _author_mesh(stage, path, _tube_data(centers, tangents, [RACHIS_RADIUS_M] * len(centers)), color)


def _author_straight_pair(stage, root_path, x, y):
    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        root = Gf.Vec3d(x, y, 0.055)
        outward = Gf.Vec3d(sign, 0.0, 0.0)
        tip = root + outward * PETIOLULE_LENGTH_M
        centers, tangents = _sample_line(root, tip)
        _author_mesh(
            stage,
            f"{root_path}/{side_name}Petiolule",
            _tube_data(
                centers,
                tangents,
                _taper_radii(PETIOLULE_ROOT_RADIUS_M, PETIOLULE_TIP_RADIUS_M, len(centers)),
            ),
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
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)

    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        key = f"{root_path}/{side_name}"
        root = Gf.Vec3d(x, y, 0.055)

        outward = _rotate_about_up(
            Gf.Vec3d(sign, 0.0, 0.0),
            LEAF_AZIMUTH_VARIATION_DEG * _stable_signed(key, "azimuth"),
        )
        tilt_deg = _stable_range(key, "tilt", *PETIOLULE_TILT_RANGE_DEG)
        direction = _tilted_direction(outward, tilt_deg)

        petiolule_length = PETIOLULE_LENGTH_M * _stable_range(
            key, "petiolule_length", *PETIOLULE_LENGTH_SCALE_RANGE
        )
        radius_scale = _stable_range(key, "radius", *PETIOLULE_RADIUS_SCALE_RANGE)

        # Almost straight petiolule: only a sub-millimetre visual perturbation.
        curve_bias = 0.00025 * _stable_signed(key, "curve")
        p0 = root
        p1 = root + direction * (petiolule_length * 0.33) + world_up * (curve_bias * 0.3)
        p2 = root + direction * (petiolule_length * 0.68) + world_up * curve_bias
        p3 = root + direction * petiolule_length
        centers, tangents = _sample_cubic(p0, p1, p2, p3)

        _author_mesh(
            stage,
            f"{root_path}/{side_name}Petiolule",
            _tube_data(
                centers,
                tangents,
                _taper_radii(
                    PETIOLULE_ROOT_RADIUS_M * radius_scale,
                    PETIOLULE_TIP_RADIUS_M * radius_scale,
                    len(centers),
                ),
            ),
            PETIOLULE_COLOR,
        )

        leaf_length = LEAF_LENGTH_M * _stable_range(key, "leaf_length", *LEAF_LENGTH_SCALE_RANGE)
        leaf_width = LEAF_HALF_WIDTH_M * _stable_range(key, "leaf_width", *LEAF_WIDTH_SCALE_RANGE)
        fold = LEAF_LONGITUDINAL_FOLD_M * _stable_range(key, "fold", *LEAF_FOLD_SCALE_RANGE)
        arch = LEAF_ARCH_LIFT_M * _stable_range(key, "arch", *LEAF_ARCH_SCALE_RANGE)
        sag = LEAF_TIP_SAG_M * _stable_range(key, "sag", *LEAF_SAG_SCALE_RANGE)

        # Tilt remains a tiny perturbation, so only a very small gravity correction
        # is needed; leaf shape variation itself remains independent and visible.
        tilt_norm = (tilt_deg - PETIOLULE_TILT_RANGE_DEG[0]) / (
            PETIOLULE_TILT_RANGE_DEG[1] - PETIOLULE_TILT_RANGE_DEG[0]
        )
        sag *= 0.97 + 0.06 * tilt_norm

        distal_forward = _normalized(tangents[-1] * 0.94 + outward * 0.06)
        _author_leaf_blade(
            stage,
            f"{root_path}/{side_name}Leaf",
            centers[-1],
            distal_forward,
            length=leaf_length,
            half_width=leaf_width,
            fold_depth=fold,
            arch_lift=arch,
            tip_sag=sag,
            color=LEAF_COLOR,
        )

        print(
            f"[LEAF] {key} | tilt={tilt_deg:+.1f}deg | "
            f"radius={radius_scale:.2f}x | length={leaf_length / LEAF_LENGTH_M:.2f}x | "
            f"width={leaf_width / LEAF_HALF_WIDTH_M:.2f}x"
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
        _author_proposed_pair(stage, f"/World/Proposed/Pair_{index + 1:02d}", proposed_x, y)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr().Set(650.0)

    distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    distant.CreateIntensityAttr().Set(1800.0)
    distant.CreateAngleAttr().Set(0.8)
    UsdGeom.Xformable(distant.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 25.0))

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr().Set(52.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    view = Gf.Matrix4d(1.0)
    view.SetLookAt(
        Gf.Vec3d(0.78, -1.15, 0.48),
        Gf.Vec3d(0.0, 0.0, 0.06),
        Gf.Vec3d(0.0, 0.0, 1.0),
    )
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(view.GetInverse())


def main():
    stage = Usd.Stage.CreateNew(OUTPUT_USD)
    _author_scene(stage)
    stage.GetRootLayer().Save()

    print("=" * 76)
    print("TEST 5A v8 - BROAD LEAF VARIATION, SUBTLE PETIOLULE TILT")
    print("=" * 76)
    print(f"USD: {OUTPUT_USD}")
    print("LEFT  : short straight petiolule + flat 2D blade")
    print("RIGHT : varied blades/petiolules with near-level vertical inclination")
    print("Tilt range: -2 to +4 degrees; broader size/fold/arch/sag variation restored")
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
