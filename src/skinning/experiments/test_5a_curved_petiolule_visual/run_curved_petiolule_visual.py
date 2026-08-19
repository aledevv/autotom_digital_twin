"""Visual-only test for short tomato petiolules and longitudinally folded blades.

The test stays isolated from exporterV2.  The proposed shape now follows the
latest visual target:

- petiolules are short, thin, and only mildly tilted upward;
- leaf blades remain simple 2D triangle sheets;
- the dominant fold runs longitudinally from the petiolule to the leaf tip;
- a center vertex row acts like a visual midrib, with the two blade halves
  slightly lowered around it;
- only a small whole-blade distal sag remains, so the leaf does not look bent by
  a transverse/latitudinal crease;
- no physics, joints, UsdSkel, or runtime deformation are used.
"""

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
PETIOLULE_LIFT_M = 0.0032

# Main change of this revision: a longitudinal V-like fold around the visual
# midrib.  The fold is zero at the narrow base/tip and strongest over the broad
# central portion of the blade.
LEAF_LONGITUDINAL_FOLD_M = 0.0048
LEAF_FOLD_EXPONENT = 0.78

# Keep only a subtle whole-blade gravity sag.  It should not read as a transverse
# bend; the longitudinal midrib fold above must dominate the shape.
LEAF_TIP_SAG_M = 0.0045
LEAF_TIP_SAG_EXPONENT = 1.9

PAIR_Y = (-0.105, 0.0, 0.105)
PAIR_LIFT_SCALE = (0.88, 1.00, 0.93)
PAIR_FOLD_SCALE = (0.90, 1.00, 0.94)

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

    for center, normal, binormal, radius in zip(
        centers,
        normals,
        binormals,
        radii,
    ):
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
    tip_sag,
    color,
):
    """Create a 2D sheet folded along a petiolule-to-tip visual midrib.

    Each longitudinal station has three vertices: left edge, center/midrib, and
    right edge.  The center stays on the blade centerline while both edges are
    lowered along the sheet normal.  Connecting those rows produces two simple
    triangle strips meeting at one longitudinal ridge.
    """
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

        # Very small whole-blade sag. It moves the centerline smoothly and does
        # not create a transverse crease.
        center = (
            root
            + forward * (length * t)
            - world_up * (tip_sag * (t ** LEAF_TIP_SAG_EXPONENT))
        )

        # Longitudinal fold: maximum near the broad middle of the blade, zero at
        # base/tip. The midrib is the raised center row and can be read as a
        # continuation of the petiolule toward the distal tip.
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

        # Left half of the blade.
        face_counts.extend((3, 3))
        face_indices.extend((
            row0 + 0,
            row0 + 1,
            row1 + 1,
            row0 + 0,
            row1 + 1,
            row1 + 0,
        ))

        # Right half of the blade.
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
            tip_sag=0.0,
            color=LEAF_COLOR,
        )


def _author_proposed_pair(stage, root_path, x, y, lift_scale, fold_scale):
    """Proposed: subtle petiolule lift + longitudinally folded blade."""
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)

    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        root = Gf.Vec3d(x, y, 0.055)
        outward = Gf.Vec3d(sign, 0.0, 0.0)
        length = PETIOLULE_LENGTH_M
        lift = PETIOLULE_LIFT_M * lift_scale

        p0 = root
        p1 = root + outward * (length * 0.33) + world_up * (lift * 0.30)
        p2 = root + outward * (length * 0.68) + world_up * (lift * 0.78)
        p3 = root + outward * length + world_up * lift

        centers, tangents = _sample_cubic(p0, p1, p2, p3)
        radii = _taper_radii(
            PETIOLULE_ROOT_RADIUS_M,
            PETIOLULE_TIP_RADIUS_M,
            len(centers),
        )
        _author_mesh(
            stage,
            f"{root_path}/{side_name}Petiolule",
            _tube_data(centers, tangents, radii),
            PETIOLULE_COLOR,
        )

        distal_forward = _normalized(outward * 0.86 + tangents[-1] * 0.14)
        _author_leaf_blade(
            stage,
            f"{root_path}/{side_name}Leaf",
            centers[-1],
            distal_forward,
            length=LEAF_LENGTH_M,
            half_width=LEAF_HALF_WIDTH_M,
            fold_depth=LEAF_LONGITUDINAL_FOLD_M * fold_scale,
            tip_sag=LEAF_TIP_SAG_M,
            color=LEAF_COLOR,
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
    for index, (y, lift_scale, fold_scale) in enumerate(
        zip(PAIR_Y, PAIR_LIFT_SCALE, PAIR_FOLD_SCALE)
    ):
        _author_proposed_pair(
            stage,
            f"/World/Proposed/Pair_{index + 1:02d}",
            proposed_x,
            y,
            lift_scale,
            fold_scale,
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
    print("TEST 5A v3 - LONGITUDINAL LEAF MIDRIB FOLD")
    print("=" * 76)
    print(f"USD: {OUTPUT_USD}")
    print("LEFT  : short straight petiolule + flat 2D blade")
    print("RIGHT : short raised petiolule + 2D blade folded along its midrib")
    print("Blade topology: left edge / midrib / right edge per station")
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
