"""Visual-only comparison for short tomato petiolules and curved leaf blades.

This experiment is intentionally isolated from exporterV2. It compares the
current straight-petiolule/flat-blade look with a revised static organic model
based on the visual proportions observed in real tomato leaves:

- the petiolule is short and thin relative to the leaf blade;
- it has only a small upward tilt and very mild curvature;
- most of the visible curvature belongs to the leaf blade itself;
- the blade first keeps a small basal lift, then bends downward toward the tip;
- no physics, joints, UsdSkel, or runtime deformation are used.

If accepted, the normalized blade deformation can be transferred to the real
GroIMP-derived leaf dimensions while keeping petiolules rigid and inexpensive.
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
LEAF_STATIONS = 16

RACHIS_LENGTH_M = 0.36
RACHIS_RADIUS_M = 0.0060

# The important proportion in this revision: the petiolule is only ~19% of the
# blade length instead of the previous ~60%. This is much closer to the tomato
# references and also makes a rigid petiolule visually plausible.
LEAF_LENGTH_M = 0.075
LEAF_HALF_WIDTH_M = 0.025
PETIOLULE_LENGTH_M = 0.014
PETIOLULE_ROOT_RADIUS_M = 0.0024
PETIOLULE_TIP_RADIUS_M = 0.00135

# Only a few millimetres of vertical change belong to the petiolule. Most of the
# silhouette variation now comes from the leaf blade.
PETIOLULE_LIFT_M = 0.0032

# Blade rest-shape. The blade initially gains a little height, then gravity-like
# sag dominates progressively toward the distal tip.
LEAF_BASAL_LIFT_M = 0.0055
LEAF_SAG_M = 0.018
LEAF_SAG_EXPONENT = 1.72
LEAF_CAMBER_M = 0.0025

# Three proposed pairs use only small deterministic variation.
PAIR_Y = (-0.105, 0.0, 0.105)
PAIR_LIFT_SCALE = (0.88, 1.00, 0.93)
PAIR_SAG_SCALE = (0.90, 1.00, 0.94)

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
    sag,
    basal_lift,
    camber,
    color,
):
    """Create a blade whose rest shape carries most of the visible curvature."""
    forward = _normalized(forward)
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    side = Gf.Cross(world_up, forward)
    if _length(side) <= 1e-8:
        side = Gf.Cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)

    points = []
    for index in range(LEAF_STATIONS):
        t = index / float(LEAF_STATIONS - 1)

        # Tomato-like blade width: broad after the narrow base, then taper to tip.
        width_profile = math.sin(math.pi * t) ** 0.78
        width_profile *= 1.13 - 0.27 * t
        width = half_width * width_profile

        # The first term creates a gentle basal rise that peaks early. The last
        # term is the gravity-like sag and dominates only toward the distal tip.
        # A tiny sinusoidal camber prevents an unnaturally flat center section.
        basal_shape = math.sin(math.pi * min(t / 0.62, 1.0)) if t < 0.62 else 0.0
        vertical_offset = (
            basal_lift * basal_shape
            + camber * math.sin(math.pi * t)
            - sag * (t ** LEAF_SAG_EXPONENT)
        )
        center = root + forward * (length * t) + world_up * vertical_offset

        points.append(Gf.Vec3f(*(center + side * width)))
        points.append(Gf.Vec3f(*(center - side * width)))

    face_counts = []
    face_indices = []
    for station in range(LEAF_STATIONS - 1):
        a = station * 2
        b = a + 1
        c = a + 2
        d = a + 3
        face_counts.extend((3, 3))
        face_indices.extend((a, b, d, a, d, c))

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
    """Current look: straight rigid petiolules and flat blades."""
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
            sag=0.0,
            basal_lift=0.0,
            camber=0.0,
            color=LEAF_COLOR,
        )


def _author_curved_pair(stage, root_path, x, y, lift_scale, sag_scale):
    """Proposed look: short petiolule, subtle tilt, curvature mainly in blade."""
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)

    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        root = Gf.Vec3d(x, y, 0.055)
        outward = Gf.Vec3d(sign, 0.0, 0.0)
        length = PETIOLULE_LENGTH_M
        lift = PETIOLULE_LIFT_M * lift_scale

        # Mild cubic arc: almost straight, with only a small upward change.
        # The distal tangent remains close to the radial direction so the leaf
        # starts naturally instead of inheriting an exaggerated hook.
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

        # Preserve the short petiolule tangent, but make the blade direction
        # mostly radial. The deformation of the blade mesh then supplies the
        # visible gravity response.
        distal_forward = _normalized(outward * 0.86 + tangents[-1] * 0.14)
        _author_leaf_blade(
            stage,
            f"{root_path}/{side_name}Leaf",
            centers[-1],
            distal_forward,
            length=LEAF_LENGTH_M,
            half_width=LEAF_HALF_WIDTH_M,
            sag=LEAF_SAG_M * sag_scale,
            basal_lift=LEAF_BASAL_LIFT_M * lift_scale,
            camber=LEAF_CAMBER_M,
            color=LEAF_COLOR,
        )


def _author_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")

    # LEFT = baseline with the new shorter proportion but no organic deformation.
    # This makes the comparison focus on shape rather than simply on length.
    current_x = -0.28
    UsdGeom.Xform.Define(stage, "/World/Current")
    _author_rachis(stage, "/World/Current/Rachis", current_x, CURRENT_COLOR)
    _author_straight_pair(stage, "/World/Current/Pair", current_x, 0.0)

    # RIGHT = proposed visual language with three slightly different pairs.
    proposed_x = 0.24
    UsdGeom.Xform.Define(stage, "/World/Proposed")
    _author_rachis(stage, "/World/Proposed/Rachis", proposed_x, STEM_COLOR)
    for index, (y, lift_scale, sag_scale) in enumerate(
        zip(PAIR_Y, PAIR_LIFT_SCALE, PAIR_SAG_SCALE)
    ):
        _author_curved_pair(
            stage,
            f"/World/Proposed/Pair_{index + 1:02d}",
            proposed_x,
            y,
            lift_scale,
            sag_scale,
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
    print("TEST 5A v2 - SHORT PETIOLULE + CURVED LEAF REST SHAPE")
    print("=" * 76)
    print(f"USD: {OUTPUT_USD}")
    print("LEFT  : short straight petiolule + flat blade")
    print("RIGHT : short mildly raised petiolule + curved/sagged blade")
    print(
        f"Ratio: petiolule={PETIOLULE_LENGTH_M:.3f} m / "
        f"blade={LEAF_LENGTH_M:.3f} m = "
        f"{PETIOLULE_LENGTH_M / LEAF_LENGTH_M:.2f}"
    )
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
