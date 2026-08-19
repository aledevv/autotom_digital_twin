"""Visual-only comparison for static curved petiolules and gently drooping leaves.

This experiment is intentionally isolated from exporterV2. It compares the
current straight-petiolule look with a proposed static organic representation:

- the petiolule rises from the rachis;
- its centerline follows a small cubic arc;
- the petiolule tapers toward the blade;
- the blade receives a mild static downward sag that approximates gravity;
- no physics, joints, UsdSkel, or runtime deformation are used.

The test is intentionally larger than the production dimensions so the shape is
easy to inspect. If the visual language is accepted, the same normalized curve
can be scaled to the real petiolule/leaf dimensions in exporterV2.
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
CURVE_SAMPLES = 18
LEAF_STATIONS = 12

RACHIS_LENGTH_M = 0.36
RACHIS_RADIUS_M = 0.0060

# Enlarged for the isolated visual test. The production implementation should
# scale this curve according to the actual blade size / GroIMP-derived leaf data.
PETIOLULE_LENGTH_M = 0.045
PETIOLULE_ROOT_RADIUS_M = 0.0032
PETIOLULE_TIP_RADIUS_M = 0.0017

LEAF_LENGTH_M = 0.075
LEAF_HALF_WIDTH_M = 0.025
LEAF_SAG_M = 0.014
LEAF_CAMBER_M = 0.003

# Three pairs on the proposed specimen. Slight variation is deliberate so the
# shape does not look mechanically cloned.
PAIR_Y = (-0.105, 0.0, 0.105)
PAIR_LIFT = (0.026, 0.032, 0.024)
PAIR_END_DROP = (0.006, 0.008, 0.005)

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
    camber,
    color,
):
    """Create a static curved blade with a mild gravity-like distal sag."""
    forward = _normalized(forward)
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)
    side = Gf.Cross(world_up, forward)
    if _length(side) <= 1e-8:
        side = Gf.Cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)

    points = []
    for index in range(LEAF_STATIONS):
        t = index / float(LEAF_STATIONS - 1)
        width_profile = math.sin(math.pi * t) ** 0.82
        width_profile *= 1.10 - 0.24 * t
        width = half_width * width_profile

        center = (
            root
            + forward * (length * t)
            + world_up * (
                camber * math.sin(math.pi * t)
                - sag * (t ** 1.65)
            )
        )

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
            camber=0.0,
            color=LEAF_COLOR,
        )


def _author_curved_pair(stage, root_path, x, y, lift, end_drop):
    """Proposed look: upward curved static petiolules + slightly drooping blades."""
    world_up = Gf.Vec3d(0.0, 0.0, 1.0)

    for side_name, sign in (("Left", -1.0), ("Right", 1.0)):
        root = Gf.Vec3d(x, y, 0.055)
        outward = Gf.Vec3d(sign, 0.0, 0.0)
        length = PETIOLULE_LENGTH_M

        # Cubic normalized arc. The root tangent initially rises strongly, the
        # middle reaches the maximum lift, and the distal tangent relaxes
        # slightly downward before the blade starts. This mimics the upward arch
        # visible in tomato petiolules without needing another articulated joint.
        p0 = root
        p1 = root + outward * (length * 0.24) + world_up * (lift * 0.70)
        p2 = root + outward * (length * 0.70) + world_up * lift
        p3 = root + outward * length + world_up * (lift - end_drop)

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

        # Use the actual distal tangent, but blend it slightly with the radial
        # direction so the blade does not point vertically after a strong arch.
        distal_forward = _normalized(tangents[-1] + outward * 0.70)
        _author_leaf_blade(
            stage,
            f"{root_path}/{side_name}Leaf",
            centers[-1],
            distal_forward,
            length=LEAF_LENGTH_M,
            half_width=LEAF_HALF_WIDTH_M,
            sag=LEAF_SAG_M,
            camber=LEAF_CAMBER_M,
            color=LEAF_COLOR,
        )


def _author_scene(stage):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World")

    # LEFT specimen = current straight representation.
    current_x = -0.28
    UsdGeom.Xform.Define(stage, "/World/Current")
    _author_rachis(stage, "/World/Current/Rachis", current_x, CURRENT_COLOR)
    _author_straight_pair(stage, "/World/Current/Pair", current_x, 0.0)

    # RIGHT specimen = proposed static organic representation.
    proposed_x = 0.24
    UsdGeom.Xform.Define(stage, "/World/Proposed")
    _author_rachis(stage, "/World/Proposed/Rachis", proposed_x, STEM_COLOR)
    for index, (y, lift, drop) in enumerate(zip(PAIR_Y, PAIR_LIFT, PAIR_END_DROP)):
        _author_curved_pair(
            stage,
            f"/World/Proposed/Pair_{index + 1:02d}",
            proposed_x,
            y,
            lift,
            drop,
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
    eye = Gf.Vec3d(0.78, -1.15, 0.58)
    target = Gf.Vec3d(0.0, 0.0, 0.085)
    view = Gf.Matrix4d(1.0)
    view.SetLookAt(eye, target, Gf.Vec3d(0.0, 0.0, 1.0))
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(view.GetInverse())


def main():
    stage = Usd.Stage.CreateNew(OUTPUT_USD)
    _author_scene(stage)
    stage.GetRootLayer().Save()

    print("=" * 76)
    print("TEST 5A - STATIC CURVED PETIOLULE VISUAL")
    print("=" * 76)
    print(f"USD: {OUTPUT_USD}")
    print("LEFT  : current straight petiolule + flat blade")
    print("RIGHT : proposed upward cubic arc + tapered petiolule + blade sag")
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
