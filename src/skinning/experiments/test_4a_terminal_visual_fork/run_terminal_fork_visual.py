"""Visual-only test for an organic terminal fork / fake young shoot.

This experiment stays completely isolated from exporterV2.  It validates one
specific visual rule for the segmented realtime backend:

    the parent axis does NOT visually stop at the fork;
    the fake young shoot is treated as the smooth continuation of that axis,
    while the already-existing organ becomes the lateral arm of the fork.

Nothing here has physics, joints, collision, UsdSkel, or runtime sync.  The fork
is built only with carefully overlapping static meshes.

Two specimens are authored side by side:
  LEFT  = main-stem tip + existing truss-like arm + continuing young shoot
  RIGHT = lateral-branch tip + existing leaf-axis arm + continuing young shoot
"""

import math
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1400,
    "height": 900,
})

import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdLux, Vt


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)
OUTPUT_USD = os.path.join(OUTPUT_DIR, "terminal_visual_fork.usda")


# =============================================================================
# VISUAL TUNING
# =============================================================================

RADIAL_SEGMENTS = 14
CURVE_SAMPLES = 18

# The node is only slightly swollen.  The old test used a stronger bulge and it
# contributed to the blocky / sawn-off appearance.
NODE_BULGE_SCALE = 1.055
NODE_BULGE_LENGTH = 0.018

# Existing lateral organ root: short overlap into the parent.
SIDE_ROOT_OVERLAP_M = 0.010
EXISTING_ROOT_SCALE = 0.64
EXISTING_TIP_SCALE = 0.46

# Continuation shoot: starts much deeper inside the parent.  The parent itself
# continues a few millimetres *past* the nominal fork point and tapers inside
# this shoot.  Neither mesh has a visible cap at the junction.
CONTINUATION_ROOT_OVERLAP_M = 0.018
PARENT_UNDERLAP_M = 0.010
PARENT_UNDERLAP_END_SCALE = 0.72
CONTINUATION_ALIGN_M = 0.025

# Young continuation shoot.  Its root is intentionally much closer to the
# parent radius than in the first version, then it tapers strongly after it has
# cleared the hidden overlap zone.
FAKE_ROOT_SCALE = 0.90
FAKE_TIP_SCALE = 0.24
FAKE_LENGTH_M = 0.060
FAKE_ROOT_ZONE_FRACTION = 0.28

# Placeholder leaf on the fake young shoot.
LEAF_LENGTH_M = 0.030
LEAF_HALF_WIDTH_M = 0.010

# Same color for parent and continuation: the geometry, not a material boundary,
# should define the visual transition in this isolated test.
STEM_COLOR = Gf.Vec3f(0.42, 0.68, 0.30)
EXISTING_COLOR = Gf.Vec3f(0.35, 0.60, 0.25)
FAKE_COLOR = STEM_COLOR
LEAF_COLOR = Gf.Vec3f(0.24, 0.52, 0.18)


# =============================================================================
# SMALL GEOMETRY TOOLKIT
# =============================================================================


def _vec(x, y, z):
    return Gf.Vec3d(float(x), float(y), float(z))


def _length(v):
    return math.sqrt(float(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]))


def _normalized(v):
    length = _length(v)
    if length <= 1e-12:
        raise ValueError("Cannot normalize a zero-length vector")
    return Gf.Vec3d(v[0] / length, v[1] / length, v[2] / length)


def _dot(a, b):
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def _cross(a, b):
    return Gf.Vec3d(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _smoothstep(value):
    value = max(0.0, min(1.0, float(value)))
    return value * value * (3.0 - 2.0 * value)


def _quadratic_bezier(p0, p1, p2, t):
    u = 1.0 - t
    return p0 * (u * u) + p1 * (2.0 * u * t) + p2 * (t * t)


def _quadratic_tangent(p0, p1, p2, t):
    tangent = (p1 - p0) * (2.0 * (1.0 - t)) + (p2 - p1) * (2.0 * t)
    return _normalized(tangent)


def _sample_quadratic(p0, p1, p2, count=CURVE_SAMPLES):
    centers = []
    tangents = []
    for index in range(count):
        t = index / float(count - 1)
        centers.append(_quadratic_bezier(p0, p1, p2, t))
        tangents.append(_quadratic_tangent(p0, p1, p2, t))
    return centers, tangents


def _sample_line(p0, p1, count=CURVE_SAMPLES):
    centers = []
    tangent = _normalized(p1 - p0)
    tangents = []
    for index in range(count):
        t = index / float(count - 1)
        centers.append(p0 * (1.0 - t) + p1 * t)
        tangents.append(tangent)
    return centers, tangents


def _transport_frames(tangents):
    first_tangent = tangents[0]
    reference = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(_dot(first_tangent, reference)) > 0.92:
        reference = Gf.Vec3d(1.0, 0.0, 0.0)

    normal = _normalized(_cross(reference, first_tangent))
    binormal = _normalized(_cross(first_tangent, normal))

    normals = [normal]
    binormals = [binormal]
    previous_tangent = first_tangent
    previous_normal = normal

    for tangent in tangents[1:]:
        axis = _cross(previous_tangent, tangent)
        axis_length = _length(axis)

        if axis_length > 1e-10:
            axis = _normalized(axis)
            cosine = max(-1.0, min(1.0, _dot(previous_tangent, tangent)))
            angle_deg = math.degrees(math.acos(cosine))
            rotation = Gf.Rotation(axis, angle_deg)
            normal = _normalized(rotation.TransformDir(previous_normal))
        else:
            normal = previous_normal

        binormal = _normalized(_cross(tangent, normal))
        normal = _normalized(_cross(binormal, tangent))
        normals.append(normal)
        binormals.append(binormal)
        previous_tangent = tangent
        previous_normal = normal

    return normals, binormals


def _tube_mesh_data(centers, tangents, radii, cap_start=True, cap_end=True):
    if not (len(centers) == len(tangents) == len(radii)):
        raise ValueError("centers/tangents/radii length mismatch")

    normals, binormals = _transport_frames(tangents)
    points = []

    for center, normal, binormal, radius in zip(
        centers, normals, binormals, radii
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


def _author_mesh(stage, path, points, face_counts, face_indices, color):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(face_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(face_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([color]))
    return mesh


def _taper_radii(start_radius, end_radius, count):
    radii = []
    for index in range(count):
        t = index / float(count - 1)
        blend = _smoothstep(t)
        radii.append(start_radius + (end_radius - start_radius) * blend)
    return radii


def _continuation_radii(parent_radius, count):
    """Keep a broad hidden root, then taper only after the fork has cleared."""
    radii = []
    root_radius = parent_radius * FAKE_ROOT_SCALE
    tip_radius = parent_radius * FAKE_TIP_SCALE

    for index in range(count):
        t = index / float(count - 1)
        if t <= FAKE_ROOT_ZONE_FRACTION:
            # Very gentle taper while this part is still nested inside / close
            # to the parent continuation.
            q = _smoothstep(t / max(FAKE_ROOT_ZONE_FRACTION, 1e-8))
            radius = root_radius * (1.0 - 0.10 * q)
        else:
            q = _smoothstep(
                (t - FAKE_ROOT_ZONE_FRACTION)
                / max(1.0 - FAKE_ROOT_ZONE_FRACTION, 1e-8)
            )
            shoulder_radius = root_radius * 0.90
            radius = shoulder_radius + (tip_radius - shoulder_radius) * q
        radii.append(radius)

    return radii


def _parent_radii(centers, junction, parent_axis, parent_radius):
    """Create a soft node and a tapered hidden continuation past the fork."""
    radii = []
    for center in centers:
        signed = _dot(center - junction, parent_axis)
        if signed <= 0.0:
            distance = abs(signed)
            bulge = math.exp(
                -0.5 * (distance / max(NODE_BULGE_LENGTH, 1e-8)) ** 2
            )
            radius = parent_radius * (
                1.0 + (NODE_BULGE_SCALE - 1.0) * bulge
            )
        else:
            q = _smoothstep(signed / max(PARENT_UNDERLAP_M, 1e-8))
            radius = parent_radius * (
                NODE_BULGE_SCALE
                + (PARENT_UNDERLAP_END_SCALE - NODE_BULGE_SCALE) * q
            )
        radii.append(radius)
    return radii


def _author_leaf(stage, path, root, forward, up_hint, length, half_width):
    forward = _normalized(forward)
    side = _cross(up_hint, forward)
    if _length(side) < 1e-8:
        side = _cross(Gf.Vec3d(0.0, 1.0, 0.0), forward)
    side = _normalized(side)
    bend = _normalized(_cross(forward, side))

    p0 = root
    p1 = root + forward * (length * 0.28) + side * half_width * 0.92
    p2 = root + forward * (length * 0.62) + side * half_width * 0.72 + bend * 0.002
    p3 = root + forward * length
    p4 = root + forward * (length * 0.62) - side * half_width * 0.72 + bend * 0.002
    p5 = root + forward * (length * 0.28) - side * half_width * 0.92

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(*p0), Gf.Vec3f(*p1), Gf.Vec3f(*p2),
        Gf.Vec3f(*p3), Gf.Vec3f(*p4), Gf.Vec3f(*p5),
    ]))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray([3, 3, 3, 3]))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([
        0, 1, 2,
        0, 2, 3,
        0, 3, 4,
        0, 4, 5,
    ]))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([LEAF_COLOR]))


# =============================================================================
# FORK AUTHORING
# =============================================================================


def _author_fork_specimen(
    stage,
    root_path,
    parent_start,
    junction,
    parent_radius,
    existing_end,
    existing_control,
    fake_end,
):
    UsdGeom.Xform.Define(stage, root_path)

    parent_axis = _normalized(junction - parent_start)

    # ---------------------------------------------------------------------
    # Parent + hidden underlap.
    #
    # The parent no longer stops at the nominal fork point.  It continues
    # inside the fake shoot and tapers there, so there is no exposed terminal
    # ring/cap for the eye to read as a cut.
    # ---------------------------------------------------------------------
    parent_end = junction + parent_axis * PARENT_UNDERLAP_M
    parent_centers, parent_tangents = _sample_line(parent_start, parent_end)
    parent_radii = _parent_radii(
        parent_centers,
        junction,
        parent_axis,
        parent_radius,
    )
    data = _tube_mesh_data(
        parent_centers,
        parent_tangents,
        parent_radii,
        cap_start=True,
        cap_end=False,
    )
    _author_mesh(stage, f"{root_path}/Parent", *data, STEM_COLOR)

    # ---------------------------------------------------------------------
    # Existing organ = lateral arm.
    # It keeps the current organ direction and simply starts slightly inside
    # the swollen node.
    # ---------------------------------------------------------------------
    side_root = junction - parent_axis * SIDE_ROOT_OVERLAP_M
    existing_centers, existing_tangents = _sample_quadratic(
        side_root,
        existing_control,
        existing_end,
    )
    existing_radii = _taper_radii(
        parent_radius * EXISTING_ROOT_SCALE,
        parent_radius * EXISTING_TIP_SCALE,
        len(existing_centers),
    )
    data = _tube_mesh_data(
        existing_centers,
        existing_tangents,
        existing_radii,
        cap_start=False,
        cap_end=True,
    )
    _author_mesh(stage, f"{root_path}/ExistingOrgan", *data, EXISTING_COLOR)

    # ---------------------------------------------------------------------
    # Fake shoot = continuation of the parent, NOT an equal second branch.
    # P1 is placed directly along the incoming parent tangent.  This forces a
    # tangent-continuous start; only later does the shoot bend toward fake_end.
    # ---------------------------------------------------------------------
    continuation_root = (
        junction - parent_axis * CONTINUATION_ROOT_OVERLAP_M
    )
    continuation_control = junction + parent_axis * CONTINUATION_ALIGN_M
    fake_centers, fake_tangents = _sample_quadratic(
        continuation_root,
        continuation_control,
        fake_end,
    )
    fake_radii = _continuation_radii(parent_radius, len(fake_centers))
    data = _tube_mesh_data(
        fake_centers,
        fake_tangents,
        fake_radii,
        cap_start=False,
        cap_end=True,
    )
    _author_mesh(stage, f"{root_path}/FakeYoungShoot", *data, FAKE_COLOR)

    leaf_forward = fake_tangents[-1] + Gf.Vec3d(0.20, 0.04, 0.16)
    _author_leaf(
        stage,
        f"{root_path}/FakeYoungLeaf",
        fake_end,
        leaf_forward,
        Gf.Vec3d(0.0, 1.0, 0.2),
        LEAF_LENGTH_M,
        LEAF_HALF_WIDTH_M,
    )


# =============================================================================
# SCENE
# =============================================================================


def build_stage(path):
    if os.path.exists(path):
        os.remove(path)

    stage = Usd.Stage.CreateNew(path)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # LEFT: main-stem-like terminal fork.  The young shoot continues the
    # incoming parent direction before gently turning upward; the existing
    # truss-like organ is the side arm.
    left_x = -0.11
    left_junction = _vec(left_x, 0.0, 0.145)
    _author_fork_specimen(
        stage,
        "/World/MainStemFork",
        parent_start=_vec(left_x + 0.130, 0.0, 0.145),
        junction=left_junction,
        parent_radius=0.0085,
        existing_control=left_junction + _vec(-0.020, 0.002, -0.012),
        existing_end=left_junction + _vec(-0.070, 0.004, -0.038),
        fake_end=left_junction + _vec(-0.058, -0.004, 0.055),
    )

    # RIGHT: lateral-branch-like fork.  Incoming branch rises gently toward the
    # node.  The continuation keeps that direction first, then curves upward;
    # the existing leaf-axis arm leaves laterally.
    right_junction = _vec(0.145, 0.0, 0.100)
    _author_fork_specimen(
        stage,
        "/World/LateralBranchFork",
        parent_start=_vec(0.020, 0.0, 0.066),
        junction=right_junction,
        parent_radius=0.0070,
        existing_control=right_junction + _vec(0.018, 0.002, 0.002),
        existing_end=right_junction + _vec(0.058, 0.004, -0.018),
        fake_end=right_junction + _vec(0.030, -0.004, 0.060),
    )

    distant = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    distant.CreateIntensityAttr(1700.0)
    distant.CreateAngleAttr(0.5)
    distant_xf = UsdGeom.Xformable(distant)
    distant_xf.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, -25.0))

    fill = UsdLux.SphereLight.Define(stage, "/World/FillLight")
    fill.CreateIntensityAttr(2500.0)
    fill.CreateRadiusAttr(0.04)
    fill_xf = UsdGeom.Xformable(fill)
    fill_xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.35, 0.30))

    stage.GetRootLayer().Save()
    return stage


def main():
    print()
    print("=" * 78)
    print("TEST 4A v2 — CONTINUOUS TERMINAL VISUAL FORK")
    print("=" * 78)
    print()
    print("Change from v1:")
    print("  - fake shoot is now the smooth continuation of the parent axis")
    print("  - parent continues past the fork inside the fake shoot")
    print("  - parent continuation tapers while hidden; no exposed terminal cap")
    print("  - existing organ remains the lateral arm and is otherwise untouched")
    print("  - node bulge is subtler to avoid the blocky squared shoulder")
    print()
    print("LEFT  = main-stem / truss interpretation")
    print("RIGHT = lateral-branch / leaf-axis interpretation")
    print()
    print("GO if:")
    print("  [ ] parent -> young shoot reads as one continuous branch")
    print("  [ ] no obvious flat terminal cut remains at the fork")
    print("  [ ] existing organ reads as a side branch from the same node")
    print("  [ ] overlap does not produce an ugly lump or self-intersection")
    print("  [ ] fake shoot still looks young and tapered")
    print()
    print("Still NOT integrated into exporterV2.")
    print("=" * 78)

    build_stage(OUTPUT_USD)
    ctx = omni.usd.get_context()
    ctx.open_stage(OUTPUT_USD)

    for _ in range(8):
        simulation_app.update()

    print(f"\nUSD: {OUTPUT_USD}")
    print("Inspect both forks from front and oblique angles. Close Isaac Sim to exit.\n")

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
