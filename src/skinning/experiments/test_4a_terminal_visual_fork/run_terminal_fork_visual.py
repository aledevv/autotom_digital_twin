"""Visual-only test for an organic terminal fork / fake young shoot.

This experiment is intentionally isolated from exporterV2.
It validates only the final geometry idea before integration:

    existing parent axis
           |
           |____ existing organ (left/lower arm)
            \
             \___ fake young shoot + small leaf (right/upper arm)

Nothing here has physics, joints, collision, UsdSkel, or runtime sync.
The pieces are static meshes that overlap at the fork, matching the intended
realtime segmented backend strategy.

Two specimens are authored side by side:
  LEFT  = main-stem tip + existing truss-like arm + fake young shoot
  RIGHT = lateral-branch tip + existing leaf-axis arm + fake young shoot

Tune the constants in the "VISUAL TUNING" section only after inspecting Isaac Sim.
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
CURVE_SAMPLES = 15

# A subtle swollen node hides the otherwise obvious flat terminal cut.
NODE_BULGE_SCALE = 1.12
NODE_BULGE_LENGTH = 0.020

# Both children begin below/inside the parent endpoint. This is the key trick:
# there is no boolean Y union, only controlled mesh intersection.
ROOT_OVERLAP_M = 0.014

# Existing organ: deliberately a little more mature/thicker.
EXISTING_ROOT_SCALE = 0.62
EXISTING_TIP_SCALE = 0.46

# Fake shoot: young, narrower, strongly tapered and upward biased.
FAKE_ROOT_SCALE = 0.58
FAKE_TIP_SCALE = 0.24
FAKE_LENGTH_M = 0.060

# Leaf on the fake shoot. Placeholder only; exporter integration can reuse the
# final leaf asset later without changing the fork geometry.
LEAF_LENGTH_M = 0.030
LEAF_HALF_WIDTH_M = 0.010

# Display colors only for the isolated test.
STEM_COLOR = Gf.Vec3f(0.42, 0.68, 0.30)
EXISTING_COLOR = Gf.Vec3f(0.35, 0.60, 0.25)
FAKE_COLOR = Gf.Vec3f(0.48, 0.72, 0.32)
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
    """Simple parallel-transport frames for a static swept tube."""
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
            face_indices.extend((
                center_index,
                next_radial,
                radial,
            ))

    if cap_end:
        center_index = len(points)
        points.append(Gf.Vec3f(*centers[-1]))
        row = (len(centers) - 1) * RADIAL_SEGMENTS
        for radial in range(RADIAL_SEGMENTS):
            next_radial = (radial + 1) % RADIAL_SEGMENTS
            face_counts.append(3)
            face_indices.extend((
                center_index,
                row + radial,
                row + next_radial,
            ))

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


def _taper_radii(start_radius, end_radius, count, bulge=False):
    radii = []
    for index in range(count):
        t = index / float(count - 1)
        blend = _smoothstep(t)
        radius = start_radius + (end_radius - start_radius) * blend
        if bulge:
            # Local swelling near the terminal node, not along the full branch.
            radius *= 1.0 + 0.08 * math.exp(-0.5 * ((t - 0.78) / 0.13) ** 2)
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

    points = Vt.Vec3fArray([
        Gf.Vec3f(*p0),
        Gf.Vec3f(*p1),
        Gf.Vec3f(*p2),
        Gf.Vec3f(*p3),
        Gf.Vec3f(*p4),
        Gf.Vec3f(*p5),
    ])

    # Fan from root; doubleSided removes the need to duplicate the reverse faces.
    face_counts = Vt.IntArray([3, 3, 3, 3])
    face_indices = Vt.IntArray([
        0, 1, 2,
        0, 2, 3,
        0, 3, 4,
        0, 4, 5,
    ])

    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr().Set(points)
    mesh.CreateFaceVertexCountsAttr().Set(face_counts)
    mesh.CreateFaceVertexIndicesAttr().Set(face_indices)
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([LEAF_COLOR]))


def _author_fork_specimen(
    stage,
    root_path,
    parent_start,
    junction,
    parent_radius,
    existing_end,
    existing_control,
    fake_end,
    fake_control,
):
    UsdGeom.Xform.Define(stage, root_path)

    parent_axis = _normalized(junction - parent_start)
    overlap_root = junction - parent_axis * ROOT_OVERLAP_M

    # Parent mesh: leave the terminal ring OPEN. The two child roots overlap it,
    # so the eye reads one continuous fork instead of a sawn-off cylinder cap.
    parent_centers, parent_tangents = _sample_line(parent_start, junction)
    parent_radii = []
    parent_length = _length(junction - parent_start)
    for index in range(len(parent_centers)):
        t = index / float(len(parent_centers) - 1)
        distance_to_tip = (1.0 - t) * parent_length
        bulge = math.exp(
            -0.5 * (distance_to_tip / max(NODE_BULGE_LENGTH, 1e-8)) ** 2
        )
        parent_radii.append(
            parent_radius * (1.0 + (NODE_BULGE_SCALE - 1.0) * bulge)
        )

    data = _tube_mesh_data(
        parent_centers,
        parent_tangents,
        parent_radii,
        cap_start=True,
        cap_end=False,
    )
    _author_mesh(stage, f"{root_path}/Parent", *data, STEM_COLOR)

    # Existing organ arm. It starts INSIDE the parent and curves away naturally.
    existing_centers, existing_tangents = _sample_quadratic(
        overlap_root,
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

    # Fake young shoot. Also starts inside the parent, but goes to the
    # complementary side and upward, forming the visual Y.
    fake_centers, fake_tangents = _sample_quadratic(
        overlap_root,
        fake_control,
        fake_end,
    )
    fake_radii = _taper_radii(
        parent_radius * FAKE_ROOT_SCALE,
        parent_radius * FAKE_TIP_SCALE,
        len(fake_centers),
    )
    data = _tube_mesh_data(
        fake_centers,
        fake_tangents,
        fake_radii,
        cap_start=False,
        cap_end=True,
    )
    _author_mesh(stage, f"{root_path}/FakeYoungShoot", *data, FAKE_COLOR)

    leaf_forward = fake_tangents[-1] + Gf.Vec3d(0.24, 0.05, 0.20)
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

    # -------------------------------------------------------------------------
    # LEFT: main-stem terminal fork.
    # Existing left arm stands in for the truss; fake shoot goes up/right.
    # -------------------------------------------------------------------------
    left_x = -0.11
    junction = _vec(left_x, 0.0, 0.145)
    _author_fork_specimen(
        stage,
        "/World/MainStemFork",
        parent_start=_vec(left_x, 0.0, 0.020),
        junction=junction,
        parent_radius=0.0085,
        existing_control=junction + _vec(-0.028, 0.002, 0.010),
        existing_end=junction + _vec(-0.072, 0.004, -0.014),
        fake_control=junction + _vec(0.018, -0.002, 0.026),
        fake_end=junction + _vec(0.038, -0.004, FAKE_LENGTH_M),
    )

    # -------------------------------------------------------------------------
    # RIGHT: lateral branch terminal fork.
    # Parent is mostly horizontal; existing leaf-axis arm keeps its current side,
    # fake shoot curves to the complementary upper side.
    # -------------------------------------------------------------------------
    right_junction = _vec(0.145, 0.0, 0.100)
    _author_fork_specimen(
        stage,
        "/World/LateralBranchFork",
        parent_start=_vec(0.015, 0.0, 0.074),
        junction=right_junction,
        parent_radius=0.0070,
        existing_control=right_junction + _vec(0.018, 0.002, -0.014),
        existing_end=right_junction + _vec(0.055, 0.004, -0.035),
        fake_control=right_junction + _vec(0.012, -0.002, 0.025),
        fake_end=right_junction + _vec(0.037, -0.004, 0.055),
    )

    # Lighting: neutral and simple, only to inspect silhouette and seams.
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
    print("TEST 4A — TERMINAL VISUAL FORK (STATIC GEOMETRY ONLY)")
    print("=" * 78)
    print()
    print("Purpose:")
    print("  Validate the visual Y / fake-young-shoot geometry BEFORE exporterV2.")
    print()
    print("Scene:")
    print("  LEFT  = main-stem tip + existing truss-like arm + fake young shoot")
    print("  RIGHT = lateral branch + existing leaf-axis arm + fake young shoot")
    print()
    print("Important:")
    print("  - NO physics")
    print("  - NO UsdSkel")
    print("  - NO runtime synchronization")
    print("  - children intentionally overlap the parent mesh")
    print("  - existing organ is NOT replaced; it is one arm of the visual Y")
    print()
    print("GO if:")
    print("  [ ] terminal cut is no longer visually obvious")
    print("  [ ] fork reads as one biological node")
    print("  [ ] fake shoot looks young, thin and slightly curved")
    print("  [ ] overlap does not look like two tubes simply pasted together")
    print("  [ ] Y is asymmetric / organic rather than geometrically straight")
    print()
    print("If shape is wrong, tune only the constants/control points in this test.")
    print("Do NOT integrate into exporterV2 yet.")
    print("=" * 78)

    build_stage(OUTPUT_USD)
    ctx = omni.usd.get_context()
    ctx.open_stage(OUTPUT_USD)

    # Give Kit a few updates to finish opening the stage.
    for _ in range(8):
        simulation_app.update()

    print(f"\nUSD: {OUTPUT_USD}")
    print("Inspect both forks from front and an oblique angle. Close Isaac Sim to exit.\n")

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
