"""Isolated visual/physics test for a more realistic tomato truss.

Core idea validated by this experiment:

- the truss rachis keeps the existing articulated multi-link physics;
- every pedicel remains ONE rigid body attached to the rachis by the same D6
  joint used by exporterV2;
- the straight cylinder stays as the hidden collision/rigid-body proxy;
- a curved tapered tube is authored as a render-only child of that rigid body;
- the visual tube starts at the D6 attachment and ends at the exact physical
  pedicel tip, therefore the existing tomato FixedJoint remains valid;
- the whole curved pedicel moves rigidly when the root D6 rotates under fruit
  weight. No extra joints, skinning or runtime deformation are needed.

The script writes two stages for comparison and opens the proposed stage:
    output/00_current_truss.usda
    output/01_curved_rigid_pedicels.usda

Press PLAY in Isaac Sim to observe the articulated rachis and D6 pedicels.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
import sys

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1500, "height": 900})

import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, Vt


SCRIPT_DIR = Path(__file__).resolve().parent
SRC_DIR = Path(__file__).resolve().parents[3]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from exporterV2.adapters.groimp_csv.truss_builder import truss_to_complete_config
from exporterV2.core.tree_config import PlantColors, scaled
from exporterV2.core.usd.stage import build_stage


OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
CURRENT_USD = OUTPUT_DIR / "00_current_truss.usda"
PROPOSED_USD = OUTPUT_DIR / "01_curved_rigid_pedicels.usda"

RADIAL_SEGMENTS = 14
CURVE_SAMPLES = 15

# Visual-only curve amplitude relative to physical pedicel length.
CURVE_AMPLITUDE_RANGE = (0.10, 0.19)
SECONDARY_AMPLITUDE_RANGE = (-0.055, 0.055)
ROOT_RADIUS_SCALE_RANGE = (1.15, 1.28)
MID_RADIUS_SCALE_RANGE = (0.82, 0.94)
TIP_RADIUS_SCALE_RANGE = (0.96, 1.08)


# -----------------------------------------------------------------------------
# deterministic variation
# -----------------------------------------------------------------------------


def _stable_unit(key: str, salt: str) -> float:
    digest = hashlib.blake2b(
        f"{key}|{salt}|truss-test-6a".encode("utf-8"),
        digest_size=8,
    ).digest()
    return int.from_bytes(digest, "big") / float((1 << 64) - 1)


def _stable_range(key: str, salt: str, low: float, high: float) -> float:
    return low + (high - low) * _stable_unit(key, salt)


# -----------------------------------------------------------------------------
# geometry helpers
# -----------------------------------------------------------------------------


def _length(v: Gf.Vec3d) -> float:
    return math.sqrt(float(Gf.Dot(v, v)))


def _normalized(v: Gf.Vec3d) -> Gf.Vec3d:
    v = Gf.Vec3d(v)
    length = _length(v)
    if length <= 1e-12:
        raise ValueError("Cannot normalize zero vector")
    return v / length


def _cubic_point(p0, p1, p2, p3, t: float):
    u = 1.0 - t
    return (
        p0 * (u ** 3)
        + p1 * (3.0 * u * u * t)
        + p2 * (3.0 * u * t * t)
        + p3 * (t ** 3)
    )


def _cubic_tangent(p0, p1, p2, p3, t: float):
    u = 1.0 - t
    tangent = (
        (p1 - p0) * (3.0 * u * u)
        + (p2 - p1) * (6.0 * u * t)
        + (p3 - p2) * (3.0 * t * t)
    )
    return _normalized(tangent)


def _sample_centerline(height: float, branch_id: str):
    """Curved local centerline whose endpoints match the physical proxy exactly."""
    amplitude = height * _stable_range(
        branch_id,
        "curve_amp",
        *CURVE_AMPLITUDE_RANGE,
    )
    secondary = height * _stable_range(
        branch_id,
        "curve_secondary",
        *SECONDARY_AMPLITUDE_RANGE,
    )
    phase = 2.0 * math.pi * _stable_unit(branch_id, "curve_phase")
    radial = Gf.Vec3d(math.cos(phase), math.sin(phase), 0.0)
    side = Gf.Vec3d(-math.sin(phase), math.cos(phase), 0.0)

    # The physical link still spans P0 -> P3 along local +Z. The visual path has
    # a strong basal shoulder and relaxes back toward the true terminal point.
    p0 = Gf.Vec3d(0.0, 0.0, 0.0)
    p1 = radial * amplitude + side * (secondary * 0.35) + Gf.Vec3d(0.0, 0.0, 0.27 * height)
    p2 = radial * (0.42 * amplitude) + side * secondary + Gf.Vec3d(0.0, 0.0, 0.72 * height)
    p3 = Gf.Vec3d(0.0, 0.0, height)

    centers, tangents = [], []
    for index in range(CURVE_SAMPLES):
        t = index / float(CURVE_SAMPLES - 1)
        centers.append(_cubic_point(p0, p1, p2, p3, t))
        tangents.append(_cubic_tangent(p0, p1, p2, p3, t))
    return centers, tangents


def _transport_frames(tangents):
    first = tangents[0]
    reference = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(float(Gf.Dot(first, reference))) > 0.90:
        reference = Gf.Vec3d(1.0, 0.0, 0.0)

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


def _radius_profile(base_radius: float, branch_id: str):
    root_scale = _stable_range(branch_id, "root_radius", *ROOT_RADIUS_SCALE_RANGE)
    mid_scale = _stable_range(branch_id, "mid_radius", *MID_RADIUS_SCALE_RANGE)
    tip_scale = _stable_range(branch_id, "tip_radius", *TIP_RADIUS_SCALE_RANGE)

    radii = []
    for index in range(CURVE_SAMPLES):
        t = index / float(CURVE_SAMPLES - 1)
        if t < 0.45:
            q = t / 0.45
            q = q * q * (3.0 - 2.0 * q)
            scale = root_scale + (mid_scale - root_scale) * q
        else:
            q = (t - 0.45) / 0.55
            q = q * q * (3.0 - 2.0 * q)
            scale = mid_scale + (tip_scale - mid_scale) * q
        radii.append(base_radius * scale)
    return radii


def _tube_mesh_data(centers, tangents, radii):
    normals, binormals = _transport_frames(tangents)
    points = []
    for center, normal, binormal, radius in zip(centers, normals, binormals, radii):
        for radial_index in range(RADIAL_SEGMENTS):
            theta = 2.0 * math.pi * radial_index / RADIAL_SEGMENTS
            point = center + radius * (
                normal * math.cos(theta) + binormal * math.sin(theta)
            )
            points.append(Gf.Vec3f(*point))

    counts = []
    indices = []
    for ring in range(len(centers) - 1):
        row0 = ring * RADIAL_SEGMENTS
        row1 = (ring + 1) * RADIAL_SEGMENTS
        for radial_index in range(RADIAL_SEGMENTS):
            nxt = (radial_index + 1) % RADIAL_SEGMENTS
            counts.extend((3, 3))
            indices.extend((
                row0 + radial_index,
                row1 + radial_index,
                row1 + nxt,
                row0 + radial_index,
                row1 + nxt,
                row0 + nxt,
            ))

    # Close both ends. These are visual-only and can overlap the rachis/tomato a
    # little; overlap is preferable to a black gap at the attachment.
    start_center = len(points)
    points.append(Gf.Vec3f(*centers[0]))
    end_center = len(points)
    points.append(Gf.Vec3f(*centers[-1]))
    end_row = (len(centers) - 1) * RADIAL_SEGMENTS
    for radial_index in range(RADIAL_SEGMENTS):
        nxt = (radial_index + 1) % RADIAL_SEGMENTS
        counts.extend((3, 3))
        indices.extend((start_center, nxt, radial_index))
        indices.extend((end_center, end_row + radial_index, end_row + nxt))

    return points, counts, indices


def _replace_pedicel_visuals(stage, branches):
    replaced = 0
    for branch in branches:
        branch_id = branch["id"]
        if "_pedicel_" not in branch_id:
            continue

        link_path = f"/World/Stem/{branch_id}_Link_01"
        cylinder = UsdGeom.Cylinder.Get(stage, f"{link_path}/Cylinder")
        if not cylinder:
            raise RuntimeError(f"Missing physical pedicel cylinder: {link_path}/Cylinder")

        # Keep the production cylinder as mass/collision proxy but stop rendering it.
        cylinder.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

        height = scaled(float(branch["height"]))
        radius = scaled(float(branch["radius"]))
        centers, tangents = _sample_centerline(height, branch_id)
        radii = _radius_profile(radius, branch_id)
        points, counts, indices = _tube_mesh_data(centers, tangents, radii)

        mesh = UsdGeom.Mesh.Define(stage, f"{link_path}/CurvedPedicelVisual")
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
        mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateDisplayColorAttr().Set(
            Vt.Vec3fArray([Gf.Vec3f(*PlantColors.PEDICEL)])
        )
        replaced += 1

    return replaced


# -----------------------------------------------------------------------------
# stage creation
# -----------------------------------------------------------------------------


def _make_config():
    trunk = {
        "id": "trunk",
        "system": "vegetative",
        "parent": None,
        "attach_link": None,
        "n_links": 4,
        "radius": 0.008,
        "height": 0.12,
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "fixed",
    }

    truss = {
        "rachis_length": 0.13,
        "rachis_radius": 0.0018,
        "n_fruits": 7,
        "pedicel_length": 0.018,
        "pedicel_radius": 0.0010,
        "pedicel_angle": 90.0,
        "parent_rank": 2,
        "tilt_deg": 68.0,
        "azimuth_deg": 90.0,
        "tomato_radii": [0.018, 0.020, 0.017, 0.021, 0.019, 0.018, 0.020],
        "maturation": [1.0, 0.85, 0.65, 1.0, 0.45, 0.90, 0.72],
    }

    truss_branches, tomatoes = truss_to_complete_config(
        truss,
        parent_trunk_id="trunk",
        rank=2,
        organ_index=0,
    )

    terminal_bodies = [
        {
            "id": tomato["id"],
            "parent_branch_id": tomato["pedicel_id"],
            "shape": "sphere",
            "radius": tomato["radius"],
            "mass": tomato["mass"],
            "maturation": tomato["maturation"],
            # Detachment is not under test here; keep fruits attached so we can
            # judge pedicel motion/appearance without accidental break events.
            "detachment_enabled": False,
        }
        for tomato in tomatoes
    ]
    return [trunk, *truss_branches], terminal_bodies


def _add_scene_support(stage):
    if not stage.GetPrimAtPath("/World/PhysicsScene"):
        physics_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        physics_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
        physics_scene.CreateGravityMagnitudeAttr().Set(9.81)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr().Set(500.0)
    key = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    key.CreateIntensityAttr().Set(1600.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 20.0))

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr().Set(48.0)
    camera.CreateClippingRangeAttr().Set(Gf.Vec2f(0.01, 100.0))
    view = Gf.Matrix4d(1.0)
    view.SetLookAt(
        Gf.Vec3d(0.72, -0.95, 0.78),
        Gf.Vec3d(0.0, 0.10, 0.48),
        Gf.Vec3d(0.0, 0.0, 1.0),
    )
    UsdGeom.Xformable(camera.GetPrim()).AddTransformOp().Set(view.GetInverse())


def _build(path: Path, *, curved_visuals: bool):
    branches, terminal_bodies = _make_config()
    stage, _ = build_stage(
        str(path),
        branches=branches,
        locked_joints=False,
        skip_limit_check=True,
        terminal_bodies=terminal_bodies,
    )
    _add_scene_support(stage)
    replaced = _replace_pedicel_visuals(stage, branches) if curved_visuals else 0
    stage.GetRootLayer().Save()
    return replaced


def main():
    print("=" * 80)
    print("TEST 6A - ARTICULATED RACHIS + RIGID CURVED PEDICELS")
    print("=" * 80)

    _build(CURRENT_USD, curved_visuals=False)
    replaced = _build(PROPOSED_USD, curved_visuals=True)

    print(f"Current stage : {CURRENT_USD}")
    print(f"Proposed stage: {PROPOSED_USD}")
    print(f"Curved pedicel visuals authored: {replaced}")
    print("Physics topology is unchanged between the two stages.")
    print("Rachis = articulated as production; pedicel = one rigid D6 child.")
    print("The hidden straight cylinder remains the collision/mass proxy.")
    print("The curved render mesh ends at the original physical tomato joint tip.")
    print("Press PLAY to observe motion under gravity.")
    print("=" * 80)

    omni.usd.get_context().open_stage(str(PROPOSED_USD))
    for _ in range(5):
        simulation_app.update()

    try:
        from omni.kit.viewport.utility import get_active_viewport
        viewport = get_active_viewport()
        if viewport is not None:
            viewport.set_active_camera("/World/Camera")
    except Exception as exc:
        print(f"[INFO] Could not set camera automatically: {exc}")

    while simulation_app.is_running():
        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    main()
