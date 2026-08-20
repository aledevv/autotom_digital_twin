"""Second isolated truss test: diagonal pedicel root + near-rachis terminal tangent.

This reuses the physics/render-proxy machinery from run_curved_dynamic_pedicel
but changes the geometry to match the observed tomato-truss silhouette better.

Instead of a shallow S/U-shaped visual around a horizontal physical pedicel,
we use a more realistic chord and endpoint tangents:

    rachis
      |
      |\
      | \
      |  |
      |  O

The lateral pedicel rigid body is still ONE D6 child. Its physical proxy points
from the rachis toward the fruit on a diagonal chord. The render-only cubic tube
starts along that chord and progressively bends so its distal tangent becomes
approximately parallel to the rachis. The tomato still attaches at the exact
physical tip, so the existing FixedJoint topology is unchanged.
"""

from __future__ import annotations

import math

import run_curved_dynamic_pedicel as base


# Relative angle between rachis and the straight root-to-fruit chord. Production
# currently uses 90 deg, which creates the ladder-like horizontal appearance.
# A smaller angle puts the fruit both outward and farther along the rachis.
LATERAL_PEDICEL_CHORD_ANGLE_DEG = 56.0

# Cubic control-arm fractions of the physical chord length. The first arm keeps
# the basal section diagonal; the second gives a visibly straighter terminal
# segment toward the tomato instead of one broad symmetric arc.
ROOT_TANGENT_ARM_FRACTION = 0.38
TIP_TANGENT_ARM_FRACTION = 0.34

# Small deterministic side variation avoids perfectly coplanar pedicels while
# preserving the characteristic diagonal -> terminal-straight silhouette.
SIDE_VARIATION_FRACTION = 0.035

PROPOSED_USD = base.OUTPUT_DIR / "02_elbow_rigid_pedicels.usda"


_original_make_config = base._make_config
_original_sample_centerline = base._sample_centerline


def _make_config_with_diagonal_chords():
    branches, terminal_bodies = _original_make_config()
    for branch in branches:
        branch_id = branch.get("id", "")
        if "_pedicel_lat_" in branch_id:
            branch["tilt"] = LATERAL_PEDICEL_CHORD_ANGLE_DEG
    return branches, terminal_bodies


def _lateral_rot_deg(branch_id: str) -> float:
    if branch_id.endswith("_L"):
        return 90.0
    if branch_id.endswith("_R"):
        return 270.0
    raise ValueError(f"Cannot resolve lateral pedicel side from '{branch_id}'")


def _sample_elbow_centerline(height: float, branch_id: str):
    """Create one smooth elbow with exact physical root/tip positions.

    In the child rigid-body frame, the physical chord is local +Z. The root
    tangent follows +Z. The desired distal tangent is the parent rachis +Z axis
    transformed into the child frame, so after USD transforms it appears nearly
    parallel to the rachis in world space.
    """
    if "_pedicel_lat_" not in branch_id:
        # Keep terminal fruit treatment conservative in this experiment.
        return _original_sample_centerline(height, branch_id)

    tilt = LATERAL_PEDICEL_CHORD_ANGLE_DEG
    rot = _lateral_rot_deg(branch_id)

    rot_z = base.Gf.Rotation(base.Gf.Vec3d(0.0, 0.0, 1.0), rot)
    rot_tilt = base.Gf.Rotation(base.Gf.Vec3d(1.0, 0.0, 0.0), -tilt)
    child_to_parent = rot_tilt * rot_z

    root_tangent = base.Gf.Vec3d(0.0, 0.0, 1.0)
    parent_axis_in_child = base._normalized(
        child_to_parent.GetInverse().TransformDir(base.Gf.Vec3d(0.0, 0.0, 1.0))
    )

    # Tiny out-of-plane component to avoid mechanically identical silhouettes.
    phase = 2.0 * math.pi * base._stable_unit(branch_id, "elbow_side_phase")
    side = base.Gf.Vec3d(math.cos(phase), math.sin(phase), 0.0)
    side_amount = height * SIDE_VARIATION_FRACTION * (
        2.0 * base._stable_unit(branch_id, "elbow_side_amount") - 1.0
    )

    p0 = base.Gf.Vec3d(0.0, 0.0, 0.0)
    p3 = base.Gf.Vec3d(0.0, 0.0, height)
    p1 = p0 + root_tangent * (height * ROOT_TANGENT_ARM_FRACTION) + side * side_amount
    p2 = p3 - parent_axis_in_child * (height * TIP_TANGENT_ARM_FRACTION) + side * (0.35 * side_amount)

    centers = []
    tangents = []
    for index in range(base.CURVE_SAMPLES):
        t = index / float(base.CURVE_SAMPLES - 1)
        centers.append(base._cubic_point(p0, p1, p2, p3, t))
        tangents.append(base._cubic_tangent(p0, p1, p2, p3, t))
    return centers, tangents


def main():
    base._make_config = _make_config_with_diagonal_chords
    base._sample_centerline = _sample_elbow_centerline

    print("=" * 80)
    print("TEST 6A v2 - DIAGONAL -> TERMINAL-STRAIGHT PEDICEL")
    print("=" * 80)
    print("Target side profile:")
    print("      \\")
    print("       |")
    print("       O")
    print(
        f"Lateral physical chord angle from rachis: "
        f"{LATERAL_PEDICEL_CHORD_ANGLE_DEG:.1f} deg"
    )
    print("Pedicel remains one rigid D6 child; only its render mesh is curved.")

    base._build(base.CURRENT_USD, curved_visuals=False)
    replaced = base._build(PROPOSED_USD, curved_visuals=True)

    print(f"Baseline stage: {base.CURRENT_USD}")
    print(f"Elbow stage   : {PROPOSED_USD}")
    print(f"Curved visuals: {replaced}")
    print("Press PLAY: rachis and pedicel D6 physics are still active.")
    print("=" * 80)

    base.omni.usd.get_context().open_stage(str(PROPOSED_USD))
    for _ in range(5):
        base.simulation_app.update()

    try:
        from omni.kit.viewport.utility import get_active_viewport

        viewport = get_active_viewport()
        if viewport is not None:
            viewport.set_active_camera("/World/Camera")
    except Exception as exc:
        print(f"[INFO] Could not set camera automatically: {exc}")

    while base.simulation_app.is_running():
        base.simulation_app.update()

    base.simulation_app.close()


if __name__ == "__main__":
    main()
