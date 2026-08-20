"""Third isolated truss test: gravity-oriented pedicel elbow + hanging tomato.

Target side profile:

    rachis
      |
      |\
      | \
      |  |
      |  |
      |  O

The rachis keeps the production articulated physics. Each lateral pedicel is
still ONE rigid body attached by the production D6 joint. Its straight cylinder
remains hidden as mass/collision proxy, but the render mesh follows a cubic path
whose terminal tangent is explicitly aligned with WORLD GRAVITY (-Z).

Unlike the previous test, the tomato center is also moved below the visual tip
along that same gravity direction. The existing FixedJoint is preserved but its
local anchor on the tomato is moved to the corresponding off-axis point. This
lets the visual pedicel turn downward and actually connect to the top of the
fruit while keeping a single rigid pedicel and the same D6 root topology.
"""

from __future__ import annotations

import math

# IMPORTANT: import the base test first. It creates SimulationApp before importing
# pxr/omni modules, which is required when running through Isaac Sim's Python.
import run_curved_dynamic_pedicel as base

from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


# IMPORTANT: tilt is measured away from the parent rachis +Z axis.
# Values below 90 deg still have a POSITIVE component along the rachis, which is
# why the previous 56 deg version sent the fruit upward. 124 deg is the mirrored
# direction: same lateral opening (~56 deg from the downward rachis direction),
# but with a negative axial component so the physical tip is already below the
# attachment point before the visual curve turns fully toward gravity.
LATERAL_PEDICEL_CHORD_ANGLE_DEG = 56.0

# Cubic control-arm fractions of physical pedicel length. A fairly long terminal
# arm makes the last part read as a clear downward segment rather than a tiny
# tangent change hidden by the tomato.
ROOT_TANGENT_ARM_FRACTION = 0.34
TIP_TANGENT_ARM_FRACTION = 0.42
SIDE_VARIATION_FRACTION = 0.025

PROPOSED_USD = base.OUTPUT_DIR / "03_gravity_elbow_pedicels.usda"


_original_make_config = base._make_config


def _make_config_with_diagonal_chords():
    branches, terminal_bodies = _original_make_config()
    for branch in branches:
        branch_id = branch.get("id", "")
        if "_pedicel_lat_" in branch_id:
            branch["tilt"] = LATERAL_PEDICEL_CHORD_ANGLE_DEG
    return branches, terminal_bodies


def _stable_side_offset(height: float, branch_id: str) -> Gf.Vec3d:
    phase = 2.0 * math.pi * base._stable_unit(branch_id, "gravity_elbow_phase")
    direction = Gf.Vec3d(math.cos(phase), math.sin(phase), 0.0)
    amount = height * SIDE_VARIATION_FRACTION * (
        2.0 * base._stable_unit(branch_id, "gravity_elbow_side") - 1.0
    )
    return direction * amount


def _sample_gravity_elbow(
    height: float,
    branch_id: str,
    gravity_local: Gf.Vec3d,
):
    """Sample a root-diagonal -> world-down terminal cubic in pedicel local space."""
    gravity_local = base._normalized(gravity_local)
    root_tangent = Gf.Vec3d(0.0, 0.0, 1.0)
    side_offset = _stable_side_offset(height, branch_id)

    p0 = Gf.Vec3d(0.0, 0.0, 0.0)
    p3 = Gf.Vec3d(0.0, 0.0, height)

    # Cubic derivative at t=0 is parallel to p1-p0, therefore the pedicel starts
    # along the actual rigid chord. Derivative at t=1 is parallel to p3-p2, so
    # choosing p2 behind the endpoint along gravity_local guarantees that the
    # visible tip is vertical/downward in WORLD coordinates.
    p1 = p0 + root_tangent * (height * ROOT_TANGENT_ARM_FRACTION) + side_offset
    p2 = (
        p3
        - gravity_local * (height * TIP_TANGENT_ARM_FRACTION)
        + side_offset * 0.25
    )

    centers = []
    tangents = []
    for index in range(base.CURVE_SAMPLES):
        t = index / float(base.CURVE_SAMPLES - 1)
        centers.append(base._cubic_point(p0, p1, p2, p3, t))
        tangents.append(base._cubic_tangent(p0, p1, p2, p3, t))
    return centers, tangents


def _set_xform_translate(prim, value: Gf.Vec3d) -> None:
    xformable = UsdGeom.Xformable(prim)
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(value)
            return
    xformable.AddTranslateOp().Set(value)


def _author_gravity_pedicels_and_hang_tomatoes(stage, branches, terminal_bodies):
    """Replace lateral pedicel visuals and move each tomato below the visual tip."""
    body_by_parent = {
        body["parent_branch_id"]: body
        for body in terminal_bodies
        if body.get("parent_branch_id")
    }

    replaced = 0
    hung = 0

    for branch in branches:
        branch_id = branch.get("id", "")
        if "pedicel" not in branch_id:
            continue

        link_path = f"/World/Stem/{branch_id}_Link_01"
        link_prim = stage.GetPrimAtPath(link_path)
        if not link_prim:
            raise RuntimeError(f"Missing pedicel rigid body: {link_path}")

        cylinder = UsdGeom.Cylinder.Get(stage, f"{link_path}/Cylinder")
        if not cylinder:
            raise RuntimeError(f"Missing physical proxy: {link_path}/Cylinder")
        cylinder.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

        pedicel_filtered = UsdPhysics.FilteredPairsAPI(cylinder.GetPrim())
        if not pedicel_filtered:
            pedicel_filtered = UsdPhysics.FilteredPairsAPI.Apply(cylinder.GetPrim())
        pedicel_filtered.GetFilteredPairsRel().AddTarget("/World/Stem")

        link_to_world = UsdGeom.Xformable(link_prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        world_to_link = link_to_world.GetInverse()

        # Terminal direction is the actual world gravity vector expressed in the
        # pedicel local frame.
        gravity_local = base._normalized(
            world_to_link.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0))
        )

        height = base.scaled(float(branch["height"]))
        radius = base.scaled(float(branch["radius"]))
        centers, tangents = _sample_gravity_elbow(height, branch_id, gravity_local)
        radii = base._radius_profile(radius, branch_id)
        points, counts, indices = base._tube_mesh_data(centers, tangents, radii)

        mesh = UsdGeom.Mesh.Define(stage, f"{link_path}/GravityElbowPedicelVisual")
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
        mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(indices))
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateDisplayColorAttr().Set(
            Vt.Vec3fArray([Gf.Vec3f(*base.PlantColors.PEDICEL)])
        )
        replaced += 1

        body = body_by_parent.get(branch_id)
        if body is None:
            continue

        tomato_path = f"/World/Stem/{body['id']}"
        tomato_prim = stage.GetPrimAtPath(tomato_path)
        if not tomato_prim:
            raise RuntimeError(f"Missing tomato body: {tomato_path}")

        tomato_radius = base.scaled(float(body["radius"]))
        tip_local = centers[-1]
        terminal_down_local = base._normalized(tangents[-1])

        # The sphere center is placed slightly less than one radius below the pedicel
        # tip to create a small overlap (2mm). This swallows the flat end of the 
        # pedicel tube and eliminates the visual gap.
        visual_overlap = 0.002
        tomato_center_local = tip_local + terminal_down_local * (tomato_radius - visual_overlap)
        tomato_center_world = link_to_world.Transform(tomato_center_local)
        _set_xform_translate(tomato_prim, Gf.Vec3d(tomato_center_world))

        joint_path = f"{tomato_path}/TerminalBodyFixedJoint"
        joint = UsdPhysics.FixedJoint.Get(stage, joint_path)
        if not joint:
            raise RuntimeError(f"Missing tomato FixedJoint: {joint_path}")

        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*tip_local))
        joint.CreateLocalPos1Attr().Set(
            Gf.Vec3f(*(-terminal_down_local * (tomato_radius - visual_overlap)))
        )
        
        # Explicitly filter collisions between the tomato and the rest of the stem
        # to prevent collision explosions that bend the branch up.
        tomato_filtered = UsdPhysics.FilteredPairsAPI(tomato_prim)
        if not tomato_filtered:
            tomato_filtered = UsdPhysics.FilteredPairsAPI.Apply(tomato_prim)
        tomato_filtered.GetFilteredPairsRel().AddTarget("/World/Stem")

        hung += 1

    return replaced, hung


def _build_gravity_stage(path):
    branches, terminal_bodies = _make_config_with_diagonal_chords()
    stage, _ = base.build_stage(
        str(path),
        branches=branches,
        locked_joints=False,
        skip_limit_check=True,
        terminal_bodies=terminal_bodies,
    )
    base._add_scene_support(stage)
    
    replaced, hung = _author_gravity_pedicels_and_hang_tomatoes(
        stage,
        branches,
        terminal_bodies,
    )
    stage.GetRootLayer().Save()
    return replaced, hung


def main():
    print("=" * 80)
    print("TEST 6A v4 - DOWNWARD CHORD + GRAVITY ELBOW")
    print("=" * 80)
    print("Target side profile:")
    print("      \\")
    print("       \\")
    print("        |")
    print("        |")
    print("        O")
    print(
        f"Pedicel chord tilt from rachis +Z: "
        f"{LATERAL_PEDICEL_CHORD_ANGLE_DEG:.1f} deg (>90 => downward)"
    )
    print("Terminal tangent is forced to WORLD -Z (gravity).")
    print("Tomato center is moved one radius below the visual tip.")
    print("Pedicel remains ONE rigid D6 child; rachis physics is unchanged.")

    base._build(base.CURRENT_USD, curved_visuals=False)
    replaced, hung = _build_gravity_stage(PROPOSED_USD)

    print(f"Baseline stage : {base.CURRENT_USD}")
    print(f"Gravity stage  : {PROPOSED_USD}")
    print(f"Curved visuals : {replaced}")
    print(f"Hanging tomatoes: {hung}")
    print("Press PLAY to validate D6 motion with the off-axis fruit attachment.")
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
