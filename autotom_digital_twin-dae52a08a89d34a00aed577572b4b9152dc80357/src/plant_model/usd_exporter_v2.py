"""
usd_exporter_v2.py

Builds a physically simulated stem with rigid segments and elastic D6 joints.
Includes leaves attached to the stem segments.
"""

import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from .models import PlantSnapshot, InternodeNode, LeafNode

from .constants import (
    STEM_DENSITY_KG_M3,
    SEGMENT_TARGET_LENGTH_M,
    MIN_SEGMENTS_PER_INTERNODE,
    SEGMENT_GAP_M,
    STEM_JOINT_STIFFNESS_BASE,
    STEM_JOINT_STIFFNESS_TIP,
    STEM_JOINT_DAMPING,
    STEM_JOINT_BEND_LIMIT_DEG,
    GLOBAL_SCALE,
    MAX_TOTAL_SEGMENTS,
    LEAF_MASS_KG,
    LEAF_JOINT_STIFFNESS,
    LEAF_JOINT_DAMPING,
    LEAF_CONE_ANGLE_DEG,
)

from .usd_helpers import _make_material, _bind_material, _make_leaf

PLANT_ROOT_PATH_TEMPLATE = "/Plant_{plant_id}_StemV2"
_SCALE5 = GLOBAL_SCALE ** 5


# ==============================================================================
# HELPERS
# ==============================================================================

def _compute_world_base_z(node: InternodeNode) -> float:
    """Computes and caches the world Z coordinate of the node's base."""
    if hasattr(node, 'world_base_z'):
        return node.world_base_z
    if node.parent is None or not isinstance(node.parent, InternodeNode):
        node.world_base_z = 0.0
    else:
        node.world_base_z = _compute_world_base_z(node.parent) + node.parent.length
    return node.world_base_z


def _segments_for_internode(length_m: float, target_length_m: float) -> int:
    """Calculates how many rigid segments to divide an internode into."""
    if length_m <= 0:
        return max(1, MIN_SEGMENTS_PER_INTERNODE)
    n = round(length_m / target_length_m)
    return max(MIN_SEGMENTS_PER_INTERNODE, n, 1)


def _find_parent_segment(parent_segments: list[dict], target_world_z: float) -> dict:
    """Finds the segment whose [base_z, base_z+height] contains target_world_z."""
    for seg in parent_segments:
        if seg['base_z'] <= target_world_z <= seg['base_z'] + seg['height'] + 1e-9:
            return seg
    return min(parent_segments, key=lambda s: abs(s['base_z'] - target_world_z))


# ==============================================================================
# RIGID BODIES & JOINTS
# ==============================================================================

def _create_segment_link(stage: Usd.Stage, chain_path: str, seg_index: int,
                          radius: float, height: float, base_z: float, material) -> str:
    """Creates a single rigid body link for the stem."""
    link_path = f"{chain_path}/Seg{seg_index:04d}"

    xform_prim = UsdGeom.Xform.Define(stage, link_path)
    xform_prim.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, base_z))

    UsdPhysics.RigidBodyAPI.Apply(xform_prim.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform_prim.GetPrim())
    real_radius = radius / GLOBAL_SCALE
    real_height = height / GLOBAL_SCALE
    mass_kg = math.pi * (real_radius ** 2) * real_height * STEM_DENSITY_KG_M3
    mass_api.CreateMassAttr().Set(mass_kg)

    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(radius)
    cylinder.GetHeightAttr().Set(height)
    cylinder.GetAxisAttr().Set(UsdGeom.Tokens.z)
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())

    if material is not None:
        _bind_material(cylinder, material)

    return link_path


def _anchor_link_to_world(stage: Usd.Stage, link_path: str) -> None:
    """Anchors the first stem segment to the world."""
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def _configure_bend_drive(joint: UsdPhysics.Joint, stiffness: float, damping: float,
                           bend_limit_deg: float) -> None:
    """Locks translations and twist, applies spring to swing (rotX/rotY)."""
    for axis in ("transX", "transY", "transZ"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)

    for axis in ("rotX", "rotY"):
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(-bend_limit_deg)
        limit.CreateHighAttr().Set(bend_limit_deg)

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(damping)
        drive.CreateTargetPositionAttr().Set(0.0)

    limit_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    limit_z.CreateLowAttr().Set(1.0)
    limit_z.CreateHighAttr().Set(-1.0)


def _create_bend_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str,
                        parent_local_z: float, stiffness: float, damping: float,
                        bend_limit_deg: float) -> None:
    """Creates a D6 Joint between two stem segments."""
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_local_z))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    _configure_bend_drive(joint, stiffness, damping, bend_limit_deg)


def _create_leaf_joint(stage: Usd.Stage, joint_path: str,
                        parent_link: str, rametto_path: str,
                        parent_local_z: float, tip_world_z: float,
                        stiffness: float, damping: float, cone_angle_deg: float) -> None:
    """Creates a spherical joint allowing the leaf to swing freely within a cone."""
    joint = UsdPhysics.SphericalJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(rametto_path)])

    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, parent_local_z))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, tip_world_z))

    joint.GetPrim().CreateAttribute("physics:coneAngle0Limit", Sdf.ValueTypeNames.Float).Set(cone_angle_deg)
    joint.GetPrim().CreateAttribute("physics:coneAngle1Limit", Sdf.ValueTypeNames.Float).Set(cone_angle_deg)

    for axis in ("rotX", "rotY", "rotZ"):
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(stiffness)
        drive.CreateDampingAttr().Set(damping)
        drive.CreateTargetPositionAttr().Set(0.0)


# ==============================================================================
# MAIN BUILDERS
# ==============================================================================

def _build_stem_chain(stage: Usd.Stage, chain_path: str,
                       internodes: list[InternodeNode],
                       target_segment_length: float,
                       material) -> list[dict]:
    """Builds the main stem chain, returning a list of its segments."""
    segments: list[dict] = []
    seg_index = 0
    n_internodes = len(internodes)
    previous_link_path = None
    previous_link_height = None
    current_base_z = internodes[0].world_base_z if internodes else 0.0

    for i, node in enumerate(internodes):
        length = node.length * GLOBAL_SCALE
        radius = (node.width_m / 2.0) * GLOBAL_SCALE
        n_segments = _segments_for_internode(node.length, target_segment_length)

        n_gaps = max(n_segments - 1, 0)
        gap_scaled = SEGMENT_GAP_M * GLOBAL_SCALE
        seg_height = max((length - n_gaps * gap_scaled) / n_segments, 1e-5)

        t = i / max(n_internodes - 1, 1)
        stiffness = (STEM_JOINT_STIFFNESS_BASE + t * (STEM_JOINT_STIFFNESS_TIP - STEM_JOINT_STIFFNESS_BASE)) * _SCALE5
        damping = STEM_JOINT_DAMPING * _SCALE5

        for _ in range(n_segments):
            link_path = _create_segment_link(
                stage, chain_path, seg_index, radius, seg_height, current_base_z, material
            )

            if previous_link_path is None:
                _anchor_link_to_world(stage, link_path)
            else:
                joint_name = f"Joint_{seg_index-1:04d}_{seg_index:04d}"
                _create_bend_joint(
                    stage, previous_link_path, link_path, joint_name,
                    parent_local_z=previous_link_height + gap_scaled,
                    stiffness=stiffness, damping=damping,
                    bend_limit_deg=STEM_JOINT_BEND_LIMIT_DEG,
                )

            segments.append({'path': link_path, 'base_z': current_base_z, 'height': seg_height})

            previous_link_path = link_path
            previous_link_height = seg_height
            current_base_z += seg_height + gap_scaled
            seg_index += 1

    return segments


def _attach_leaf(stage: Usd.Stage, leaves_path: str, joints_path: str,
                  leaf_node: LeafNode, stem_segments: list[dict], materials: dict):
    """Attaches a leaf to the appropriate stem segment."""
    if leaf_node.parent is None or not isinstance(leaf_node.parent, InternodeNode):
        return

    tip_world_z = leaf_node.parent.world_base_z + leaf_node.parent.length * GLOBAL_SCALE
    parent_seg = _find_parent_segment(stem_segments, tip_world_z)
    parent_local_z = tip_world_z - parent_seg['base_z']

    leaf_id = f"o{leaf_node.key.order}_r{leaf_node.key.rank}_i{leaf_node.key.organ_index}"
    leaf_group = f"{leaves_path}/Leaf_{leaf_id}"
    UsdGeom.Xform.Define(stage, leaf_group)

    rametto_path = f"{leaf_group}/Rametto"
    rametto_xform = UsdGeom.Xform.Define(stage, rametto_path)

    rametto_xform.AddScaleOp().Set(Gf.Vec3f(GLOBAL_SCALE, GLOBAL_SCALE, GLOBAL_SCALE))
    rametto_xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, tip_world_z))

    _make_leaf(stage, rametto_path, leaf_node, 0.0, materials)

    UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(rametto_path))
    mass_api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(rametto_path))
    mass_api.CreateMassAttr().Set(LEAF_MASS_KG)

    filtered_pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(rametto_path))
    filtered_pairs.GetFilteredPairsRel().AddTarget(Sdf.Path(parent_seg['path']))

    _create_leaf_joint(
        stage, joint_path=f"{joints_path}/Joint_Leaf_{leaf_id}",
        parent_link=parent_seg['path'], rametto_path=rametto_path,
        parent_local_z=parent_local_z, tip_world_z=0.0,
        stiffness=LEAF_JOINT_STIFFNESS, damping=LEAF_JOINT_DAMPING,
        cone_angle_deg=LEAF_CONE_ANGLE_DEG,
    )


def build_stem_stage(snapshot: PlantSnapshot, output_path: str) -> tuple:
    """
    Builds a USD Stage with the main stem and its attached leaves.
    Returns (stage, stem_path). PhysicsScene is added later in Isaac Sim.
    """
    stage = Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    plant_path = PLANT_ROOT_PATH_TEMPLATE.format(plant_id=snapshot.plant_id)
    plant_prim = UsdGeom.Xform.Define(stage, plant_path).GetPrim()
    stage.SetDefaultPrim(plant_prim)

    stem_path = f"{plant_path}/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())

    mats_path = f"{plant_path}/Materials"
    UsdGeom.Xform.Define(stage, mats_path)
    mat_stem = _make_material(stage, f"{mats_path}/Stem", (0.45, 0.30, 0.10))
    mat_leaf = _make_material(stage, f"{mats_path}/Leaf", (0.15, 0.55, 0.10))
    mat_pedicel = _make_material(stage, f"{mats_path}/Pedicel", (0.20, 0.50, 0.10))
    materials = {"leaf": mat_leaf, "pedicel": mat_pedicel}

    all_internodes = [n for n in snapshot.organs if isinstance(n, InternodeNode)]
    if not all_internodes:
        print("[WARN] No internodes found. Stage is empty.")
        return stage, stem_path

    # Only process main stem (minimum order)
    min_order = min(n.key.order for n in all_internodes)
    internodes = [n for n in all_internodes if n.key.order == min_order]
    internodes.sort(key=lambda n: n.key.rank)

    # Compute absolute Z heights
    for n in internodes:
        _compute_world_base_z(n)

    total_wood_length = sum(n.length for n in internodes)
    target_len = max(total_wood_length / MAX_TOTAL_SEGMENTS, SEGMENT_TARGET_LENGTH_M)

    # Build the main stem
    stem_segments = _build_stem_chain(
        stage, stem_path, internodes, target_len, mat_stem
    )

    # Attach leaves
    leaves_path = f"{plant_path}/Leaves"
    UsdGeom.Xform.Define(stage, leaves_path)
    joints_path = f"{plant_path}/Joints"
    UsdGeom.Xform.Define(stage, joints_path)

    leaves = [n for n in snapshot.organs if isinstance(n, LeafNode) and n.parent and n.parent.key.order == min_order]
    
    for leaf_node in leaves:
        _attach_leaf(stage, leaves_path, joints_path, leaf_node, stem_segments, materials)

    print(f"[INFO] Created {len(stem_segments)} stem segments and {len(leaves)} leaves.")

    return stage, stem_path


def export_stem_usd_v2(snapshot: PlantSnapshot, output_path: str) -> None:
    """Wrapper to build the stage and save it to disk."""
    stage, _ = build_stem_stage(snapshot, output_path)
    stage.GetRootLayer().Save()
    print(f"[USD] Saved stem V2 -> {output_path}")