"""PlantState checkpoints routed through the established ExporterV2 backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from types import SimpleNamespace
from typing import Any

from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

from plant_state import PlantState

from .core.physics import (
    apply_physx_articulation_settings,
    apply_physx_joint_armature,
    apply_physx_rigid_body_solver_settings,
    apply_physx_scene_settings,
)
from .core.skinning import (
    VisualAxisData,
    VisualProfile,
    VisualSegment,
    build_visual_axes,
    resolve_vegetative_graph,
)
from .core.skinning.adapter import _quat_from_column_rotation
from .core.skinning.leaf_blade import (
    LEAF_ARCH_LIFT_FRACTION,
    LEAF_HALF_WIDTH_FRACTION,
    LEAF_LENGTH_FRACTION,
    LEAF_LONGITUDINAL_FOLD_FRACTION,
    LEAF_TIP_SAG_FRACTION,
    author_leaf_blade,
)
from .core.skinning.visual_segmented import author_segmented_visual_axis
from .core.tree_config import (
    GLOBAL_SCALE,
    PhysicsRuntimeConfig,
    PlantColors,
    TrussPhysicsConfig,
    compute_moment_of_inertia,
)
from .core.usd import build_stage
from .core.usd.collision import add_collision_filter
from .core.usd.materials import get_or_create_tomato_truss_material
from .core.usd.pedicel_geometry import (
    create_gravity_elbow_mesh,
    sample_gravity_elbow,
)
from .plant_state_branches import (
    LATERAL_JOINT_POLICIES,
    LEAF_JOINT_POLICIES,
    TRUSS_CALIBRATION_PRESETS,
    TRUSS_DAMPING_CHOICES,
    VISUAL_QUALITY_MODES,
    StemBranchesResult,
    apply_checkpoint_physics_policy,
    build_leaf_branches,
    build_leaf_support_branches,
    build_lateral_branches,
    build_stem_branches,
    build_truss_branches,
)


STEM_CHECKPOINT_SCHEMA = "exporter_v2_stem_checkpoint/1.0"
LATERALS_CHECKPOINT_SCHEMA = "exporter_v2_laterals_checkpoint/1.0"
LEAF_SUPPORTS_CHECKPOINT_SCHEMA = "exporter_v2_leaf_supports_checkpoint/1.0"
LEAVES_CHECKPOINT_SCHEMA = "exporter_v2_leaves_checkpoint/1.0"
TRUSS_SUPPORTS_CHECKPOINT_SCHEMA = "exporter_v2_truss_supports_checkpoint/1.0"
FRUIT_VISUAL_CHECKPOINT_SCHEMA = "exporter_v2_fruit_visual_checkpoint/1.0"
FULL_CHECKPOINT_SCHEMA = "exporter_v2_full_checkpoint/1.0"
INCREMENTAL_PROFILES = (
    "stem",
    "laterals",
    "leaf-supports",
    "leaves",
    "truss-supports",
    "fruit-visual",
    "full",
)
INITIAL_OVERLAP_POLICIES = ("filter", "error")
TRUSS_ARMATURE_MULTIPLIERS = (0.0, 1.0, 4.0)
TERMINAL_SOLVER_PRESETS = ("current", "stabilized")


class IncrementalCheckpointError(ValueError):
    """Raised when a conservative V2 checkpoint fails its audit."""


# Compatibility names retained for callers of the completed stem checkpoint.
StemCheckpointError = IncrementalCheckpointError


@dataclass(frozen=True)
class IncrementalCheckpointManifest:
    metadata: dict[str, Any]
    expected: dict[str, int]
    authored: dict[str, Any]
    topology: dict[str, Any]
    physics: dict[str, Any]
    collisions: dict[str, Any]
    errors: tuple[str, ...]
    schema_version: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


StemCheckpointManifest = IncrementalCheckpointManifest


@dataclass(frozen=True)
class IncrementalCheckpointPlan:
    adapter: StemBranchesResult
    physical_link_count: int
    predicted_d6_joints: int
    debug_profile: str


StemCheckpointPlan = IncrementalCheckpointPlan


def manifest_path_for(path: Path) -> Path:
    return path.with_suffix(".manifest.json")


def _custom(prim, name: str, value_type, value) -> None:
    prim.CreateAttribute(name, value_type, custom=True).Set(value)


def _author_stage_metadata(
    stage,
    state: PlantState,
    adapter: StemBranchesResult,
    physics_preset: str,
    *,
    truss_armature_multiplier: float = 0.0,
    terminal_solver_preset: str = "current",
    allow_experimental_fruit_physics: bool = False,
) -> None:
    stem = stage.GetPrimAtPath("/World/Stem")
    _custom(stem, "autotom:plantStateSchema", Sdf.ValueTypeNames.String, state.schema_version)
    _custom(stem, "autotom:debugProfile", Sdf.ValueTypeNames.String, adapter.debug_profile)
    _custom(stem, "autotom:poseMode", Sdf.ValueTypeNames.String, adapter.pose_mode)
    _custom(
        stem,
        "autotom:appendagePoseMode",
        Sdf.ValueTypeNames.String,
        adapter.appendage_pose_mode,
    )
    _custom(stem, "autotom:physicsPreset", Sdf.ValueTypeNames.String, physics_preset)
    _custom(
        stem,
        "autotom:trussArmatureMultiplier",
        Sdf.ValueTypeNames.Double,
        truss_armature_multiplier,
    )
    _custom(
        stem,
        "autotom:terminalSolverPreset",
        Sdf.ValueTypeNames.String,
        terminal_solver_preset,
    )
    _custom(
        stem,
        "autotom:lateralJointPolicy",
        Sdf.ValueTypeNames.String,
        adapter.lateral_joint_policy,
    )
    _custom(
        stem,
        "autotom:trussCalibrationPreset",
        Sdf.ValueTypeNames.String,
        adapter.truss_calibration_preset,
    )
    _custom(
        stem,
        "autotom:visualQuality",
        Sdf.ValueTypeNames.String,
        adapter.visual_quality,
    )
    if adapter.leaf_joint_policy is not None:
        _custom(
            stem,
            "autotom:leafJointPolicy",
            Sdf.ValueTypeNames.String,
            adapter.leaf_joint_policy,
        )
    _custom(
        stem,
        "autotom:physicalPetiolules",
        Sdf.ValueTypeNames.Bool,
        adapter.physical_petiolules,
    )
    _custom(stem, "autotom:collidersEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:drivesEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(stem, "autotom:articulationEnabled", Sdf.ValueTypeNames.Bool, True)
    _custom(
        stem,
        "autotom:terminalBodiesPhysical",
        Sdf.ValueTypeNames.Bool,
        any(body.get("physical", True) for body in adapter.terminal_bodies),
    )
    _custom(
        stem,
        "autotom:experimentalFruitPhysics",
        Sdf.ValueTypeNames.Bool,
        adapter.debug_profile == "full" and allow_experimental_fruit_physics,
    )
    _custom(
        stem,
        "autotom:fruitPhysicsSupportStatus",
        Sdf.ValueTypeNames.String,
        (
            "unsupported_experimental"
            if adapter.debug_profile == "full"
            else "not_authored"
        ),
    )
    root = next(node for node in state.nodes if node.id == state.root_node_id)
    _custom(
        stem,
        "autotom:groimpOrigin",
        Sdf.ValueTypeNames.Double3,
        Gf.Vec3d(*root.pose.world_start),
    )

    for branch in adapter.branches:
        prim = stage.GetPrimAtPath(f"/World/Stem/Vegetative/{branch['id']}")
        if not prim or not prim.IsValid():
            continue
        _custom(prim, "autotom:branchId", Sdf.ValueTypeNames.String, branch["id"])
        _custom(prim, "autotom:branchKind", Sdf.ValueTypeNames.String, branch["kind"])
        _custom(prim, "autotom:jointType", Sdf.ValueTypeNames.String, branch["joint_type"])
        if branch.get("source_parent_node_id") is not None:
            _custom(
                prim,
                "autotom:sourceParentNodeId",
                Sdf.ValueTypeNames.String,
                branch["source_parent_node_id"],
            )

    for record in adapter.degenerate_organs:
        prim = UsdGeom.Xform.Define(
            stage,
            f"/World/Stem/Vegetative/{record['xform_name']}",
        ).GetPrim()
        _custom(prim, "autotom:entityKind", Sdf.ValueTypeNames.String, "degenerate_organ")
        _custom(prim, "autotom:organType", Sdf.ValueTypeNames.String, "Leaf")
        _custom(prim, "autotom:canonicalOrganId", Sdf.ValueTypeNames.String, record["organ_id"])
        _custom(prim, "autotom:canonicalNodeId", Sdf.ValueTypeNames.String, record["node_id"])
        _custom(prim, "autotom:groimpNodeId", Sdf.ValueTypeNames.Int64, record["groimp_node_id"])
        _custom(prim, "autotom:diagnostic", Sdf.ValueTypeNames.String, record["reason"])


def _author_rigid_leaf_visuals(stage, adapter: StemBranchesResult) -> list[dict[str, Any]]:
    """Bind canonical petiolules and V2 blades directly to support bodies."""

    if not adapter.rigid_leaf_visuals:
        return []
    body_by_axis = {}
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() != "physical_link":
            continue
        axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if axis_id is not None:
            body_by_axis[str(axis_id)] = str(prim.GetPath())

    authored = []
    for record in adapter.rigid_leaf_visuals:
        host_path = body_by_axis.get(record["host_axis_id"])
        if host_path is None:
            raise IncrementalCheckpointError(
                f"leaf visual {record['axis_id']} cannot resolve host "
                f"{record['host_axis_id']}"
            )
        host_matrix = UsdGeom.Xformable(
            stage.GetPrimAtPath(host_path)
        ).ComputeLocalToWorldTransform(0)
        world_to_host = host_matrix.GetInverse()
        frame = record["rest_frame"]
        start = Gf.Vec3d(
            *(float(frame[row][3]) * GLOBAL_SCALE for row in range(3))
        )
        head = Gf.Vec3d(*(float(frame[row][2]) for row in range(3))).GetNormalized()
        length = float(record["length"]) * GLOBAL_SCALE
        radius = float(record["radius"]) * GLOBAL_SCALE

        if record.get("physical", False):
            blade_path = f"{host_path}/LeafBlade"
            if not stage.GetPrimAtPath(blade_path):
                raise IncrementalCheckpointError(
                    f"physical petiolule {record['axis_id']} has no V2 leaf blade"
                )
            authored.append(
                {
                    **record,
                    "host_body_path": host_path,
                    "root_path": host_path,
                    "blade_length": min(
                        0.09, max(0.04, length * LEAF_LENGTH_FRACTION)
                    ),
                    "renderer": "v2_segmented_physical_petiolule",
                    "visual_stats": {"physical_support": True, "meshes": 1},
                }
            )
            continue

        root_path = f"{host_path}/RigidLeafVisuals/{record['id']}"
        root_xform = UsdGeom.Xform.Define(stage, root_path)
        root = root_xform.GetPrim()
        orientation = _quat_from_column_rotation(frame)
        desired_world = Gf.Matrix4d(1.0)
        desired_world.SetTransform(
            Gf.Rotation(Gf.Quatd(orientation)), start
        )
        root_xform.AddTransformOp().Set(desired_world * world_to_host)
        actual_world = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0)
        frame_error = max(
            abs(float(actual_world[row][column] - desired_world[row][column]))
            for row in range(4)
            for column in range(4)
        )
        if frame_error > 1e-8:
            raise IncrementalCheckpointError(
                f"leaf visual {record['axis_id']} could not author its canonical "
                f"frame (matrix error {frame_error:.3g})"
            )
        _custom(root, "autotom:entityKind", Sdf.ValueTypeNames.String, "rigid_leaf_visual")
        _custom(root, "autotom:attachmentMode", Sdf.ValueTypeNames.String, "rigid_visual")
        _custom(root, "autotom:canonicalOrganId", Sdf.ValueTypeNames.String, record["organ_id"])
        _custom(root, "autotom:canonicalPrimitiveId", Sdf.ValueTypeNames.String, record["axis_id"])
        _custom(root, "autotom:groimpNodeId", Sdf.ValueTypeNames.Int64, record["groimp_node_id"])
        _custom(root, "autotom:role", Sdf.ValueTypeNames.String, record["role"])
        _custom(root, "autotom:hostPrimitiveId", Sdf.ValueTypeNames.String, record["host_axis_id"])

        profile_raw = adapter.branches[0]["visual_profile"]
        profile = VisualProfile(
            radial_segments=int(profile_raw["radial_segments"]),
            axial_spacing_m=float(profile_raw["axial_spacing_m"]),
            radius_transition_samples=int(
                profile_raw["radius_transition_samples"]
            ),
        )
        member = SimpleNamespace(
            definition={"id": record["id"], "kind": "petiolule"},
            spec=SimpleNamespace(visual=profile),
            centered_terminal=False,
            centered_terminal_host=False,
        )
        visual_axis = VisualAxisData(
            axis_id=record["axis_id"],
            members=[member],
            member_offsets={record["id"]: 0.0},
            member_lengths={record["id"]: length},
            visual_segments=[
                VisualSegment(
                    source_id=record["axis_id"],
                    start_arc=0.0,
                    length=length,
                    radius=radius,
                    end_radius=radius * 0.65,
                )
            ],
            link_paths=[root_path],
            link_bases=[start],
            link_orientations=[orientation],
            bone_starts=[0.0],
            bone_lengths=[length],
            start=start,
            axis=head,
            orientation=orientation,
            total_length=length,
            visual_root_path=root_path,
            skel_root_path=f"{root_path}/UnusedSkelRoot",
            skeleton_path=f"{root_path}/UnusedSkeleton",
            animation_path=f"{root_path}/UnusedAnimation",
            mesh_path=f"{root_path}/UnusedMesh",
            parent_radius=float(
                stage.GetPrimAtPath(host_path)
                .GetAttribute("autotom:visualRadius")
                .Get()
            ),
            attachment_arcs=[],
        )
        if record.get("physical", False):
            # The historical segmented mesh is already authored by the
            # physical petiolule branch.  Only the blade belongs here.
            visual_stats = {"physical_support": True, "meshes": 0}
        else:
            visual_stats = author_segmented_visual_axis(stage, visual_axis)

        blade_length = min(0.09, max(0.04, length * LEAF_LENGTH_FRACTION))
        author_leaf_blade(
            stage,
            f"{root_path}/LeafBlade",
            start + head * length,
            head,
            length=blade_length,
            half_width=blade_length * LEAF_HALF_WIDTH_FRACTION,
            fold_depth=blade_length * LEAF_LONGITUDINAL_FOLD_FRACTION,
            arch_lift=blade_length * LEAF_ARCH_LIFT_FRACTION,
            tip_sag=blade_length * LEAF_TIP_SAG_FRACTION,
            color=PlantColors.LEAF_BLADE,
            # LeafBlade is a child of the canonical petiolule Xform, not a
            # direct child of the physical rachis body.  Its vertices must
            # therefore be expressed in the petiolule frame.  Using
            # ``world_to_host`` here applied the petiolule transform twice and
            # left every blade visibly detached from its support.
            world_to_link=actual_world.GetInverse(),
        )
        added_mass = float(record.get("aggregated_mass_kg", 0.0))
        if added_mass > 0.0:
            # Preserve the lean V2 topology (no petiolule/blade rigid bodies),
            # while retaining the canonical leaf load. The dry biomass is
            # concentrated near the blade centroid and combined with the host
            # mass/COM, so gravity produces the expected bending moment.
            host_prim = stage.GetPrimAtPath(host_path)
            mass_api = UsdPhysics.MassAPI(host_prim)
            mass_attr = mass_api.GetMassAttr()
            com_attr = mass_api.GetCenterOfMassAttr()
            previous_mass = float(mass_attr.Get())
            previous_com_value = com_attr.Get()
            previous_com = Gf.Vec3d(*previous_com_value)
            blade_centroid_world = start + head * (
                length + 0.4 * blade_length
            )
            blade_centroid_local = world_to_host.Transform(blade_centroid_world)
            combined_mass = previous_mass + added_mass
            combined_com = (
                previous_com * previous_mass + blade_centroid_local * added_mass
            ) / combined_mass
            mass_attr.Set(combined_mass)
            com_attr.Set(Gf.Vec3f(*combined_com))
            current_visual_mass = float(
                host_prim.GetAttribute("autotom:aggregatedLeafVisualMassKg").Get()
                or 0.0
            )
            _custom(
                host_prim,
                "autotom:aggregatedLeafVisualMassKg",
                Sdf.ValueTypeNames.Double,
                current_visual_mass + added_mass,
            )
        _custom(
            root,
            "autotom:aggregatedMassKg",
            Sdf.ValueTypeNames.Double,
            added_mass,
        )
        _custom(
            root,
            "autotom:massSource",
            Sdf.ValueTypeNames.String,
            str(record.get("mass_source", "none")),
        )
        authored.append(
            {
                **record,
                "host_body_path": host_path,
                "root_path": root_path,
                "blade_length": blade_length,
                "renderer": "v2_segmented_organic",
                "visual_stats": visual_stats,
            }
        )
    return authored


def _author_historical_truss_visuals(
    stage, adapter: StemBranchesResult
) -> list[dict[str, Any]]:
    """Dress canonical trusses with the established continuous V2 appearance."""

    styles = {
        spec["canonical_axis_id"]: branch.get("visual_style")
        for branch in adapter.branches
        for spec in branch.get("link_specs", ())
        if branch.get("visual_style")
    }
    if not styles:
        return []
    authored = []
    for prim in list(stage.Traverse()):
        if prim.GetAttribute("autotom:entityKind").Get() != "physical_link":
            continue
        axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        style = styles.get(str(axis_id))
        if style is None:
            continue
        organic = next(
            (
                child
                for child in prim.GetChildren()
                if child.IsA(UsdGeom.Mesh)
                and child.GetName().startswith("OrganicVisual_")
            ),
            None,
        )
        length = float(prim.GetAttribute("autotom:sourceLength").Get())
        radius = float(prim.GetAttribute("autotom:visualRadius").Get())
        if style == "historical_truss_rachis":
            if organic is None:
                raise IncrementalCheckpointError(
                    f"truss link {prim.GetPath()} has no segmented organic mesh"
                )
            mesh = UsdGeom.Mesh(organic)
            mesh.CreateDisplayColorAttr().Set([PlantColors.TRUSS_RACHIS])
            UsdShade.MaterialBindingAPI.Apply(organic).Bind(
                get_or_create_tomato_truss_material(stage)
            )
            visual_path = str(organic.GetPath())
            visual_topology = "continuous_segmented_organic_axis"
        elif style == "historical_pedicel":
            if organic is not None:
                UsdGeom.Imageable(organic).MakeInvisible()
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            rotation = Gf.Rotation(matrix.ExtractRotationQuat())
            gravity_local = rotation.GetInverse().TransformDir(
                Gf.Vec3d(0.0, 0.0, -1.0)
            )
            centers, tangents = sample_gravity_elbow(
                length, str(axis_id), gravity_local
            )
            visual = create_gravity_elbow_mesh(
                stage,
                str(prim.GetPath()),
                centers,
                tangents,
                radius,
                str(axis_id),
            )
            visual_path = str(visual.GetPath())
            visual_topology = "gravity_elbow"
        else:
            raise IncrementalCheckpointError(
                f"unsupported historical truss visual style {style!r}"
            )
        _custom(prim, "autotom:visualStyle", Sdf.ValueTypeNames.String, style)
        authored.append(
            {
                "axis_id": str(axis_id),
                "body_path": str(prim.GetPath()),
                "visual_path": visual_path,
                "style": style,
                "visual_topology": visual_topology,
            }
        )
    return sorted(authored, key=lambda item: item["axis_id"])


def _segment_distance(a0, a1, b0, b1) -> float:
    """Shortest distance between finite 3-D line segments."""

    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    aa = u * u
    bb = u * v
    cc = v * v
    dd = u * w
    ee = v * w
    denominator = aa * cc - bb * bb
    small = 1e-15

    # Spheres are represented by zero-length swept segments.  Handle both
    # point/segment orders explicitly: the general segment algorithm below
    # assumes non-degenerate directions and used to miss sphere-as-right-hand
    # overlaps depending solely on deterministic path ordering.
    if aa < small and cc < small:
        return math.sqrt(w * w)
    if aa < small:
        t = max(0.0, min(1.0, ee / cc))
        delta = w - t * v
        return math.sqrt(delta * delta)
    if cc < small:
        s = max(0.0, min(1.0, -dd / aa))
        delta = w + s * u
        return math.sqrt(delta * delta)

    s_num = 0.0
    s_den = denominator
    t_num = 0.0
    t_den = denominator

    if denominator < small:
        s_num = 0.0
        s_den = 1.0
        t_num = ee
        t_den = cc
    else:
        s_num = bb * ee - cc * dd
        t_num = aa * ee - bb * dd
        if s_num < 0.0:
            s_num = 0.0
            t_num = ee
            t_den = cc
        elif s_num > s_den:
            s_num = s_den
            t_num = ee + bb
            t_den = cc

    if t_num < 0.0:
        t_num = 0.0
        if -dd < 0.0:
            s_num = 0.0
        elif -dd > aa:
            s_num = s_den
        else:
            s_num = -dd
            s_den = aa
    elif t_num > t_den:
        t_num = t_den
        if -dd + bb < 0.0:
            s_num = 0.0
        elif -dd + bb > aa:
            s_num = s_den
        else:
            s_num = -dd + bb
            s_den = aa

    sc = 0.0 if abs(s_num) < small else s_num / s_den
    tc = 0.0 if abs(t_num) < small else t_num / t_den
    delta = w + sc * u - tc * v
    return math.sqrt(delta * delta)


def _rigid_body_path(prim) -> str | None:
    current = prim
    while current and current.IsValid() and not current.IsPseudoRoot():
        if current.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(current.GetPath())
        current = current.GetParent()
    return None


def _collider_records(stage):
    """Return deterministic swept-segment records for authored colliders.

    Capsules are exact. Cylinders use their finite centerline and radius as a
    conservative narrow phase, while spheres are zero-length segments.
    """

    records = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        body_path = _rigid_body_path(prim)
        if body_path is None:
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        center = matrix.ExtractTranslation()
        if prim.IsA(UsdGeom.Capsule):
            shape = UsdGeom.Capsule(prim)
            shape_type = "capsule"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = float(shape.GetHeightAttr().Get()) * 0.5
        elif prim.IsA(UsdGeom.Cylinder):
            shape = UsdGeom.Cylinder(prim)
            shape_type = "cylinder"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = float(shape.GetHeightAttr().Get()) * 0.5
        elif prim.IsA(UsdGeom.Sphere):
            shape = UsdGeom.Sphere(prim)
            shape_type = "sphere"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = 0.0
        else:
            continue
        direction = Gf.Vec3d(
            matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        ).GetNormalized()
        start = center - direction * half_spine
        end = center + direction * half_spine
        records.append(
            {
                "path": str(prim.GetPath()),
                "body_path": body_path,
                "shape": shape_type,
                "start": start,
                "end": end,
                "radius": radius,
                "aabb_min": tuple(
                    min(float(start[i]), float(end[i])) - radius for i in range(3)
                ),
                "aabb_max": tuple(
                    max(float(start[i]), float(end[i])) + radius for i in range(3)
                ),
            }
        )
    return sorted(records, key=lambda item: item["path"])


def _filtered_targets(stage, body_path: str) -> set[str]:
    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        return set()
    relationship = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
    return {str(path) for path in relationship.GetTargets()}


def _body_metadata(stage, body_path: str) -> dict[str, Any]:
    prim = stage.GetPrimAtPath(body_path)
    return {
        "canonical_primitive_id": prim.GetAttribute(
            "autotom:canonicalPrimitiveId"
        ).Get(),
        "canonical_organ_id": prim.GetAttribute("autotom:canonicalOrganId").Get(),
        "role": prim.GetAttribute("autotom:role").Get(),
    }


def _author_truss_joint_armatures(
    stage,
    multiplier: float,
) -> list[dict[str, Any]]:
    """Add optional local-inertia armature only to truss D6 joints."""

    multiplier = float(multiplier)
    if multiplier == 0.0:
        return []
    authored: list[dict[str, Any]] = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "PhysicsJoint":
            continue
        joint = UsdPhysics.Joint(prim)
        targets = joint.GetBody1Rel().GetTargets()
        if len(targets) != 1:
            continue
        body = stage.GetPrimAtPath(targets[0])
        role = body.GetAttribute("autotom:role").Get()
        if role not in {"truss_rachis", "pedicel"}:
            continue
        length = float(body.GetAttribute("autotom:sourceLength").Get())
        radius = float(body.GetAttribute("autotom:visualRadius").Get())
        mass = float(UsdPhysics.MassAPI(body).GetMassAttr().Get())
        local_inertia = compute_moment_of_inertia(radius, length, mass)
        armature = multiplier * local_inertia
        apply_physx_joint_armature(stage, str(prim.GetPath()), armature)
        authored.append(
            {
                "joint_path": str(prim.GetPath()),
                "body_path": str(body.GetPath()),
                "role": str(role),
                "local_inertia_kg_m2": local_inertia,
                "multiplier": multiplier,
                "armature_kg_m2": armature,
            }
        )
    return authored


def _aabb_overlap(left, right) -> bool:
    return all(
        left["aabb_min"][axis] <= right["aabb_max"][axis]
        and right["aabb_min"][axis] <= left["aabb_max"][axis]
        for axis in range(3)
    )


def _audit_collider_overlaps(stage) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    colliders = _collider_records(stage)
    filtered_cache = {
        record["body_path"]: _filtered_targets(stage, record["body_path"])
        for record in colliders
    }
    filtered = []
    active = []
    for index, left in enumerate(colliders):
        for right in colliders[index + 1 :]:
            if left["body_path"] == right["body_path"]:
                continue
            if not _aabb_overlap(left, right):
                continue
            distance = _segment_distance(
                left["start"], left["end"], right["start"], right["end"]
            )
            depth = left["radius"] + right["radius"] - distance
            if depth <= 1e-9:
                continue
            is_filtered = (
                right["body_path"] in filtered_cache[left["body_path"]]
                or left["body_path"] in filtered_cache[right["body_path"]]
            )
            record = {
                "body_a": left["body_path"],
                "body_b": right["body_path"],
                "collider_a": left["path"],
                "collider_b": right["path"],
                "shape_a": left["shape"],
                "shape_b": right["shape"],
                "depth": round(float(depth), 12),
                "filtered": is_filtered,
                "metadata_a": _body_metadata(stage, left["body_path"]),
                "metadata_b": _body_metadata(stage, right["body_path"]),
            }
            (filtered if is_filtered else active).append(record)
    key = lambda value: (
        value["body_a"], value["body_b"], value["collider_a"], value["collider_b"]
    )
    return sorted(filtered, key=key), sorted(active, key=key)


def _audit_capsule_overlaps(stage) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    """Compatibility alias retained for existing checkpoint tests."""

    return _audit_collider_overlaps(stage)


def _auto_filter_initial_overlaps(stage) -> list[dict[str, Any]]:
    """Filter each still-active initial overlap once at rigid-body granularity."""

    _filtered, active = _audit_collider_overlaps(stage)
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for record in active:
        pair = tuple(sorted((record["body_a"], record["body_b"])))
        grouped.setdefault(pair, []).append(record)
    applied = []
    for (body_a, body_b), contacts in sorted(grouped.items()):
        add_collision_filter(stage, body_a, body_b)
        add_collision_filter(stage, body_b, body_a)
        deepest = max(contact["depth"] for contact in contacts)
        applied.append(
            {
                "body_a": body_a,
                "body_b": body_b,
                "reason": "auto_filtered_initial_authored_overlap",
                "depth": deepest,
                "contact_count": len(contacts),
                "contacts": contacts,
                "permanent_pair_filter": True,
            }
        )
    return applied


def _audit_stage(
    stage,
    state: PlantState,
    adapter: StemBranchesResult,
    physics_preset: str,
    usd_path: Path,
    approved_collision_filters: list[dict[str, Any]],
    rigid_leaf_visuals: list[dict[str, Any]],
    *,
    historical_truss_visuals: list[dict[str, Any]] | None = None,
    initial_overlap_policy: str = "filter",
    allow_over_budget: bool = False,
    truss_armatures: list[dict[str, Any]] | None = None,
    truss_armature_multiplier: float = 0.0,
    terminal_solver_preset: str = "current",
    allow_experimental_fruit_physics: bool = False,
) -> IncrementalCheckpointManifest:
    historical_truss_visuals = historical_truss_visuals or []
    truss_armatures = truss_armatures or []
    specs = [spec for branch in adapter.branches for spec in branch["link_specs"]]
    spec_by_axis = {spec["canonical_axis_id"]: spec for spec in specs}
    support_bodies = [
        prim
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "physical_link"
    ]
    bodies = [
        prim
        for prim in stage.Traverse()
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    fixed_joints = [
        prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsFixedJoint"
    ]
    d6_joints = [
        prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsJoint"
    ]
    meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
    all_organic_meshes = [
        prim for prim in meshes if prim.GetName().startswith("OrganicVisual_")
    ]
    organic_meshes = [
        prim
        for prim in all_organic_meshes
        if prim.GetParent().GetAttribute("autotom:entityKind").Get()
        == "physical_link"
    ]
    petiolule_meshes = [
        prim
        for prim in all_organic_meshes
        if prim.GetParent().GetAttribute("autotom:entityKind").Get()
        == "rigid_leaf_visual"
    ]
    leaf_blades = [prim for prim in meshes if prim.GetName() == "LeafBlade"]
    terminal_fork_shoots = [
        prim for prim in meshes if prim.GetName() == "TerminalForkYoungShoot"
    ]
    terminal_fork_leaves = [
        prim for prim in meshes if prim.GetName() == "TerminalForkYoungLeaf"
    ]
    cylinders = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Cylinder)]
    capsules = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Capsule)]
    spheres = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Sphere)]
    paths = [str(prim.GetPath()) for prim in support_bodies]
    errors = []

    def mesh_complexity(selected) -> dict[str, int]:
        points = 0
        faces = 0
        triangles = 0
        for prim in selected:
            mesh = UsdGeom.Mesh(prim)
            points += len(mesh.GetPointsAttr().Get() or ())
            counts = list(mesh.GetFaceVertexCountsAttr().Get() or ())
            faces += len(counts)
            triangles += sum(max(0, int(count) - 2) for count in counts)
        return {
            "meshes": len(selected),
            "points": points,
            "faces": faces,
            "triangles": triangles,
        }

    mesh_complexity_by_role = {
        "all": mesh_complexity(meshes),
        "organic": mesh_complexity(organic_meshes),
        "organic_all": mesh_complexity(all_organic_meshes),
        "petiolules": mesh_complexity(petiolule_meshes),
        "leaf_blades": mesh_complexity(leaf_blades),
    }

    resolved = resolve_vegetative_graph(
        list(adapter.branches), locked_joints=physics_preset == "locked"
    )
    resolved_axes = build_visual_axes(resolved, "/World/PlantVisual")

    expected_support_count = len(specs)
    expected_terminal_physical = sum(
        body.get("physical", True) for body in adapter.terminal_bodies
    )
    expected_terminal_visual = len(adapter.terminal_bodies) - expected_terminal_physical
    expected_count = expected_support_count + expected_terminal_physical
    expected_d6 = (
        sum(branch["n_links"] for branch in adapter.branches if branch["joint_type"] != "fixed")
        if physics_preset == "flexible"
        else 0
    )
    expected_fixed = expected_support_count - expected_d6 + expected_terminal_physical
    expected_axes = len({branch["visual_axis_id"] for branch in adapter.branches})
    branch_by_id = {branch["id"]: branch for branch in adapter.branches}
    expected_terminal_forks = sum(
        branch.get("kind") == "leaf_petiole"
        and not branch.get("disable_centered_terminal", False)
        and branch.get("parent") in branch_by_id
        and branch_by_id[branch["parent"]].get("kind")
        in {"stem", "lateral_branch"}
        and int(branch.get("attach_link", -1))
        == int(branch_by_id[branch["parent"]]["n_links"])
        for branch in adapter.branches
    )
    if len(bodies) != expected_count:
        errors.append(f"physical links {len(bodies)} != expected {expected_count}")
    if len(fixed_joints) != expected_fixed:
        errors.append(f"fixed joints {len(fixed_joints)} != expected {expected_fixed}")
    if len(d6_joints) != expected_d6:
        errors.append(f"D6 joints {len(d6_joints)} != expected {expected_d6}")
    if len(organic_meshes) != expected_support_count:
        errors.append(
            f"organic meshes {len(organic_meshes)} != expected {expected_support_count}"
        )
    if len(capsules) != expected_support_count * 2:
        errors.append(
            f"capsule colliders {len(capsules)} != expected {expected_support_count * 2}"
        )
    if len(resolved_axes) != expected_axes:
        errors.append(f"visual axes {len(resolved_axes)} != expected {expected_axes}")
    expected_truss_cylinders = 0
    if len(cylinders) != 0:
        errors.append(
            f"visual cylinders {len(cylinders)} != expected 0"
        )
    if len(paths) != len(set(paths)):
        errors.append("duplicate USD physical-link paths")
    if len(terminal_fork_shoots) != expected_terminal_forks:
        errors.append(
            f"terminal fork shoots {len(terminal_fork_shoots)} != expected "
            f"{expected_terminal_forks}"
        )
    if len(terminal_fork_leaves) != expected_terminal_forks:
        errors.append(
            f"terminal fork leaves {len(terminal_fork_leaves)} != expected "
            f"{expected_terminal_forks}"
        )

    authored_poses = []
    if adapter.pose_mode == "canonical":
        for prim in support_bodies:
            axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
            spec = spec_by_axis.get(axis_id)
            if spec is None:
                errors.append(f"body {prim.GetPath()} has unknown canonical axis {axis_id!r}")
                continue
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
            position = matrix.ExtractTranslation()
            direction = Gf.Vec3d(
                matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
            ).GetNormalized()
            length = float(spec["length"]) * GLOBAL_SCALE
            endpoint = position + direction * length
            expected_position = Gf.Vec3d(
                *(float(spec["rest_frame"][row][3]) * GLOBAL_SCALE for row in range(3))
            )
            expected_direction = Gf.Vec3d(
                *(float(spec["rest_frame"][row][2]) for row in range(3))
            ).GetNormalized()
            expected_endpoint = expected_position + expected_direction * length
            if (position - expected_position).GetLength() > 1e-6:
                errors.append(f"rest-position mismatch for {prim.GetPath()}")
            if (direction - expected_direction).GetLength() > 1e-6:
                errors.append(f"rest-direction mismatch for {prim.GetPath()}")
            if (endpoint - expected_endpoint).GetLength() > 1e-6:
                errors.append(f"rest-endpoint mismatch for {prim.GetPath()}")
            expected_radius = float(spec["radius"]) * GLOBAL_SCALE
            actual_radius = float(prim.GetAttribute("autotom:visualRadius").Get())
            if not math.isclose(actual_radius, expected_radius, abs_tol=1e-6):
                errors.append(f"visual-radius mismatch for {prim.GetPath()}")
            authored_poses.append(
                {
                    "axis_id": axis_id,
                    "body_path": str(prim.GetPath()),
                    "source_frame": spec.get("source_rest_frame", spec["rest_frame"]),
                    "authored_frame": spec["rest_frame"],
                    "source_length": float(spec.get("source_length", spec["length"])),
                    "authored_length": float(spec["length"]),
                    "authored_length_scale": float(
                        spec.get("authored_length_scale", 1.0)
                    ),
                    "authored_start": [float(value) for value in position],
                    "authored_direction": [float(value) for value in direction],
                    "authored_endpoint": [float(value) for value in endpoint],
                }
            )

    terminal_body_poses = []
    terminal_by_primitive = {
        str(body.get("canonical_primitive_id")): body
        for body in adapter.terminal_bodies
    }
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() not in {
            "terminal_body",
            "terminal_visual",
        }:
            continue
        primitive_id = str(
            prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        )
        source = terminal_by_primitive.get(primitive_id)
        if source is None:
            errors.append(
                f"terminal body {prim.GetPath()} has unknown canonical primitive "
                f"{primitive_id!r}"
            )
            continue
        authored_center = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            0
        ).ExtractTranslation()
        expected_center_raw = source.get("rest_center")
        if expected_center_raw is not None:
            expected_center = Gf.Vec3d(
                *(float(value) * GLOBAL_SCALE for value in expected_center_raw)
            )
            if (authored_center - expected_center).GetLength() > 1e-6:
                errors.append(f"terminal-center mismatch for {prim.GetPath()}")
        authored_quat = prim.GetAttribute("xformOp:orient").Get()
        terminal_body_poses.append(
            {
                "canonical_primitive_id": primitive_id,
                "body_path": str(prim.GetPath()),
                "source_center": source["source_center"],
                "source_frame": source.get("source_frame", source.get("rest_frame")),
                "authored_frame": source.get("rest_frame"),
                "authored_center": [float(value) for value in authored_center],
                "authored_orientation": (
                    [
                        float(authored_quat.GetReal()),
                        *(float(value) for value in authored_quat.GetImaginary()),
                    ]
                    if authored_quat is not None
                    else None
                ),
                "physical": bool(source.get("physical", True)),
            }
        )

    filtered_overlaps, active_overlaps = _audit_collider_overlaps(stage)
    if active_overlaps and adapter.pose_mode == "canonical":
        errors.append(
            f"{len(active_overlaps)} active collider overlaps are not covered by collision filtering"
        )

    gains = [
        {
            "branch_id": branch.branch_id,
            "joint_type": "fixed" if branch.locked_joints else branch.joint_type,
            "stiffness": branch.gains.stiffness,
            "damping": branch.gains.damping,
            "attachment_stiffness": branch.gains.attachment_stiffness,
            "attachment_damping": branch.gains.attachment_damping,
            "link_masses": list(branch.link_masses),
            "visual_radii": list(branch.link_radii),
            "collider_radii": list(branch.link_collider_radii),
        }
        for branch in resolved
    ]
    authored_body_masses = []
    for prim in bodies:
        mass_api = UsdPhysics.MassAPI(prim)
        authored_body_masses.append(
            {
                "body_path": str(prim.GetPath()),
                "mass_kg": float(mass_api.GetMassAttr().Get()),
                "center_of_mass": [
                    float(value) for value in mass_api.GetCenterOfMassAttr().Get()
                ],
                "aggregated_leaf_visual_mass_kg": float(
                    prim.GetAttribute("autotom:aggregatedLeafVisualMassKg").Get()
                    or 0.0
                ),
            }
        )

    schema = {
        "stem": STEM_CHECKPOINT_SCHEMA,
        "laterals": LATERALS_CHECKPOINT_SCHEMA,
        "leaf-supports": LEAF_SUPPORTS_CHECKPOINT_SCHEMA,
        "leaves": LEAVES_CHECKPOINT_SCHEMA,
        "truss-supports": TRUSS_SUPPORTS_CHECKPOINT_SCHEMA,
        "fruit-visual": FRUIT_VISUAL_CHECKPOINT_SCHEMA,
        "full": FULL_CHECKPOINT_SCHEMA,
    }[adapter.debug_profile]
    expected = {
        "internodes": sum(spec.get("axis_role") == "internode" for spec in specs),
        "rigid_bodies": expected_count,
        "fixed_joints": expected_fixed,
        "d6_joints": expected_d6,
        "capsule_colliders": expected_support_count * 2,
        "organic_meshes": expected_support_count,
        "visual_axes": expected_axes,
    }
    if adapter.debug_profile in {
        "leaf-supports",
        "leaves",
        "truss-supports",
        "fruit-visual",
        "full",
    }:
        expected.update(
            {
                "leaf_organs": len(
                    {
                        organ_id
                        for organ_id in adapter.represented_organ_ids
                        if any(
                            organ.id == organ_id and organ.organ_type == "Leaf"
                            for organ in state.organs
                        )
                    }
                ),
                "leaf_visual_axes": sum(
                    axis.axis_id.startswith(("Leaf_", "LatLeaf_"))
                    for axis in resolved_axes
                ),
                "leaf_support_links": sum(
                    spec.get("axis_role") in {"petiole", "leaf_rachis"}
                    for spec in specs
                ),
                "degenerate_leaf_organs": len(adapter.degenerate_organs),
            }
        )
    if adapter.debug_profile in {
        "leaves",
        "truss-supports",
        "fruit-visual",
        "full",
    }:
        expected_visual_only_petiolules = sum(
            not record.get("physical", False)
            for record in adapter.rigid_leaf_visuals
        )
        expected.update(
            {
                "rigid_leaf_visuals": len(adapter.rigid_leaf_visuals),
                "petiolule_visual_meshes": expected_visual_only_petiolules,
                "leaf_blades": len(adapter.rigid_leaf_visuals),
                "total_meshes": expected_support_count
                + expected_visual_only_petiolules
                + len(adapter.rigid_leaf_visuals),
                "terminal_forks": expected_terminal_forks,
            }
        )
        expected["total_meshes"] += 2 * expected_terminal_forks
        if len(rigid_leaf_visuals) != len(adapter.rigid_leaf_visuals):
            errors.append(
                f"rigid leaf visuals {len(rigid_leaf_visuals)} != expected "
                f"{len(adapter.rigid_leaf_visuals)}"
            )
        if len(petiolule_meshes) != expected_visual_only_petiolules:
            errors.append(
                f"petiolule meshes {len(petiolule_meshes)} != expected "
                f"{expected_visual_only_petiolules}"
            )
        if len(leaf_blades) != len(adapter.rigid_leaf_visuals):
            errors.append(
                f"leaf blades {len(leaf_blades)} != expected "
                f"{len(adapter.rigid_leaf_visuals)}"
            )
    if adapter.debug_profile in {"truss-supports", "fruit-visual", "full"}:
        expected.update(
            {
                "truss_rachis_links": sum(
                    spec.get("axis_role") == "truss_rachis" for spec in specs
                ),
                "pedicel_links": sum(
                    spec.get("axis_role") == "pedicel" for spec in specs
                ),
                "historical_truss_visuals": len(historical_truss_visuals),
                "fruit_spheres": len(adapter.terminal_bodies),
                "terminal_physical_bodies": expected_terminal_physical,
                "terminal_visual_bodies": expected_terminal_visual,
            }
        )
        if len(spheres) != len(adapter.terminal_bodies):
            errors.append(
                f"fruit spheres {len(spheres)} != expected {len(adapter.terminal_bodies)}"
            )
    return IncrementalCheckpointManifest(
        metadata={
            "day": state.metadata.simulation_time,
            "plant_id": state.metadata.plant_id,
            "debug_profile": adapter.debug_profile,
            "pose_mode": adapter.pose_mode,
            "appendage_pose_mode": adapter.appendage_pose_mode,
            "physics_preset": physics_preset,
            "leaf_joint_policy": adapter.leaf_joint_policy,
            "lateral_joint_policy": adapter.lateral_joint_policy,
            "truss_calibration_preset": adapter.truss_calibration_preset,
            "truss_damping_override": adapter.truss_damping_override,
            "truss_armature_multiplier": truss_armature_multiplier,
            "terminal_solver_preset": terminal_solver_preset,
            "physical_petiolules": adapter.physical_petiolules,
            "initial_overlap_policy": initial_overlap_policy,
            "allow_over_budget": allow_over_budget,
            "experimental_fruit_physics": (
                adapter.debug_profile == "full"
                and allow_experimental_fruit_physics
            ),
            "fruit_physics_support_status": (
                "unsupported_experimental"
                if adapter.debug_profile == "full"
                else "not_authored"
            ),
            "visual_quality": adapter.visual_quality,
            "visual_profile": (
                dict(adapter.branches[0]["visual_profile"])
                if adapter.branches
                else None
            ),
            "visual_renderer": "historical_v2_skinned_segmented",
            "global_scale": GLOBAL_SCALE,
            "usd": str(usd_path),
        },
        expected=expected,
        authored={
            "rigid_bodies": len(bodies),
            "support_rigid_bodies": len(support_bodies),
            "fixed_joints": len(fixed_joints),
            "d6_joints": len(d6_joints),
            "capsule_colliders": len(capsules),
            "organic_meshes": len(organic_meshes),
            "rigid_visual_organic_meshes": len(petiolule_meshes),
            "petiolule_visual_meshes": len(petiolule_meshes),
            "leaf_blades": len(leaf_blades),
            "terminal_fork_shoots": len(terminal_fork_shoots),
            "terminal_fork_leaves": len(terminal_fork_leaves),
            "total_meshes": len(meshes),
            "visual_axes": len(resolved_axes),
            "visual_cylinders": len(cylinders),
            "fruit_spheres": len(spheres),
            "historical_truss_visuals": historical_truss_visuals,
            "paths": paths,
            "poses": authored_poses,
            "terminal_body_poses": sorted(
                terminal_body_poses,
                key=lambda item: item["canonical_primitive_id"],
            ),
            "mesh_complexity": mesh_complexity_by_role,
        },
        topology={
            "branches": [
                {
                    "id": branch["id"],
                    "parent": branch["parent"],
                    "attach_link": branch["attach_link"],
                    "joint_type": branch["joint_type"],
                    "visual_profile": branch.get("visual_profile"),
                    "link_axis_ids": [spec["canonical_axis_id"] for spec in branch["link_specs"]],
                    "link_groimp_node_ids": [spec["groimp_node_id"] for spec in branch["link_specs"]],
                }
                for branch in adapter.branches
            ],
            "attachment_map": list(adapter.attachment_map),
            "source_axis_ids": list(adapter.source_axis_ids),
            "represented_organ_ids": list(adapter.represented_organ_ids),
            "collapsed_duplicates": list(adapter.collapsed_duplicates),
            "degenerate_organs": list(adapter.degenerate_organs),
            "rigid_leaf_visuals": rigid_leaf_visuals,
            "terminal_bodies": list(adapter.terminal_bodies),
        },
        physics={
            "branch_gains": gains,
            "authored_body_masses": authored_body_masses,
            "aggregated_leaf_visual_mass_kg": sum(
                float(record.get("aggregated_mass_kg", 0.0))
                for record in rigid_leaf_visuals
            ),
            "visual_axis_ids": [axis.axis_id for axis in resolved_axes],
            "leaf_support_policy": (
                {
                    "petiole": "d6",
                    "leaf_rachis": (
                        "d6_distributed"
                        if adapter.leaf_joint_policy == "distributed"
                        else "fixed_to_petiole"
                    ),
                    "policy": adapter.leaf_joint_policy,
                    "visual_geometry": "complete_canonical_segmented",
                }
                if adapter.debug_profile
                in {
                    "leaf-supports",
                    "leaves",
                    "truss-supports",
                    "fruit-visual",
                    "full",
                }
                else None
            ),
            "truss_profile": (
                {
                    "rachis_young_modulus_pa": next((
                        float(branch["young_modulus"])
                        for branch in adapter.branches
                        if branch.get("truss_component") == "rachis"
                    ), None),
                    "pedicel_young_modulus_pa": next((
                        float(branch["young_modulus"])
                        for branch in adapter.branches
                        if branch.get("truss_component") == "pedicel"
                    ), None),
                    "rachis_damping_ratio": next((
                        float(branch["damping_ratio"])
                        for branch in adapter.branches
                        if branch.get("truss_component") == "rachis"
                    ), None),
                    "pedicel_damping_ratio": next((
                        float(branch["damping_ratio"])
                        for branch in adapter.branches
                        if branch.get("truss_component") == "pedicel"
                    ), None),
                    "density_kg_m3": next((
                        float(branch["density"])
                        for branch in adapter.branches
                        if branch.get("truss_component") in {"rachis", "pedicel"}
                    ), None),
                    "pedicel_bend_limit_deg": TrussPhysicsConfig.PEDICEL_BEND_LIMIT_DEG,
                    "pedicel_drive_stiffness_scale": next((
                        float(branch["drive_stiffness_scale"])
                        for branch in adapter.branches
                        if branch.get("truss_component") == "pedicel"
                    ), None),
                    "tomato_break_force_n": TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N,
                }
                if adapter.debug_profile in {"truss-supports", "fruit-visual", "full"}
                else None
            ),
            "truss_armatures": truss_armatures,
            "terminal_solver": {
                "preset": terminal_solver_preset,
                "position_iterations": (
                    PhysicsRuntimeConfig.STABILIZED_TERMINAL_BODY_SOLVER_POSITION_ITERATIONS
                    if terminal_solver_preset == "stabilized"
                    else PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS
                ),
                "velocity_iterations": (
                    PhysicsRuntimeConfig.STABILIZED_TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS
                    if terminal_solver_preset == "stabilized"
                    else PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS
                ),
            },
            "joint_budget": {
                "predicted_d6": expected_d6,
                "target": 220,
                "review_max": 230,
                "over_budget_override": allow_over_budget,
            },
        },
        collisions={
            "blocking_policy": (
                "legacy_diagnostic_only"
                if adapter.pose_mode != "canonical"
                else (
                    "auto_filter_exact_initial_body_pairs"
                    if initial_overlap_policy == "filter"
                    else "canonical_active_overlaps_fail"
                )
            ),
            "filtered_overlaps": filtered_overlaps,
            "active_overlaps": active_overlaps,
            "applied_initial_filters": approved_collision_filters,
            "permanent_filter_warning": (
                "Filtered body pairs can pass through each other for the rest "
                "of the simulation."
            ),
        },
        errors=tuple(errors),
        schema_version=schema,
    )


def _apply_approved_collision_filters(stage, adapter: StemBranchesResult) -> list[dict[str, Any]]:
    """Apply only explicitly approved source-overlap pairs, never a general rule."""

    if adapter.pose_mode != "canonical":
        return []
    body_by_axis = {}
    for prim in stage.Traverse():
        axis_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if axis_id is not None:
            body_by_axis[str(axis_id)] = str(prim.GetPath())

    applied = []
    for record in adapter.approved_collision_filters:
        axis_a = record["axis_a"]
        axis_b = record["axis_b"]
        body_a = body_by_axis.get(axis_a)
        body_b = body_by_axis.get(axis_b)
        if body_a is None or body_b is None:
            raise IncrementalCheckpointError(
                f"approved collision pair cannot resolve bodies: {axis_a}, {axis_b}"
            )
        add_collision_filter(stage, body_a, body_b)
        add_collision_filter(stage, body_b, body_a)
        applied.append({**record, "body_a": body_a, "body_b": body_b})
    return applied


def save_manifest(manifest: IncrementalCheckpointManifest, path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            manifest.to_dict(),
            indent=2,
            sort_keys=True,
            ensure_ascii=False,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return path


def export_incremental_checkpoint(
    state: PlantState,
    output_path: str | Path,
    *,
    debug_profile: str,
    pose_mode: str = "canonical",
    appendage_pose_mode: str = "v2-aesthetic",
    physics_preset: str = "flexible",
    physics_hz: int = 480,
    leaf_joint_policy: str = "distributed",
    lateral_joint_policy: str = "dynamic",
    truss_calibration_preset: str = "current",
    truss_damping_override: float | None = None,
    truss_armature_multiplier: float = 0.0,
    terminal_solver_preset: str = "current",
    visual_quality: str = "realistic",
    physical_petiolules: bool = False,
    initial_overlap_policy: str = "filter",
    allow_near_budget: bool = False,
    allow_over_budget: bool = False,
    allow_experimental_fruit_physics: bool = False,
) -> tuple[IncrementalCheckpointPlan, Path, Path]:
    """Build and audit one PlantState profile with the original V2 backend."""

    if debug_profile not in INCREMENTAL_PROFILES:
        raise IncrementalCheckpointError(
            f"incremental profile must be one of {INCREMENTAL_PROFILES}, got {debug_profile!r}"
        )
    if debug_profile == "full" and not allow_experimental_fruit_physics:
        raise IncrementalCheckpointError(
            "full PlantState fruit physics is unsupported and requires "
            "--allow-experimental-fruit-physics; use truss-supports for the "
            "validated fruit-free builder"
        )
    if physics_preset not in ("locked", "flexible"):
        raise IncrementalCheckpointError(
            f"physics_preset must be locked or flexible, got {physics_preset!r}"
        )
    if leaf_joint_policy not in LEAF_JOINT_POLICIES:
        raise IncrementalCheckpointError(
            "leaf_joint_policy must be one of "
            f"{LEAF_JOINT_POLICIES}, got {leaf_joint_policy!r}"
        )
    if lateral_joint_policy not in LATERAL_JOINT_POLICIES:
        raise IncrementalCheckpointError(
            "lateral_joint_policy must be one of "
            f"{LATERAL_JOINT_POLICIES}, got {lateral_joint_policy!r}"
        )
    if truss_calibration_preset not in TRUSS_CALIBRATION_PRESETS:
        raise IncrementalCheckpointError(
            "truss_calibration_preset must be one of "
            f"{TRUSS_CALIBRATION_PRESETS}, got {truss_calibration_preset!r}"
        )
    if (
        truss_damping_override is not None
        and float(truss_damping_override) not in TRUSS_DAMPING_CHOICES
    ):
        raise IncrementalCheckpointError(
            "truss_damping_override must be one of "
            f"{TRUSS_DAMPING_CHOICES}, got {truss_damping_override!r}"
        )
    truss_armature_multiplier = float(truss_armature_multiplier)
    if truss_armature_multiplier not in TRUSS_ARMATURE_MULTIPLIERS:
        raise IncrementalCheckpointError(
            "truss_armature_multiplier must be one of "
            f"{TRUSS_ARMATURE_MULTIPLIERS}, got {truss_armature_multiplier!r}"
        )
    if terminal_solver_preset not in TERMINAL_SOLVER_PRESETS:
        raise IncrementalCheckpointError(
            "terminal_solver_preset must be one of "
            f"{TERMINAL_SOLVER_PRESETS}, got {terminal_solver_preset!r}"
        )
    if visual_quality not in VISUAL_QUALITY_MODES:
        raise IncrementalCheckpointError(
            "visual_quality must be one of "
            f"{VISUAL_QUALITY_MODES}, got {visual_quality!r}"
        )
    if initial_overlap_policy not in INITIAL_OVERLAP_POLICIES:
        raise IncrementalCheckpointError(
            "initial_overlap_policy must be one of "
            f"{INITIAL_OVERLAP_POLICIES}, got {initial_overlap_policy!r}"
        )
    if physical_petiolules and debug_profile not in {
        "leaves",
        "truss-supports",
        "fruit-visual",
        "full",
    }:
        raise IncrementalCheckpointError(
            "physical petiolules require leaves or a cumulative later profile"
        )
    if allow_over_budget and not physical_petiolules:
        raise IncrementalCheckpointError(
            "--allow-over-budget is diagnostic-only and requires "
            "--physical-petiolules"
        )
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    adapter_builders = {
        "stem": build_stem_branches,
        "laterals": build_lateral_branches,
        "leaf-supports": build_leaf_support_branches,
        "leaves": build_leaf_branches,
        "truss-supports": build_truss_branches,
        "fruit-visual": build_truss_branches,
        "full": build_truss_branches,
    }
    adapter_kwargs = {
        "pose_mode": pose_mode,
        "visual_quality": visual_quality,
    }
    if debug_profile in {
        "leaf-supports",
        "leaves",
        "truss-supports",
        "fruit-visual",
        "full",
    }:
        adapter_kwargs["leaf_joint_policy"] = leaf_joint_policy
    if debug_profile in {"leaves", "truss-supports", "fruit-visual", "full"}:
        adapter_kwargs["physical_petiolules"] = physical_petiolules
        adapter_kwargs["appendage_pose_mode"] = appendage_pose_mode
    if debug_profile in {"fruit-visual", "full"}:
        adapter_kwargs["include_fruits"] = True
        adapter_kwargs["physical_fruits"] = debug_profile == "full"
    adapter = adapter_builders[debug_profile](state, **adapter_kwargs)
    adapter = apply_checkpoint_physics_policy(
        adapter,
        lateral_joint_policy=lateral_joint_policy,
        truss_calibration_preset=truss_calibration_preset,
        truss_damping_override=truss_damping_override,
    )
    locked = physics_preset == "locked"
    predicted_d6 = (
        sum(
            branch["n_links"]
            for branch in adapter.branches
            if branch["joint_type"] != "fixed"
        )
        if not locked
        else 0
    )
    if predicted_d6 > 230 and not allow_over_budget:
        raise IncrementalCheckpointError(
            f"predicted D6 joints {predicted_d6} exceed the hard diagnostic "
            "limit 230; physical petiolules require --allow-over-budget"
        )
    if 220 < predicted_d6 <= 230 and not (allow_near_budget or allow_over_budget):
        raise IncrementalCheckpointError(
            f"predicted D6 joints {predicted_d6} are in the 221-230 review "
            "band; use --allow-near-budget"
        )
    if allow_over_budget and predicted_d6 > 230:
        print(
            f"[WARNING] diagnostic export authoring {predicted_d6} D6 joints: "
            "physical petiolules substantially increase simulation cost and "
            "may destabilize PhysX"
        )
    stage, stem_path = build_stage(
        str(destination),
        branches=list(adapter.branches),
        terminal_bodies=list(adapter.terminal_bodies),
        locked_joints=locked,
        skip_limit_check=allow_over_budget,
        branch_backend="skinned",
        skinning_visual_mode="segmented",
    )
    truss_armatures = _author_truss_joint_armatures(
        stage, truss_armature_multiplier
    )
    if terminal_solver_preset == "stabilized":
        for prim in stage.Traverse():
            if prim.GetAttribute("autotom:entityKind").Get() == "terminal_body":
                apply_physx_rigid_body_solver_settings(
                    stage,
                    str(prim.GetPath()),
                    PhysicsRuntimeConfig.STABILIZED_TERMINAL_BODY_SOLVER_POSITION_ITERATIONS,
                    PhysicsRuntimeConfig.STABILIZED_TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS,
                )
    _author_stage_metadata(
        stage,
        state,
        adapter,
        physics_preset,
        truss_armature_multiplier=truss_armature_multiplier,
        terminal_solver_preset=terminal_solver_preset,
        allow_experimental_fruit_physics=allow_experimental_fruit_physics,
    )
    rigid_leaf_visuals = _author_rigid_leaf_visuals(stage, adapter)
    historical_truss_visuals = _author_historical_truss_visuals(stage, adapter)
    approved_filters = _apply_approved_collision_filters(stage, adapter)
    auto_filters = (
        _auto_filter_initial_overlaps(stage)
        if initial_overlap_policy == "filter" and adapter.pose_mode == "canonical"
        else []
    )
    apply_physx_scene_settings(stage, physics_hz=physics_hz)
    apply_physx_articulation_settings(stage, stem_path)
    stage.GetRootLayer().Save()
    manifest = _audit_stage(
        stage,
        state,
        adapter,
        physics_preset,
        destination,
        [*approved_filters, *auto_filters],
        rigid_leaf_visuals,
        historical_truss_visuals=historical_truss_visuals,
        initial_overlap_policy=initial_overlap_policy,
        allow_over_budget=allow_over_budget,
        truss_armatures=truss_armatures,
        truss_armature_multiplier=truss_armature_multiplier,
        terminal_solver_preset=terminal_solver_preset,
        allow_experimental_fruit_physics=allow_experimental_fruit_physics,
    )
    manifest_path = save_manifest(manifest, manifest_path_for(destination))
    if manifest.errors:
        raise IncrementalCheckpointError("; ".join(manifest.errors))
    return (
        IncrementalCheckpointPlan(
            adapter=adapter,
            physical_link_count=sum(branch["n_links"] for branch in adapter.branches),
            predicted_d6_joints=predicted_d6,
            debug_profile=debug_profile,
        ),
        destination,
        manifest_path,
    )


def export_stem_checkpoint(
    state: PlantState,
    output_path: str | Path,
    *,
    pose_mode: str = "canonical",
    physics_preset: str = "flexible",
    physics_hz: int = 480,
) -> tuple[IncrementalCheckpointPlan, Path, Path]:
    """Compatibility wrapper for the completed stem checkpoint."""

    return export_incremental_checkpoint(
        state,
        output_path,
        debug_profile="stem",
        pose_mode=pose_mode,
        physics_preset=physics_preset,
        physics_hz=physics_hz,
    )
