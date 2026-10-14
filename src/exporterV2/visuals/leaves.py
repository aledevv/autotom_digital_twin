"""Leaflets: a short organic stalk (petiolule) plus a blade at its tip.

Leaflets are visual only. They are parented to the rigid body of the petiole
or leaf rachis they grow from, and their mass is added to that body so the
leaf still bends under its own weight.

Blade shapes come from ``leaf_shapes`` (``real_leaves`` / ``real_leaves_i3``)
or from the procedural blade in ``core/skinning/leaf_blade.py`` (``simple``).
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Iterable

from pxr import Gf, Sdf, UsdGeom, UsdPhysics

from ..core import tree_config
from ..core.skinning import VisualAxisData, VisualProfile, VisualSegment
from ..core.skinning.adapter import _quat_from_column_rotation
from ..core.skinning.leaf_blade import (
    LEAF_ARCH_LIFT_FRACTION,
    LEAF_HALF_WIDTH_FRACTION,
    LEAF_LENGTH_FRACTION,
    LEAF_LONGITUDINAL_FOLD_FRACTION,
    LEAF_TIP_SAG_FRACTION,
    author_leaf_blade,
)
from ..core.skinning.visual_segmented import author_segmented_visual_axis

# The petiolule tapers to this fraction of its base radius.
PETIOLULE_TIP_RADIUS_FRACTION = 0.65
# Blade length is the petiolule length times LEAF_LENGTH_FRACTION, clamped.
BLADE_LENGTH_MIN_M = 0.04
BLADE_LENGTH_MAX_M = 0.09
# Where the blade weight acts, as a fraction of the blade length.
BLADE_CENTROID_FRACTION = 0.4


class LeafVisualError(ValueError):
    pass


def _custom(prim, name: str, value_type, value) -> None:
    prim.CreateAttribute(name, value_type, custom=True).Set(value)


def blade_length(petiolule_length: float) -> float:
    return min(BLADE_LENGTH_MAX_M, max(BLADE_LENGTH_MIN_M, petiolule_length * LEAF_LENGTH_FRACTION))


def author_leaf_visuals(
    stage,
    records: Iterable[dict[str, Any]],
    visual_profile: dict[str, Any],
    leaf_shapes=None,
) -> list[dict[str, Any]]:
    """Author every leaflet record under its host rigid body."""

    records = list(records)
    if not records:
        return []
    scale = tree_config.GLOBAL_SCALE
    body_by_node = {}
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() != "physical_link":
            continue
        node_id = prim.GetAttribute("autotom:canonicalPrimitiveId").Get()
        if node_id is not None:
            body_by_node[str(node_id)] = str(prim.GetPath())
    profile = VisualProfile(
        radial_segments=int(visual_profile["radial_segments"]),
        axial_spacing_m=float(visual_profile["axial_spacing_m"]),
        radius_transition_samples=int(visual_profile["radius_transition_samples"]),
    )

    authored = []
    for record in records:
        host_path = body_by_node.get(record["host_axis_id"])
        if host_path is None:
            raise LeafVisualError(f"leaflet {record['axis_id']} has no host body {record['host_axis_id']}")
        host_prim = stage.GetPrimAtPath(host_path)
        world_to_host = UsdGeom.Xformable(host_prim).ComputeLocalToWorldTransform(0).GetInverse()
        frame = record["rest_frame"]
        start = Gf.Vec3d(*(float(frame[row][3]) * scale for row in range(3)))
        head = Gf.Vec3d(*(float(frame[row][2]) for row in range(3))).GetNormalized()
        length = float(record["length"]) * scale
        radius = float(record["radius"]) * scale

        root_path = f"{host_path}/RigidLeafVisuals/{record['id']}"
        root_xform = UsdGeom.Xform.Define(stage, root_path)
        root = root_xform.GetPrim()
        orientation = _quat_from_column_rotation(frame)
        desired_world = Gf.Matrix4d(1.0)
        desired_world.SetTransform(Gf.Rotation(Gf.Quatd(orientation)), start)
        root_xform.AddTransformOp().Set(desired_world * world_to_host)
        actual_world = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0)
        frame_error = max(
            abs(float(actual_world[row][column] - desired_world[row][column]))
            for row in range(4)
            for column in range(4)
        )
        if frame_error > 1e-8:
            raise LeafVisualError(f"leaflet {record['axis_id']} frame error {frame_error:.3g}")
        _custom(root, "autotom:entityKind", Sdf.ValueTypeNames.String, "rigid_leaf_visual")
        _custom(root, "autotom:attachmentMode", Sdf.ValueTypeNames.String, "rigid_visual")
        _custom(root, "autotom:canonicalOrganId", Sdf.ValueTypeNames.String, record["organ_id"])
        _custom(root, "autotom:canonicalPrimitiveId", Sdf.ValueTypeNames.String, record["axis_id"])
        _custom(root, "autotom:skeletonNodeId", Sdf.ValueTypeNames.String, record["node_id"])
        _custom(root, "autotom:role", Sdf.ValueTypeNames.String, record["role"])
        _custom(root, "autotom:hostPrimitiveId", Sdf.ValueTypeNames.String, record["host_axis_id"])

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
                    end_radius=radius * PETIOLULE_TIP_RADIUS_FRACTION,
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
            parent_radius=float(host_prim.GetAttribute("autotom:visualRadius").Get()),
            attachment_arcs=[],
        )
        visual_stats = author_segmented_visual_axis(stage, visual_axis)

        blade = blade_length(length)
        author_leaf_blade(
            stage,
            f"{root_path}/LeafBlade",
            start + head * length,
            head,
            length=blade,
            half_width=blade * LEAF_HALF_WIDTH_FRACTION,
            fold_depth=blade * LEAF_LONGITUDINAL_FOLD_FRACTION,
            arch_lift=blade * LEAF_ARCH_LIFT_FRACTION,
            tip_sag=blade * LEAF_TIP_SAG_FRACTION,
            color=tree_config.PlantColors.LEAF_BLADE,
            # Blade vertices are expressed in the petiolule frame.
            world_to_link=actual_world.GetInverse(),
            shape=(leaf_shapes.for_leaf(record["leaflet_id"]) if leaf_shapes is not None else None),
        )
        added_mass = float(record.get("aggregated_mass_kg", 0.0))
        if added_mass > 0.0:
            _add_point_mass(host_prim, world_to_host.Transform(start + head * (length + BLADE_CENTROID_FRACTION * blade)), added_mass)
        _custom(root, "autotom:aggregatedMassKg", Sdf.ValueTypeNames.Double, added_mass)
        _custom(root, "autotom:massSource", Sdf.ValueTypeNames.String, str(record.get("mass_source", "none")))
        authored.append(
            {
                **record,
                "host_body_path": host_path,
                "root_path": root_path,
                "blade_length": blade,
                "visual_stats": visual_stats,
            }
        )
    return authored


def _add_point_mass(body_prim, point_local, mass: float) -> None:
    """Add ``mass`` at ``point_local`` to the body, updating its centre of mass."""

    mass_api = UsdPhysics.MassAPI(body_prim)
    previous_mass = float(mass_api.GetMassAttr().Get())
    previous_com = Gf.Vec3d(*mass_api.GetCenterOfMassAttr().Get())
    combined = previous_mass + mass
    mass_api.GetMassAttr().Set(combined)
    mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(*((previous_com * previous_mass + point_local * mass) / combined)))
    current = float(body_prim.GetAttribute("autotom:aggregatedLeafVisualMassKg").Get() or 0.0)
    _custom(body_prim, "autotom:aggregatedLeafVisualMassKg", Sdf.ValueTypeNames.Double, current + mass)
