"""Static V1 renderer consuming canonical :mod:`plant_state` data."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

import numpy as np
from pxr import Sdf, Usd, UsdGeom

from plant_state import FruitsProperties, PlantState

from .adapter import V1OrganView, build_v1_render_view, legacy_leaf_view
from .audit import V1AuditError, audit_v1_stage, manifest_path_for, save_v1_manifest
from .constants import ROOT_SPHERE_RADIUS
from .usd_helpers import (
    _bind_material,
    _make_leaf,
    _make_material,
    _make_sphere,
    _set_transform,
)


def _token(value: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9_]", "_", value)
    if not sanitized or sanitized[0].isdigit():
        sanitized = "id_" + sanitized
    return sanitized


def _matrix(value: Any) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4):
        raise ValueError("V1 transforms must be 4x4")
    return matrix


def _centered_axis_matrix(frame: Any, length: float) -> np.ndarray:
    result = _matrix(frame).copy()
    result[:3, 3] += result[:3, 2] * (float(length) / 2.0)
    return result


def _set_string(prim: Usd.Prim, name: str, value: str | None) -> None:
    prim.CreateAttribute(name, Sdf.ValueTypeNames.String, custom=True).Set(
        "" if value is None else str(value)
    )


def _tag_entity(
    prim: Usd.Prim,
    *,
    entity_kind: str,
    node_id: str | None = None,
    organ_type: str | None = None,
    geometry_role: str | None = None,
) -> None:
    _set_string(prim, "autotom:entityKind", entity_kind)
    if node_id is not None:
        _set_string(prim, "autotom:nodeId", node_id)
    if organ_type is not None:
        _set_string(prim, "autotom:organType", organ_type)
    if geometry_role is not None:
        _set_string(prim, "autotom:geometryRole", geometry_role)


def _define_cylinder(
    stage: Usd.Stage,
    path: str,
    *,
    length: float,
    radius: float,
    frame: Any,
    material: Usd.Prim,
    node_id: str,
    role: str,
) -> UsdGeom.Cylinder:
    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.GetHeightAttr().Set(float(length))
    cylinder.GetRadiusAttr().Set(float(radius))
    cylinder.GetAxisAttr().Set(UsdGeom.Tokens.z)
    _set_transform(cylinder, _centered_axis_matrix(frame, length))
    _bind_material(cylinder, material)
    _tag_entity(
        cylinder.GetPrim(), entity_kind="geometry", node_id=node_id, geometry_role=role
    )
    return cylinder


def _materials(stage: Usd.Stage, plant_path: str) -> dict[str, Usd.Prim]:
    path = f"{plant_path}/Materials"
    UsdGeom.Scope.Define(stage, path)
    return {
        "stem": _make_material(stage, f"{path}/Stem", (0.45, 0.30, 0.10)),
        "root": _make_material(stage, f"{path}/Root", (0.55, 0.35, 0.15)),
        "leaf": _make_material(stage, f"{path}/Leaf", (0.15, 0.55, 0.10)),
        "pedicel": _make_material(stage, f"{path}/Pedicel", (0.20, 0.50, 0.10)),
        "fruit_ripe": _make_material(stage, f"{path}/FruitRipe", (0.90, 0.17, 0.10)),
        "fruit_unripe": _make_material(
            stage, f"{path}/FruitUnripe", (0.45, 0.58, 0.25)
        ),
    }


def _author_topology(stage: Usd.Stage, plant_path: str, state: PlantState) -> None:
    root = f"{plant_path}/Topology"
    UsdGeom.Scope.Define(stage, root)
    operations = {
        operation.node_id: operation for operation in state.turtle_operations
    }
    for node in sorted(state.nodes, key=lambda item: item.id):
        prim = UsdGeom.Xform.Define(stage, f"{root}/{_token(node.id)}").GetPrim()
        _tag_entity(prim, entity_kind="topology_node", node_id=node.id)
        _set_string(prim, "autotom:sourceType", node.source_type)
        _set_string(prim, "autotom:category", node.category)
        _set_string(prim, "autotom:parentId", node.parent_id)
        _set_string(prim, "autotom:incomingEdgeKind", node.incoming_edge_kind)
        prim.CreateAttribute(
            "autotom:groimpNodeId", Sdf.ValueTypeNames.Int64, custom=True
        ).Set(node.groimp_node_id)
        operation = operations.get(node.id)
        if operation is not None:
            _set_string(prim, "autotom:turtleOperation", operation.operation)
            _set_string(
                prim,
                "autotom:turtleParameters",
                json.dumps(
                    operation.parameters,
                    sort_keys=True,
                    separators=(",", ":"),
                    allow_nan=False,
                ),
            )
            _set_string(prim, "autotom:turtleProvenance", operation.provenance)


def _organ_group(
    stage: Usd.Stage, geometry_root: str, view: V1OrganView
) -> tuple[str, UsdGeom.Xform]:
    organ_type = view.organ.organ_type
    path = f"{geometry_root}/{organ_type}/{_token(view.node.id)}"
    UsdGeom.Scope.Define(stage, f"{geometry_root}/{organ_type}")
    group = UsdGeom.Xform.Define(stage, path)
    _set_transform(group, _matrix(view.node.pose.incoming_world))
    prim = group.GetPrim()
    _tag_entity(
        prim,
        entity_kind="organ",
        node_id=view.node.id,
        organ_type=organ_type,
    )
    _set_string(prim, "autotom:parentId", view.node.parent_id)
    _set_string(prim, "autotom:incomingEdgeKind", view.node.incoming_edge_kind)
    if view.duplicate_of is not None:
        _set_string(prim, "autotom:geometryDuplicateOf", view.duplicate_of)
    topology_path = f"{geometry_root.rsplit('/', 1)[0]}/Topology/{_token(view.node.id)}"
    prim.CreateRelationship("autotom:topologyNode", custom=True).SetTargets(
        [Sdf.Path(topology_path)]
    )
    return path, group


def _render_internode(
    stage: Usd.Stage,
    group_path: str,
    view: V1OrganView,
    materials: dict[str, Usd.Prim],
) -> None:
    axes = [axis for axis in view.axes if axis.role == "internode"]
    if len(axes) != 1:
        raise V1AuditError(
            f"Internode {view.node.id} requires exactly one canonical axis, found {len(axes)}"
        )
    axis = axes[0]
    _define_cylinder(
        stage,
        f"{group_path}/Internode",
        length=axis.length,
        radius=axis.radius,
        frame=axis.local_frame,
        material=materials["stem"],
        node_id=view.node.id,
        role="internode",
    )


def _render_leaf(
    stage: Usd.Stage,
    group_path: str,
    view: V1OrganView,
    materials: dict[str, Usd.Prim],
) -> None:
    visuals = UsdGeom.Scope.Define(stage, f"{group_path}/Visuals").GetPrim()
    _tag_entity(
        visuals,
        entity_kind="geometry",
        node_id=view.node.id,
        geometry_role="leaf_group",
    )
    _make_leaf(stage, f"{group_path}/Visuals", legacy_leaf_view(view), 0.0, materials)


def _fruit_index(primitive_id: str) -> int | None:
    match = re.search(r":fruit:(\d+)$", primitive_id)
    return None if match is None else int(match.group(1))


def _render_fruits(
    stage: Usd.Stage,
    group_path: str,
    view: V1OrganView,
    materials: dict[str, Usd.Prim],
) -> None:
    visuals = UsdGeom.Scope.Define(stage, f"{group_path}/Visuals").GetPrim()
    _tag_entity(
        visuals,
        entity_kind="geometry",
        node_id=view.node.id,
        geometry_role="fruit_group",
    )
    for axis in view.axes:
        _define_cylinder(
            stage,
            f"{group_path}/{_token(axis.id)}",
            length=axis.length,
            radius=axis.radius,
            frame=axis.local_frame,
            material=materials["pedicel"],
            node_id=view.node.id,
            role=axis.role,
        )

    properties = view.organ.properties
    if not isinstance(properties, FruitsProperties):
        raise TypeError("Fruits organ has incompatible canonical properties")
    ages = properties.fruit_degree_days or ()
    for sphere in view.spheres:
        index = _fruit_index(sphere.id)
        age = ages[index] if index is not None and index < len(ages) else 0.0
        material = (
            materials["fruit_ripe"]
            if age >= properties.ripening_degree_days
            else materials["fruit_unripe"]
        )
        primitive = _make_sphere(
            stage,
            f"{group_path}/{_token(sphere.id)}",
            sphere.radius,
            *sphere.local_center,
        )
        _bind_material(primitive, material)
        _tag_entity(
            primitive.GetPrim(),
            entity_kind="geometry",
            node_id=view.node.id,
            geometry_role="fruit",
        )


def _render_root(
    stage: Usd.Stage,
    group_path: str,
    view: V1OrganView,
    materials: dict[str, Usd.Prim],
) -> None:
    sphere = _make_sphere(
        stage,
        f"{group_path}/RootMarker",
        ROOT_SPHERE_RADIUS,
        0.0,
        0.0,
        -ROOT_SPHERE_RADIUS,
    )
    _bind_material(sphere, materials["root"])
    _tag_entity(
        sphere.GetPrim(),
        entity_kind="geometry",
        node_id=view.node.id,
        geometry_role="root_marker",
    )


def export_plant_usd(state: PlantState, output_path: str | Path) -> Path:
    """Render one canonical PlantState as a complete static V1 USDA stage."""

    view = build_v1_render_view(state)
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(destination))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    plant_path = f"/Plant_{state.metadata.plant_id}"
    plant = UsdGeom.Xform.Define(stage, plant_path).GetPrim()
    stage.SetDefaultPrim(plant)
    _set_string(plant, "autotom:plantStateSchema", state.schema_version)
    _set_string(plant, "autotom:renderer", "exporterV1/plant_state")
    plant.CreateAttribute("autotom:plantId", Sdf.ValueTypeNames.Int, custom=True).Set(
        state.metadata.plant_id
    )
    if state.metadata.simulation_time is not None:
        plant.CreateAttribute(
            "autotom:simulationTime", Sdf.ValueTypeNames.Double, custom=True
        ).Set(float(state.metadata.simulation_time))

    materials = _materials(stage, plant_path)
    _author_topology(stage, plant_path, state)
    geometry_root = f"{plant_path}/Geometry"
    UsdGeom.Scope.Define(stage, geometry_root)

    for organ_view in view.organs:
        group_path, _ = _organ_group(stage, geometry_root, organ_view)
        if not organ_view.render_geometry:
            continue
        organ_type = organ_view.organ.organ_type
        if organ_type == "Internode":
            _render_internode(stage, group_path, organ_view, materials)
        elif organ_type == "Leaf":
            _render_leaf(stage, group_path, organ_view, materials)
        elif organ_type == "Fruits":
            _render_fruits(stage, group_path, organ_view, materials)
        elif organ_type == "Root":
            _render_root(stage, group_path, organ_view, materials)

    stage.GetRootLayer().Save()
    manifest = audit_v1_stage(state, destination)
    save_v1_manifest(manifest, manifest_path_for(destination))
    if manifest.errors:
        raise V1AuditError("; ".join(manifest.errors))
    return destination
