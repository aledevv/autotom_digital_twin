"""GroIMP adapter for the exporter-independent canonical :mod:`plant_state`."""

from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import math
from pathlib import Path
import sys
from typing import Any, Mapping

import numpy as np

from plant_state import (
    AxisGeometry,
    CommonOrganProperties,
    FruitsProperties,
    InternodeProperties,
    LeafProperties,
    MeristemProperties,
    NodePose,
    OrganRecord,
    PlantBaseProperties,
    PlantEdge,
    PlantMetadata,
    PlantNode,
    PlantState,
    PlantStateValidationError,
    RootProperties,
    SphereGeometry,
    TrussProperties,
    TurtleOperation,
    save_plant_state,
    validate_plant_state,
)

from .client import GroIMPError
from .geometry import build_rendered_geometry
from .inspector import (
    DEFAULT_API_URL,
    DEFAULT_FUNCTION,
    inspect_project,
    inspect_workbench,
)
from .models import GraphNode, GroIMPGraphSnapshot
from .queries import query_model_time
from .turtle import Matrix4, TurtleResolution, resolve_turtle


_ORGAN_TYPES = frozenset(
    {"PlantBase", "Root", "Internode", "Leaf", "Truss", "Fruits", "Meristem"}
)
_BIOLOGICAL_DESCENDANT_TYPES = _ORGAN_TYPES - {"PlantBase"}
_TURTLE_TYPES = frozenset({"RH", "RL", "RU", "RG", "Translate"})
_STRUCTURAL_EDGE_KINDS = frozenset({"successor", "branch"})


class PlantExtractionError(ValueError):
    """Raised when one unambiguous canonical plant cannot be extracted."""


def _simple_type(node: GraphNode) -> str:
    return node.type.rsplit(".", 1)[-1]


def _canonical_node_id(groimp_node_id: int) -> str:
    return f"node:{groimp_node_id}"


def _matrix(value: Any) -> Matrix4:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (4, 4) or not np.isfinite(array).all():
        raise PlantExtractionError("canonical transforms must be finite 4x4 matrices")
    return tuple(tuple(float(item) for item in row) for row in array)  # type: ignore[return-value]


def _relative(parent_world: Any, child_world: Any) -> Matrix4:
    return _matrix(
        np.linalg.inv(np.asarray(parent_world, dtype=np.float64))
        @ np.asarray(child_world, dtype=np.float64)
    )


def _transform_point(matrix: Any, point: Any) -> tuple[float, float, float]:
    result = np.asarray(matrix, dtype=np.float64) @ np.array((*point, 1.0), dtype=np.float64)
    return tuple(float(item) for item in result[:3])  # type: ignore[return-value]


def _transform_direction(matrix: Any, direction: Any) -> tuple[float, float, float]:
    result = np.asarray(matrix, dtype=np.float64)[:3, :3] @ np.asarray(direction, dtype=np.float64)
    norm = float(np.linalg.norm(result))
    if norm == 0.0:
        raise PlantExtractionError("canonical geometry has a zero direction")
    return tuple(float(item / norm) for item in result)  # type: ignore[return-value]


def _descendants(snapshot: GroIMPGraphSnapshot, root_id: int) -> set[int]:
    children: dict[int, list[int]] = {}
    for edge in snapshot.edges:
        if edge.kind in _STRUCTURAL_EDGE_KINDS:
            children.setdefault(edge.source, []).append(edge.target)
    selected: set[int] = set()
    stack = [root_id]
    while stack:
        node_id = stack.pop()
        if node_id in selected:
            continue
        selected.add(node_id)
        stack.extend(sorted(children.get(node_id, []), reverse=True))
    return selected


def _select_plant_base(
    snapshot: GroIMPGraphSnapshot,
    plant_id: int,
    strict: bool,
) -> tuple[int, set[int], dict[str, Any]]:
    nodes_by_id = {node.id: node for node in snapshot.nodes}
    candidates = [
        node
        for node in snapshot.nodes
        if _simple_type(node) == "PlantBase"
        and int(node.attributes.get("plant_number", -1)) == plant_id
    ]
    if not candidates:
        raise PlantExtractionError(f"No PlantBase found for plant_id={plant_id}")

    scored: list[tuple[int, int, set[int]]] = []
    for candidate in candidates:
        subtree = _descendants(snapshot, candidate.id)
        biological_count = sum(
            _simple_type(nodes_by_id[node_id]) in _BIOLOGICAL_DESCENDANT_TYPES
            for node_id in subtree
        )
        scored.append((biological_count, candidate.id, subtree))
    best_count = max(item[0] for item in scored)
    best = sorted((item for item in scored if item[0] == best_count), key=lambda item: item[1])
    ambiguous_ids = [item[1] for item in best]
    if len(best) > 1 and strict:
        raise PlantExtractionError(
            f"Multiple equivalent PlantBase subgraphs for plant_id={plant_id}: {ambiguous_ids}"
        )
    _, selected_id, selected_nodes = best[0]
    excluded_markers = sorted(
        item[1] for item in scored if item[1] != selected_id and item[0] == 0
    )
    diagnostics = {
        "plant_base_candidates": [
            {"groimp_node_id": item[1], "biological_descendant_count": item[0]}
            for item in sorted(scored, key=lambda value: value[1])
        ],
        "selected_plant_base": selected_id,
        "excluded_marker_plant_bases": excluded_markers,
        "ambiguous_best_candidates": ambiguous_ids if len(best) > 1 else [],
    }
    return selected_id, selected_nodes, diagnostics


def _number(attributes: Mapping[str, Any], name: str) -> float:
    try:
        value = float(attributes[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise PlantExtractionError(f"required numeric organ attribute {name!r} is missing") from exc
    if not math.isfinite(value):
        raise PlantExtractionError(f"organ attribute {name!r} is not finite")
    return value


def _optional_number(attributes: Mapping[str, Any], name: str) -> float | None:
    value = attributes.get(name)
    return None if value is None else float(value)


def _optional_int(attributes: Mapping[str, Any], name: str) -> int | None:
    value = attributes.get(name)
    return None if value is None else int(value)


def _optional_bool(attributes: Mapping[str, Any], name: str) -> bool | None:
    value = attributes.get(name)
    return None if value is None else bool(value)


def _array(attributes: Mapping[str, Any], name: str) -> tuple[float, ...] | None:
    value = attributes.get(name)
    if value is None:
        return None
    if not isinstance(value, (list, tuple)):
        raise PlantExtractionError(f"organ attribute {name!r} is not an array")
    return tuple(float(item) for item in value)


def _common(node: GraphNode, plant_id: int) -> CommonOrganProperties:
    attributes = node.attributes
    return CommonOrganProperties(
        plant_id=int(attributes.get("plant_number", plant_id)),
        rank=_optional_int(attributes, "rank"),
        order=_optional_int(attributes, "order"),
        parent_rank=_optional_int(attributes, "parent_rank"),
        age_days=_optional_int(attributes, "age_in_days_d"),
        age_degree_days=_optional_number(attributes, "age_in_degree_days_dd"),
        declared_length=_optional_number(attributes, "length"),
        area=_optional_number(attributes, "area_m2"),
        dry_biomass=_optional_number(attributes, "dry_biomass_mg"),
        is_fruit=_optional_bool(attributes, "isFruit"),
        is_root=_optional_bool(attributes, "isRoot"),
        is_stem_truss=_optional_bool(attributes, "isStemTruss"),
    )


def _specific_properties(
    node: GraphNode,
    resolution: TurtleResolution,
):
    attributes = node.attributes
    organ_type = _simple_type(node)
    if organ_type == "PlantBase":
        return PlantBaseProperties(
            row=int(_number(attributes, "row")),
            position=int(_number(attributes, "pos")),
            age_days=int(_number(attributes, "age_in_days_d")),
            age_degree_days=_number(attributes, "age_in_degree_days_dd"),
            initial_angle=_number(attributes, "initialAngle"),
            internode_count=_optional_number(attributes, "nr_internodes"),
            leaf_area=_optional_number(attributes, "leafArea"),
        )
    if organ_type == "Root":
        return RootProperties()
    if organ_type == "Internode":
        pose = resolution.poses[node.id]
        effective_length = float(
            np.linalg.norm(np.asarray(pose.end_position) - np.asarray(pose.start_position))
        )
        source = (
            "groimp_anchor_calibrated"
            if pose.effect == "advance_anchor_calibrated"
            else "groimp_api"
        )
        return InternodeProperties(
            diameter=_number(attributes, "internode_width_m"),
            length_increment_daily=_optional_number(attributes, "length_increment_daily_m"),
            effective_length=effective_length,
            effective_length_source=source,
        )
    if organ_type == "Leaf":
        return LeafProperties(
            blade_count=int(_number(attributes, "bladesNr")),
            petiole_length=_number(attributes, "lengthPetiole"),
            petiole_diameter=_number(attributes, "diameterPetiole"),
            petiolule_diameter=_number(attributes, "diameterPetiolule"),
            rachis_diameter=_number(attributes, "diameterSegment"),
            petiole_angle=_number(attributes, "anglePetiole"),
            petiole_azimuth=_number(attributes, "counterClocKWiseOrientationPetiole"),
            curvature=_number(attributes, "leafCurvature"),
            blade_area_total=_number(attributes, "area_m2bladesTotal"),
            rachis_segment_lengths=_array(attributes, "segmentsLength"),
            petiolule_lengths=_array(attributes, "lengthPetiolules"),
            blade_areas=_array(attributes, "area_m2blades"),
            petiolule_inclinations=_array(attributes, "inclinationOnSegmentsPetiolules"),
            segment_azimuths=_array(attributes, "counterClocKWiseOrientationSegments"),
        )
    if organ_type == "Truss":
        return TrussProperties()
    if organ_type == "Fruits":
        return FruitsProperties(
            fruit_count=int(_number(attributes, "fruitNr")),
            paired=bool(attributes.get("fruitPairing", False)),
            pedicel_length=_number(attributes, "PETIOLELENGTH"),
            rachis_segment_length=_number(attributes, "INTERNODETRUSSLENGTH"),
            fruit_radii=_array(attributes, "fruitRadius"),
            fruit_degree_days=_array(attributes, "degreeDaysStorage"),
            rachis_bend_angle=_number(attributes, "internodeTrussAngle"),
            rachis_radius=_number(attributes, "internodeTrussdiameter"),
            fruit_spacing_angle=_number(attributes, "angleAmongSubsequentFruits"),
            ripening_degree_days=_number(attributes, "Ripening_dd"),
        )
    if organ_type == "Meristem":
        return MeristemProperties(
            has_auxiliary_bud=bool(attributes["has_already_auxiliary_bud"]),
            has_truss_bud=bool(attributes["has_already_truss_bud"]),
        )
    raise PlantExtractionError(f"unsupported organ type {organ_type}")


def _plant_metadata(
    plant_id: int,
    metadata: PlantMetadata | Mapping[str, Any] | None,
) -> PlantMetadata:
    if isinstance(metadata, PlantMetadata):
        if metadata.plant_id != plant_id:
            raise PlantExtractionError("metadata plant_id disagrees with extraction plant_id")
        return metadata
    values = dict(metadata or {})
    supported = {
        "simulation_time",
        "source",
        "source_model",
        "source_project_sha256",
        "units",
        "conventions",
    }
    unexpected = sorted(set(values) - supported)
    if unexpected:
        raise PlantExtractionError(f"unsupported canonical metadata fields: {unexpected}")
    defaults = PlantMetadata(None, plant_id)
    return PlantMetadata(
        simulation_time=values.get("simulation_time"),
        plant_id=plant_id,
        source=str(values.get("source", "groimp_api")),
        source_model=values.get("source_model"),
        source_project_sha256=values.get("source_project_sha256"),
        units=dict(values.get("units", {})) or defaults.units,
        conventions=dict(values.get("conventions", {})) or defaults.conventions,
    )


def extract_plant_state(
    snapshot: GroIMPGraphSnapshot,
    turtle_resolution: TurtleResolution,
    *,
    plant_id: int = 1,
    metadata: PlantMetadata | Mapping[str, Any] | None = None,
    strict: bool = True,
) -> PlantState:
    """Convert one validated native plant subtree into canonical data."""

    if plant_id < 1:
        raise ValueError("plant_id must be one or greater")
    selected_base, included_ids, selection_diagnostics = _select_plant_base(
        snapshot, plant_id, strict
    )
    unresolved = sorted(included_ids - set(turtle_resolution.poses))
    if unresolved:
        raise PlantExtractionError(f"selected plant has unresolved turtle nodes: {unresolved}")
    nodes_by_id = {node.id: node for node in snapshot.nodes}

    included_edges = sorted(
        (
            edge
            for edge in snapshot.edges
            if edge.source in included_ids and edge.target in included_ids
        ),
        key=lambda edge: (edge.source, edge.target, edge.raw_code),
    )
    parents = {
        edge.target: edge
        for edge in included_edges
        if edge.kind in _STRUCTURAL_EDGE_KINDS
    }
    canonical_edges = tuple(
        PlantEdge(
            source=_canonical_node_id(edge.source),
            target=_canonical_node_id(edge.target),
            kind=edge.kind,
            raw_code=edge.raw_code,
        )
        for edge in included_edges
    )

    canonical_nodes: list[PlantNode] = []
    for node_id in sorted(included_ids):
        node = nodes_by_id[node_id]
        pose = turtle_resolution.poses[node_id]
        incoming = _matrix(pose.incoming_frame.matrix)
        outgoing = _matrix(pose.outgoing_frame.matrix)
        parent = parents.get(node_id)
        node_type = _simple_type(node)
        category = (
            "plant_base"
            if node_type == "PlantBase"
            else "organ"
            if node_type in _ORGAN_TYPES
            else "turtle"
            if node_type in _TURTLE_TYPES
            else "auxiliary"
        )
        canonical_nodes.append(
            PlantNode(
                id=_canonical_node_id(node_id),
                groimp_node_id=node_id,
                source_type=node.type,
                category=category,
                parent_id=None if parent is None else _canonical_node_id(parent.source),
                incoming_edge_kind=None if parent is None else parent.kind,
                incoming_edge_raw_code=None if parent is None else parent.raw_code,
                pose=NodePose(
                    incoming_world=incoming,
                    outgoing_world=outgoing,
                    local_effect=_relative(incoming, outgoing),
                    world_start=tuple(float(item) for item in pose.start_position),
                    world_end=tuple(float(item) for item in pose.end_position),
                ),
                source_attributes=dict(sorted(node.attributes.items())),
            )
        )

    rendered = build_rendered_geometry(snapshot, turtle_resolution, strict=strict)
    canonical_axes: list[AxisGeometry] = []
    canonical_spheres: list[SphereGeometry] = []
    for axis in rendered.axes:
        if axis.source_node_id not in included_ids:
            continue
        owner_incoming = turtle_resolution.poses[axis.source_node_id].incoming_frame.matrix
        inverse_owner = np.linalg.inv(np.asarray(owner_incoming, dtype=np.float64))
        local_frame = _matrix(inverse_owner @ np.asarray(axis.frame.matrix, dtype=np.float64))
        local_start = _transform_point(inverse_owner, axis.start)
        local_end = _transform_point(inverse_owner, axis.end)
        local_direction = _transform_direction(inverse_owner, axis.direction)
        length_source = (
            "groimp_anchor_calibrated"
            if axis.role == "internode"
            and turtle_resolution.poses[axis.source_node_id].effect == "advance_anchor_calibrated"
            else "groimp_api"
        )
        geometry_source = (
            "groimp_turtle" if axis.role == "internode" else "validated_rgg_production"
        )
        canonical_axes.append(
            AxisGeometry(
                id=axis.primitive_id,
                owner_node_id=_canonical_node_id(axis.source_node_id),
                organ_type=axis.organ_type,
                role=axis.role,
                local_frame=local_frame,
                world_frame=_matrix(axis.frame.matrix),
                local_start=local_start,
                local_end=local_end,
                world_start=axis.start,
                world_end=axis.end,
                local_direction=local_direction,
                world_direction=axis.direction,
                length=axis.length,
                radius=axis.radius,
                length_source=length_source,
                geometry_source=geometry_source,
            )
        )
    for sphere in rendered.spheres:
        if sphere.source_node_id not in included_ids:
            continue
        owner_incoming = turtle_resolution.poses[sphere.source_node_id].incoming_frame.matrix
        inverse_owner = np.linalg.inv(np.asarray(owner_incoming, dtype=np.float64))
        canonical_spheres.append(
            SphereGeometry(
                id=sphere.primitive_id,
                owner_node_id=_canonical_node_id(sphere.source_node_id),
                organ_type=sphere.organ_type,
                role=sphere.role,
                local_frame=_matrix(
                    inverse_owner @ np.asarray(sphere.frame.matrix, dtype=np.float64)
                ),
                world_frame=_matrix(sphere.frame.matrix),
                local_center=_transform_point(inverse_owner, sphere.center),
                world_center=sphere.center,
                radius=sphere.radius,
                geometry_source="validated_rgg_production",
            )
        )
    canonical_axes.sort(key=lambda item: item.id)
    canonical_spheres.sort(key=lambda item: item.id)

    primitives_by_owner: dict[str, list[str]] = {}
    for primitive in [*canonical_axes, *canonical_spheres]:
        primitives_by_owner.setdefault(primitive.owner_node_id, []).append(primitive.id)

    organs: list[OrganRecord] = []
    turtle_operations: list[TurtleOperation] = []
    for node_id in sorted(included_ids):
        node = nodes_by_id[node_id]
        node_type = _simple_type(node)
        canonical_id = _canonical_node_id(node_id)
        if node_type in _ORGAN_TYPES:
            organs.append(
                OrganRecord(
                    id=f"organ:{node_id}",
                    node_id=canonical_id,
                    organ_type=node_type,
                    common=_common(node, plant_id),
                    properties=_specific_properties(node, turtle_resolution),
                    primitive_ids=tuple(sorted(primitives_by_owner.get(canonical_id, []))),
                )
            )
        if node_type in _TURTLE_TYPES:
            parameters: dict[str, float]
            if node_type in {"RH", "RL", "RU"}:
                parameters = {"angle": _number(node.attributes, "angle")}
            elif node_type == "Translate":
                parameters = {
                    "x": _number(node.attributes, "translateX"),
                    "y": _number(node.attributes, "translateY"),
                    "z": _number(node.attributes, "translateZ"),
                }
            else:
                parameters = {}
            turtle_operations.append(
                TurtleOperation(
                    id=f"turtle:{node_id}",
                    node_id=canonical_id,
                    operation=node_type,
                    parameters=parameters,
                    local_transform=_relative(
                        turtle_resolution.poses[node_id].incoming_frame.matrix,
                        turtle_resolution.poses[node_id].outgoing_frame.matrix,
                    ),
                )
            )

    included_type_counts: dict[str, int] = {}
    for node_id in included_ids:
        node_type = _simple_type(nodes_by_id[node_id])
        included_type_counts[node_type] = included_type_counts.get(node_type, 0) + 1
    diagnostics = {
        **selection_diagnostics,
        "included_node_count": len(included_ids),
        "excluded_node_count": len(snapshot.nodes) - len(included_ids),
        "included_type_counts": dict(sorted(included_type_counts.items())),
        "unsupported_passthrough_types": sorted(
            {
                nodes_by_id[node_id].type
                for node_id in included_ids
                if _simple_type(nodes_by_id[node_id]) not in _ORGAN_TYPES | _TURTLE_TYPES
            }
        ),
        "renderer_cache_offsets_applied": False,
        "leaf_surface_assets_canonicalized": False,
        "identity_scope": "stable_within_one_groimp_workbench",
        "source_turtle_diagnostics": dict(turtle_resolution.diagnostics),
        "source_geometry_diagnostics": dict(rendered.diagnostics),
    }
    state = PlantState(
        metadata=_plant_metadata(plant_id, metadata),
        root_node_id=_canonical_node_id(selected_base),
        nodes=tuple(canonical_nodes),
        edges=canonical_edges,
        organs=tuple(organs),
        turtle_operations=tuple(turtle_operations),
        axes=tuple(canonical_axes),
        spheres=tuple(canonical_spheres),
        diagnostics=diagnostics,
    )
    validation_errors = validate_plant_state(state, strict=strict)
    if validation_errors:
        state = replace(
            state,
            diagnostics={**state.diagnostics, "validation_errors": list(validation_errors)},
        )
    return state


def extract_workbench_state(
    workbench: Any,
    *,
    plant_id: int = 1,
    strict: bool = True,
) -> PlantState:
    """Extract current live state without taking ownership of the workbench."""

    snapshot = inspect_workbench(workbench)
    resolution = resolve_turtle(snapshot, strict=strict)
    return extract_plant_state(
        snapshot,
        resolution,
        plant_id=plant_id,
        metadata={"simulation_time": query_model_time(workbench)},
        strict=strict,
    )


def extract_project_state(
    project_path: str | Path,
    *,
    steps: int,
    plant_id: int = 1,
    api_url: str = DEFAULT_API_URL,
    function_name: str = DEFAULT_FUNCTION,
    strict: bool = True,
) -> PlantState:
    """Extract from a lifecycle-safe isolated project run."""

    source = Path(project_path).expanduser().resolve()
    report = inspect_project(
        source,
        api_url=api_url,
        steps=steps,
        function_name=function_name,
    )
    resolution = resolve_turtle(report.snapshot, strict=strict)
    return extract_plant_state(
        report.snapshot,
        resolution,
        plant_id=plant_id,
        metadata={
            "simulation_time": report.metadata.get("simulation_time"),
            "source_model": source.name,
            "source_project_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        },
        strict=strict,
    )


def _non_negative_int(value: str) -> int:
    result = int(value)
    if result < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return result


def _positive_int(value: str) -> int:
    result = int(value)
    if result < 1:
        raise argparse.ArgumentTypeError("must be one or greater")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Extract one canonical plant_state/1.0 JSON document from GroIMP."
    )
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--steps", type=_non_negative_int, required=True)
    parser.add_argument("--plant-id", type=_positive_int, default=1)
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    parser.add_argument("--function", dest="function_name", default=DEFAULT_FUNCTION)
    parser.add_argument("--output", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        state = extract_project_state(
            args.project,
            steps=args.steps,
            plant_id=args.plant_id,
            api_url=args.api_url,
            function_name=args.function_name,
        )
        destination = save_plant_state(state, args.output)
    except (
        FileNotFoundError,
        GroIMPError,
        PlantExtractionError,
        PlantStateValidationError,
        ValueError,
    ) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    print(
        f"[OK] PlantState: plant_id={state.metadata.plant_id}, "
        f"nodes={len(state.nodes)}, organs={len(state.organs)}, "
        f"axes={len(state.axes)}, spheres={len(state.spheres)}"
    )
    print(f"[OK] Canonical JSON saved: {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
