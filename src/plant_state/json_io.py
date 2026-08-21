"""Deterministic JSON persistence for :mod:`plant_state`."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from .models import (
    AxisGeometry,
    CommonOrganProperties,
    FruitsProperties,
    InternodeProperties,
    LeafProperties,
    Matrix4,
    MeristemProperties,
    NodePose,
    OrganRecord,
    PlantBaseProperties,
    PlantEdge,
    PlantMetadata,
    PlantNode,
    PlantState,
    RootProperties,
    SphereGeometry,
    TrussProperties,
    TurtleOperation,
    Vector3,
)
from .schema import PLANT_STATE_SCHEMA_VERSION
from .validation import validate_plant_state


class PlantStateSchemaError(ValueError):
    """Raised when JSON does not match the declared canonical schema."""


def _mapping(value: Any, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlantStateSchemaError(f"{path} must be an object")
    return value


def _sequence(value: Any, path: str) -> list[Any]:
    if not isinstance(value, list):
        raise PlantStateSchemaError(f"{path} must be an array")
    return value


def _exact_keys(value: Mapping[str, Any], expected: set[str], path: str) -> None:
    actual = set(value)
    missing = sorted(expected - actual)
    unknown = sorted(actual - expected)
    if missing or unknown:
        detail = []
        if missing:
            detail.append(f"missing={missing}")
        if unknown:
            detail.append(f"unknown={unknown}")
        raise PlantStateSchemaError(f"{path} has invalid fields ({', '.join(detail)})")


def _vector(value: Any, path: str) -> Vector3:
    items = _sequence(value, path)
    if len(items) != 3:
        raise PlantStateSchemaError(f"{path} must contain three numbers")
    return tuple(float(item) for item in items)  # type: ignore[return-value]


def _matrix(value: Any, path: str) -> Matrix4:
    rows = _sequence(value, path)
    if len(rows) != 4 or any(not isinstance(row, list) or len(row) != 4 for row in rows):
        raise PlantStateSchemaError(f"{path} must be a 4x4 array")
    return tuple(tuple(float(item) for item in row) for row in rows)  # type: ignore[return-value]


def _float_tuple(value: Any, path: str) -> tuple[float, ...] | None:
    if value is None:
        return None
    return tuple(float(item) for item in _sequence(value, path))


def _boolean(value: Any, path: str) -> bool:
    if not isinstance(value, bool):
        raise PlantStateSchemaError(f"{path} must be a boolean")
    return value


def _metadata(value: Any) -> PlantMetadata:
    raw = _mapping(value, "metadata")
    fields = {
        "simulation_time", "plant_id", "source", "source_model",
        "source_project_sha256", "units", "conventions",
    }
    _exact_keys(raw, fields, "metadata")
    units = _mapping(raw["units"], "metadata.units")
    conventions = _mapping(raw["conventions"], "metadata.conventions")
    return PlantMetadata(
        simulation_time=raw["simulation_time"],
        plant_id=int(raw["plant_id"]),
        source=str(raw["source"]),
        source_model=None if raw["source_model"] is None else str(raw["source_model"]),
        source_project_sha256=(
            None
            if raw["source_project_sha256"] is None
            else str(raw["source_project_sha256"])
        ),
        units={str(key): str(item) for key, item in units.items()},
        conventions={str(key): str(item) for key, item in conventions.items()},
    )


def _pose(value: Any, path: str) -> NodePose:
    raw = _mapping(value, path)
    fields = {
        "incoming_world", "outgoing_world", "local_effect", "world_start",
        "world_end", "orientation_source",
    }
    _exact_keys(raw, fields, path)
    return NodePose(
        incoming_world=_matrix(raw["incoming_world"], f"{path}.incoming_world"),
        outgoing_world=_matrix(raw["outgoing_world"], f"{path}.outgoing_world"),
        local_effect=_matrix(raw["local_effect"], f"{path}.local_effect"),
        world_start=_vector(raw["world_start"], f"{path}.world_start"),
        world_end=_vector(raw["world_end"], f"{path}.world_end"),
        orientation_source=str(raw["orientation_source"]),
    )


def _node(value: Any, index: int) -> PlantNode:
    path = f"nodes[{index}]"
    raw = _mapping(value, path)
    fields = {
        "id", "groimp_node_id", "source_type", "category", "parent_id",
        "incoming_edge_kind", "incoming_edge_raw_code", "pose", "source_attributes",
    }
    _exact_keys(raw, fields, path)
    attributes = _mapping(raw["source_attributes"], f"{path}.source_attributes")
    return PlantNode(
        id=str(raw["id"]),
        groimp_node_id=int(raw["groimp_node_id"]),
        source_type=str(raw["source_type"]),
        category=str(raw["category"]),
        parent_id=None if raw["parent_id"] is None else str(raw["parent_id"]),
        incoming_edge_kind=(
            None if raw["incoming_edge_kind"] is None else str(raw["incoming_edge_kind"])
        ),
        incoming_edge_raw_code=(
            None
            if raw["incoming_edge_raw_code"] is None
            else int(raw["incoming_edge_raw_code"])
        ),
        pose=_pose(raw["pose"], f"{path}.pose"),
        source_attributes=dict(attributes),
    )


def _edge(value: Any, index: int) -> PlantEdge:
    path = f"edges[{index}]"
    raw = _mapping(value, path)
    _exact_keys(raw, {"source", "target", "kind", "raw_code"}, path)
    return PlantEdge(
        source=str(raw["source"]),
        target=str(raw["target"]),
        kind=str(raw["kind"]),
        raw_code=int(raw["raw_code"]),
    )


def _common(value: Any, path: str) -> CommonOrganProperties:
    raw = _mapping(value, path)
    fields = {
        "plant_id", "rank", "order", "parent_rank", "age_days",
        "age_degree_days", "declared_length", "area", "dry_biomass",
        "is_fruit", "is_root", "is_stem_truss",
    }
    _exact_keys(raw, fields, path)

    def integer(name: str) -> int | None:
        return None if raw[name] is None else int(raw[name])

    def number(name: str) -> float | None:
        return None if raw[name] is None else float(raw[name])

    def boolean(name: str) -> bool | None:
        return None if raw[name] is None else _boolean(raw[name], f"{path}.{name}")

    return CommonOrganProperties(
        plant_id=int(raw["plant_id"]),
        rank=integer("rank"),
        order=integer("order"),
        parent_rank=integer("parent_rank"),
        age_days=integer("age_days"),
        age_degree_days=number("age_degree_days"),
        declared_length=number("declared_length"),
        area=number("area"),
        dry_biomass=number("dry_biomass"),
        is_fruit=boolean("is_fruit"),
        is_root=boolean("is_root"),
        is_stem_truss=boolean("is_stem_truss"),
    )


_PROPERTY_FIELDS: dict[str, set[str]] = {
    "PlantBase": {"row", "position", "age_days", "age_degree_days", "initial_angle", "internode_count", "leaf_area"},
    "Root": set(),
    "Internode": {"diameter", "length_increment_daily", "effective_length", "effective_length_source"},
    "Leaf": {
        "blade_count", "petiole_length", "petiole_diameter", "petiolule_diameter",
        "rachis_diameter", "petiole_angle", "petiole_azimuth", "curvature",
        "blade_area_total", "rachis_segment_lengths", "petiolule_lengths",
        "blade_areas", "petiolule_inclinations", "segment_azimuths",
    },
    "Truss": set(),
    "Fruits": {
        "fruit_count", "paired", "pedicel_length", "rachis_segment_length",
        "fruit_radii", "fruit_degree_days", "rachis_bend_angle", "rachis_radius",
        "fruit_spacing_angle", "ripening_degree_days",
    },
    "Meristem": {"has_auxiliary_bud", "has_truss_bud"},
}


def _properties(organ_type: str, value: Any, path: str):
    raw = _mapping(value, path)
    if organ_type not in _PROPERTY_FIELDS:
        raise PlantStateSchemaError(f"{path} has unsupported organ type {organ_type!r}")
    _exact_keys(raw, _PROPERTY_FIELDS[organ_type], path)
    if organ_type == "PlantBase":
        return PlantBaseProperties(
            row=int(raw["row"]), position=int(raw["position"]),
            age_days=int(raw["age_days"]), age_degree_days=float(raw["age_degree_days"]),
            initial_angle=float(raw["initial_angle"]),
            internode_count=None if raw["internode_count"] is None else float(raw["internode_count"]),
            leaf_area=None if raw["leaf_area"] is None else float(raw["leaf_area"]),
        )
    if organ_type == "Root":
        return RootProperties()
    if organ_type == "Internode":
        return InternodeProperties(
            diameter=float(raw["diameter"]),
            length_increment_daily=(None if raw["length_increment_daily"] is None else float(raw["length_increment_daily"])),
            effective_length=float(raw["effective_length"]),
            effective_length_source=str(raw["effective_length_source"]),
        )
    if organ_type == "Leaf":
        return LeafProperties(
            blade_count=int(raw["blade_count"]), petiole_length=float(raw["petiole_length"]),
            petiole_diameter=float(raw["petiole_diameter"]), petiolule_diameter=float(raw["petiolule_diameter"]),
            rachis_diameter=float(raw["rachis_diameter"]), petiole_angle=float(raw["petiole_angle"]),
            petiole_azimuth=float(raw["petiole_azimuth"]), curvature=float(raw["curvature"]),
            blade_area_total=float(raw["blade_area_total"]),
            rachis_segment_lengths=_float_tuple(raw["rachis_segment_lengths"], f"{path}.rachis_segment_lengths"),
            petiolule_lengths=_float_tuple(raw["petiolule_lengths"], f"{path}.petiolule_lengths"),
            blade_areas=_float_tuple(raw["blade_areas"], f"{path}.blade_areas"),
            petiolule_inclinations=_float_tuple(raw["petiolule_inclinations"], f"{path}.petiolule_inclinations"),
            segment_azimuths=_float_tuple(raw["segment_azimuths"], f"{path}.segment_azimuths"),
        )
    if organ_type == "Truss":
        return TrussProperties()
    if organ_type == "Fruits":
        return FruitsProperties(
            fruit_count=int(raw["fruit_count"]), paired=_boolean(raw["paired"], f"{path}.paired"),
            pedicel_length=float(raw["pedicel_length"]),
            rachis_segment_length=float(raw["rachis_segment_length"]),
            fruit_radii=_float_tuple(raw["fruit_radii"], f"{path}.fruit_radii"),
            fruit_degree_days=_float_tuple(raw["fruit_degree_days"], f"{path}.fruit_degree_days"),
            rachis_bend_angle=float(raw["rachis_bend_angle"]),
            rachis_radius=float(raw["rachis_radius"]),
            fruit_spacing_angle=float(raw["fruit_spacing_angle"]),
            ripening_degree_days=float(raw["ripening_degree_days"]),
        )
    return MeristemProperties(
        has_auxiliary_bud=_boolean(raw["has_auxiliary_bud"], f"{path}.has_auxiliary_bud"),
        has_truss_bud=_boolean(raw["has_truss_bud"], f"{path}.has_truss_bud"),
    )


def _organ(value: Any, index: int) -> OrganRecord:
    path = f"organs[{index}]"
    raw = _mapping(value, path)
    fields = {"id", "node_id", "organ_type", "common", "properties", "primitive_ids", "attribute_source"}
    _exact_keys(raw, fields, path)
    organ_type = str(raw["organ_type"])
    return OrganRecord(
        id=str(raw["id"]), node_id=str(raw["node_id"]), organ_type=organ_type,
        common=_common(raw["common"], f"{path}.common"),
        properties=_properties(organ_type, raw["properties"], f"{path}.properties"),
        primitive_ids=tuple(str(item) for item in _sequence(raw["primitive_ids"], f"{path}.primitive_ids")),
        attribute_source=str(raw["attribute_source"]),
    )


def _turtle(value: Any, index: int) -> TurtleOperation:
    path = f"turtle_operations[{index}]"
    raw = _mapping(value, path)
    fields = {"id", "node_id", "operation", "parameters", "local_transform", "provenance"}
    _exact_keys(raw, fields, path)
    parameters = _mapping(raw["parameters"], f"{path}.parameters")
    return TurtleOperation(
        id=str(raw["id"]), node_id=str(raw["node_id"]), operation=str(raw["operation"]),
        parameters={str(key): float(item) for key, item in parameters.items()},
        local_transform=_matrix(raw["local_transform"], f"{path}.local_transform"),
        provenance=str(raw["provenance"]),
    )


def _axis(value: Any, index: int) -> AxisGeometry:
    path = f"axes[{index}]"
    raw = _mapping(value, path)
    fields = {
        "id", "owner_node_id", "organ_type", "role", "local_frame", "world_frame",
        "local_start", "local_end", "world_start", "world_end", "local_direction",
        "world_direction", "length", "radius", "length_source", "geometry_source",
    }
    _exact_keys(raw, fields, path)
    return AxisGeometry(
        id=str(raw["id"]), owner_node_id=str(raw["owner_node_id"]),
        organ_type=str(raw["organ_type"]), role=str(raw["role"]),
        local_frame=_matrix(raw["local_frame"], f"{path}.local_frame"),
        world_frame=_matrix(raw["world_frame"], f"{path}.world_frame"),
        local_start=_vector(raw["local_start"], f"{path}.local_start"),
        local_end=_vector(raw["local_end"], f"{path}.local_end"),
        world_start=_vector(raw["world_start"], f"{path}.world_start"),
        world_end=_vector(raw["world_end"], f"{path}.world_end"),
        local_direction=_vector(raw["local_direction"], f"{path}.local_direction"),
        world_direction=_vector(raw["world_direction"], f"{path}.world_direction"),
        length=float(raw["length"]), radius=float(raw["radius"]),
        length_source=str(raw["length_source"]), geometry_source=str(raw["geometry_source"]),
    )


def _sphere(value: Any, index: int) -> SphereGeometry:
    path = f"spheres[{index}]"
    raw = _mapping(value, path)
    fields = {
        "id", "owner_node_id", "organ_type", "role", "local_frame", "world_frame",
        "local_center", "world_center", "radius", "geometry_source",
    }
    _exact_keys(raw, fields, path)
    return SphereGeometry(
        id=str(raw["id"]), owner_node_id=str(raw["owner_node_id"]),
        organ_type=str(raw["organ_type"]), role=str(raw["role"]),
        local_frame=_matrix(raw["local_frame"], f"{path}.local_frame"),
        world_frame=_matrix(raw["world_frame"], f"{path}.world_frame"),
        local_center=_vector(raw["local_center"], f"{path}.local_center"),
        world_center=_vector(raw["world_center"], f"{path}.world_center"),
        radius=float(raw["radius"]), geometry_source=str(raw["geometry_source"]),
    )


def plant_state_from_dict(payload: Mapping[str, Any], *, validate: bool = True) -> PlantState:
    """Construct a state from the exact ``plant_state/1.0`` wire shape."""

    fields = {
        "schema_version", "metadata", "root_node_id", "nodes", "edges", "organs",
        "turtle_operations", "axes", "spheres", "diagnostics",
    }
    _exact_keys(payload, fields, "PlantState")
    version = str(payload["schema_version"])
    if version != PLANT_STATE_SCHEMA_VERSION:
        raise PlantStateSchemaError(
            f"Unsupported PlantState schema {version!r}; expected {PLANT_STATE_SCHEMA_VERSION!r}"
        )
    diagnostics = _mapping(payload["diagnostics"], "diagnostics")
    state = PlantState(
        metadata=_metadata(payload["metadata"]),
        root_node_id=str(payload["root_node_id"]),
        nodes=tuple(_node(item, index) for index, item in enumerate(_sequence(payload["nodes"], "nodes"))),
        edges=tuple(_edge(item, index) for index, item in enumerate(_sequence(payload["edges"], "edges"))),
        organs=tuple(_organ(item, index) for index, item in enumerate(_sequence(payload["organs"], "organs"))),
        turtle_operations=tuple(_turtle(item, index) for index, item in enumerate(_sequence(payload["turtle_operations"], "turtle_operations"))),
        axes=tuple(_axis(item, index) for index, item in enumerate(_sequence(payload["axes"], "axes"))),
        spheres=tuple(_sphere(item, index) for index, item in enumerate(_sequence(payload["spheres"], "spheres"))),
        diagnostics=dict(diagnostics),
        schema_version=version,
    )
    if validate:
        validate_plant_state(state)
    return state


def save_plant_state(state: PlantState, path: str | Path) -> Path:
    """Validate and write strict, deterministic JSON with a final newline."""

    validate_plant_state(state)
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(state.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination


def load_plant_state(path: str | Path, *, validate: bool = True) -> PlantState:
    """Load an exact ``plant_state/1.0`` document without GroIMP dependencies."""

    source = Path(path).expanduser().resolve()
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise PlantStateSchemaError(f"Invalid PlantState JSON in {source}: {exc}") from exc
    return plant_state_from_dict(_mapping(payload, "PlantState"), validate=validate)
