"""Declarative XL queries used to enrich raw ProjectGraph nodes."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
import math
from typing import Any

from .client import GroIMPRequestError, run_json_call
from .models import GraphEdge, GraphNode, GroIMPGraphSnapshot, WorldAnchor


RECORD_PREFIX = "__AUTOTOM_INSPECT__"
TIME_PREFIX = "__AUTOTOM_TIME__"
EDGE_KINDS = {256: "successor", 512: "branch"}


@dataclass(frozen=True)
class FieldSpec:
    name: str
    kind: str
    required: bool = True


ORGAN_FIELDS = (
    FieldSpec("plant_number", "int"),
    FieldSpec("organ_type", "int"),
    FieldSpec("order", "int"),
    FieldSpec("rank", "int"),
    FieldSpec("parent_rank", "int"),
    FieldSpec("isFruit", "bool"),
    FieldSpec("isRoot", "bool"),
    FieldSpec("isStemTruss", "bool"),
    FieldSpec("age_in_days_d", "int"),
    FieldSpec("age_in_degree_days_dd", "float"),
    FieldSpec("length", "float"),
)

GROWING_ORGAN_FIELDS = ORGAN_FIELDS + (
    FieldSpec("area_m2", "float"),
    FieldSpec("dry_biomass_mg", "float"),
)


TYPE_FIELDS: dict[str, tuple[FieldSpec, ...]] = {
    "organs.Root": GROWING_ORGAN_FIELDS,
    "organs.Internode": GROWING_ORGAN_FIELDS
    + (
        FieldSpec("internode_width_m", "float"),
        FieldSpec("length_increment_daily_m", "float", required=False),
    ),
    "organs.Leaf": GROWING_ORGAN_FIELDS
    + (
        FieldSpec("bladesNr", "int"),
        FieldSpec("lengthPetiole", "float"),
        FieldSpec("diameterPetiole", "float"),
        FieldSpec("diameterPetiolule", "float"),
        FieldSpec("diameterSegment", "float"),
        FieldSpec("anglePetiole", "float"),
        FieldSpec("counterClocKWiseOrientationPetiole", "float"),
        FieldSpec("leafCurvature", "float"),
        FieldSpec("area_m2bladesTotal", "float"),
        FieldSpec("segmentsLength", "float_array"),
        FieldSpec("lengthPetiolules", "float_array"),
        FieldSpec("area_m2blades", "float_array"),
        FieldSpec("inclinationOnSegmentsPetiolules", "float_array"),
        FieldSpec("counterClocKWiseOrientationSegments", "float_array"),
    ),
    "organs.Truss": GROWING_ORGAN_FIELDS,
    "organs.Fruits": GROWING_ORGAN_FIELDS
    + (
        FieldSpec("fruitPairing", "bool"),
        FieldSpec("fruitNr", "int"),
        FieldSpec("PETIOLELENGTH", "float"),
        FieldSpec("INTERNODETRUSSLENGTH", "float"),
        FieldSpec("fruitRadius", "float_array"),
        FieldSpec("degreeDaysStorage", "float_array"),
        FieldSpec("internodeTrussAngle", "float"),
        FieldSpec("internodeTrussdiameter", "float"),
        FieldSpec("angleAmongSubsequentFruits", "float"),
        FieldSpec("Ripening_dd", "float"),
    ),
    "organs.Meristem": ORGAN_FIELDS
    + (
        FieldSpec("has_already_auxiliary_bud", "bool"),
        FieldSpec("has_already_truss_bud", "bool"),
    ),
    "plant_level.PlantBase": (
        FieldSpec("plant_number", "int"),
        FieldSpec("row", "int"),
        FieldSpec("pos", "int"),
        FieldSpec("age_in_days_d", "int"),
        FieldSpec("age_in_degree_days_dd", "float"),
        FieldSpec("initialAngle", "float"),
        FieldSpec("nr_internodes", "float", required=False),
        FieldSpec("leafArea", "float", required=False),
    ),
    "de.grogra.turtle.RH": (FieldSpec("angle", "float"),),
    "de.grogra.turtle.RL": (FieldSpec("angle", "float"),),
    "de.grogra.turtle.RU": (FieldSpec("angle", "float"),),
    "de.grogra.turtle.RG": (),
    "de.grogra.turtle.Translate": (
        FieldSpec("translateX", "float"),
        FieldSpec("translateY", "float"),
        FieldSpec("translateZ", "float"),
    ),
}


def _xl_value_expression(variable: str, field: FieldSpec) -> str:
    # Attribute access through [] is the reliable XL form for inherited module
    # fields. Direct Java-style access can silently yield no console output for
    # types such as Root even though the field exists.
    value = f"{variable}[{field.name}]"
    if field.kind == "float_array":
        # XL needs the explicit cast to select Arrays.toString(float[]).
        # Arrays.toString is also null-safe and returns the literal "null".
        return f"java.util.Arrays.toString((float[]) {value})"
    return value


def build_attribute_query(node_type: str, field: FieldSpec) -> str:
    """Build one narrow query so optional-field failures remain isolated."""

    value = _xl_value_expression("n", field)
    return (
        f"for ({node_type} n : (* {node_type} *)) "
        "{ "
        f'println("{RECORD_PREFIX}\\t" + n.getId() + "\\t" + {value}); '
        "}"
    )


def build_anchor_query(node_type: str) -> str:
    return (
        f"for ({node_type} n : (* {node_type} *)) "
        "{ "
        f'println("{RECORD_PREFIX}\\t" + n.getId() + "\\t" '
        '+ location(n).x + "\\t" + location(n).y + "\\t" + location(n).z '
        '+ "\\t" + direction(n).x + "\\t" + direction(n).y + "\\t" '
        "+ direction(n).z); "
        "}"
    )


def coerce_value(raw_value: str, kind: str) -> Any:
    """Convert an XL console scalar into its JSON-friendly Python value."""

    value = raw_value.strip()
    if kind == "int":
        return int(float(value))
    if kind == "float":
        result = float(value)
        return result if math.isfinite(result) else value
    if kind == "bool":
        lowered = value.lower()
        if lowered not in {"true", "false"}:
            raise ValueError(f"Invalid boolean value: {raw_value!r}")
        return lowered == "true"
    if kind == "float_array":
        if value == "null":
            return None
        if value in {"", "0", "[]"}:
            return []
        if value.startswith("[") and value.endswith("]"):
            value = value[1:-1].strip()
            if not value:
                return []
            return [float(item.strip()) for item in value.split(",")]
        return [float(item) for item in value.split("_")]
    if kind == "str":
        return value
    raise ValueError(f"Unsupported XL field kind: {kind}")


def parse_attribute_lines(lines: list[Any], kind: str) -> dict[int, Any]:
    values: dict[int, Any] = {}
    for raw_line in lines:
        line = str(raw_line).strip()
        if not line.startswith(RECORD_PREFIX + "\t"):
            continue
        parts = line.split("\t", 2)
        if len(parts) != 3:
            continue
        values[int(parts[1])] = coerce_value(parts[2], kind)
    return values


def parse_anchor_lines(lines: list[Any]) -> dict[int, WorldAnchor]:
    anchors: dict[int, WorldAnchor] = {}
    for raw_line in lines:
        line = str(raw_line).strip()
        if not line.startswith(RECORD_PREFIX + "\t"):
            continue
        parts = line.split("\t")
        if len(parts) != 8:
            continue
        values = tuple(float(item) for item in parts[2:])
        anchors[int(parts[1])] = WorldAnchor(
            position=values[0:3],
            direction=values[3:6],
        )
    return anchors


def _parse_raw_edge(raw_edge: Any) -> GraphEdge:
    if isinstance(raw_edge, (list, tuple)) and len(raw_edge) >= 3:
        source, target, raw_code = raw_edge[:3]
    elif isinstance(raw_edge, dict):
        source = raw_edge.get("source", raw_edge.get("from"))
        target = raw_edge.get("target", raw_edge.get("to"))
        raw_code = raw_edge.get("type", raw_edge.get("edgeType"))
    else:
        raise ValueError(f"Unsupported ProjectGraph edge: {raw_edge!r}")
    code = int(raw_code)
    return GraphEdge(
        source=int(source),
        target=int(target),
        kind=EDGE_KINDS.get(code, "unknown"),
        raw_code=code,
    )


def parse_project_graph(raw_graph: dict[str, Any]) -> GroIMPGraphSnapshot:
    """Parse GroPy's ProjectGraph JSON without discarding unknown nodes/edges."""

    nodes = sorted(
        (
            GraphNode(id=int(raw_node["id"]), type=str(raw_node.get("type", "UNKNOWN")))
            for raw_node in raw_graph.get("projectgraphNodes", [])
        ),
        key=lambda node: node.id,
    )
    edges = sorted(
        (_parse_raw_edge(raw_edge) for raw_edge in raw_graph.get("projectgraphEdges", [])),
        key=lambda edge: (edge.source, edge.target, edge.raw_code),
    )
    counts = dict(sorted(Counter(node.type for node in nodes).items()))
    unknown_codes = sorted({edge.raw_code for edge in edges if edge.kind == "unknown"})
    return GroIMPGraphSnapshot(
        root_id=(
            int(raw_graph["projectgraphRoot"])
            if raw_graph.get("projectgraphRoot") is not None
            else None
        ),
        nodes=nodes,
        edges=edges,
        counts_by_type=counts,
        diagnostics={"unknown_edge_codes": unknown_codes},
    )


def _run_xl_query(workbench: Any, query: str, operation: str) -> dict[str, Any]:
    return run_json_call(workbench.runXLQuery(query), operation=operation)


def enrich_snapshot(workbench: Any, snapshot: GroIMPGraphSnapshot) -> None:
    """Populate configured attributes and direct GroIMP world anchors in place."""

    nodes_by_id = {node.id: node for node in snapshot.nodes}
    present_types = set(snapshot.counts_by_type)
    missing_optional: list[dict[str, Any]] = []
    queried_types: list[str] = []

    for node_type, fields in TYPE_FIELDS.items():
        if node_type not in present_types:
            continue
        queried_types.append(node_type)
        for field in fields:
            query = build_attribute_query(node_type, field)
            try:
                payload = _run_xl_query(
                    workbench,
                    query,
                    operation=f"XL query {node_type}.{field.name}",
                )
                values = parse_attribute_lines(payload.get("console", []), field.kind)
            except (GroIMPRequestError, ValueError) as exc:
                if field.required:
                    raise GroIMPRequestError(
                        f"Required GroIMP field {node_type}.{field.name} could not be extracted: {exc}"
                    ) from exc
                missing_optional.append(
                    {"node_type": node_type, "field": field.name, "error": str(exc)}
                )
                for node in snapshot.nodes:
                    if node.type == node_type:
                        node.attributes[field.name] = None
                continue

            expected_ids = {node.id for node in snapshot.nodes if node.type == node_type}
            missing_ids = expected_ids - values.keys()
            if missing_ids:
                if field.required:
                    raise GroIMPRequestError(
                        f"Required field {node_type}.{field.name} missing for node IDs "
                        f"{sorted(missing_ids)}"
                    )
                missing_optional.append(
                    {
                        "node_type": node_type,
                        "field": field.name,
                        "missing_node_ids": sorted(missing_ids),
                    }
                )
            for node_id in expected_ids:
                nodes_by_id[node_id].attributes[field.name] = values.get(node_id)

        anchor_payload = _run_xl_query(
            workbench,
            build_anchor_query(node_type),
            operation=f"XL world anchor query {node_type}",
        )
        anchors = parse_anchor_lines(anchor_payload.get("console", []))
        expected_ids = {node.id for node in snapshot.nodes if node.type == node_type}
        missing_anchors = expected_ids - anchors.keys()
        if missing_anchors:
            raise GroIMPRequestError(
                f"GroIMP world anchor missing for {node_type} node IDs {sorted(missing_anchors)}"
            )
        for node_id, anchor in anchors.items():
            nodes_by_id[node_id].world_anchor = anchor

    for node in snapshot.nodes:
        node.attributes = dict(sorted(node.attributes.items()))

    snapshot.diagnostics.update(
        {
            "queried_types": sorted(queried_types),
            "unenriched_node_types": sorted(present_types - TYPE_FIELDS.keys()),
            "missing_optional_fields": missing_optional,
        }
    )


def query_model_time(workbench: Any) -> int | None:
    """Read the model's global time counter when the project exposes it."""

    query = f'println("{TIME_PREFIX}\\t" + time);'
    try:
        payload = _run_xl_query(workbench, query, operation="XL model time query")
    except GroIMPRequestError:
        return None
    for raw_line in payload.get("console", []):
        line = str(raw_line).strip()
        if line.startswith(TIME_PREFIX + "\t"):
            return int(float(line.split("\t", 1)[1]))
    return None
