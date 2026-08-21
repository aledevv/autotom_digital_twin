"""Semantic comparison of native GroIMP, CSV, and legacy exporter outputs."""

from __future__ import annotations

import csv
from dataclasses import asdict, dataclass, field
import itertools
import json
import math
from pathlib import Path
import re
from typing import Any, Iterable, Sequence

import numpy as np

from .geometry import ReconstructedGeometry
from .models import GraphNode, GroIMPGraphSnapshot
from .turtle import TurtleResolution


COMPARISON_REPORT_SCHEMA_VERSION = "groimp_migration_comparison/1.0"
CLASSIFICATIONS = (
    "EXPECTED_IMPROVEMENT",
    "EXPECTED_SIMPLIFICATION",
    "PHYSICS_ADAPTATION",
    "UNKNOWN_DIFFERENCE",
    "LIKELY_BUG",
)
_BIOLOGICAL_TYPES = frozenset({"Root", "Internode", "Leaf", "Truss", "Fruits"})


@dataclass(frozen=True)
class RepresentationDifference:
    representation: str
    entity: str
    field: str
    expected: Any
    observed: Any
    absolute_error: float | None
    classification: str
    rationale: str


@dataclass
class MigrationComparisonReport:
    metadata: dict[str, Any]
    counts: dict[str, dict[str, int]]
    topology: dict[str, Any]
    matches: list[dict[str, Any]]
    differences: list[RepresentationDifference]
    exporter_summaries: dict[str, Any]
    diagnostics: dict[str, Any] = field(default_factory=dict)
    report_schema_version: str = COMPARISON_REPORT_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


_CSV_FIELD_MAP: dict[str, str] = {
    "plant_id": "plant_number",
    "rank": "rank",
    "order": "order",
    "age_dd": "age_in_degree_days_dd",
    "dry_biomass_mg": "dry_biomass_mg",
    "area_m2": "area_m2",
    "length": "length",
    "is_fruit": "isFruit",
    "is_root": "isRoot",
    "internode_width_m": "internode_width_m",
    "leaf_length_petiole": "lengthPetiole",
    "leaf_diameter_petiole": "diameterPetiole",
    "leaf_angle_petiole": "anglePetiole",
    "leaf_ccw_orientation": "counterClocKWiseOrientationPetiole",
    "leaf_curvature": "leafCurvature",
    "leaf_blades_nr": "bladesNr",
    "leaf_area_blades_total": "area_m2bladesTotal",
    "leaf_segments_length": "segmentsLength",
    "leaf_area_m2blades": "area_m2blades",
    "leaf_inclination_segments": "inclinationOnSegmentsPetiolules",
    "fruit_nr": "fruitNr",
    "fruit_radii": "fruitRadius",
    "fruit_age_dd": "degreeDaysStorage",
    "fruit_ripening_dd": "Ripening_dd",
    "fruit_truss_angle": "internodeTrussAngle",
}


def _simple_type(node: GraphNode) -> str:
    return node.type.rsplit(".", 1)[-1]


def _parse_csv_value(value: str) -> Any:
    stripped = value.strip()
    lowered = stripped.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    if "_" in stripped:
        return [float(part) for part in stripped.split("_") if part]
    if lowered in {"", "nan"}:
        return None
    try:
        return float(stripped)
    except ValueError:
        return stripped


def read_graph_csv(csv_path: str | Path) -> list[dict[str, Any]]:
    with Path(csv_path).open(newline="", encoding="utf-8-sig") as stream:
        return [
            {str(key).strip(): _parse_csv_value(str(value)) for key, value in row.items()}
            for row in csv.DictReader(stream, skipinitialspace=True)
        ]


def _semantic_key_node(node: GraphNode) -> tuple[int, str, int, int]:
    attributes = node.attributes
    return (
        int(attributes.get("plant_number", 0)),
        _simple_type(node),
        int(attributes.get("order", 0)),
        int(attributes.get("rank", 0)),
    )


def _semantic_key_row(row: dict[str, Any]) -> tuple[int, str, int, int]:
    return (
        int(row["plant_id"]),
        str(row["organ_class"]),
        int(row["order"]),
        int(row["rank"]),
    )


def _sequence(value: Any) -> list[float] | None:
    if value is None:
        return []
    if isinstance(value, (list, tuple)):
        return [float(item) for item in value]
    return None


def _value_error(expected: Any, observed: Any) -> float:
    expected_sequence, observed_sequence = _sequence(expected), _sequence(observed)
    if expected_sequence is not None or observed_sequence is not None:
        if expected_sequence is None or observed_sequence is None:
            return 1e12
        if len(expected_sequence) != len(observed_sequence):
            return 1e9 + abs(len(expected_sequence) - len(observed_sequence))
        return sum(abs(a - b) for a, b in zip(expected_sequence, observed_sequence))
    if isinstance(expected, bool) or isinstance(observed, bool):
        return 0.0 if bool(expected) == bool(observed) else 1.0
    try:
        return abs(float(expected) - float(observed))
    except (TypeError, ValueError):
        return 0.0 if expected == observed else 1.0


def _pair_cost(node: GraphNode, row: dict[str, Any]) -> float:
    cost = 0.0
    for csv_name, native_name in _CSV_FIELD_MAP.items():
        if native_name in node.attributes and csv_name in row:
            cost += _value_error(node.attributes[native_name], row[csv_name])
    return cost


def _minimum_assignment(
    nodes: Sequence[GraphNode], rows: Sequence[dict[str, Any]]
) -> tuple[list[tuple[GraphNode, dict[str, Any]]], list[GraphNode], list[dict[str, Any]]]:
    """Exact deterministic assignment for small duplicate semantic groups."""

    nodes = sorted(nodes, key=lambda item: item.id)
    rows = sorted(rows, key=lambda item: int(item.get("organ_index", 0)))
    pair_count = min(len(nodes), len(rows))
    if pair_count == 0:
        return [], list(nodes), list(rows)
    best: tuple[float, tuple[int, ...]] | None = None
    if max(len(nodes), len(rows)) <= 9:
        if len(nodes) <= len(rows):
            for indexes in itertools.permutations(range(len(rows)), len(nodes)):
                score = sum(_pair_cost(node, rows[index]) for node, index in zip(nodes, indexes))
                candidate = (score, indexes)
                if best is None or candidate < best:
                    best = candidate
            assert best is not None
            pairs = [(node, rows[index]) for node, index in zip(nodes, best[1])]
            used_rows = set(best[1])
            return pairs, [], [row for index, row in enumerate(rows) if index not in used_rows]
        for indexes in itertools.permutations(range(len(nodes)), len(rows)):
            score = sum(_pair_cost(nodes[index], row) for index, row in zip(indexes, rows))
            candidate = (score, indexes)
            if best is None or candidate < best:
                best = candidate
        assert best is not None
        pairs = [(nodes[index], row) for index, row in zip(best[1], rows)]
        used_nodes = set(best[1])
        return pairs, [node for index, node in enumerate(nodes) if index not in used_nodes], []

    remaining_rows = list(rows)
    pairs = []
    for node in nodes[:pair_count]:
        index = min(
            range(len(remaining_rows)),
            key=lambda item: (_pair_cost(node, remaining_rows[item]), item),
        )
        pairs.append((node, remaining_rows.pop(index)))
    return pairs, list(nodes[pair_count:]), remaining_rows


def _is_close(expected: Any, observed: Any) -> bool:
    expected_sequence, observed_sequence = _sequence(expected), _sequence(observed)
    if expected_sequence is not None or observed_sequence is not None:
        return (
            expected_sequence is not None
            and observed_sequence is not None
            and len(expected_sequence) == len(observed_sequence)
            and all(math.isclose(a, b, rel_tol=1e-7, abs_tol=1e-9) for a, b in zip(expected_sequence, observed_sequence))
        )
    if isinstance(expected, bool) or isinstance(observed, bool):
        return bool(expected) == bool(observed)
    try:
        return math.isclose(float(expected), float(observed), rel_tol=1e-7, abs_tol=1e-9)
    except (TypeError, ValueError):
        return expected == observed


def _expected_parent(node: GraphNode) -> tuple[int, str]:
    attributes = node.attributes
    node_type = _simple_type(node)
    rank = int(attributes.get("rank", 0))
    order = int(attributes.get("order", 0))
    parent_rank = int(attributes.get("parent_rank", -1))
    if node_type == "Root":
        return -1, "none"
    if node_type == "Internode":
        if parent_rank == -1 or (order == 0 and rank == 0):
            return 0, "Root"
        return (rank - 1 if order == 0 or rank > 1 else parent_rank), "Internode"
    return rank, "Internode"


def _extract_v1_geometry(usd_path: str | Path | None) -> list[dict[str, Any]]:
    if usd_path is None:
        return []
    from pxr import Gf, Usd, UsdGeom

    stage = Usd.Stage.Open(str(usd_path))
    cache = UsdGeom.XformCache()
    result: list[dict[str, Any]] = []
    for prim in stage.Traverse():
        node_attribute = prim.GetAttribute("autotom:nodeId")
        role_attribute = prim.GetAttribute("autotom:geometryRole")
        node_id = node_attribute.Get() if node_attribute else None
        geometry_role = role_attribute.Get() if role_attribute else None
        if prim.IsA(UsdGeom.Cylinder):
            cylinder = UsdGeom.Cylinder(prim)
            height = float(cylinder.GetHeightAttr().Get())
            radius = float(cylinder.GetRadiusAttr().Get())
            transform = cache.GetLocalToWorldTransform(prim)
            start = transform.Transform(Gf.Vec3d(0, 0, -height / 2.0))
            end = transform.Transform(Gf.Vec3d(0, 0, height / 2.0))
            result.append(
                {
                    "path": str(prim.GetPath()),
                    "kind": "axis",
                    "start": tuple(float(v) for v in start),
                    "end": tuple(float(v) for v in end),
                    "length": height,
                    "radius": radius,
                    "node_id": node_id,
                    "geometry_role": geometry_role,
                }
            )
        elif prim.IsA(UsdGeom.Sphere):
            sphere = UsdGeom.Sphere(prim)
            transform = cache.GetLocalToWorldTransform(prim)
            center = transform.Transform(Gf.Vec3d(0, 0, 0))
            result.append(
                {
                    "path": str(prim.GetPath()),
                    "kind": "sphere",
                    "center": tuple(float(v) for v in center),
                    "radius": float(sphere.GetRadiusAttr().Get()),
                    "node_id": node_id,
                    "geometry_role": geometry_role,
                }
            )
    return sorted(result, key=lambda item: item["path"])


def compare_representations(
    snapshot: GroIMPGraphSnapshot,
    turtle_resolution: TurtleResolution,
    csv_path: str | Path,
    *,
    geometry: ReconstructedGeometry | None = None,
    v1_usd_path: str | Path | None = None,
    v2_branches: Sequence[dict[str, Any]] = (),
    v2_terminal_bodies: Sequence[dict[str, Any]] = (),
) -> MigrationComparisonReport:
    """Compare biological values and legacy geometric representations."""

    rows = read_graph_csv(csv_path)
    nodes = [node for node in snapshot.nodes if _simple_type(node) in _BIOLOGICAL_TYPES]
    node_groups: dict[tuple[int, str, int, int], list[GraphNode]] = {}
    row_groups: dict[tuple[int, str, int, int], list[dict[str, Any]]] = {}
    for node in nodes:
        node_groups.setdefault(_semantic_key_node(node), []).append(node)
    for row in rows:
        row_groups.setdefault(_semantic_key_row(row), []).append(row)

    pairs: list[tuple[GraphNode, dict[str, Any]]] = []
    unmatched_nodes: list[GraphNode] = []
    unmatched_rows: list[dict[str, Any]] = []
    for key in sorted(set(node_groups) | set(row_groups)):
        group_pairs, missing_nodes, missing_rows = _minimum_assignment(
            node_groups.get(key, []), row_groups.get(key, [])
        )
        pairs.extend(group_pairs)
        unmatched_nodes.extend(missing_nodes)
        unmatched_rows.extend(missing_rows)

    differences: list[RepresentationDifference] = []
    matches: list[dict[str, Any]] = []
    topology_mismatches = 0
    field_comparisons: dict[str, dict[str, float | int]] = {}
    for node, row in sorted(pairs, key=lambda item: item[0].id):
        entity = (
            f"node:{node.id}/{_simple_type(node)}/o{int(row['order'])}/"
            f"r{int(row['rank'])}/i{int(row['organ_index'])}"
        )
        field_matches = 0
        for csv_name, native_name in _CSV_FIELD_MAP.items():
            if native_name not in node.attributes or csv_name not in row:
                continue
            expected, observed = node.attributes[native_name], row[csv_name]
            if isinstance(expected, (list, tuple)) or expected is None:
                if not isinstance(observed, (list, tuple)):
                    if observed is None or (float(observed) == 0.0 and not expected):
                        observed = []
                    else:
                        observed = [float(observed)]
                if expected is None:
                    expected = []
            metric = field_comparisons.setdefault(csv_name, {"count": 0, "max_absolute_error": 0.0})
            metric["count"] = int(metric["count"]) + 1
            value_error = _value_error(expected, observed)
            if value_error < 1e9:
                metric["max_absolute_error"] = max(float(metric["max_absolute_error"]), value_error)
            if _is_close(expected, observed):
                field_matches += 1
                continue
            differences.append(
                RepresentationDifference(
                    representation="graph_csv",
                    entity=entity,
                    field=csv_name,
                    expected=expected,
                    observed=observed,
                    absolute_error=_value_error(expected, observed),
                    classification="LIKELY_BUG",
                    rationale="same-run CSV differs from the native GroIMP attribute",
                )
            )
        expected_parent = _expected_parent(node)
        observed_parent = (int(row["parent_rank"]), str(row["parent_organ_class"]))
        if expected_parent != observed_parent:
            topology_mismatches += 1
            differences.append(
                RepresentationDifference(
                    representation="graph_csv",
                    entity=entity,
                    field="parent",
                    expected=expected_parent,
                    observed=observed_parent,
                    absolute_error=None,
                    classification="LIKELY_BUG",
                    rationale="same-run CSV biological parent metadata is inconsistent",
                )
            )
        matches.append(
            {
                "node_id": node.id,
                "organ_index": int(row["organ_index"]),
                "semantic_key": list(_semantic_key_node(node)),
                "matching_cost": _pair_cost(node, row),
                "equal_fields": field_matches,
            }
        )

    for node in sorted(unmatched_nodes, key=lambda item: item.id):
        differences.append(
            RepresentationDifference(
                representation="graph_csv",
                entity=f"node:{node.id}/{_simple_type(node)}",
                field="organ_presence",
                expected="present",
                observed="missing",
                absolute_error=None,
                classification="LIKELY_BUG",
                rationale="a biological native organ is absent from the same-run CSV",
            )
        )
    for row in sorted(unmatched_rows, key=lambda item: (_semantic_key_row(item), item["organ_index"])):
        differences.append(
            RepresentationDifference(
                representation="graph_csv",
                entity=f"csv:{_semantic_key_row(row)}/i{int(row['organ_index'])}",
                field="organ_presence",
                expected="missing",
                observed="present",
                absolute_error=None,
                classification="LIKELY_BUG",
                rationale="a same-run CSV organ has no native GroIMP counterpart",
            )
        )

    v1_geometry = _extract_v1_geometry(v1_usd_path)
    native_internode_nodes = [
        node for node in nodes if _simple_type(node) == "Internode"
    ]
    native_by_id = {str(node.id): node for node in native_internode_nodes}
    native_internodes = {
        (int(node.attributes["order"]), int(node.attributes["rank"])): node
        for node in native_internode_nodes
    }
    internode_pattern = re.compile(r"Internode_o(-?\d+)_r(-?\d+)$")
    for primitive in v1_geometry:
        match = internode_pattern.search(primitive["path"])
        if primitive["kind"] != "axis":
            continue
        tagged_node_id = str(primitive.get("node_id") or "").removeprefix("node:")
        node = native_by_id.get(tagged_node_id)
        if node is None and match:
            key = (int(match.group(1)), int(match.group(2)))
            node = native_internodes.get(key)
        if node is None or node.id not in turtle_resolution.poses:
            continue
        pose = turtle_resolution.poses[node.id]
        endpoint_error = max(
            float(np.linalg.norm(np.asarray(primitive["start"]) - np.asarray(pose.start_position))),
            float(np.linalg.norm(np.asarray(primitive["end"]) - np.asarray(pose.end_position))),
        )
        observed_vector = np.asarray(primitive["end"]) - np.asarray(primitive["start"])
        observed_vector /= np.linalg.norm(observed_vector)
        expected_vector = np.asarray(pose.outgoing_frame.position) - np.asarray(pose.incoming_frame.position)
        expected_vector /= np.linalg.norm(expected_vector)
        direction_error = float(
            math.degrees(
                math.acos(float(np.clip(np.dot(expected_vector, observed_vector), -1.0, 1.0)))
            )
        )
        if endpoint_error > 1e-6:
            differences.append(
                RepresentationDifference(
                    representation="exporter_v1_usd",
                    entity=primitive["path"],
                    field="world_endpoints_and_direction",
                    expected={"start": pose.start_position, "end": pose.end_position, "direction": tuple(expected_vector)},
                    observed={"start": primitive["start"], "end": primitive["end"], "direction": tuple(observed_vector), "direction_error_degrees": direction_error},
                    absolute_error=endpoint_error,
                    classification="EXPECTED_IMPROVEMENT",
                    rationale="V1 reconstructs stem placement heuristically on world Z",
                )
            )

    v1_internode_count = sum(
        item["kind"] == "axis"
        and (
            item.get("geometry_role") == "internode"
            or bool(internode_pattern.search(item["path"]))
        )
        for item in v1_geometry
    )
    if v1_geometry and v1_internode_count < len(native_internode_nodes):
        differences.append(
            RepresentationDifference(
                representation="exporter_v1_usd",
                entity="stem internodes",
                field="organ_count",
                expected=len(native_internode_nodes),
                observed=v1_internode_count,
                absolute_error=float(len(native_internode_nodes) - v1_internode_count),
                classification="EXPECTED_IMPROVEMENT",
                rationale="V1 USD is missing one or more native internode organ groups",
            )
        )

    if geometry is not None:
        csv_world_omissions = sum(
            1 for primitive in geometry.axes if primitive.role == "internode"
        )
        if csv_world_omissions:
            differences.append(
                RepresentationDifference(
                    representation="graph_csv",
                    entity="all axis-like organs",
                    field="world_orientation_and_endpoints",
                    expected=f"{len(geometry.axes)} reconstructed axes",
                    observed="not encoded",
                    absolute_error=None,
                    classification="EXPECTED_SIMPLIFICATION",
                    rationale="the legacy CSV stores local biological parameters only",
                )
            )

    if v2_branches:
        differences.append(
            RepresentationDifference(
                representation="exporter_v2_config",
                entity="branch configuration",
                field="segmentation_and_dimensions",
                expected="native organ primitives",
                observed=f"{len(v2_branches)} physics branches",
                absolute_error=None,
                classification="PHYSICS_ADAPTATION",
                rationale="V2 averages, clamps and resamples biological axes for PhysX",
            )
        )

    counts_native: dict[str, int] = {}
    counts_csv: dict[str, int] = {}
    for node in nodes:
        counts_native[_simple_type(node)] = counts_native.get(_simple_type(node), 0) + 1
    for row in rows:
        key = str(row["organ_class"])
        counts_csv[key] = counts_csv.get(key, 0) + 1
    classifications = {
        classification: sum(item.classification == classification for item in differences)
        for classification in CLASSIFICATIONS
    }
    blocking = classifications["UNKNOWN_DIFFERENCE"] + classifications["LIKELY_BUG"]
    return MigrationComparisonReport(
        metadata={
            "csv_path": str(Path(csv_path).resolve()),
            "representations": ["native_groimp", "graph_csv", "exporter_v1_usd", "exporter_v2_config"],
            "status": "passed" if blocking == 0 else "investigation_required",
        },
        counts={
            "native_biological": dict(sorted(counts_native.items())),
            "graph_csv": dict(sorted(counts_csv.items())),
            "exporter_v1_primitives": {
                "axis": sum(item["kind"] == "axis" for item in v1_geometry),
                "sphere": sum(item["kind"] == "sphere" for item in v1_geometry),
            },
            "exporter_v2": {
                "branches": len(v2_branches),
                "terminal_bodies": len(v2_terminal_bodies),
            },
        },
        topology={
            "native_structural_edges": sum(edge.kind in {"successor", "branch"} for edge in snapshot.edges),
            "csv_parent_links": sum(str(row["parent_organ_class"]) != "none" for row in rows),
            "parent_metadata_mismatches": topology_mismatches,
        },
        matches=matches,
        differences=sorted(
            differences,
            key=lambda item: (item.representation, item.entity, item.field, item.classification),
        ),
        exporter_summaries={
            "v1": {"usd_path": str(v1_usd_path) if v1_usd_path else None, "primitives": v1_geometry},
            "v2": {
                "branch_ids": sorted(str(branch.get("id", "")) for branch in v2_branches),
                "total_links": sum(int(branch.get("n_links", 0)) for branch in v2_branches),
                "terminal_body_ids": sorted(str(body.get("id", "")) for body in v2_terminal_bodies),
            },
        },
        diagnostics={
            "classifications": classifications,
            "unmatched_native_node_ids": sorted(node.id for node in unmatched_nodes),
            "unmatched_csv_rows": len(unmatched_rows),
            "duplicate_matching": "minimum scalar/vector cost; node ID and organ_index tie-break",
            "field_comparisons": dict(sorted(field_comparisons.items())),
        },
    )


def save_comparison_report(
    report: MigrationComparisonReport, output_path: str | Path
) -> Path:
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination


def save_comparison_markdown(
    report: MigrationComparisonReport, output_path: str | Path
) -> Path:
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# GroIMP migration comparison",
        "",
        f"Status: `{report.metadata['status']}`",
        "",
        "## Counts",
        "",
        "```json",
        json.dumps(report.counts, indent=2, sort_keys=True),
        "```",
        "",
        "## Difference classifications",
        "",
    ]
    for classification, count in report.diagnostics["classifications"].items():
        lines.append(f"- `{classification}`: {count}")
    lines.extend(("", "## Differences", ""))
    if not report.differences:
        lines.append("No differences.")
    for difference in report.differences:
        lines.append(
            f"- **{difference.classification}** — `{difference.representation}` / "
            f"`{difference.entity}` / `{difference.field}`: {difference.rationale}"
        )
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination
