"""End-to-end Phase C/D validation CLI over one isolated GroIMP run."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import io
from pathlib import Path
import sys
from typing import Any

from .client import GroIMPClient, GroIMPError, run_json_call
from .comparison import (
    MigrationComparisonReport,
    compare_representations,
    save_comparison_markdown,
    save_comparison_report,
)
from .geometry import (
    GeometryValidationReport,
    ObjMesh,
    ReconstructedGeometry,
    build_rendered_geometry,
    parse_obj,
    save_debug_obj,
    save_geometry_report,
    validate_rendered_geometry,
)
from .extractor import extract_plant_state
from .inspector import (
    DEFAULT_API_URL,
    DEFAULT_FUNCTION,
    _infer_simulation_time,
    _sanitized_url,
    inspect_workbench,
    save_report,
)
from .models import InspectionReport, StepResult
from .queries import query_model_time
from .runtime import isolated_project
from .turtle import TurtleResolution, resolve_turtle


_GRAPH_CSV_FIELDS = (
    "day", "plant_id", "organ_class", "organ_index", "rank", "order",
    "parent_rank", "parent_organ_class", "age_dd", "dry_biomass_mg", "area_m2",
    "length", "is_fruit", "is_root", "internode_width_m", "leaf_length_petiole",
    "leaf_diameter_petiole", "leaf_angle_petiole", "leaf_ccw_orientation",
    "leaf_curvature", "leaf_blades_nr", "leaf_area_blades_total",
    "leaf_rachis_length", "leaf_segments_length", "leaf_area_m2blades",
    "leaf_inclination_segments", "fruit_nr", "fruit_radii", "fruit_age_dd",
    "fruit_ripening_dd", "fruit_truss_angle",
)


def _comparison_csv_bytes(snapshot, simulation_time: int) -> bytes:
    """Project the live snapshot into the legacy graph CSV wire shape.

    This is a validation artifact, not the Phase G compatibility adapter. It
    is used when an older GSZ exposes the simulation function but not the
    checkout's newer ``exportPlantGraph`` helper.
    """

    stream = io.StringIO(newline="")
    writer = csv.DictWriter(stream, fieldnames=_GRAPH_CSV_FIELDS, lineterminator="\n")
    writer.writeheader()
    counters: dict[tuple[int, int, int, str], int] = {}

    def array(value: Any) -> str:
        values = value or []
        return "0" if not values else "_".join(str(float(item)) for item in values)

    for node in sorted(snapshot.nodes, key=lambda item: item.id):
        organ_type = node.type.rsplit(".", 1)[-1]
        if organ_type not in {"Root", "Internode", "Leaf", "Truss", "Fruits"}:
            continue
        attributes = node.attributes
        plant = int(attributes["plant_number"])
        rank = int(attributes["rank"])
        order = int(attributes["order"])
        key = (plant, rank, order, organ_type)
        organ_index = counters.get(key, 0)
        counters[key] = organ_index + 1
        parent_rank = int(attributes.get("parent_rank", -1))
        if organ_type == "Root":
            csv_parent_rank, parent_class = -1, "none"
        elif organ_type == "Internode":
            if parent_rank == -1 or (order == 0 and rank == 0):
                csv_parent_rank, parent_class = 0, "Root"
            else:
                csv_parent_rank = rank - 1 if order == 0 or rank > 1 else parent_rank
                parent_class = "Internode"
        else:
            csv_parent_rank, parent_class = rank, "Internode"
        row: dict[str, Any] = {field: 0 for field in _GRAPH_CSV_FIELDS}
        row.update(
            {
                "day": simulation_time,
                "plant_id": plant,
                "organ_class": organ_type,
                "organ_index": organ_index,
                "rank": rank,
                "order": order,
                "parent_rank": csv_parent_rank,
                "parent_organ_class": parent_class,
                "age_dd": attributes.get("age_in_degree_days_dd", 0),
                "dry_biomass_mg": attributes.get("dry_biomass_mg", 0),
                "area_m2": attributes.get("area_m2", 0),
                "length": attributes.get("length", 0),
                "is_fruit": str(bool(attributes.get("isFruit", False))).lower(),
                "is_root": str(bool(attributes.get("isRoot", False))).lower(),
            }
        )
        if organ_type == "Internode":
            row["internode_width_m"] = attributes["internode_width_m"]
        elif organ_type == "Leaf":
            segments = attributes.get("segmentsLength") or []
            petiolules = attributes.get("lengthPetiolules") or []
            row.update(
                {
                    "leaf_length_petiole": attributes["lengthPetiole"],
                    "leaf_diameter_petiole": attributes["diameterPetiole"],
                    "leaf_angle_petiole": attributes["anglePetiole"],
                    "leaf_ccw_orientation": attributes["counterClocKWiseOrientationPetiole"],
                    "leaf_curvature": attributes["leafCurvature"],
                    "leaf_blades_nr": attributes["bladesNr"],
                    "leaf_area_blades_total": attributes["area_m2bladesTotal"],
                    "leaf_rachis_length": sum(segments) + sum(petiolules),
                    "leaf_segments_length": array(segments),
                    "leaf_area_m2blades": array(attributes.get("area_m2blades")),
                    "leaf_inclination_segments": array(
                        attributes.get("inclinationOnSegmentsPetiolules")
                    ),
                }
            )
        elif organ_type == "Fruits":
            row.update(
                {
                    "fruit_nr": attributes["fruitNr"],
                    "fruit_radii": array(attributes.get("fruitRadius")),
                    "fruit_age_dd": array(attributes.get("degreeDaysStorage")),
                    "fruit_ripening_dd": attributes["Ripening_dd"],
                    "fruit_truss_angle": attributes["internodeTrussAngle"],
                }
            )
        writer.writerow(row)
    return stream.getvalue().encode("utf-8")


@dataclass
class MigrationValidationBundle:
    inspection: InspectionReport
    turtle: TurtleResolution
    geometry: ReconstructedGeometry
    geometry_validation: GeometryValidationReport
    comparison: MigrationComparisonReport
    csv_name: str
    csv_capture_mode: str
    csv_bytes: bytes
    v1_usd_bytes: bytes
    groimp_obj_bytes: dict[int, bytes]
    v2_branches: list[dict[str, Any]]
    v2_terminal_bodies: list[dict[str, Any]]


def _representative_node_ids(snapshot, turtle: TurtleResolution, geometry: ReconstructedGeometry) -> list[int]:
    """Choose deterministic real-plant coverage without exporting every subtree."""

    by_role: dict[str, list[int]] = {}
    primitive_counts: dict[int, int] = {}
    for primitive in [*geometry.axes, *geometry.spheres]:
        by_role.setdefault(primitive.role, []).append(primitive.source_node_id)
        primitive_counts[primitive.source_node_id] = primitive_counts.get(primitive.source_node_id, 0) + 1
    selected: set[int] = set()
    nodes = {node.id: node for node in snapshot.nodes}
    internodes = sorted(set(by_role.get("internode", [])))
    main = [node_id for node_id in internodes if int(nodes[node_id].attributes.get("order", 0)) == 0]
    lateral = [
        node_id
        for node_id in internodes
        if int(nodes[node_id].attributes.get("order", 0)) > 0
        and turtle.poses[node_id].effect == "advance_anchor_calibrated"
    ]
    if main:
        selected.add(main[0])
    if lateral:
        selected.add(lateral[0])
    leaf_nodes = sorted(set(by_role.get("leaf_rachis", [])))
    if not leaf_nodes:
        leaf_nodes = sorted(set(by_role.get("petiole", [])))
    if leaf_nodes:
        selected.add(leaf_nodes[0])
    fruit_nodes = sorted(
        set(by_role.get("truss_rachis", [])),
        key=lambda node_id: (-primitive_counts[node_id], node_id),
    )
    if fruit_nodes:
        selected.add(fruit_nodes[0])
    return sorted(selected)


def _selected_geometry(
    geometry: ReconstructedGeometry, node_ids: list[int]
) -> ReconstructedGeometry:
    selected = set(node_ids)
    return ReconstructedGeometry(
        axes=[item for item in geometry.axes if item.source_node_id in selected],
        spheres=[item for item in geometry.spheres if item.source_node_id in selected],
        diagnostics={
            **geometry.diagnostics,
            "validation_scope": "deterministic_representative_organs",
            "selected_node_ids": node_ids,
        },
        connections=geometry.connections,
    )


def validate_project(
    project_path: str | Path,
    *,
    steps: int,
    plant_id: int = 1,
    api_url: str = DEFAULT_API_URL,
    function_name: str = DEFAULT_FUNCTION,
) -> MigrationValidationBundle:
    """Run Phase C/D against artifacts created by one disposable simulation."""

    if steps < 1:
        raise ValueError("steps must be one or greater for migration validation")
    source_project = Path(project_path).expanduser().resolve()
    step_results: list[StepResult] = []
    client = GroIMPClient(api_url)

    with isolated_project(source_project) as runtime_project:
        runtime_root = runtime_project.parent
        with client.open_project(str(runtime_project)) as workbench:
            runtime_output = str(runtime_root / "output") + "/"
            escaped_output = runtime_output.replace("\\", "\\\\").replace('"', '\\"')
            output_payload = run_json_call(
                workbench.runXLQuery(
                    f'println("__AUTOTOM_OUTPUT__\\t" + (PATH_OUTPUT = "{escaped_output}"));'
                ),
                operation="temporary PATH_OUTPUT configuration",
            )
            if not any(
                str(line).startswith("__AUTOTOM_OUTPUT__\t")
                for line in output_payload.get("console", [])
            ):
                raise GroIMPError("GroIMP did not accept the temporary PATH_OUTPUT")
            for step_number in range(1, steps + 1):
                payload = client.run_function(workbench, function_name)
                step_results.append(
                    StepResult(
                        step=step_number,
                        function_name=function_name,
                        console=tuple(str(line) for line in payload.get("console", [])),
                        logs=tuple(str(line) for line in payload.get("logs", [])),
                    )
                )
            simulation_time = query_model_time(workbench)
            if simulation_time is None:
                simulation_time = _infer_simulation_time(step_results)
            if simulation_time is None:
                raise GroIMPError("cannot determine the simulation day for generated CSV")
            csv_name = f"graph_day_{int(simulation_time)}.csv"
            csv_path = runtime_root / "output" / "dynamic_output" / "graphs" / csv_name
            # getWD() is server-dependent. Invoke the model's existing CSV
            # exporter with an explicit disposable path to guarantee that B is
            # captured from this exact open state.
            escaped_csv_path = str(csv_path).replace("\\", "\\\\").replace('"', '\\"')
            run_json_call(
                workbench.runXLQuery(f'exportPlantGraph("{escaped_csv_path}");'),
                operation="same-run graph CSV export",
            )
            snapshot = inspect_workbench(workbench)
            turtle = resolve_turtle(snapshot)
            geometry = build_rendered_geometry(snapshot, turtle, strict=True)
            plant_state = extract_plant_state(
                snapshot,
                turtle,
                plant_id=plant_id,
                metadata={
                    "simulation_time": simulation_time,
                    "source_model": source_project.name,
                    "source_project_sha256": hashlib.sha256(
                        source_project.read_bytes()
                    ).hexdigest(),
                },
                strict=True,
            )
            selected_ids = _representative_node_ids(snapshot, turtle, geometry)
            meshes: dict[int, ObjMesh] = {}
            groimp_obj_bytes: dict[int, bytes] = {}
            for node_id in selected_ids:
                payload = client.export_subscene_obj(workbench, node_id)
                groimp_obj_bytes[node_id] = payload
                meshes[node_id] = parse_obj(payload)

        if csv_path.is_file():
            csv_capture_mode = "same_run_rgg_export"
        else:
            csv_path.write_bytes(_comparison_csv_bytes(snapshot, int(simulation_time)))
            csv_capture_mode = "same_run_native_projection_for_legacy_gsz"
        csv_bytes = csv_path.read_bytes()

        v1_path = runtime_root / f"legacy_v1_day_{int(simulation_time)}.usda"
        from exporterV1.usd_exporter import export_plant_usd

        export_plant_usd(plant_state, v1_path)
        v1_usd_bytes = v1_path.read_bytes()

        from exporterV2.adapters.groimp_csv.parser import parse_csv_to_branches

        v2_branches, v2_terminal_bodies, _ = parse_csv_to_branches(
            int(simulation_time),
            plant_id,
            include_terminal_bodies=True,
            save_json=False,
            csv_path=csv_path,
        )
        selected = _selected_geometry(geometry, selected_ids)
        geometry_validation = validate_rendered_geometry(selected, meshes)
        comparison = compare_representations(
            snapshot,
            turtle,
            csv_path,
            geometry=geometry,
            v1_usd_path=v1_path,
            v2_branches=v2_branches,
            v2_terminal_bodies=v2_terminal_bodies,
        )
        comparison.metadata["csv_capture_mode"] = csv_capture_mode

    inspection = InspectionReport(
        metadata={
            "source_project": str(source_project),
            "api_url": _sanitized_url(api_url),
            "function_name": function_name,
            "steps_requested": steps,
            "steps_completed": len(step_results),
            "simulation_time": simulation_time,
            "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        },
        steps=step_results,
        snapshot=snapshot,
        diagnostics={
            "isolation": "temporary_project_copy",
            "source_project_modified": False,
            **snapshot.diagnostics,
        },
    )
    return MigrationValidationBundle(
        inspection=inspection,
        turtle=turtle,
        geometry=geometry,
        geometry_validation=geometry_validation,
        comparison=comparison,
        csv_name=csv_name,
        csv_capture_mode=csv_capture_mode,
        csv_bytes=csv_bytes,
        v1_usd_bytes=v1_usd_bytes,
        groimp_obj_bytes=groimp_obj_bytes,
        v2_branches=list(v2_branches),
        v2_terminal_bodies=list(v2_terminal_bodies),
    )


def save_bundle(bundle: MigrationValidationBundle, output_dir: str | Path) -> Path:
    destination = Path(output_dir).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    save_report(bundle.inspection, destination / "native_inspection.json")
    (destination / bundle.csv_name).write_bytes(bundle.csv_bytes)
    (destination / "legacy_v1.usda").write_bytes(bundle.v1_usd_bytes)
    subscene_dir = destination / "groimp_subscenes"
    subscene_dir.mkdir(exist_ok=True)
    for node_id, payload in sorted(bundle.groimp_obj_bytes.items()):
        (subscene_dir / f"node_{node_id}.obj").write_bytes(payload)
    (destination / "legacy_v2_config.json").write_text(
        json.dumps(
            {
                "branches": bundle.v2_branches,
                "terminal_bodies": bundle.v2_terminal_bodies,
            },
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    save_geometry_report(bundle.geometry_validation, destination / "geometry_validation.json")
    save_comparison_report(bundle.comparison, destination / "migration_comparison.json")
    save_comparison_markdown(bundle.comparison, destination / "migration_comparison.md")
    save_debug_obj(bundle.geometry, destination / "debug_geometry.obj")
    return destination


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate GroIMP rendering and compare native/CSV/V1/V2 representations."
    )
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--plant-id", type=int, default=1)
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    parser.add_argument("--function", dest="function_name", default=DEFAULT_FUNCTION)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        bundle = validate_project(
            args.project,
            steps=args.steps,
            plant_id=args.plant_id,
            api_url=args.api_url,
            function_name=args.function_name,
        )
        destination = save_bundle(bundle, args.output_dir)
    except (GroIMPError, FileNotFoundError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    summary = bundle.geometry_validation.summary
    print(
        f"[OK] Geometry: passed={summary['passed']}, ambiguous={summary['ambiguous']}, "
        f"failed={summary['failed']}, not_recoverable={summary['not_recoverable']}"
    )
    print(f"[OK] Comparison: {bundle.comparison.metadata['status']}")
    print(f"[OK] Artifacts: {destination}")
    if summary["failed"] or bundle.comparison.metadata["status"] != "passed":
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
