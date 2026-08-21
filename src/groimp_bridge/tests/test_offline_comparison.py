"""Offline semantic-diff tests for migration Phase D."""

from __future__ import annotations

import csv
import json
from types import SimpleNamespace

from groimp_bridge.comparison import (
    compare_representations,
    save_comparison_markdown,
    save_comparison_report,
)
from groimp_bridge.geometry import build_rendered_geometry
from groimp_bridge.models import GraphEdge, GraphNode, GroIMPGraphSnapshot
from groimp_bridge.turtle import resolve_turtle


CSV_FIELDS = [
    "day", "plant_id", "organ_class", "organ_index", "rank", "order",
    "parent_rank", "parent_organ_class", "age_dd", "dry_biomass_mg", "area_m2",
    "length", "is_fruit", "is_root", "internode_width_m",
]


def _native_snapshot():
    root = GraphNode(
        1,
        "organs.Root",
        {
            "plant_number": 1, "rank": 0, "order": 0, "parent_rank": -1,
            "age_in_degree_days_dd": 2.0, "dry_biomass_mg": 3.0,
            "area_m2": 0.0, "length": 0.0, "isFruit": False, "isRoot": True,
        },
    )
    internode = GraphNode(
        2,
        "organs.Internode",
        {
            "plant_number": 1, "rank": 0, "order": 0, "parent_rank": -1,
            "age_in_degree_days_dd": 2.0, "dry_biomass_mg": 4.0,
            "area_m2": 0.2, "length": 0.1, "isFruit": False, "isRoot": False,
            "internode_width_m": 0.01,
        },
    )
    return GroIMPGraphSnapshot(
        root_id=0,
        nodes=[GraphNode(0, "Node"), root, internode],
        edges=[GraphEdge(0, 1, "successor", 256), GraphEdge(1, 2, "successor", 256)],
        counts_by_type={},
    )


def _write_csv(path, *, bad_length=False):
    rows = [
        [1, 1, "Root", 0, 0, 0, -1, "none", 2, 3, 0, 0, "false", "true", 0],
        [1, 1, "Internode", 0, 0, 0, 0, "Root", 2, 4, 0.2,
         0.2 if bad_length else 0.1, "false", "false", 0.01],
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_FIELDS)
        writer.writerows(rows)


def test_same_run_comparison_and_exporter_classifications_are_non_blocking(tmp_path):
    snapshot = _native_snapshot()
    turtle = resolve_turtle(snapshot)
    geometry = build_rendered_geometry(snapshot, turtle, strict=True)
    csv_path = tmp_path / "graph.csv"
    _write_csv(csv_path)

    report = compare_representations(
        snapshot,
        turtle,
        csv_path,
        geometry=geometry,
        v2_branches=[{"id": "trunk", "n_links": 1}],
    )

    assert report.metadata["status"] == "passed"
    assert len(report.matches) == 2
    assert report.diagnostics["classifications"]["PHYSICS_ADAPTATION"] == 1
    assert report.diagnostics["classifications"]["EXPECTED_SIMPLIFICATION"] == 1
    json_one = save_comparison_report(report, tmp_path / "one.json")
    json_two = save_comparison_report(report, tmp_path / "two.json")
    assert json_one.read_bytes() == json_two.read_bytes()
    assert json.loads(json_one.read_text())["report_schema_version"] == "groimp_migration_comparison/1.0"
    markdown = save_comparison_markdown(report, tmp_path / "report.md")
    assert "PHYSICS_ADAPTATION" in markdown.read_text()


def test_same_run_scalar_difference_is_likely_bug(tmp_path):
    snapshot = _native_snapshot()
    turtle = resolve_turtle(snapshot)
    csv_path = tmp_path / "bad.csv"
    _write_csv(csv_path, bad_length=True)

    report = compare_representations(snapshot, turtle, csv_path)

    assert report.metadata["status"] == "investigation_required"
    assert any(
        difference.field == "length" and difference.classification == "LIKELY_BUG"
        for difference in report.differences
    )


def test_migration_cli_writes_to_requested_temporary_directory(monkeypatch, tmp_path, capsys):
    from groimp_bridge import migration_validation

    fake = SimpleNamespace(
        geometry_validation=SimpleNamespace(
            summary={"passed": 2, "ambiguous": 0, "failed": 0, "not_recoverable": 0}
        ),
        comparison=SimpleNamespace(metadata={"status": "passed"}),
    )
    monkeypatch.setattr(migration_validation, "validate_project", lambda *args, **kwargs: fake)

    def fake_save(bundle, output_dir):
        destination = output_dir.resolve()
        destination.mkdir(parents=True)
        (destination / "migration_comparison.json").write_text("{}\n")
        return destination

    monkeypatch.setattr(migration_validation, "save_bundle", fake_save)
    destination = tmp_path / "phase-cd"
    result = migration_validation.main(
        ["--project", "model/project_bridge.gsz", "--steps", "1", "--output-dir", str(destination)]
    )

    assert result == 0
    assert (destination / "migration_comparison.json").is_file()
    assert "Comparison: passed" in capsys.readouterr().out
