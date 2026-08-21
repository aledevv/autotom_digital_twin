"""Opt-in integration tests against the local GroIMP server."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from groimp_bridge.client import GroIMPConnectionError
from groimp_bridge.inspector import inspect_project
from groimp_bridge.turtle import resolve_turtle


pytestmark = pytest.mark.groimp

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT_PATH = PROJECT_ROOT / "model" / "project_bridge.gsz"
REFERENCE_CSV = (
    PROJECT_ROOT
    / "model"
    / "output"
    / "dynamic_output"
    / "graphs"
    / "graph_day_1.csv"
)


def _require_live_tests() -> None:
    if os.environ.get("RUN_GROIMP_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_TESTS=1 to enable GroIMP integration tests")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_artifact_hashes() -> dict[str, str]:
    paths = [PROJECT_PATH]
    for directory in (PROJECT_ROOT / "model" / "input", PROJECT_ROOT / "model" / "output"):
        paths.extend(path for path in directory.rglob("*") if path.is_file())
    return {
        str(path.relative_to(PROJECT_ROOT)): _sha256(path)
        for path in sorted(paths)
    }


def _assert_resolution_matches_groimp(report):
    resolution = resolve_turtle(report.snapshot)
    assert resolution.diagnostics["unresolved_node_ids"] == []
    for node in report.snapshot.nodes:
        if node.world_anchor is None:
            continue
        pose = resolution.poses[node.id]
        if node.type == "de.grogra.turtle.Translate":
            frame = pose.outgoing_frame
        else:
            frame = pose.incoming_frame
        assert frame.position == pytest.approx(node.world_anchor.position, abs=1e-6)
        assert frame.head == pytest.approx(node.world_anchor.direction, abs=1e-6)
    return resolution


def _inspect_or_skip(steps: int):
    try:
        return inspect_project(PROJECT_PATH, steps=steps)
    except GroIMPConnectionError as exc:
        pytest.skip(f"GroIMP server is unavailable: {exc}")


def test_live_day_1_ground_truth_and_source_outputs_unchanged():
    _require_live_tests()
    before_hashes = _source_artifact_hashes()

    report = _inspect_or_skip(1)

    assert _source_artifact_hashes() == before_hashes
    assert len(report.snapshot.nodes) == 51
    assert len(report.snapshot.edges) == 50
    assert report.snapshot.counts_by_type["organs.Internode"] == 3
    assert report.snapshot.counts_by_type["organs.Leaf"] == 5
    assert report.metadata["simulation_time"] == 1
    assert report.diagnostics["source_project_modified"] is False

    internodes = sorted(
        (node for node in report.snapshot.nodes if node.type == "organs.Internode"),
        key=lambda node: node.attributes["rank"],
    )
    assert [node.attributes["length"] for node in internodes] == pytest.approx(
        [0.008802679, 0.008836136, 0.006407313]
    )
    assert all(node.world_anchor is not None for node in internodes)
    assert all(node.world_anchor.direction == pytest.approx((0.0, 0.0, 1.0)) for node in internodes)
    _assert_resolution_matches_groimp(report)


@pytest.mark.slow
def test_live_day_25_covers_reproductive_organs_and_arrays():
    _require_live_tests()
    if os.environ.get("RUN_GROIMP_SLOW_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_SLOW_TESTS=1 to enable the day-25 integration test")

    report = _inspect_or_skip(25)

    assert report.snapshot.counts_by_type["organs.Truss"] > 0
    assert report.snapshot.counts_by_type["organs.Fruits"] > 0
    fruits = [node for node in report.snapshot.nodes if node.type == "organs.Fruits"]
    assert fruits
    assert all(isinstance(node.attributes["fruitRadius"], list) for node in fruits)
    assert all(isinstance(node.attributes["degreeDaysStorage"], list) for node in fruits)
    assert any(edge.kind == "branch" for edge in report.snapshot.edges)
    _assert_resolution_matches_groimp(report)


@pytest.mark.slow
def test_live_day_80_resolves_the_full_mature_plant_without_source_changes():
    _require_live_tests()
    if os.environ.get("RUN_GROIMP_SLOW_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_SLOW_TESTS=1 to enable the day-80 integration test")

    before_hashes = _source_artifact_hashes()
    report = _inspect_or_skip(80)
    resolution = _assert_resolution_matches_groimp(report)

    assert _source_artifact_hashes() == before_hashes
    assert report.metadata["simulation_time"] == 80
    assert report.diagnostics["source_project_modified"] is False
    assert len(report.snapshot.nodes) > 200
    assert len(report.snapshot.edges) == len(report.snapshot.nodes) - 1
    assert report.snapshot.counts_by_type["organs.Internode"] >= 20
    assert report.snapshot.counts_by_type["organs.Leaf"] >= 20
    assert report.snapshot.counts_by_type["organs.Truss"] > 0
    assert report.snapshot.counts_by_type["organs.Fruits"] > 0
    assert report.snapshot.counts_by_type["de.grogra.turtle.RG"] > 0
    assert sum(edge.kind == "branch" for edge in report.snapshot.edges) >= 20
    assert resolution.diagnostics["anchor_calibrated_internodes"] >= 20
    assert resolution.diagnostics["unresolved_node_ids"] == []
