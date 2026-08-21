"""Opt-in integration tests against the local GroIMP server."""

from __future__ import annotations

import hashlib
import os
from pathlib import Path

import pytest

from groimp_bridge.client import GroIMPConnectionError
from groimp_bridge.inspector import inspect_project


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


def _inspect_or_skip(steps: int):
    try:
        return inspect_project(PROJECT_PATH, steps=steps)
    except GroIMPConnectionError as exc:
        pytest.skip(f"GroIMP server is unavailable: {exc}")


def test_live_day_1_ground_truth_and_source_outputs_unchanged():
    _require_live_tests()
    before_hash = _sha256(REFERENCE_CSV)

    report = _inspect_or_skip(1)

    assert _sha256(REFERENCE_CSV) == before_hash
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
