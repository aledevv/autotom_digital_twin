"""Opt-in canonical PlantState validation against the real tomato model."""

from __future__ import annotations

from collections import Counter
import hashlib
import os
from pathlib import Path

import pytest
import numpy as np

from groimp_bridge.client import GroIMPConnectionError
from groimp_bridge.extractor import extract_plant_state
from groimp_bridge.geometry import build_rendered_geometry
from groimp_bridge.inspector import inspect_project
from groimp_bridge.turtle import resolve_turtle
from plant_state import load_plant_state, plant_states_equivalent, save_plant_state
from exporterV1.audit import audit_v1_stage
from exporterV1.usd_exporter import export_plant_usd


pytestmark = pytest.mark.groimp

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PROJECT_PATH = PROJECT_ROOT / "model" / "project_bridge.gsz"


def _require_live_tests(*, slow: bool = False) -> None:
    if os.environ.get("RUN_GROIMP_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_TESTS=1 to enable GroIMP integration tests")
    if slow and os.environ.get("RUN_GROIMP_SLOW_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_SLOW_TESTS=1 for day-25/day-80 PlantState tests")


def _source_hashes() -> dict[str, str]:
    paths = [PROJECT_PATH]
    for directory in (PROJECT_ROOT / "model" / "input", PROJECT_ROOT / "model" / "output"):
        paths.extend(path for path in directory.rglob("*") if path.is_file())
    return {
        str(path.relative_to(PROJECT_ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(paths)
    }


def _extract_or_skip(day: int):
    try:
        report = inspect_project(PROJECT_PATH, steps=day)
    except GroIMPConnectionError as exc:
        pytest.skip(f"GroIMP server is unavailable: {exc}")
    resolution = resolve_turtle(report.snapshot)
    state = extract_plant_state(
        report.snapshot,
        resolution,
        metadata={
            "simulation_time": report.metadata["simulation_time"],
            "source_model": PROJECT_PATH.name,
            "source_project_sha256": hashlib.sha256(PROJECT_PATH.read_bytes()).hexdigest(),
        },
    )
    return report, resolution, state


def _assert_source_equivalence(report, resolution, state) -> None:
    state_by_groimp_id = {node.groimp_node_id: node for node in state.nodes}
    assert set(state_by_groimp_id) == {
        node.id
        for node in report.snapshot.nodes
        if node.id in {
            item.groimp_node_id for item in state.nodes
        }
    }
    for groimp_id, node in state_by_groimp_id.items():
        pose = resolution.poses[groimp_id]
        assert np.allclose(node.pose.incoming_world, pose.incoming_frame.matrix, atol=1e-12)
        assert np.allclose(node.pose.outgoing_world, pose.outgoing_frame.matrix, atol=1e-12)
    rendered = build_rendered_geometry(report.snapshot, resolution, strict=True)
    selected = set(state_by_groimp_id)
    assert {axis.id for axis in state.axes} == {
        axis.primitive_id for axis in rendered.axes if axis.source_node_id in selected
    }
    assert {sphere.id for sphere in state.spheres} == {
        sphere.primitive_id for sphere in rendered.spheres if sphere.source_node_id in selected
    }


def _assert_round_trip(state, tmp_path: Path, day: int) -> None:
    path = save_plant_state(state, tmp_path / f"plant_state_day_{day}.json")
    loaded = load_plant_state(path)
    assert loaded == state
    assert plant_states_equivalent(loaded, state)


def _assert_v1_export(state, tmp_path: Path, day: int) -> None:
    path = export_plant_usd(state, tmp_path / f"tree_v1_day_{day}.usda")
    manifest = audit_v1_stage(state, path)
    assert manifest.errors == ()
    assert manifest.metadata["status"] == "passed"
    assert manifest.diagnostics["filtering_applied"] is False
    assert manifest.usd_organ_prims == manifest.plant_state_organs
    assert manifest.expected_geometry == manifest.created_geometry
    assert manifest.topology["usd_parent_links"] == len(state.edges)


def test_live_day_1_canonical_state_round_trip_and_source_unchanged(tmp_path):
    _require_live_tests()
    before = _source_hashes()
    report, resolution, state = _extract_or_skip(1)
    assert _source_hashes() == before

    counts = Counter(organ.organ_type for organ in state.organs)
    assert counts == Counter(
        {"PlantBase": 1, "Root": 1, "Internode": 3, "Leaf": 5, "Meristem": 2}
    )
    assert len(state.nodes) == 26
    assert len(state.edges) == 25
    internodes = sorted(
        (organ for organ in state.organs if organ.organ_type == "Internode"),
        key=lambda organ: organ.common.rank,
    )
    assert [organ.common.declared_length for organ in internodes] == pytest.approx(
        [0.008802679, 0.008836136, 0.006407313]
    )
    assert all(organ.properties.effective_length_source == "groimp_anchor_calibrated" for organ in internodes)
    _assert_source_equivalence(report, resolution, state)
    _assert_round_trip(state, tmp_path, 1)
    _assert_v1_export(state, tmp_path, 1)


@pytest.mark.slow
def test_live_day_25_canonical_state_covers_branches_and_reproductive_organs(tmp_path):
    _require_live_tests(slow=True)
    before = _source_hashes()
    report, resolution, state = _extract_or_skip(25)
    assert _source_hashes() == before
    counts = Counter(organ.organ_type for organ in state.organs)
    assert counts["Internode"] == 15
    assert counts["Leaf"] == 17
    assert counts["Truss"] == 1
    assert counts["Fruits"] == 1
    assert any(edge.kind == "branch" for edge in state.edges)
    assert {axis.role for axis in state.axes} >= {
        "internode", "petiole", "leaf_rachis", "truss_rachis", "pedicel"
    }
    assert state.spheres
    _assert_source_equivalence(report, resolution, state)
    _assert_round_trip(state, tmp_path, 25)
    _assert_v1_export(state, tmp_path, 25)


@pytest.mark.slow
def test_live_day_80_excludes_marker_and_preserves_full_mature_plant(tmp_path):
    _require_live_tests(slow=True)
    before = _source_hashes()
    report, resolution, state = _extract_or_skip(80)
    assert _source_hashes() == before
    counts = Counter(organ.organ_type for organ in state.organs)
    assert counts["PlantBase"] == 1
    assert counts["Root"] == 1
    assert counts["Internode"] == 26
    assert counts["Leaf"] == 27
    assert counts["Truss"] == 9
    assert counts["Fruits"] == 9
    assert len(state.nodes) == 251
    assert len(state.edges) == 250
    assert state.diagnostics["excluded_marker_plant_bases"] == [421657]
    assert 421657 not in {node.groimp_node_id for node in state.nodes}
    assert len(state.spheres) == 72
    assert state.diagnostics["leaf_surface_assets_canonicalized"] is False
    _assert_source_equivalence(report, resolution, state)
    _assert_round_trip(state, tmp_path, 80)
    _assert_v1_export(state, tmp_path, 80)
