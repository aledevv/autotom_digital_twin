"""Opt-in GroIMP oracle tests for controlled turtle semantics."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from groimp_bridge.client import GroIMPClient, GroIMPConnectionError, run_json_call
from groimp_bridge.inspector import DEFAULT_API_URL, inspect_workbench
from groimp_bridge.queries import (
    FieldSpec,
    build_anchor_query,
    build_attribute_query,
    parse_anchor_lines,
    parse_attribute_lines,
)
from groimp_bridge.turtle import resolve_turtle


pytestmark = pytest.mark.groimp

FIXTURE_PATH = Path(__file__).parent / "fixtures" / "turtle_semantics.rgg"


def _require_live_tests() -> None:
    if os.environ.get("RUN_GROIMP_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_TESTS=1 to enable GroIMP integration tests")


def _enrich_fixture_type(workbench, snapshot, node_type, fields):
    nodes_by_id = {node.id: node for node in snapshot.nodes}
    for field in fields:
        payload = run_json_call(
            workbench.runXLQuery(build_attribute_query(node_type, field)),
            operation=f"fixture query {node_type}.{field.name}",
        )
        for node_id, value in parse_attribute_lines(payload.get("console", []), field.kind).items():
            nodes_by_id[node_id].attributes[field.name] = value
    payload = run_json_call(
        workbench.runXLQuery(build_anchor_query(node_type)),
        operation=f"fixture anchors {node_type}",
    )
    for node_id, anchor in parse_anchor_lines(payload.get("console", [])).items():
        nodes_by_id[node_id].world_anchor = anchor


def _fixture_snapshot():
    client = GroIMPClient(DEFAULT_API_URL)
    source = FIXTURE_PATH.read_text(encoding="utf-8")
    try:
        with client.create_project(name="autotom-phase-b-turtle-fixture") as workbench:
            client.update_source(workbench, "Model.rgg", source)
            client.compile(workbench)
            snapshot = inspect_workbench(workbench)
            _enrich_fixture_type(
                workbench,
                snapshot,
                "Model.Probe",
                (FieldSpec("marker", "int"),),
            )
            _enrich_fixture_type(
                workbench,
                snapshot,
                "Model.FrameProbe",
                (FieldSpec("frameId", "int"), FieldSpec("axisId", "int")),
            )
            _enrich_fixture_type(
                workbench,
                snapshot,
                "Model.Internode",
                (FieldSpec("length", "float"),),
            )
            return snapshot
    except GroIMPConnectionError as exc:
        pytest.skip(f"GroIMP server is unavailable: {exc}")


def _probes_by_marker(snapshot):
    return {
        node.attributes["marker"]: node
        for node in snapshot.nodes
        if node.type == "Model.Probe"
    }


def test_controlled_fixture_matches_groimp_positions_directions_and_full_basis():
    _require_live_tests()
    snapshot = _fixture_snapshot()
    resolution = resolve_turtle(snapshot)
    probes = _probes_by_marker(snapshot)

    assert resolution.diagnostics["unresolved_node_ids"] == []
    assert set(probes) == set(range(23))

    # Direct GroIMP observations establish units, signs, composition, and scopes.
    assert probes[1].world_anchor.position == pytest.approx((0, 0, 2), abs=1e-9)
    assert probes[2].world_anchor.position == pytest.approx((0, 0, 5), abs=1e-9)
    assert probes[3].world_anchor.direction == pytest.approx((1, 0, 0), abs=1e-9)
    assert probes[4].world_anchor.direction == pytest.approx((0, -1, 0), abs=1e-9)
    assert probes[5].world_anchor.direction == pytest.approx((0, 1, 0), abs=1e-9)
    assert probes[6].world_anchor.direction == pytest.approx((1, 0, 0), abs=1e-9)
    assert probes[7].world_anchor.position == pytest.approx((-2, 1, 3), abs=1e-9)
    assert probes[8].world_anchor.position == pytest.approx((3, 2, -1), abs=1e-9)
    assert probes[9].world_anchor.position == pytest.approx((1, -3, 2), abs=1e-9)
    assert probes[12].world_anchor == probes[10].world_anchor
    assert probes[13].world_anchor == probes[0].world_anchor
    assert probes[14].world_anchor.position == pytest.approx((0, 0, 1), abs=1e-9)
    assert probes[15].world_anchor.position == pytest.approx((0, 0, 1), abs=1e-9)
    assert probes[15].world_anchor.direction == pytest.approx((0, 1, 0), abs=1e-9)
    assert probes[17].world_anchor.position == pytest.approx((0, 0, 2), abs=1e-9)
    assert probes[20].world_anchor.direction == pytest.approx((0, 0, -1), abs=1e-8)
    assert probes[22].world_anchor.direction == pytest.approx((0, 0, -1), abs=1e-9)

    # Every probe independently validates the resolver against GroIMP's oracle.
    for probe in probes.values():
        anchor = probe.world_anchor
        pose = resolution.poses[probe.id]
        assert anchor is not None
        assert pose.incoming_frame.position == pytest.approx(anchor.position, abs=1e-9)
        assert pose.incoming_frame.head == pytest.approx(anchor.direction, abs=1e-9)

    compound = resolution.poses[probes[18].id].incoming_frame
    axes = {
        node.attributes["axisId"]: node.world_anchor.direction
        for node in snapshot.nodes
        if node.type == "Model.FrameProbe"
        and node.attributes["frameId"] == 1
        and node.world_anchor is not None
    }
    assert axes[0] == pytest.approx(compound.left, abs=1e-9)
    assert axes[1] == pytest.approx(compound.up, abs=1e-9)
    assert axes[2] == pytest.approx(compound.head, abs=1e-9)

    gravity_frame = resolution.poses[probes[20].id].incoming_frame
    gravity_axes = {
        node.attributes["axisId"]: node.world_anchor.direction
        for node in snapshot.nodes
        if node.type == "Model.FrameProbe"
        and node.attributes["frameId"] == 2
        and node.world_anchor is not None
    }
    assert gravity_axes[0] == pytest.approx(gravity_frame.left, abs=1e-8)
    assert gravity_axes[1] == pytest.approx(gravity_frame.up, abs=1e-8)
    assert gravity_axes[2] == pytest.approx(gravity_frame.head, abs=1e-8)
