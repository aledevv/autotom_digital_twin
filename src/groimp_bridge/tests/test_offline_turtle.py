"""Offline tests for GroIMP turtle-frame reconstruction."""

from __future__ import annotations

import numpy as np
import pytest

from groimp_bridge.client import GroIMPClient
from groimp_bridge.models import GraphEdge, GraphNode, GroIMPGraphSnapshot, WorldAnchor
from groimp_bridge.turtle import (
    TurtleFrame,
    TurtleResolutionError,
    resolve_turtle,
)


def _snapshot(
    nodes: list[GraphNode],
    edges: list[GraphEdge],
    *,
    root_id: int = 0,
) -> GroIMPGraphSnapshot:
    return GroIMPGraphSnapshot(
        root_id=root_id,
        nodes=nodes,
        edges=edges,
        counts_by_type={},
    )


def _node(node_id: int, node_type: str, **attributes) -> GraphNode:
    return GraphNode(id=node_id, type=node_type, attributes=attributes)


def _edge(source: int, target: int, kind: str = "successor") -> GraphEdge:
    raw_code = {"successor": 256, "branch": 512}.get(kind, 999)
    return GraphEdge(source=source, target=target, kind=kind, raw_code=raw_code)


def test_frame_uses_left_up_head_columns_and_local_post_multiplication():
    frame = (
        TurtleFrame.identity()
        .rotate_local("head", 90)
        .rotate_local("up", 90)
        .translate_local(1, 2, 3)
    )

    np.testing.assert_allclose(frame.left, (0, 0, -1), atol=1e-12)
    np.testing.assert_allclose(frame.up, (-1, 0, 0), atol=1e-12)
    np.testing.assert_allclose(frame.head, (0, 1, 0), atol=1e-12)
    np.testing.assert_allclose(frame.position, (-2, 3, -1), atol=1e-12)


@pytest.mark.parametrize(
    ("node_type", "angle", "expected_head"),
    [
        ("de.grogra.turtle.RU", 90, (1, 0, 0)),
        ("de.grogra.turtle.RU", -90, (-1, 0, 0)),
        ("de.grogra.turtle.RL", 90, (0, -1, 0)),
        ("de.grogra.turtle.RL", -90, (0, 1, 0)),
        ("de.grogra.turtle.RH", 90, (0, 0, 1)),
    ],
)
def test_rotation_signs_match_groimp(node_type, angle, expected_head):
    snapshot = _snapshot(
        [_node(0, "Node"), _node(1, node_type, angle=angle)],
        [_edge(0, 1)],
    )

    resolution = resolve_turtle(snapshot)

    np.testing.assert_allclose(
        resolution.poses[1].outgoing_frame.head,
        expected_head,
        atol=1e-12,
    )


def test_rg_aligns_head_to_gravity_and_preserves_roll_by_minimal_rotation():
    frame = TurtleFrame.identity().rotate_local("up", 90).rotate_local("head", 37)
    snapshot = _snapshot(
        [_node(0, "Node"), _node(1, "de.grogra.turtle.RG")],
        [_edge(0, 1)],
    )

    resolution = resolve_turtle(snapshot, initial_frame=frame)
    result = resolution.poses[1].outgoing_frame

    np.testing.assert_allclose(result.left, (-0.79863551, 0.60181502, 0), atol=1e-8)
    np.testing.assert_allclose(result.up, (0.60181502, 0.79863551, 0), atol=1e-8)
    np.testing.assert_allclose(result.head, (0, 0, -1), atol=1e-12)
    assert resolution.poses[1].effect == "align_gravity"


def test_rg_antiparallel_case_rotates_around_local_left():
    snapshot = _snapshot(
        [_node(0, "Node"), _node(1, "de.grogra.turtle.RG")],
        [_edge(0, 1)],
    )

    result = resolve_turtle(snapshot).poses[1].outgoing_frame

    np.testing.assert_allclose(result.left, (1, 0, 0), atol=1e-12)
    np.testing.assert_allclose(result.up, (0, -1, 0), atol=1e-12)
    np.testing.assert_allclose(result.head, (0, 0, -1), atol=1e-12)


def test_resolver_advances_internodes_and_isolates_branch_state():
    snapshot = _snapshot(
        [
            _node(0, "Node"),
            _node(
                1,
                "de.grogra.turtle.Translate",
                translateX=1,
                translateY=2,
                translateZ=3,
            ),
            _node(2, "de.grogra.turtle.RH", angle=90),
            _node(3, "de.grogra.turtle.RU", angle=90),
            _node(4, "organs.Internode", length=2),
            _node(5, "organs.Internode", length=1),
            _node(6, "organs.Leaf"),
        ],
        [
            _edge(3, 4),
            _edge(0, 1),
            _edge(4, 5, "branch"),
            _edge(2, 3),
            _edge(4, 6),
            _edge(1, 2),
        ],
    )

    resolution = resolve_turtle(snapshot)

    np.testing.assert_allclose(resolution.poses[4].start_position, (1, 2, 3), atol=1e-12)
    np.testing.assert_allclose(resolution.poses[4].end_position, (1, 4, 3), atol=1e-12)
    np.testing.assert_allclose(resolution.poses[5].end_position, (1, 5, 3), atol=1e-12)
    np.testing.assert_allclose(resolution.poses[6].start_position, (1, 4, 3), atol=1e-12)
    assert resolution.poses[4].effect == "advance"
    assert resolution.traversal_order == (0, 1, 2, 3, 4, 6, 5)


def test_native_successor_anchors_override_stale_declared_internode_length():
    internode = _node(1, "organs.Internode", length=2.25)
    internode.world_anchor = WorldAnchor(position=(0, 0, 0), direction=(0, 0, 1))
    successor = _node(2, "de.grogra.turtle.RH", angle=0)
    successor.world_anchor = WorldAnchor(position=(0, 0, 2), direction=(0, 0, 1))
    snapshot = _snapshot(
        [_node(0, "Node"), internode, successor],
        [_edge(0, 1), _edge(1, 2)],
    )

    resolution = resolve_turtle(snapshot)

    assert resolution.poses[1].end_position == pytest.approx((0, 0, 2))
    assert resolution.poses[1].effect == "advance_anchor_calibrated"
    assert resolution.diagnostics["anchor_calibrated_internodes"] == 1
    assert resolution.diagnostics["max_declared_vs_effective_length_delta"] == pytest.approx(0.25)


def test_unknown_edges_and_nodes_are_diagnostic_not_guessed():
    snapshot = _snapshot(
        [_node(0, "Node"), _node(1, "custom.Mystery"), _node(2, "custom.Orphan")],
        [_edge(0, 1), _edge(1, 2, "unknown")],
    )

    resolution = resolve_turtle(snapshot)

    assert resolution.diagnostics["unknown_edge_codes"] == [999]
    assert resolution.diagnostics["unsupported_node_types"] == ["custom.Mystery"]
    assert resolution.diagnostics["unresolved_node_ids"] == [2]


def test_non_strict_mode_reports_missing_edges_and_multiple_parents():
    snapshot = _snapshot(
        [_node(0, "Node"), _node(1, "Probe"), _node(2, "Probe")],
        [_edge(0, 2), _edge(1, 2, "branch"), _edge(2, 99)],
    )

    resolution = resolve_turtle(snapshot, strict=False)

    assert resolution.diagnostics["missing_edge_nodes"] == [[2, 99]]
    assert resolution.diagnostics["multiple_parents"] == {"2": [0, 1]}


@pytest.mark.parametrize(
    ("snapshot", "message"),
    [
        (
            _snapshot(
                [_node(0, "Node"), _node(1, "Probe"), _node(2, "Probe")],
                [_edge(0, 2), _edge(1, 2, "branch")],
            ),
            "multiple structural parents",
        ),
        (
            _snapshot(
                [_node(0, "Node"), _node(1, "Probe")],
                [_edge(0, 1), _edge(1, 0)],
            ),
            "structural cycle",
        ),
        (
            _snapshot([_node(0, "Node")], [_edge(0, 99)]),
            "missing nodes",
        ),
    ],
)
def test_strict_mode_rejects_ambiguous_topology(snapshot, message):
    with pytest.raises(TurtleResolutionError, match=message):
        resolve_turtle(snapshot)


def test_resolver_rejects_missing_or_non_finite_turtle_attributes():
    missing = _snapshot(
        [_node(0, "Node"), _node(1, "organs.Internode")],
        [_edge(0, 1)],
    )
    non_finite = _snapshot(
        [_node(0, "Node"), _node(1, "de.grogra.turtle.RH", angle=float("nan"))],
        [_edge(0, 1)],
    )

    with pytest.raises(TurtleResolutionError, match="requires numeric attribute 'length'"):
        resolve_turtle(missing)
    with pytest.raises(TurtleResolutionError, match="non-finite attribute 'angle'"):
        resolve_turtle(non_finite)


class _FakeHTTPResponse:
    status_code = 200
    text = '{"id": "fixture"}'


class _FakeCall:
    def __init__(self, value=None):
        self.value = value
        self.result = _FakeHTTPResponse()

    def run(self):
        return self

    def read(self):
        return self.value


class _FakeWorkbench:
    def __init__(self):
        self.closed = False
        self.updated = None
        self.compiled = False

    def close(self):
        self.closed = True
        return _FakeCall({})

    def updateFile(self, name, content):
        self.updated = (name, content)
        return _FakeCall({"console": [], "logs": []})

    def compile(self):
        self.compiled = True
        return _FakeCall({"console": [], "logs": []})


class _FakeLink:
    def __init__(self, workbench):
        self.workbench = workbench
        self.created = None

    def createWB(self, template, name):
        self.created = (template, name)
        return _FakeCall(self.workbench)


def test_temporary_workbench_lifecycle_update_and_compile():
    workbench = _FakeWorkbench()
    link = _FakeLink(workbench)
    client = GroIMPClient("http://example/api", gro_link_factory=lambda _: link)

    with client.create_project(template="newRGG", name="fixture") as opened:
        client.update_source(opened, "Model.rgg", "module Fixture;")
        client.compile(opened)

    assert link.created == ("newRGG", "fixture")
    assert workbench.updated == ("Model.rgg", "module Fixture;")
    assert workbench.compiled is True
    assert workbench.closed is True


def test_temporary_workbench_closes_when_fixture_consumer_raises():
    workbench = _FakeWorkbench()
    link = _FakeLink(workbench)
    client = GroIMPClient("http://example/api", gro_link_factory=lambda _: link)

    with pytest.raises(RuntimeError, match="fixture failed"):
        with client.create_project(name="fixture"):
            raise RuntimeError("fixture failed")

    assert workbench.closed is True
