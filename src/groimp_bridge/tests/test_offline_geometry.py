"""Offline reconstruction and OBJ validation tests for migration Phase C."""

from __future__ import annotations

import json
import math

import pytest

from groimp_bridge.geometry import (
    ObjMesh,
    build_rendered_geometry,
    parse_obj,
    save_debug_obj,
    save_geometry_report,
    validate_rendered_geometry,
)
from groimp_bridge.models import GraphEdge, GraphNode, GroIMPGraphSnapshot
from groimp_bridge.turtle import resolve_turtle


def _snapshot(nodes, edges):
    return GroIMPGraphSnapshot(root_id=0, nodes=nodes, edges=edges, counts_by_type={})


def _edge(source, target, kind="successor"):
    return GraphEdge(source, target, kind, 256 if kind == "successor" else 512)


def test_leaf_and_fruit_productions_expand_into_axis_primitives():
    leaf = GraphNode(
        1,
        "organs.Leaf",
        {
            "counterClocKWiseOrientationPetiole": 30.0,
            "anglePetiole": 45.0,
            "lengthPetiole": 0.1,
            "diameterPetiole": 0.01,
            "bladesNr": 3,
            "lengthPetiolules": [0.03, 0.02],
            "inclinationOnSegmentsPetiolules": [10.0, 20.0],
            "segmentsLength": [0.04],
            "diameterPetiolule": 0.004,
            "diameterSegment": 0.006,
            "leafCurvature": 100.0,
        },
    )
    fruits = GraphNode(
        2,
        "organs.Fruits",
        {
            "fruitNr": 3,
            "degreeDaysStorage": [1.0, 2.0, 3.0],
            "fruitRadius": [0.01, 0.02, 0.03],
            "fruitPairing": False,
            "INTERNODETRUSSLENGTH": 0.05,
            "PETIOLELENGTH": 0.02,
            "internodeTrussdiameter": 0.003,
            "internodeTrussAngle": 9.0,
            "angleAmongSubsequentFruits": 35.0,
        },
    )
    snapshot = _snapshot(
        [GraphNode(0, "Node"), leaf, fruits],
        [_edge(0, 1), _edge(0, 2, "branch")],
    )

    geometry = build_rendered_geometry(snapshot, resolve_turtle(snapshot), strict=True)

    leaf_axes = [axis for axis in geometry.axes if axis.source_node_id == 1]
    fruit_axes = [axis for axis in geometry.axes if axis.source_node_id == 2]
    assert len(leaf_axes) == 7
    assert {axis.role for axis in leaf_axes} == {
        "petiole",
        "petiolule_left",
        "petiolule_right",
        "leaf_rachis",
        "rachis_terminal",
    }
    assert len(fruit_axes) == 5
    assert sum(axis.role == "truss_rachis" for axis in fruit_axes) == 2
    assert sum(axis.role == "pedicel" for axis in fruit_axes) == 3
    assert len(geometry.spheres) == 3
    assert geometry.spheres[-1].radius == pytest.approx(0.03)


def _cylinder_mesh(length=2.0, radius=0.5, sides=12):
    vertices = []
    for z in (0.0, length):
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            vertices.append((radius * math.cos(angle), radius * math.sin(angle), z))
    faces = []
    for index in range(sides):
        nxt = (index + 1) % sides
        faces.append((index, nxt, sides + nxt, sides + index))
    return ObjMesh(tuple(vertices), tuple(faces))


def test_obj_axis_mapping_component_matching_and_serialization(tmp_path):
    payload = "v 1 2 3\nv 4 5 6\nv 7 8 9\nf 1 2 3\n"
    parsed = parse_obj(payload)
    assert parsed.vertices[0] == (1.0, 3.0, 2.0)

    internode = GraphNode(
        1,
        "organs.Internode",
        {"length": 2.0, "internode_width_m": 1.0},
    )
    snapshot = _snapshot([GraphNode(0, "Node"), internode], [_edge(0, 1)])
    geometry = build_rendered_geometry(snapshot, resolve_turtle(snapshot), strict=True)
    report = validate_rendered_geometry(geometry, {1: _cylinder_mesh()})

    assert report.summary == {
        "passed": 1,
        "ambiguous": 0,
        "failed": 0,
        "not_recoverable": 0,
    }
    first = save_geometry_report(report, tmp_path / "one.json")
    second = save_geometry_report(report, tmp_path / "two.json")
    assert first.read_bytes() == second.read_bytes()
    assert json.loads(first.read_text())["report_schema_version"] == "groimp_geometry_validation/1.0"
    overlay = save_debug_obj(geometry, tmp_path / "debug.obj")
    assert "l 1 2" in overlay.read_text()


def test_missing_mesh_is_explicitly_not_recoverable():
    internode = GraphNode(
        1,
        "organs.Internode",
        {"length": 1.0, "internode_width_m": 0.1},
    )
    snapshot = _snapshot([GraphNode(0, "Node"), internode], [_edge(0, 1)])
    geometry = build_rendered_geometry(snapshot, resolve_turtle(snapshot), strict=True)

    report = validate_rendered_geometry(geometry, {})

    assert report.summary["not_recoverable"] == 1
    assert report.checks[0].diagnostic == "subscene was not exported"
