"""Opt-in rendered geometry oracle tests against GroIMP."""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from groimp_bridge.client import GroIMPClient, GroIMPConnectionError, run_json_call
from groimp_bridge.geometry import (
    AxisPrimitive,
    ReconstructedGeometry,
    SpherePrimitive,
    parse_obj,
    validate_rendered_geometry,
)
from groimp_bridge.inspector import DEFAULT_API_URL, inspect_workbench
from groimp_bridge.queries import (
    FieldSpec,
    build_anchor_query,
    build_attribute_query,
    parse_anchor_lines,
    parse_attribute_lines,
)
from groimp_bridge.turtle import TurtleFrame


pytestmark = pytest.mark.groimp
FIXTURE = Path(__file__).parent / "fixtures" / "rendered_geometry.rgg"


def _require_live_tests():
    if os.environ.get("RUN_GROIMP_TESTS") != "1":
        pytest.skip("set RUN_GROIMP_TESTS=1 to enable GroIMP integration tests")


def _query(workbench, node_type, field):
    payload = run_json_call(
        workbench.runXLQuery(build_attribute_query(node_type, field)),
        operation=f"fixture {node_type}.{field.name}",
    )
    return parse_attribute_lines(payload.get("console", []), field.kind)


def _anchors(workbench, node_type):
    payload = run_json_call(
        workbench.runXLQuery(build_anchor_query(node_type)),
        operation=f"fixture {node_type} anchors",
    )
    return parse_anchor_lines(payload.get("console", []))


def test_controlled_cylinder_and_sphere_match_exported_obj():
    _require_live_tests()
    client = GroIMPClient(DEFAULT_API_URL)
    try:
        with client.create_project(name="autotom-phase-c-rendered-geometry") as workbench:
            client.update_source(workbench, "Model.rgg", FIXTURE.read_text(encoding="utf-8"))
            client.compile(workbench)
            snapshot = inspect_workbench(workbench)
            axis_node = next(node for node in snapshot.nodes if node.type == "Model.AxisProbe")
            ball_node = next(node for node in snapshot.nodes if node.type == "Model.BallProbe")
            axis_anchor = _anchors(workbench, "Model.AxisProbe")[axis_node.id]
            ball_anchor = _anchors(workbench, "Model.BallProbe")[ball_node.id]
            axis_length = _query(
                workbench, "Model.AxisProbe", FieldSpec("axisLength", "float")
            )[axis_node.id]
            axis_radius = _query(
                workbench, "Model.AxisProbe", FieldSpec("axisRadius", "float")
            )[axis_node.id]
            ball_radius = _query(
                workbench, "Model.BallProbe", FieldSpec("ballRadius", "float")
            )[ball_node.id]
            direction = axis_anchor.direction
            end = tuple(
                axis_anchor.position[index] + axis_length * direction[index]
                for index in range(3)
            )
            frame = TurtleFrame.identity().translate_local(*axis_anchor.position)
            geometry = ReconstructedGeometry(
                axes=[
                    AxisPrimitive(
                        "fixture-axis", axis_node.id, "AxisProbe", "internode",
                        axis_anchor.position, end, direction, axis_length, axis_radius, frame,
                    )
                ],
                spheres=[
                    SpherePrimitive(
                        "fixture-ball", ball_node.id, "BallProbe", "fruit",
                        ball_anchor.position, ball_radius,
                        TurtleFrame.identity().translate_local(*ball_anchor.position),
                    )
                ],
            )
            meshes = {
                axis_node.id: parse_obj(client.export_subscene_obj(workbench, axis_node.id)),
                ball_node.id: parse_obj(client.export_subscene_obj(workbench, ball_node.id)),
            }
            report = validate_rendered_geometry(geometry, meshes)
    except GroIMPConnectionError as exc:
        pytest.skip(f"GroIMP server is unavailable: {exc}")

    assert report.summary["passed"] == 2
    assert report.summary["failed"] == 0
    assert report.summary["not_recoverable"] == 0
