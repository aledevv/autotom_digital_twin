"""Offline coverage for the GroIMP ground-truth inspector."""

from __future__ import annotations

import json
from pathlib import Path
from zipfile import ZipFile

import pytest

from groimp_bridge.client import GroIMPClient
from groimp_bridge.inspector import main, save_report
from groimp_bridge.models import (
    GraphEdge,
    GraphNode,
    GroIMPGraphSnapshot,
    InspectionReport,
    StepResult,
)
from groimp_bridge.queries import (
    FieldSpec,
    build_anchor_query,
    build_attribute_query,
    coerce_value,
    parse_anchor_lines,
    parse_attribute_lines,
    parse_project_graph,
)
from groimp_bridge.runtime import OUTPUT_DIRECTORIES, isolated_project


def test_parse_project_graph_preserves_and_classifies_edges():
    raw = {
        "projectgraphRoot": 0,
        "projectgraphNodes": [
            {"id": 2, "type": "organs.Leaf"},
            {"id": 0, "type": "de.grogra.graph.impl.Node"},
            {"id": 1, "type": "organs.Internode"},
        ],
        "projectgraphEdges": [[1, 2, 512], [0, 1, 256], [2, 3, 999]],
    }

    snapshot = parse_project_graph(raw)

    assert [node.id for node in snapshot.nodes] == [0, 1, 2]
    assert [(edge.source, edge.target, edge.kind) for edge in snapshot.edges] == [
        (0, 1, "successor"),
        (1, 2, "branch"),
        (2, 3, "unknown"),
    ]
    assert snapshot.diagnostics["unknown_edge_codes"] == [999]
    assert snapshot.counts_by_type["organs.Leaf"] == 1


@pytest.mark.parametrize(
    ("raw", "kind", "expected"),
    [
        ("3", "int", 3),
        ("3.25", "float", 3.25),
        ("true", "bool", True),
        ("false", "bool", False),
        ("null", "float_array", None),
        ("[1.0, 2.5, -3]", "float_array", [1.0, 2.5, -3.0]),
        ("0", "float_array", []),
        ("1.0_2.5_-3", "float_array", [1.0, 2.5, -3.0]),
    ],
)
def test_coerce_value(raw, kind, expected):
    assert coerce_value(raw, kind) == expected


def test_query_builders_and_console_parsers():
    field_query = build_attribute_query(
        "organs.Internode", FieldSpec("length", "float")
    )
    anchor_query = build_anchor_query("organs.Internode")

    assert "n[length]" in field_query
    assert "location(n).x" in anchor_query
    assert "direction(n).z" in anchor_query
    assert parse_attribute_lines(
        ["unrelated", "__AUTOTOM_INSPECT__\t42\t0.125"], "float"
    ) == {42: 0.125}
    anchors = parse_anchor_lines(
        ["__AUTOTOM_INSPECT__\t42\t1\t2\t3\t0\t0\t1"]
    )
    assert anchors[42].position == (1.0, 2.0, 3.0)
    assert anchors[42].direction == (0.0, 0.0, 1.0)


class _FakeHTTPResponse:
    status_code = 200
    text = '{"id": "fake"}'


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

    def close(self):
        self.closed = True
        return _FakeCall({})

    def exportSubScene(self, extension, nodeid):
        assert extension == "obj"
        assert nodeid == 42
        return _FakeCall(b"v 0 0 0\n")


class _FakeLink:
    def __init__(self, workbench):
        self.workbench = workbench
        self.opened_path = None

    def openWB(self, path):
        self.opened_path = path
        return _FakeCall(self.workbench)


def test_client_closes_workbench_when_consumer_raises():
    workbench = _FakeWorkbench()
    link = _FakeLink(workbench)
    client = GroIMPClient("http://example/api", gro_link_factory=lambda _: link)

    with pytest.raises(RuntimeError, match="consumer failure"):
        with client.open_project("project.gsz") as opened:
            assert opened is workbench
            raise RuntimeError("consumer failure")

    assert workbench.closed is True
    assert link.opened_path == "project.gsz"


def test_client_exports_subscene_as_in_memory_bytes():
    workbench = _FakeWorkbench()

    assert GroIMPClient.export_subscene_obj(workbench, 42) == b"v 0 0 0\n"


def test_isolated_project_copies_only_project_inputs_and_empty_outputs(tmp_path):
    source_dir = tmp_path / "model"
    source_dir.mkdir()
    project = source_dir / "project_bridge.gsz"
    parameters = (
        'static String PATH_INPUT = getWD()+"input/";\n'
        'static String PATH_OUTPUT = getWD()+"output/";\n'
    )
    with ZipFile(project, "w") as archive:
        archive.writestr("param/parameters.rgg", parameters)
        archive.writestr("Model.rgg", "module Model;\n")
    source_bytes = project.read_bytes()
    (source_dir / "input").mkdir()
    (source_dir / "input" / "values.csv").write_text("x\n1\n", encoding="utf-8")
    (source_dir / "output").mkdir()
    (source_dir / "output" / "must_not_copy.txt").write_text("old", encoding="utf-8")

    with isolated_project(project) as copied_project:
        runtime_root = copied_project.parent
        with ZipFile(copied_project) as archive:
            runtime_parameters = archive.read("param/parameters.rgg").decode(
                "windows-1252"
            )
        assert f'PATH_INPUT = "{runtime_root}/input/"' in runtime_parameters
        assert f'PATH_OUTPUT = "{runtime_root}/output/"' in runtime_parameters
        assert "getWD()" not in runtime_parameters
        assert (runtime_root / "input" / "values.csv").is_file()
        assert not (runtime_root / "output" / "must_not_copy.txt").exists()
        for relative in OUTPUT_DIRECTORIES:
            assert (runtime_root / "output" / relative).is_dir()

    assert not runtime_root.exists()
    assert project.read_bytes() == source_bytes


def _sample_report() -> InspectionReport:
    snapshot = GroIMPGraphSnapshot(
        root_id=0,
        nodes=[GraphNode(id=1, type="organs.Internode", attributes={"length": 0.1})],
        edges=[GraphEdge(source=0, target=1, kind="successor", raw_code=256)],
        counts_by_type={"organs.Internode": 1},
    )
    return InspectionReport(
        metadata={"steps_completed": 1, "simulation_time": 1},
        steps=[StepResult(step=1, function_name="Dynamic_Model")],
        snapshot=snapshot,
    )


def test_save_report_writes_versioned_strict_json(tmp_path):
    output = save_report(_sample_report(), tmp_path / "nested" / "report.json")
    payload = json.loads(output.read_text(encoding="utf-8"))

    assert payload["report_schema_version"] == "groimp_inspection/1.0"
    assert payload["snapshot"]["edges"][0]["raw_code"] == 256
    assert payload["snapshot"]["nodes"][0]["attributes"]["length"] == 0.1


def test_cli_uses_inspector_and_writes_report(monkeypatch, tmp_path, capsys):
    from groimp_bridge import inspector

    monkeypatch.setattr(inspector, "inspect_project", lambda *args, **kwargs: _sample_report())
    destination = tmp_path / "cli-report.json"

    exit_code = main(
        [
            "--project",
            "model/project_bridge.gsz",
            "--steps",
            "1",
            "--output",
            str(destination),
        ]
    )

    assert exit_code == 0
    assert destination.is_file()
    assert "1 nodes" in capsys.readouterr().out
