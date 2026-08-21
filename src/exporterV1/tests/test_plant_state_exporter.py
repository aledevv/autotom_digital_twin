"""Offline coverage for the lossless canonical V1 rendering path."""

from __future__ import annotations

from collections import Counter
from dataclasses import replace
import json
import sys

import pytest
from pxr import Usd

from exporterV1.adapter import build_v1_render_view
from exporterV1.audit import audit_v1_stage, manifest_path_for
from exporterV1.cli import main
from exporterV1.usd_exporter import export_plant_usd
from exporterV1.isaac_app import _arguments
from groimp_bridge.extractor import extract_plant_state
from groimp_bridge.tests.test_offline_extractor import _snapshot
from groimp_bridge.turtle import resolve_turtle
from plant_state import save_plant_state


@pytest.fixture
def canonical_state():
    snapshot = _snapshot()
    return extract_plant_state(
        snapshot,
        resolve_turtle(snapshot),
        metadata={
            "simulation_time": 5,
            "source_model": "fixture.gsz",
            "source_project_sha256": "a" * 64,
        },
    )


def test_static_export_preserves_every_organ_node_and_parent(canonical_state, tmp_path):
    destination = export_plant_usd(canonical_state, tmp_path / "plant.usda")
    manifest = audit_v1_stage(canonical_state, destination)
    assert manifest.errors == ()
    assert manifest.metadata["status"] == "passed"
    assert manifest.plant_state_organs == {
        "Fruits": 1,
        "Internode": 1,
        "Leaf": 1,
        "Meristem": 1,
        "PlantBase": 1,
    }
    assert manifest.expected_geometry == manifest.created_geometry == {
        "internode": 1,
        "leaf_group": 1,
        "fruit_group": 1,
        "fruit": 2,
    }
    assert manifest.topology["usd_parent_links"] == len(canonical_state.edges)
    assert manifest.diagnostics["filtering_applied"] is False

    stage = Usd.Stage.Open(str(destination))
    organ_ids = {
        prim.GetAttribute("autotom:nodeId").Get()
        for prim in stage.Traverse()
        if prim.GetAttribute("autotom:entityKind").Get() == "organ"
    }
    assert organ_ids == {organ.node_id for organ in canonical_state.organs}
    assert manifest_path_for(destination).is_file()


def test_exact_coincident_duplicate_keeps_organ_but_suppresses_second_visual(
    canonical_state, tmp_path
):
    leaf = next(organ for organ in canonical_state.organs if organ.organ_type == "Leaf")
    node = next(node for node in canonical_state.nodes if node.id == leaf.node_id)
    duplicate_node_id = "node:999"
    primitive_map = {
        primitive_id: f"{primitive_id}:duplicate"
        for primitive_id in leaf.primitive_ids
    }
    duplicate_node = replace(node, id=duplicate_node_id, groimp_node_id=999)
    duplicate_axes = tuple(
        replace(
            axis,
            id=primitive_map[axis.id],
            owner_node_id=duplicate_node_id,
        )
        for axis in canonical_state.axes
        if axis.owner_node_id == node.id
    )
    duplicate_organ = replace(
        leaf,
        id=f"{leaf.id}:duplicate",
        node_id=duplicate_node_id,
        primitive_ids=tuple(primitive_map[item] for item in leaf.primitive_ids),
    )
    parent_edge = next(edge for edge in canonical_state.edges if edge.target == node.id)
    duplicate_edge = replace(parent_edge, target=duplicate_node_id)
    duplicate_state = replace(
        canonical_state,
        nodes=(*canonical_state.nodes, duplicate_node),
        edges=(*canonical_state.edges, duplicate_edge),
        organs=(*canonical_state.organs, duplicate_organ),
        axes=(*canonical_state.axes, *duplicate_axes),
    )
    view = build_v1_render_view(duplicate_state)
    duplicate = next(item for item in view.organs if item.node.id == duplicate_node_id)
    assert duplicate.render_geometry is False
    assert duplicate.duplicate_of == node.id
    destination = export_plant_usd(duplicate_state, tmp_path / "duplicates.usda")
    manifest = audit_v1_stage(duplicate_state, destination)
    assert manifest.errors == ()
    assert manifest.plant_state_organs["Leaf"] == 2
    assert manifest.usd_organ_prims["Leaf"] == 2
    assert manifest.created_geometry["leaf_group"] == 1
    assert manifest.diagnostics["duplicate_geometry_of"] == {
        duplicate_node_id: node.id
    }
    assert manifest.diagnostics["filtering_applied"] is True


def test_cli_validates_metadata_and_writes_deterministic_manifest(
    canonical_state, tmp_path, capsys
):
    source = save_plant_state(canonical_state, tmp_path / "plant_state.json")
    destination = tmp_path / "plant.usda"
    assert main(
        [
            "--day",
            "5",
            "--plant-id",
            "1",
            "--input",
            str(source),
            "--output",
            str(destination),
        ]
    ) == 0
    payload = json.loads(manifest_path_for(destination).read_text())
    assert payload["schema_version"] == "exporter_v1_manifest/1.0"
    assert payload["metadata"]["status"] == "passed"
    assert main(["--day", "6", "--input", str(source)]) == 1
    assert "requested day 6" in capsys.readouterr().err


def test_render_view_never_filters_zero_area_leaf(canonical_state):
    leaf_index = next(
        index
        for index, organ in enumerate(canonical_state.organs)
        if organ.organ_type == "Leaf"
    )
    leaf = canonical_state.organs[leaf_index]
    zero_leaf = replace(
        leaf,
        properties=replace(leaf.properties, blade_area_total=0.0),
    )
    organs = list(canonical_state.organs)
    organs[leaf_index] = zero_leaf
    state = replace(canonical_state, organs=tuple(organs))
    view = build_v1_render_view(state)
    assert Counter(item.organ.organ_type for item in view.organs)["Leaf"] == 1
    assert view.diagnostics["zero_area_leaf_node_ids"] == [leaf.node_id]
    assert view.diagnostics["filtering_applied"] is False


def test_isaac_arguments_are_removed_before_simulation_app(monkeypatch, tmp_path):
    stage = tmp_path / "plant.usda"
    monkeypatch.setattr(
        sys,
        "argv",
        ["isaac_app.py", "--usd", str(stage), "--headless", "--/kit/test=true"],
    )
    args = _arguments()
    assert args.usd == stage
    assert args.headless is True
    assert sys.argv == ["isaac_app.py", "--/kit/test=true"]
