"""Offline tests for canonical PlantState extraction and persistence."""

from __future__ import annotations

from dataclasses import replace
import json

import pytest

from groimp_bridge.extractor import (
    PlantExtractionError,
    extract_plant_state,
    extract_workbench_state,
    main,
)
from groimp_bridge.models import GraphEdge, GraphNode, GroIMPGraphSnapshot
from groimp_bridge.turtle import resolve_turtle
from plant_state import (
    InternodeProperties,
    LeafProperties,
    PlantStateSchemaError,
    PlantStateValidationError,
    load_plant_state,
    plant_states_equivalent,
    save_plant_state,
    validate_plant_state,
)


def _edge(source: int, target: int, kind: str = "successor") -> GraphEdge:
    return GraphEdge(source, target, kind, 256 if kind == "successor" else 512)


def _common(**values):
    return {
        "plant_number": 1,
        "organ_type": 0,
        "order": 0,
        "rank": 0,
        "parent_rank": -1,
        "isFruit": False,
        "isRoot": False,
        "isStemTruss": False,
        "age_in_days_d": 5,
        "age_in_degree_days_dd": 50.0,
        "length": 0.0,
        "area_m2": 0.01,
        "dry_biomass_mg": 2.0,
        **values,
    }


def _snapshot(*, equivalent_candidates: bool = False) -> GroIMPGraphSnapshot:
    base = GraphNode(
        10,
        "plant_level.PlantBase",
        {
            "plant_number": 1,
            "row": 2,
            "pos": 3,
            "age_in_days_d": 5,
            "age_in_degree_days_dd": 50.0,
            "initialAngle": 12.0,
            "nr_internodes": 1.0,
            "leafArea": 0.01,
        },
    )
    internode = GraphNode(
        12,
        "organs.Internode",
        _common(length=0.2, internode_width_m=0.02, length_increment_daily_m=None),
    )
    leaf = GraphNode(
        13,
        "organs.Leaf",
        _common(
            rank=1,
            bladesNr=3,
            lengthPetiole=0.1,
            diameterPetiole=0.01,
            diameterPetiolule=0.004,
            diameterSegment=0.006,
            anglePetiole=45.0,
            counterClocKWiseOrientationPetiole=30.0,
            leafCurvature=100.0,
            area_m2bladesTotal=0.02,
            segmentsLength=[0.04],
            lengthPetiolules=[0.03, 0.02],
            area_m2blades=[0.01, 0.01, 0.0],
            inclinationOnSegmentsPetiolules=[10.0, 20.0],
            counterClocKWiseOrientationSegments=None,
        ),
    )
    fruits = GraphNode(
        14,
        "organs.Fruits",
        _common(
            rank=2,
            isFruit=True,
            fruitPairing=False,
            fruitNr=2,
            PETIOLELENGTH=0.02,
            INTERNODETRUSSLENGTH=0.05,
            fruitRadius=[0.01, 0.02],
            degreeDaysStorage=[1.0, 2.0],
            internodeTrussAngle=9.0,
            internodeTrussdiameter=0.003,
            angleAmongSubsequentFruits=35.0,
            Ripening_dd=100.0,
        ),
    )
    meristem = GraphNode(
        15,
        "organs.Meristem",
        {
            key: value
            for key, value in _common(
                rank=3,
                has_already_auxiliary_bud=True,
                has_already_truss_bud=False,
            ).items()
            if key not in {"area_m2", "dry_biomass_mg"}
        },
    )
    marker = GraphNode(
        5,
        "plant_level.PlantBase",
        {
            "plant_number": 1,
            "row": 2,
            "pos": 3,
            "age_in_days_d": 0,
            "age_in_degree_days_dd": 0.0,
            "initialAngle": 0.0,
            "nr_internodes": 0.0,
            "leafArea": 0.0,
        },
    )
    nodes = [
        GraphNode(0, "de.grogra.graph.impl.Node"),
        marker,
        GraphNode(6, "de.grogra.imp3d.objects.Sphere"),
        base,
        GraphNode(11, "de.grogra.turtle.RH", {"angle": 15.0}),
        internode,
        leaf,
        fruits,
        meristem,
    ]
    edges = [
        _edge(0, 5, "branch"),
        _edge(5, 6),
        _edge(0, 10, "branch"),
        _edge(10, 11),
        _edge(11, 12),
        _edge(12, 13, "branch"),
        _edge(12, 14, "branch"),
        _edge(12, 15),
    ]
    if equivalent_candidates:
        nodes.append(
            GraphNode(7, "organs.Root", _common(isRoot=True, organ_type=10))
        )
        edges.append(_edge(5, 7))
        # Give each candidate the same number of biological descendants.
        nodes = [node for node in nodes if node.id not in {13, 14, 15}]
        edges = [edge for edge in edges if edge.target not in {13, 14, 15}]
    return GroIMPGraphSnapshot(
        root_id=0,
        nodes=nodes,
        edges=edges,
        counts_by_type={},
    )


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


def test_extracts_one_biological_subtree_with_typed_organs_and_geometry(canonical_state):
    state = canonical_state
    assert state.schema_version == "plant_state/1.0"
    assert state.root_node_id == "node:10"
    assert {node.groimp_node_id for node in state.nodes} == {10, 11, 12, 13, 14, 15}
    assert state.diagnostics["excluded_marker_plant_bases"] == [5]
    assert state.metadata.units["length"] == "m"
    assert state.metadata.conventions["rotation_columns"].endswith("local_z_head")

    internode = next(organ for organ in state.organs if organ.organ_type == "Internode")
    assert isinstance(internode.properties, InternodeProperties)
    assert internode.common.declared_length == pytest.approx(0.2)
    assert internode.properties.effective_length == pytest.approx(0.2)
    assert internode.properties.diameter == pytest.approx(0.02)
    assert internode.properties.effective_length_source == "groimp_api"

    leaf = next(organ for organ in state.organs if organ.organ_type == "Leaf")
    assert isinstance(leaf.properties, LeafProperties)
    assert leaf.properties.petiolule_lengths == (0.03, 0.02)
    assert leaf.properties.segment_azimuths is None
    assert {axis.role for axis in state.axes if axis.owner_node_id == leaf.node_id} >= {
        "petiole", "leaf_rachis", "petiolule_left", "petiolule_right"
    }
    assert len(state.spheres) == 2
    assert {operation.operation for operation in state.turtle_operations} == {"RH"}
    assert validate_plant_state(state) == ()


def test_json_is_deterministic_strict_and_round_trips_without_groimp(canonical_state, tmp_path):
    first = save_plant_state(canonical_state, tmp_path / "one.json")
    second = save_plant_state(canonical_state, tmp_path / "two.json")
    assert first.read_bytes() == second.read_bytes()
    assert first.read_bytes().endswith(b"\n")
    loaded = load_plant_state(first)
    assert plant_states_equivalent(canonical_state, loaded)
    assert loaded == canonical_state

    payload = json.loads(first.read_text())
    payload["schema_version"] = "plant_state/2.0"
    (tmp_path / "future.json").write_text(json.dumps(payload))
    with pytest.raises(PlantStateSchemaError, match="Unsupported PlantState schema"):
        load_plant_state(tmp_path / "future.json")
    payload["schema_version"] = "plant_state/1.0"
    payload["unexpected"] = True
    (tmp_path / "unknown.json").write_text(json.dumps(payload))
    with pytest.raises(PlantStateSchemaError, match="unknown"):
        load_plant_state(tmp_path / "unknown.json")
    del payload["unexpected"]
    payload["organs"][0]["common"]["is_fruit"] = "false"
    (tmp_path / "wrong_bool.json").write_text(json.dumps(payload))
    with pytest.raises(PlantStateSchemaError, match="must be a boolean"):
        load_plant_state(tmp_path / "wrong_bool.json")


def test_validator_rejects_broken_pose_and_references(canonical_state):
    node = canonical_state.nodes[1]
    bad_matrix = tuple(
        tuple(2.0 if row == column == 0 else value for column, value in enumerate(values))
        for row, values in enumerate(node.pose.incoming_world)
    )
    broken_node = replace(node, pose=replace(node.pose, incoming_world=bad_matrix))
    broken = replace(
        canonical_state,
        nodes=(canonical_state.nodes[0], broken_node, *canonical_state.nodes[2:]),
    )
    with pytest.raises(PlantStateValidationError, match="not unit length"):
        validate_plant_state(broken)
    assert validate_plant_state(broken, strict=False)

    internode_index = next(
        index
        for index, organ in enumerate(canonical_state.organs)
        if organ.organ_type == "Internode"
    )
    organ = canonical_state.organs[internode_index]
    negative = replace(
        canonical_state,
        organs=(
            *canonical_state.organs[:internode_index],
            replace(organ, properties=replace(organ.properties, diameter=-0.01)),
            *canonical_state.organs[internode_index + 1 :],
        ),
    )
    with pytest.raises(PlantStateValidationError, match="negative internode dimensions"):
        validate_plant_state(negative)

    cycle = replace(
        canonical_state,
        edges=(
            *canonical_state.edges,
            replace(
                canonical_state.edges[0],
                source=canonical_state.nodes[-1].id,
                target=canonical_state.root_node_id,
            ),
        ),
    )
    with pytest.raises(PlantStateValidationError, match="cycle"):
        validate_plant_state(cycle)


def test_equivalent_plant_bases_fail_strict_and_are_deterministic_non_strict():
    snapshot = _snapshot(equivalent_candidates=True)
    resolution = resolve_turtle(snapshot)
    with pytest.raises(PlantExtractionError, match="Multiple equivalent PlantBase"):
        extract_plant_state(snapshot, resolution)
    state = extract_plant_state(snapshot, resolution, strict=False)
    assert state.root_node_id == "node:5"
    assert state.diagnostics["ambiguous_best_candidates"] == [5, 10]


def test_cli_writes_reloadable_json(monkeypatch, canonical_state, tmp_path, capsys):
    monkeypatch.setattr(
        "groimp_bridge.extractor.extract_project_state",
        lambda *args, **kwargs: canonical_state,
    )
    output = tmp_path / "plant_state.json"
    assert main([
        "--project", "fixture.gsz", "--steps", "5", "--plant-id", "1",
        "--output", str(output),
    ]) == 0
    assert load_plant_state(output) == canonical_state
    assert "[OK] PlantState" in capsys.readouterr().out


def test_workbench_api_does_not_take_lifecycle_ownership(monkeypatch):
    snapshot = _snapshot()

    class CallerOwnedWorkbench:
        close_called = False

        def close(self):
            self.close_called = True
            raise AssertionError("extract_workbench_state must not close the workbench")

    workbench = CallerOwnedWorkbench()
    monkeypatch.setattr("groimp_bridge.extractor.inspect_workbench", lambda value: snapshot)
    monkeypatch.setattr("groimp_bridge.extractor.query_model_time", lambda value: 5)
    state = extract_workbench_state(workbench)
    assert state.metadata.simulation_time == 5
    assert workbench.close_called is False
