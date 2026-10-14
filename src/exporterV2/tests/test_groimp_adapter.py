from __future__ import annotations

from collections import Counter
from functools import lru_cache
from pathlib import Path

import pytest

from exporterV2.adapters.groimp import PEDICEL_LENGTH_SCALE, main, plant_state_to_skeleton
from plant_skeleton import load_skeleton, save_skeleton, skeleton_warnings
from plant_skeleton.geometry import segment_length
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]


@lru_cache(maxsize=None)
def skeleton_for_day(day: int):
    return plant_state_to_skeleton(load_plant_state(PROJECT_ROOT / f"data/plant_states/plant_state_day_{day}.json"))


# Rigid links the old PlantState exporter built for each day.
@pytest.mark.parametrize("day, links", [(1, 9), (10, 16), (25, 51), (50, 133), (80, 216), (160, 216)])
def test_segment_count_matches_the_old_exporter(day, links):
    skeleton = skeleton_for_day(day)
    assert len(skeleton.nodes) == links
    assert skeleton_warnings(skeleton) == []


def test_day_160_structure():
    skeleton = skeleton_for_day(160)
    kinds = Counter(node.kind for node in skeleton.nodes)
    assert kinds == {
        "stem": 10, "branch": 16, "petiole": 27, "leaf_rachis": 28, "truss_rachis": 63, "pedicel": 72,
    }
    assert Counter(leaflet.side for leaflet in skeleton.leaflets) == {"left": 53, "right": 53, "terminal": 25}
    assert len(skeleton.fruits) == 72
    assert skeleton.root.start == (0.0, 0.0, 0.0)
    assert skeleton.metadata.time == 160.0
    # Side branches are grouped explicitly; GroIMP ids are kept in node ids.
    assert all(node.axis and node.axis.startswith("branch_g") for node in skeleton.nodes if node.kind == "branch")
    assert all(node.id.startswith("g") for node in skeleton.nodes)


def test_pedicels_are_lengthened_and_fruits_carry_ripeness():
    state = load_plant_state(PROJECT_ROOT / "data/plant_states/plant_state_day_160.json")
    skeleton = skeleton_for_day(160)
    groimp = sum(axis.length for axis in state.axes if axis.role == "pedicel" and axis.length > 0.0)
    ours = sum(segment_length(node.start, node.end) for node in skeleton.nodes if node.kind == "pedicel")
    assert ours == pytest.approx(groimp * PEDICEL_LENGTH_SCALE)
    assert all(0.0 <= fruit.ripeness <= 1.0 for fruit in skeleton.fruits)
    assert any(fruit.ripeness == 1.0 for fruit in skeleton.fruits)


def test_cli_writes_the_default_input_file(tmp_path):
    output = tmp_path / "skeleton.json"
    assert main(["--day", "25", "--output", str(output)]) == 0
    expected = save_skeleton(skeleton_for_day(25), tmp_path / "expected.json")
    assert output.read_text() == expected.read_text()
    assert len(load_skeleton(output).nodes) == 51


def test_committed_input_is_groimp_day_160(tmp_path):
    # input/skeleton.json is what run_mainV2.sh opens by default.
    fresh = tmp_path / "day160.json"
    save_skeleton(skeleton_for_day(160), fresh)
    assert (PROJECT_ROOT / "input" / "skeleton.json").read_text() == fresh.read_text()
