"""Offline coverage for consecutive PlantState extraction orchestration."""

from dataclasses import replace
from pathlib import Path

from groimp_bridge import batch_extractor
from plant_state import load_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]


def test_batch_cli_writes_deterministic_day_names(monkeypatch, tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_1.json"
    )
    states = [
        replace(state, metadata=replace(state.metadata, simulation_time=day))
        for day in (2, 3)
    ]
    monkeypatch.setattr(
        batch_extractor,
        "iter_project_states",
        lambda *args, **kwargs: iter(states),
    )
    assert batch_extractor.main(
        [
            "--project",
            "model/project_bridge.gsz",
            "--from-day",
            "2",
            "--to-day",
            "3",
            "--output-dir",
            str(tmp_path),
        ]
    ) == 0
    assert (tmp_path / "plant_state_day_2.json").is_file()
    assert (tmp_path / "plant_state_day_3.json").is_file()


def test_batch_cli_rejects_conflicts_before_opening_groimp(
    monkeypatch, tmp_path, capsys
):
    (tmp_path / "plant_state_day_1.json").write_text("existing", encoding="utf-8")
    monkeypatch.setattr(
        batch_extractor,
        "iter_project_states",
        lambda *args, **kwargs: (_ for _ in ()).throw(
            AssertionError("GroIMP must not be contacted")
        ),
    )
    assert batch_extractor.main(
        [
            "--project",
            "model/project_bridge.gsz",
            "--to-day",
            "2",
            "--output-dir",
            str(tmp_path),
        ]
    ) == 2
    assert "already exist" in capsys.readouterr().err


def test_batch_cli_day_shortcut_and_atomic_publish(monkeypatch, tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_1.json"
    )
    requested = {}

    def fake_states(*args, **kwargs):
        requested.update(kwargs)
        return iter([replace(state, metadata=replace(state.metadata, simulation_time=10))])

    monkeypatch.setattr(batch_extractor, "iter_project_states", fake_states)
    assert batch_extractor.main(
        [
            "--project",
            "model/project_bridge.gsz",
            "--day",
            "10",
            "--output-dir",
            str(tmp_path),
        ]
    ) == 0
    assert requested["from_day"] == 10
    assert requested["to_day"] == 10
    assert load_plant_state(tmp_path / "plant_state_day_10.json").metadata.simulation_time == 10
    assert not list(tmp_path.glob("*.tmp"))


def test_batch_cli_skip_existing_resumes_without_overwriting(monkeypatch, tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_1.json"
    )
    destination = tmp_path / "plant_state_day_1.json"
    destination.write_text("keep-me", encoding="utf-8")
    monkeypatch.setattr(
        batch_extractor,
        "iter_project_states",
        lambda *args, **kwargs: iter([state]),
    )
    assert batch_extractor.main(
        [
            "--project",
            "model/project_bridge.gsz",
            "--day",
            "1",
            "--output-dir",
            str(tmp_path),
            "--skip-existing",
        ]
    ) == 0
    assert destination.read_text(encoding="utf-8") == "keep-me"


def test_batch_cli_resume_extracts_only_missing_days(monkeypatch, tmp_path):
    state = load_plant_state(
        PROJECT_ROOT / "data" / "plant_states" / "plant_state_day_1.json"
    )
    (tmp_path / "plant_state_day_1.json").write_text("keep-me", encoding="utf-8")
    requested = {}

    def fake_states(*args, **kwargs):
        requested.update(kwargs)
        return iter([replace(state, metadata=replace(state.metadata, simulation_time=2))])

    monkeypatch.setattr(batch_extractor, "iter_project_states", fake_states)
    assert batch_extractor.main(
        [
            "--project",
            "model/project_bridge.gsz",
            "--from-day",
            "1",
            "--to-day",
            "2",
            "--output-dir",
            str(tmp_path),
            "--skip-existing",
        ]
    ) == 0
    assert requested["selected_days"] == {2}
    assert load_plant_state(tmp_path / "plant_state_day_2.json").metadata.simulation_time == 2
