"""Serverless command line interface for the canonical V1 exporter."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from plant_state import PlantStateSchemaError, PlantStateValidationError, load_plant_state

from .adapter import V1TopologyError
from .audit import V1AuditError, manifest_path_for
from .usd_exporter import export_plant_usd


def repository_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_input_path(day: int, plant_id: int) -> Path:
    filename = (
        f"plant_state_day_{day}.json"
        if plant_id == 1
        else f"plant_state_day_{day}_plant_{plant_id}.json"
    )
    return repository_root() / "data" / "plant_states" / filename


def default_output_path(day: int, plant_id: int) -> Path:
    suffix = "" if plant_id == 1 else f"_plant_{plant_id}"
    return repository_root() / "data" / "usd_models" / f"tree_v1_day_{day}{suffix}.usda"


def _positive_int(value: str) -> int:
    result = int(value)
    if result < 1:
        raise argparse.ArgumentTypeError("must be one or greater")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a static V1 USDA stage from plant_state/1.0."
    )
    parser.add_argument("--day", type=_positive_int, required=True)
    parser.add_argument("--plant-id", type=_positive_int, default=1)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--output", type=Path)
    return parser


def _missing_input_message(path: Path, day: int, plant_id: int) -> str:
    return (
        f"PlantState input does not exist: {path}\n"
        "Prepare it explicitly (GroIMP must be available only for this step):\n"
        "  uv run python -m groimp_bridge.extractor "
        f"--project model/project_bridge.gsz --steps {day} --plant-id {plant_id} "
        f"--output {path}"
    )


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    input_path = (args.input or default_input_path(args.day, args.plant_id)).expanduser()
    output_path = (args.output or default_output_path(args.day, args.plant_id)).expanduser()
    if not input_path.is_file():
        print(
            f"[ERROR] {_missing_input_message(input_path, args.day, args.plant_id)}",
            file=sys.stderr,
        )
        return 2

    try:
        state = load_plant_state(input_path)
        if state.metadata.plant_id != args.plant_id:
            raise ValueError(
                f"PlantState plant_id={state.metadata.plant_id}, requested {args.plant_id}"
            )
        if state.metadata.simulation_time is None or abs(
            float(state.metadata.simulation_time) - args.day
        ) > 1e-12:
            raise ValueError(
                "PlantState simulation_time="
                f"{state.metadata.simulation_time!r}, requested day {args.day}"
            )
        destination = export_plant_usd(state, output_path)
    except (
        OSError,
        PlantStateSchemaError,
        PlantStateValidationError,
        V1TopologyError,
        V1AuditError,
        ValueError,
    ) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    print(
        f"[OK] V1 static stage: day={args.day}, plant_id={args.plant_id}, "
        f"nodes={len(state.nodes)}, organs={len(state.organs)}"
    )
    print(f"[OK] USDA: {destination}")
    print(f"[OK] Manifest: {manifest_path_for(destination)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
