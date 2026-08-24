"""Extract consecutive canonical PlantState days from one GroIMP workbench."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import sys
import tempfile
from typing import Iterator

from plant_state import PlantState, PlantStateValidationError, save_plant_state

from .client import GroIMPClient, GroIMPError
from .extractor import PlantExtractionError, extract_plant_state
from .inspector import (
    DEFAULT_API_URL,
    DEFAULT_FUNCTION,
    _infer_simulation_time,
    inspect_workbench,
    override_model_duration,
)
from .models import StepResult
from .queries import query_model_time
from .runtime import isolated_project, override_runtime_paths
from .turtle import resolve_turtle


def iter_project_states(
    project_path: str | Path,
    *,
    from_day: int,
    to_day: int,
    plant_id: int = 1,
    api_url: str = DEFAULT_API_URL,
    function_name: str = DEFAULT_FUNCTION,
    strict: bool = True,
    selected_days: set[int] | None = None,
) -> Iterator[PlantState]:
    """Yield selected states from one consecutive, lifecycle-safe simulation.

    GroIMP is always advanced through every preceding day. ``selected_days``
    only avoids the relatively expensive graph snapshot and canonical geometry
    extraction for days whose JSON is already present during a resumed run.
    """

    if from_day < 1 or to_day < from_day:
        raise ValueError("require 1 <= from_day <= to_day")
    if selected_days is not None and not selected_days.issubset(
        set(range(from_day, to_day + 1))
    ):
        raise ValueError("selected_days must be inside the requested range")
    source_project = Path(project_path).expanduser().resolve()
    parameters_path = source_project.parent / "param" / "parameters.rgg"
    source_hash = hashlib.sha256(source_project.read_bytes()).hexdigest()
    client = GroIMPClient(api_url)

    with isolated_project(source_project) as runtime_project:
        with client.open_project(str(runtime_project)) as workbench:
            parameters = parameters_path.read_text(encoding="utf-8")
            parameters = override_model_duration(parameters, to_day)
            parameters = override_runtime_paths(parameters, runtime_project.parent)
            client.update_source(workbench, "param/parameters.rgg", parameters)
            client.compile(workbench)

            for expected_day in range(1, to_day + 1):
                payload = client.run_function(workbench, function_name)
                actual_day = query_model_time(workbench)
                if actual_day is None:
                    actual_day = _infer_simulation_time(
                        [
                            StepResult(
                                step=expected_day,
                                function_name=function_name,
                                console=tuple(
                                    str(line) for line in payload.get("console", [])
                                ),
                                logs=tuple(
                                    str(line) for line in payload.get("logs", [])
                                ),
                            )
                        ]
                    )
                if actual_day is None or int(actual_day) != expected_day:
                    raise PlantExtractionError(
                        f"GroIMP day mismatch: expected {expected_day}, observed {actual_day}"
                    )
                if expected_day < from_day or (
                    selected_days is not None and expected_day not in selected_days
                ):
                    continue
                snapshot = inspect_workbench(workbench)
                resolution = resolve_turtle(snapshot, strict=strict)
                yield extract_plant_state(
                    snapshot,
                    resolution,
                    plant_id=plant_id,
                    metadata={
                        "simulation_time": actual_day,
                        "source_model": source_project.name,
                        "source_project_sha256": source_hash,
                    },
                    strict=strict,
                )


def _positive_int(value: str) -> int:
    result = int(value)
    if result < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return result


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Extract a consecutive PlantState range from one isolated GroIMP run."
        )
    )
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--from-day", type=_positive_int, default=1)
    parser.add_argument(
        "--to-day",
        type=_positive_int,
        help="Last day of a consecutive range (inclusive).",
    )
    parser.add_argument(
        "--day",
        type=_positive_int,
        help="Extract only this day; shorthand for --from-day N --to-day N.",
    )
    parser.add_argument("--plant-id", type=_positive_int, default=1)
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    parser.add_argument("--function", dest="function_name", default=DEFAULT_FUNCTION)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace existing day JSON files only after all conflicts are checked.",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help=(
            "Resume a range: retain existing JSON files and write only missing days. "
            "The isolated GroIMP simulation is still advanced in order."
        ),
    )
    return parser


def _destination(output_dir: Path, day: int, plant_id: int) -> Path:
    suffix = "" if plant_id == 1 else f"_plant_{plant_id}"
    return output_dir / f"plant_state_day_{day}{suffix}.json"


def _save_atomically(state: PlantState, destination: Path) -> Path:
    """Validate fully before atomically publishing one completed day."""

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            prefix=f".{destination.name}.",
            suffix=".tmp",
            dir=destination.parent,
            delete=False,
        ) as temporary:
            temporary_path = Path(temporary.name)
        save_plant_state(state, temporary_path)
        os.replace(temporary_path, destination)
    finally:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
    return destination


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.overwrite and args.skip_existing:
        print("[ERROR] --overwrite and --skip-existing are mutually exclusive", file=sys.stderr)
        return 2
    if args.day is not None:
        if args.to_day is not None or args.from_day != 1:
            print(
                "[ERROR] --day cannot be combined with --from-day or --to-day",
                file=sys.stderr,
            )
            return 2
        from_day = to_day = args.day
    else:
        if args.to_day is None:
            print("[ERROR] provide --day N or --to-day N", file=sys.stderr)
            return 2
        from_day = args.from_day
        to_day = args.to_day
    if to_day < from_day:
        print("[ERROR] --to-day must be >= --from-day", file=sys.stderr)
        return 2
    output_dir = args.output_dir.expanduser().resolve()
    destinations = {
        day: _destination(output_dir, day, args.plant_id)
        for day in range(from_day, to_day + 1)
    }
    conflicts = [path for path in destinations.values() if path.exists()]
    if conflicts and not (args.overwrite or args.skip_existing):
        print(
            f"[ERROR] {len(conflicts)} output file(s) already exist; "
            "use --overwrite to replace them or --skip-existing to resume",
            file=sys.stderr,
        )
        return 2
    selected_days = {
        day
        for day, destination in destinations.items()
        if not (args.skip_existing and destination.exists())
    }
    if not selected_days:
        print(
            f"[OK] Nothing to extract; retained {len(conflicts)} existing file(s)."
        )
        return 0

    try:
        written = 0
        for state in iter_project_states(
            args.project,
            from_day=from_day,
            to_day=to_day,
            plant_id=args.plant_id,
            api_url=args.api_url,
            function_name=args.function_name,
            selected_days=selected_days,
        ):
            day = int(state.metadata.simulation_time)
            destination = _save_atomically(state, destinations[day])
            written += 1
            print(
                f"[OK] day={day} nodes={len(state.nodes)} "
                f"axes={len(state.axes)} spheres={len(state.spheres)} -> {destination}",
                flush=True,
            )
    except KeyboardInterrupt:
        print(
            "\n[ERROR] Extraction interrupted. Completed JSON files are valid; "
            "rerun with --skip-existing to resume.",
            file=sys.stderr,
        )
        return 130
    except (
        FileNotFoundError,
        GroIMPError,
        PlantExtractionError,
        PlantStateValidationError,
        ValueError,
    ) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    skipped = len(conflicts) if args.skip_existing else 0
    print(f"[OK] Extracted {written} PlantState file(s); skipped {skipped} existing file(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
