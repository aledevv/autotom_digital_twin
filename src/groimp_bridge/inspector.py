"""Ground-truth ProjectGraph inspector and command-line entry point."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import sys
from typing import Any
from urllib.parse import urlsplit, urlunsplit

from .client import GroIMPClient, GroIMPError, run_json_call
from .models import GroIMPGraphSnapshot, InspectionReport, StepResult
from .queries import enrich_snapshot, parse_project_graph, query_model_time
from .runtime import isolated_project


DEFAULT_API_URL = "http://localhost:58081/api/"
DEFAULT_FUNCTION = "Dynamic_Model"
DAY_PATTERN = re.compile(r"\bday\s+is\s+(-?\d+(?:\.\d+)?)", re.IGNORECASE)


def _sanitized_url(url: str) -> str:
    parts = urlsplit(url)
    hostname = parts.hostname or ""
    if parts.port is not None:
        hostname = f"{hostname}:{parts.port}"
    return urlunsplit((parts.scheme, hostname, parts.path, "", ""))


def inspect_workbench(workbench: Any) -> GroIMPGraphSnapshot:
    """Inspect the current state of an already-open GroIMP workbench."""

    raw_graph = run_json_call(
        workbench.getProjectGraph(),
        operation="ProjectGraph extraction",
    )
    snapshot = parse_project_graph(raw_graph)
    enrich_snapshot(workbench, snapshot)
    return snapshot


def _infer_simulation_time(step_results: list[StepResult]) -> int | float | None:
    """Use the model's standard console line when its static counter is not queryable."""

    for step_result in reversed(step_results):
        for line in reversed(step_result.console):
            match = DAY_PATTERN.search(line)
            if match:
                value = float(match.group(1))
                return int(value) if value.is_integer() else value
    return None


def inspect_project(
    project_path: str | Path,
    *,
    api_url: str = DEFAULT_API_URL,
    steps: int = 1,
    function_name: str = DEFAULT_FUNCTION,
) -> InspectionReport:
    """Run and inspect a GSZ project without writing into its source directory."""

    if steps < 0:
        raise ValueError("steps must be zero or greater")

    source_project = Path(project_path).expanduser().resolve()
    step_results: list[StepResult] = []
    client = GroIMPClient(api_url)

    with isolated_project(source_project) as runtime_project:
        with client.open_project(str(runtime_project)) as workbench:
            for step_number in range(1, steps + 1):
                payload = client.run_function(workbench, function_name)
                step_results.append(
                    StepResult(
                        step=step_number,
                        function_name=function_name,
                        console=tuple(str(line) for line in payload.get("console", [])),
                        logs=tuple(str(line) for line in payload.get("logs", [])),
                    )
                )

            simulation_time = query_model_time(workbench)
            if simulation_time is None:
                simulation_time = _infer_simulation_time(step_results)
            snapshot = inspect_workbench(workbench)

    diagnostics = {
        "isolation": "temporary_project_copy",
        "source_project_modified": False,
        **snapshot.diagnostics,
    }
    metadata = {
        "source_project": str(source_project),
        "api_url": _sanitized_url(api_url),
        "function_name": function_name,
        "steps_requested": steps,
        "steps_completed": len(step_results),
        "simulation_time": simulation_time,
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
    }
    return InspectionReport(
        metadata=metadata,
        steps=step_results,
        snapshot=snapshot,
        diagnostics=diagnostics,
    )


def save_report(report: InspectionReport, output_path: str | Path) -> Path:
    """Serialize an inspection report as deterministic, strict JSON."""

    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(
            report.to_dict(),
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    return destination


def _non_negative_int(value: str) -> int:
    number = int(value)
    if number < 0:
        raise argparse.ArgumentTypeError("must be zero or greater")
    return number


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Inspect a GroIMP ProjectGraph and write a raw versioned JSON report."
    )
    parser.add_argument("--project", type=Path, required=True, help="Path to project .gsz")
    parser.add_argument("--steps", type=_non_negative_int, default=1)
    parser.add_argument("--api-url", default=DEFAULT_API_URL)
    parser.add_argument("--function", dest="function_name", default=DEFAULT_FUNCTION)
    parser.add_argument("--output", type=Path, required=True, help="Report JSON path")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    try:
        report = inspect_project(
            args.project,
            api_url=args.api_url,
            steps=args.steps,
            function_name=args.function_name,
        )
        output_path = save_report(report, args.output)
    except (GroIMPError, FileNotFoundError, ValueError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    print(
        f"[OK] GroIMP inspection: {len(report.snapshot.nodes)} nodes, "
        f"{len(report.snapshot.edges)} edges, "
        f"simulation_time={report.metadata['simulation_time']}"
    )
    print(f"[OK] Report saved: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
