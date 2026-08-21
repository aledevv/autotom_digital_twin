"""GroIMP bridge primitives and ground-truth inspection tools."""

from pathlib import Path
from typing import Any

from .models import (
    GraphEdge,
    GraphNode,
    GroIMPGraphSnapshot,
    InspectionReport,
    StepResult,
    WorldAnchor,
)


def inspect_project(
    project_path: str | Path,
    *,
    api_url: str = "http://localhost:58081/api/",
    steps: int = 1,
    function_name: str = "Dynamic_Model",
) -> InspectionReport:
    """Lazily import the CLI module so ``python -m`` remains warning-free."""

    from .inspector import inspect_project as _inspect_project

    return _inspect_project(
        project_path,
        api_url=api_url,
        steps=steps,
        function_name=function_name,
    )


def inspect_workbench(workbench: Any) -> GroIMPGraphSnapshot:
    """Inspect an already-open workbench without taking ownership of it."""

    from .inspector import inspect_workbench as _inspect_workbench

    return _inspect_workbench(workbench)

__all__ = [
    "GraphEdge",
    "GraphNode",
    "GroIMPGraphSnapshot",
    "InspectionReport",
    "StepResult",
    "WorldAnchor",
    "inspect_project",
    "inspect_workbench",
]
