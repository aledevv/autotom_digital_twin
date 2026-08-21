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
from .turtle import (
    ResolvedNodePose,
    TurtleFrame,
    TurtleResolution,
    TurtleResolutionError,
    resolve_turtle,
)
from .geometry import (
    AxisPrimitive,
    GeometryTolerance,
    GeometryConnection,
    GeometryValidationReport,
    ObjMesh,
    ReconstructedGeometry,
    SpherePrimitive,
    build_rendered_geometry,
    parse_obj,
    validate_rendered_geometry,
)
from .comparison import MigrationComparisonReport, compare_representations


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


def validate_project(
    project_path: str | Path,
    *,
    steps: int,
    plant_id: int = 1,
    api_url: str = "http://localhost:58081/api/",
    function_name: str = "Dynamic_Model",
):
    """Lazily run the combined Phase C/D validation pipeline."""

    from .migration_validation import validate_project as _validate_project

    return _validate_project(
        project_path,
        steps=steps,
        plant_id=plant_id,
        api_url=api_url,
        function_name=function_name,
    )

__all__ = [
    "GraphEdge",
    "GraphNode",
    "AxisPrimitive",
    "GeometryTolerance",
    "GeometryConnection",
    "GeometryValidationReport",
    "GroIMPGraphSnapshot",
    "InspectionReport",
    "MigrationComparisonReport",
    "ObjMesh",
    "ReconstructedGeometry",
    "StepResult",
    "SpherePrimitive",
    "ResolvedNodePose",
    "TurtleFrame",
    "TurtleResolution",
    "TurtleResolutionError",
    "WorldAnchor",
    "build_rendered_geometry",
    "compare_representations",
    "inspect_project",
    "inspect_workbench",
    "parse_obj",
    "resolve_turtle",
    "validate_project",
    "validate_rendered_geometry",
]
