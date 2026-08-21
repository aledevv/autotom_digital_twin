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
from plant_state import PlantState


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


def extract_plant_state(
    snapshot: GroIMPGraphSnapshot,
    turtle_resolution: TurtleResolution,
    *,
    plant_id: int = 1,
    metadata=None,
    strict: bool = True,
) -> PlantState:
    """Lazily adapt a native snapshot to the canonical domain model."""

    from .extractor import extract_plant_state as _extract_plant_state

    return _extract_plant_state(
        snapshot,
        turtle_resolution,
        plant_id=plant_id,
        metadata=metadata,
        strict=strict,
    )


def extract_workbench_state(
    workbench: Any,
    *,
    plant_id: int = 1,
    strict: bool = True,
) -> PlantState:
    """Extract canonical state while leaving workbench ownership to the caller."""

    from .extractor import extract_workbench_state as _extract_workbench_state

    return _extract_workbench_state(workbench, plant_id=plant_id, strict=strict)


def extract_project_state(
    project_path: str | Path,
    *,
    steps: int,
    plant_id: int = 1,
    api_url: str = "http://localhost:58081/api/",
    function_name: str = "Dynamic_Model",
    strict: bool = True,
) -> PlantState:
    """Extract canonical state through the isolated project runtime."""

    from .extractor import extract_project_state as _extract_project_state

    return _extract_project_state(
        project_path,
        steps=steps,
        plant_id=plant_id,
        api_url=api_url,
        function_name=function_name,
        strict=strict,
    )

__all__ = [
    "GraphEdge",
    "GraphNode",
    "PlantState",
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
    "extract_plant_state",
    "extract_project_state",
    "extract_workbench_state",
    "inspect_project",
    "inspect_workbench",
    "parse_obj",
    "resolve_turtle",
    "validate_project",
    "validate_rendered_geometry",
]
