"""Versioned data models for raw GroIMP inspection reports."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


REPORT_SCHEMA_VERSION = "groimp_inspection/1.0"


@dataclass(frozen=True)
class WorldAnchor:
    """Position and head direction reported directly by GroIMP."""

    position: tuple[float, float, float]
    direction: tuple[float, float, float]


@dataclass
class GraphNode:
    """One raw ProjectGraph node, optionally enriched through XL queries."""

    id: int
    type: str
    attributes: dict[str, Any] = field(default_factory=dict)
    world_anchor: WorldAnchor | None = None


@dataclass(frozen=True)
class GraphEdge:
    """One ProjectGraph edge with both interpreted and original edge type."""

    source: int
    target: int
    kind: str
    raw_code: int


@dataclass
class GroIMPGraphSnapshot:
    """Deterministic raw snapshot returned by an open GroIMP workbench."""

    root_id: int | None
    nodes: list[GraphNode]
    edges: list[GraphEdge]
    counts_by_type: dict[str, int]
    diagnostics: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class StepResult:
    """Console and log output returned by one RGG function execution."""

    step: int
    function_name: str
    console: tuple[str, ...] = ()
    logs: tuple[str, ...] = ()


@dataclass
class InspectionReport:
    """Top-level serializable result of an isolated GroIMP inspection."""

    metadata: dict[str, Any]
    steps: list[StepResult]
    snapshot: GroIMPGraphSnapshot
    diagnostics: dict[str, Any] = field(default_factory=dict)
    report_schema_version: str = REPORT_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-ready mapping with stable field structure."""

        return asdict(self)
