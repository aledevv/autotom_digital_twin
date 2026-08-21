"""Canonical, exporter-independent data model for one GroIMP plant."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, TypeAlias

from .schema import (
    DEFAULT_CONVENTIONS,
    DEFAULT_UNITS,
    PLANT_STATE_SCHEMA_VERSION,
)


Vector3: TypeAlias = tuple[float, float, float]
Matrix4: TypeAlias = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


@dataclass(frozen=True)
class PlantMetadata:
    simulation_time: int | float | None
    plant_id: int
    source: str = "groimp_api"
    source_model: str | None = None
    source_project_sha256: str | None = None
    units: dict[str, str] = field(default_factory=lambda: dict(DEFAULT_UNITS))
    conventions: dict[str, str] = field(
        default_factory=lambda: dict(DEFAULT_CONVENTIONS)
    )


@dataclass(frozen=True)
class NodePose:
    incoming_world: Matrix4
    outgoing_world: Matrix4
    local_effect: Matrix4
    world_start: Vector3
    world_end: Vector3
    orientation_source: str = "groimp_turtle"


@dataclass(frozen=True)
class PlantNode:
    id: str
    groimp_node_id: int
    source_type: str
    category: str
    parent_id: str | None
    incoming_edge_kind: str | None
    incoming_edge_raw_code: int | None
    pose: NodePose
    source_attributes: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class PlantEdge:
    source: str
    target: str
    kind: str
    raw_code: int


@dataclass(frozen=True)
class CommonOrganProperties:
    plant_id: int
    rank: int | None
    order: int | None
    parent_rank: int | None
    age_days: int | None
    age_degree_days: float | None
    declared_length: float | None
    area: float | None
    dry_biomass: float | None
    is_fruit: bool | None
    is_root: bool | None
    is_stem_truss: bool | None


@dataclass(frozen=True)
class PlantBaseProperties:
    row: int
    position: int
    age_days: int
    age_degree_days: float
    initial_angle: float
    internode_count: float | None
    leaf_area: float | None


@dataclass(frozen=True)
class RootProperties:
    pass


@dataclass(frozen=True)
class InternodeProperties:
    diameter: float
    length_increment_daily: float | None
    effective_length: float
    effective_length_source: str


@dataclass(frozen=True)
class LeafProperties:
    blade_count: int
    petiole_length: float
    petiole_diameter: float
    petiolule_diameter: float
    rachis_diameter: float
    petiole_angle: float
    petiole_azimuth: float
    curvature: float
    blade_area_total: float
    rachis_segment_lengths: tuple[float, ...] | None
    petiolule_lengths: tuple[float, ...] | None
    blade_areas: tuple[float, ...] | None
    petiolule_inclinations: tuple[float, ...] | None
    segment_azimuths: tuple[float, ...] | None


@dataclass(frozen=True)
class TrussProperties:
    pass


@dataclass(frozen=True)
class FruitsProperties:
    fruit_count: int
    paired: bool
    pedicel_length: float
    rachis_segment_length: float
    fruit_radii: tuple[float, ...] | None
    fruit_degree_days: tuple[float, ...] | None
    rachis_bend_angle: float
    rachis_radius: float
    fruit_spacing_angle: float
    ripening_degree_days: float


@dataclass(frozen=True)
class MeristemProperties:
    has_auxiliary_bud: bool
    has_truss_bud: bool


OrganSpecificProperties: TypeAlias = (
    PlantBaseProperties
    | RootProperties
    | InternodeProperties
    | LeafProperties
    | TrussProperties
    | FruitsProperties
    | MeristemProperties
)


@dataclass(frozen=True)
class OrganRecord:
    id: str
    node_id: str
    organ_type: str
    common: CommonOrganProperties
    properties: OrganSpecificProperties
    primitive_ids: tuple[str, ...]
    attribute_source: str = "groimp_api"


@dataclass(frozen=True)
class TurtleOperation:
    id: str
    node_id: str
    operation: str
    parameters: dict[str, float]
    local_transform: Matrix4
    provenance: str = "groimp_api"


@dataclass(frozen=True)
class AxisGeometry:
    id: str
    owner_node_id: str
    organ_type: str
    role: str
    local_frame: Matrix4
    world_frame: Matrix4
    local_start: Vector3
    local_end: Vector3
    world_start: Vector3
    world_end: Vector3
    local_direction: Vector3
    world_direction: Vector3
    length: float
    radius: float
    length_source: str
    geometry_source: str


@dataclass(frozen=True)
class SphereGeometry:
    id: str
    owner_node_id: str
    organ_type: str
    role: str
    local_frame: Matrix4
    world_frame: Matrix4
    local_center: Vector3
    world_center: Vector3
    radius: float
    geometry_source: str


@dataclass(frozen=True)
class PlantState:
    metadata: PlantMetadata
    root_node_id: str
    nodes: tuple[PlantNode, ...]
    edges: tuple[PlantEdge, ...]
    organs: tuple[OrganRecord, ...]
    turtle_operations: tuple[TurtleOperation, ...]
    axes: tuple[AxisGeometry, ...]
    spheres: tuple[SphereGeometry, ...]
    diagnostics: dict[str, Any] = field(default_factory=dict)
    schema_version: str = PLANT_STATE_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        """Return the canonical JSON-ready structure."""

        return asdict(self)
