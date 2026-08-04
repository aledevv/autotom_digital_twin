# This file contains the data definition of the Tomato Plant

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional

# --- Identity key (immutable, usable as dict key) ---

@dataclass(frozen=True)
class OrganKey:
    """Unique identifier for an organ at a given day."""
    day: int
    plant_id: int
    order: int
    rank: int
    organ_class: str
    organ_index: int


# --- Base organ node ---

@dataclass
class OrganNode:
    """
    Represents a single plant organ at a specific simulation day.
    Fields common to all organ types (Internode, Leaf, Fruits, Root).
    """
    key: OrganKey
    parent_rank: int
    parent_organ_class: str

    # Physiological state
    age_dd: float
    dry_biomass_mg: float
    area_m2: float
    length: float

    # Type flags
    is_fruit: bool
    is_root: bool

    # Hierarchy links (populated after all nodes are created)
    parent: Optional[OrganNode] = field(default=None, repr=False)
    children: list[OrganNode] = field(default_factory=list, repr=False)


# --- Organ-specific subclasses ---

@dataclass
class InternodeNode(OrganNode):
    """Internode: cylindrical stem segment."""
    width_m: float = 0.0


@dataclass
class LeafNode(OrganNode):
    """Compound leaf with petiole, rachis and leaflets."""
    length_petiole: float = 0.0
    diameter_petiole: float = 0.003
    angle_petiole: float = 0.0
    ccw_orientation: float = 0.0
    curvature: float = 0.0
    blades_nr: int = 1
    area_blades_total: float = 0.0
    rachis_length: float = 0.0
    leaf_segments_length: list[float] = field(default_factory=list)
    leaf_area_m2blades: list[float] = field(default_factory=list)
    leaf_inclination_segments: list[float] = field(default_factory=list)


@dataclass
class FruitsNode(OrganNode):
    """Fruit truss with one or more individual fruits."""
    fruit_nr: int = 0
    fruit_radii: list[float] = field(default_factory=list)
    fruit_age_dd: list[float] = field(default_factory=list)
    ripening_dd: float = 0.0
    truss_angle: float = 9.0


@dataclass
class RootNode(OrganNode):
    """Root — no additional geometry parameters."""
    pass


# --- Plant snapshot (one plant, one day) ---

@dataclass
class PlantSnapshot:
    """
    All organs of a single plant at a single simulation day.
    by_key allows O(1) lookup by OrganKey.
    """
    day: int
    plant_id: int
    organs: list[OrganNode] = field(default_factory=list)
    by_key: dict[OrganKey, OrganNode] = field(default_factory=dict)
    by_position: dict[tuple, list(OrganNode)] = field(default_factory=dict)
