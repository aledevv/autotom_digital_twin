import os

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
VERSION_DIR  = os.path.dirname(SCRIPT_DIR)      
SRC_DIR      = os.path.dirname(VERSION_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

from dataclasses import dataclass, field
from plant_model.models import PlantSnapshot, InternodeNode, LeafNode, FruitsNode
from .plant_builder import PlantBuilder
from .config import SimulationConfig
from .constants import PHYLLOTAXIS


@dataclass
class StemSegmentParams:
    order: int
    rank: int
    length: float
    radius: float


@dataclass
class LeafParams:
    order: int
    rank: int
    organ_index: int
    parent_rank: int
    parent_order: int
    total_length: float
    radius_start: float
    radius_end: float
    azimuth: float


@dataclass
class FruitParams:
    parent_rank: int
    radii: list[float]
    ages_dd: list[float]
    truss_angle: float


@dataclass
class OrganConfig:
    """Per-organ toggles for physics + segmentation, used for stability testing."""
    physics: bool = False
    num_segments: int = 1


@dataclass
class BuildConfig:
    """Top-level knob panel — one flag/segment-count per organ type."""
    stem: OrganConfig = field(default_factory=lambda: OrganConfig(physics=False, num_segments=1))
    leaf: OrganConfig = field(default_factory=lambda: OrganConfig(physics=False, num_segments=2))
    fruit: OrganConfig = field(default_factory=lambda: OrganConfig(physics=False, num_segments=1))
    # branch, truss, etc. when they are added

# ---------------------------------------------------------------------
# HELPERS
# ---------------------------------------------------------------------

def _azimuth_for(node) -> float:
    ccw = getattr(node, "ccw_orientation", 0.0)
    if abs(ccw) > 1e-3:
        return ccw
    return (node.parent_rank * PHYLLOTAXIS) % 360.0


# --------------------------------------------------------------------- #
# EXTRACTORS: PlantSnapshot -> pure param dataclasses (one per organ type)
# --------------------------------------------------------------------- #

def extract_stem_segments(snapshot: PlantSnapshot) -> list[StemSegmentParams]:
    nodes = [n for n in snapshot.organs if isinstance(n, InternodeNode)]
    return [
        StemSegmentParams(
            order=n.key.order, rank=n.key.rank,
            length=n.length, radius=n.width_m / 2.0,
        )
        for n in nodes
    ]


def extract_leaf_params(snapshot: PlantSnapshot) -> list[LeafParams]:
    params = []
    for n in snapshot.organs:
        if not isinstance(n, LeafNode):
            continue
        radius_start = n.diameter_petiole / 2.0
        params.append(LeafParams(
            order=n.key.order, rank=n.key.rank, organ_index=n.key.organ_index,
            parent_order=0,           # leaves always attach to the main trunk (order=0)
            parent_rank=n.parent_rank,
            total_length=n.length_petiole + n.rachis_length,
            radius_start=radius_start,
            radius_end=radius_start * 0.5,
            azimuth=_azimuth_for(n),
        ))
    return params


def extract_fruit_params(snapshot: PlantSnapshot) -> list[FruitParams]:
    return [
        FruitParams(
            parent_rank=n.parent_rank,
            radii=n.fruit_radii,
            ages_dd=n.fruit_age_dd,
            truss_angle=n.truss_angle,
        )
        for n in snapshot.organs if isinstance(n, FruitsNode)
    ]


# --------------------------------------------------------------------- #
# ORCHESTRATOR: extractors -> builder calls, single entry point
# --------------------------------------------------------------------- #

def build_plant_from_snapshot(snapshot, builder: PlantBuilder, config: SimulationConfig):
    
    # Any organ physics-enabled that attaches to the stem requires the
    # stem segments to be valid RigidBody anchors, even if the stem
    # itself stays fixed/non-articulated.
    needs_stem_anchor = (
        config.stem.physics_enabled
        or config.leaf.physics_enabled
        or config.branch.physics_enabled
        or config.fruit.physics_enabled
    )

    stem_segments = extract_stem_segments(snapshot)
    rank_to_id = builder.add_main_stem_segments(
        "Stem",
        segments=[vars(s) for s in stem_segments],
        mass_per_segment=config.stem.mass_per_segment,
        physics=needs_stem_anchor,
    )

    for leaf in extract_leaf_params(snapshot):
        parent_id = rank_to_id.get((leaf.parent_order, leaf.parent_rank))
        if parent_id is None:
            print(f"[WARN] Leaf o{leaf.order}_r{leaf.rank}_i{leaf.organ_index}: "
                f"parent (order={leaf.parent_order}, rank={leaf.parent_rank}) not found, skipping.")
            continue
        leaf_id = f"Leaf_o{leaf.order}_r{leaf.rank}_i{leaf.organ_index}"
        builder.add_leaf(
            parent_id=parent_id,
            base_id=leaf_id,
            total_length=leaf.total_length,
            radius_start=leaf.radius_start,
            radius_end=leaf.radius_end,
            rot_around_parent=leaf.azimuth,
            num_segments=config.leaf.max_segments,
            physics=config.leaf.physics_enabled,
            stiffness_base=config.leaf.stiffness_base,
            stiffness_tip=config.leaf.stiffness_tip,
            damping_ratio=config.leaf.damping_ratio,
            max_bend_angle=config.leaf.max_bend_angle,
            twist_limit=config.leaf.twist_limit,
            density=config.leaf.density,
        )