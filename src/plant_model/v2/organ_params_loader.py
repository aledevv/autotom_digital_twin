import os

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
VERSION_DIR  = os.path.dirname(SCRIPT_DIR)      
SRC_DIR      = os.path.dirname(VERSION_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

from dataclasses import dataclass, field
from plant_model.models import PlantSnapshot, InternodeNode, LeafNode, FruitsNode
from .plant_builder import PlantBuilder


@dataclass
class StemParams:
    total_length: float
    radius: float
    num_internodes: int


@dataclass
class LeafParams:
    parent_rank: int
    petiole_length: float
    petiole_radius: float
    blade_area_total: float
    angle: float


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

# --------------------------------------------------------------------- #
# EXTRACTORS: PlantSnapshot -> pure param dataclasses (one per organ type)
# --------------------------------------------------------------------- #

def extract_stem_params(snapshot: PlantSnapshot) -> StemParams:
    nodes = sorted(
        (n for n in snapshot.organs if isinstance(n, InternodeNode) and n.key.order == 0),
        key=lambda n: n.key.rank,
    )
    if not nodes:
        raise ValueError("No main-stem internodes found")
    return StemParams(
        total_length=sum(n.length for n in nodes),
        radius=sum(n.width_m / 2.0 for n in nodes) / len(nodes), # this is mean radius of all nodes
        num_internodes=len(nodes),
    )


def extract_leaf_params(snapshot: PlantSnapshot) -> list[LeafParams]:
    return [
        LeafParams(
            parent_rank=n.parent_rank,
            petiole_length=n.length_petiole,
            petiole_radius=n.diameter_petiole / 2.0,
            blade_area_total=n.area_blades_total,
            angle=n.angle_petiole,
        )
        for n in snapshot.organs if isinstance(n, LeafNode)
    ]


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

def build_plant_from_snapshot(snapshot, builder: PlantBuilder, config: BuildConfig = BuildConfig()):
    stem = extract_stem_params(snapshot)
    builder.add_main_stem(
        "Stem_01",
        total_length=stem.total_length,
        radius=stem.radius,
        physics=config.stem.physics_enabled,
        max_segments=config.stem.max_segments,
    )

    # for i, leaf in enumerate(extract_leaf_params(snapshot)):
    #     builder.add_leaf(
    #         parent_id=f"Stem_{leaf.parent_rank:02d}",
    #         id=f"Leaf_{i:03d}",
    #         total_length=leaf.petiole_length + leaf.rachis_length,
    #         radius_start=leaf.petiole_radius,
    #         radius_end=leaf.petiole_radius * 0.6,
    #         num_segments=config.leaf.num_segments,
    #         physics=config.leaf.physics_enabled,
    #     )

    # for i, fruit in enumerate(extract_fruit_params(snapshot)):
    #     builder.add_fruit(
    #         parent_id=f"Stem_{fruit.parent_rank:02d}",
    #         id=f"Fruit_{i:03d}",
    #         physics=config.fruit.physics_enabled,
    #    )