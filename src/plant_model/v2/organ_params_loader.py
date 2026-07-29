from dataclasses import dataclass

from plant_model.models import PlantSnapshot, InternodeNode, LeafNode
from .plant_builder import PlantBuilder
from .config import SimulationConfig
from .constants import PHYLLOTAXIS


# --------------------------------------------------------------------- #
# PARAM DATACLASSES
# --------------------------------------------------------------------- #

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
    total_length: float       # petiole length only (rachis dropped)
    radius_start: float
    radius_end: float
    tilt_angle: float         # degrees from parent axis; 90°=horizontal
    azimuth: float
    z_offset_ratio: float     # where along the trunk parent to attach (1.0 = tip)


# --------------------------------------------------------------------- #
# HELPERS
# --------------------------------------------------------------------- #

def _azimuth_for(node) -> float:
    ccw = getattr(node, "ccw_orientation", 0.0)
    if abs(ccw) > 1e-3:
        return ccw
    return (node.key.rank * PHYLLOTAXIS) % 360.0


# --------------------------------------------------------------------- #
# EXTRACTORS: PlantSnapshot -> pure param dataclasses
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


def extract_leaf_params(snapshot: PlantSnapshot, internodes: list[StemSegmentParams]) -> list[LeafParams]:
    trunk_len: dict[int, float] = {}
    lateral_len: dict[tuple, float] = {}
    for seg in internodes:
        if seg.order == 0:
            trunk_len[seg.rank] = seg.length
        else:
            key = (seg.order, seg.rank)
            if key not in lateral_len:
                lateral_len[key] = seg.length

    params = []
    seen: set[tuple] = set()

    for n in snapshot.organs:
        if not isinstance(n, LeafNode):
            continue
        radius_start = n.diameter_petiole / 2.0
        tilt = n.angle_petiole
        azimuth = _azimuth_for(n)

        dedup_key = (n.key.order, n.parent_rank, round(azimuth, 2))
        if dedup_key in seen:
            print(f"[SKIP] Leaf o{n.key.order}_r{n.key.rank}_i{n.key.organ_index}: "
                  f"duplicate at parent_rank={n.parent_rank} azimuth={azimuth:.1f}°")
            continue
        seen.add(dedup_key)

        if n.key.order == 0:
            z_off = 1.0
        else:
            lat = lateral_len.get((n.key.order, n.parent_rank), 0.0)
            trk = trunk_len.get(n.parent_rank, 1.0)
            z_off = lat / trk if trk > 0 else 1.0

        print(f"[Leaf o{n.key.order}_r{n.key.rank}_i{n.key.organ_index}] "
              f"angle_petiole={tilt:.1f}° azimuth={azimuth:.1f}° z_offset_ratio={z_off:.3f}")
        params.append(LeafParams(
            order=n.key.order, rank=n.key.rank, organ_index=n.key.organ_index,
            parent_order=0,
            parent_rank=n.parent_rank,
            total_length=n.length_petiole,
            radius_start=radius_start,
            radius_end=radius_start * 0.5,
            tilt_angle=tilt,
            azimuth=azimuth,
            z_offset_ratio=z_off,
        ))
    return params


# --------------------------------------------------------------------- #
# ORCHESTRATOR
# --------------------------------------------------------------------- #

def build_plant_from_snapshot(snapshot: PlantSnapshot, builder: PlantBuilder, config: SimulationConfig):
    needs_stem_anchor = (
        config.stem.physics_enabled
        or config.leaf.physics_enabled
    )

    stem_segments = extract_stem_segments(snapshot)
    rank_to_id = builder.add_main_stem_segments(
        "Stem",
        segments=[vars(s) for s in stem_segments],
        mass_per_segment=config.stem.mass_per_segment,
        physics=needs_stem_anchor,
    )

    for leaf in extract_leaf_params(snapshot, stem_segments):
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
            z_offset_ratio=leaf.z_offset_ratio,
            tilt_angle=leaf.tilt_angle,
            rot_around_parent=leaf.azimuth,
            num_petiole_segments=config.leaf.num_petiole_segments,
            physics=config.leaf.physics_enabled,
            stiffness_base=config.leaf.stiffness_base,
            stiffness_tip=config.leaf.stiffness_tip,
            damping_ratio=config.leaf.damping_ratio,
            max_bend_angle=config.leaf.max_bend_angle,
            twist_limit=config.leaf.twist_limit,
            density=config.leaf.density,
        )
