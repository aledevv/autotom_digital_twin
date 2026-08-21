"""Lossless PlantState view used by the V1 static renderer."""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
import json
from types import SimpleNamespace

from plant_state import AxisGeometry, OrganRecord, PlantNode, PlantState, SphereGeometry
from plant_state import LeafProperties, validate_plant_state


class V1TopologyError(ValueError):
    """Raised when V1 cannot preserve canonical organ identity/topology."""


@dataclass(frozen=True)
class V1OrganView:
    node: PlantNode
    organ: OrganRecord
    axes: tuple[AxisGeometry, ...]
    spheres: tuple[SphereGeometry, ...]
    render_geometry: bool = True
    duplicate_of: str | None = None


@dataclass(frozen=True)
class V1RenderView:
    state: PlantState
    organs: tuple[V1OrganView, ...]
    organ_counts: dict[str, int]
    diagnostics: dict[str, object]


def _visual_signature(view: V1OrganView) -> str:
    """Identity-free exact signature for coincident V1 visual geometry."""

    node = view.node
    organ = view.organ
    payload = {
        "organ_type": organ.organ_type,
        "incoming_world": node.pose.incoming_world,
        "properties": asdict(organ.properties),
        "axes": [
            {
                key: value
                for key, value in asdict(axis).items()
                if key not in {"id", "owner_node_id"}
            }
            for axis in view.axes
        ],
        "spheres": [
            {
                key: value
                for key, value in asdict(sphere).items()
                if key not in {"id", "owner_node_id"}
            }
            for sphere in view.spheres
        ],
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)


def build_v1_render_view(state: PlantState) -> V1RenderView:
    """Build a complete organ view with exact visual deduplication metadata."""

    validate_plant_state(state)
    nodes = {node.id: node for node in state.nodes}
    axes: dict[str, list[AxisGeometry]] = defaultdict(list)
    spheres: dict[str, list[SphereGeometry]] = defaultdict(list)
    for primitive in state.axes:
        axes[primitive.owner_node_id].append(primitive)
    for primitive in state.spheres:
        spheres[primitive.owner_node_id].append(primitive)

    views = tuple(
        V1OrganView(
            node=nodes[organ.node_id],
            organ=organ,
            axes=tuple(sorted(axes.get(organ.node_id, ()), key=lambda item: item.id)),
            spheres=tuple(sorted(spheres.get(organ.node_id, ()), key=lambda item: item.id)),
        )
        for organ in sorted(state.organs, key=lambda item: item.node_id)
    )

    # Preserve every organ prim and topology record, but render only one copy
    # of visual geometry that would be exactly coincident. This is deliberately
    # exact: near overlaps and organs with different visual parameters survive.
    visual_types = {"Root", "Internode", "Leaf", "Fruits"}
    first_by_signature: dict[str, str] = {}
    duplicate_of: dict[str, str] = {}
    rendered_views: list[V1OrganView] = []
    for view in views:
        if view.organ.organ_type not in visual_types:
            rendered_views.append(view)
            continue
        signature = _visual_signature(view)
        original = first_by_signature.setdefault(signature, view.node.id)
        if original == view.node.id:
            rendered_views.append(view)
        else:
            duplicate_of[view.node.id] = original
            rendered_views.append(
                V1OrganView(
                    node=view.node,
                    organ=view.organ,
                    axes=view.axes,
                    spheres=view.spheres,
                    render_geometry=False,
                    duplicate_of=original,
                )
            )
    views = tuple(rendered_views)

    counts = Counter(view.organ.organ_type for view in views)
    zero_area_leaves = [
        view.node.id
        for view in views
        if isinstance(view.organ.properties, LeafProperties)
        and view.organ.properties.blade_area_total <= 0.0
    ]
    return V1RenderView(
        state=state,
        organs=views,
        organ_counts=dict(sorted(counts.items())),
        diagnostics={
            "filtering_applied": bool(duplicate_of),
            "filtering_policy": "exact_coincident_visual_geometry_only",
            "duplicate_geometry_of": dict(sorted(duplicate_of.items())),
            "zero_area_leaf_node_ids": sorted(zero_area_leaves),
        },
    )


def legacy_leaf_view(view: V1OrganView) -> SimpleNamespace:
    """Expose canonical leaf data through the small interface used by V1 visuals."""

    properties = view.organ.properties
    if not isinstance(properties, LeafProperties):
        raise TypeError("legacy_leaf_view requires a Leaf organ")
    common = view.organ.common
    segment_lengths = list(properties.rachis_segment_lengths or ())
    petiolule_lengths = list(properties.petiolule_lengths or ())
    return SimpleNamespace(
        key=SimpleNamespace(
            rank=common.rank or 0,
            order=common.order or 0,
            organ_index=view.node.groimp_node_id,
        ),
        length_petiole=properties.petiole_length,
        diameter_petiole=properties.petiole_diameter,
        angle_petiole=properties.petiole_angle,
        ccw_orientation=properties.petiole_azimuth,
        curvature=properties.curvature,
        blades_nr=properties.blade_count,
        area_blades_total=properties.blade_area_total,
        rachis_length=sum(segment_lengths) + sum(petiolule_lengths),
        leaf_segments_length=segment_lengths,
        leaf_area_m2blades=list(properties.blade_areas or ()),
        leaf_inclination_segments=list(properties.petiolule_inclinations or ()),
    )
