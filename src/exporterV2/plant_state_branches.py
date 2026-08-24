"""Conservative PlantState adapters for the established ExporterV2 backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from plant_state import AxisGeometry, OrganRecord, PlantState, validate_plant_state


POSE_MODES = ("canonical", "legacy")


class PlantStateBranchesError(ValueError):
    """Raised when PlantState cannot be represented by the legacy V2 contract."""


@dataclass(frozen=True)
class StemBranchesResult:
    branches: tuple[dict[str, Any], ...]
    source_axis_ids: tuple[str, ...]
    represented_organ_ids: tuple[str, ...]
    collapsed_duplicates: tuple[dict[str, Any], ...]
    pose_mode: str


def _rebased_frame(axis: AxisGeometry, origin) -> list[list[float]]:
    frame = [list(row) for row in axis.world_frame]
    for row in range(3):
        frame[row][3] = float(axis.world_start[row] - origin[row])
    return [[float(value) for value in row] for row in frame]


def _exact_duplicate_key(
    organ: OrganRecord,
    axis: AxisGeometry,
    parent_id: str | None,
) -> tuple[Any, ...]:
    return (
        organ.organ_type,
        parent_id,
        repr(asdict(organ.common)),
        repr(asdict(organ.properties)),
        tuple(axis.world_start),
        tuple(axis.world_end),
        tuple(tuple(row) for row in axis.world_frame),
        axis.length,
        axis.radius,
    )


def build_stem_branches(
    state: PlantState,
    *,
    pose_mode: str = "canonical",
) -> StemBranchesResult:
    """Return one fixed ``trunk`` BRANCHES chain backed by canonical internodes."""

    validate_plant_state(state)
    if pose_mode not in POSE_MODES:
        raise PlantStateBranchesError(
            f"pose_mode must be one of {POSE_MODES}, got {pose_mode!r}"
        )

    nodes = {node.id: node for node in state.nodes}
    organs = {organ.node_id: organ for organ in state.organs}
    root = nodes.get(state.root_node_id)
    if root is None:
        raise PlantStateBranchesError(f"missing PlantBase root {state.root_node_id}")

    selected = []
    for axis in state.axes:
        if axis.role != "internode":
            continue
        organ = organs.get(axis.owner_node_id)
        if organ is None or organ.organ_type != "Internode":
            raise PlantStateBranchesError(
                f"axis {axis.id} has no matching Internode organ"
            )
        if int(organ.common.order or 0) == 0:
            selected.append((organ, axis, nodes[axis.owner_node_id]))

    if not selected:
        raise PlantStateBranchesError("PlantState has no order=0 Internode axes")
    selected.sort(
        key=lambda item: (
            item[0].common.rank is None,
            item[0].common.rank if item[0].common.rank is not None else 0,
            item[2].groimp_node_id,
            item[1].id,
        )
    )
    ranks = [item[0].common.rank for item in selected]
    if any(rank is None for rank in ranks) or len(set(ranks)) != len(ranks):
        raise PlantStateBranchesError(
            f"main-stem Internode ranks must be present and unique, got {ranks}"
        )

    unique = []
    duplicate_groups: dict[tuple[Any, ...], list[tuple]] = {}
    for item in selected:
        organ, axis, node = item
        key = _exact_duplicate_key(organ, axis, node.parent_id)
        duplicate_groups.setdefault(key, []).append(item)
    collapsed = []
    for group in duplicate_groups.values():
        primary = min(group, key=lambda item: item[2].groimp_node_id)
        unique.append((primary, group))
        if len(group) > 1:
            collapsed.append(
                {
                    "kept": primary[0].id,
                    "represented": [item[0].id for item in group],
                    "reason": "exact type/parent/attributes/pose duplicate",
                }
            )
    unique.sort(key=lambda item: selected.index(item[0]))

    origin = root.pose.world_start
    link_specs = []
    visual_segments = []
    represented = []
    source_axis_ids = []
    for (organ, axis, node), group in unique:
        organ_ids = [item[0].id for item in group]
        represented.extend(organ_ids)
        source_axis_ids.extend(item[1].id for item in group)
        spec = {
            "id": f"Internode_g{node.groimp_node_id}",
            "groimp_node_id": int(node.groimp_node_id),
            "canonical_organ_id": organ.id,
            "canonical_axis_id": axis.id,
            "represented_organ_ids": organ_ids,
            "length": float(axis.length),
            "radius": float(axis.radius),
        }
        if pose_mode == "canonical":
            spec["rest_frame"] = _rebased_frame(axis, origin)
        link_specs.append(spec)
        visual_segments.append(
            {
                "source_id": spec["id"],
                "length": float(axis.length),
                "radius": float(axis.radius),
            }
        )

    lengths = [spec["length"] for spec in link_specs]
    radii = [spec["radius"] for spec in link_specs]
    branch = {
        "id": "trunk",
        "kind": "stem",
        "system": "vegetative",
        "visual_axis_id": "trunk",
        "visual_segments": visual_segments,
        "parent": None,
        "attach_link": None,
        "n_links": len(link_specs),
        "radius": sum(radii) / len(radii),
        "height": sum(lengths) / len(lengths),
        "tilt": 0.0,
        "rot": 0.0,
        "joint_type": "fixed",
        "link_specs": link_specs,
        "source_origin": [float(value) for value in origin],
        "plant_state_schema": state.schema_version,
    }
    return StemBranchesResult(
        branches=(branch,),
        source_axis_ids=tuple(source_axis_ids),
        represented_organ_ids=tuple(represented),
        collapsed_duplicates=tuple(collapsed),
        pose_mode=pose_mode,
    )

