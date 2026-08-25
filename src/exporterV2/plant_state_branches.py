"""Conservative PlantState adapters for the established ExporterV2 backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import math
import random
from typing import Any

from plant_state import (
    AxisGeometry,
    OrganRecord,
    PlantState,
    SphereGeometry,
    validate_plant_state,
)

from .core.tree_config import TrussGeometryConfig, TrussPhysicsConfig


POSE_MODES = ("canonical", "legacy")
APPENDAGE_POSE_MODES = ("v2-aesthetic", "canonical")
LEAF_JOINT_POLICIES = ("optimized", "distributed")
VISUAL_QUALITY_MODES = ("realistic", "performance")
LATERAL_JOINT_POLICIES = ("dynamic", "fixed")
TRUSS_CALIBRATION_PRESETS = ("current", "compliant", "balanced", "firm")
TRUSS_DAMPING_CHOICES = (1.0, 2.0, 4.0, 7.0)

_TRUSS_CALIBRATION_VALUES = {
    "compliant": {
        "rachis_young_modulus": 10.0e9,
        "pedicel_young_modulus": 2.0e9,
        "damping_ratio": 1.0,
        "pedicel_drive_scale": 0.2,
    },
    "balanced": {
        "rachis_young_modulus": 20.0e9,
        "pedicel_young_modulus": 4.0e9,
        "damping_ratio": 2.0,
        "pedicel_drive_scale": 0.2,
    },
    "firm": {
        "rachis_young_modulus": 40.0e9,
        "pedicel_young_modulus": 8.0e9,
        "damping_ratio": 2.0,
        "pedicel_drive_scale": 0.2,
    },
}

# This is the established V2 skinning profile. Keep it as the PlantState
# default: although canonical axes already provide the centerline, dense axial
# sampling and radius-transition rings are what give branches their organic V2
# silhouette instead of a visibly faceted/cylindrical appearance.
PLANT_STATE_REALISTIC_VISUAL_PROFILE = {
    "radial_segments": 14,
    "axial_spacing_m": 0.005,
    "radius_transition_samples": 9,
}

# Explicit fallback for very large plants or slower GPUs. This is never chosen
# implicitly, so a performance optimization cannot silently degrade visuals.
PLANT_STATE_PERFORMANCE_VISUAL_PROFILE = {
    "radial_segments": 12,
    "axial_spacing_m": 0.012,
    "radius_transition_samples": 5,
}


def _segmented_visual_profile(visual_quality: str) -> dict[str, float | int]:
    if visual_quality not in VISUAL_QUALITY_MODES:
        raise PlantStateBranchesError(
            f"visual_quality must be one of {VISUAL_QUALITY_MODES}, "
            f"got {visual_quality!r}"
        )
    source = (
        PLANT_STATE_REALISTIC_VISUAL_PROFILE
        if visual_quality == "realistic"
        else PLANT_STATE_PERFORMANCE_VISUAL_PROFILE
    )
    return dict(source)


class PlantStateBranchesError(ValueError):
    """Raised when PlantState cannot be represented by the legacy V2 contract."""


@dataclass(frozen=True)
class StemBranchesResult:
    branches: tuple[dict[str, Any], ...]
    source_axis_ids: tuple[str, ...]
    represented_organ_ids: tuple[str, ...]
    collapsed_duplicates: tuple[dict[str, Any], ...]
    pose_mode: str
    debug_profile: str = "stem"
    attachment_map: tuple[dict[str, Any], ...] = ()
    degenerate_organs: tuple[dict[str, Any], ...] = ()
    approved_collision_filters: tuple[dict[str, Any], ...] = ()
    rigid_leaf_visuals: tuple[dict[str, Any], ...] = ()
    terminal_bodies: tuple[dict[str, Any], ...] = ()
    leaf_joint_policy: str | None = None
    physical_petiolules: bool = False
    visual_quality: str = "realistic"
    appendage_pose_mode: str = "v2-aesthetic"
    lateral_joint_policy: str = "dynamic"
    truss_calibration_preset: str = "current"
    truss_damping_override: float | None = None


def apply_checkpoint_physics_policy(
    result: StemBranchesResult,
    *,
    lateral_joint_policy: str = "dynamic",
    truss_calibration_preset: str = "current",
    truss_damping_override: float | None = None,
) -> StemBranchesResult:
    """Apply diagnostic policies to PlantState BRANCHES without mutating input."""

    if lateral_joint_policy not in LATERAL_JOINT_POLICIES:
        raise PlantStateBranchesError(
            "lateral_joint_policy must be one of "
            f"{LATERAL_JOINT_POLICIES}, got {lateral_joint_policy!r}"
        )
    if truss_calibration_preset not in TRUSS_CALIBRATION_PRESETS:
        raise PlantStateBranchesError(
            "truss_calibration_preset must be one of "
            f"{TRUSS_CALIBRATION_PRESETS}, got {truss_calibration_preset!r}"
        )
    if truss_damping_override is not None:
        truss_damping_override = float(truss_damping_override)
        if truss_damping_override not in TRUSS_DAMPING_CHOICES:
            raise PlantStateBranchesError(
                "truss_damping_override must be one of "
                f"{TRUSS_DAMPING_CHOICES}, got {truss_damping_override!r}"
            )

    if truss_calibration_preset == "current":
        calibration = {
            "rachis_young_modulus": float(
                TrussPhysicsConfig.RACHIS_YOUNG_MODULUS
            ),
            "pedicel_young_modulus": float(
                TrussPhysicsConfig.PEDICEL_YOUNG_MODULUS
            ),
            "rachis_damping_ratio": float(
                TrussPhysicsConfig.RACHIS_DAMPING_RATIO
            ),
            "pedicel_damping_ratio": float(
                TrussPhysicsConfig.PEDICEL_DAMPING_RATIO
            ),
            "pedicel_drive_scale": float(
                TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE
            ),
        }
    else:
        source = _TRUSS_CALIBRATION_VALUES[truss_calibration_preset]
        calibration = {
            **source,
            "rachis_damping_ratio": source["damping_ratio"],
            "pedicel_damping_ratio": source["damping_ratio"],
        }
    if truss_damping_override is not None:
        calibration["rachis_damping_ratio"] = truss_damping_override
        calibration["pedicel_damping_ratio"] = truss_damping_override

    branches = []
    for source_branch in result.branches:
        branch = dict(source_branch)
        if branch.get("kind") == "lateral_branch" and lateral_joint_policy == "fixed":
            branch["joint_type"] = "fixed"
            branch["attachment_joint_type"] = "fixed"
        component = branch.get("truss_component")
        if component == "rachis":
            branch["young_modulus"] = calibration["rachis_young_modulus"]
            branch["damping_ratio"] = calibration["rachis_damping_ratio"]
            branch["density"] = float(TrussPhysicsConfig.PLANT_DENSITY)
        elif component == "pedicel":
            branch["young_modulus"] = calibration["pedicel_young_modulus"]
            branch["damping_ratio"] = calibration["pedicel_damping_ratio"]
            branch["drive_stiffness_scale"] = calibration["pedicel_drive_scale"]
            branch["density"] = float(TrussPhysicsConfig.PLANT_DENSITY)
        branches.append(branch)

    return replace(
        result,
        branches=tuple(branches),
        lateral_joint_policy=lateral_joint_policy,
        truss_calibration_preset=truss_calibration_preset,
        truss_damping_override=truss_damping_override,
    )


def _rebased_frame(axis: AxisGeometry, origin) -> list[list[float]]:
    frame = [list(row) for row in axis.world_frame]
    for row in range(3):
        frame[row][3] = float(axis.world_start[row] - origin[row])
    return [[float(value) for value in row] for row in frame]


def _matmul3(left, right) -> list[list[float]]:
    return [
        [
            sum(float(left[row][inner]) * float(right[inner][column]) for inner in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _v2_appendage_frame(
    source: AxisGeometry,
    parent: AxisGeometry,
    origin,
    *,
    tilt_deg: float,
    rotation_deg: float,
) -> list[list[float]]:
    """Return the old V2 local tilt/azimuth recipe in PlantState convention."""

    tilt = math.radians(float(tilt_deg))
    rotation = math.radians(float(rotation_deg))
    cos_tilt, sin_tilt = math.cos(tilt), math.sin(tilt)
    cos_rot, sin_rot = math.cos(rotation), math.sin(rotation)
    rotate_z = (
        (cos_rot, -sin_rot, 0.0),
        (sin_rot, cos_rot, 0.0),
        (0.0, 0.0, 1.0),
    )
    # The historical resolver applies a negative rotation around local X.
    rotate_x = (
        (1.0, 0.0, 0.0),
        (0.0, cos_tilt, sin_tilt),
        (0.0, -sin_tilt, cos_tilt),
    )
    parent_rotation = [list(row[:3]) for row in parent.world_frame[:3]]
    rotation_world = _matmul3(
        parent_rotation,
        _matmul3(rotate_z, rotate_x),
    )
    result = [
        [rotation_world[row][column] for column in range(3)]
        + [float(source.world_start[row] - origin[row])]
        for row in range(3)
    ]
    result.append([0.0, 0.0, 0.0, 1.0])
    return result


def _validate_appendage_pose_mode(value: str) -> None:
    if value not in APPENDAGE_POSE_MODES:
        raise PlantStateBranchesError(
            f"appendage_pose_mode must be one of {APPENDAGE_POSE_MODES}, got {value!r}"
        )


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


def _internode_items(state: PlantState):
    nodes = {node.id: node for node in state.nodes}
    organs = {organ.node_id: organ for organ in state.organs}
    result = {}
    for axis in state.axes:
        if axis.role != "internode":
            continue
        organ = organs.get(axis.owner_node_id)
        if organ is None or organ.organ_type != "Internode":
            raise PlantStateBranchesError(
                f"axis {axis.id} has no matching Internode organ"
            )
        node = nodes.get(axis.owner_node_id)
        if node is None:
            raise PlantStateBranchesError(
                f"axis {axis.id} references missing node {axis.owner_node_id}"
            )
        result[node.id] = (organ, axis, node)
    return result


def _deduplicate_items(items):
    duplicate_groups: dict[tuple[Any, ...], list[tuple]] = {}
    for item in items:
        organ, axis, node = item
        key = _exact_duplicate_key(organ, axis, node.parent_id)
        duplicate_groups.setdefault(key, []).append(item)

    unique = []
    collapsed = []
    canonical_node = {}
    for group in duplicate_groups.values():
        primary = min(group, key=lambda item: item[2].groimp_node_id)
        unique.append((primary, group))
        for item in group:
            canonical_node[item[2].id] = primary[2].id
        if len(group) > 1:
            collapsed.append(
                {
                    "kept": primary[0].id,
                    "represented": [item[0].id for item in group],
                    "reason": "exact type/parent/attributes/pose duplicate",
                }
            )
    return unique, collapsed, canonical_node


def _link_spec(item, group, origin, pose_mode: str) -> dict[str, Any]:
    organ, axis, node = item
    organ_ids = [candidate[0].id for candidate in group]
    spec = {
        "id": f"Internode_g{node.groimp_node_id}",
        "groimp_node_id": int(node.groimp_node_id),
        "canonical_node_id": node.id,
        "canonical_organ_id": organ.id,
        "canonical_axis_id": axis.id,
        "axis_role": axis.role,
        "represented_organ_ids": organ_ids,
        "length": float(axis.length),
        "radius": float(axis.radius),
    }
    if pose_mode == "canonical":
        spec["rest_frame"] = _rebased_frame(axis, origin)
    return spec


def _legacy_lateral_orientations(chain_roots, attachment_keys):
    """Reproduce the old V2 tomato tilt/azimuth policy deterministically."""

    from .profiles.tomato_default import TOMATO_PROFILE

    config = TOMATO_PROFILE["lateral_branches"]
    tilt = float(config.get("tilt_deg", 45.0))
    bases = tuple(float(value) for value in config.get("rot_base_deg", (0.0, 180.0)))
    jitter = float(config.get("rot_jitter_deg", 0.0))
    minimum = float(config.get("min_angle_separation_deg", 60.0))
    rotations_by_parent: dict[tuple[str, int], list[float]] = {}
    result = {}

    sibling_ordinals: dict[tuple[str, int], int] = {}
    for root_id in chain_roots:
        parent_branch, attach_link = attachment_keys[root_id]
        key = (parent_branch, attach_link)
        ordinal = sibling_ordinals.get(key, 0)
        sibling_ordinals[key] = ordinal + 1
        base = bases[ordinal] if ordinal < len(bases) else ordinal * 90.0
        if jitter > 0.0:
            rng = random.Random(attach_link * 1000 + ordinal)
            base = (base + rng.uniform(-jitter, jitter)) % 360.0
        rotation = base
        for _ in range(10):
            conflict = None
            for neighbor in (attach_link, attach_link - 1, attach_link + 1):
                for existing in rotations_by_parent.get((parent_branch, neighbor), ()):
                    difference = abs(rotation - existing)
                    difference = min(difference, 360.0 - difference)
                    if difference < minimum:
                        conflict = existing
                        break
                if conflict is not None:
                    break
            if conflict is None:
                break
            rotation = (conflict + minimum + 5.0) % 360.0
        rotations_by_parent.setdefault(key, []).append(rotation)
        result[root_id] = (tilt, rotation)
    return result


def build_stem_branches(
    state: PlantState,
    *,
    pose_mode: str = "canonical",
    visual_quality: str = "realistic",
) -> StemBranchesResult:
    """Return one fixed ``trunk`` BRANCHES chain backed by canonical internodes."""

    validate_plant_state(state)
    if pose_mode not in POSE_MODES:
        raise PlantStateBranchesError(
            f"pose_mode must be one of {POSE_MODES}, got {pose_mode!r}"
        )

    nodes = {node.id: node for node in state.nodes}
    root = nodes.get(state.root_node_id)
    if root is None:
        raise PlantStateBranchesError(f"missing PlantBase root {state.root_node_id}")

    selected = []
    for organ, axis, node in _internode_items(state).values():
        if int(organ.common.order or 0) == 0:
            selected.append((organ, axis, node))

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

    unique, collapsed, _ = _deduplicate_items(selected)
    unique.sort(key=lambda item: selected.index(item[0]))

    origin = root.pose.world_start
    link_specs = []
    visual_segments = []
    represented = []
    source_axis_ids = []
    for (organ, axis, node), group in unique:
        spec = _link_spec((organ, axis, node), group, origin, pose_mode)
        organ_ids = spec["represented_organ_ids"]
        represented.extend(organ_ids)
        source_axis_ids.extend(item[1].id for item in group)
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
        "visual_profile": _segmented_visual_profile(visual_quality),
    }
    return StemBranchesResult(
        branches=(branch,),
        source_axis_ids=tuple(source_axis_ids),
        represented_organ_ids=tuple(represented),
        collapsed_duplicates=tuple(collapsed),
        pose_mode=pose_mode,
        visual_quality=visual_quality,
    )


def build_lateral_branches(
    state: PlantState,
    *,
    pose_mode: str = "canonical",
    visual_quality: str = "realistic",
) -> StemBranchesResult:
    """Return the fixed stem plus native lateral Internode chains."""

    validate_plant_state(state)
    if pose_mode not in POSE_MODES:
        raise PlantStateBranchesError(
            f"pose_mode must be one of {POSE_MODES}, got {pose_mode!r}"
        )

    stem = build_stem_branches(
        state, pose_mode=pose_mode, visual_quality=visual_quality
    )
    nodes = {node.id: node for node in state.nodes}
    root = nodes[state.root_node_id]
    all_internodes = _internode_items(state)
    lateral_items = [
        item
        for item in all_internodes.values()
        if int(item[0].common.order or 0) > 0
    ]
    if not lateral_items:
        return StemBranchesResult(
            branches=stem.branches,
            source_axis_ids=stem.source_axis_ids,
            represented_organ_ids=stem.represented_organ_ids,
            collapsed_duplicates=stem.collapsed_duplicates,
            pose_mode=pose_mode,
            debug_profile="laterals",
            visual_quality=visual_quality,
        )

    unique, collapsed, canonical_node = _deduplicate_items(lateral_items)
    primary_items = {item[2].id: (item, group) for item, group in unique}

    stem_location = {}
    for index, spec in enumerate(stem.branches[0]["link_specs"], start=1):
        stem_location[spec["canonical_node_id"]] = ("trunk", index)

    def nearest_internode_ancestor(node_id: str) -> str:
        current = nodes[node_id].parent_id
        visited = {node_id}
        while current is not None:
            if current in visited:
                raise PlantStateBranchesError(
                    f"cycle while resolving Internode ancestor of {node_id}"
                )
            visited.add(current)
            if current in all_internodes:
                return canonical_node.get(current, current)
            parent = nodes.get(current)
            if parent is None:
                raise PlantStateBranchesError(
                    f"node {node_id} references missing ancestor {current}"
                )
            current = parent.parent_id
        raise PlantStateBranchesError(
            f"lateral Internode {node_id} has no structural Internode ancestor"
        )

    parent_internode = {}
    for node_id, ((organ, _axis, _node), _group) in primary_items.items():
        parent_id = nearest_internode_ancestor(node_id)
        if parent_id == node_id:
            raise PlantStateBranchesError(f"lateral Internode {node_id} is self-parented")
        parent_item = all_internodes.get(parent_id)
        if parent_item is None:
            raise PlantStateBranchesError(
                f"lateral Internode {node_id} has unsupported parent {parent_id}"
            )
        parent_order = int(parent_item[0].common.order or 0)
        order = int(organ.common.order or 0)
        if parent_order > order:
            raise PlantStateBranchesError(
                f"lateral Internode {node_id} order {order} descends from order {parent_order}"
            )
        parent_internode[node_id] = parent_id

    same_order_children: dict[str, list[str]] = {}
    for node_id, parent_id in parent_internode.items():
        organ = primary_items[node_id][0][0]
        parent_organ = all_internodes[parent_id][0]
        if int(organ.common.order or 0) == int(parent_organ.common.order or 0):
            same_order_children.setdefault(parent_id, []).append(node_id)
    for children in same_order_children.values():
        children.sort(key=lambda value: nodes[value].groimp_node_id)

    chain_starts = []
    for node_id, ((organ, _axis, node), _group) in primary_items.items():
        parent_id = parent_internode[node_id]
        parent_item = all_internodes[parent_id]
        same_order = int(parent_item[0].common.order or 0) == int(organ.common.order or 0)
        if not same_order or len(same_order_children.get(parent_id, ())) != 1:
            chain_starts.append(node_id)
    chain_starts.sort(
        key=lambda value: (
            int(primary_items[value][0][0].common.order or 0),
            nodes[value].groimp_node_id,
        )
    )

    chains = []
    covered = set()
    for start in chain_starts:
        chain = []
        current = start
        while current not in covered:
            chain.append(current)
            covered.add(current)
            children = same_order_children.get(current, ())
            if len(children) != 1:
                break
            current = children[0]
        chains.append(chain)
    if covered != set(primary_items):
        missing = sorted(set(primary_items) - covered)
        raise PlantStateBranchesError(
            f"could not assign lateral Internodes to native chains: {missing}"
        )

    node_location = dict(stem_location)
    provisional = []
    unresolved = list(chains)
    while unresolved:
        progress = False
        for chain in list(unresolved):
            parent_node_id = parent_internode[chain[0]]
            parent_location = node_location.get(parent_node_id)
            if parent_location is None:
                continue
            provisional_index = len(provisional)
            provisional_id = f"__lateral_{provisional_index}"
            provisional.append((provisional_id, chain, parent_location, parent_node_id))
            for link_index, node_id in enumerate(chain, start=1):
                node_location[node_id] = (provisional_id, link_index)
            unresolved.remove(chain)
            progress = True
        if not progress:
            blocked = [chain[0] for chain in unresolved]
            raise PlantStateBranchesError(
                f"cannot resolve lateral chain parents for {blocked}"
            )

    siblings: dict[tuple[str, int], list[int]] = {}
    for index, (_provisional_id, chain, parent_location, _parent_node_id) in enumerate(provisional):
        siblings.setdefault(parent_location, []).append(index)
    sibling_ordinal = {}
    for indices in siblings.values():
        indices.sort(key=lambda value: nodes[provisional[value][1][0]].groimp_node_id)
        for ordinal, index in enumerate(indices):
            sibling_ordinal[index] = ordinal

    final_ids = {}
    for index, (provisional_id, chain, (parent_branch, attach_link), _parent_node_id) in enumerate(provisional):
        root_groimp = nodes[chain[0]].groimp_node_id
        ordinal = sibling_ordinal[index]
        if parent_branch == "trunk":
            branch_id = f"Branch_s{attach_link}_o{ordinal}_g{root_groimp}"
        else:
            parent_root = nodes[provisional[int(parent_branch.rsplit("_", 1)[1])][1][0]].groimp_node_id
            branch_id = (
                f"Branch_b{parent_root}_l{attach_link}_o{ordinal}_g{root_groimp}"
            )
        final_ids[provisional_id] = branch_id

    attachment_keys = {
        chain[0]: (final_ids.get(parent_branch, parent_branch), attach_link)
        for _provisional_id, chain, (parent_branch, attach_link), _parent_node_id in provisional
    }
    legacy_orientations = _legacy_lateral_orientations(
        [chain[0] for _provisional_id, chain, _parent, _node in provisional],
        attachment_keys,
    )

    branches = list(stem.branches)
    source_axis_ids = list(stem.source_axis_ids)
    represented = list(stem.represented_organ_ids)
    attachment_map = []
    for provisional_id, chain, (parent_branch, attach_link), parent_node_id in provisional:
        branch_id = final_ids[provisional_id]
        resolved_parent = final_ids.get(parent_branch, parent_branch)
        specs = []
        visual_segments = []
        for node_id in chain:
            item, group = primary_items[node_id]
            spec = _link_spec(item, group, root.pose.world_start, pose_mode)
            specs.append(spec)
            visual_segments.append(
                {
                    "source_id": spec["id"],
                    "length": spec["length"],
                    "radius": spec["radius"],
                }
            )
            source_axis_ids.extend(candidate[1].id for candidate in group)
            represented.extend(candidate[0].id for candidate in group)
        lengths = [spec["length"] for spec in specs]
        radii = [spec["radius"] for spec in specs]
        tilt, rotation = legacy_orientations[chain[0]]
        branches.append(
            {
                "id": branch_id,
                "kind": "lateral_branch",
                "system": "vegetative",
                "visual_axis_id": branch_id,
                "visual_segments": visual_segments,
                "parent": resolved_parent,
                "attach_link": attach_link,
                "n_links": len(specs),
                "radius": sum(radii) / len(radii),
                "height": sum(lengths) / len(lengths),
                "tilt": tilt,
                "rot": rotation,
                "joint_type": "d6",
                "attachment_joint_type": "d6",
                "link_specs": specs,
                "source_origin": [float(value) for value in root.pose.world_start],
                "source_parent_node_id": parent_node_id,
                "plant_state_schema": state.schema_version,
                "visual_profile": _segmented_visual_profile(visual_quality),
            }
        )
        attachment_map.append(
            {
                "branch_id": branch_id,
                "root_node_id": chain[0],
                "root_groimp_node_id": nodes[chain[0]].groimp_node_id,
                "parent_branch_id": resolved_parent,
                "parent_node_id": parent_node_id,
                "parent_groimp_node_id": nodes[parent_node_id].groimp_node_id,
                "attach_link": attach_link,
                "link_node_ids": list(chain),
            }
        )

    return StemBranchesResult(
        branches=tuple(branches),
        source_axis_ids=tuple(source_axis_ids),
        represented_organ_ids=tuple(represented),
        collapsed_duplicates=tuple((*stem.collapsed_duplicates, *collapsed)),
        pose_mode=pose_mode,
        debug_profile="laterals",
        attachment_map=tuple(attachment_map),
        visual_quality=visual_quality,
    )


def _leaf_duplicate_key(organ, node, axes) -> tuple[Any, ...]:
    """Identity used by the explicit, lossless duplicate policy."""

    geometry = tuple(
        (
            axis.role,
            axis.id.rsplit(":", 1)[-1],
            tuple(axis.world_start),
            tuple(axis.world_end),
            tuple(tuple(row) for row in axis.world_frame),
            axis.length,
            axis.radius,
        )
        for axis in sorted(axes, key=lambda value: value.id)
    )
    return (
        organ.organ_type,
        node.parent_id,
        repr(asdict(organ.common)),
        repr(asdict(organ.properties)),
        tuple(tuple(row) for row in node.pose.incoming_world),
        tuple(tuple(row) for row in node.pose.outgoing_world),
        geometry,
    )


def _leaf_link_spec(
    organ,
    node,
    axis: AxisGeometry,
    represented_organ_ids: list[str],
    origin,
    pose_mode: str,
    spec_id: str,
) -> dict[str, Any]:
    spec = {
        "id": spec_id,
        "groimp_node_id": int(node.groimp_node_id),
        "canonical_node_id": node.id,
        "canonical_organ_id": organ.id,
        "canonical_axis_id": axis.id,
        "axis_role": axis.role,
        "represented_organ_ids": list(represented_organ_ids),
        "length": float(axis.length),
        "radius": float(axis.radius),
    }
    if pose_mode == "canonical":
        spec["rest_frame"] = _rebased_frame(axis, origin)
    return spec


def _legacy_leaf_orientation(organ, parent_branch: str, sibling_ordinal: int) -> tuple[float, float]:
    """Retain the old procedural leaf orientation for diagnostic comparison."""

    from .core.tree_config import PHYLLOTAXIS

    rank = int(organ.common.rank or 0)
    properties = organ.properties
    if int(organ.common.order or 0) == 0:
        azimuth = float(properties.petiole_azimuth)
        if abs(azimuth) <= 1e-3:
            azimuth = (rank * PHYLLOTAXIS) % 360.0
        return float(properties.petiole_angle), azimuth

    parent_ordinal = sibling_ordinal
    marker = "_o"
    if marker in parent_branch:
        try:
            parent_ordinal = int(parent_branch.split(marker, 1)[1].split("_", 1)[0])
        except ValueError:
            pass
    rng = random.Random(rank * 1000 + parent_ordinal)
    return 35.0, rng.uniform(-90.0, 90.0) % 360.0


def build_leaf_support_branches(
    state: PlantState,
    *,
    pose_mode: str = "canonical",
    leaf_joint_policy: str = "distributed",
    visual_quality: str = "realistic",
) -> StemBranchesResult:
    """Return stem, native laterals, petioles, and main leaf rachides."""

    validate_plant_state(state)
    if pose_mode not in POSE_MODES:
        raise PlantStateBranchesError(
            f"pose_mode must be one of {POSE_MODES}, got {pose_mode!r}"
        )
    if leaf_joint_policy not in LEAF_JOINT_POLICIES:
        raise PlantStateBranchesError(
            "leaf_joint_policy must be one of "
            f"{LEAF_JOINT_POLICIES}, got {leaf_joint_policy!r}"
        )

    structural = build_lateral_branches(
        state, pose_mode=pose_mode, visual_quality=visual_quality
    )
    nodes = {node.id: node for node in state.nodes}
    organs = {organ.node_id: organ for organ in state.organs}
    root = nodes[state.root_node_id]
    axes_by_owner: dict[str, list[AxisGeometry]] = {}
    for axis in state.axes:
        axes_by_owner.setdefault(axis.owner_node_id, []).append(axis)

    internode_location = {}
    for branch in structural.branches:
        for index, spec in enumerate(branch["link_specs"], start=1):
            if spec["axis_role"] == "internode":
                internode_location[spec["canonical_node_id"]] = (branch["id"], index)

    def nearest_internode(node_id: str) -> str:
        current = nodes[node_id].parent_id
        visited = {node_id}
        while current is not None:
            if current in visited:
                raise PlantStateBranchesError(
                    f"cycle while resolving leaf parent for {node_id}"
                )
            visited.add(current)
            organ = organs.get(current)
            if organ is not None and organ.organ_type == "Internode":
                if current not in internode_location:
                    raise PlantStateBranchesError(
                        f"leaf {node_id} resolves to excluded Internode {current}"
                    )
                return current
            parent = nodes.get(current)
            if parent is None:
                raise PlantStateBranchesError(
                    f"leaf {node_id} references missing ancestor {current}"
                )
            current = parent.parent_id
        raise PlantStateBranchesError(f"leaf {node_id} has no Internode ancestor")

    leaves = []
    for organ in state.organs:
        if organ.organ_type != "Leaf":
            continue
        node = nodes[organ.node_id]
        axes = axes_by_owner.get(node.id, [])
        petioles = [axis for axis in axes if axis.role == "petiole"]
        if len(petioles) != 1:
            raise PlantStateBranchesError(
                f"Leaf {node.id} must have exactly one petiole axis, found {len(petioles)}"
            )
        rachides = sorted(
            (axis for axis in axes if axis.role == "leaf_rachis"),
            key=lambda axis: int(axis.id.rsplit(":", 1)[1]),
        )
        leaves.append((organ, node, petioles[0], rachides, nearest_internode(node.id)))
    leaves.sort(
        key=lambda item: (
            int(item[0].common.order or 0),
            item[0].common.rank is None,
            item[0].common.rank or 0,
            item[1].groimp_node_id,
        )
    )

    duplicate_groups: dict[tuple[Any, ...], list[tuple]] = {}
    for item in leaves:
        organ, node, petiole, rachides, _parent = item
        duplicate_groups.setdefault(
            _leaf_duplicate_key(organ, node, [petiole, *rachides]), []
        ).append(item)

    selected = []
    collapsed = list(structural.collapsed_duplicates)
    for group in duplicate_groups.values():
        primary = min(group, key=lambda item: item[1].groimp_node_id)
        selected.append((primary, group))
        if len(group) > 1:
            collapsed.append(
                {
                    "kept": primary[0].id,
                    "represented": [item[0].id for item in group],
                    "reason": "exact type/parent/attributes/pose duplicate",
                }
            )
    selected.sort(key=lambda value: leaves.index(value[0]))

    sibling_groups: dict[tuple[str, int, int], list[int]] = {}
    selected_locations = []
    for index, ((organ, _node, _petiole, _rachides, parent_node), _group) in enumerate(selected):
        parent_branch, attach_link = internode_location[parent_node]
        key = (parent_branch, attach_link, int(organ.common.rank or 0))
        sibling_groups.setdefault(key, []).append(index)
        selected_locations.append((parent_branch, attach_link))
    sibling_ordinals = {}
    for indices in sibling_groups.values():
        indices.sort(key=lambda index: selected[index][0][1].groimp_node_id)
        sibling_ordinals.update({index: ordinal for ordinal, index in enumerate(indices)})

    branches = list(structural.branches)
    source_axis_ids = list(structural.source_axis_ids)
    represented = list(structural.represented_organ_ids)
    attachment_map = list(structural.attachment_map)
    degenerate = []
    threshold = 1e-9
    for index, ((organ, node, petiole, rachides, parent_node), group) in enumerate(selected):
        represented_ids = [candidate[0].id for candidate in group]
        represented.extend(represented_ids)
        source_axis_ids.extend(
            axis.id
            for candidate in group
            for axis in (candidate[2], *candidate[3])
        )
        parent_branch, attach_link = selected_locations[index]
        ordinal = sibling_ordinals[index]
        rank = organ.common.rank
        if rank is None:
            raise PlantStateBranchesError(f"Leaf {node.id} has no rank")
        prefix = "Leaf" if int(organ.common.order or 0) == 0 else "LatLeaf"
        base = f"{prefix}_r{rank}_o{ordinal}_g{node.groimp_node_id}"

        petiole_degenerate = petiole.length <= threshold or petiole.radius <= threshold
        rachis_degenerate = [
            axis.id for axis in rachides if axis.length <= threshold or axis.radius <= threshold
        ]
        if petiole_degenerate:
            if any(
                axis.length > threshold and axis.radius > threshold for axis in rachides
            ):
                raise PlantStateBranchesError(
                    f"Leaf {node.id} has a degenerate petiole but positive rachis geometry"
                )
            degenerate.append(
                {
                    "organ_id": organ.id,
                    "node_id": node.id,
                    "groimp_node_id": node.groimp_node_id,
                    "xform_name": base,
                    "axis_ids": [petiole.id, *(axis.id for axis in rachides)],
                    "reason": "non-positive canonical leaf-support dimensions",
                }
            )
            continue
        if rachis_degenerate:
            raise PlantStateBranchesError(
                f"Leaf {node.id} has partially degenerate rachis axes {rachis_degenerate}"
            )

        tilt, rotation = _legacy_leaf_orientation(organ, parent_branch, ordinal)
        axis_id = f"{base}_axis"
        petiole_id = f"{base}_petiole"
        petiole_spec = _leaf_link_spec(
            organ,
            node,
            petiole,
            represented_ids,
            root.pose.world_start,
            pose_mode,
            f"Petiole_g{node.groimp_node_id}",
        )
        rachis_joint_type = (
            "d6" if leaf_joint_policy == "distributed" else "fixed"
        )
        branches.append(
            {
                "id": petiole_id,
                "kind": "leaf_petiole",
                "system": "vegetative",
                "visual_axis_id": axis_id,
                "visual_segments": [
                    {
                        "source_id": petiole_spec["id"],
                        "length": petiole_spec["length"],
                        "radius": petiole_spec["radius"],
                    }
                ],
                "parent": parent_branch,
                "attach_link": attach_link,
                "attach_frac": 1.0,
                "n_links": 1,
                "radius": petiole_spec["radius"],
                "height": petiole_spec["length"],
                "tilt": tilt,
                "rot": rotation,
                "joint_type": "d6",
                "attachment_joint_type": "d6",
                # The support-only checkpoint excludes terminal visual fork
                # dressing. build_leaf_branches enables it once complete leaf
                # visuals are present.
                "disable_centered_terminal": True,
                "link_specs": [petiole_spec],
                "source_origin": [float(value) for value in root.pose.world_start],
                "source_parent_node_id": parent_node,
                "plant_state_schema": state.schema_version,
                "visual_profile": _segmented_visual_profile(visual_quality),
                # Historical V2 damping ratio. Stiffness remains unchanged.
                "damping_ratio": 0.3,
            }
        )
        attachment_map.append(
            {
                "kind": "leaf_petiole",
                "branch_id": petiole_id,
                "root_node_id": node.id,
                "root_groimp_node_id": node.groimp_node_id,
                "parent_branch_id": parent_branch,
                "parent_node_id": parent_node,
                "parent_groimp_node_id": nodes[parent_node].groimp_node_id,
                "attach_link": attach_link,
                "link_node_ids": [node.id],
            }
        )

        if not rachides:
            continue
        rachis_id = f"{base}_rachis"
        rachis_specs = [
            _leaf_link_spec(
                organ,
                node,
                axis,
                represented_ids,
                root.pose.world_start,
                pose_mode,
                f"LeafRachis_g{node.groimp_node_id}_s{segment + 1:02d}",
            )
            for segment, axis in enumerate(rachides)
        ]
        branches.append(
            {
                "id": rachis_id,
                "kind": "leaf_rachis",
                "system": "vegetative",
                "visual_axis_id": axis_id,
                "visual_segments": [
                    {
                        "source_id": spec["id"],
                        "length": spec["length"],
                        "radius": spec["radius"],
                    }
                    for spec in rachis_specs
                ],
                "parent": petiole_id,
                "attach_link": 1,
                "attach_frac": 1.0,
                "n_links": len(rachis_specs),
                "radius": sum(spec["radius"] for spec in rachis_specs) / len(rachis_specs),
                "height": sum(spec["length"] for spec in rachis_specs) / len(rachis_specs),
                "tilt": 0.0,
                "rot": 0.0,
                # ``optimized`` keeps the complete rachis rigidly attached to
                # the petiole. ``distributed`` restores the established V2
                # bending chain without making petiolules physical bodies.
                "joint_type": rachis_joint_type,
                "attachment_joint_type": rachis_joint_type,
                "link_specs": rachis_specs,
                "source_origin": [float(value) for value in root.pose.world_start],
                "source_parent_node_id": node.id,
                "plant_state_schema": state.schema_version,
                "visual_profile": _segmented_visual_profile(visual_quality),
                "damping_ratio": 0.3,
            }
        )
        attachment_map.append(
            {
                "kind": "leaf_rachis",
                "branch_id": rachis_id,
                "root_node_id": node.id,
                "root_groimp_node_id": node.groimp_node_id,
                "parent_branch_id": petiole_id,
                "parent_node_id": node.id,
                "parent_groimp_node_id": node.groimp_node_id,
                "attach_link": 1,
                "link_node_ids": [node.id] * len(rachis_specs),
            }
        )

    return StemBranchesResult(
        branches=tuple(branches),
        source_axis_ids=tuple(source_axis_ids),
        represented_organ_ids=tuple(represented),
        collapsed_duplicates=tuple(collapsed),
        pose_mode=pose_mode,
        debug_profile="leaf-supports",
        attachment_map=tuple(attachment_map),
        degenerate_organs=tuple(degenerate),
        # Native overlaps are detected from the authored colliders for every
        # day.  Keeping GroIMP IDs here made the checkpoint day-specific.
        approved_collision_filters=(),
        leaf_joint_policy=leaf_joint_policy,
        visual_quality=visual_quality,
    )


def _point_segment_attachment(point, axis: AxisGeometry) -> tuple[float, float]:
    delta = tuple(axis.world_end[index] - axis.world_start[index] for index in range(3))
    offset = tuple(point[index] - axis.world_start[index] for index in range(3))
    denominator = sum(value * value for value in delta)
    fraction = (
        max(
            0.0,
            min(
                1.0,
                sum(offset[index] * delta[index] for index in range(3))
                / denominator,
            ),
        )
        if denominator > 0.0
        else 0.0
    )
    closest = tuple(
        axis.world_start[index] + fraction * delta[index] for index in range(3)
    )
    distance = sum(
        (point[index] - closest[index]) ** 2 for index in range(3)
    ) ** 0.5
    return distance, fraction


def build_leaf_branches(
    state: PlantState,
    *,
    pose_mode: str = "canonical",
    appendage_pose_mode: str = "v2-aesthetic",
    leaf_joint_policy: str = "distributed",
    physical_petiolules: bool = False,
    visual_quality: str = "realistic",
) -> StemBranchesResult:
    """Add canonical petiolules and leaf blades.

    Petiolules are visual-only by default.  ``physical_petiolules`` is an
    intentionally expensive diagnostic switch: it restores one rigid body and
    one D6 joint per positive petiolule/terminal rachis axis.
    """

    supports = build_leaf_support_branches(
        state,
        pose_mode=pose_mode,
        leaf_joint_policy=leaf_joint_policy,
        visual_quality=visual_quality,
    )
    if pose_mode != "canonical":
        raise PlantStateBranchesError(
            "the leaves checkpoint requires canonical petiolule frames"
        )
    _validate_appendage_pose_mode(appendage_pose_mode)

    nodes = {node.id: node for node in state.nodes}
    organs = {organ.node_id: organ for organ in state.organs}
    root = nodes[state.root_node_id]
    axes_by_owner: dict[str, list[AxisGeometry]] = {}
    for axis in state.axes:
        axes_by_owner.setdefault(axis.owner_node_id, []).append(axis)

    included_roles = {"petiolule_left", "petiolule_right", "rachis_terminal"}
    visual_records = []
    source_axis_ids = list(supports.source_axis_ids)
    for node_id, axes in sorted(
        axes_by_owner.items(), key=lambda item: nodes[item[0]].groimp_node_id
    ):
        organ = organs.get(node_id)
        if organ is None or organ.organ_type != "Leaf":
            continue
        support_axes = [
            axis
            for axis in axes
            if axis.role in {"petiole", "leaf_rachis"}
            and axis.length > 1e-9
            and axis.radius > 1e-9
        ]
        visual_axes = sorted(
            (axis for axis in axes if axis.role in included_roles),
            key=lambda axis: (
                {"petiolule_left": 0, "petiolule_right": 1, "rachis_terminal": 2}[
                    axis.role
                ],
                int(axis.id.rsplit(":", 1)[1]),
            ),
        )
        positive_visual_count = sum(
            axis.length > 1e-9 and axis.radius > 1e-9 for axis in visual_axes
        )
        leaf_dry_mass_kg = max(float(organ.common.dry_biomass or 0.0), 0.0) * 1e-6
        mass_per_visual_kg = (
            leaf_dry_mass_kg / positive_visual_count
            if positive_visual_count
            else 0.0
        )
        for axis in visual_axes:
            source_axis_ids.append(axis.id)
            if axis.length <= 1e-9 or axis.radius <= 1e-9:
                continue
            if not support_axes:
                raise PlantStateBranchesError(
                    f"leaf visual {axis.id} has no positive support axis"
                )
            candidates = []
            for support in support_axes:
                distance, fraction = _point_segment_attachment(
                    axis.world_start, support
                )
                candidates.append((distance, fraction, support.id, support))
            distance, fraction, _support_id, host = min(candidates)
            if distance > 1e-6:
                raise PlantStateBranchesError(
                    f"leaf visual {axis.id} is {distance:.6g}m from its nearest support"
                )
            segment = int(axis.id.rsplit(":", 1)[1])
            source_rest_frame = _rebased_frame(axis, root.pose.world_start)
            rest_frame = source_rest_frame
            if appendage_pose_mode == "v2-aesthetic":
                if axis.role == "rachis_terminal":
                    tilt_deg = 0.0
                    rotation_deg = 0.0
                else:
                    inclinations = tuple(
                        getattr(organ.properties, "petiolule_inclinations", ()) or ()
                    )
                    tilt_deg = (
                        float(inclinations[segment])
                        if segment < len(inclinations)
                        else 90.0
                    )
                    rotation_deg = 90.0 if axis.role == "petiolule_left" else 270.0
                rest_frame = _v2_appendage_frame(
                    axis,
                    host,
                    root.pose.world_start,
                    tilt_deg=tilt_deg,
                    rotation_deg=rotation_deg,
                )
            visual_records.append(
                {
                    "id": (
                        f"LeafVisual_g{nodes[node_id].groimp_node_id}_"
                        f"{axis.role}_{segment + 1:02d}"
                    ),
                    "node_id": node_id,
                    "organ_id": organ.id,
                    "groimp_node_id": int(nodes[node_id].groimp_node_id),
                    "axis_id": axis.id,
                    "role": axis.role,
                    "host_axis_id": host.id,
                    "host_fraction": float(fraction),
                    "rest_frame": rest_frame,
                    "source_rest_frame": source_rest_frame,
                    "appendage_pose_mode": appendage_pose_mode,
                    "length": float(axis.length),
                    "radius": float(axis.radius),
                    # PlantState dry biomass is stored in mg. Petiolules and
                    # blades remain visual-only, but their source mass is
                    # aggregated into the host rachis rigid body below.
                    "aggregated_mass_kg": mass_per_visual_kg,
                    "mass_source": "plant_state_leaf_dry_biomass",
                }
            )

    complete_branches = []
    for source in supports.branches:
        branch = dict(source)
        if branch.get("kind") == "leaf_petiole":
            branch.pop("disable_centered_terminal", None)
        complete_branches.append(branch)

    attachment_map = list(supports.attachment_map)
    if physical_petiolules:
        host_locations = {}
        for branch in complete_branches:
            for index, spec in enumerate(branch["link_specs"], start=1):
                host_locations[spec["canonical_axis_id"]] = (branch["id"], index)
        physical_records = []
        for record in visual_records:
            location = host_locations.get(record["host_axis_id"])
            if location is None:
                raise PlantStateBranchesError(
                    f"physical petiolule {record['axis_id']} cannot resolve host "
                    f"{record['host_axis_id']}"
                )
            parent_branch, attach_link = location
            branch_id = f"{record['id']}_physical"
            spec = {
                "id": record["id"],
                "groimp_node_id": int(record["groimp_node_id"]),
                "canonical_node_id": record["node_id"],
                "canonical_organ_id": record["organ_id"],
                "canonical_axis_id": record["axis_id"],
                "axis_role": record["role"],
                "represented_organ_ids": [record["organ_id"]],
                "length": float(record["length"]),
                "radius": float(record["radius"]),
                "rest_frame": record["rest_frame"],
                "source_rest_frame": record["source_rest_frame"],
            }
            complete_branches.append(
                {
                    "id": branch_id,
                    "kind": "petiolule",
                    "system": "vegetative",
                    "visual_axis_id": branch_id,
                    "visual_segments": [
                        {
                            "source_id": spec["id"],
                            "length": spec["length"],
                            "radius": spec["radius"],
                        }
                    ],
                    "parent": parent_branch,
                    "attach_link": attach_link,
                    "attach_frac": float(record["host_fraction"]),
                    "n_links": 1,
                    "radius": spec["radius"],
                    "height": spec["length"],
                    "tilt": 0.0,
                    "rot": 0.0,
                    "joint_type": "d6",
                    "attachment_joint_type": "d6",
                    "link_specs": [spec],
                    "source_origin": [
                        float(value) for value in root.pose.world_start
                    ],
                    "source_parent_node_id": record["node_id"],
                    "plant_state_schema": state.schema_version,
                    "visual_profile": _segmented_visual_profile(visual_quality),
                    "damping_ratio": 0.3,
                }
            )
            attachment_map.append(
                {
                    "kind": "petiolule",
                    "branch_id": branch_id,
                    "root_node_id": record["node_id"],
                    "root_groimp_node_id": record["groimp_node_id"],
                    "parent_branch_id": parent_branch,
                    "parent_node_id": record["node_id"],
                    "parent_groimp_node_id": record["groimp_node_id"],
                    "attach_link": attach_link,
                    "link_node_ids": [record["node_id"]],
                }
            )
            physical_records.append(
                {
                    **record,
                    "host_axis_id": record["axis_id"],
                    "physical": True,
                    "physical_branch_id": branch_id,
                    "aggregated_mass_kg": 0.0,
                }
            )
        visual_records = physical_records
    return StemBranchesResult(
        branches=tuple(complete_branches),
        source_axis_ids=tuple(source_axis_ids),
        represented_organ_ids=supports.represented_organ_ids,
        collapsed_duplicates=supports.collapsed_duplicates,
        pose_mode=pose_mode,
        debug_profile="leaves",
        attachment_map=tuple(attachment_map),
        degenerate_organs=supports.degenerate_organs,
        approved_collision_filters=supports.approved_collision_filters,
        rigid_leaf_visuals=tuple(visual_records),
        leaf_joint_policy=supports.leaf_joint_policy,
        physical_petiolules=physical_petiolules,
        visual_quality=visual_quality,
        appendage_pose_mode=appendage_pose_mode,
    )


def _canonical_axis_spec(
    organ: OrganRecord,
    node,
    axis: AxisGeometry,
    origin,
    *,
    label: str,
) -> dict[str, Any]:
    source_frame = _rebased_frame(axis, origin)
    return {
        "id": label,
        "groimp_node_id": int(node.groimp_node_id),
        "canonical_node_id": node.id,
        "canonical_organ_id": organ.id,
        "canonical_axis_id": axis.id,
        "axis_role": axis.role,
        "represented_organ_ids": [organ.id],
        "length": float(axis.length),
        "radius": float(axis.radius),
        "rest_frame": source_frame,
        "source_rest_frame": source_frame,
    }


def _axis_index(axis: AxisGeometry) -> int:
    try:
        return int(axis.id.rsplit(":", 1)[1])
    except (ValueError, IndexError) as exc:
        raise PlantStateBranchesError(
            f"canonical primitive {axis.id!r} has no numeric suffix"
        ) from exc


def _nearest_axis_endpoint(point, axes: list[AxisGeometry]) -> tuple[int, float]:
    candidates = []
    for index, axis in enumerate(axes, start=1):
        distance = math.sqrt(
            sum((float(point[i]) - float(axis.world_end[i])) ** 2 for i in range(3))
        )
        candidates.append((distance, index))
    if not candidates:
        raise PlantStateBranchesError("cannot attach to an empty canonical axis chain")
    distance, index = min(candidates)
    if distance > 1e-6:
        raise PlantStateBranchesError(
            f"canonical attachment is {distance:.6g}m from the nearest chain endpoint"
        )
    return index, distance


def build_truss_branches(
    state: PlantState,
    *,
    pose_mode: str = "canonical",
    appendage_pose_mode: str = "v2-aesthetic",
    leaf_joint_policy: str = "distributed",
    physical_petiolules: bool = False,
    visual_quality: str = "realistic",
    include_fruits: bool = False,
    physical_fruits: bool = False,
) -> StemBranchesResult:
    """Add native truss rachides, pedicels, and optional tomato bodies."""

    if pose_mode != "canonical":
        raise PlantStateBranchesError(
            "canonical truss checkpoints require --pose-mode canonical"
        )
    if physical_fruits and not include_fruits:
        raise PlantStateBranchesError("physical_fruits requires include_fruits")
    _validate_appendage_pose_mode(appendage_pose_mode)

    leaves = build_leaf_branches(
        state,
        pose_mode=pose_mode,
        appendage_pose_mode=appendage_pose_mode,
        leaf_joint_policy=leaf_joint_policy,
        physical_petiolules=physical_petiolules,
        visual_quality=visual_quality,
    )
    nodes = {node.id: node for node in state.nodes}
    organs = {organ.node_id: organ for organ in state.organs}
    root = nodes[state.root_node_id]
    origin = root.pose.world_start
    axes_by_owner: dict[str, list[AxisGeometry]] = {}
    spheres_by_owner: dict[str, list[SphereGeometry]] = {}
    for axis in state.axes:
        axes_by_owner.setdefault(axis.owner_node_id, []).append(axis)
    for sphere in state.spheres:
        spheres_by_owner.setdefault(sphere.owner_node_id, []).append(sphere)

    pedicel_length_scale = float(
        TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE
    )
    if not math.isfinite(pedicel_length_scale) or pedicel_length_scale <= 0.0:
        raise PlantStateBranchesError(
            "TrussGeometryConfig.PLANT_STATE_PEDICEL_LENGTH_SCALE must be "
            f"finite and positive, got {pedicel_length_scale!r}"
        )

    internode_location = {}
    for branch in leaves.branches:
        for index, spec in enumerate(branch["link_specs"], start=1):
            if spec["axis_role"] == "internode":
                internode_location[spec["canonical_node_id"]] = (branch["id"], index)

    def nearest_internode(node_id: str) -> str:
        current = nodes[node_id].parent_id
        visited = {node_id}
        while current is not None:
            if current in visited:
                raise PlantStateBranchesError(
                    f"cycle while resolving Fruits parent for {node_id}"
                )
            visited.add(current)
            organ = organs.get(current)
            if organ is not None and organ.organ_type == "Internode":
                if current not in internode_location:
                    raise PlantStateBranchesError(
                        f"Fruits {node_id} resolves to excluded Internode {current}"
                    )
                return current
            parent = nodes.get(current)
            if parent is None:
                raise PlantStateBranchesError(
                    f"Fruits {node_id} references missing ancestor {current}"
                )
            current = parent.parent_id
        raise PlantStateBranchesError(f"Fruits {node_id} has no Internode ancestor")

    fruits_organs = sorted(
        (organ for organ in state.organs if organ.organ_type == "Fruits"),
        key=lambda organ: (
            int(organ.common.order or 0),
            int(organ.common.rank or 0),
            nodes[organ.node_id].groimp_node_id,
        ),
    )
    branches = list(leaves.branches)
    source_axis_ids = list(leaves.source_axis_ids)
    represented = list(leaves.represented_organ_ids)
    represented.extend(
        organ.id for organ in state.organs if organ.organ_type == "Truss"
    )
    attachment_map = list(leaves.attachment_map)
    degenerate = list(leaves.degenerate_organs)
    terminal_bodies = []
    sibling_ordinals: dict[tuple[str, int, int], int] = {}

    for organ in fruits_organs:
        node = nodes[organ.node_id]
        parent_node = nearest_internode(node.id)
        parent_branch, parent_link = internode_location[parent_node]
        rank = int(organ.common.rank or 0)
        sibling_key = (parent_branch, parent_link, rank)
        ordinal = sibling_ordinals.get(sibling_key, 0)
        sibling_ordinals[sibling_key] = ordinal + 1
        base = f"Truss_r{rank}_o{ordinal}_g{node.groimp_node_id}"
        owner_axes = axes_by_owner.get(node.id, [])
        rachides = sorted(
            (axis for axis in owner_axes if axis.role == "truss_rachis"),
            key=_axis_index,
        )
        pedicels = sorted(
            (axis for axis in owner_axes if axis.role == "pedicel"),
            key=_axis_index,
        )
        spheres = sorted(spheres_by_owner.get(node.id, []), key=lambda item: int(item.id.rsplit(":", 1)[1]))
        positive_rachides = [
            axis for axis in rachides if axis.length > 1e-9 and axis.radius > 1e-9
        ]
        positive_pedicels = [
            axis for axis in pedicels if axis.length > 1e-9 and axis.radius > 1e-9
        ]
        source_axis_ids.extend(axis.id for axis in (*rachides, *pedicels))
        represented.append(organ.id)

        if not positive_rachides:
            degenerate.append(
                {
                    "organ_id": organ.id,
                    "node_id": node.id,
                    "groimp_node_id": node.groimp_node_id,
                    "xform_name": base,
                    "axis_ids": [axis.id for axis in (*rachides, *pedicels)],
                    "reason": "no positive canonical truss rachis geometry",
                }
            )
            if positive_pedicels or any(sphere.radius > 1e-9 for sphere in spheres):
                raise PlantStateBranchesError(
                    f"Fruits {node.id} has pedicels/fruits without a positive rachis"
                )
            continue
        if len(positive_rachides) != len(rachides) or len(positive_pedicels) != len(pedicels):
            raise PlantStateBranchesError(
                f"Fruits {node.id} has partially degenerate truss axes"
            )

        rachis_id = f"{base}_rachis"
        rachis_specs = [
            _canonical_axis_spec(
                organ,
                node,
                axis,
                origin,
                label=f"TrussRachis_g{node.groimp_node_id}_s{index + 1:02d}",
            )
            for index, axis in enumerate(positive_rachides)
        ]
        branches.append(
            {
                "id": rachis_id,
                "kind": "truss_rachis",
                # Route canonical trusses through the same explicit-frame
                # rigid-link backend as the vegetative structure.  The
                # historical truss visual dressing is restored after link
                # authoring; BRANCHES without link_specs keep the old hybrid
                # path unchanged.
                "system": "vegetative",
                "physics_profile": "truss",
                "truss_component": "rachis",
                "young_modulus": float(TrussPhysicsConfig.RACHIS_YOUNG_MODULUS),
                "damping_ratio": float(TrussPhysicsConfig.RACHIS_DAMPING_RATIO),
                "visual_style": "historical_truss_rachis",
                "visual_axis_id": rachis_id,
                "visual_segments": [
                    {
                        "source_id": spec["id"],
                        "length": spec["length"],
                        "radius": spec["radius"],
                    }
                    for spec in rachis_specs
                ],
                "visual_profile": _segmented_visual_profile(visual_quality),
                "parent": parent_branch,
                "attach_link": parent_link,
                "attach_frac": 1.0,
                "n_links": len(rachis_specs),
                "radius": sum(spec["radius"] for spec in rachis_specs) / len(rachis_specs),
                "height": sum(spec["length"] for spec in rachis_specs) / len(rachis_specs),
                "tilt": 0.0,
                "rot": 0.0,
                "joint_type": "d6",
                "attachment_joint_type": "d6",
                "link_specs": rachis_specs,
                "source_origin": [float(value) for value in origin],
                "source_parent_node_id": parent_node,
                "plant_state_schema": state.schema_version,
            }
        )
        attachment_map.append(
            {
                "kind": "truss_rachis",
                "branch_id": rachis_id,
                "root_node_id": node.id,
                "root_groimp_node_id": node.groimp_node_id,
                "parent_branch_id": parent_branch,
                "parent_node_id": parent_node,
                "parent_groimp_node_id": nodes[parent_node].groimp_node_id,
                "attach_link": parent_link,
                "link_node_ids": [node.id] * len(rachis_specs),
            }
        )

        pedicel_ids = []
        pedicel_specs = []
        has_terminal_pedicel = len(positive_pedicels) % 2 == 1
        for index, axis in enumerate(positive_pedicels):
            attach_link, _distance = _nearest_axis_endpoint(
                axis.world_start, positive_rachides
            )
            pedicel_id = f"{base}_pedicel_{index + 1:02d}"
            pedicel_ids.append(pedicel_id)
            spec = _canonical_axis_spec(
                organ,
                node,
                axis,
                origin,
                label=f"Pedicel_g{node.groimp_node_id}_f{index + 1:02d}",
            )
            spec["source_length"] = float(spec["length"])
            spec["authored_length_scale"] = pedicel_length_scale
            spec["length"] = float(spec["length"]) * pedicel_length_scale
            if appendage_pose_mode == "v2-aesthetic":
                terminal = has_terminal_pedicel and index == len(positive_pedicels) - 1
                if terminal:
                    tilt_deg, rotation_deg = 0.0, 0.0
                else:
                    tilt_deg = 56.0
                    rotation_deg = 90.0 if index % 2 == 0 else 270.0
                spec["rest_frame"] = _v2_appendage_frame(
                    axis,
                    positive_rachides[attach_link - 1],
                    origin,
                    tilt_deg=tilt_deg,
                    rotation_deg=rotation_deg,
                )
                spec["v2_tilt_deg"] = tilt_deg
                spec["v2_rotation_deg"] = rotation_deg
            pedicel_specs.append(spec)
            branches.append(
                {
                    "id": pedicel_id,
                    "kind": "pedicel",
                    "system": "vegetative",
                    "physics_profile": "truss",
                    "truss_component": "pedicel",
                    "young_modulus": float(TrussPhysicsConfig.PEDICEL_YOUNG_MODULUS),
                    "damping_ratio": float(TrussPhysicsConfig.PEDICEL_DAMPING_RATIO),
                    "visual_style": "historical_pedicel",
                    "visual_axis_id": pedicel_id,
                    "visual_segments": [
                        {
                            "source_id": spec["id"],
                            "length": spec["length"],
                            "radius": spec["radius"],
                        }
                    ],
                    "visual_profile": _segmented_visual_profile(visual_quality),
                    "parent": rachis_id,
                    "attach_link": attach_link,
                    "attach_frac": 1.0,
                    "n_links": 1,
                    "radius": spec["radius"],
                    "height": spec["length"],
                    "tilt": 0.0,
                    "rot": 0.0,
                    "joint_type": "d6",
                    "attachment_joint_type": "d6",
                    "bend_limit_deg": 25.0,
                    "drive_stiffness_scale": float(
                        TrussPhysicsConfig.PEDICEL_DRIVE_STIFFNESS_SCALE
                    ),
                    "link_specs": [spec],
                    "source_origin": [float(value) for value in origin],
                    "source_parent_node_id": node.id,
                    "plant_state_schema": state.schema_version,
                }
            )
            attachment_map.append(
                {
                    "kind": "pedicel",
                    "branch_id": pedicel_id,
                    "root_node_id": node.id,
                    "root_groimp_node_id": node.groimp_node_id,
                    "parent_branch_id": rachis_id,
                    "parent_node_id": node.id,
                    "parent_groimp_node_id": node.groimp_node_id,
                    "attach_link": attach_link,
                    "link_node_ids": [node.id],
                }
            )

        if include_fruits:
            positive_spheres = [sphere for sphere in spheres if sphere.radius > 1e-9]
            if len(positive_spheres) != len(pedicel_ids):
                raise PlantStateBranchesError(
                    f"Fruits {node.id} has {len(positive_spheres)} positive spheres but "
                    f"{len(pedicel_ids)} positive pedicels"
                )
            degree_days = tuple(getattr(organ.properties, "fruit_degree_days", ()) or ())
            ripening = float(getattr(organ.properties, "ripening_degree_days", 0.0) or 0.0)
            for index, (sphere, pedicel_id, pedicel_spec) in enumerate(
                zip(positive_spheres, pedicel_ids, pedicel_specs)
            ):
                maturation = (
                    max(0.0, min(1.0, float(degree_days[index]) / ripening))
                    if ripening > 0.0 and index < len(degree_days)
                    else 0.0
                )
                terminal_bodies.append(
                    {
                        "id": f"{base}_tomato_{index + 1:02d}",
                        "shape": "sphere",
                        "parent_branch_id": pedicel_id,
                        "radius": float(sphere.radius),
                        "mass": (4.0 / 3.0) * math.pi * float(sphere.radius) ** 3 * 1000.0,
                        "maturation": maturation,
                        "physical": physical_fruits,
                        "detachment_enabled": physical_fruits,
                        "exclude_from_articulation": physical_fruits,
                        "break_force": 6.0 if physical_fruits else None,
                        "canonical_node_id": node.id,
                        "canonical_organ_id": organ.id,
                        "canonical_primitive_id": sphere.id,
                        "parent_axis_id": positive_pedicels[index].id,
                        **(
                            {
                                "rest_center": [
                                    float(sphere.world_center[row] - origin[row])
                                    for row in range(3)
                                ],
                                "rest_frame": [list(row) for row in sphere.world_frame],
                            }
                            if appendage_pose_mode == "canonical"
                            else {
                                # With no explicit center the historical terminal-body
                                # builder places the tomato at the curved V2 pedicel tip.
                                "rest_frame": pedicel_spec["rest_frame"],
                            }
                        ),
                        "source_center": [float(value) for value in sphere.world_center],
                        "source_frame": [list(row) for row in sphere.world_frame],
                        "appendage_pose_mode": appendage_pose_mode,
                    }
                )

    profile = "truss-supports"
    if include_fruits:
        profile = "full" if physical_fruits else "fruit-visual"
    return StemBranchesResult(
        branches=tuple(branches),
        source_axis_ids=tuple(source_axis_ids),
        represented_organ_ids=tuple(represented),
        collapsed_duplicates=leaves.collapsed_duplicates,
        pose_mode=pose_mode,
        debug_profile=profile,
        attachment_map=tuple(attachment_map),
        degenerate_organs=tuple(degenerate),
        approved_collision_filters=(),
        rigid_leaf_visuals=leaves.rigid_leaf_visuals,
        terminal_bodies=tuple(terminal_bodies),
        leaf_joint_policy=leaf_joint_policy,
        physical_petiolules=physical_petiolules,
        visual_quality=visual_quality,
        appendage_pose_mode=appendage_pose_mode,
    )
