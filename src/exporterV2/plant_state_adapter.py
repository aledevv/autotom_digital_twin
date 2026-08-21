"""Canonical PlantState to ExporterV2 visual and physical authoring plan.

This module deliberately has no Isaac Sim dependency.  It is the boundary
between source truth and the V2-specific physics/collision adaptation.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field, replace
import json
import math
from typing import Any, Iterable

import numpy as np

from plant_state import AxisGeometry, PlantState, SphereGeometry, validate_plant_state


V2_MANIFEST_SCHEMA_VERSION = "exporter_v2_manifest/1.0"
PHYSICAL_AXIS_ROLES = frozenset(
    {"internode", "petiole", "leaf_rachis", "truss_rachis", "pedicel"}
)
VISUAL_ONLY_AXIS_ROLES = frozenset(
    {"petiolule_left", "petiolule_right", "rachis_terminal"}
)
JOINT_TARGET = 220
JOINT_WARNING_MAX = 230
CANONICAL_SCALE = 2.0
MAX_AZIMUTH_CORRECTION_DEG = 5.0
MAX_TILT_CORRECTION_DEG = 3.0
MAX_POSITION_SHIFT_M = 0.002
COLLISION_MARGIN_M = 0.00025
COLLIDER_RADIUS_SCALE = 0.90
COLLIDER_LENGTH_SCALE = 0.92
MIN_COLLIDER_RADIUS_WORLD = 0.002
_GEOMETRY_TOLERANCE = 1e-8


class V2PlantStateError(ValueError):
    """Raised when a canonical state cannot be mapped without guessing."""


def validate_joint_budget(
    predicted_joints: int,
    *,
    allow_near_budget: bool = False,
    optimize: bool = False,
) -> str:
    """Validate Phase-J budget thresholds and return the selected action."""

    if predicted_joints < 0:
        raise V2PlantStateError("predicted joint count cannot be negative")
    if predicted_joints > JOINT_WARNING_MAX and not optimize:
        raise V2PlantStateError(
            f"predicted D6 joints {predicted_joints} exceed hard review threshold "
            f"{JOINT_WARNING_MAX}; rerun with --optimize"
        )
    if JOINT_TARGET < predicted_joints <= JOINT_WARNING_MAX and not (
        allow_near_budget or optimize
    ):
        raise V2PlantStateError(
            f"predicted D6 joints {predicted_joints} are in the 221-230 review "
            "band; use --allow-near-budget or --optimize"
        )
    return "optimize" if optimize and predicted_joints > JOINT_TARGET else "unchanged"


@dataclass(frozen=True)
class Pose:
    start: tuple[float, float, float]
    end: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]

    @property
    def direction(self) -> tuple[float, float, float]:
        delta = np.asarray(self.end) - np.asarray(self.start)
        length = float(np.linalg.norm(delta))
        if length <= 0.0:
            return (0.0, 0.0, 1.0)
        return tuple(float(value) for value in delta / length)

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.asarray(self.end) - np.asarray(self.start)))


@dataclass(frozen=True)
class V2VisualAxis:
    id: str
    owner_node_id: str
    organ_type: str
    role: str
    source_pose: Pose
    authored_pose: Pose
    radius: float
    host_link_id: str
    render_geometry: bool = True
    duplicate_of: str | None = None


@dataclass(frozen=True)
class V2VisualSphere:
    id: str
    owner_node_id: str
    organ_type: str
    role: str
    source_center: tuple[float, float, float]
    authored_center: tuple[float, float, float]
    radius: float
    host_link_id: str
    render_geometry: bool = True
    duplicate_of: str | None = None


@dataclass(frozen=True)
class V2PhysicalLink:
    id: str
    canonical_axis_id: str
    owner_node_id: str
    role: str
    parent_id: str | None
    source_pose: Pose
    authored_pose: Pose
    visual_radius: float
    collider_radius: float
    joint_type: str
    canonical_organ_ids: tuple[str, ...]
    canonical_primitive_ids: tuple[str, ...]


@dataclass(frozen=True)
class CollisionRecord:
    body_a: str
    body_b: str
    kind: str
    overlap_m: float
    reason: str


@dataclass(frozen=True)
class CollisionAdjustment:
    link_id: str
    reason: str
    azimuth_delta_deg: float = 0.0
    tilt_delta_deg: float = 0.0
    shift_m: float = 0.0


@dataclass(frozen=True)
class V2AuthoringPlan:
    state: PlantState
    source_origin: tuple[float, float, float]
    scale: float
    physics_preset: str
    visual_axes: tuple[V2VisualAxis, ...]
    visual_spheres: tuple[V2VisualSphere, ...]
    physical_links: tuple[V2PhysicalLink, ...]
    intentional_collision_filters: tuple[CollisionRecord, ...]
    unresolved_collision_filters: tuple[CollisionRecord, ...]
    collision_adjustments: tuple[CollisionAdjustment, ...]
    diagnostics: dict[str, Any] = field(default_factory=dict)

    @property
    def predicted_d6_joints(self) -> int:
        return sum(
            link.joint_type == "d6" and link.parent_id is not None
            for link in self.physical_links
        )


def _pose_for_axis(
    axis: AxisGeometry,
    origin: np.ndarray,
    scale: float,
) -> Pose:
    matrix = np.asarray(axis.world_frame, dtype=np.float64)
    rotation = matrix[:3, :3]
    start = (np.asarray(axis.world_start, dtype=np.float64) - origin) * scale
    end = (np.asarray(axis.world_end, dtype=np.float64) - origin) * scale
    return Pose(
        start=tuple(float(value) for value in start),
        end=tuple(float(value) for value in end),
        rotation=tuple(tuple(float(value) for value in row) for row in rotation),
    )


def _point_segment_distance(point, start, end) -> tuple[float, float]:
    point = np.asarray(point, dtype=np.float64)
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    vector = end - start
    denominator = float(vector @ vector)
    if denominator <= 1e-30:
        return float(np.linalg.norm(point - start)), 0.0
    fraction = max(0.0, min(1.0, float((point - start) @ vector / denominator)))
    return float(np.linalg.norm(point - (start + fraction * vector))), fraction


def _segment_distance(p1, q1, p2, q2) -> float:
    """Exact closest distance between two finite line segments."""

    p1, q1, p2, q2 = (
        np.asarray(value, dtype=np.float64) for value in (p1, q1, p2, q2)
    )
    u, v, w = q1 - p1, q2 - p2, p1 - p2
    a, b, c = float(u @ u), float(u @ v), float(v @ v)
    d, e = float(u @ w), float(v @ w)
    denominator = a * c - b * b
    epsilon = 1e-30
    if denominator < epsilon:
        s_num, s_den, t_num, t_den = 0.0, 1.0, e, c
    else:
        s_num, t_num = b * e - c * d, a * e - b * d
        s_den = t_den = denominator
        if s_num < 0.0:
            s_num, t_num, t_den = 0.0, e, c
        elif s_num > s_den:
            s_num, t_num, t_den = s_den, e + b, c
    if t_num < 0.0:
        t_num = 0.0
        if -d < 0.0:
            s_num = 0.0
        elif -d > a:
            s_num = s_den
        else:
            s_num, s_den = -d, a
    elif t_num > t_den:
        t_num = t_den
        if -d + b < 0.0:
            s_num = 0.0
        elif -d + b > a:
            s_num = s_den
        else:
            s_num, s_den = -d + b, a
    s = 0.0 if abs(s_num) < epsilon else s_num / s_den
    t = 0.0 if abs(t_num) < epsilon else t_num / t_den
    return float(np.linalg.norm(w + s * u - t * v))


def capsule_capsule_overlap(
    pose_a: Pose,
    radius_a: float,
    pose_b: Pose,
    radius_b: float,
    *,
    margin: float = 0.0,
) -> float:
    distance = _segment_distance(pose_a.start, pose_a.end, pose_b.start, pose_b.end)
    return radius_a + radius_b + margin - distance


def sphere_capsule_overlap(
    center: Iterable[float],
    sphere_radius: float,
    pose: Pose,
    capsule_radius: float,
    *,
    margin: float = 0.0,
) -> float:
    distance, _ = _point_segment_distance(center, pose.start, pose.end)
    return sphere_radius + capsule_radius + margin - distance


def sphere_sphere_overlap(
    center_a: Iterable[float],
    radius_a: float,
    center_b: Iterable[float],
    radius_b: float,
    *,
    margin: float = 0.0,
) -> float:
    distance = float(
        np.linalg.norm(np.asarray(center_a, dtype=np.float64) - np.asarray(center_b))
    )
    return radius_a + radius_b + margin - distance


def authored_capsule_radius(pose: Pose, radius: float) -> float:
    total = max(pose.length * COLLIDER_LENGTH_SCALE, 1e-5)
    return min(radius, max(total * 0.49, 1e-5))


def authored_capsule_pose(pose: Pose, radius: float) -> Pose:
    """Return the centre-line used by the shortened authored USD capsule.

    V2 deliberately leaves a small collider-free region at both attachment
    ends.  Collision planning must use that same finite segment rather than
    the complete visual axis, otherwise legitimate joints are over-reported.
    """

    if pose.length <= 0.0:
        return pose
    total = max(pose.length * COLLIDER_LENGTH_SCALE, 1e-5)
    authored_radius = authored_capsule_radius(pose, radius)
    spine = max(total - 2.0 * authored_radius, 1e-5)
    inset = max(0.0, (pose.length - spine) / 2.0)
    direction = np.asarray(pose.direction, dtype=np.float64)
    start = np.asarray(pose.start, dtype=np.float64) + direction * inset
    end = np.asarray(pose.end, dtype=np.float64) - direction * inset
    return replace(
        pose,
        start=tuple(float(value) for value in start),
        end=tuple(float(value) for value in end),
    )


def _ancestor_ids(state: PlantState, node_id: str) -> set[str]:
    nodes = {node.id: node for node in state.nodes}
    result: set[str] = set()
    current = nodes.get(node_id)
    while current is not None and current.parent_id is not None:
        result.add(current.parent_id)
        current = nodes.get(current.parent_id)
    return result


_ROLE_PARENT_PRIORITY = {
    "internode": {"internode": 0},
    "petiole": {"internode": 0},
    "leaf_rachis": {"petiole": 0, "leaf_rachis": 1},
    "truss_rachis": {"truss_rachis": 0, "internode": 1},
    "pedicel": {"truss_rachis": 0},
}


def _physical_parent_ids(state: PlantState, axes: list[AxisGeometry]) -> dict[str, str | None]:
    result: dict[str, str | None] = {}
    for child in axes:
        ancestors = _ancestor_ids(state, child.owner_node_id)
        candidates = []
        for candidate in axes:
            if candidate.id == child.id:
                continue
            distance, fraction = _point_segment_distance(
                child.world_start, candidate.world_start, candidate.world_end
            )
            if distance > _GEOMETRY_TOLERANCE:
                continue
            same_owner = child.owner_node_id == candidate.owner_node_id
            relation = (
                0
                if same_owner
                else 1
                if candidate.owner_node_id in ancestors
                else 2
            )
            role_priority = _ROLE_PARENT_PRIORITY.get(child.role, {}).get(
                candidate.role, 9
            )
            endpoint_priority = 0 if abs(1.0 - fraction) <= 1e-7 else 1
            candidates.append(
                (
                    relation,
                    role_priority,
                    endpoint_priority,
                    distance,
                    candidate.id,
                )
            )
        candidates.sort()
        if candidates and candidates[0][0] < 2 and candidates[0][1] < 9:
            result[child.id] = candidates[0][4]
        else:
            result[child.id] = None

    roots = [axis_id for axis_id, parent_id in result.items() if parent_id is None]
    if len(roots) != 1:
        raise V2PlantStateError(
            f"physical topology must have one root, found {len(roots)}: {roots}"
        )
    for start in result:
        seen: set[str] = set()
        current: str | None = start
        while current is not None:
            if current in seen:
                raise V2PlantStateError(f"cycle in physical topology at {current}")
            seen.add(current)
            current = result[current]
    return result


def _owner_duplicate_payload(node, organ) -> dict[str, Any]:
    return {
        "source_type": node.source_type,
        "category": node.category,
        "parent_id": node.parent_id,
        "source_attributes": node.source_attributes,
        "organ_type": organ.organ_type,
        "common": asdict(organ.common),
        "properties": asdict(organ.properties),
        "attribute_source": organ.attribute_source,
    }


def _exact_axis_signature(axis: AxisGeometry, node, organ) -> str:
    payload = {
        "geometry": {
            key: value
            for key, value in asdict(axis).items()
            if key not in {"id", "owner_node_id"}
        },
        "owner": _owner_duplicate_payload(node, organ),
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _exact_sphere_signature(sphere: SphereGeometry, node, organ) -> str:
    payload = {
        "geometry": {
            key: value
            for key, value in asdict(sphere).items()
            if key not in {"id", "owner_node_id"}
        },
        "owner": _owner_duplicate_payload(node, organ),
    }
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _children(links: Iterable[V2PhysicalLink]) -> dict[str, list[str]]:
    result: dict[str, list[str]] = defaultdict(list)
    for link in links:
        if link.parent_id is not None:
            result[link.parent_id].append(link.id)
    for values in result.values():
        values.sort()
    return result


def _descendants(root_id: str, children: dict[str, list[str]]) -> set[str]:
    result = {root_id}
    stack = [root_id]
    while stack:
        current = stack.pop()
        for child in children.get(current, ()):
            if child not in result:
                result.add(child)
                stack.append(child)
    return result


def _rotation(axis: Iterable[float], degrees: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    norm = float(np.linalg.norm(axis))
    if norm <= 1e-15:
        return np.eye(3)
    x, y, z = axis / norm
    angle = math.radians(degrees)
    c, s, q = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.asarray(
        [
            [c + x * x * q, x * y * q - z * s, x * z * q + y * s],
            [y * x * q + z * s, c + y * y * q, y * z * q - x * s],
            [z * x * q - y * s, z * y * q + x * s, c + z * z * q],
        ]
    )


def _transform_pose(pose: Pose, pivot, rotation, translation=None) -> Pose:
    pivot = np.asarray(pivot, dtype=np.float64)
    translation = (
        np.zeros(3) if translation is None else np.asarray(translation, dtype=np.float64)
    )
    start = pivot + rotation @ (np.asarray(pose.start) - pivot) + translation
    end = pivot + rotation @ (np.asarray(pose.end) - pivot) + translation
    frame = rotation @ np.asarray(pose.rotation)
    return Pose(
        start=tuple(float(value) for value in start),
        end=tuple(float(value) for value in end),
        rotation=tuple(tuple(float(value) for value in row) for row in frame),
    )


def _shared_attachment(a: V2PhysicalLink, b: V2PhysicalLink) -> bool:
    if a.parent_id == b.id or b.parent_id == a.id:
        return True
    for left in (a.authored_pose.start, a.authored_pose.end):
        for right in (b.authored_pose.start, b.authored_pose.end):
            if np.linalg.norm(np.asarray(left) - np.asarray(right)) <= 1e-9:
                return True
    return False


def _active_link_overlaps(
    links: list[V2PhysicalLink],
) -> tuple[list[CollisionRecord], list[CollisionRecord]]:
    intentional: list[CollisionRecord] = []
    active: list[CollisionRecord] = []
    for index, left in enumerate(links):
        for right in links[index + 1 :]:
            overlap = capsule_capsule_overlap(
                authored_capsule_pose(left.authored_pose, left.collider_radius),
                authored_capsule_radius(left.authored_pose, left.collider_radius),
                authored_capsule_pose(right.authored_pose, right.collider_radius),
                authored_capsule_radius(right.authored_pose, right.collider_radius),
                margin=COLLISION_MARGIN_M,
            )
            if overlap <= 0.0:
                continue
            record = CollisionRecord(
                body_a=left.id,
                body_b=right.id,
                kind="capsule_capsule",
                overlap_m=float(overlap),
                reason=(
                    "intentional_attachment_contact"
                    if _shared_attachment(left, right)
                    else "initial_unrelated_overlap"
                ),
            )
            (intentional if _shared_attachment(left, right) else active).append(record)
    return intentional, active


def _replace_subtree(
    links: list[V2PhysicalLink],
    root_id: str,
    rotation: np.ndarray,
    translation=None,
) -> list[V2PhysicalLink]:
    by_id = {link.id: link for link in links}
    pivot = by_id[root_id].authored_pose.start
    members = _descendants(root_id, _children(links))
    return [
        replace(
            link,
            authored_pose=_transform_pose(
                link.authored_pose, pivot, rotation, translation
            ),
        )
        if link.id in members
        else link
        for link in links
    ]


def _adapt_collisions(
    links: list[V2PhysicalLink],
) -> tuple[list[V2PhysicalLink], list[CollisionRecord], list[CollisionRecord], list[CollisionAdjustment]]:
    intentional, active = _active_link_overlaps(links)
    adjustments: list[CollisionAdjustment] = []
    unresolved: list[CollisionRecord] = []
    attempts = 0
    while active and attempts < max(20, len(links) * 2):
        attempts += 1
        collision = active[0]
        by_id = {link.id: link for link in links}
        left, right = by_id[collision.body_a], by_id[collision.body_b]
        if left.role == "internode" and right.role != "internode":
            movable, obstacle = right, left
        elif right.role == "internode" and left.role != "internode":
            movable, obstacle = left, right
        else:
            movable, obstacle = max((left, right), key=lambda link: link.id), min(
                (left, right), key=lambda link: link.id
            )
        parent = by_id.get(movable.parent_id)
        azimuth_axis = (
            np.asarray(parent.authored_pose.direction)
            if parent is not None
            else np.asarray((0.0, 0.0, 1.0))
        )
        tilt_axis = (
            np.asarray(parent.authored_pose.rotation)[:, 0]
            if parent is not None
            else np.asarray((1.0, 0.0, 0.0))
        )
        candidate = None
        chosen = None
        def resolves_pair(trial_links: list[V2PhysicalLink]) -> bool:
            trial_by_id = {link.id: link for link in trial_links}
            trial_left = trial_by_id[collision.body_a]
            trial_right = trial_by_id[collision.body_b]
            return capsule_capsule_overlap(
                authored_capsule_pose(
                    trial_left.authored_pose, trial_left.collider_radius
                ),
                authored_capsule_radius(
                    trial_left.authored_pose, trial_left.collider_radius
                ),
                authored_capsule_pose(
                    trial_right.authored_pose, trial_right.collider_radius
                ),
                authored_capsule_radius(
                    trial_right.authored_pose, trial_right.collider_radius
                ),
                margin=COLLISION_MARGIN_M,
            ) <= 0.0

        for mode, axis, maximum in (
            ("azimuth", azimuth_axis, MAX_AZIMUTH_CORRECTION_DEG),
            ("tilt", tilt_axis, MAX_TILT_CORRECTION_DEG),
        ):
            for magnitude in np.arange(0.5, maximum + 0.25, 0.5):
                for signed in (float(magnitude), -float(magnitude)):
                    trial = _replace_subtree(links, movable.id, _rotation(axis, signed))
                    if resolves_pair(trial):
                        candidate = trial
                        chosen = CollisionAdjustment(
                            link_id=movable.id,
                            reason=collision.reason,
                            azimuth_delta_deg=signed if mode == "azimuth" else 0.0,
                            tilt_delta_deg=signed if mode == "tilt" else 0.0,
                        )
                        break
                if candidate is not None:
                    break
            if candidate is not None:
                break
        if candidate is None:
            obstacle_start = np.asarray(obstacle.authored_pose.start)
            distance, fraction = _point_segment_distance(
                movable.authored_pose.start,
                obstacle.authored_pose.start,
                obstacle.authored_pose.end,
            )
            closest = obstacle_start + fraction * (
                np.asarray(obstacle.authored_pose.end) - obstacle_start
            )
            direction = np.asarray(movable.authored_pose.start) - closest
            if np.linalg.norm(direction) <= 1e-12:
                direction = tilt_axis
            direction /= np.linalg.norm(direction)
            for shift in np.arange(0.00025, MAX_POSITION_SHIFT_M + 0.000125, 0.00025):
                trial = _replace_subtree(
                    links, movable.id, np.eye(3), direction * shift * CANONICAL_SCALE
                )
                if resolves_pair(trial):
                    candidate = trial
                    chosen = CollisionAdjustment(
                        link_id=movable.id,
                        reason=collision.reason,
                        shift_m=float(shift),
                    )
                    break
        if candidate is None:
            unresolved.append(collision)
            # Remove this pair from subsequent planning; it will be filtered in USD.
            active = [
                item
                for item in active[1:]
                if {item.body_a, item.body_b}
                != {collision.body_a, collision.body_b}
            ]
        else:
            links = candidate
            adjustments.append(chosen)
            intentional, active = _active_link_overlaps(links)
            active = [
                item
                for item in active
                if {item.body_a, item.body_b}
                not in ({record.body_a, record.body_b} for record in unresolved)
            ]
    unresolved.extend(active)
    intentional, _ = _active_link_overlaps(links)
    return links, intentional, unresolved, adjustments


def _remap_visual_geometry(
    visual_axes: list[V2VisualAxis],
    visual_spheres: list[V2VisualSphere],
    links: list[V2PhysicalLink],
    source_pose_by_axis: dict[str, Pose],
) -> tuple[list[V2VisualAxis], list[V2VisualSphere]]:
    link_by_id = {link.id: link for link in links}
    axes = [
        replace(
            axis,
            authored_pose=_map_pose_through_host(
                axis.source_pose,
                source_pose_by_axis[axis.host_link_id],
                link_by_id[axis.host_link_id].authored_pose,
            ),
        )
        for axis in visual_axes
    ]
    spheres = []
    for sphere in visual_spheres:
        source_host = source_pose_by_axis[sphere.host_link_id]
        authored_host = link_by_id[sphere.host_link_id].authored_pose
        local = np.asarray(source_host.rotation).T @ (
            np.asarray(sphere.source_center) - np.asarray(source_host.start)
        )
        center = np.asarray(authored_host.start) + np.asarray(
            authored_host.rotation
        ) @ local
        spheres.append(
            replace(
                sphere,
                authored_center=tuple(float(value) for value in center),
            )
        )
    return axes, spheres


def _terminal_overlaps(
    spheres: list[V2VisualSphere], links: list[V2PhysicalLink]
) -> tuple[list[CollisionRecord], list[CollisionRecord]]:
    intentional: list[CollisionRecord] = []
    active: list[CollisionRecord] = []
    rendered = [sphere for sphere in spheres if sphere.render_geometry]
    for sphere in rendered:
        for link in links:
            overlap = sphere_capsule_overlap(
                sphere.authored_center,
                sphere.radius,
                authored_capsule_pose(link.authored_pose, link.collider_radius),
                authored_capsule_radius(link.authored_pose, link.collider_radius),
                margin=COLLISION_MARGIN_M,
            )
            if overlap <= 0.0:
                continue
            is_host = link.id == sphere.host_link_id
            record = CollisionRecord(
                body_a=sphere.id,
                body_b=link.id,
                kind="sphere_capsule",
                overlap_m=float(overlap),
                reason=(
                    "intentional_fruit_attachment_contact"
                    if is_host
                    else "fruit_axis_overlap"
                ),
            )
            (intentional if is_host else active).append(record)
    for index, left in enumerate(rendered):
        for right in rendered[index + 1 :]:
            overlap = sphere_sphere_overlap(
                left.authored_center,
                left.radius,
                right.authored_center,
                right.radius,
                margin=COLLISION_MARGIN_M,
            )
            if overlap > 0.0:
                active.append(
                    CollisionRecord(
                        body_a=left.id,
                        body_b=right.id,
                        kind="sphere_sphere",
                        overlap_m=float(overlap),
                        reason="fruit_fruit_overlap",
                    )
                )
    return intentional, active


def _adapt_terminal_collisions(
    links: list[V2PhysicalLink],
    visual_axes: list[V2VisualAxis],
    visual_spheres: list[V2VisualSphere],
    source_pose_by_axis: dict[str, Pose],
) -> tuple[
    list[V2PhysicalLink],
    list[V2VisualAxis],
    list[V2VisualSphere],
    list[CollisionRecord],
    list[CollisionRecord],
    list[CollisionAdjustment],
]:
    """Move the fruit support subtree before filtering terminal overlaps."""

    intentional, active = _terminal_overlaps(visual_spheres, links)
    initial_active_count = len(active)
    _, initially_active_links = _active_link_overlaps(links)
    permitted_link_pairs = {
        frozenset((record.body_a, record.body_b))
        for record in initially_active_links
    }
    ignored: set[frozenset[str]] = set()
    processed: set[frozenset[str]] = set()
    unresolved: list[CollisionRecord] = []
    adjustments: list[CollisionAdjustment] = []
    maximum_processed_pairs = max(40, initial_active_count * 2)
    while active and len(processed) < maximum_processed_pairs:
        collision = next(
            (
                item
                for item in active
                if frozenset((item.body_a, item.body_b)) not in processed
            ),
            None,
        )
        if collision is None:
            break
        pair_key = frozenset((collision.body_a, collision.body_b))
        spheres_by_id = {sphere.id: sphere for sphere in visual_spheres}
        links_by_id = {link.id: link for link in links}
        left_sphere = spheres_by_id[collision.body_a]
        if collision.kind == "sphere_capsule":
            other_center = None
            obstacle_link = links_by_id[collision.body_b]
            movable_host_id = left_sphere.host_link_id
        else:
            right_sphere = spheres_by_id[collision.body_b]
            movable_sphere = max((left_sphere, right_sphere), key=lambda item: item.id)
            movable_host_id = movable_sphere.host_link_id
            left_sphere = movable_sphere
            other_center = np.asarray(
                right_sphere.authored_center
                if movable_sphere.id == left_sphere.id and right_sphere.id != left_sphere.id
                else spheres_by_id[collision.body_a].authored_center
            )
            obstacle_link = None
        movable = links_by_id[movable_host_id]
        parent = links_by_id.get(movable.parent_id)
        azimuth_axis = np.asarray(
            parent.authored_pose.direction if parent else (0.0, 0.0, 1.0)
        )
        tilt_axis = np.asarray(parent.authored_pose.rotation)[:, 0] if parent else np.asarray((1.0, 0.0, 0.0))

        def trial_geometry(trial_links):
            return _remap_visual_geometry(
                visual_axes, visual_spheres, trial_links, source_pose_by_axis
            )

        def resolves(trial_links, trial_spheres) -> bool:
            trial_sphere_map = {sphere.id: sphere for sphere in trial_spheres}
            trial_link_map = {link.id: link for link in trial_links}
            sphere = trial_sphere_map[collision.body_a]
            if collision.kind == "sphere_capsule":
                obstacle = trial_link_map[collision.body_b]
                return sphere_capsule_overlap(
                    sphere.authored_center,
                    sphere.radius,
                    authored_capsule_pose(obstacle.authored_pose, obstacle.collider_radius),
                    authored_capsule_radius(obstacle.authored_pose, obstacle.collider_radius),
                    margin=COLLISION_MARGIN_M,
                ) <= 0.0
            right = trial_sphere_map[collision.body_b]
            return sphere_sphere_overlap(
                sphere.authored_center,
                sphere.radius,
                right.authored_center,
                right.radius,
                margin=COLLISION_MARGIN_M,
            ) <= 0.0

        def acceptable(trial_links, trial_spheres) -> bool:
            if not resolves(trial_links, trial_spheres):
                return False
            _, trial_active_links = _active_link_overlaps(trial_links)
            return all(
                frozenset((record.body_a, record.body_b)) in permitted_link_pairs
                for record in trial_active_links
            )

        candidate = None
        chosen = None
        candidate_axes = candidate_spheres = None
        for mode, axis, maximum in (
            ("azimuth", azimuth_axis, MAX_AZIMUTH_CORRECTION_DEG),
            ("tilt", tilt_axis, MAX_TILT_CORRECTION_DEG),
        ):
            magnitudes = (
                (1.0, 2.5, 5.0)
                if maximum == MAX_AZIMUTH_CORRECTION_DEG
                else (1.0, 2.0, 3.0)
            )
            for magnitude in magnitudes:
                for signed in (float(magnitude), -float(magnitude)):
                    trial = _replace_subtree(links, movable_host_id, _rotation(axis, signed))
                    trial_axes, trial_spheres = trial_geometry(trial)
                    if acceptable(trial, trial_spheres):
                        candidate, candidate_axes, candidate_spheres = trial, trial_axes, trial_spheres
                        chosen = CollisionAdjustment(
                            link_id=movable_host_id,
                            reason=collision.reason,
                            azimuth_delta_deg=signed if mode == "azimuth" else 0.0,
                            tilt_delta_deg=signed if mode == "tilt" else 0.0,
                        )
                        break
                if candidate is not None:
                    break
            if candidate is not None:
                break
        if candidate is None:
            movable_center = np.asarray(left_sphere.authored_center)
            if obstacle_link is not None:
                capsule = authored_capsule_pose(
                    obstacle_link.authored_pose, obstacle_link.collider_radius
                )
                _, fraction = _point_segment_distance(
                    movable_center, capsule.start, capsule.end
                )
                closest = np.asarray(capsule.start) + fraction * (
                    np.asarray(capsule.end) - np.asarray(capsule.start)
                )
            else:
                closest = other_center
            direction = movable_center - closest
            if np.linalg.norm(direction) <= 1e-12:
                direction = tilt_axis
            direction /= np.linalg.norm(direction)
            for shift in (0.0005, 0.001, 0.002):
                trial = _replace_subtree(
                    links,
                    movable_host_id,
                    np.eye(3),
                    direction * shift * CANONICAL_SCALE,
                )
                trial_axes, trial_spheres = trial_geometry(trial)
                if acceptable(trial, trial_spheres):
                    candidate, candidate_axes, candidate_spheres = trial, trial_axes, trial_spheres
                    chosen = CollisionAdjustment(
                        link_id=movable_host_id,
                        reason=collision.reason,
                        shift_m=float(shift),
                    )
                    break
        if candidate is None:
            unresolved.append(
                replace(
                    collision,
                    reason=f"{collision.reason}; correction limits exhausted",
                )
            )
            ignored.add(frozenset((collision.body_a, collision.body_b)))
        else:
            links = candidate
            visual_axes = candidate_axes
            visual_spheres = candidate_spheres
            adjustments.append(chosen)
        processed.add(pair_key)
        intentional, active = _terminal_overlaps(visual_spheres, links)
    recorded_pairs = {
        frozenset((record.body_a, record.body_b)) for record in unresolved
    }
    unresolved.extend(
        replace(record, reason=f"{record.reason}; correction limits exhausted")
        for record in active
        if frozenset((record.body_a, record.body_b)) not in recorded_pairs
    )
    return links, visual_axes, visual_spheres, intentional, unresolved, adjustments


def _host_for_visual_axis(
    axis: AxisGeometry,
    physical_axes: list[AxisGeometry],
    parent_ids: dict[str, str | None],
) -> str:
    if axis.id in parent_ids:
        return axis.id
    candidates = []
    for physical in physical_axes:
        distance, fraction = _point_segment_distance(
            axis.world_start, physical.world_start, physical.world_end
        )
        if distance <= _GEOMETRY_TOLERANCE:
            same_owner = axis.owner_node_id == physical.owner_node_id
            candidates.append(
                (0 if same_owner else 1, 0 if abs(1.0 - fraction) < 1e-7 else 1, physical.id)
            )
    if not candidates:
        raise V2PlantStateError(f"visual axis {axis.id} has no physical host")
    return min(candidates)[2]


def _host_for_sphere(sphere: SphereGeometry, physical_axes: list[AxisGeometry]) -> str:
    candidates = []
    for physical in physical_axes:
        if physical.owner_node_id != sphere.owner_node_id:
            continue
        distance = float(
            np.linalg.norm(np.asarray(sphere.world_center) - np.asarray(physical.world_end))
        )
        role_priority = 0 if physical.role == "pedicel" else 1
        candidates.append((distance, role_priority, physical.id))
    if not candidates:
        raise V2PlantStateError(f"sphere {sphere.id} has no physical host")
    return min(candidates)[2]


def _map_pose_through_host(source: Pose, source_host: Pose, authored_host: Pose) -> Pose:
    source_rotation = np.asarray(source_host.rotation)
    authored_rotation = np.asarray(authored_host.rotation)
    local_start = source_rotation.T @ (
        np.asarray(source.start) - np.asarray(source_host.start)
    )
    local_end = source_rotation.T @ (
        np.asarray(source.end) - np.asarray(source_host.start)
    )
    local_rotation = source_rotation.T @ np.asarray(source.rotation)
    start = np.asarray(authored_host.start) + authored_rotation @ local_start
    end = np.asarray(authored_host.start) + authored_rotation @ local_end
    rotation = authored_rotation @ local_rotation
    return Pose(
        tuple(float(value) for value in start),
        tuple(float(value) for value in end),
        tuple(tuple(float(value) for value in row) for row in rotation),
    )


def build_v2_authoring_plan(
    state: PlantState,
    *,
    physics_preset: str = "flexible",
    allow_near_budget: bool = False,
    optimize: bool = False,
) -> V2AuthoringPlan:
    """Build a deterministic, collision-adapted V2 plan from PlantState."""

    validate_plant_state(state)
    if physics_preset not in {"locked", "flexible"}:
        raise V2PlantStateError(
            f"physics_preset must be 'locked' or 'flexible', got {physics_preset!r}"
        )
    nodes = {node.id: node for node in state.nodes}
    if state.root_node_id not in nodes:
        raise V2PlantStateError(f"missing root node {state.root_node_id}")
    origin = np.asarray(nodes[state.root_node_id].pose.world_start, dtype=np.float64)
    physical_axes = [axis for axis in state.axes if axis.role in PHYSICAL_AXIS_ROLES]
    unsupported = sorted(
        {axis.role for axis in state.axes}
        - PHYSICAL_AXIS_ROLES
        - VISUAL_ONLY_AXIS_ROLES
    )
    if unsupported:
        raise V2PlantStateError(f"unsupported canonical axis roles: {unsupported}")
    parent_ids = _physical_parent_ids(state, physical_axes)
    organ_by_node = {organ.node_id: organ for organ in state.organs}
    links = []
    source_pose_by_axis = {}
    for axis in physical_axes:
        pose = _pose_for_axis(axis, origin, CANONICAL_SCALE)
        source_pose_by_axis[axis.id] = pose
        organ = organ_by_node[axis.owner_node_id]
        joint_type = "fixed" if physics_preset == "locked" else "d6"
        links.append(
            V2PhysicalLink(
                id=axis.id,
                canonical_axis_id=axis.id,
                owner_node_id=axis.owner_node_id,
                role=axis.role,
                parent_id=parent_ids[axis.id],
                source_pose=pose,
                authored_pose=pose,
                visual_radius=float(axis.radius * CANONICAL_SCALE),
                collider_radius=max(
                    float(axis.radius * CANONICAL_SCALE * COLLIDER_RADIUS_SCALE),
                    MIN_COLLIDER_RADIUS_WORLD,
                ),
                joint_type=joint_type,
                canonical_organ_ids=(organ.id,),
                canonical_primitive_ids=(axis.id,),
            )
        )
    raw_dynamic_count = (
        sum(link.parent_id is not None for link in links)
        if physics_preset == "flexible"
        else 0
    )
    validate_joint_budget(
        raw_dynamic_count,
        allow_near_budget=allow_near_budget,
        optimize=optimize,
    )
    # Canonical days 1/25/80 are already within budget.  For future larger
    # plants, deterministic fixed-joint aggregation preserves every visual.
    aggregated: list[str] = []
    if optimize and raw_dynamic_count > JOINT_TARGET:
        priority = {"pedicel": 0, "truss_rachis": 1, "leaf_rachis": 2, "petiole": 3}
        for index in sorted(
            range(len(links)),
            key=lambda value: (priority.get(links[value].role, 9), links[value].id),
        ):
            if sum(
                link.joint_type == "d6" and link.parent_id is not None
                for link in links
            ) <= JOINT_TARGET:
                break
            if links[index].parent_id is None:
                continue
            links[index] = replace(links[index], joint_type="fixed")
            aggregated.append(links[index].id)
    links, intentional, unresolved, adjustments = _adapt_collisions(links)
    link_by_id = {link.id: link for link in links}

    first_axis_signature: dict[str, str] = {}
    visual_axes = []
    duplicates: dict[str, str] = {}
    for axis in state.axes:
        source = _pose_for_axis(axis, origin, CANONICAL_SCALE)
        host_id = _host_for_visual_axis(axis, physical_axes, parent_ids)
        authored = _map_pose_through_host(
            source, source_pose_by_axis[host_id], link_by_id[host_id].authored_pose
        )
        signature = _exact_axis_signature(
            axis, nodes[axis.owner_node_id], organ_by_node[axis.owner_node_id]
        )
        original = first_axis_signature.setdefault(signature, axis.id)
        render = original == axis.id
        if not render:
            duplicates[axis.id] = original
        visual_axes.append(
            V2VisualAxis(
                id=axis.id,
                owner_node_id=axis.owner_node_id,
                organ_type=axis.organ_type,
                role=axis.role,
                source_pose=source,
                authored_pose=authored,
                radius=float(axis.radius * CANONICAL_SCALE),
                host_link_id=host_id,
                render_geometry=render,
                duplicate_of=None if render else original,
            )
        )

    first_sphere_signature: dict[str, str] = {}
    visual_spheres = []
    for sphere in state.spheres:
        source_center = tuple(
            float(value)
            for value in (np.asarray(sphere.world_center) - origin) * CANONICAL_SCALE
        )
        host_id = _host_for_sphere(sphere, physical_axes)
        source_host = source_pose_by_axis[host_id]
        authored_host = link_by_id[host_id].authored_pose
        local = np.asarray(source_host.rotation).T @ (
            np.asarray(source_center) - np.asarray(source_host.start)
        )
        authored_center = np.asarray(authored_host.start) + np.asarray(
            authored_host.rotation
        ) @ local
        signature = _exact_sphere_signature(
            sphere,
            nodes[sphere.owner_node_id],
            organ_by_node[sphere.owner_node_id],
        )
        original = first_sphere_signature.setdefault(signature, sphere.id)
        render = original == sphere.id
        if not render:
            duplicates[sphere.id] = original
        visual_spheres.append(
            V2VisualSphere(
                id=sphere.id,
                owner_node_id=sphere.owner_node_id,
                organ_type=sphere.organ_type,
                role=sphere.role,
                source_center=source_center,
                authored_center=tuple(float(value) for value in authored_center),
                radius=float(sphere.radius * CANONICAL_SCALE),
                host_link_id=host_id,
                render_geometry=render,
                duplicate_of=None if render else original,
            )
        )

    # Fruit spheres are separate rigid bodies.  Apply the same bounded
    # azimuth/tilt/shift policy to their support subtree, then filter only the
    # pairs that cannot be separated within the declared limits.
    (
        links,
        visual_axes,
        visual_spheres,
        terminal_intentional,
        terminal_unresolved,
        terminal_adjustments,
    ) = _adapt_terminal_collisions(
        links, visual_axes, visual_spheres, source_pose_by_axis
    )
    link_intentional, newly_active_links = _active_link_overlaps(links)
    existing_pairs = {
        frozenset((record.body_a, record.body_b)) for record in unresolved
    }
    unresolved.extend(
        replace(record, reason=f"{record.reason}; introduced by terminal correction")
        for record in newly_active_links
        if frozenset((record.body_a, record.body_b)) not in existing_pairs
    )
    intentional = [*link_intentional, *terminal_intentional]
    unresolved.extend(terminal_unresolved)
    adjustments.extend(terminal_adjustments)

    diagnostics = {
        "canonical_axis_count": len(state.axes),
        "canonical_sphere_count": len(state.spheres),
        "physical_axis_count": len(links),
        "visual_only_axis_count": len(state.axes) - len(links),
        "predicted_d6_joints": sum(
            link.joint_type == "d6" and link.parent_id is not None
            for link in links
        ),
        "joint_target": JOINT_TARGET,
        "joint_warning_max": JOINT_WARNING_MAX,
        "aggregated_physical_link_ids": sorted(aggregated),
        "duplicate_geometry_of": dict(sorted(duplicates.items())),
        "axis_roles": dict(sorted(Counter(axis.role for axis in state.axes).items())),
        "collider_radius_scale": COLLIDER_RADIUS_SCALE,
        "collider_length_scale": COLLIDER_LENGTH_SCALE,
        "minimum_collider_radius_world_m": MIN_COLLIDER_RADIUS_WORLD,
    }
    return V2AuthoringPlan(
        state=state,
        source_origin=tuple(float(value) for value in origin),
        scale=CANONICAL_SCALE,
        physics_preset=physics_preset,
        visual_axes=tuple(visual_axes),
        visual_spheres=tuple(visual_spheres),
        physical_links=tuple(links),
        intentional_collision_filters=tuple(
            sorted(intentional, key=lambda item: (item.body_a, item.body_b))
        ),
        unresolved_collision_filters=tuple(
            sorted(unresolved, key=lambda item: (item.body_a, item.body_b))
        ),
        collision_adjustments=tuple(adjustments),
        diagnostics=diagnostics,
    )
