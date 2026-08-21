"""Deterministic reconstruction of GroIMP turtle frames from ProjectGraph edges."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable

import numpy as np

from .models import GraphNode, GroIMPGraphSnapshot


Vector3 = tuple[float, float, float]
Matrix4 = tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]


class TurtleResolutionError(ValueError):
    """Raised when a ProjectGraph cannot be resolved without ambiguity."""


def _matrix_tuple(value: Any) -> Matrix4:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (4, 4):
        raise ValueError(f"Expected a 4x4 turtle matrix, got shape {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError("Turtle matrices must contain only finite values")
    if not np.allclose(array[3], (0.0, 0.0, 0.0, 1.0), atol=1e-12):
        raise ValueError("The last row of a turtle matrix must be [0, 0, 0, 1]")
    return tuple(tuple(float(item) for item in row) for row in array)  # type: ignore[return-value]


@dataclass(frozen=True)
class TurtleFrame:
    """A local-to-world frame using GroIMP's left/up/head basis convention.

    Column vectors are used. Matrix columns 0, 1 and 2 are respectively the
    local left (X), up (Y), and head (Z) axes; column 3 is world position.
    """

    matrix: Matrix4

    def __post_init__(self) -> None:
        object.__setattr__(self, "matrix", _matrix_tuple(self.matrix))

    @classmethod
    def identity(cls) -> TurtleFrame:
        return cls(
            (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        )

    @classmethod
    def from_matrix(cls, matrix: Any) -> TurtleFrame:
        return cls(_matrix_tuple(matrix))

    @property
    def position(self) -> Vector3:
        return tuple(self.matrix[row][3] for row in range(3))  # type: ignore[return-value]

    @property
    def left(self) -> Vector3:
        return tuple(self.matrix[row][0] for row in range(3))  # type: ignore[return-value]

    @property
    def up(self) -> Vector3:
        return tuple(self.matrix[row][1] for row in range(3))  # type: ignore[return-value]

    @property
    def head(self) -> Vector3:
        return tuple(self.matrix[row][2] for row in range(3))  # type: ignore[return-value]

    def compose_local(self, local_matrix: Any) -> TurtleFrame:
        """Post-multiply one local operation: ``world @ local``."""

        return TurtleFrame.from_matrix(
            np.asarray(self.matrix, dtype=np.float64)
            @ np.asarray(local_matrix, dtype=np.float64)
        )

    def rotate_local(self, axis: str, angle_degrees: float) -> TurtleFrame:
        radians = math.radians(float(angle_degrees))
        cosine = math.cos(radians)
        sine = math.sin(radians)
        if axis == "left":
            rotation = (
                (1.0, 0.0, 0.0, 0.0),
                (0.0, cosine, -sine, 0.0),
                (0.0, sine, cosine, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        elif axis == "up":
            rotation = (
                (cosine, 0.0, sine, 0.0),
                (0.0, 1.0, 0.0, 0.0),
                (-sine, 0.0, cosine, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        elif axis == "head":
            rotation = (
                (cosine, -sine, 0.0, 0.0),
                (sine, cosine, 0.0, 0.0),
                (0.0, 0.0, 1.0, 0.0),
                (0.0, 0.0, 0.0, 1.0),
            )
        else:
            raise ValueError(f"Unknown local turtle axis: {axis!r}")
        return self.compose_local(rotation)

    def translate_local(self, x: float, y: float, z: float) -> TurtleFrame:
        translation = np.identity(4, dtype=np.float64)
        translation[:3, 3] = (float(x), float(y), float(z))
        return self.compose_local(translation)

    def align_head_to_negative_world_z(self) -> TurtleFrame:
        """Apply GroIMP ``RG``: minimal rotation of head toward gravity."""

        matrix = np.asarray(self.matrix, dtype=np.float64)
        rotation = matrix[:3, :3]
        head = rotation[:, 2]
        head = head / np.linalg.norm(head)
        target = np.array((0.0, 0.0, -1.0), dtype=np.float64)
        dot = float(np.clip(np.dot(head, target), -1.0, 1.0))
        if dot >= 1.0 - 1e-12:
            return self
        if dot <= -1.0 + 1e-12:
            # GroIMP resolves the antiparallel case around the local left axis.
            return self.rotate_local("left", 180.0)

        axis = np.cross(head, target)
        axis /= np.linalg.norm(axis)
        x, y, z = axis
        skew = np.array(
            ((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)),
            dtype=np.float64,
        )
        world_rotation = (
            np.identity(3, dtype=np.float64)
            + skew * math.sqrt(max(0.0, 1.0 - dot * dot))
            + (skew @ skew) * (1.0 - dot)
        )
        result = matrix.copy()
        result[:3, :3] = world_rotation @ rotation
        return TurtleFrame.from_matrix(result)


@dataclass(frozen=True)
class ResolvedNodePose:
    """Incoming and outgoing world frame for one ProjectGraph node."""

    node_id: int
    node_type: str
    incoming_frame: TurtleFrame
    outgoing_frame: TurtleFrame
    effect: str

    @property
    def start_position(self) -> Vector3:
        return self.incoming_frame.position

    @property
    def end_position(self) -> Vector3:
        return self.outgoing_frame.position


@dataclass
class TurtleResolution:
    """Resolved node poses plus deterministic traversal diagnostics."""

    poses: dict[int, ResolvedNodePose]
    traversal_order: tuple[int, ...]
    diagnostics: dict[str, Any] = field(default_factory=dict)


_STRUCTURAL_EDGE_KINDS = frozenset({"successor", "branch"})
_KNOWN_PASSTHROUGH_TYPES = frozenset(
    {
        "Node",
        "RGGRoot",
        "Root",
        "Leaf",
        "Truss",
        "Fruits",
        "Meristem",
        "PlantBase",
        "CropBase",
        "Probe",
        "FrameProbe",
    }
)


def _simple_type(node_type: str) -> str:
    return node_type.rsplit(".", 1)[-1]


def _required_number(node: GraphNode, name: str) -> float:
    try:
        value = float(node.attributes[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise TurtleResolutionError(
            f"Node {node.id} ({node.type}) requires numeric attribute {name!r}"
        ) from exc
    if not math.isfinite(value):
        raise TurtleResolutionError(
            f"Node {node.id} ({node.type}) has non-finite attribute {name!r}"
        )
    return value


def _apply_node(
    node: GraphNode,
    incoming: TurtleFrame,
    unsupported_types: set[str],
    internode_advances: dict[int, float],
) -> tuple[TurtleFrame, str]:
    node_type = _simple_type(node.type)
    if node_type == "RH":
        return incoming.rotate_local("head", _required_number(node, "angle")), "rotate_head"
    if node_type == "RL":
        return incoming.rotate_local("left", _required_number(node, "angle")), "rotate_left"
    if node_type == "RU":
        return incoming.rotate_local("up", _required_number(node, "angle")), "rotate_up"
    if node_type == "RG":
        return incoming.align_head_to_negative_world_z(), "align_gravity"
    if node_type == "Translate":
        return (
            incoming.translate_local(
                _required_number(node, "translateX"),
                _required_number(node, "translateY"),
                _required_number(node, "translateZ"),
            ),
            "translate",
        )
    if node_type == "Internode":
        if node.id in internode_advances:
            return (
                incoming.translate_local(0.0, 0.0, internode_advances[node.id]),
                "advance_anchor_calibrated",
            )
        return incoming.translate_local(0.0, 0.0, _required_number(node, "length")), "advance"
    if node_type not in _KNOWN_PASSTHROUGH_TYPES:
        unsupported_types.add(node.type)
    return incoming, "passthrough"


def _infer_internode_advances(
    snapshot: GroIMPGraphSnapshot,
    nodes_by_id: dict[int, GraphNode],
) -> tuple[dict[int, float], dict[str, Any]]:
    """Recover the effective M-step from adjacent GroIMP world anchors.

    The tomato model mutates ``M.length`` during a simulation step. GroIMP's
    rendered turtle cache can consequently retain the previous effective step
    while the public organ attribute already contains the new biological
    length. Adjacent native anchors are therefore the authoritative oracle when
    present. Snapshots without anchors retain the declared-length fallback.
    """

    successors: dict[int, list[int]] = {}
    for edge in snapshot.edges:
        if edge.kind == "successor":
            successors.setdefault(edge.source, []).append(edge.target)

    advances: dict[int, float] = {}
    deltas: list[float] = []
    rejected: list[dict[str, Any]] = []
    for node in snapshot.nodes:
        if _simple_type(node.type) != "Internode" or node.world_anchor is None:
            continue
        targets = sorted(successors.get(node.id, []))
        if len(targets) != 1:
            if targets:
                rejected.append({"node_id": node.id, "reason": "multiple_successors"})
            continue
        target = nodes_by_id.get(targets[0])
        if (
            target is None
            or target.world_anchor is None
            or _simple_type(target.type) == "Translate"
        ):
            continue
        direction = np.asarray(node.world_anchor.direction, dtype=np.float64)
        norm = float(np.linalg.norm(direction))
        if norm <= 1e-12:
            rejected.append({"node_id": node.id, "reason": "zero_direction"})
            continue
        head = direction / norm
        displacement = np.asarray(target.world_anchor.position, dtype=np.float64) - np.asarray(
            node.world_anchor.position,
            dtype=np.float64,
        )
        advance = float(np.dot(displacement, head))
        residual = float(np.linalg.norm(displacement - advance * head))
        tolerance = max(1e-7, abs(advance) * 1e-5)
        if advance < -tolerance or residual > tolerance:
            rejected.append(
                {
                    "node_id": node.id,
                    "reason": "non_axial_successor",
                    "residual": residual,
                }
            )
            continue
        advances[node.id] = max(0.0, advance)
        declared = node.attributes.get("length")
        if isinstance(declared, (int, float)) and math.isfinite(float(declared)):
            deltas.append(abs(float(declared) - advance))

    return advances, {
        "anchor_calibrated_internodes": len(advances),
        "max_declared_vs_effective_length_delta": max(deltas, default=0.0),
        "rejected_internode_anchor_calibrations": rejected,
    }


def _cycle_nodes(children: dict[int, list[int]], roots: Iterable[int]) -> list[int]:
    completed: set[int] = set()
    active: set[int] = set()

    def visit(node_id: int) -> list[int]:
        if node_id in active:
            return [node_id]
        if node_id in completed:
            return []
        active.add(node_id)
        for child_id in children.get(node_id, []):
            found = visit(child_id)
            if found:
                return [node_id, *found]
        active.remove(node_id)
        completed.add(node_id)
        return []

    for root_id in roots:
        found = visit(root_id)
        if found:
            return found
    return []


def resolve_turtle(
    snapshot: GroIMPGraphSnapshot,
    *,
    initial_frame: TurtleFrame | None = None,
    strict: bool = True,
) -> TurtleResolution:
    """Resolve GroIMP turtle state over successor and branch edges.

    Every child receives its parent's outgoing frame. Branches are evaluated
    independently, so a branch cannot mutate the frame used by its siblings or
    by the successor path. Unknown edge kinds are retained as diagnostics and
    are deliberately not interpreted as structural edges.
    """

    nodes_by_id = {node.id: node for node in snapshot.nodes}
    if len(nodes_by_id) != len(snapshot.nodes):
        raise TurtleResolutionError("ProjectGraph contains duplicate node IDs")
    if not nodes_by_id:
        return TurtleResolution(
            poses={},
            traversal_order=(),
            diagnostics={
                "root_id": None,
                "unknown_edge_codes": [],
                "unsupported_node_types": [],
                "unresolved_node_ids": [],
            },
        )

    children_with_kind: dict[int, list[tuple[str, int]]] = {}
    parents: dict[int, int] = {}
    unknown_edge_codes: set[int] = set()
    missing_edge_nodes: list[tuple[int, int]] = []
    multiple_parents: dict[int, set[int]] = {}

    for edge in sorted(
        snapshot.edges,
        key=lambda item: (item.source, item.target, item.raw_code),
    ):
        if edge.kind not in _STRUCTURAL_EDGE_KINDS:
            unknown_edge_codes.add(edge.raw_code)
            continue
        if edge.source not in nodes_by_id or edge.target not in nodes_by_id:
            missing_edge_nodes.append((edge.source, edge.target))
            continue
        previous_parent = parents.get(edge.target)
        if previous_parent is not None and previous_parent != edge.source:
            multiple_parents.setdefault(edge.target, {previous_parent}).add(edge.source)
            continue
        parents[edge.target] = edge.source
        children_with_kind.setdefault(edge.source, []).append((edge.kind, edge.target))

    if strict and missing_edge_nodes:
        raise TurtleResolutionError(
            f"Structural edges reference missing nodes: {missing_edge_nodes}"
        )
    if strict and multiple_parents:
        detail = {key: sorted(value) for key, value in sorted(multiple_parents.items())}
        raise TurtleResolutionError(f"Nodes have multiple structural parents: {detail}")

    kind_order = {"successor": 0, "branch": 1}
    for outgoing in children_with_kind.values():
        outgoing.sort(key=lambda item: (kind_order[item[0]], item[1]))
    children = {
        node_id: [target for _, target in outgoing]
        for node_id, outgoing in children_with_kind.items()
    }

    candidate_roots = sorted(node_id for node_id in nodes_by_id if node_id not in parents)
    root_id = snapshot.root_id if snapshot.root_id in nodes_by_id else None
    if root_id is None:
        root_id = candidate_roots[0] if candidate_roots else min(nodes_by_id)

    internode_advances, advance_diagnostics = _infer_internode_advances(
        snapshot,
        nodes_by_id,
    )

    cycle = _cycle_nodes(children, candidate_roots or [root_id])
    if cycle and strict:
        raise TurtleResolutionError(f"ProjectGraph contains a structural cycle: {cycle}")

    poses: dict[int, ResolvedNodePose] = {}
    traversal_order: list[int] = []
    unsupported_types: set[str] = set()
    active: set[int] = set()

    def visit(node_id: int, incoming: TurtleFrame) -> None:
        if node_id in active or node_id in poses:
            return
        active.add(node_id)
        node = nodes_by_id[node_id]
        outgoing, effect = _apply_node(
            node,
            incoming,
            unsupported_types,
            internode_advances,
        )
        poses[node_id] = ResolvedNodePose(
            node_id=node_id,
            node_type=node.type,
            incoming_frame=incoming,
            outgoing_frame=outgoing,
            effect=effect,
        )
        traversal_order.append(node_id)
        for child_id in children.get(node_id, []):
            visit(child_id, outgoing)
        active.remove(node_id)

    visit(root_id, initial_frame or TurtleFrame.identity())
    unresolved = sorted(set(nodes_by_id) - set(poses))
    diagnostics = {
        "root_id": root_id,
        "unknown_edge_codes": sorted(unknown_edge_codes),
        "missing_edge_nodes": [list(item) for item in missing_edge_nodes],
        "multiple_parents": {
            str(key): sorted(value) for key, value in sorted(multiple_parents.items())
        },
        "unsupported_node_types": sorted(unsupported_types),
        "unresolved_node_ids": unresolved,
        "coordinate_convention": "local_to_world_columns_left_up_head",
        "composition": "world_at_local",
        "angle_unit": "degrees",
        **advance_diagnostics,
    }
    if cycle:
        diagnostics["cycle"] = cycle
    return TurtleResolution(
        poses=poses,
        traversal_order=tuple(traversal_order),
        diagnostics=diagnostics,
    )
