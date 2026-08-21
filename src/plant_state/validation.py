"""Structural and numeric validation for canonical plant states."""

from __future__ import annotations

from dataclasses import asdict
import math
from typing import Any, Iterable, Mapping, Sequence

from .models import (
    AxisGeometry,
    FruitsProperties,
    InternodeProperties,
    LeafProperties,
    Matrix4,
    MeristemProperties,
    PlantBaseProperties,
    PlantState,
    RootProperties,
    SphereGeometry,
    TrussProperties,
)
from .schema import (
    PLANT_STATE_SCHEMA_VERSION,
    STRUCTURAL_EDGE_KINDS,
    SUPPORTED_ORGAN_TYPES,
    TURTLE_OPERATION_TYPES,
)


class PlantStateValidationError(ValueError):
    """Raised when a canonical state violates schema invariants."""

    def __init__(self, errors: Iterable[str]):
        self.errors = tuple(errors)
        super().__init__("Invalid PlantState: " + "; ".join(self.errors))


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(float(a) * float(b) for a, b in zip(left, right))


def _matrix_multiply(left: Matrix4, right: Matrix4) -> Matrix4:
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )  # type: ignore[return-value]


def _rigid_inverse(matrix: Matrix4) -> Matrix4:
    rotation = tuple(tuple(matrix[row][column] for column in range(3)) for row in range(3))
    translation = tuple(matrix[row][3] for row in range(3))
    inverse_rotation = tuple(tuple(rotation[column][row] for column in range(3)) for row in range(3))
    inverse_translation = tuple(-_dot(row, translation) for row in inverse_rotation)
    return tuple(
        tuple(inverse_rotation[row][column] for column in range(3)) + (inverse_translation[row],)
        for row in range(3)
    ) + ((0.0, 0.0, 0.0, 1.0),)  # type: ignore[return-value]


def _matrix_close(left: Matrix4, right: Matrix4, tolerance: float = 1e-9) -> bool:
    return all(
        math.isclose(left[row][column], right[row][column], rel_tol=0.0, abs_tol=tolerance)
        for row in range(4)
        for column in range(4)
    )


def _vector_close(left: Sequence[float], right: Sequence[float], tolerance: float = 1e-9) -> bool:
    return len(left) == len(right) and all(
        math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=tolerance)
        for a, b in zip(left, right)
    )


def _validate_matrix(name: str, matrix: Any, errors: list[str]) -> None:
    if len(matrix) != 4 or any(len(row) != 4 for row in matrix):
        errors.append(f"{name} is not 4x4")
        return
    if any(not math.isfinite(float(value)) for row in matrix for value in row):
        errors.append(f"{name} contains a non-finite value")
        return
    if not _vector_close(matrix[3], (0.0, 0.0, 0.0, 1.0), 1e-12):
        errors.append(f"{name} has an invalid homogeneous last row")
    columns = [tuple(float(matrix[row][column]) for row in range(3)) for column in range(3)]
    for index, column in enumerate(columns):
        if not math.isclose(_dot(column, column), 1.0, rel_tol=0.0, abs_tol=1e-8):
            errors.append(f"{name} rotation column {index} is not unit length")
    for left in range(3):
        for right in range(left + 1, 3):
            if not math.isclose(_dot(columns[left], columns[right]), 0.0, rel_tol=0.0, abs_tol=1e-8):
                errors.append(f"{name} rotation columns {left}/{right} are not orthogonal")
    determinant = (
        columns[0][0] * (columns[1][1] * columns[2][2] - columns[1][2] * columns[2][1])
        - columns[1][0] * (columns[0][1] * columns[2][2] - columns[0][2] * columns[2][1])
        + columns[2][0] * (columns[0][1] * columns[1][2] - columns[0][2] * columns[1][1])
    )
    if not math.isclose(determinant, 1.0, rel_tol=0.0, abs_tol=1e-8):
        errors.append(f"{name} rotation determinant is not +1")


def _duplicates(values: Iterable[str]) -> list[str]:
    seen: set[str] = set()
    duplicate: set[str] = set()
    for value in values:
        if value in seen:
            duplicate.add(value)
        seen.add(value)
    return sorted(duplicate)


def _validate_axis(axis: AxisGeometry, node_ids: set[str], errors: list[str]) -> None:
    if axis.owner_node_id not in node_ids:
        errors.append(f"axis {axis.id} references missing owner {axis.owner_node_id}")
    _validate_matrix(f"axis {axis.id} local_frame", axis.local_frame, errors)
    _validate_matrix(f"axis {axis.id} world_frame", axis.world_frame, errors)
    if axis.length < 0.0 or axis.radius < 0.0:
        errors.append(f"axis {axis.id} has negative dimensions")
    if not _vector_close(tuple(axis.world_frame[row][3] for row in range(3)), axis.world_start):
        errors.append(f"axis {axis.id} world frame does not start at world_start")
    if not _vector_close(tuple(axis.local_frame[row][3] for row in range(3)), axis.local_start):
        errors.append(f"axis {axis.id} local frame does not start at local_start")
    world_delta = tuple(axis.world_end[index] - axis.world_start[index] for index in range(3))
    local_delta = tuple(axis.local_end[index] - axis.local_start[index] for index in range(3))
    expected_world = tuple(axis.world_direction[index] * axis.length for index in range(3))
    expected_local = tuple(axis.local_direction[index] * axis.length for index in range(3))
    if not _vector_close(world_delta, expected_world):
        errors.append(f"axis {axis.id} world endpoint is inconsistent with direction/length")
    if not _vector_close(local_delta, expected_local):
        errors.append(f"axis {axis.id} local endpoint is inconsistent with direction/length")


def _validate_sphere(sphere: SphereGeometry, node_ids: set[str], errors: list[str]) -> None:
    if sphere.owner_node_id not in node_ids:
        errors.append(f"sphere {sphere.id} references missing owner {sphere.owner_node_id}")
    _validate_matrix(f"sphere {sphere.id} local_frame", sphere.local_frame, errors)
    _validate_matrix(f"sphere {sphere.id} world_frame", sphere.world_frame, errors)
    if sphere.radius < 0.0:
        errors.append(f"sphere {sphere.id} has negative radius")
    if not _vector_close(tuple(sphere.world_frame[row][3] for row in range(3)), sphere.world_center):
        errors.append(f"sphere {sphere.id} world frame does not match its center")
    if not _vector_close(tuple(sphere.local_frame[row][3] for row in range(3)), sphere.local_center):
        errors.append(f"sphere {sphere.id} local frame does not match its center")


def validate_plant_state(state: PlantState, *, strict: bool = True) -> tuple[str, ...]:
    """Validate a state, raising in strict mode and returning all errors otherwise."""

    errors: list[str] = []
    if state.schema_version != PLANT_STATE_SCHEMA_VERSION:
        errors.append(
            f"unsupported schema version {state.schema_version!r}; expected {PLANT_STATE_SCHEMA_VERSION!r}"
        )
    if state.metadata.plant_id < 1:
        errors.append("metadata.plant_id must be one or greater")
    if state.metadata.simulation_time is not None:
        if isinstance(state.metadata.simulation_time, bool) or not isinstance(
            state.metadata.simulation_time, (int, float)
        ):
            errors.append("metadata.simulation_time must be numeric or null")
        elif not math.isfinite(float(state.metadata.simulation_time)):
            errors.append("metadata.simulation_time must be finite")
    if state.metadata.source_project_sha256 is not None:
        digest = state.metadata.source_project_sha256
        if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest.lower()):
            errors.append("metadata.source_project_sha256 must be a 64-character hexadecimal digest")
    node_ids = {node.id for node in state.nodes}
    for duplicate in _duplicates(node.id for node in state.nodes):
        errors.append(f"duplicate node id {duplicate}")
    if state.root_node_id not in node_ids:
        errors.append(f"missing root node {state.root_node_id}")

    structural_edges = [edge for edge in state.edges if edge.kind in STRUCTURAL_EDGE_KINDS]
    parents: dict[str, tuple[str, str, int]] = {}
    children: dict[str, list[str]] = {}
    for edge in state.edges:
        if edge.source not in node_ids or edge.target not in node_ids:
            errors.append(f"edge {edge.source}->{edge.target} references a missing node")
        if edge.kind not in STRUCTURAL_EDGE_KINDS:
            continue
        if edge.target in parents:
            errors.append(f"node {edge.target} has multiple structural parents")
        parents[edge.target] = (edge.source, edge.kind, edge.raw_code)
        children.setdefault(edge.source, []).append(edge.target)

    for node in state.nodes:
        if node.category not in {"plant_base", "organ", "turtle", "auxiliary"}:
            errors.append(f"node {node.id} has invalid category {node.category!r}")
        _validate_matrix(f"node {node.id} incoming_world", node.pose.incoming_world, errors)
        _validate_matrix(f"node {node.id} outgoing_world", node.pose.outgoing_world, errors)
        _validate_matrix(f"node {node.id} local_effect", node.pose.local_effect, errors)
        expected_local = _matrix_multiply(
            _rigid_inverse(node.pose.incoming_world), node.pose.outgoing_world
        )
        if not _matrix_close(expected_local, node.pose.local_effect):
            errors.append(f"node {node.id} local_effect is inconsistent with world frames")
        if not _vector_close(
            tuple(node.pose.incoming_world[row][3] for row in range(3)), node.pose.world_start
        ):
            errors.append(f"node {node.id} world_start is inconsistent")
        if not _vector_close(
            tuple(node.pose.outgoing_world[row][3] for row in range(3)), node.pose.world_end
        ):
            errors.append(f"node {node.id} world_end is inconsistent")
        parent = parents.get(node.id)
        if node.id == state.root_node_id:
            if parent is not None or node.parent_id is not None:
                errors.append("root node must not have a parent inside PlantState")
            if node.category != "plant_base":
                errors.append("root node must be a PlantBase")
        elif parent is None:
            errors.append(f"node {node.id} is disconnected from the root")
        else:
            if (node.parent_id, node.incoming_edge_kind, node.incoming_edge_raw_code) != parent:
                errors.append(f"node {node.id} parent metadata does not match its edge")

    active: set[str] = set()
    complete: set[str] = set()

    def visit(node_id: str) -> None:
        if node_id in active:
            errors.append(f"structural cycle reaches {node_id}")
            return
        if node_id in complete:
            return
        active.add(node_id)
        for child in children.get(node_id, []):
            visit(child)
        active.remove(node_id)
        complete.add(node_id)

    if state.root_node_id in node_ids:
        visit(state.root_node_id)
        for missing in sorted(node_ids - complete):
            errors.append(f"node {missing} is unreachable from the root")

    organ_ids = {organ.id for organ in state.organs}
    for duplicate in _duplicates(organ.id for organ in state.organs):
        errors.append(f"duplicate organ id {duplicate}")
    primitive_ids = {axis.id for axis in state.axes} | {sphere.id for sphere in state.spheres}
    primitive_owner = {
        primitive.id: primitive.owner_node_id for primitive in [*state.axes, *state.spheres]
    }
    for duplicate in _duplicates([axis.id for axis in state.axes] + [sphere.id for sphere in state.spheres]):
        errors.append(f"duplicate primitive id {duplicate}")
    organ_node_ids: set[str] = set()
    by_node = {node.id: node for node in state.nodes}
    property_types = {
        "PlantBase": PlantBaseProperties,
        "Root": RootProperties,
        "Internode": InternodeProperties,
        "Leaf": LeafProperties,
        "Truss": TrussProperties,
        "Fruits": FruitsProperties,
        "Meristem": MeristemProperties,
    }
    for organ in state.organs:
        if organ.node_id not in node_ids:
            errors.append(f"organ {organ.id} references missing node {organ.node_id}")
            continue
        organ_node_ids.add(organ.node_id)
        node = by_node[organ.node_id]
        if organ.organ_type not in SUPPORTED_ORGAN_TYPES:
            errors.append(f"organ {organ.id} has unsupported type {organ.organ_type}")
        elif not isinstance(organ.properties, property_types[organ.organ_type]):
            errors.append(f"organ {organ.id} has properties for another organ type")
        if node.source_type.rsplit(".", 1)[-1] != organ.organ_type:
            errors.append(f"organ {organ.id} type disagrees with its source node")
        if organ.common.plant_id != state.metadata.plant_id:
            errors.append(f"organ {organ.id} belongs to another plant")
        for primitive_id in organ.primitive_ids:
            if primitive_id not in primitive_ids:
                errors.append(f"organ {organ.id} references missing primitive {primitive_id}")
            elif primitive_owner[primitive_id] != organ.node_id:
                errors.append(f"organ {organ.id} references a primitive owned by another node")
        for name, value in (
            ("declared_length", organ.common.declared_length),
            ("area", organ.common.area),
            ("dry_biomass", organ.common.dry_biomass),
        ):
            if value is not None and value < 0.0:
                errors.append(f"organ {organ.id} has negative {name}")
        if isinstance(organ.properties, InternodeProperties):
            if organ.properties.diameter < 0.0 or organ.properties.effective_length < 0.0:
                errors.append(f"organ {organ.id} has negative internode dimensions")
        if isinstance(organ.properties, LeafProperties):
            dimensions = (
                organ.properties.petiole_length,
                organ.properties.petiole_diameter,
                organ.properties.petiolule_diameter,
                organ.properties.rachis_diameter,
                organ.properties.blade_area_total,
                *(organ.properties.rachis_segment_lengths or ()),
                *(organ.properties.petiolule_lengths or ()),
                *(organ.properties.blade_areas or ()),
            )
            if organ.properties.blade_count < 0 or any(value < 0.0 for value in dimensions):
                errors.append(f"organ {organ.id} has negative leaf dimensions")
            count = max(organ.properties.blade_count - 1, 0)
            for name, values in (
                ("petiolule_lengths", organ.properties.petiolule_lengths),
                ("petiolule_inclinations", organ.properties.petiolule_inclinations),
            ):
                if count and (values is None or len(values) < count):
                    errors.append(f"organ {organ.id} {name} is shorter than blade_count")
            segment_count = max(organ.properties.blade_count - 2, 0)
            values = organ.properties.rachis_segment_lengths
            if segment_count and (values is None or len(values) < segment_count):
                errors.append(f"organ {organ.id} rachis segments are shorter than blade_count")
        if isinstance(organ.properties, FruitsProperties):
            if (
                organ.properties.fruit_count < 0
                or organ.properties.pedicel_length < 0.0
                or organ.properties.rachis_segment_length < 0.0
                or organ.properties.rachis_radius < 0.0
                or any(value < 0.0 for value in organ.properties.fruit_radii or ())
            ):
                errors.append(f"organ {organ.id} has negative fruit dimensions")
        if isinstance(organ.properties, FruitsProperties) and organ.properties.fruit_count > 0:
            for name, values in (
                ("fruit_radii", organ.properties.fruit_radii),
                ("fruit_degree_days", organ.properties.fruit_degree_days),
            ):
                if values is None or len(values) < organ.properties.fruit_count:
                    errors.append(f"organ {organ.id} {name} is shorter than fruit_count")

    expected_organ_nodes = {
        node.id
        for node in state.nodes
        if node.source_type.rsplit(".", 1)[-1] in SUPPORTED_ORGAN_TYPES
    }
    for node_id in sorted(expected_organ_nodes - organ_node_ids):
        errors.append(f"supported organ node {node_id} has no organ record")
    for node_id in sorted(organ_node_ids - expected_organ_nodes):
        errors.append(f"organ record for non-organ node {node_id}")

    turtle_node_ids = {operation.node_id for operation in state.turtle_operations}
    for operation in state.turtle_operations:
        if operation.operation not in TURTLE_OPERATION_TYPES:
            errors.append(f"unsupported turtle operation {operation.operation}")
        if operation.node_id not in node_ids:
            errors.append(f"turtle operation {operation.id} references a missing node")
        _validate_matrix(f"turtle operation {operation.id}", operation.local_transform, errors)
        expected_parameters = (
            {"angle"}
            if operation.operation in {"RH", "RL", "RU"}
            else {"x", "y", "z"}
            if operation.operation == "Translate"
            else set()
        )
        if set(operation.parameters) != expected_parameters:
            errors.append(f"turtle operation {operation.id} has invalid parameters")
    expected_turtle_nodes = {
        node.id
        for node in state.nodes
        if node.source_type.rsplit(".", 1)[-1] in TURTLE_OPERATION_TYPES
    }
    for node_id in sorted(expected_turtle_nodes - turtle_node_ids):
        errors.append(f"turtle node {node_id} has no operation record")

    for axis in state.axes:
        _validate_axis(axis, node_ids, errors)
    for sphere in state.spheres:
        _validate_sphere(sphere, node_ids, errors)

    axes_by_owner: dict[str, list[AxisGeometry]] = {}
    spheres_by_owner: dict[str, list[SphereGeometry]] = {}
    for axis in state.axes:
        axes_by_owner.setdefault(axis.owner_node_id, []).append(axis)
    for sphere in state.spheres:
        spheres_by_owner.setdefault(sphere.owner_node_id, []).append(sphere)
    for organ in state.organs:
        if organ.organ_type == "Internode" and not any(
            axis.role == "internode" for axis in axes_by_owner.get(organ.node_id, [])
        ):
            errors.append(f"internode {organ.id} has no canonical axis")
        if organ.organ_type == "Leaf" and not axes_by_owner.get(organ.node_id):
            errors.append(f"leaf {organ.id} has no canonical supporting axes")
        if (
            organ.organ_type == "Fruits"
            and isinstance(organ.properties, FruitsProperties)
            and organ.properties.fruit_count > 0
            and not spheres_by_owner.get(organ.node_id)
        ):
            errors.append(f"fruit module {organ.id} has no canonical fruit spheres")

    # Ensure arbitrary source attributes and diagnostics cannot hide NaN/Inf.
    def walk(value: Any, path: str) -> None:
        if isinstance(value, float) and not math.isfinite(value):
            errors.append(f"{path} contains a non-finite value")
        elif isinstance(value, Mapping):
            for key, item in value.items():
                walk(item, f"{path}.{key}")
        elif isinstance(value, (list, tuple)):
            for index, item in enumerate(value):
                walk(item, f"{path}[{index}]")

    walk(asdict(state), "state")
    result = tuple(dict.fromkeys(errors))
    if result and strict:
        raise PlantStateValidationError(result)
    return result


def _equivalent(left: Any, right: Any, tolerance: float) -> bool:
    if isinstance(left, bool) or isinstance(right, bool):
        return left is right
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)
    if isinstance(left, Mapping) and isinstance(right, Mapping):
        return left.keys() == right.keys() and all(
            _equivalent(left[key], right[key], tolerance) for key in left
        )
    if isinstance(left, (list, tuple)) and isinstance(right, (list, tuple)):
        return len(left) == len(right) and all(
            _equivalent(a, b, tolerance) for a, b in zip(left, right)
        )
    return left == right


def plant_states_equivalent(left: PlantState, right: PlantState, *, atol: float = 1e-12) -> bool:
    """Compare all canonical fields with an absolute floating-point tolerance."""

    if atol < 0.0:
        raise ValueError("atol must be non-negative")
    return _equivalent(left.to_dict(), right.to_dict(), atol)
