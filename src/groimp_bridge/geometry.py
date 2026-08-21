"""Rendered-organ reconstruction and mesh validation for GroIMP scenes."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping

import numpy as np

from .models import GraphNode, GroIMPGraphSnapshot
from .turtle import TurtleFrame, TurtleResolution


GEOMETRY_REPORT_SCHEMA_VERSION = "groimp_geometry_validation/1.0"
Vector3 = tuple[float, float, float]


class GeometryReconstructionError(ValueError):
    """Raised when a supported organ has inconsistent rendering attributes."""


@dataclass(frozen=True)
class GeometryTolerance:
    anchor_position: float = 1e-6
    anchor_direction: float = 1e-5
    absolute_mesh: float = 1e-5
    relative_mesh: float = 1e-3

    def endpoint(self, length: float) -> float:
        return max(self.absolute_mesh, self.relative_mesh * abs(float(length)))

    def dimension(self, value: float) -> float:
        return max(self.absolute_mesh, self.relative_mesh * abs(float(value)))


@dataclass(frozen=True)
class AxisPrimitive:
    primitive_id: str
    source_node_id: int
    organ_type: str
    role: str
    start: Vector3
    end: Vector3
    direction: Vector3
    length: float
    radius: float
    frame: TurtleFrame


@dataclass(frozen=True)
class SpherePrimitive:
    primitive_id: str
    source_node_id: int
    organ_type: str
    role: str
    center: Vector3
    radius: float
    frame: TurtleFrame


@dataclass(frozen=True)
class GeometryConnection:
    source_node_id: int
    target_node_id: int
    kind: str
    start: Vector3
    end: Vector3


@dataclass
class ReconstructedGeometry:
    axes: list[AxisPrimitive]
    spheres: list[SpherePrimitive]
    diagnostics: dict[str, Any] = field(default_factory=dict)
    connections: list[GeometryConnection] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ObjMesh:
    """Minimal OBJ representation in GroIMP world-axis convention."""

    vertices: tuple[Vector3, ...]
    faces: tuple[tuple[int, ...], ...]
    groimp_generated: bool = False

    def connected_components(self) -> tuple[np.ndarray, ...]:
        """Return face-connected vertex components in deterministic order."""

        if not self.vertices:
            return ()
        parent = list(range(len(self.vertices)))

        def find(value: int) -> int:
            while parent[value] != value:
                parent[value] = parent[parent[value]]
                value = parent[value]
            return value

        def union(left: int, right: int) -> None:
            left_root, right_root = find(left), find(right)
            if left_root != right_root:
                parent[max(left_root, right_root)] = min(left_root, right_root)

        used: set[int] = set()
        for face in self.faces:
            if not face:
                continue
            used.update(face)
            for vertex in face[1:]:
                union(face[0], vertex)
        groups: dict[int, list[int]] = {}
        for index in sorted(used or range(len(self.vertices))):
            groups.setdefault(find(index), []).append(index)
        vertices = np.asarray(self.vertices, dtype=np.float64)
        return tuple(vertices[indexes] for _, indexes in sorted(groups.items()))


@dataclass(frozen=True)
class PrimitiveValidation:
    primitive_id: str
    source_node_id: int
    role: str
    primitive_kind: str
    status: str
    errors: dict[str, float]
    measured: dict[str, Any]
    diagnostic: str | None = None


@dataclass
class GeometryValidationReport:
    checks: list[PrimitiveValidation]
    summary: dict[str, int]
    tolerances: dict[str, float]
    diagnostics: dict[str, Any] = field(default_factory=dict)
    report_schema_version: str = GEOMETRY_REPORT_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _simple_type(node: GraphNode) -> str:
    return node.type.rsplit(".", 1)[-1]


def _number(node: GraphNode, name: str) -> float:
    try:
        value = float(node.attributes[name])
    except (KeyError, TypeError, ValueError) as exc:
        raise GeometryReconstructionError(
            f"Node {node.id} ({node.type}) requires numeric {name!r}"
        ) from exc
    if not math.isfinite(value):
        raise GeometryReconstructionError(
            f"Node {node.id} ({node.type}) has non-finite {name!r}"
        )
    return value


def _array(node: GraphNode, name: str) -> list[float]:
    raw = node.attributes.get(name)
    if raw is None:
        return []
    if not isinstance(raw, (list, tuple)):
        raise GeometryReconstructionError(
            f"Node {node.id} ({node.type}) requires array {name!r}"
        )
    return [float(value) for value in raw]


def _axis(
    node: GraphNode,
    role: str,
    index: int,
    frame: TurtleFrame,
    length: float,
    radius: float,
) -> tuple[AxisPrimitive, TurtleFrame]:
    if length < 0 or radius < 0:
        raise GeometryReconstructionError(
            f"Node {node.id} has negative geometry for {role}: {length=}, {radius=}"
        )
    outgoing = frame.translate_local(0.0, 0.0, length)
    direction = tuple(float(value) for value in frame.head)
    primitive = AxisPrimitive(
        primitive_id=f"node-{node.id}:{role}:{index}",
        source_node_id=node.id,
        organ_type=_simple_type(node),
        role=role,
        start=frame.position,
        end=outgoing.position,
        direction=direction,  # type: ignore[arg-type]
        length=float(length),
        radius=float(radius),
        frame=frame,
    )
    return primitive, outgoing


def _leaf_geometry(node: GraphNode, initial: TurtleFrame) -> list[AxisPrimitive]:
    axes: list[AxisPrimitive] = []
    ccw = _number(node, "counterClocKWiseOrientationPetiole")
    frame = initial.rotate_local("head", ccw).rotate_local(
        "left", _number(node, "anglePetiole")
    )
    petiole, frame = _axis(
        node,
        "petiole",
        0,
        frame,
        _number(node, "lengthPetiole"),
        _number(node, "diameterPetiole") / 2.0,
    )
    axes.append(petiole)

    blades = int(_number(node, "bladesNr"))
    if blades <= 1:
        return axes
    lengths = _array(node, "lengthPetiolules")
    inclinations = _array(node, "inclinationOnSegmentsPetiolules")
    segments = _array(node, "segmentsLength")
    frame = frame.rotate_local("head", -ccw)

    for q in range(blades - 1):
        if q >= len(lengths) or q >= len(inclinations):
            raise GeometryReconstructionError(
                f"Leaf node {node.id} arrays are shorter than bladesNr={blades}"
            )
        frame = frame.rotate_local("head", ccw)
        left_frame = (
            frame.rotate_local("up", 90.0)
            .rotate_local("left", 90.0 + inclinations[q])
            .rotate_local("head", 180.0)
        )
        left, _ = _axis(
            node,
            "petiolule_left",
            q,
            left_frame,
            lengths[q],
            _number(node, "diameterPetiolule") / 2.0,
        )
        right_frame = frame.rotate_local("up", 90.0).rotate_local(
            "left", 90.0 - inclinations[q]
        )
        right, _ = _axis(
            node,
            "petiolule_right",
            q,
            right_frame,
            lengths[q],
            _number(node, "diameterPetiolule") / 2.0,
        )
        axes.extend((left, right))

        terminal = q == blades - 2
        if terminal:
            curvature = 90.0 - inclinations[q]
            segment_length = lengths[q]
            radius = _number(node, "diameterPetiolule") / 2.0
            role = "rachis_terminal"
        else:
            if q >= len(segments):
                raise GeometryReconstructionError(
                    f"Leaf node {node.id} segmentsLength is missing index {q}"
                )
            curvature = _number(node, "leafCurvature") - 90.0
            segment_length = segments[q]
            radius = _number(node, "diameterSegment") / 2.0
            role = "leaf_rachis"
        frame = frame.rotate_local("left", curvature)
        rachis, frame = _axis(node, role, q, frame, segment_length, radius)
        axes.append(rachis)
        frame = frame.rotate_local("head", -ccw)
    return axes


def _fruit_geometry(
    node: GraphNode, initial: TurtleFrame
) -> tuple[list[AxisPrimitive], list[SpherePrimitive]]:
    axes: list[AxisPrimitive] = []
    spheres: list[SpherePrimitive] = []
    fruit_count = int(_number(node, "fruitNr"))
    if fruit_count <= 0:
        return axes, spheres
    ages = _array(node, "degreeDaysStorage")
    radii = _array(node, "fruitRadius")
    if len(ages) < fruit_count or len(radii) < fruit_count:
        raise GeometryReconstructionError(
            f"Fruits node {node.id} arrays are shorter than fruitNr={fruit_count}"
        )
    paired = bool(node.attributes.get("fruitPairing", False))
    rachis_length = _number(node, "INTERNODETRUSSLENGTH")
    pedicel_length = _number(node, "PETIOLELENGTH")
    rachis_radius = _number(node, "internodeTrussdiameter")
    bend = _number(node, "internodeTrussAngle")
    fruit_angle = _number(node, "angleAmongSubsequentFruits")
    frame = initial.rotate_local("left", 45.0)

    rachis, frame = _axis(node, "truss_rachis", 0, frame, rachis_length, rachis_radius)
    axes.append(rachis)

    def add_fruit(index: int, branch_frame: TurtleFrame) -> None:
        pedicel, pedicel_end = _axis(
            node, "pedicel", index, branch_frame, pedicel_length, rachis_radius
        )
        axes.append(pedicel)
        radius = radii[index]
        center_frame = pedicel_end.translate_local(0.0, 0.0, radius)
        spheres.append(
            SpherePrimitive(
                primitive_id=f"node-{node.id}:fruit:{index}",
                source_node_id=node.id,
                organ_type=_simple_type(node),
                role="fruit",
                center=center_frame.position,
                radius=radius,
                frame=center_frame,
            )
        )

    add_fruit(
        0,
        frame.rotate_local("up", -90.0).rotate_local("left", fruit_angle),
    )
    for index in range(1, max(fruit_count - 1, 1)):
        if ages[index] == 0:
            continue
        if not paired or index % 2 == 0:
            frame = frame.rotate_local("left", bend)
            rachis, frame = _axis(
                node, "truss_rachis", index, frame, rachis_length, rachis_radius
            )
            axes.append(rachis)
        side = -90.0 if index % 2 == 0 else 90.0
        add_fruit(
            index,
            frame.rotate_local("up", side).rotate_local("left", fruit_angle),
        )
    terminal = fruit_count - 1
    if terminal > 0 and ages[terminal] != 0:
        frame = frame.rotate_local("left", bend)
        add_fruit(terminal, frame)
    return axes, spheres


def build_rendered_geometry(
    snapshot: GroIMPGraphSnapshot,
    turtle_resolution: TurtleResolution,
    *,
    strict: bool = False,
) -> ReconstructedGeometry:
    """Reconstruct axis-like primitives emitted by the tomato RGG productions."""

    axes: list[AxisPrimitive] = []
    spheres: list[SpherePrimitive] = []
    skipped: list[dict[str, Any]] = []
    for node in sorted(snapshot.nodes, key=lambda item: item.id):
        node_type = _simple_type(node)
        if node_type not in {"Internode", "Leaf", "Fruits"}:
            continue
        pose = turtle_resolution.poses.get(node.id)
        if pose is None:
            skipped.append({"node_id": node.id, "reason": "unresolved_turtle_pose"})
            continue
        try:
            if node_type == "Internode":
                length = float(
                    np.linalg.norm(
                        np.asarray(pose.end_position) - np.asarray(pose.start_position)
                    )
                )
                primitive, _ = _axis(
                    node,
                    "internode",
                    0,
                    pose.incoming_frame,
                    length,
                    _number(node, "internode_width_m") / 2.0,
                )
                axes.append(primitive)
            elif node_type == "Leaf":
                axes.extend(_leaf_geometry(node, pose.incoming_frame))
            else:
                fruit_axes, fruit_spheres = _fruit_geometry(node, pose.incoming_frame)
                axes.extend(fruit_axes)
                spheres.extend(fruit_spheres)
        except GeometryReconstructionError as exc:
            if strict:
                raise
            skipped.append({"node_id": node.id, "reason": str(exc)})
    axes.sort(key=lambda item: item.primitive_id)
    spheres.sort(key=lambda item: item.primitive_id)
    connections = [
        GeometryConnection(
            source_node_id=edge.source,
            target_node_id=edge.target,
            kind=edge.kind,
            start=turtle_resolution.poses[edge.source].outgoing_frame.position,
            end=turtle_resolution.poses[edge.target].incoming_frame.position,
        )
        for edge in sorted(
            snapshot.edges, key=lambda item: (item.source, item.target, item.raw_code)
        )
        if edge.kind in {"successor", "branch"}
        and edge.source in turtle_resolution.poses
        and edge.target in turtle_resolution.poses
    ]
    return ReconstructedGeometry(
        axes=axes,
        spheres=spheres,
        diagnostics={
            "skipped_nodes": skipped,
            "axis_count": len(axes),
            "sphere_count": len(spheres),
            "covered_roles": sorted({item.role for item in [*axes, *spheres]}),
        },
        connections=connections,
    )


def parse_obj(payload: bytes | str) -> ObjMesh:
    """Parse OBJ and convert ``(x,y,z)`` into GroIMP ``(x,z,y)`` axes."""

    text = payload.decode("utf-8", errors="replace") if isinstance(payload, bytes) else payload
    vertices: list[Vector3] = []
    faces: list[tuple[int, ...]] = []
    for raw_line in text.splitlines():
        line = raw_line.strip()
        parts = line.split()
        if parts and parts[0] == "v":
            if len(parts) >= 4:
                x, y, z = (float(value) for value in parts[1:4])
                vertices.append((x, z, y))
        elif parts and parts[0] == "f":
            indexes: list[int] = []
            for item in parts[1:]:
                raw_index = int(item.split("/", 1)[0])
                indexes.append(raw_index - 1 if raw_index > 0 else len(vertices) + raw_index)
            if len(indexes) >= 3:
                faces.append(tuple(indexes))
    return ObjMesh(
        vertices=tuple(vertices),
        faces=tuple(faces),
        groimp_generated="OBJ File Generated by GroIMP" in text,
    )


def _axis_measurement(
    primitive: AxisPrimitive,
    vertices: np.ndarray,
    tolerance: GeometryTolerance,
) -> tuple[str, dict[str, float], dict[str, Any], str | None]:
    start = np.asarray(primitive.start)
    direction = np.asarray(primitive.direction)
    length = primitive.length
    endpoint_tolerance = tolerance.endpoint(length)
    radius_tolerance = tolerance.dimension(primitive.radius)
    if len(vertices) < 6:
        return "not_recoverable", {}, {}, "OBJ has too few vertices"
    relative = vertices - start
    axial_all = relative @ direction
    radial_vectors = relative - axial_all[:, None] * direction
    radial_all = np.linalg.norm(radial_vectors, axis=1)
    shell_mask = (
        (axial_all >= -endpoint_tolerance)
        & (axial_all <= length + endpoint_tolerance)
        & (np.abs(radial_all - primitive.radius) <= radius_tolerance)
    )
    axial = axial_all[shell_mask]
    radial = radial_all[shell_mask]
    if len(axial) < 6:
        return "not_recoverable", {}, {}, "no cylindrical vertex shell near prediction"
    minimum, maximum = float(np.min(axial)), float(np.max(axial))
    measured_radius = float(np.median(radial))
    errors = {
        "start": abs(minimum),
        "end": abs(maximum - length),
        "length": abs((maximum - minimum) - length),
        "radius": abs(measured_radius - primitive.radius),
    }
    measured = {
        "matched_vertex_count": int(len(axial)),
        "start": tuple(float(v) for v in start + minimum * direction),
        "end": tuple(float(v) for v in start + maximum * direction),
        "direction": primitive.direction,
        "length": maximum - minimum,
        "radius": measured_radius,
    }
    passed = (
        errors["start"] <= endpoint_tolerance
        and errors["end"] <= endpoint_tolerance
        and errors["length"] <= endpoint_tolerance * 2.0
        and errors["radius"] <= radius_tolerance
    )
    if passed:
        return "passed", errors, measured, None
    if measured["length"] < primitive.length * 0.5:
        return (
            "ambiguous",
            errors,
            measured,
            "a nearby overlapping shell was found, but the target axis is not uniquely recoverable",
        )
    return "failed", errors, measured, None


def _sphere_measurement(
    primitive: SpherePrimitive,
    vertices: np.ndarray,
    tolerance: GeometryTolerance,
) -> tuple[str, dict[str, float], dict[str, Any], str | None]:
    expected = np.asarray(primitive.center)
    dimension_tolerance = tolerance.dimension(primitive.radius)
    if len(vertices) < 8:
        return "not_recoverable", {}, {}, "OBJ has too few vertices"
    distances_from_expected = np.linalg.norm(vertices - expected, axis=1)
    mask = np.abs(distances_from_expected - primitive.radius) <= dimension_tolerance
    selected = vertices[mask]
    if len(selected) < 8:
        return "not_recoverable", {}, {}, "no spherical vertex shell near prediction"
    lower, upper = np.min(selected, axis=0), np.max(selected, axis=0)
    center = (lower + upper) / 2.0
    distances = np.linalg.norm(selected - center, axis=1)
    radius = float(np.median(distances))
    center_error = float(np.linalg.norm(center - expected))
    radius_error = abs(radius - primitive.radius)
    errors = {"center": center_error, "radius": radius_error}
    measured = {
        "matched_vertex_count": int(len(selected)),
        "center": tuple(float(value) for value in center),
        "radius": radius,
    }
    passed = errors["center"] <= dimension_tolerance and errors["radius"] <= dimension_tolerance
    return ("passed" if passed else "failed", errors, measured, None)


def validate_rendered_geometry(
    geometry: ReconstructedGeometry,
    meshes: Mapping[int, ObjMesh],
    *,
    tolerance: GeometryTolerance | None = None,
) -> GeometryValidationReport:
    """Match reconstructed primitives to face-connected OBJ components."""

    tolerance = tolerance or GeometryTolerance()
    vertices = {
        node_id: np.asarray(mesh.vertices, dtype=np.float64)
        for node_id, mesh in sorted(meshes.items())
    }
    # ProjectGraph anchors and the rendered scene can be one growth update out
    # of sync. GroIMP's OBJ writer emits the first axis cap-center as the first
    # vertex for Internode, Leaf (petiole), and Fruits (truss rachis) subscenes.
    # Align only the per-organ translation; directions and dimensions remain
    # independently testable and the offset is retained as a diagnostic.
    anchor_role_order = ("internode", "petiole", "truss_rachis")
    offsets: dict[int, np.ndarray] = {}
    anchor_checks: list[dict[str, Any]] = []
    for node_id, node_vertices in vertices.items():
        if not meshes[node_id].groimp_generated:
            continue
        if len(node_vertices) == 0:
            continue
        candidates = [item for item in geometry.axes if item.source_node_id == node_id]
        anchor_axis = next(
            (
                item
                for role in anchor_role_order
                for item in candidates
                if item.role == role
            ),
            None,
        )
        if anchor_axis is None:
            continue
        offset = node_vertices[0] - np.asarray(anchor_axis.start)
        offsets[node_id] = offset
        error = float(np.linalg.norm(offset))
        anchor_checks.append(
            {
                "node_id": node_id,
                "role": anchor_axis.role,
                "offset": tuple(float(value) for value in offset),
                "error": error,
                "tolerance": tolerance.anchor_position,
                "status": "passed" if error <= tolerance.anchor_position else "renderer_cache_offset",
            }
        )

    def shifted_axis(item: AxisPrimitive) -> AxisPrimitive:
        offset = offsets.get(item.source_node_id)
        if offset is None:
            return item
        return replace(
            item,
            start=tuple(float(value) for value in np.asarray(item.start) + offset),
            end=tuple(float(value) for value in np.asarray(item.end) + offset),
        )

    def shifted_sphere(item: SpherePrimitive) -> SpherePrimitive:
        offset = offsets.get(item.source_node_id)
        if offset is None:
            return item
        return replace(
            item,
            center=tuple(float(value) for value in np.asarray(item.center) + offset),
        )
    checks: list[PrimitiveValidation] = []
    for original in geometry.axes:
        primitive = shifted_axis(original)
        if primitive.source_node_id not in vertices:
            result = ("not_recoverable", {}, {}, "subscene was not exported")
        else:
            result = _axis_measurement(
                primitive, vertices[primitive.source_node_id], tolerance
            )
        checks.append(
            PrimitiveValidation(
                primitive_id=primitive.primitive_id,
                source_node_id=primitive.source_node_id,
                role=primitive.role,
                primitive_kind="axis",
                status=result[0],
                errors=result[1],
                measured=result[2],
                diagnostic=result[3],
            )
        )
    for original in geometry.spheres:
        primitive = shifted_sphere(original)
        if primitive.source_node_id not in vertices:
            result = ("not_recoverable", {}, {}, "subscene was not exported")
        else:
            result = _sphere_measurement(
                primitive, vertices[primitive.source_node_id], tolerance
            )
        checks.append(
            PrimitiveValidation(
                primitive_id=primitive.primitive_id,
                source_node_id=primitive.source_node_id,
                role=primitive.role,
                primitive_kind="sphere",
                status=result[0],
                errors=result[1],
                measured=result[2],
                diagnostic=result[3],
            )
        )
    checks.sort(key=lambda item: item.primitive_id)
    statuses = {status: sum(item.status == status for item in checks) for status in (
        "passed", "ambiguous", "failed", "not_recoverable"
    )}
    return GeometryValidationReport(
        checks=checks,
        summary=statuses,
        tolerances=asdict(tolerance),
        diagnostics={
            "mesh_node_ids": sorted(meshes),
            "obj_axis_mapping": "(x,y,z)_obj -> (x,z,y)_groimp",
            "anchor_checks": anchor_checks,
            "anchor_calibration": "translation only; OBJ first axis cap-center",
            "reconstruction": geometry.diagnostics,
        },
    )


def save_geometry_report(
    report: GeometryValidationReport, output_path: str | Path
) -> Path:
    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(report.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination


def save_debug_obj(geometry: ReconstructedGeometry, output_path: str | Path) -> Path:
    """Write deterministic line primitives for starts, endpoints and local axes."""

    destination = Path(output_path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    lines = ["# AutoTom GroIMP reconstructed geometry debug overlay"]
    vertex_index = 1
    for primitive in geometry.axes:
        lines.append(f"g {primitive.primitive_id.replace(':', '_')}")
        points = [primitive.start, primitive.end]
        axis_scale = max(primitive.length * 0.2, primitive.radius * 2.0, 1e-4)
        origin = np.asarray(primitive.start)
        points.extend(
            tuple(float(value) for value in origin + axis_scale * np.asarray(axis))
            for axis in (primitive.frame.left, primitive.frame.up, primitive.frame.head)
        )
        for point in points:
            lines.append("v " + " ".join(f"{value:.12g}" for value in point))
        lines.extend(
            (
                f"l {vertex_index} {vertex_index + 1}",
                f"l {vertex_index} {vertex_index + 2}",
                f"l {vertex_index} {vertex_index + 3}",
                f"l {vertex_index} {vertex_index + 4}",
            )
        )
        vertex_index += 5
    for connection in geometry.connections:
        lines.append(
            f"g connection_{connection.kind}_{connection.source_node_id}_{connection.target_node_id}"
        )
        for point in (connection.start, connection.end):
            lines.append("v " + " ".join(f"{value:.12g}" for value in point))
        lines.append(f"l {vertex_index} {vertex_index + 1}")
        vertex_index += 2
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return destination
