"""Deterministic completeness audit for canonical V1 USD stages."""

from __future__ import annotations

from collections import Counter
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
from typing import Any

from pxr import Usd

from plant_state import PlantState

from .adapter import build_v1_render_view


V1_MANIFEST_SCHEMA_VERSION = "exporter_v1_manifest/1.0"


class V1AuditError(ValueError):
    """Raised when an exported stage loses canonical entities."""


@dataclass(frozen=True)
class V1ExportManifest:
    metadata: dict[str, Any]
    plant_state_organs: dict[str, int]
    usd_organ_prims: dict[str, int]
    expected_geometry: dict[str, int]
    created_geometry: dict[str, int]
    non_visual_by_design: dict[str, int]
    topology: dict[str, int]
    diagnostics: dict[str, Any] = field(default_factory=dict)
    errors: tuple[str, ...] = ()
    schema_version: str = V1_MANIFEST_SCHEMA_VERSION

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _attribute(prim, name: str):
    attribute = prim.GetAttribute(name)
    return attribute.Get() if attribute else None


def audit_v1_stage(state: PlantState, usd_path: str | Path) -> V1ExportManifest:
    """Compare canonical entities with tagged prims in a reopened USD stage."""

    path = Path(usd_path).expanduser().resolve()
    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise V1AuditError(f"cannot open exported USD stage: {path}")

    organ_counts = Counter(organ.organ_type for organ in state.organs)
    render_view = build_v1_render_view(state)
    rendered_views = [view for view in render_view.organs if view.render_geometry]
    usd_organs: Counter[str] = Counter()
    geometry: Counter[str] = Counter()
    topology_node_ids: set[str] = set()
    topology_parentage: dict[str, tuple[str | None, str | None]] = {}
    rendered_node_ids: set[str] = set()
    paths: list[str] = []
    for prim in stage.Traverse():
        paths.append(str(prim.GetPath()))
        entity_kind = _attribute(prim, "autotom:entityKind")
        if entity_kind == "topology_node":
            node_id = _attribute(prim, "autotom:nodeId")
            if node_id:
                topology_node_ids.add(str(node_id))
                parent = str(_attribute(prim, "autotom:parentId") or "") or None
                edge_kind = (
                    str(_attribute(prim, "autotom:incomingEdgeKind") or "") or None
                )
                topology_parentage[str(node_id)] = (parent, edge_kind)
        elif entity_kind == "organ":
            organ_type = _attribute(prim, "autotom:organType")
            node_id = _attribute(prim, "autotom:nodeId")
            if organ_type:
                usd_organs[str(organ_type)] += 1
            if node_id:
                rendered_node_ids.add(str(node_id))
        elif entity_kind == "geometry":
            role = _attribute(prim, "autotom:geometryRole")
            if role:
                geometry[str(role)] += 1

    expected_geometry = {
        "internode": sum(view.organ.organ_type == "Internode" for view in rendered_views),
        "leaf_group": sum(view.organ.organ_type == "Leaf" for view in rendered_views),
        "fruit_group": sum(view.organ.organ_type == "Fruits" for view in rendered_views),
        "fruit": sum(
            len(view.spheres)
            for view in rendered_views
            if view.organ.organ_type == "Fruits"
        ),
    }
    created_geometry = {
        "internode": geometry.get("internode", 0),
        "leaf_group": geometry.get("leaf_group", 0),
        "fruit_group": geometry.get("fruit_group", 0),
        "fruit": geometry.get("fruit", 0),
    }
    errors: list[str] = []
    if dict(sorted(usd_organs.items())) != dict(sorted(organ_counts.items())):
        errors.append(
            f"organ counts differ: PlantState={dict(sorted(organ_counts.items()))}, "
            f"USD={dict(sorted(usd_organs.items()))}"
        )
    expected_node_ids = {node.id for node in state.nodes}
    if topology_node_ids != expected_node_ids:
        errors.append(
            "topology node coverage differs: "
            f"missing={sorted(expected_node_ids - topology_node_ids)}, "
            f"extra={sorted(topology_node_ids - expected_node_ids)}"
        )
    expected_organ_node_ids = {organ.node_id for organ in state.organs}
    if rendered_node_ids != expected_organ_node_ids:
        errors.append(
            "organ node coverage differs: "
            f"missing={sorted(expected_organ_node_ids - rendered_node_ids)}, "
            f"extra={sorted(rendered_node_ids - expected_organ_node_ids)}"
        )
    expected_parentage = {
        node.id: (node.parent_id, node.incoming_edge_kind) for node in state.nodes
    }
    if topology_parentage != expected_parentage:
        differing = sorted(
            node_id
            for node_id in set(topology_parentage) | set(expected_parentage)
            if topology_parentage.get(node_id) != expected_parentage.get(node_id)
        )
        errors.append(f"topology parentage differs for nodes: {differing}")
    for key, expected in expected_geometry.items():
        if created_geometry[key] != expected:
            errors.append(
                f"geometry count {key} differs: expected={expected}, "
                f"created={created_geometry[key]}"
            )
    if len(paths) != len(set(paths)):
        errors.append("USD traversal contains duplicate paths")

    non_visual = {
        key: organ_counts.get(key, 0)
        for key in ("PlantBase", "Truss", "Meristem")
        if organ_counts.get(key, 0)
    }
    return V1ExportManifest(
        metadata={
            "status": "passed" if not errors else "failed",
            "usd_file": path.name,
            "plant_id": state.metadata.plant_id,
            "simulation_time": state.metadata.simulation_time,
            "plant_state_schema": state.schema_version,
        },
        plant_state_organs=dict(sorted(organ_counts.items())),
        usd_organ_prims=dict(sorted(usd_organs.items())),
        expected_geometry=expected_geometry,
        created_geometry=created_geometry,
        non_visual_by_design=non_visual,
        topology={
            "plant_state_nodes": len(state.nodes),
            "usd_topology_nodes": len(topology_node_ids),
            "plant_state_edges": len(state.edges),
            "usd_parent_links": sum(parent is not None for parent, _ in topology_parentage.values()),
        },
        diagnostics={
            **render_view.diagnostics,
            "path_count": len(paths),
        },
        errors=tuple(errors),
    )


def manifest_path_for(usd_path: str | Path) -> Path:
    path = Path(usd_path).expanduser().resolve()
    return path.with_suffix(".manifest.json")


def save_v1_manifest(manifest: V1ExportManifest, path: str | Path) -> Path:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(manifest.to_dict(), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return destination
