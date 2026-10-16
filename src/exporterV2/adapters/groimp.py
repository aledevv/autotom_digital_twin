"""GroIMP adapter: ``plant_state/1.0`` JSON -> ``plant_skeleton/1.0``.

This is the reference example for writing a new adapter. Everything that is
specific to GroIMP lives here, so the exporter never sees it:

* only order-0 internodes form the stem; higher orders become side branches;
* GroIMP sometimes emits exact duplicate organs, which are dropped;
* organs with zero-size geometry (young leaves, empty trusses) are skipped;
* pedicels are drawn 3x longer than in GroIMP so large tomatoes do not crowd
  the truss rachis (``PEDICEL_LENGTH_SCALE``);
* leaf dry biomass (mg) becomes leaflet mass (kg).

Run it from the command line to refresh the exporter input:

    uv run python -m exporterV2.adapters.groimp --day 160
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from pathlib import Path
from typing import Any, Iterable

from plant_skeleton import (
    PlantSkeleton,
    SkeletonFruit,
    SkeletonLeaflet,
    SkeletonMetadata,
    SkeletonNode,
    save_skeleton,
    validate_skeleton,
)
from plant_skeleton.geometry import closest_fraction, norm, normalized, sub
from plant_state import AxisGeometry, PlantState, load_plant_state, validate_plant_state


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PEDICEL_LENGTH_SCALE = 3.0
_MIN_SIZE = 1e-9
_SIDE = {"petiolule_left": "left", "petiolule_right": "right", "rachis_terminal": "terminal"}


class GroIMPAdapterError(ValueError):
    pass


def _index(axis: AxisGeometry) -> int:
    return int(axis.id.rsplit(":", 1)[1])


def _positive(axis: AxisGeometry) -> bool:
    return axis.length > _MIN_SIZE and axis.radius > _MIN_SIZE


def _rotation(axis_or_sphere) -> tuple:
    return tuple(tuple(float(value) for value in row[:3]) for row in axis_or_sphere.world_frame[:3])


class _Builder:
    """Collects skeleton items with positions relative to the plant base."""

    def __init__(self, state: PlantState):
        self.state = state
        self.nodes_by_id = {node.id: node for node in state.nodes}
        self.organs_by_node = {organ.node_id: organ for organ in state.organs}
        root = self.nodes_by_id[state.root_node_id]
        self.origin = tuple(float(value) for value in root.pose.world_start)
        self.axes_by_owner: dict[str, list[AxisGeometry]] = {}
        for axis in state.axes:
            self.axes_by_owner.setdefault(axis.owner_node_id, []).append(axis)
        self.spheres_by_owner: dict[str, list] = {}
        for sphere in state.spheres:
            self.spheres_by_owner.setdefault(sphere.owner_node_id, []).append(sphere)
        self.nodes: list[SkeletonNode] = []
        self.leaflets: list[SkeletonLeaflet] = []
        self.fruits: list[SkeletonFruit] = []
        # GroIMP axis id -> skeleton node id, for every exported segment.
        self.node_for_axis: dict[str, str] = {}
        # GroIMP internode node id -> skeleton node id (duplicates included).
        self.node_for_internode: dict[str, str] = {}

    def gid(self, node_id: str) -> int:
        return int(self.nodes_by_id[node_id].groimp_node_id)

    def local(self, point: Iterable[float]) -> tuple[float, float, float]:
        return sub(tuple(point), self.origin)

    def add_node(self, node_id, parent, kind, axis: AxisGeometry, organ_id, *, chain=None, length_scale=1.0):
        start = self.local(axis.world_start)
        end = self.local(axis.world_end)
        if length_scale != 1.0:
            direction = normalized(sub(end, start))
            length = float(axis.length) * length_scale
            end = tuple(start[i] + direction[i] * length for i in range(3))
        self.nodes.append(
            SkeletonNode(
                id=node_id,
                parent=parent,
                kind=kind,
                start=start,
                end=end,
                radius=float(axis.radius),
                frame=_rotation(axis),
                axis=chain,
                organ=organ_id,
            )
        )
        self.node_for_axis[axis.id] = node_id
        return node_id


def _dedupe(items: list[tuple], key) -> list[tuple]:
    """Keep the lowest GroIMP id of each group of exact duplicates, in order."""

    groups: dict[Any, list[tuple]] = {}
    for item in items:
        groups.setdefault(key(item), []).append(item)
    kept = {id(min(group, key=lambda item: item[-1])) for group in groups.values()}
    duplicates = {}
    for group in groups.values():
        primary = min(group, key=lambda item: item[-1])
        for item in group:
            duplicates[id(item)] = primary
    return [item for item in items if id(item) in kept], duplicates


def _internodes(builder: _Builder):
    result = {}
    for axis in builder.state.axes:
        if axis.role != "internode":
            continue
        organ = builder.organs_by_node.get(axis.owner_node_id)
        if organ is None or organ.organ_type != "Internode":
            raise GroIMPAdapterError(f"axis {axis.id} has no Internode organ")
        result[axis.owner_node_id] = (organ, axis, builder.gid(axis.owner_node_id))
    return result


def _internode_key(builder: _Builder):
    def key(item):
        organ, axis, _gid = item
        return (
            organ.organ_type,
            builder.nodes_by_id[organ.node_id].parent_id,
            repr(asdict(organ.common)),
            repr(asdict(organ.properties)),
            tuple(axis.world_start),
            tuple(axis.world_end),
            tuple(tuple(row) for row in axis.world_frame),
            axis.length,
            axis.radius,
        )

    return key


def _add_structure(builder: _Builder) -> None:
    """Stem chain plus side branches, following GroIMP branch orders."""

    internodes = _internodes(builder)
    stem = [item for item in internodes.values() if int(item[0].common.order or 0) == 0]
    if not stem:
        raise GroIMPAdapterError("PlantState has no order-0 internodes")
    stem.sort(key=lambda item: (item[0].common.rank is None, item[0].common.rank or 0, item[2], item[1].id))
    stem, stem_duplicates = _dedupe(stem, _internode_key(builder))
    parent = None
    for organ, axis, gid in stem:
        parent = builder.add_node(f"g{gid}", parent, "stem", axis, organ.id)
    for item in internodes.values():
        if id(item) in stem_duplicates:
            builder.node_for_internode[item[0].node_id] = f"g{stem_duplicates[id(item)][2]}"

    lateral = [item for item in internodes.values() if int(item[0].common.order or 0) > 0]
    lateral, lateral_duplicates = _dedupe(lateral, _internode_key(builder))
    for item in internodes.values():
        if id(item) in lateral_duplicates:
            builder.node_for_internode[item[0].node_id] = f"g{lateral_duplicates[id(item)][2]}"
    kept = {item[0].node_id: item for item in lateral}

    def parent_internode(node_id: str) -> str:
        current = builder.nodes_by_id[node_id].parent_id
        while current is not None:
            if current in internodes:
                return current
            current = builder.nodes_by_id[current].parent_id
        raise GroIMPAdapterError(f"internode {node_id} has no internode ancestor")

    parent_of = {node_id: parent_internode(node_id) for node_id in kept}
    order = {node_id: int(item[0].common.order or 0) for node_id, item in internodes.items()}
    same_order_children: dict[str, list[str]] = {}
    for node_id, parent_id in parent_of.items():
        if order[node_id] == order[parent_id]:
            same_order_children.setdefault(parent_id, []).append(node_id)
    for children in same_order_children.values():
        children.sort(key=builder.gid)
    starts = [
        node_id
        for node_id, parent_id in parent_of.items()
        if order[node_id] != order[parent_id] or len(same_order_children.get(parent_id, ())) != 1
    ]
    starts.sort(key=lambda node_id: (order[node_id], builder.gid(node_id)))
    chains = []
    for start in starts:
        chain, current = [], start
        while True:
            chain.append(current)
            children = same_order_children.get(current, ())
            if len(children) != 1:
                break
            current = children[0]
        chains.append(chain)

    # Emit chains only once their parent segment exists.
    pending = list(chains)
    while pending:
        progressed = False
        for chain in list(pending):
            parent_node = builder.node_for_internode.get(parent_of[chain[0]])
            if parent_node is None:
                continue
            chain_id = f"branch_g{builder.gid(chain[0])}"
            previous = parent_node
            for node_id in chain:
                organ, axis, gid = kept[node_id]
                previous = builder.add_node(f"g{gid}", previous, "branch", axis, organ.id, chain=chain_id)
                builder.node_for_internode[node_id] = previous
            pending.remove(chain)
            progressed = True
        if not progressed:
            raise GroIMPAdapterError(f"cannot attach side branches {[chain[0] for chain in pending]}")


def _nearest_internode(builder: _Builder, node_id: str) -> str:
    current = builder.nodes_by_id[node_id].parent_id
    while current is not None:
        organ = builder.organs_by_node.get(current)
        if organ is not None and organ.organ_type == "Internode":
            return builder.node_for_internode[current]
        current = builder.nodes_by_id[current].parent_id
    raise GroIMPAdapterError(f"{node_id} has no internode ancestor")


def _add_leaves(builder: _Builder) -> None:
    leaves = []
    for organ in builder.state.organs:
        if organ.organ_type != "Leaf":
            continue
        axes = builder.axes_by_owner.get(organ.node_id, [])
        petioles = [axis for axis in axes if axis.role == "petiole"]
        if len(petioles) != 1:
            raise GroIMPAdapterError(f"leaf {organ.node_id} must have one petiole axis")
        rachides = sorted((axis for axis in axes if axis.role == "leaf_rachis"), key=_index)
        leaves.append((organ, petioles[0], rachides, builder.gid(organ.node_id)))
    leaves.sort(key=lambda item: (int(item[0].common.order or 0), item[0].common.rank is None, item[0].common.rank or 0, item[3]))

    def key(item):
        organ, petiole, rachides, _gid = item
        node = builder.nodes_by_id[organ.node_id]
        geometry = tuple(
            (axis.role, axis.id.rsplit(":", 1)[-1], tuple(axis.world_start), tuple(axis.world_end),
             tuple(tuple(row) for row in axis.world_frame), axis.length, axis.radius)
            for axis in sorted([petiole, *rachides], key=lambda value: value.id)
        )
        return (organ.organ_type, node.parent_id, repr(asdict(organ.common)), repr(asdict(organ.properties)),
                tuple(tuple(row) for row in node.pose.incoming_world),
                tuple(tuple(row) for row in node.pose.outgoing_world), geometry)

    leaves, _duplicates = _dedupe(leaves, key)
    for organ, petiole, rachides, gid in leaves:
        if not _positive(petiole):
            continue  # young leaf without geometry yet
        if any(not _positive(axis) for axis in rachides):
            raise GroIMPAdapterError(f"leaf {organ.node_id} has partially degenerate rachis axes")
        previous = builder.add_node(f"g{gid}_petiole", _nearest_internode(builder, organ.node_id), "petiole", petiole, organ.id)
        for segment, axis in enumerate(rachides, start=1):
            previous = builder.add_node(f"g{gid}_rachis{segment:02d}", previous, "leaf_rachis", axis, organ.id)
        _add_leaflets(builder, organ, gid, [petiole, *rachides])


def _add_leaflets(builder: _Builder, organ, gid: int, supports: list[AxisGeometry]) -> None:
    axes = builder.axes_by_owner.get(organ.node_id, [])
    visuals = sorted(
        (axis for axis in axes if axis.role in _SIDE),
        key=lambda axis: ({"petiolule_left": 0, "petiolule_right": 1, "rachis_terminal": 2}[axis.role], _index(axis)),
    )
    positive = [axis for axis in visuals if _positive(axis)]
    mass_each = max(float(organ.common.dry_biomass or 0.0), 0.0) * 1e-6 / len(positive) if positive else 0.0
    inclinations = tuple(getattr(organ.properties, "petiolule_inclinations", ()) or ())
    for axis in positive:
        distance, _fraction, host = min(
            (*closest_fraction(axis.world_start, support.world_start, support.world_end), support.id)
            for support in supports
        )
        if distance > 1e-6:
            raise GroIMPAdapterError(f"leaflet {axis.id} is {distance:.3g} m from its rachis")
        segment = _index(axis)
        inclination = None
        if axis.role != "rachis_terminal":
            inclination = float(inclinations[segment]) if segment < len(inclinations) else 90.0
        builder.leaflets.append(
            SkeletonLeaflet(
                id=f"g{gid}_{_SIDE[axis.role]}{segment + 1:02d}",
                node=builder.node_for_axis[host],
                side=_SIDE[axis.role],
                start=builder.local(axis.world_start),
                end=builder.local(axis.world_end),
                radius=float(axis.radius),
                frame=_rotation(axis),
                inclination_deg=inclination,
                mass_kg=mass_each,
            )
        )


def _add_trusses(builder: _Builder) -> None:
    fruits = sorted(
        (organ for organ in builder.state.organs if organ.organ_type == "Fruits"),
        key=lambda organ: (int(organ.common.order or 0), int(organ.common.rank or 0), builder.gid(organ.node_id)),
    )
    for organ in fruits:
        gid = builder.gid(organ.node_id)
        axes = builder.axes_by_owner.get(organ.node_id, [])
        rachides = sorted((axis for axis in axes if axis.role == "truss_rachis"), key=_index)
        pedicels = sorted((axis for axis in axes if axis.role == "pedicel"), key=_index)
        spheres = sorted(builder.spheres_by_owner.get(organ.node_id, []), key=lambda item: int(item.id.rsplit(":", 1)[1]))
        if not any(_positive(axis) for axis in rachides):
            continue  # truss not grown yet
        if not all(_positive(axis) for axis in (*rachides, *pedicels)):
            raise GroIMPAdapterError(f"truss {organ.node_id} has partially degenerate axes")
        previous = _nearest_internode(builder, organ.node_id)
        rachis_ids = []
        for index, axis in enumerate(rachides, start=1):
            previous = builder.add_node(f"g{gid}_truss{index:02d}", previous, "truss_rachis", axis, organ.id)
            rachis_ids.append(previous)
        positive_spheres = [sphere for sphere in spheres if sphere.radius > _MIN_SIZE]
        if positive_spheres and len(positive_spheres) != len(pedicels):
            raise GroIMPAdapterError(f"truss {organ.node_id} has {len(positive_spheres)} fruits for {len(pedicels)} pedicels")
        degree_days = tuple(getattr(organ.properties, "fruit_degree_days", ()) or ())
        ripening = float(getattr(organ.properties, "ripening_degree_days", 0.0) or 0.0)
        for index, axis in enumerate(pedicels, start=1):
            distances = [norm(sub(axis.world_start, rachis.world_end)) for rachis in rachides]
            nearest = min(range(len(rachides)), key=distances.__getitem__)
            if distances[nearest] > 1e-6:
                raise GroIMPAdapterError(f"pedicel {axis.id} does not start at a rachis segment end")
            pedicel_id = builder.add_node(
                f"g{gid}_pedicel{index:02d}", rachis_ids[nearest], "pedicel", axis, organ.id,
                length_scale=PEDICEL_LENGTH_SCALE,
            )
            if not positive_spheres:
                continue
            sphere = positive_spheres[index - 1]
            ripeness = 0.0
            if ripening > 0.0 and index - 1 < len(degree_days):
                ripeness = max(0.0, min(1.0, float(degree_days[index - 1]) / ripening))
            builder.fruits.append(
                SkeletonFruit(
                    id=f"g{gid}_fruit{index:02d}",
                    node=pedicel_id,
                    radius=float(sphere.radius),
                    ripeness=ripeness,
                    center=builder.local(sphere.world_center),
                )
            )


def plant_state_to_skeleton(state: PlantState) -> PlantSkeleton:
    """Convert one GroIMP PlantState into the exporter's skeleton format."""

    validate_plant_state(state)
    builder = _Builder(state)
    _add_structure(builder)
    _add_leaves(builder)
    _add_trusses(builder)
    skeleton = PlantSkeleton(
        nodes=tuple(builder.nodes),
        leaflets=tuple(builder.leaflets),
        fruits=tuple(builder.fruits),
        metadata=SkeletonMetadata(
            plant_id=int(state.metadata.plant_id),
            source="groimp",
            description=f"GroIMP {state.metadata.source_model or 'model'} converted by exporterV2.adapters.groimp",
            time=None if state.metadata.simulation_time is None else float(state.metadata.simulation_time),
        ),
    )
    return validate_skeleton(skeleton)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Convert a GroIMP PlantState JSON into input/skeleton.json")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--day", type=int, help="read data/plant_states/plant_state_day_N.json")
    source.add_argument("--input", type=Path, help="read this PlantState JSON instead")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "input" / "skeleton.json")
    args = parser.parse_args(argv)
    path = args.input or PROJECT_ROOT / "data" / "plant_states" / f"plant_state_day_{args.day}.json"
    skeleton = plant_state_to_skeleton(load_plant_state(path))
    save_skeleton(skeleton, args.output)
    print(
        f"[OK] {args.output}: {len(skeleton.nodes)} segments, "
        f"{len(skeleton.leaflets)} leaflets, {len(skeleton.fruits)} fruits"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
