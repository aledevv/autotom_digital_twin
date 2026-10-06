"""Hardware-independent complexity profile of a plant and of its simulation model.

The profile separates three levels so that a measured cost can later be linked
to counts instead of to a hand-weighted index:

* ``plant``  -- level A, the biological structure read from PlantState JSON;
* ``model``  -- level B, the authored OpenUSD physics and visual workload;
* ``solver`` -- the configuration that multiplies the level-B workload.

Only ``pxr`` (usd-core) is required.  Example::

    uv run python -m exporterV2.complexity_profile \
      data/usd_models/tree_v2_day_50.usda \
      --plant-state data/plant_states/plant_state_day_50.json
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import itertools
import json
from pathlib import Path
from typing import Any

from exporterV2.performance_benchmark import collect_stage_statistics


SCHEMA_VERSION = "autotom_complexity_profile/1.0"
D6_AXES = ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")
_JOINT_TYPES = {
    "PhysicsJoint": "d6",
    "PhysicsFixedJoint": "fixed",
    "PhysicsRevoluteJoint": "revolute",
    "PhysicsPrismaticJoint": "prismatic",
    "PhysicsSphericalJoint": "spherical",
    "PhysicsDistanceJoint": "distance",
}
_COLLIDER_TYPES = ("Capsule", "Sphere", "Cube", "Cylinder", "Cone", "Mesh", "Plane")


def _get(prim, name: str, default=None):
    attr = prim.GetAttribute(name)
    if attr and attr.HasAuthoredValue():
        return attr.Get()
    return default


def _targets(prim, name: str) -> list[str]:
    rel = prim.GetRelationship(name)
    return [str(target) for target in rel.GetTargets()] if rel else []


def _owning_body(prim, bodies: set[str]) -> str | None:
    while prim and prim.IsValid() and not prim.IsPseudoRoot():
        path = str(prim.GetPath())
        if path in bodies:
            return path
        prim = prim.GetParent()
    return None


def _is_visible(prim) -> bool:
    from pxr import UsdGeom

    imageable = UsdGeom.Imageable(prim)
    if not imageable:
        return True
    if imageable.ComputeVisibility() == UsdGeom.Tokens.invisible:
        return False
    return imageable.ComputePurpose() in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render)


def joint_dofs(prim) -> dict[str, int]:
    """Return free DOFs, limited DOFs and driven axes of one authored joint.

    A D6 axis without a limit is free; ``low > high`` locks it, as in
    ``core/usd/joints.py:configure_joint_drives``.
    """

    kind = _JOINT_TYPES.get(prim.GetTypeName())
    schemas = set(prim.GetAppliedSchemas())
    drives = sum(1 for schema in schemas if schema.startswith("PhysicsDriveAPI:"))
    if kind == "d6":
        free = limited = 0
        for axis in D6_AXES:
            low = _get(prim, f"limit:{axis}:physics:low")
            high = _get(prim, f"limit:{axis}:physics:high")
            if low is None or high is None:
                free += 1
            elif float(low) > float(high):
                continue
            else:
                free += 1
                limited += 1
        return {"dofs": free, "limited": limited, "drives": drives}
    if kind in ("revolute", "prismatic"):
        low = _get(prim, "physics:lowerLimit")
        high = _get(prim, "physics:upperLimit")
        limited = int(low is not None and high is not None)
        return {"dofs": 1, "limited": limited, "drives": drives}
    if kind == "spherical":
        limited = int(_get(prim, "physics:coneAngle0Limit") is not None)
        return {"dofs": 3, "limited": limited, "drives": drives}
    return {"dofs": 0, "limited": 0, "drives": drives}


def _articulation_members(stage, bodies: set[str], joints: list[dict]) -> list[set[str]]:
    from pxr import UsdPhysics

    adjacency: dict[str, set[str]] = defaultdict(set)
    for joint in joints:
        if joint["excluded"] or not joint["enabled"]:
            continue
        a, b = joint["body0"], joint["body1"]
        if a in bodies and b in bodies:
            adjacency[a].add(b)
            adjacency[b].add(a)
    articulations = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            continue
        root = str(prim.GetPath())
        seeds = {body for body in bodies if body == root or body.startswith(root + "/")}
        members, queue = set(seeds), deque(seeds)
        while queue:
            for nxt in adjacency[queue.popleft()]:
                if nxt not in members:
                    members.add(nxt)
                    queue.append(nxt)
        articulations.append(
            {
                "root": root,
                "members": members,
                "self_collisions": bool(
                    _get(prim, "physxArticulation:enabledSelfCollisions", True)
                ),
                "position_iterations": _get(
                    prim, "physxArticulation:solverPositionIterationCount"
                ),
                "velocity_iterations": _get(
                    prim, "physxArticulation:solverVelocityIterationCount"
                ),
            }
        )
    return articulations


def stage_complexity(path: str | Path) -> dict[str, Any]:
    """Return level-B model counts and the solver configuration of a stage."""

    from pxr import Usd, UsdGeom, UsdPhysics

    base = collect_stage_statistics(path)
    stage = Usd.Stage.Open(str(base["path"]))
    prims = list(stage.Traverse())
    bodies = {str(p.GetPath()) for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    kinematic = {
        str(p.GetPath())
        for p in prims
        if str(p.GetPath()) in bodies and _get(p, "physics:kinematicEnabled", False)
    }

    joints = []
    for prim in prims:
        kind = _JOINT_TYPES.get(prim.GetTypeName())
        if kind is None:
            continue
        body0 = _targets(prim, "physics:body0")
        body1 = _targets(prim, "physics:body1")
        joints.append(
            {
                "kind": kind,
                "body0": body0[0] if body0 else None,
                "body1": body1[0] if body1 else None,
                "excluded": bool(_get(prim, "physics:excludeFromArticulation", False)),
                "enabled": bool(_get(prim, "physics:jointEnabled", True)),
                "breakable": _get(prim, "physics:breakForce") is not None
                or _get(prim, "physics:breakTorque") is not None,
                **joint_dofs(prim),
            }
        )
    articulations = _articulation_members(stage, bodies, joints)
    member_of = {
        body: index for index, art in enumerate(articulations) for body in art["members"]
    }

    reduced = [j for j in joints if j["enabled"] and not j["excluded"]]
    maximal = [j for j in joints if j["enabled"] and j["excluded"]]
    dofs = sum(j["dofs"] for j in reduced)
    # Reduced-coordinate joints add solver rows only for limits and drives.
    # Maximal-coordinate joints (fruit attachments) also constrain each
    # locked DOF, i.e. 6 - free rows.
    constraint_rows = sum(j["limited"] + j["drives"] for j in reduced) + sum(
        (6 - j["dofs"]) + j["limited"] + j["drives"] for j in maximal
    )

    colliders = []
    collider_types: Counter[str] = Counter()
    for prim in prims:
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        collider_types[prim.GetTypeName() or "<typeless>"] += 1
        colliders.append(_owning_body(prim, bodies))

    filtered: set[frozenset[str]] = set()
    for prim in prims:
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            source = str(prim.GetPath())
            for target in UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel().GetTargets():
                filtered.add(frozenset((source, str(target))))
    jointed = {
        frozenset((j["body0"], j["body1"]))
        for j in joints
        if j["body0"] and j["body1"] and j["enabled"]
    }

    raw_pairs = self_excluded = filter_excluded = joint_excluded = static_pairs = 0
    candidate_pairs = 0
    for owner_a, owner_b in itertools.combinations(colliders, 2):
        if owner_a is not None and owner_a == owner_b:
            continue
        raw_pairs += 1
        dynamic_a = owner_a is not None and owner_a not in kinematic
        dynamic_b = owner_b is not None and owner_b not in kinematic
        if not (dynamic_a or dynamic_b):
            static_pairs += 1
            continue
        art_a, art_b = member_of.get(owner_a), member_of.get(owner_b)
        if art_a is not None and art_a == art_b and not articulations[art_a]["self_collisions"]:
            self_excluded += 1
            continue
        pair = frozenset((owner_a, owner_b))
        if pair in filtered:
            filter_excluded += 1
            continue
        if pair in jointed:
            joint_excluded += 1
            continue
        candidate_pairs += 1

    visible_meshes = visible_triangles = 0
    for prim in prims:
        if prim.IsA(UsdGeom.Mesh) and _is_visible(prim):
            visible_meshes += 1
            counts = UsdGeom.Mesh(prim).GetFaceVertexCountsAttr().Get() or ()
            visible_triangles += sum(max(0, int(count) - 2) for count in counts)

    scene = next((p for p in prims if p.IsA(UsdPhysics.Scene)), None)
    scene_attrs = {}
    if scene is not None:
        for name in (
            "physxScene:solverType",
            "physxScene:timeStepsPerSecond",
            "physxScene:enableGPUDynamics",
            "physxScene:broadphaseType",
            "physxScene:enableCCD",
            "physxScene:enableStabilization",
        ):
            value = _get(scene, name)
            scene_attrs[name.split(":", 1)[1]] = value if not hasattr(value, "item") else value.item()
    position_iterations = max(
        (a["position_iterations"] or 0 for a in articulations), default=0
    )
    hz = scene_attrs.get("timeStepsPerSecond") or base["authored_physics_hz"] or 0

    joint_counts = Counter(j["kind"] for j in joints)
    model = {
        "rigid_bodies": len(bodies),
        "kinematic_bodies": len(kinematic),
        "articulations": len(articulations),
        "max_links_per_articulation": max((len(a["members"]) for a in articulations), default=0),
        "joints": dict(sorted(joint_counts.items())),
        "articulation_joints": len(reduced),
        "maximal_joints": len(maximal),
        "breakable_joints": sum(j["breakable"] for j in joints),
        "dofs": dofs,
        "limited_dofs": sum(j["limited"] for j in reduced),
        "drives": sum(j["drives"] for j in joints),
        "constraint_rows": constraint_rows,
        "collision_shapes": len(colliders),
        "collision_shape_types": dict(sorted(collider_types.items())),
        "collision_pairs": {
            "raw": raw_pairs,
            "static_static": static_pairs,
            "articulation_self_excluded": self_excluded,
            "filtered_pairs_excluded": filter_excluded,
            "jointed_excluded": joint_excluded,
            "candidate": candidate_pairs,
        },
        "filtered_pairs_authored": len(filtered),
        "mesh_prims": base["prim_types"].get("Mesh", 0),
        "visible_mesh_prims": visible_meshes,
        "mesh_triangles": base["mesh_triangles"],
        "visible_triangles": visible_triangles,
        "mesh_points": base["mesh_points"],
        "material_count": base["material_count"],
        "total_prims": base["total_prims"],
        "file_bytes": base["file_bytes"],
    }
    solver = {
        **scene_attrs,
        "articulation_position_iterations": position_iterations,
        "articulation_velocity_iterations": max(
            (a["velocity_iterations"] or 0 for a in articulations), default=0
        ),
        # Featherstone-style cost proxy: work per simulated second.
        "work_proxy_dof_iters_per_s": dofs * position_iterations * hz,
    }
    return {"path": base["path"], "sha256": base["sha256"], "model": model, "solver": solver}


def plant_complexity(plant_state: str | Path | dict) -> dict[str, Any]:
    """Return level-A structural counts from a PlantState JSON document.

    Branching order follows the MTG convention: it increases by one at every
    ``branch`` edge on the path from the root (0 = main stem).
    """

    if not isinstance(plant_state, dict):
        plant_state = json.loads(Path(plant_state).read_text())
    organs = plant_state.get("organs", [])
    organ_by_node = {organ["node_id"]: organ for organ in organs}
    parent: dict[str, tuple[str, str]] = {}
    for edge in plant_state.get("edges", []):
        parent[edge["target"]] = (edge["source"], edge["kind"])

    order: dict[str, int] = {}
    depth: dict[str, int] = {}

    def resolve(node: str) -> tuple[int, int]:
        chain = []
        while node not in order and node in parent:
            chain.append(node)
            node = parent[node][0]
        if node not in order:
            order[node], depth[node] = 0, int(node in organ_by_node)
        for child in reversed(chain):
            source, kind = parent[child]
            order[child] = order[source] + (kind == "branch")
            depth[child] = depth[source] + (child in organ_by_node)
        return order[node], depth[node]

    for node in organ_by_node:
        resolve(node)
    organ_types = Counter(organ["organ_type"] for organ in organs)
    organ_orders = [order[node] for node in organ_by_node]
    order_counts = Counter(order[node] for node, organ in organ_by_node.items()
                           if organ["organ_type"] in ("Internode", "Leaf", "Truss"))
    return {
        "day": plant_state.get("metadata", {}).get("simulation_time"),
        "organs": len(organs),
        "organ_types": dict(sorted(organ_types.items())),
        "metamers": organ_types.get("Internode", 0),
        "fruits": len(plant_state.get("spheres", [])),
        "graph_nodes": len(plant_state.get("nodes", [])),
        "graph_edges": len(plant_state.get("edges", [])),
        "branch_edges": sum(e["kind"] == "branch" for e in plant_state.get("edges", [])),
        "max_branching_order": max(organ_orders, default=0),
        "organs_by_branching_order": {str(k): v for k, v in sorted(order_counts.items())},
        "max_topological_depth": max((depth[node] for node in organ_by_node), default=0),
        "geometry_axes": len(plant_state.get("axes", [])),
    }


def complexity_profile(
    usd_path: str | Path,
    *,
    manifest: str | Path | None = None,
    plant_state: str | Path | None = None,
) -> dict[str, Any]:
    """Combine level A, level B and the solver configuration in one document."""

    usd_path = Path(usd_path).expanduser().resolve()
    if manifest is None:
        guess = usd_path.with_suffix(".manifest.json")
        manifest = guess if guess.is_file() else None
    profile: dict[str, Any] = {"schema_version": SCHEMA_VERSION, **stage_complexity(usd_path)}
    if manifest is not None:
        document = json.loads(Path(manifest).read_text())
        profile["variant"] = document.get("metadata", {})
        profile["manifest"] = str(Path(manifest).resolve())
    if plant_state is not None:
        profile["plant"] = plant_complexity(plant_state)
        profile["plant_state"] = str(Path(plant_state).resolve())
    return profile


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("usd", type=Path)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--plant-state", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    profile = complexity_profile(args.usd, manifest=args.manifest, plant_state=args.plant_state)
    text = json.dumps(profile, indent=2, sort_keys=True, default=str)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
