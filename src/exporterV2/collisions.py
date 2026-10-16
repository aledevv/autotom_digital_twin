"""Initial-overlap check for the authored colliders.

Overlapping colliders at rest make PhysX push bodies apart on the first step.
``auto_filter_initial_overlaps`` finds each overlapping pair of rigid bodies
and disables collisions between exactly that pair; ``collider_overlaps``
reports what is left.
"""

from __future__ import annotations

import math
from typing import Any

from pxr import Gf, UsdGeom, UsdPhysics

from .core.usd.collision import add_collision_filter


def _segment_distance(a0, a1, b0, b1) -> float:
    """Shortest distance between finite 3-D line segments."""

    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    aa = u * u
    bb = u * v
    cc = v * v
    dd = u * w
    ee = v * w
    denominator = aa * cc - bb * bb
    small = 1e-15

    # Spheres are represented by zero-length swept segments.  Handle both
    # point/segment orders explicitly: the general segment algorithm below
    # assumes non-degenerate directions and used to miss sphere-as-right-hand
    # overlaps depending solely on deterministic path ordering.
    if aa < small and cc < small:
        return math.sqrt(w * w)
    if aa < small:
        t = max(0.0, min(1.0, ee / cc))
        delta = w - t * v
        return math.sqrt(delta * delta)
    if cc < small:
        s = max(0.0, min(1.0, -dd / aa))
        delta = w + s * u
        return math.sqrt(delta * delta)

    s_num = 0.0
    s_den = denominator
    t_num = 0.0
    t_den = denominator

    if denominator < small:
        s_num = 0.0
        s_den = 1.0
        t_num = ee
        t_den = cc
    else:
        s_num = bb * ee - cc * dd
        t_num = aa * ee - bb * dd
        if s_num < 0.0:
            s_num = 0.0
            t_num = ee
            t_den = cc
        elif s_num > s_den:
            s_num = s_den
            t_num = ee + bb
            t_den = cc

    if t_num < 0.0:
        t_num = 0.0
        if -dd < 0.0:
            s_num = 0.0
        elif -dd > aa:
            s_num = s_den
        else:
            s_num = -dd
            s_den = aa
    elif t_num > t_den:
        t_num = t_den
        if -dd + bb < 0.0:
            s_num = 0.0
        elif -dd + bb > aa:
            s_num = s_den
        else:
            s_num = -dd + bb
            s_den = aa

    sc = 0.0 if abs(s_num) < small else s_num / s_den
    tc = 0.0 if abs(t_num) < small else t_num / t_den
    delta = w + sc * u - tc * v
    return math.sqrt(delta * delta)


def _rigid_body_path(prim) -> str | None:
    current = prim
    while current and current.IsValid() and not current.IsPseudoRoot():
        if current.HasAPI(UsdPhysics.RigidBodyAPI):
            return str(current.GetPath())
        current = current.GetParent()
    return None


def _collider_records(stage):
    """Return deterministic swept-segment records for authored colliders.

    Capsules are exact. Cylinders use their finite centerline and radius as a
    conservative narrow phase, while spheres are zero-length segments.
    """

    records = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        body_path = _rigid_body_path(prim)
        if body_path is None:
            continue
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0)
        center = matrix.ExtractTranslation()
        if prim.IsA(UsdGeom.Capsule):
            shape = UsdGeom.Capsule(prim)
            shape_type = "capsule"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = float(shape.GetHeightAttr().Get()) * 0.5
        elif prim.IsA(UsdGeom.Cylinder):
            shape = UsdGeom.Cylinder(prim)
            shape_type = "cylinder"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = float(shape.GetHeightAttr().Get()) * 0.5
        elif prim.IsA(UsdGeom.Sphere):
            shape = UsdGeom.Sphere(prim)
            shape_type = "sphere"
            radius = float(shape.GetRadiusAttr().Get())
            half_spine = 0.0
        else:
            continue
        direction = Gf.Vec3d(
            matrix.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        ).GetNormalized()
        start = center - direction * half_spine
        end = center + direction * half_spine
        records.append(
            {
                "path": str(prim.GetPath()),
                "body_path": body_path,
                "shape": shape_type,
                "start": start,
                "end": end,
                "radius": radius,
                "aabb_min": tuple(
                    min(float(start[i]), float(end[i])) - radius for i in range(3)
                ),
                "aabb_max": tuple(
                    max(float(start[i]), float(end[i])) + radius for i in range(3)
                ),
            }
        )
    return sorted(records, key=lambda item: item["path"])


def _filtered_targets(stage, body_path: str) -> set[str]:
    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        return set()
    relationship = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
    return {str(path) for path in relationship.GetTargets()}


def _body_metadata(stage, body_path: str) -> dict[str, Any]:
    prim = stage.GetPrimAtPath(body_path)
    return {
        "node_id": prim.GetAttribute("autotom:canonicalPrimitiveId").Get(),
        "canonical_organ_id": prim.GetAttribute("autotom:canonicalOrganId").Get(),
        "role": prim.GetAttribute("autotom:role").Get(),
    }


def _aabb_overlap(left, right) -> bool:
    return all(
        left["aabb_min"][axis] <= right["aabb_max"][axis]
        and right["aabb_min"][axis] <= left["aabb_max"][axis]
        for axis in range(3)
    )


def collider_overlaps(stage) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    colliders = _collider_records(stage)
    filtered_cache = {
        record["body_path"]: _filtered_targets(stage, record["body_path"])
        for record in colliders
    }
    filtered = []
    active = []
    for index, left in enumerate(colliders):
        for right in colliders[index + 1 :]:
            if left["body_path"] == right["body_path"]:
                continue
            if not _aabb_overlap(left, right):
                continue
            distance = _segment_distance(
                left["start"], left["end"], right["start"], right["end"]
            )
            depth = left["radius"] + right["radius"] - distance
            if depth <= 1e-9:
                continue
            is_filtered = (
                right["body_path"] in filtered_cache[left["body_path"]]
                or left["body_path"] in filtered_cache[right["body_path"]]
            )
            record = {
                "body_a": left["body_path"],
                "body_b": right["body_path"],
                "collider_a": left["path"],
                "collider_b": right["path"],
                "shape_a": left["shape"],
                "shape_b": right["shape"],
                "depth": round(float(depth), 12),
                "filtered": is_filtered,
                "metadata_a": _body_metadata(stage, left["body_path"]),
                "metadata_b": _body_metadata(stage, right["body_path"]),
            }
            (filtered if is_filtered else active).append(record)
    key = lambda value: (
        value["body_a"], value["body_b"], value["collider_a"], value["collider_b"]
    )
    return sorted(filtered, key=key), sorted(active, key=key)


def auto_filter_initial_overlaps(stage) -> list[dict[str, Any]]:
    """Filter each still-active initial overlap once at rigid-body granularity."""

    _filtered, active = collider_overlaps(stage)
    grouped: dict[tuple[str, str], list[dict[str, Any]]] = {}
    for record in active:
        pair = tuple(sorted((record["body_a"], record["body_b"])))
        grouped.setdefault(pair, []).append(record)
    applied = []
    for (body_a, body_b), contacts in sorted(grouped.items()):
        add_collision_filter(stage, body_a, body_b)
        add_collision_filter(stage, body_b, body_a)
        deepest = max(contact["depth"] for contact in contacts)
        applied.append(
            {
                "body_a": body_a,
                "body_b": body_b,
                "reason": "auto_filtered_initial_authored_overlap",
                "depth": deepest,
                "contact_count": len(contacts),
                "contacts": contacts,
                "permanent_pair_filter": True,
            }
        )
    return applied


