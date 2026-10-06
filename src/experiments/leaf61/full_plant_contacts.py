"""Explicit full-plant contact filters and bounded PhysX evidence."""

import math
import time


def configure(stage, native, rows, mode):
    from pxr import PhysxSchema, UsdPhysics

    leaves_group = UsdPhysics.CollisionGroup.Define(
        stage, "/World/FlexibleLeafCollisions"
    )
    native_group = UsdPhysics.CollisionGroup.Define(
        stage, "/World/NativePlantCollisions"
    )
    leaves_group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
        ["/World/FlexibleLeaves"]
    )
    native_group.GetCollidersCollectionAPI().CreateIncludesRel().SetTargets(
        [p.GetPath() for p in native]
    )
    leaves_group.CreateFilteredGroupsRel().SetTargets(
        [leaves_group.GetPath(), native_group.GetPath()]
        if mode == "off"
        else ([leaves_group.GetPath()] if mode == "plant" else [])
    )
    native_group.CreateFilteredGroupsRel().SetTargets(
        [leaves_group.GetPath()] if mode == "off" else []
    )
    excluded = []
    if mode != "off":
        native_paths = {str(p.GetPath()) for p in native}
        neighbors = {p: {p} for p in native_paths}
        for prim in stage.Traverse():
            if not prim.IsA(UsdPhysics.Joint):
                continue
            j = UsdPhysics.Joint(prim)
            left, right = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
            if (
                left
                and right
                and str(left[0]) in native_paths
                and str(right[0]) in native_paths
            ):
                neighbors[str(left[0])].add(str(right[0]))
                neighbors[str(right[0])].add(str(left[0]))
        for row in rows:
            links = [row["prefix"] + f"/Link{j}" for j in range(row.get("segments", 3))]
            for link in links:
                targets = sorted(neighbors[row["host"]] | (set(links) - {link}))
                UsdPhysics.FilteredPairsAPI.Apply(
                    stage.GetPrimAtPath(link)
                ).CreateFilteredPairsRel().SetTargets(targets)
                PhysxSchema.PhysxContactReportAPI.Apply(
                    stage.GetPrimAtPath(link)
                ).CreateThresholdAttr().Set(0)
                excluded.append({"body": link, "filtered_bodies": targets})
    return {
        "mode": mode,
        "leaf_native_contacts": mode != "off",
        "leaf_leaf_contacts": mode == "all",
        "native_native_unchanged": True,
        "excluded_attachment_pairs": excluded,
        "policy": "Own host, its directly joint-connected native neighbors, and segments of the same blade are excluded. All other plant bodies, including fruits, can contact blades."
        if mode != "off"
        else "New leaf contacts with plant and other blades disabled.",
    }


class ContactMonitor:
    def __init__(self, sample_hz=None):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        self.sample_hz = sample_hz
        self.last_sample = -1
        self.sampled_callbacks = 0
        self.time = 0.0
        self.callback_seconds = 0.0
        self.pairs = {}
        self.body_keys = {}
        self.decode = PhysicsSchemaTools.intToSdfPath
        self.subscription = (
            get_physx_simulation_interface().subscribe_contact_report_events(
                self.contact
            )
        )

    def contact(self, headers, details):
        if self.sample_hz is not None:
            sample = int(self.time * self.sample_hz + 1e-8)
            if sample == self.last_sample:
                return
            self.last_sample = sample
        self.sampled_callbacks += 1
        tick = time.perf_counter()
        for h in headers:
            if h.num_contact_data == 0:
                continue
            actor_key = (h.actor0, h.actor1)
            if actor_key not in self.body_keys:
                self.body_keys[actor_key] = tuple(
                    sorted(str(self.decode(v)) for v in actor_key)
                )
            key = self.body_keys[actor_key]
            bodies = key
            if not any("/FlexibleLeaves/" in v for v in bodies):
                continue
            if key not in self.pairs:
                self.pairs[key] = {
                    "bodies": bodies,
                    "first_s": self.time,
                    "last_s": self.time,
                    "points": 0,
                    "impulsive_points": 0,
                    "max_penetration_m": 0.0,
                    "first_step_penetration_m": 0.0,
                    "max_penetration_after_8s_m": 0.0,
                    "impulse_sum_Ns": 0.0,
                    "kind": "leaf_leaf"
                    if all("/FlexibleLeaves/" in b for b in bodies)
                    else "leaf_plant",
                }
            row = self.pairs[key]
            row["last_s"] = self.time
            for d in details[
                h.contact_data_offset : h.contact_data_offset + h.num_contact_data
            ]:
                penetration = max(0.0, -float(d.separation))
                x, y, z = d.impulse
                impulse = math.sqrt(x * x + y * y + z * z)
                row["points"] += 1
                row["impulsive_points"] += int(impulse > 0)
                row["impulse_sum_Ns"] += impulse
                row["max_penetration_m"] = max(row["max_penetration_m"], penetration)
                if self.time <= 1 / 480:
                    row["first_step_penetration_m"] = max(
                        row["first_step_penetration_m"], penetration
                    )
                if self.time >= 8:
                    row["max_penetration_after_8s_m"] = max(
                        row["max_penetration_after_8s_m"], penetration
                    )
        self.callback_seconds += time.perf_counter() - tick

    def result(self):
        rows = list(self.pairs.values())
        return {
            "pairs": rows,
            "sample_hz": self.sample_hz,
            "sampled_callbacks": self.sampled_callbacks,
            "impulses_complete": self.sample_hz is None,
            "contact_callback_seconds": self.callback_seconds,
            "settled_window_observed": self.time >= 8,
            "leaf_plant_impulsive_pairs": sum(
                r["kind"] == "leaf_plant" and r["impulsive_points"] > 0 for r in rows
            ),
            "leaf_leaf_impulsive_pairs": sum(
                r["kind"] == "leaf_leaf" and r["impulsive_points"] > 0 for r in rows
            ),
            "first_step_penetration_max_m": max(
                (r["first_step_penetration_m"] for r in rows), default=0
            ),
            "penetration_after_8s_max_m": max(
                (r["max_penetration_after_8s_m"] for r in rows), default=0
            ),
        }


def exclude_initial_pairs(stage, evidence):
    """Explicitly omit only measured first-step overlaps from a previous run."""
    from pxr import UsdPhysics

    excluded = []
    for row in evidence["pairs"]:
        if row["first_step_penetration_m"] <= 0.0001:
            continue
        left, right = row["bodies"]
        if not stage.GetPrimAtPath(left) or not stage.GetPrimAtPath(right):
            raise RuntimeError("Initial-overlap evidence references a missing body")
        if not any("/FlexibleLeaves/" in p for p in (left, right)):
            raise RuntimeError("Refusing to change native-native filtering")
        UsdPhysics.FilteredPairsAPI.Apply(
            stage.GetPrimAtPath(left)
        ).CreateFilteredPairsRel().AddTarget(right)
        excluded.append(
            {
                "bodies": [left, right],
                "measured_initial_penetration_m": row["first_step_penetration_m"],
            }
        )
    return excluded
