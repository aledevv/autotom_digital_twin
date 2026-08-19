"""Hybrid orchestration for ExporterV2 skinned vegetative branches."""

import os
from typing import Dict, Iterable

from pxr import UsdGeom

from .adapter import branch_system, resolve_vegetative_graph
from .axis import build_visual_axes
from .branch_physics import author_branch_joints, author_rigid_links
from .fork_bridge import author_centered_existing_arm_bridge
from .global_visual import author_global_visual_axes
from .mesh import author_visual_axis
from .terminal_fork import author_terminal_visual_forks
from .visual_modes import (
    author_rigid_visual_axis,
    author_segmented_visual_axis,
    author_static_visual_axis,
)


VALID_VISUAL_MODES = (
    "skinned",
    "static",
    "rigid-single",
    "global",
    "segmented",
    "segmented-fork",
)
VISUAL_MODE_ENV = "AUTOTOM_SKINNING_VISUAL_MODE"


def _is_structural_terminal_host(branch: dict) -> bool:
    branch_id = str(branch.get("id", "")).lower()
    kind = str(branch.get("kind", "")).lower()
    return (
        branch.get("parent") is None
        or branch_id.startswith("branch_r")
        or kind in {"stem", "trunk", "branch", "lateral_branch"}
    )


def _mark_centered_terminal_leaf_branches(all_branch_defs: Dict[str, dict]) -> None:
    """Center real terminal petioles for the segmented-fork visual mode.

    This deliberately changes only the attachment offset of an already-existing
    leaf petiole.  The branch, joint, mass, collision proxy and downstream leaf
    hierarchy are preserved.  Trusses stay on the legacy path for now because
    they are authored by the separate truss subsystem after this builder.
    """
    for parent in all_branch_defs.values():
        if branch_system(parent) != "vegetative":
            continue
        if not _is_structural_terminal_host(parent):
            continue

        parent_id = parent.get("id")
        parent_links = int(parent.get("n_links", 0))
        if not parent_id or parent_links <= 0:
            continue

        candidates = []
        for child in all_branch_defs.values():
            if branch_system(child) != "vegetative":
                continue
            if child.get("parent") != parent_id:
                continue
            child_id = str(child.get("id", "")).lower()
            if "petiole" not in child_id:
                continue
            try:
                attach_link = int(child.get("attach_link", -1))
                attach_frac = float(child.get("attach_frac", 1.0))
            except (TypeError, ValueError):
                continue
            if attach_link == parent_links and attach_frac >= 0.95:
                candidates.append(child)

        if candidates:
            # Deterministic: if malformed input contains multiple terminal
            # petioles, center only one instead of stacking several on axis.
            chosen = sorted(candidates, key=lambda item: str(item.get("id", "")))[0]
            chosen["_terminal_fork_centered"] = True


def build_skinned_vegetative_structure(
    stage,
    stem_path: str,
    branches: Iterable[dict],
    *,
    all_branch_defs: Dict[str, dict],
    locked_joints: bool = False,
    legacy_physics: bool = False,
    visual_mode: str = None,
):
    """Build vegetative physics and one of the supported smooth visual modes."""
    if visual_mode is None:
        visual_mode = os.environ.get(VISUAL_MODE_ENV, "skinned")
    if visual_mode not in VALID_VISUAL_MODES:
        raise ValueError(
            f"Unsupported visual_mode={visual_mode!r}; expected one of {VALID_VISUAL_MODES}"
        )

    # Mark BEFORE graph resolution so the actual petiole joint starts on the
    # parent centerline.  No visual bridge is required for leaf forks anymore.
    if visual_mode == "segmented-fork":
        _mark_centered_terminal_leaf_branches(all_branch_defs)

    physics_parent = f"{stem_path}/Vegetative"
    visual_parent = "/World/PlantVisual"
    UsdGeom.Xform.Define(stage, physics_parent)
    UsdGeom.Xform.Define(stage, visual_parent)

    resolved = resolve_vegetative_graph(
        branches,
        all_branch_defs=all_branch_defs,
        physics_parent_path=physics_parent,
        visual_parent_path=visual_parent,
        locked_joints=locked_joints,
        legacy_physics=legacy_physics,
    )
    by_id = {branch.branch_id: branch for branch in resolved}

    for branch in resolved:
        author_rigid_links(stage, branch)
    for branch in resolved:
        author_branch_joints(stage, branch, by_id)

    visual_axes = build_visual_axes(resolved, visual_parent)
    axis_by_member = {
        member.branch_id: axis
        for axis in visual_axes
        for member in axis.members
    }

    for child in resolved:
        if child.parent_id is None:
            continue
        parent_axis = axis_by_member[child.parent_id]
        child_axis = axis_by_member[child.branch_id]
        if child_axis is parent_axis:
            continue

        parent = by_id[child.parent_id]
        attach_frac = max(
            0.0,
            min(1.0, float(child.definition.get("attach_frac", 1.0))),
        )
        normalized_arc = (
            child.parent_link_index + attach_frac
        ) / parent.n_links
        local_arc = normalized_arc * parent_axis.member_lengths[parent.branch_id]
        global_arc = parent_axis.member_offsets[parent.branch_id] + local_arc
        parent_axis.attachment_arcs.append(round(global_arc, 12))

    for axis in visual_axes:
        axis.attachment_arcs = sorted(set(axis.attachment_arcs))

    if visual_mode == "global":
        stats = author_global_visual_axes(stage, visual_axes, resolved)
        print(
            "[SKIN-VISUAL] "
            f"mode=global | shared_skeletons=1 | "
            f"axes={stats['axes']} | bones={stats['bones']} | meshes={stats['meshes']}"
        )
        return {
            branch.branch_id: branch.as_registry_entry()
            for branch in resolved
        }

    fork_records = []
    fork_parent_ids = set()
    if visual_mode == "segmented-fork":
        fork_records = author_terminal_visual_forks(
            stage,
            visual_axes,
            all_branch_defs,
        )
        fork_parent_ids = {record["parent"] for record in fork_records}

        # Leaf petioles are now physically centered at their existing joint, so
        # they need no sleeve/bridge.  Trusses are still authored later by the
        # legacy truss subsystem; keep the short visual bridge only for those
        # until that separate subsystem is centered too.
        for record in fork_records:
            child_def = all_branch_defs.get(record["existing_child"])
            if child_def is None or branch_system(child_def) != "truss":
                continue
            parent_axis = axis_by_member.get(record["parent"])
            if parent_axis is None:
                continue
            author_centered_existing_arm_bridge(
                stage,
                parent_axis,
                child_def,
            )

    counts = {
        "skinned_axes": 0,
        "static_axes": 0,
        "rigid_single_axes": 0,
        "segmented_axes": 0,
        "segmented_meshes": 0,
        "segmented_tongues": 0,
    }

    segmented_mode = visual_mode in ("segmented", "segmented-fork")

    for axis in visual_axes:
        if visual_mode == "static":
            author_static_visual_axis(stage, axis)
            counts["static_axes"] += 1
            continue

        if visual_mode == "rigid-single" and len(axis.link_paths) == 1:
            author_rigid_visual_axis(stage, axis)
            counts["rigid_single_axes"] += 1
            continue

        if segmented_mode:
            terminal_member_id = axis.members[-1].branch_id
            terminal_tip_scale = (
                0.48
                if visual_mode == "segmented-fork"
                and terminal_member_id in fork_parent_ids
                else 1.0
            )
            stats = author_segmented_visual_axis(
                stage,
                axis,
                terminal_tip_scale=terminal_tip_scale,
            )
            counts["segmented_axes"] += 1
            counts["segmented_meshes"] += stats["segments"]
            counts["segmented_tongues"] += stats["tongues"]
            continue

        author_visual_axis(stage, axis)
        counts["skinned_axes"] += 1

    if segmented_mode:
        print(
            "[SKIN-VISUAL] "
            f"mode={visual_mode} | UsdSkel=0 | "
            f"axes={counts['segmented_axes']} | "
            f"rigid_meshes={counts['segmented_meshes']} | "
            f"joint_tongues={counts['segmented_tongues']} | "
            f"terminal_forks={len(fork_records)}"
        )
        if fork_records:
            preview = ", ".join(
                f"{record['parent']}->{record['existing_child']}"
                for record in fork_records[:8]
            )
            if len(fork_records) > 8:
                preview += f", ... (+{len(fork_records) - 8})"
            print(f"[TERMINAL-FORK] {preview}")
    else:
        print(
            "[SKIN-VISUAL] "
            f"mode={visual_mode} | "
            f"skinned_axes={counts['skinned_axes']} | "
            f"rigid_single_axes={counts['rigid_single_axes']} | "
            f"static_axes={counts['static_axes']}"
        )

    return {branch.branch_id: branch.as_registry_entry() for branch in resolved}
