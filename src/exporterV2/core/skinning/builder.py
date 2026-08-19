"""Hybrid orchestration for ExporterV2 skinned vegetative branches."""

import os
from typing import Dict, Iterable

from pxr import UsdGeom

from .adapter import resolve_vegetative_graph
from .axis import build_visual_axes
from .branch_physics import author_branch_joints, author_rigid_links
from .mesh import author_visual_axis
from .visual_modes import author_rigid_visual_axis, author_static_visual_axis


VALID_VISUAL_MODES = ("skinned", "static", "rigid-single")
VISUAL_MODE_ENV = "AUTOTOM_SKINNING_VISUAL_MODE"


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
    """Build vegetative physics and one of the supported smooth visual modes.

    visual_mode:
      - ``skinned``: current full UsdSkel implementation.
      - ``static``: exact same smooth tube meshes, but no UsdSkel anywhere.
      - ``rigid-single``: one-link axes are plain meshes parented directly to
        their PhysX rigid body; multi-link axes keep normal UsdSkel skinning.

    If ``visual_mode`` is omitted, the diagnostic environment variable
    ``AUTOTOM_SKINNING_VISUAL_MODE`` is used, falling back to ``skinned``.
    """
    if visual_mode is None:
        visual_mode = os.environ.get(VISUAL_MODE_ENV, "skinned")
    if visual_mode not in VALID_VISUAL_MODES:
        raise ValueError(
            f"Unsupported visual_mode={visual_mode!r}; expected one of {VALID_VISUAL_MODES}"
        )

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

    counts = {
        "skinned_axes": 0,
        "static_axes": 0,
        "rigid_single_axes": 0,
    }

    for axis in visual_axes:
        axis.attachment_arcs = sorted(set(axis.attachment_arcs))

        if visual_mode == "static":
            author_static_visual_axis(stage, axis)
            counts["static_axes"] += 1
            continue

        if visual_mode == "rigid-single" and len(axis.link_paths) == 1:
            author_rigid_visual_axis(stage, axis)
            counts["rigid_single_axes"] += 1
            continue

        author_visual_axis(stage, axis)
        counts["skinned_axes"] += 1

    print(
        "[SKIN-VISUAL] "
        f"mode={visual_mode} | "
        f"skinned_axes={counts['skinned_axes']} | "
        f"rigid_single_axes={counts['rigid_single_axes']} | "
        f"static_axes={counts['static_axes']}"
    )

    return {branch.branch_id: branch.as_registry_entry() for branch in resolved}
