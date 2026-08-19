import copy
import math

import pytest
from pxr import Sdf, Usd, UsdGeom, UsdSkel

from exporterV2.core.optimizations.techniques.leaf_branch_reduce import (
    LeafBranchReductionTechnique,
)
from exporterV2.core.optimizations.techniques.stem_collapse import (
    StemCollapseTechnique,
)
from exporterV2.core.skinning import (
    SkinningRuntime,
    build_skinned_vegetative_structure,
    build_visual_axes,
    partition_branches,
    resolve_vegetative_graph,
)
from exporterV2.core.skinning.schema import (
    BRANCH_ID_ATTR,
    PHYSICS_LINKS_REL,
    SCHEMA_VERSION_ATTR,
    VISUAL_AXIS_ID_ATTR,
)


def _segment(source_id, length, radius):
    return {"source_id": source_id, "length": length, "radius": radius}


def _leaf_graph():
    axis_id = "Leaf_r2_o0_axis"
    return [
        {
            "id": "trunk",
            "system": "vegetative",
            "visual_axis_id": "trunk",
            "visual_segments": [_segment("trunk", 0.20, 0.01)],
            "parent": None,
            "attach_link": None,
            "n_links": 2,
            "radius": 0.01,
            "height": 0.10,
            "tilt": 0.0,
            "rot": 0.0,
            "joint_type": "fixed",
        },
        {
            "id": "Leaf_r2_o0_petiole",
            "system": "vegetative",
            "visual_axis_id": axis_id,
            "visual_segments": [_segment("Leaf_r2_o0_petiole", 0.03, 0.004)],
            "parent": "trunk",
            "attach_link": 2,
            "n_links": 1,
            "radius": 0.004,
            "height": 0.03,
            "tilt": 40.0,
            "rot": 90.0,
        },
        {
            "id": "Leaf_r2_o0_rachis",
            "system": "vegetative",
            "visual_axis_id": axis_id,
            "visual_segments": [_segment("Leaf_r2_o0_rachis", 0.08, 0.003)],
            "parent": "Leaf_r2_o0_petiole",
            "attach_link": 1,
            "n_links": 4,
            "radius": 0.003,
            "height": 0.02,
            "tilt": 0.0,
            "rot": 0.0,
        },
        {
            "id": "Leaf_r2_o0_rachis_petiolule_lat_0_left",
            "system": "vegetative",
            "visual_axis_id": "Leaf_r2_o0_rachis_petiolule_lat_0_left",
            "visual_segments": [
                _segment("Leaf_r2_o0_rachis_petiolule_lat_0_left", 0.01, 0.002)
            ],
            "parent": "Leaf_r2_o0_rachis",
            "attach_link": 2,
            "attach_frac": 0.5,
            "n_links": 1,
            "radius": 0.002,
            "height": 0.01,
            "tilt": 75.0,
            "rot": 90.0,
        },
        {
            "id": "Leaf_r2_o0_rachis_petiolule_term",
            "system": "vegetative",
            "visual_axis_id": axis_id,
            "visual_segments": [
                _segment("Leaf_r2_o0_rachis_petiolule_term", 0.01, 0.002)
            ],
            "parent": "Leaf_r2_o0_rachis",
            "attach_link": 4,
            "n_links": 1,
            "radius": 0.002,
            "height": 0.01,
            "tilt": 0.0,
            "rot": 0.0,
        },
    ]


def _build(path, branches):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, "/World/Stem")
    build_skinned_vegetative_structure(
        stage,
        "/World/Stem",
        branches,
        all_branch_defs={branch["id"]: branch for branch in branches},
    )
    return stage


def _mesh_snapshot(stage, path):
    mesh = UsdGeom.Mesh(stage.GetPrimAtPath(path))
    assert mesh
    return (
        list(mesh.GetPointsAttr().Get()),
        list(mesh.GetFaceVertexCountsAttr().Get()),
        list(mesh.GetFaceVertexIndicesAttr().Get()),
    )


@pytest.mark.parametrize("day", (1, 40))
def test_csv_metadata_groups_leaf_axes_and_excludes_truss(day):
    from exporterV2.adapters.groimp_csv import parse_csv_to_branches

    branches, _, _ = parse_csv_to_branches(
        day=day,
        plant_id=1,
        include_terminal_bodies=True,
        save_json=False,
    )
    vegetative, truss = partition_branches(branches)
    assert all(branch.get("visual_axis_id") for branch in vegetative)
    assert all(branch.get("visual_segments") for branch in vegetative)
    assert all("visual_axis_id" not in branch for branch in truss)
    resolved = resolve_vegetative_graph(
        vegetative,
        all_branch_defs={branch["id"]: branch for branch in branches},
    )
    axes = build_visual_axes(resolved)
    assert {axis.axis_id for axis in axes} == {
        branch["visual_axis_id"] for branch in vegetative
    }

    leaf_axes = {}
    for branch in vegetative:
        axis_id = branch["visual_axis_id"]
        if axis_id.endswith("_axis"):
            leaf_axes.setdefault(axis_id, []).append(branch["id"])
    assert leaf_axes
    for members in leaf_axes.values():
        assert any(member.endswith("_petiole") for member in members)
        if any(member.endswith("_rachis") for member in members):
            assert any(member.endswith("_petiolule_term") for member in members)
        assert not any("_petiolule_lat_" in member for member in members)


def test_leaf_axis_authors_one_skeleton_with_flattened_physics_links(tmp_path):
    stage = _build(tmp_path / "leaf_axis.usda", _leaf_graph())
    axis_root = stage.GetPrimAtPath("/World/PlantVisual/Leaf_r2_o0_axis/SkelRoot")
    assert axis_root.IsA(UsdSkel.Root)
    assert axis_root.GetAttribute(VISUAL_AXIS_ID_ATTR).Get() == "Leaf_r2_o0_axis"
    assert axis_root.GetAttribute(SCHEMA_VERSION_ATTR).Get() == 2
    assert len(axis_root.GetRelationship(PHYSICS_LINKS_REL).GetTargets()) == 6
    assert not stage.GetPrimAtPath(
        "/World/PlantVisual/Leaf_r2_o0_petiole/SkelRoot"
    ).IsValid()
    assert stage.GetPrimAtPath(
        "/World/PlantVisual/Leaf_r2_o0_rachis_petiolule_lat_0_left/SkelRoot"
    ).IsA(UsdSkel.Root)

    stage.GetRootLayer().Save()
    runtime = SkinningRuntime.discover(Usd.Stage.Open(str(tmp_path / "leaf_axis.usda")))
    assert runtime.branch_count == 3
    runtime.sync()


def test_mesh_topology_is_identical_after_leaf_physics_merge(tmp_path):
    baseline = _leaf_graph()
    merged, report = LeafBranchReductionTechnique().apply(copy.deepcopy(baseline))
    assert report.joints_saved == 4
    merged_branch = next(
        branch for branch in merged if branch["id"] == "Leaf_r2_o0_merged"
    )
    assert [
        segment["source_id"] for segment in merged_branch["visual_segments"]
    ] == ["Leaf_r2_o0_petiole", "Leaf_r2_o0_rachis"]

    baseline_stage = _build(tmp_path / "baseline.usda", baseline)
    merged_stage = _build(tmp_path / "merged.usda", merged)
    mesh_path = "/World/PlantVisual/Leaf_r2_o0_axis/SkelRoot/BranchMesh"
    assert _mesh_snapshot(baseline_stage, mesh_path) == _mesh_snapshot(
        merged_stage,
        mesh_path,
    )

    baseline_root = baseline_stage.GetPrimAtPath(
        "/World/PlantVisual/Leaf_r2_o0_axis/SkelRoot"
    )
    merged_root = merged_stage.GetPrimAtPath(
        "/World/PlantVisual/Leaf_r2_o0_axis/SkelRoot"
    )
    assert len(baseline_root.GetRelationship(PHYSICS_LINKS_REL).GetTargets()) == 6
    assert len(merged_root.GetRelationship(PHYSICS_LINKS_REL).GetTargets()) == 2


def test_main_stem_mesh_is_identical_after_physics_collapse(tmp_path):
    baseline = _leaf_graph()
    baseline[0]["n_links"] = 10
    baseline[0]["height"] = 0.02
    baseline[0]["joint_type"] = "d6"
    baseline[1]["attach_link"] = 4
    baseline[1]["attach_frac"] = 0.25
    collapsed, _ = StemCollapseTechnique(target_segments=3).apply(
        copy.deepcopy(baseline)
    )

    baseline_stage = _build(tmp_path / "stem_baseline.usda", baseline)
    collapsed_stage = _build(tmp_path / "stem_collapsed.usda", collapsed)
    mesh_path = "/World/PlantVisual/trunk/SkelRoot/BranchMesh"
    assert _mesh_snapshot(baseline_stage, mesh_path) == _mesh_snapshot(
        collapsed_stage,
        mesh_path,
    )
    baseline_root = baseline_stage.GetPrimAtPath("/World/PlantVisual/trunk/SkelRoot")
    collapsed_root = collapsed_stage.GetPrimAtPath("/World/PlantVisual/trunk/SkelRoot")
    assert len(baseline_root.GetRelationship(PHYSICS_LINKS_REL).GetTargets()) == 10
    assert len(collapsed_root.GetRelationship(PHYSICS_LINKS_REL).GetTargets()) == 3


def test_explicit_axis_rejects_non_tip_continuation():
    branches = _leaf_graph()
    rachis = next(branch for branch in branches if branch["id"].endswith("_rachis"))
    rachis["attach_frac"] = 0.5
    resolved = resolve_vegetative_graph(
        branches,
        all_branch_defs={branch["id"]: branch for branch in branches},
    )
    with pytest.raises(ValueError, match="attach_frac is 0.5"):
        build_visual_axes(resolved)


def test_stem_collapse_preserves_existing_fractional_attachment():
    branches = _leaf_graph()
    trunk = branches[0]
    trunk["n_links"] = 10
    trunk["height"] = 0.02
    trunk["joint_type"] = "d6"
    child = branches[1]
    child["attach_link"] = 4
    child["attach_frac"] = 0.25

    modified, _ = StemCollapseTechnique(target_segments=3).apply(branches)
    remapped = next(branch for branch in modified if branch["id"] == child["id"])
    old_fraction = (4 - 1 + 0.25) / 10
    new_fraction = (
        remapped["attach_link"] - 1 + remapped["attach_frac"]
    ) / 3
    assert math.isclose(new_fraction, old_fraction, abs_tol=1e-12)


def test_runtime_discovers_schema_v1_branch_name(tmp_path):
    stage = _build(tmp_path / "schema_v1.usda", _leaf_graph())
    root = stage.GetPrimAtPath("/World/PlantVisual/Leaf_r2_o0_axis/SkelRoot")
    root.RemoveProperty(VISUAL_AXIS_ID_ATTR)
    root.GetAttribute(BRANCH_ID_ATTR).Set("legacy_leaf_branch")
    runtime = SkinningRuntime.discover(stage)
    assert any(branch.name == "legacy_leaf_branch" for branch in runtime.branches)
