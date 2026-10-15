"""End-to-end exports. Leaves use the ``simple`` generator so no worker starts."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re

import pytest
from pxr import Usd, UsdGeom, UsdPhysics

from exporterV2 import cli
from exporterV2.config import physics_config_from_dict, visuals_config_from_dict
from exporterV2.exporter import ExportError, export_plant
from plant_skeleton import load_skeleton


PROJECT_ROOT = Path(__file__).resolve().parents[3]
SMALL_PLANT = PROJECT_ROOT / "input" / "examples" / "small_plant.json"
GROIMP_DAY_160 = PROJECT_ROOT / "input" / "skeleton.json"
REFERENCE = Path(__file__).parent / "data" / "groimp_day160_reference.json"
SIMPLE_LEAVES = {"leaves": {"generator": "simple"}}


def _export(tmp_path, skeleton_path=SMALL_PLANT, physics=None, visuals=None, name="plant.usda"):
    return export_plant(
        load_skeleton(skeleton_path),
        tmp_path / name,
        physics_config_from_dict(physics),
        visuals_config_from_dict({**SIMPLE_LEAVES, **(visuals or {})}),
    )


def test_small_hand_written_plant(tmp_path):
    result = _export(tmp_path)
    authored = result.manifest["authored"]
    assert result.manifest["errors"] == []
    assert authored["rigid_bodies"] == 16
    assert authored["fixed_joints"] == 4  # the stem
    assert authored["d6_joints"] == 12
    assert authored["leaflets"] == authored["leaf_blades"] == 6
    assert authored["fruit_spheres"] == 3
    assert authored["terminal_forks"] == 1  # leaf B ends the stem
    stage = Usd.Stage.Open(str(result.usd_path))
    stem = stage.GetPrimAtPath("/World/Stem")
    assert stem.GetAttribute("autotom:skeletonSchema").Get() == "plant_skeleton/1.0"
    assert stem.GetAttribute("autotom:fruitMode").Get() == "visual"
    assert stem.GetAttribute("autotom:globalScale").Get() == 2.0
    nodes = {prim.GetAttribute("autotom:skeletonNodeId").Get() for prim in stage.Traverse()}
    assert {"stem1", "shoot2", "pedicel3"} <= nodes


def test_welded_stem_is_one_continuous_mesh(tmp_path):
    result = _export(tmp_path)
    assert result.manifest["authored"]["organic_meshes"] == 16 - 3  # 4 stem links -> 1 mesh
    stage = Usd.Stage.Open(str(result.usd_path))
    stem_meshes = [prim for prim in stage.Traverse() if "/trunk/" in str(prim.GetPath()) and prim.GetName().startswith("OrganicVisual_")]
    assert [prim.GetName() for prim in stem_meshes] == ["OrganicVisual_01"]


def test_bending_stem_keeps_one_mesh_per_link(tmp_path):
    result = _export(tmp_path, physics={"segments": {"stem": {"joint": "d6"}}})
    authored = result.manifest["authored"]
    assert result.manifest["errors"] == []
    assert authored["d6_joints"] == 12 + 3
    assert authored["organic_meshes"] == 16


def test_physical_fruits_become_rigid_bodies(tmp_path):
    result = _export(tmp_path, physics={"fruits": {"mode": "physical"}})
    assert result.manifest["authored"]["rigid_bodies"] == 16 + 3


def test_global_scale_changes_every_length(tmp_path):
    two = _export(tmp_path, name="two.usda")
    three = _export(tmp_path, physics={"global_scale": 3.0}, name="three.usda")
    for a, b in zip(two.manifest["poses"], three.manifest["poses"]):
        assert b["length"] == pytest.approx(a["length"] * 1.5)
        assert b["start"] == pytest.approx([value * 1.5 for value in a["start"]])


def test_too_many_joints_asks_for_optimization(tmp_path):
    with pytest.raises(ExportError, match="joints.optimize"):
        _export(tmp_path, physics={"joints": {"joint_target": 5, "joint_max": 8}})
    result = _export(tmp_path, physics={"joints": {"joint_target": 8, "joint_max": 8, "optimize": "full"}})
    assert result.manifest["authored"]["post_merge"]["d6_joints"] <= 8


def test_cli_defaults_and_errors(tmp_path, capsys):
    output = tmp_path / "cli.usda"
    visuals = tmp_path / "visuals.yaml"
    visuals.write_text("leaves:\n  generator: simple\n")
    assert cli.main(["--input", str(SMALL_PLANT), "--visuals", str(visuals), "--output", str(output)]) == 0
    assert output.is_file() and output.with_suffix(".manifest.json").is_file()
    assert "16 segments -> 16 rigid bodies" in capsys.readouterr().out
    assert cli.build_argument_parser().parse_args([]).input == PROJECT_ROOT / "input" / "skeleton.json"
    broken = tmp_path / "broken.json"
    broken.write_text('{"schema_version": "plant_skeleton/1.0", "nodes": []}')
    assert cli.main(["--input", str(broken), "--output", str(output)]) == 1
    assert "has no nodes" in capsys.readouterr().err


@pytest.fixture(scope="module")
def day160(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("day160")
    return _export(tmp, GROIMP_DAY_160, physics={"fruits": {"mode": "none"}})


def test_groimp_day_160_matches_the_old_exporter(day160):
    """Same rigid links, poses, masses and joint gains as the PlantState exporter."""

    reference = json.loads(REFERENCE.read_text())
    authored = day160.manifest["authored"]
    expected_counts = reference["counts"]
    assert authored["rigid_bodies"] == expected_counts["rigid_bodies"]
    assert authored["fixed_joints"] == expected_counts["fixed_joints"]
    assert authored["d6_joints"] == expected_counts["d6_joints"]
    assert authored["capsule_colliders"] == expected_counts["capsule_colliders"]
    # The welded stem is now one continuous mesh instead of one per link.
    stem_links = sum("/trunk/" in pose["body_path"] for pose in day160.manifest["poses"])
    merged = stem_links - 1
    assert authored["organic_meshes"] == expected_counts["organic_meshes"] - merged
    assert authored["leaf_blades"] == expected_counts["leaf_blades"]
    assert authored["petiolule_meshes"] == expected_counts["petiolule_visual_meshes"]
    assert authored["terminal_forks"] == expected_counts["terminal_fork_shoots"]
    assert authored["total_meshes"] == expected_counts["total_meshes"] - merged
    assert authored["truss_visuals"] == reference["truss_visuals"]

    poses = sorted(pose["start"] + pose["direction"] for pose in day160.manifest["poses"])
    for ours, theirs in zip(poses, reference["links_start_direction"]):
        assert ours == pytest.approx(theirs, abs=1e-8)

    masses = sorted(body["mass_kg"] for body in day160.manifest["physics"]["body_masses"])
    # USD stores masses as 32-bit floats.
    assert masses == pytest.approx(reference["body_masses_kg"], rel=1e-6)

    usda = day160.usd_path.read_text()
    stiffness = sorted(float(x) for x in re.findall(r"drive:rot[XY]:physics:stiffness = ([0-9.e+-]+)", usda))
    damping = sorted(float(x) for x in re.findall(r"drive:rot[XY]:physics:damping = ([0-9.e+-]+)", usda))
    assert stiffness == pytest.approx(reference["drive_stiffness"], rel=1e-6)
    assert damping == pytest.approx(reference["drive_damping"], rel=1e-6)


def test_leaflets_hang_from_their_rachis(day160):
    stage = Usd.Stage.Open(str(day160.usd_path))
    for prim in stage.Traverse():
        if prim.GetAttribute("autotom:entityKind").Get() != "rigid_leaf_visual":
            continue
        assert prim.GetParent().GetParent().HasAPI(UsdPhysics.RigidBodyAPI)
        assert prim.GetChild("LeafBlade").IsA(UsdGeom.Mesh)


@pytest.mark.parametrize(
    "visuals",
    [
        {"stems": {"junction_bulge": False}},
        {"stems": {"terminal_forks": False}},
        {"pedicels": {"gravity_curve": False}},
        {"quality": "performance"},
        {"pose": "canonical"},
    ],
)
def test_every_visual_switch_exports(tmp_path, visuals):
    result = _export(tmp_path, visuals=visuals)
    assert result.manifest["errors"] == []
    assert result.manifest["authored"]["rigid_bodies"] == 16


def test_fruit_size_and_calyx_are_visual_only(tmp_path):
    def spheres(result):
        stage = Usd.Stage.Open(str(result.usd_path))
        return {
            prim.GetParent().GetName(): (UsdGeom.Sphere(prim).GetRadiusAttr().Get(), prim.GetParent().GetChild("Calyx").IsValid())
            for prim in stage.Traverse()
            if prim.IsA(UsdGeom.Sphere)
        }

    physical = {"fruits": {"mode": "physical"}}
    default = _export(tmp_path, physics=physical, name="default.usda")
    true_size = _export(tmp_path, physics=physical, visuals={"fruits": {"size_scale": 1.0, "calyx": False}}, name="true.usda")
    assert default.manifest["authored"]["calyces"] == 3
    assert true_size.manifest["authored"]["calyces"] == 0
    for name, (radius, has_calyx) in spheres(default).items():
        full_radius, full_calyx = spheres(true_size)[name]
        assert radius == pytest.approx(full_radius * 0.5)
        assert has_calyx and not full_calyx
    masses = lambda result: sorted(body["mass_kg"] for body in result.manifest["physics"]["body_masses"])
    assert masses(default) == pytest.approx(masses(true_size))
