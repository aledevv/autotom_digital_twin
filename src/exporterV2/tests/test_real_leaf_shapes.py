"""Real generators, reproducible shape identity, and unchanged plant mechanics."""
from dataclasses import replace
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

from exporterV2.leaf_shapes.config import load_leaf_shape_config, PROJECT_ROOT
from exporterV2.leaf_shapes.client import prepare_leaf_shapes, leaflet_seed
from exporterV2.adapters.groimp import plant_state_to_skeleton
from exporterV2.config import physics_config_from_dict, visuals_config_from_dict
from exporterV2.exporter import export_plant
from plant_state import load_plant_state


def _day10():
    return plant_state_to_skeleton(load_plant_state(PROJECT_ROOT / "data/plant_states/plant_state_day_10.json"))


def test_config_precedence_and_errors(tmp_path):
    path = tmp_path / "leaf.yaml"
    path.write_text("backend: i3\nseed: 7\ngenerator:\n  neighbours: 6\n")
    config = load_leaf_shape_config(path, backend="gaussian", seed=11)
    assert (config.backend, config.seed, config.generator["neighbours"]) == ("gaussian", 11, 6)
    assert config.generator["model"] == "gaussian_efd_d16"
    assert load_leaf_shape_config().backend == "gaussian"
    for field in ({"backend": "unknown"}, {"seed": -1}, {"seed": True}, {"timeout_seconds": 0}):
        with pytest.raises(ValueError):
            replace(config, **field)
    path.write_text("generator: []")
    with pytest.raises(ValueError, match="mapping"):
        load_leaf_shape_config(path)
    path.write_text("backend: [")
    with pytest.raises(ValueError, match="YAML"):
        load_leaf_shape_config(path)


@pytest.mark.parametrize("backend", ["gaussian", "i3"])
def test_generators_reproduce_individual_shapes_across_processes(backend):
    leaves = [(f"axis-{i}", role) for i, role in enumerate(("left", "left", "right", "terminal"))]
    config = load_leaf_shape_config(backend=backend)
    first = prepare_leaf_shapes(leaves, plant_id=1, config=config)
    second = prepare_leaf_shapes(list(reversed(leaves)), plant_id=1, config=config)
    different = prepare_leaf_shapes(leaves, plant_id=1, config=replace(config, seed=43))
    assert first.shapes == second.shapes
    assert len({shape["contour_sha256"] for shape in first.shapes.values()}) == len(leaves)
    for key, shape in first.shapes.items():
        assert shape["contour_sha256"] != different.shapes[key]["contour_sha256"]
        assert len(shape["vertices"]) == 1024
        assert len(shape["triangles"]) == 1022
        assert shape["vertices"] == shape["contour_xy"]
        assert shape["base"] == [0, 0] and shape["tip"] == [0, 1]
        assert shape["provenance"]["role"] == shape["role"]
        assert shape["provenance"]["seed"] == shape["seed"]
    assert first.runtime["python"].startswith("3.12.")


def test_seed_is_structural_not_backend_dependent():
    seeds = {leaflet_seed(42, 1, f"axis-{i}", "left") for i in range(1000)}
    assert len(seeds) == 1000
    assert seeds.isdisjoint({leaflet_seed(43, 1, f"axis-{i}", "left") for i in range(1000)})
    assert leaflet_seed(42, 1, "axis", "left") != leaflet_seed(42, 1, "axis", "right")


def test_legacy_does_not_start_worker(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError("worker invoked")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    batch = prepare_leaf_shapes([("a", "left")], plant_id=1, config=load_leaf_shape_config(backend="legacy"))
    assert batch.for_leaf("a")["backend"] == "legacy"
    with pytest.raises(ValueError, match="no fallback"):
        batch.for_leaf("missing")


def test_missing_model_fails_without_fallback(tmp_path, monkeypatch):
    config = load_leaf_shape_config()
    config = replace(config, generator={**config.generator, "model": str(tmp_path/"absent")})
    with pytest.raises(ValueError, match="gaussian leaf generation failed"):
        prepare_leaf_shapes([("a", "left")], plant_id=1, config=config)
    destination = tmp_path/"existing.usda"
    destination.write_text("preserve previous export")
    from exporterV2.leaf_shapes import config as leaf_config
    monkeypatch.setattr(leaf_config, "load_leaf_shape_config", lambda **_: config)
    with pytest.raises(ValueError, match="gaussian leaf generation failed"):
        export_plant(_day10(), destination, physics_config_from_dict(), visuals_config_from_dict())
    assert destination.read_text() == "preserve previous export"


def test_timeout_stops_worker(monkeypatch):
    from exporterV2.leaf_shapes import client
    class HungWorker:
        def wait(self, timeout):
            raise subprocess.TimeoutExpired("worker", timeout)
    process = HungWorker()
    stopped = []
    monkeypatch.setattr(subprocess, "Popen", lambda *a, **kw: process)
    monkeypatch.setattr(client, "_stop_worker", lambda p: stopped.append(p))
    with pytest.raises(ValueError, match="timed out"):
        prepare_leaf_shapes([("a", "left")], plant_id=1)
    assert stopped == [process]


def structural_signature(stage):
    result = {}
    for prim in stage.Traverse():
        if prim.GetName().startswith("LeafBlade"):
            continue
        result[str(prim.GetPath())] = (
            prim.GetTypeName(), tuple(prim.GetAppliedSchemas()),
            {a.GetName(): repr(a.Get()) for a in prim.GetAttributes() if a.GetName() != "autotom:leafShapes"},
            {r.GetName(): [str(p) for p in r.GetTargets()] for r in prim.GetRelationships()},
        )
    return result


@pytest.fixture(scope="module")
def day10_exports(tmp_path_factory):
    skeleton = _day10()
    folder = tmp_path_factory.mktemp("leaf-plants")
    result = {}
    for backend, generator in (("legacy", "simple"), ("gaussian", "real_leaves"), ("i3", "real_leaves_i3")):
        export = export_plant(
            skeleton, folder/f"{backend}.usda", physics_config_from_dict(),
            visuals_config_from_dict({"leaves": {"generator": generator}}),
        )
        result[backend] = (Usd.Stage.Open(str(export.usd_path)), json.loads(export.manifest_path.read_text()))
    return result


def test_backends_only_change_blades(day10_exports):
    baseline = structural_signature(day10_exports["legacy"][0])
    for backend in ("gaussian", "i3"):
        stage, manifest = day10_exports[backend]
        assert structural_signature(stage) == baseline
        assert not manifest["errors"]
        shapes = manifest["metadata"]["leaf_shapes"]["leaves"]
        assert len({s["seed"] for s in shapes}) == len(shapes)
        assert len({s["contour_sha256"] for s in shapes}) == len(shapes)
        assert {s["role"] for s in shapes} == {"left", "right", "terminal"}
    assert [s["seed"] for s in day10_exports["gaussian"][1]["metadata"]["leaf_shapes"]["leaves"]] == [s["seed"] for s in day10_exports["i3"][1]["metadata"]["leaf_shapes"]["leaves"]]


@pytest.mark.parametrize("backend", ["gaussian", "i3"])
def test_generated_blades_attach_and_keep_material(day10_exports, backend):
    stage, manifest = day10_exports[backend]
    for record in manifest["topology"]["leaflets"]:
        root = stage.GetPrimAtPath(record["root_path"])
        prim = root.GetChild("LeafBlade")
        mesh = UsdGeom.Mesh(prim)
        points = np.asarray(mesh.GetPointsAttr().Get())
        faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        assert len(points) == 1024 and len(faces) == 1022
        assert np.isfinite(points).all()
        areas = np.linalg.norm(np.cross(points[faces[:, 1]]-points[faces[:, 0]], points[faces[:, 2]]-points[faces[:, 0]]), axis=1)
        assert (areas > 0).all()
        actual = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(0).Transform(Gf.Vec3d(prim.GetAttribute("autotom:leafShapeBase").Get()))
        frame = record["rest_frame"]
        start = Gf.Vec3d(*(frame[row][3]*2 for row in range(3)))
        head = Gf.Vec3d(*(frame[row][2] for row in range(3))).GetNormalized()
        assert (actual - (start + head*record["length"]*2)).GetLength() < 1e-8
        assert UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterial()
        assert not prim.HasAPI(UsdPhysics.RigidBodyAPI) and not prim.HasAPI(UsdPhysics.CollisionAPI)
        assert prim.GetAttribute("autotom:leafletRole").Get() == record["leaflet_role"]
