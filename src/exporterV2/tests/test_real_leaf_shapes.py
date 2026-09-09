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
from exporterV2.plant_state_legacy_backend import export_incremental_checkpoint
from plant_state import load_plant_state


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


def test_missing_model_fails_without_fallback(tmp_path):
    config = load_leaf_shape_config()
    config = replace(config, generator={**config.generator, "model": str(tmp_path/"absent")})
    with pytest.raises(ValueError, match="gaussian leaf generation failed"):
        prepare_leaf_shapes([("a", "left")], plant_id=1, config=config)
    state = load_plant_state(PROJECT_ROOT / "data/plant_states/plant_state_day_10.json")
    destination = tmp_path/"existing.usda"
    destination.write_text("preserve previous export")
    with pytest.raises(ValueError, match="gaussian leaf generation failed"):
        export_incremental_checkpoint(state, destination, debug_profile="leaves", leaf_shape_config=config)
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
    state = load_plant_state(PROJECT_ROOT / "data/plant_states/plant_state_day_10.json")
    folder = tmp_path_factory.mktemp("leaf-plants")
    result = {}
    for backend in ("legacy", "gaussian", "i3"):
        _, usd, manifest = export_incremental_checkpoint(
            state, folder/f"{backend}.usda", debug_profile="leaves",
            leaf_shape_config=load_leaf_shape_config(backend=backend),
        )
        result[backend] = (Usd.Stage.Open(str(usd)), json.loads(manifest.read_text()))
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
    for record in manifest["topology"]["rigid_leaf_visuals"]:
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


@pytest.fixture(scope="module")
def physical_leaf_case():
    from exporterV2.plant_state_branches import build_leaf_branches
    state = load_plant_state(PROJECT_ROOT / "data/plant_states/plant_state_day_10.json")
    adapter = build_leaf_branches(state, physical_petiolules=True, leaf_joint_policy="optimized")
    shapes = prepare_leaf_shapes([(r["leaflet_id"], r["leaflet_role"]) for r in adapter.rigid_leaf_visuals], plant_id=1)
    return adapter, shapes


@pytest.mark.parametrize("mode", ["segmented", "static", "skinned", "rigid-single"])
def test_physical_petiolules_receive_same_blades_in_visual_modes(tmp_path, physical_leaf_case, mode):
    from exporterV2.core.usd.stage import build_stage
    adapter, shapes = physical_leaf_case
    stage, _ = build_stage(str(tmp_path/f"{mode}.usda"), branches=list(adapter.branches),
                           branch_backend="skinned", skinning_visual_mode=mode, leaf_shapes=shapes)
    blades = [p for p in stage.Traverse() if p.GetName() == "LeafBlade"]
    assert len(blades) == len(shapes.shapes)
    for prim in blades:
        identity = prim.GetAttribute("autotom:leafShapeStructuralId").Get()
        shape = shapes.for_leaf(identity)
        assert prim.GetAttribute("autotom:leafShapeSeed").Get() == shape["seed"]
        assert prim.GetAttribute("autotom:leafletRole").Get() == shape["role"]
        assert len(UsdGeom.Mesh(prim).GetPointsAttr().Get()) == 1024


def test_direct_plant_state_exporter_uses_same_generator(tmp_path):
    from exporterV2.plant_state_adapter import build_v2_authoring_plan
    from exporterV2.plant_state_usd import export_plant_state_v2, audit_v2_stage
    state = load_plant_state(PROJECT_ROOT / "data/plant_states/plant_state_day_10.json")
    plan = build_v2_authoring_plan(state, physics_preset="locked", debug_profile="leaves")
    path = export_plant_state_v2(plan, tmp_path/"direct.usda")
    manifest = audit_v2_stage(plan, path)
    assert not manifest.errors
    assert manifest.metadata["leaf_shapes"]["config"]["backend"] == "gaussian"
    stage = Usd.Stage.Open(str(path))
    blades = [p for p in stage.Traverse() if p.GetName().startswith("LeafBlade")]
    assert len(blades) == 23
    assert all(len(UsdGeom.Mesh(p).GetPointsAttr().Get()) == 1024 for p in blades)
