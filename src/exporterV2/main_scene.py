"""Main scene: day-160 tomato with the five original main-stem trusses.

Small tomatoes with calyces, slender pedicels, continuous stem.
It is rebuilt from the exporter in four steps:

1. export the day-160 PlantState with the ``main-rank6-standard`` scaffold
   (``experimental_fixture='full'``) to get the plant and the truss
   attachment frames on the stem;
2. graft: replace the scaffold trusses with the five tested trusses stored in
   ``assets/thesis_trusses/five_trusses.usda`` (ranks 6-10, 20 bodies each);
   lateral trusses are removed, vegetation is untouched;
3. slender trusses (visual only): thin rachis cylinders and tapered pedicels;
4. calyces and thinner pedicels (visual only, photo-inspired, not measured).

The welded trunk is one continuous mesh (core/skinning/visual_segmented.py).
Physics of the grafted trusses is copied, not recomputed: masses, inertias,
drives and colliders are the tested ones. Ported from the thesis experiments
``src/experiments/detachable_fruit_v2/{build_v23_graft_scaffold,
graft_original_trusses, slender_truss_visuals, natural_truss_visuals}.py``
on branch ``thesis-archive``.

Usage: ``uv run python -m exporterV2.main_scene [--output DIR]``, or
``./run_mainV2.sh`` (no arguments) to build and open it in Isaac Sim.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / "assets" / "thesis_trusses"
DAY160 = ROOT / "data" / "plant_states" / "plant_state_day_160.json"
TRUSS_RANKS = range(6, 11)
BODIES_PER_TRUSS = 20

# Visual-only proportions (photo-inspired, not measured GroIMP properties).
PEDICEL_TIP_TAPER = 0.8  # pedicel tip radius / base radius
PEDICEL_THINNING = 0.8  # extra thinning: 3 mm -> 2.4 mm at base
CALYX_COLOR = (0.12, 0.24, 0.045)


def export_scaffold(output: Path, plant_state: Path = DAY160) -> Path:
    """Step 1: export the plant with standard trusses as placeholders."""

    from plant_state import load_plant_state

    from .plant_state_legacy_backend import export_incremental_checkpoint

    _, path, _ = export_incremental_checkpoint(
        load_plant_state(plant_state),
        output,
        debug_profile="full",
        physics_hz=60,
        allow_experimental_fruit_physics=True,
        experimental_truss_preset="main-rank6-standard",
        experimental_fixture="full",
    )
    return Path(path)


def _vegetation_attributes(stage) -> dict[str, str]:
    return {
        str(a.GetPath()): str(a.Get())
        for p in stage.Traverse()
        if "/Vegetative/" in str(p.GetPath())
        for a in p.GetAttributes()
    }


def graft_trusses(scaffold: Path) -> tuple[Usd.Stage, dict]:
    """Step 2: swap scaffold trusses for the five stored ones; drop lateral trusses."""

    from .fruit_experiments import audit
    from .standard_truss import upright_attachment

    source = Usd.Stage.Open(str(ASSETS / "five_trusses.usda"))
    stage = Usd.Stage.Open(Usd.Stage.Open(str(scaffold)).Flatten())
    assert UsdGeom.GetStageMetersPerUnit(source) == UsdGeom.GetStageMetersPerUnit(stage)
    assert UsdGeom.GetStageUpAxis(source) == UsdGeom.GetStageUpAxis(stage)
    before = _vegetation_attributes(stage)
    cache = UsdGeom.XformCache()
    selected, old_metadata = [], {}
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not prim.GetAttribute("autotom:standardTruss").Get():
            continue
        old_metadata[path] = {
            x.GetName(): x.Get() for x in prim.GetAttributes() if x.GetName().startswith("autotom:")
        }
        if path.endswith("_rachis_Link_01"):
            joint = UsdPhysics.Joint(stage.GetPrimAtPath(path + "/AttachJoint"))
            parent = str(joint.GetBody0Rel().GetTargets()[0])
            if "/Vegetative/trunk/" in parent:  # main-stem trusses only
                rank = int(re.search(r"Truss_r(\d+)_", path)[1])
                selected.append(dict(rank=rank, path=path, parent=parent, frame=cache.GetLocalToWorldTransform(prim)))
    assert sorted(x["rank"] for x in selected) == list(TRUSS_RANKS)
    for path in old_metadata:
        stage.RemovePrim(path)
    for scope in ("Looks", "Materials"):
        if source.GetPrimAtPath("/World/" + scope):
            Sdf.CopySpec(source.GetRootLayer(), "/World/" + scope, stage.GetRootLayer(), "/World/OriginalTruss" + scope)

    mass_records = json.loads((ASSETS / "five_trusses_mass.json").read_text())["bodies"]
    expected = {}
    for item in selected:
        rank = item["rank"]
        prefix = f"Truss_r{rank}_o0"
        new_prefix = item["path"].split("/")[-1].removesuffix("_rachis_Link_01")
        bodies = [b for b in mass_records if prefix + "_" in b["path"]]
        assert len(bodies) == BODIES_PER_TRUSS
        mapping = {b["path"]: b["path"].replace(prefix, new_prefix) for b in bodies}
        root = f"/World/Stem/{prefix}_rachis_Link_01"
        source_frame = cache.GetLocalToWorldTransform(source.GetPrimAtPath(root))
        transform = source_frame.GetInverse() * upright_attachment(source_frame, item["frame"])
        for b in bodies:
            old, new = b["path"], mapping[b["path"]]
            UsdGeom.Xform.Define(stage, str(Sdf.Path(new).GetParentPath()))
            Sdf.CopySpec(source.GetRootLayer(), old, stage.GetRootLayer(), new)
            prim = stage.GetPrimAtPath(new)
            matrix = cache.GetLocalToWorldTransform(source.GetPrimAtPath(old)) * transform
            prim.GetAttribute("xformOp:translate").Set(matrix.ExtractTranslation())
            prim.GetAttribute("xformOp:orient").Set(Gf.Quatf(matrix.ExtractRotationQuat()))
            for child in Usd.PrimRange(prim):
                for rel in child.GetRelationships():
                    targets = []
                    for target in rel.GetTargets():
                        text = str(target)
                        match = next((k for k in mapping if text == k or text.startswith(k + "/")), None)
                        if old == root and rel.GetName() == "physics:body0":
                            new_target = item["parent"]
                        elif match:
                            new_target = mapping[match] + text[len(match):]
                        elif text.startswith("/World/Looks/"):
                            new_target = text.replace("/World/Looks/", "/World/OriginalTrussLooks/")
                        elif text.startswith("/World/Materials/"):
                            new_target = text.replace("/World/Materials/", "/World/OriginalTrussMaterials/")
                        else:
                            new_target = text
                        if stage.GetPrimAtPath(Sdf.Path(new_target).GetPrimPath()) or match:
                            targets.append(Sdf.Path(new_target))
                    rel.SetTargets(targets)
            for name, value in old_metadata[new].items():
                if isinstance(value, str):
                    prim.CreateAttribute(name, Sdf.ValueTypeNames.String).Set(value)
            prim.CreateAttribute("autotom:standardTruss", Sdf.ValueTypeNames.String).Set("main-original-ranks-experiment")
            prim.CreateAttribute("autotom:role", Sdf.ValueTypeNames.String).Set(b["role"])
            # Masses measured by PhysX on the stored trusses, made explicit.
            mass = UsdPhysics.MassAPI(prim)
            inertia = np.asarray(b["inertia"]).reshape(3, 3)
            assert np.allclose(inertia, np.diag(np.diag(inertia)), atol=1e-16)
            mass.CreateMassAttr().Set(b["mass_kg"])
            mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*np.diag(inertia)))
            mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(*np.asarray(b["com"]).reshape(3)))
            q = np.asarray(b["com_orientation"]).reshape(4)
            mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(q[0]), Gf.Vec3f(*q[1:])))
            expected[new] = {k: b[k] for k in ("mass_kg", "inertia", "com", "com_orientation")}
        # Re-express the root attachment in the new parent stem link.
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(mapping[root] + "/AttachJoint"))
        local1 = Gf.Matrix4d().SetRotate(joint.GetLocalRot1Attr().Get())
        local1.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
        current = UsdGeom.XformCache()
        local0 = (
            local1
            * current.GetLocalToWorldTransform(stage.GetPrimAtPath(mapping[root]))
            * current.GetLocalToWorldTransform(stage.GetPrimAtPath(item["parent"])).GetInverse()
        )
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local0.ExtractTranslation()))
        joint.GetLocalRot0Attr().Set(Gf.Quatf(local0.ExtractRotationQuat()))
    # Collision filters that pointed at removed scaffold bodies.
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            old_targets = rel.GetTargets()
            kept = [x for x in old_targets if stage.GetPrimAtPath(x.GetPrimPath())]
            if old_targets != kept:
                assert "filter" in rel.GetName().lower() or "collection" in rel.GetName().lower(), str(rel)
                rel.SetTargets(kept)
    assert _vegetation_attributes(stage) == before, "vegetation attributes changed"
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    scene.GetAttribute("physxScene:solverType").Set("PGS")
    scene.GetAttribute("physxScene:enableGPUDynamics").Set(False)
    stage.GetPrimAtPath("/World/Stem").GetAttribute("autotom:experimentalTrussPreset").Set("main-original-ranks-experiment")
    check = audit(stage)
    assert not check["errors"], check["errors"]
    return stage, expected


def slender_trusses(stage, plant_state: Path = DAY160) -> int:
    """Step 3 (visual only): GroIMP radii for rachis visuals, tapered pedicels.

    The original rachis cylinders stay as invisible colliders; a thinner
    visual cylinder is added next to each one.
    """

    data = json.loads(plant_state.read_text())
    changes = 0
    for body in list(stage.Traverse()):
        match = re.search(r"Truss_r\d+_o0_g(\d+)_rachis_", str(body.GetPath()))
        if not match or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        cylinder = UsdGeom.Cylinder(stage.GetPrimAtPath(str(body.GetPath()) + "/Cylinder"))
        if not cylinder:
            continue
        pedicel = "pedicel" in body.GetName()
        role = "pedicel" if pedicel else "truss_rachis"
        radii = [a["radius"] for a in data["axes"] if a["owner_node_id"] == "node:" + match[1] and a["role"] == role]
        assert radii and max(radii) - min(radii) < 1e-10
        radius = radii[0]
        if pedicel:
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(str(body.GetPath()) + "/GravityElbowPedicelVisual"))
            assert mesh and not mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            assert (len(points) - 2) % 14 == 0  # 14 vertices per ring, two caps
            rings = points[:-2].reshape(-1, 14, 3)
            centers = rings.mean(axis=1, keepdims=True)
            offsets = rings - centers
            lengths = np.linalg.norm(offsets, axis=2, keepdims=True)
            target = np.linspace(radius, radius * PEDICEL_TIP_TAPER, len(rings))[:, None, None]
            rings[:] = centers + offsets / lengths * target
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
            mesh.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))]))
        else:
            cylinder.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            visual = UsdGeom.Cylinder.Define(stage, str(body.GetPath()) + "/SlenderRachisVisual")
            for attr in cylinder.GetPrim().GetAttributes():
                if attr.GetName().startswith(("physics:", "physx")) or attr.GetName() in ("visibility", "extent"):
                    continue
                value = attr.Get()
                if value is not None:
                    visual.GetPrim().CreateAttribute(attr.GetName(), attr.GetTypeName()).Set(value)
            visual.GetRadiusAttr().Set(radius)
        changes += 1
    assert changes == len(TRUSS_RANKS) * 12, changes  # 4 rachis + 8 pedicels per truss
    return changes


def calyx_points(radius, attachment):
    """Five tapered sepals around the pedicel attachment of a sphere."""

    axis = np.asarray(attachment, dtype=float)
    axis /= np.linalg.norm(axis)
    helper = np.eye(3)[np.argmin(np.abs(axis))]
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    points, counts, indices = [], [], []
    for blade in range(5):
        theta = 2 * math.pi * blade / 5
        radial = math.cos(theta) * u + math.sin(theta) * v
        side = np.cross(axis, radial)
        start = len(points)
        for row in range(13):
            t = row / 12
            distance = radius * (0.055 + 0.70 * t)
            width = radius * (0.09 + 0.10 * math.sin(math.pi * t)) * (1 - t) + radius * 0.001
            for across in (-1, 0, 1):
                offset = side * width * across + radial * distance
                height = math.sqrt(radius**2 - float(offset @ offset))
                height += radius * (0.012 + 0.065 * t**5 + (0.018 if across == 0 else 0))
                points.append(offset + axis * height)
            if row:
                previous = start + (row - 1) * 3
                for col in range(2):
                    indices.extend([previous + col, previous + col + 1, previous + col + 4, previous + col + 3])
                    counts.append(4)
    hub = len(points)
    points.append(axis * radius * 1.045)
    for i in range(20):
        theta = i * math.tau / 20
        points.append(axis * radius * 1.012 + radius * 0.16 * (math.cos(theta) * u + math.sin(theta) * v))
    for i in range(20):
        indices.extend([hub, hub + 1 + i, hub + 1 + (i + 1) % 20])
        counts.append(3)
    return np.asarray(points), counts, indices


def calyces_and_thin_pedicels(stage) -> int:
    """Step 4 (visual only): one calyx per fruit, pedicels thinned to 2.4 mm."""

    fruits = pedicels = 0
    for prim in list(stage.Traverse()):
        if prim.GetName() == "GravityElbowPedicelVisual":
            mesh = UsdGeom.Mesh(prim)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            rings = points[:-2].reshape(-1, 14, 3)
            centers = rings.mean(axis=1, keepdims=True)
            rings[:] = centers + PEDICEL_THINNING * (rings - centers)
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
            mesh.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))]))
            pedicels += 1
        if prim.GetAttribute("autotom:role").Get() != "fruit":
            continue
        sphere = UsdGeom.Sphere(stage.GetPrimAtPath(str(prim.GetPath()) + "/Sphere"))
        joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(str(prim.GetPath()) + "/TerminalBodyFixedJoint"))
        points, counts, indices = calyx_points(sphere.GetRadiusAttr().Get(), joint.GetLocalPos1Attr().Get())
        mesh = UsdGeom.Mesh.Define(stage, str(prim.GetPath()) + "/CalyxVisual")
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        mesh.CreateFaceVertexCountsAttr().Set(counts)
        mesh.CreateFaceVertexIndicesAttr().Set(indices)
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(*CALYX_COLOR)])
        mesh.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))]))
        fruits += 1
    assert fruits == pedicels == len(TRUSS_RANKS) * 8, (fruits, pedicels)
    return fruits


def build(output: Path, plant_state: Path = DAY160) -> Path:
    """Run the four steps; write ``scene.usda`` and the GUI ``config.json``."""

    output.mkdir(parents=True, exist_ok=True)
    scaffold = export_scaffold(output / "scaffold" / "scaffold.usda", plant_state)
    stage, expected = graft_trusses(scaffold)
    slender_trusses(stage, plant_state)
    fruits = calyces_and_thin_pedicels(stage)
    scene = output / "scene.usda"
    stage.GetRootLayer().Export(str(scene))
    config = json.loads((ASSETS / "runtime.json").read_text())
    config.update(
        run_dir=str(output.resolve()),
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        expected_body_properties=expected,
    )
    (output / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    print(f"[OK] main scene {scene}: 5 main-stem trusses, {fruits} fruits with calyces")
    return scene


def prepare_gui_run(output: Path, *, real_time: bool = True) -> Path:
    """Copy the built scene into a fresh ``runs/run-*`` folder for one GUI session.

    Isaac Sim writes its reports next to the scene and refuses to overwrite a
    previous run, so every session gets its own folder.
    """

    import shutil
    import tempfile

    config = json.loads((output / "config.json").read_text())
    scene = output / "scene.usda"
    if hashlib.sha256(scene.read_bytes()).hexdigest() != config["scene_sha256"]:
        raise ValueError(f"{scene} changed after it was built; rebuild it")
    (output / "runs").mkdir(exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix="run-", dir=output / "runs"))
    shutil.copyfile(scene, run / "scene.usda")
    config.update(run_dir=str(run), gui=True, gui_real_time=real_time, duration=60.0, force_target=None)
    (run / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    return run


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Build the day-160 main scene.")
    parser.add_argument("--output", type=Path, default=ROOT / "output" / "main_scene")
    parser.add_argument("--input", type=Path, default=DAY160, help="day-160 PlantState JSON")
    parser.add_argument("--prepare-gui-run", action="store_true",
                        help="do not build; copy the built scene into a fresh run folder and print it")
    args = parser.parse_args(argv)
    if args.prepare_gui_run:
        print(prepare_gui_run(args.output))
        return 0
    build(args.output, args.input)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
