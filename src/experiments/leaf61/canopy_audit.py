"""Read-only USD audit: frozen background, identical geometry and exact body counts."""

import argparse
import json
from pathlib import Path


def audit(directory):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    reference = None
    records = []
    for count in (0, 1, 5):
        run = directory / f"n{count}-full-headless"
        stage = Usd.Stage.Open(str(run / "scene.usda"))
        source = Usd.Stage.Open(str(run / "input_canopy.usda"))
        cache = UsdGeom.XformCache()
        source_cache = UsdGeom.XformCache()
        visual = {}
        visible_blades = 0
        frozen_count = 0
        rigid, joints, articulations, colliders = [], [], [], []
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                rigid.append(path)
            if prim.IsA(UsdPhysics.Joint):
                joints.append(path)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                articulations.append(path)
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                colliders.append(path)
            if path.startswith("/World/Canopy/"):
                assert not any(
                    s.startswith(("Physics", "Physx")) for s in prim.GetAppliedSchemas()
                ), path
                if prim.IsA(UsdGeom.Gprim):
                    old = source.GetPrimAtPath(
                        path.replace("/World/Canopy", "/World", 1)
                    )
                    assert old and cache.GetLocalToWorldTransform(
                        prim
                    ) == source_cache.GetLocalToWorldTransform(old), path
                    frozen_count += 1
            if prim.IsA(UsdGeom.Gprim) and path != "/World/Probe/Collider":
                image = UsdGeom.Imageable(prim)
                if image.ComputeVisibility() != "invisible":
                    visual[path] = {
                        "type": prim.GetTypeName(),
                        "world_transform": cache.GetLocalToWorldTransform(prim),
                        "geometry": {
                            key: str(prim.GetAttribute(key).Get())
                            for key in (
                                "points",
                                "faceVertexIndices",
                                "faceVertexCounts",
                                "radius",
                                "height",
                                "size",
                            )
                        },
                    }
                    if prim.GetName() == "LeafBlade" or path.endswith("/Leaf/Mesh"):
                        visible_blades += 1
        assert len(rigid) == 4 * count + 1, rigid
        assert len(joints) == 4 * count, joints
        assert len(articulations) == count, articulations
        assert len(colliders) == 4 * count + 1, colliders
        assert visible_blades == 131, visible_blades
        if reference is None:
            reference = visual
        else:
            assert visual.keys() == reference.keys(), "Visible primitive set changed"
            for path, value in visual.items():
                assert Gf.IsClose(
                    value["world_transform"], reference[path]["world_transform"], 2e-7
                ), path
                assert value["geometry"] == reference[path]["geometry"], path
                assert value["type"] == reference[path]["type"], path
        records.append(
            {
                "dynamic_laminae": count,
                "rigid_bodies_including_fixed_roots_and_probe": len(rigid),
                "joints_including_fixed_roots": len(joints),
                "articulations": len(articulations),
                "visible_laminae": visible_blades,
                "frozen_source_gprims": frozen_count,
                "visible_gprims_excluding_probe": len(visual),
            }
        )
    result = {
        "passed": True,
        "initial_visual_geometry_identical": True,
        "transform_comparison_tolerance": 2e-7,
        "source_world_transforms_preserved": True,
        "cases": records,
    }
    (directory / "usd_audit.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    print(json.dumps(audit(parser.parse_args().directory), indent=2))
