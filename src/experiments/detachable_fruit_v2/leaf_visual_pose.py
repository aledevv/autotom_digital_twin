"""Visual-only petiolule droop and smooth shading; no exporter/runtime changes."""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument('--droop-degrees', type=float, default=10.)
    a = ap.parse_args()
    if not 0 <= a.droop_degrees <= 15:
        ap.error('droop must be between 0 and 15 degrees')
    stage = Usd.Stage.Open(str(a.source / 'scene.usda'))
    before = {str(x.GetPath()): str(x.Get()) for p in stage.Traverse() for x in p.GetAttributes()}
    allowed, changes = set(), []
    cache = UsdGeom.XformCache()
    for p in list(stage.Traverse()):
        if not (p.GetName().startswith('LeafVisual_') and 'petiolule_' in p.GetName()):
            continue
        assert not any(q.HasAPI(UsdPhysics.RigidBodyAPI) or q.HasAPI(UsdPhysics.CollisionAPI)
                       for q in Usd.PrimRange(p))
        xform = UsdGeom.Xformable(p)
        ops = xform.GetOrderedXformOps()
        assert len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform
        world = cache.GetLocalToWorldTransform(p)
        base = world.ExtractTranslation()
        direction = world.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        down = Gf.Vec3d(0, 0, -1)
        angle = math.degrees(math.acos(max(-1., min(1., Gf.Dot(direction, down)))))
        tilt = min(a.droop_degrees, angle)
        axis = Gf.Cross(direction, down)
        if axis.GetLength() < 1e-9 or tilt < 1e-6:
            continue
        rotation = Gf.Matrix4d().SetRotate(Gf.Rotation(axis.GetNormalized(), tilt))
        # Row-vector USD matrices: rotate the frame in world space, fix its base.
        rotated = world * rotation
        rotated.SetTranslateOnly(base)
        local = rotated * cache.GetLocalToWorldTransform(p.GetParent()).GetInverse()
        ops[0].Set(local)
        allowed.add(str(ops[0].GetAttr().GetPath()))
        actual = local * cache.GetLocalToWorldTransform(p.GetParent())
        assert (actual.ExtractTranslation() - base).GetLength() < 1e-10
        after_dir = actual.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
        after_angle = math.degrees(math.acos(max(-1., min(1., Gf.Dot(after_dir, down)))))
        assert abs((angle - after_angle) - tilt) < 1e-5
        changes.append(dict(path=str(p.GetPath()), tilt_degrees=tilt,
                            base_shift_m=(actual.ExtractTranslation() - base).GetLength()))
    shaded = []
    for p in stage.Traverse():
        if not (p.IsA(UsdGeom.Mesh) and p.GetName().startswith('OrganicVisual') and '/Leaf_' in str(p.GetPath())):
            continue
        assert not p.HasAPI(UsdPhysics.CollisionAPI)
        mesh = UsdGeom.Mesh(p)
        points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
        counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
        assert np.all(counts == 3)
        faces = np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        normals = np.zeros_like(points)
        face_normals = np.cross(points[faces[:, 1]] - points[faces[:, 0]], points[faces[:, 2]] - points[faces[:, 0]])
        for col in range(3):
            np.add.at(normals, faces[:, col], face_normals)
        length = np.linalg.norm(normals, axis=1)
        good = length > 1e-15
        normals[good] /= length[good, None]
        normals[~good] = [0, 0, 1]
        mesh.CreateNormalsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in normals]))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        allowed.add(str(p.GetPath()) + '.normals')
        shaded.append(str(p.GetPath()))
    assert changes and shaded
    for path, value in before.items():
        if path not in allowed:
            assert str(stage.GetAttributeAtPath(path).Get()) == value, path
    a.output.mkdir(parents=True, exist_ok=False)
    stage.GetRootLayer().Export(str(a.output / 'scene.usda'))
    config = json.loads((a.source / 'config.json').read_text())
    config.update(run_dir=str(a.output.resolve()), scene_sha256=hashlib.sha256((a.output / 'scene.usda').read_bytes()).hexdigest())
    config['leaf_visual_pose'] = dict(source=str(a.source.resolve()), petiolules=changes,
        smooth_shaded=shaded, droop_degrees=a.droop_degrees,
        note='Aesthetic rest-pose change; static relative to the existing physical leaf support')
    root = Path(__file__).resolve().parents[3]
    config['implementation_sha256'][str(Path(__file__).resolve().relative_to(root))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (a.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    print(f'Prepared {len(changes)} drooping petiolules and {len(shaded)} smooth-shaded support meshes')


if __name__ == '__main__':
    main()
