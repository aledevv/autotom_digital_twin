"""Prepare a visual-only variant of a verified truss fixture; keep physics intact."""
import argparse
import hashlib
import json
from pathlib import Path
import re

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    data_path = root / 'data/plant_states/plant_state_day_160.json'
    data = json.loads(data_path.read_text())
    stage = Usd.Stage.Open(str(args.source / 'scene.usda'))
    before = {str(a.GetPath()): str(a.Get()) for p in stage.Traverse()
              for a in p.GetAttributes()}
    allowed = set()
    changes = []
    for body in list(stage.Traverse()):
        match = re.search(r'Truss_r\d+_o0_g(\d+)_rachis_', str(body.GetPath()))
        if not match or not body.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        cylinder = UsdGeom.Cylinder(stage.GetPrimAtPath(str(body.GetPath()) + '/Cylinder'))
        if not cylinder:
            continue
        pedicel = 'pedicel' in body.GetName()
        role = 'pedicel' if pedicel else 'truss_rachis'
        radii = [a['radius'] for a in data['axes']
                 if a['owner_node_id'] == 'node:' + match[1] and a['role'] == role]
        assert radii and max(radii) - min(radii) < 1e-10
        radius = radii[0]
        if pedicel:
            mesh = UsdGeom.Mesh(stage.GetPrimAtPath(str(body.GetPath()) + '/GravityElbowPedicelVisual'))
            assert mesh and not mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            # The original procedural tube uses 14 vertices per ring and two caps.
            assert (len(points) - 2) % 14 == 0
            rings = points[:-2].reshape(-1, 14, 3)
            centers = rings.mean(axis=1, keepdims=True)
            offsets = rings - centers
            lengths = np.linalg.norm(offsets, axis=2, keepdims=True)
            assert np.all(lengths > 0)
            # Mild visual taper is an aesthetic choice, not measured GroIMP data.
            target = np.linspace(radius, radius * 0.8, len(rings))[:, None, None]
            rings[:] = centers + offsets / lengths * target
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
            mesh.CreateExtentAttr().Set(Vt.Vec3fArray([
                Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))]))
            allowed.update(str(mesh.GetPath()) + '.' + a for a in ('points', 'extent'))
        else:
            # Keep the original cylinder as the unchanged, invisible collider.
            cylinder.CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
            allowed.add(str(cylinder.GetPath()) + '.visibility')
            visual = UsdGeom.Cylinder.Define(stage, str(body.GetPath()) + '/SlenderRachisVisual')
            for attr in cylinder.GetPrim().GetAttributes():
                if attr.GetName().startswith(('physics:', 'physx')) or attr.GetName() in ('visibility', 'extent'):
                    continue
                value = attr.Get()
                if value is not None:
                    visual.GetPrim().CreateAttribute(attr.GetName(), attr.GetTypeName()).Set(value)
            visual.GetRadiusAttr().Set(radius)
            assert not visual.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
        changes.append(dict(body=str(body.GetPath()), role=role, base_radius_m=radius))
    assert len(changes) == 60
    for path, value in before.items():
        if path not in allowed:
            assert str(stage.GetAttributeAtPath(path).Get()) == value, path
    args.output.mkdir(parents=True, exist_ok=False)
    stage.GetRootLayer().Export(str(args.output / 'scene.usda'))
    config = json.loads((args.source / 'config.json').read_text())
    config.update(run_dir=str(args.output.resolve()), scene_sha256=hashlib.sha256((args.output / 'scene.usda').read_bytes()).hexdigest())
    config['visual_variant'] = dict(source=str(args.source.resolve()), changes=changes,
                                  physics_attributes_unchanged=True,
                                  groimp_sha256=hashlib.sha256(data_path.read_bytes()).hexdigest(),
                                  pedicel_tip_radius_multiplier=0.8)
    (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    print(f'Prepared {args.output}: {len(changes)} visual changes, physics preserved')


if __name__ == '__main__':
    main()
