"""Prepare tapered visual rachides and a bounded leaf-compliance comparison."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source', type=Path)
    ap.add_argument('output', type=Path)
    ap.add_argument('--leaf-stiffness', type=float, choices=(1.0, 0.8), default=0.8)
    ap.add_argument('--leaf-probe', action='store_true')
    ap.add_argument('--probe-force', type=float, choices=(0.002, 0.05), default=0.002)
    args = ap.parse_args()
    stage = Usd.Stage.Open(str(args.source / 'scene.usda'))
    before = {str(a.GetPath()): str(a.Get()) for p in stage.Traverse() for a in p.GetAttributes()}
    allowed, changes = set(), []
    leaf_bodies = []
    for p in list(stage.Traverse()):
        if p.GetAttribute('autotom:role').Get() in ('petiole', 'leaf_rachis'):
            leaf_bodies.append(p)
            for joint in Usd.PrimRange(p):
                if not joint.IsA(UsdPhysics.Joint):
                    continue
                for axis in ('rotX', 'rotY'):
                    a = joint.GetAttribute(f'drive:{axis}:physics:stiffness')
                    if a and a.Get() is not None:
                        value = float(a.Get())
                        a.Set(value * args.leaf_stiffness)
                        allowed.add(str(a.GetPath()))
                        changes.append(dict(attribute=str(a.GetPath()), before=value, after=float(a.Get())))
        if p.GetName() != 'SlenderRachisVisual':
            continue
        cylinder = UsdGeom.Cylinder(p)
        assert not p.HasAPI(UsdPhysics.CollisionAPI)
        segment = int(re.search(r'_Link_(\d+)', str(p.GetPath()))[1]) - 1
        height, radius = cylinder.GetHeightAttr().Get(), cylinder.GetRadiusAttr().Get()
        assert cylinder.GetAxisAttr().Get() == 'Z'
        points, counts, indices = [], [], []
        for row in range(9):
            t = row / 8
            # Shared end radii across the four links; tips retain 65% of base.
            r = radius * (1 - 0.35 * (segment + t) / 4)
            center = np.array([0.0005 * math.sin(math.pi * t) ** 2, 0, height * (t - 0.5)])
            for col in range(16):
                theta = col * math.tau / 16
                points.append(center + [r * math.cos(theta), r * math.sin(theta), 0])
                if row:
                    prev = (row - 1) * 16
                    indices.extend([prev + col, prev + (col + 1) % 16,
                                    prev + 16 + (col + 1) % 16, prev + 16 + col])
                    counts.append(4)
        # Close both ends, including the distal tip.
        for row, reverse in ((0, True), (8, False)):
            ring = list(range(row * 16, (row + 1) * 16))
            indices.extend(ring[::-1] if reverse else ring)
            counts.append(16)
        mesh = UsdGeom.Mesh.Define(stage, str(p.GetParent().GetPath()) + '/TaperedRachisVisual')
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*v) for v in points]))
        mesh.CreateFaceVertexCountsAttr().Set(counts)
        mesh.CreateFaceVertexIndicesAttr().Set(indices)
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDisplayColorAttr().Set(cylinder.GetDisplayColorAttr().Get())
        for a in p.GetAttributes():
            if a.GetName().startswith('xformOp') and a.Get() is not None:
                mesh.GetPrim().CreateAttribute(a.GetName(), a.GetTypeName()).Set(a.Get())
        UsdGeom.Imageable(p).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)
        allowed.add(str(p.GetPath()) + '.visibility')
    assert len(leaf_bodies) == 55 and len(changes) == 110
    for path, value in before.items():
        if path not in allowed:
            assert str(stage.GetAttributeAtPath(path).Get()) == value, path
    args.output.mkdir(parents=True, exist_ok=False)
    stage.GetRootLayer().Export(str(args.output / 'scene.usda'))
    config = json.loads((args.source / 'config.json').read_text())
    config.update(run_dir=str(args.output.resolve()), duration=60., gui=False, force_target=None)
    config['scene_sha256'] = hashlib.sha256((args.output / 'scene.usda').read_bytes()).hexdigest()
    config['realism_variant'] = dict(leaf_stiffness_scale=args.leaf_stiffness, changes=changes,
        damping_unchanged=True, masses_colliders_limits_unchanged=True,
        visual_rachis_tip_radius_scale=0.65, visual_rachis_bow_m=0.0005)
    if args.leaf_probe:
        # Longest leaf rachis body; deterministic and independent of source order.
        candidates = [p for p in leaf_bodies if p.GetAttribute('autotom:role').Get() == 'leaf_rachis']
        def length(p):
            return sum(float(UsdGeom.Capsule(c).GetHeightAttr().Get())
                       for c in p.GetChildren() if c.IsA(UsdGeom.Capsule))
        target = sorted(candidates, key=lambda p: (-length(p), str(p.GetPath())))[0]
        config['leaf_compliance_probe'] = dict(body=str(target.GetPath()), start_s=30.,
            end_s=35., ramp_s=1., force_n=[0., 0., -args.probe_force])
    for rel in ('src/experiments/detachable_fruit_v2/prepare_realism_candidate.py',
                'src/experiments/detachable_fruit_v2/run_leaf_probe.py'):
        config['implementation_sha256'][rel] = hashlib.sha256((Path(__file__).resolve().parents[3] / rel).read_bytes()).hexdigest()
    (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    print(f'Prepared {args.output}, leaf stiffness x{args.leaf_stiffness}, probe={args.leaf_probe}')


if __name__ == '__main__':
    main()
