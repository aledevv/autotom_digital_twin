"""Add lightweight, non-colliding calyces and slender pedicels to a USD fixture.

The supplied photo guides appearance only. Fruit radii stay exactly as authored;
calyx proportions and pedicel taper are aesthetic, not measured morphology.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt


def calyx_points(radius, attachment):
    axis = np.asarray(attachment, dtype=float)
    axis /= np.linalg.norm(axis)
    helper = np.eye(3)[np.argmin(np.abs(axis))]
    u = np.cross(axis, helper)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    points, counts, indices = [], [], []
    # Five tapered sepals, with a raised central ridge and subtly lifted tips.
    # The full patch lies just outside the existing spherical fruit surface.
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
                height = math.sqrt(radius ** 2 - float(offset @ offset))
                height += radius * (0.012 + 0.065 * t ** 5 + (0.018 if across == 0 else 0))
                points.append(offset + axis * height)
            if row:
                previous = start + (row - 1) * 3
                for col in range(2):
                    indices.extend([previous + col, previous + col + 1,
                                    previous + col + 4, previous + col + 3])
                    counts.append(4)
    # Small hub closes the base of the five blades, around the pedicel insertion.
    hub = len(points)
    points.append(axis * radius * 1.045)
    for i in range(20):
        theta = i * math.tau / 20
        points.append(axis * radius * 1.012 + radius * 0.16 * (math.cos(theta) * u + math.sin(theta) * v))
    for i in range(20):
        indices.extend([hub, hub + 1 + i, hub + 1 + (i + 1) % 20])
        counts.append(3)
    return np.asarray(points), counts, indices


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    stage = Usd.Stage.Open(str(args.source / 'scene.usda'))
    before = {str(a.GetPath()): str(a.Get()) for p in stage.Traverse() for a in p.GetAttributes()}
    relationships = {str(r.GetPath()): list(r.GetTargets()) for p in stage.Traverse() for r in p.GetRelationships()}
    schemas = {str(p.GetPath()): p.GetAppliedSchemas() for p in stage.Traverse()}
    allowed = set()
    fruit_count = pedicel_count = 0
    for prim in list(stage.Traverse()):
        if prim.GetName() == 'GravityElbowPedicelVisual':
            assert not prim.HasAPI(UsdPhysics.CollisionAPI)
            mesh = UsdGeom.Mesh(prim)
            points = np.asarray(mesh.GetPointsAttr().Get(), dtype=float)
            rings = points[:-2].reshape(-1, 14, 3)
            centers = rings.mean(axis=1, keepdims=True)
            # 2.4mm at base, 1.92mm at tip, starting from the 3mm slender variant.
            rings[:] = centers + 0.8 * (rings - centers)
            mesh.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
            mesh.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))]))
            allowed.update(str(prim.GetPath()) + '.' + attr for attr in ('points', 'extent'))
            pedicel_count += 1
        if prim.GetAttribute('autotom:role').Get() != 'fruit':
            continue
        sphere = UsdGeom.Sphere(stage.GetPrimAtPath(str(prim.GetPath()) + '/Sphere'))
        joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath(str(prim.GetPath()) + '/TerminalBodyFixedJoint'))
        points, counts, indices = calyx_points(sphere.GetRadiusAttr().Get(), joint.GetLocalPos1Attr().Get())
        assert np.isfinite(points).all()
        mesh = UsdGeom.Mesh.Define(stage, str(prim.GetPath()) + '/CalyxVisual')
        mesh.CreatePointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
        mesh.CreateFaceVertexCountsAttr().Set(counts)
        mesh.CreateFaceVertexIndicesAttr().Set(indices)
        mesh.CreateDoubleSidedAttr().Set(True)
        mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
        mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.12, 0.24, 0.045)])
        mesh.CreateExtentAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*points.min(axis=0)), Gf.Vec3f(*points.max(axis=0))]))
        assert not mesh.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
        fruit_count += 1
    assert fruit_count == pedicel_count == 40
    for path, value in before.items():
        if path not in allowed:
            assert str(stage.GetAttributeAtPath(path).Get()) == value, path
    for path, targets in relationships.items():
        assert stage.GetRelationshipAtPath(path).GetTargets() == targets, path
    for path, value in schemas.items():
        assert stage.GetPrimAtPath(path).GetAppliedSchemas() == value, path
    args.output.mkdir(parents=True, exist_ok=False)
    stage.GetRootLayer().Export(str(args.output / 'scene.usda'))
    config = json.loads((args.source / 'config.json').read_text())
    config.update(run_dir=str(args.output.resolve()), scene_sha256=hashlib.sha256((args.output / 'scene.usda').read_bytes()).hexdigest())
    config['natural_visual_variant'] = dict(source=str(args.source.resolve()),
        calyces=fruit_count, pedicels=pedicel_count, fruit_radius_unchanged=True,
        physics_attributes_and_relationships_unchanged=True,
        pedicel_radius_multiplier=0.8, calyx_source='aesthetic reference photo, not measured')
    script = Path(__file__).resolve()
    config.setdefault('implementation_sha256', {})[str(script.relative_to(script.parents[3]))] = hashlib.sha256(script.read_bytes()).hexdigest()
    (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    print(f'Prepared {fruit_count} calyces and {pedicel_count} pedicels; original fruit radii and physics preserved')


if __name__ == '__main__':
    main()
