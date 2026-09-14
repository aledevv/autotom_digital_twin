"""Prepare the single-variable fixed stem/lateral attachment comparison."""
import argparse
import hashlib
import json
from pathlib import Path

from pxr import Usd, UsdPhysics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--duration', type=float, default=20.)
    args = parser.parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    stage = Usd.Stage.Open(str(source / 'scene.usda'))
    candidates = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint):
            continue
        joint = UsdPhysics.Joint(prim)
        a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
        if len(a) == len(b) == 1 and '/Vegetative/trunk/' in str(a[0]) and '/Vegetative/Branch_' in str(b[0]):
            candidates.append(prim)
    if len(candidates) != 1:
        raise ValueError(f'Expected one stem/lateral attachment, found {len(candidates)}')
    prim = candidates[0]
    before = {'type': prim.GetTypeName(), 'apis': list(prim.GetAppliedSchemas()),
              'properties': {a.GetName(): str(a.Get()) for a in prim.GetAttributes()}}
    prim.SetTypeName('PhysicsFixedJoint')
    for schema in list(prim.GetAppliedSchemas()):
        if schema.startswith(('PhysicsDriveAPI:', 'PhysicsLimitAPI:')):
            prim.RemoveAppliedSchema(schema)
    for name in list(prim.GetPropertyNames()):
        if name.startswith(('drive:', 'limit:')):
            prim.RemoveProperty(name)
    stage.GetRootLayer().Export(str(output / 'scene.usda'))
    # Ensure the mutation is confined to this joint, including all geometry,
    # body properties, relationships and attachment frames elsewhere.
    original = Usd.Stage.Open(str(source / 'scene.usda'))
    for old in original.Traverse():
        new = stage.GetPrimAtPath(old.GetPath())
        if old.GetPath() != prim.GetPath():
            assert old.GetTypeName() == new.GetTypeName()
            assert old.GetAppliedSchemas() == new.GetAppliedSchemas()
        for attr in old.GetAttributes():
            if old.GetPath() == prim.GetPath() and attr.GetName().startswith(('drive:', 'limit:')):
                continue
            assert attr.Get() == new.GetAttribute(attr.GetName()).Get(), str(attr.GetPath())
        for rel in old.GetRelationships():
            assert rel.GetTargets() == new.GetRelationship(rel.GetName()).GetTargets()
    config = json.loads((source / 'config.json').read_text())
    config.update(run_dir=str(output), duration=args.duration, gui=False,
                  scene_sha256=hashlib.sha256((output / 'scene.usda').read_bytes()).hexdigest())
    (output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    change = dict(source=str(source), source_sha256=hashlib.sha256((source / 'scene.usda').read_bytes()).hexdigest(),
                  joint=str(prim.GetPath()), before=before, after_type=prim.GetTypeName(),
                  unchanged_properties_verified=True)
    (output / 'variant.json').write_text(json.dumps(change, indent=2) + '\n')
    print(json.dumps(change, indent=2))


if __name__ == '__main__':
    main()
