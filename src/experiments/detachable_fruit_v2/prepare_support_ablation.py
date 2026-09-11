"""Isolate native fruit support mechanics on private copies of audited scenes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src'))
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from exporterV2.fruit_experiments import attachment_records, audit, bodies_and_joints, joint_frame, value


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def properties(stage):
    return {str(p.GetPath()): str(p.Get()) if isinstance(p, Usd.Attribute) else str(p.GetTargets())
            for prim in stage.Traverse() for p in prim.GetProperties()}


def reduce_support(stage, fruit, mode):
    before = properties(stage)
    bodies, joints = bodies_and_joints(stage)
    selected = next(r for r in attachment_records(stage) if r['fruit'] == fruit)
    root = next(p for p in bodies if bodies[p].GetChild('RootFixedJoint'))
    incoming = {str(j.GetBody1Rel().GetTargets()[0]): j for j in joints if j.GetBody1Rel().GetTargets()}
    keep = {root, selected['parent'], fruit}
    if mode == 'rachis':
        keep |= {p for p, b in bodies.items() if value(b, 'autotom:role') in ('internode', 'truss_rachis')}
    elif mode not in ('rigid', 'articulated'):
        raise ValueError(mode)
    pedicel_joint = incoming[selected['parent']]
    if mode != 'rachis':
        frame = joint_frame(stage, pedicel_joint, 0)
        local = frame * UsdGeom.Xformable(bodies[root]).ComputeLocalToWorldTransform(0).GetInverse()
        pedicel_joint.CreateBody0Rel().SetTargets([root])
        pedicel_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(local.ExtractTranslation()))
        pedicel_joint.CreateLocalRot0Attr().Set(Gf.Quatf(local.ExtractRotationQuat()))
        if mode == 'rigid':
            for axis in ('rotX', 'rotY', 'rotZ'):
                limit = UsdPhysics.LimitAPI.Apply(pedicel_joint.GetPrim(), axis)
                limit.CreateLowAttr().Set(1.)
                limit.CreateHighAttr().Set(-1.)
    for j in joints:
        b = j.GetBody1Rel().GetTargets()
        if not b or str(b[0]) not in keep:
            stage.RemovePrim(j.GetPath())
    for path in sorted(set(bodies)-keep, key=len, reverse=True):
        stage.RemovePrim(path)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
            rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
            rel.SetTargets([p for p in rel.GetTargets() if stage.GetPrimAtPath(p)])
    after = properties(stage)
    changes = [{'property':p, 'original':before.get(p), 'modified':after.get(p)}
               for p in sorted(set(before)|set(after)) if before.get(p) != after.get(p)]
    # Only the selected pedicel's incoming constraint and removed-body filters may change.
    joint_prefix = str(pedicel_joint.GetPath())+'.'
    for c in changes:
        if c['modified'] is not None and c['original'] != c['modified']:
            assert c['property'].startswith(joint_prefix) or c['property'].endswith('.physics:filteredPairs'), c
    result = audit(stage)
    if result['errors']:
        raise ValueError(result['errors'])
    assert len(result['attachments']) == 1
    assert set(bodies_and_joints(stage)[0]) == keep
    return dict(mode=mode, selected=selected, retained=sorted(keep),
                removed=sorted(set(bodies)-keep), changes=changes, audit=result)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-case', type=Path, required=True)
    p.add_argument('--run-dir', type=Path, required=True)
    p.add_argument('--mode', choices=('rigid', 'articulated', 'rachis'), required=True)
    p.add_argument('--fruit-number', type=int, default=7)
    p.add_argument('--gui', action='store_true')
    p.add_argument('--duration', type=float, default=20)
    a = p.parse_args()
    if not 1 <= a.fruit_number <= 8:
        p.error('The paired rank6 truss has fruit numbers 1 through 8')
    out = a.run_dir.resolve()
    if (out/'config.json').exists():
        raise ValueError('Use a fresh run directory')
    source = (a.source_case/'scene.usda').resolve()
    stage = Usd.Stage.Open(Usd.Stage.Open(str(source)).Flatten())
    records = attachment_records(stage)
    config = json.loads((a.source_case/'config.json').read_text())
    # Original builder order: main alternating L/R at nodes0..3, v2.3 numbered1..8.
    suffix = (f"pedicel_lat_{(a.fruit_number-1)//2}_{'L' if a.fruit_number%2 else 'R'}_tomato"
              if config['method'] == 'main' else f'tomato_{a.fruit_number:02d}')
    fruit = next(r['fruit'] for r in records if r['fruit'].endswith(suffix))
    manifest = reduce_support(stage, fruit, a.mode)
    config.update(scenario='support-ablation', support_mode=a.mode, fruit=fruit,
                  corresponding_fruit_number=a.fruit_number,
                  run_dir=str(out), source_usd=str(source), source_sha256=sha(source),
                  source_case_config_sha256=sha(a.source_case/'config.json'),
                  duration=a.duration, gui=a.gui, observe_spontaneous_breaks=False,
                  current_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=ROOT,text=True).strip())
    # Frame the isolated grasp target; identical camera offset for both constructions.
    center = UsdGeom.Xformable(stage.GetPrimAtPath(fruit)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    config.update(camera_target=list(center), camera_eye=list(center+Gf.Vec3d(.35,.35,.25)))
    out.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Export(str(out/'scene.usda'))
    config['scene_sha256'] = sha(out/'scene.usda')
    files = set(config['implementation_sha256']) | {str(Path(__file__).relative_to(ROOT))}
    config['implementation_sha256'] = {f:sha(ROOT/f) for f in sorted(files)}
    (out/'ablation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    print(a.mode,config['method'],len(manifest['retained']),'bodies',fruit)
