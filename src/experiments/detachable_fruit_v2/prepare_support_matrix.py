"""Independent rachis entry/internal locks and diagnostic support density controls."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from pxr import Gf, Usd, UsdPhysics
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.drive_experiments import drive_records
from exporterV2.fruit_experiments import audit, bodies_and_joints
from experiments.detachable_fruit_v2.prepare_support_ablation import properties, sha


def apply_controls(stage, effective, ratio, lock_entry=False, lock_internal=False, without_fruit=False):
    before = properties(stage)
    expected = {}
    for body in effective['bodies']:
        factor = ratio if body['role'] in ('truss_rachis','pedicel') else 1.
        if without_fruit and body['role']=='fruit':
            stage.RemovePrim(body['path'])
            continue
        inertia = np.asarray(body['inertia']).reshape(3,3)
        if factor != 1:
            # The source supports use diagonal tensors in their recorded COM frame.
            if not np.allclose(inertia, np.diag(np.diag(inertia)), rtol=0, atol=1e-16):
                raise ValueError('Non-diagonal source inertia requires explicit frame conversion')
            api = UsdPhysics.MassAPI(stage.GetPrimAtPath(body['path']))
            api.CreateMassAttr().Set(body['mass_kg']*factor)
            api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*(np.diag(inertia)*factor)))
            q = np.asarray(body['com_orientation']).reshape(4)
            api.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:])))
        expected[body['path']] = dict(mass_kg=body['mass_kg']*factor,
            inertia=(inertia*factor).reshape(9).tolist(), com=body['com'], com_orientation=body['com_orientation'])
    locked = set()
    for row in drive_records(stage):
        if ((lock_entry and row['role']=='rachis_attachment') or
                (lock_internal and row['role']=='rachis_internal')):
            prim = stage.GetPrimAtPath(row['joint'])
            for axis in ('rotX','rotY'):
                limit = UsdPhysics.LimitAPI.Apply(prim,axis)
                limit.CreateLowAttr().Set(1.)
                limit.CreateHighAttr().Set(-1.)
            locked.add(row['joint'])
    if without_fruit:
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.FilteredPairsAPI):
                rel = UsdPhysics.FilteredPairsAPI(prim).GetFilteredPairsRel()
                rel.SetTargets([p for p in rel.GetTargets() if stage.GetPrimAtPath(p)])
    after = properties(stage)
    changes = [{'property':p,'original':before.get(p),'modified':after.get(p)}
               for p in sorted(before.keys()|after.keys()) if before.get(p)!=after.get(p)]
    result = audit(stage)
    if result['errors']:
        raise ValueError(result['errors'])
    return dict(changes=changes, expected_body_properties=expected, audit=result,
                locked_joints=sorted(locked), expected_articulation_dofs=effective['articulation']['dofs']-2*len(locked))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source-case',type=Path,required=True)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--density',type=int,choices=(2000,20000),required=True)
    p.add_argument('--lock-entry',action='store_true')
    p.add_argument('--lock-internal',action='store_true')
    p.add_argument('--without-fruit',action='store_true')
    p.add_argument('--gui',action='store_true')
    p.add_argument('--duration',type=float,default=20)
    a=p.parse_args()
    out=a.run_dir.resolve()
    if (out/'config.json').exists():
        raise ValueError('Use a fresh directory')
    source=(a.source_case/'scene.usda').resolve()
    stage=Usd.Stage.Open(Usd.Stage.Open(str(source)).Flatten())
    config=json.loads((a.source_case/'config.json').read_text())
    effective=json.loads((a.source_case/'headless-effective.json').read_text())
    ratio=a.density/(20000 if config['method']=='main' else 2000)
    manifest=apply_controls(stage,effective,ratio,a.lock_entry,a.lock_internal,a.without_fruit)
    config.update(run_dir=str(out), source_usd=str(source),source_sha256=sha(source),
        source_effective_sha256=sha(a.source_case/'headless-effective.json'),
        scenario='support-matrix',density=a.density,density_ratio=ratio,
        lock_entry=a.lock_entry,lock_internal=a.lock_internal,without_fruit=a.without_fruit,
        duration=a.duration,gui=a.gui,observe_spontaneous_breaks=False,
        expected_articulation_dofs=manifest['expected_articulation_dofs'],
        expected_body_properties=manifest['expected_body_properties'],
        current_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip())
    if a.without_fruit:
        config['fruit']=None
    out.mkdir(parents=True,exist_ok=True)
    stage.GetRootLayer().Export(str(out/'scene.usda'))
    config['scene_sha256']=sha(out/'scene.usda')
    files=set(config['implementation_sha256'])|{str(Path(__file__).relative_to(ROOT))}
    config['implementation_sha256']={f:sha(ROOT/f) for f in sorted(files)}
    (out/'controls.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    print(out.name,'bodies',len(manifest['expected_body_properties']),'expected DOFs',manifest['expected_articulation_dofs'])
