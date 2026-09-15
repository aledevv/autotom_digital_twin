"""Select one nearby admissible support pair and copy matched native-drag cases."""
import hashlib
import json
from pathlib import Path
import shutil
import sys
import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.plant_state_legacy_backend import _collider_records


def closest(x,y):
    p,q=np.asarray(x['start']),np.asarray(y['start'])
    a,b=np.asarray(x['end'])-p,np.asarray(y['end'])-q
    candidates=[]
    for t in (0.,1.):
        u=np.clip(np.dot(p+t*a-q,b)/np.dot(b,b),0,1)
        candidates.append((p+t*a,q+u*b))
    for u in (0.,1.):
        t=np.clip(np.dot(q+u*b-p,a)/np.dot(a,a),0,1)
        candidates.append((p+t*a,q+u*b))
    t,u=np.linalg.lstsq(np.column_stack((a,-b)),q-p,rcond=None)[0]
    if 0<=t<=1 and 0<=u<=1:candidates.append((p+t*a,q+u*b))
    return min(candidates,key=lambda v:np.linalg.norm(v[0]-v[1]))


def main():
    base=Path(sys.argv[1]);stage=Usd.Stage.Open(str(base/'C-screen/scene.usda'))
    audit=json.loads((base/'C-screen/pair-audit.json').read_text());shapes=_collider_records(stage)
    candidates=[]
    for row in audit['pairs']:
        if not row['enabled'] or row['baseline'] or row['gap_m'] is None or not 0<row['gap_m']<=.025:continue
        i,j=row['i'],row['j']
        if stage.GetPrimAtPath(shapes[i]['body_path']).GetAttribute('autotom:role').Get()!='leaf_rachis':i,j=j,i
        if stage.GetPrimAtPath(shapes[i]['body_path']).GetAttribute('autotom:role').Get()=='leaf_rachis':
            candidates.append((row['gap_m'],i,j))
    if not candidates:raise ValueError('No admissible nearby pair; use a manual gesture, do not increase force')
    gap,i,j=min(candidates);x,y=shapes[i],shapes[j];p,q=closest(x,y);cache=UsdGeom.XformCache()
    local=[]
    for shape,point in ((x,p),(y,q)):
        inv=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(shape['body_path'])).GetInverse()
        local.append(list(inv.Transform(Gf.Vec3d(*point))))
    drag=dict(body_a=x['body_path'],body_b=y['body_path'],collider_a=x['path'],collider_b=y['path'],
        local_a=local[0],local_b=local[1],radii_sum=x['radius']+y['radius'],initial_gap_m=gap)
    for variant in 'ABC':
        source=base/f'{variant}-screen';out=base/f'{variant}-drag';out.mkdir(exist_ok=False)
        shutil.copyfile(source/'scene.usda',out/'scene.usda')
        c=json.loads((source/'config.json').read_text());c.update(run_dir=str(out.resolve()),duration=60.,branch_drag=dict(drag))
        if variant!='A':c['branch_drag']['replay']=str((base/'A-drag/native-branch-rays.json').resolve())
        for file in Path(__file__).parent.glob('*.py'):
            c['implementation_sha256'][str(file.relative_to(ROOT))]=hashlib.sha256(file.read_bytes()).hexdigest()
        (out/'config.json').write_text(json.dumps(c,indent=2)+'\n')
    (base/'selected-pair.json').write_text(json.dumps(drag,indent=2)+'\n')
    print(json.dumps(drag),flush=True)


if __name__=='__main__':main()
