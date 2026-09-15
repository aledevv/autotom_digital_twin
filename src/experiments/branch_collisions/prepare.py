"""Audited collider-pair policies for a fixed full-plant experiment."""
import argparse
from collections import Counter, defaultdict
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.fruit_experiments import bodies_and_joints, value
from exporterV2.plant_state_legacy_backend import _collider_records, _segment_distance


def under(path, root):
    return path==root or path.startswith(root+'/')


def matrix(stage, shapes, bodies, joints):
    n=len(shapes);allowed=np.ones((n,n),dtype=bool)
    internal={p for p,b in bodies.items() if value(b,'autotom:role')!='fruit'}
    self_enabled=bool(stage.GetPrimAtPath('/World/Stem').GetAttribute('physxArticulation:enabledSelfCollisions').Get())
    joined=set()
    for j in joints:
        a,b=j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets()
        if a and b and not j.GetCollisionEnabledAttr().Get():joined.add(frozenset((str(a[0]),str(b[0]))))
    for i,a in enumerate(shapes):
        for k,b in enumerate(shapes):
            pair=frozenset((a['body_path'],b['body_path']))
            if (a['body_path']==b['body_path'] or pair in joined
                or (not self_enabled and pair <= internal)):
                allowed[i,k]=False
    for p in stage.Traverse():
        rel=UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel()
        if not rel:continue
        left=[i for i,s in enumerate(shapes) if under(s['path'],str(p.GetPath()))]
        right=[i for i,s in enumerate(shapes) if any(under(s['path'],str(t)) for t in rel.GetTargets())]
        allowed[np.ix_(left,right)]=False;allowed[np.ix_(right,left)]=False
    return allowed


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('source',type=Path);ap.add_argument('output',type=Path)
    ap.add_argument('--variant',choices=list('ABCD'),required=True)
    ap.add_argument('--offsets',type=Path)
    ap.add_argument('--duration',type=float,default=20.)
    a=ap.parse_args()
    if a.variant!='A' and not a.offsets:ap.error('B/C/D require measured baseline contact offsets')
    stage=Usd.Stage.Open(str(a.source/'scene.usda'))
    bodies,joints=bodies_and_joints(stage);shapes=_collider_records(stage)
    assert len(bodies)==181 and len(shapes)==262
    for s in shapes:
        prim=stage.GetPrimAtPath(s['path'])
        transform=np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(prim))[:3,:3]
        assert np.allclose(transform@transform.T,np.eye(3),atol=1e-6), 'Unexpected scaled collider'
        if s['shape']!='sphere':assert prim.GetAttribute('axis').Get()=='Z'
    assert not any(p.IsA(UsdPhysics.CollisionGroup) for p in stage.Traverse())
    assert all(value(stage.GetPrimAtPath(s['path']),'physics:collisionEnabled',True) for s in shapes)
    baseline=matrix(stage,shapes,bodies,joints);desired=baseline.copy()
    adjacency=defaultdict(set)
    for j in joints:
        x,y=j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets()
        if x and y:adjacency[str(x[0])].add(str(y[0]));adjacency[str(y[0])].add(str(x[0]))
    nearby={p:adjacency[p] | set().union(*(adjacency[q] for q in adjacency[p])) for p in bodies}
    offsets=json.loads(a.offsets.read_text()) if a.offsets else {}
    margins={p:max([float(x) for x in row['contact'] if x>=0],default=0.) for p,row in offsets.items()}
    reasons=Counter();pairs=[]
    before={str(x.GetPath()):str(x.Get()) for p in stage.Traverse() for x in p.GetAttributes()}
    for i,k in itertools.combinations(range(len(shapes)),2):
        x,y=shapes[i],shapes[k];bp,bq=x['body_path'],y['body_path'];p,q=bodies[bp],bodies[bq]
        rp,rq=value(p,'autotom:role'),value(q,'autotom:role')
        reason='eligible';gap=None
        if bp==bq:reason='same_body'
        elif 'fruit' in (rp,rq):reason='preserve_fruit_policy'
        elif bq in nearby[bp]:reason='within_two_joints'
        elif value(p,'autotom:branchId')==value(q,'autotom:branchId'):reason='same_axis'
        elif value(p,'autotom:canonicalOrganId')==value(q,'autotom:canonicalOrganId'):reason='same_organ'
        else:
            gap=_segment_distance(x['start'],x['end'],y['start'],y['end'])-x['radius']-y['radius']
            if offsets and gap <= max(.001,margins[bp]+margins[bq]):reason='initial_contact_margin'
            elif 'truss_rachis' in (rp,rq) or 'pedicel' in (rp,rq):reason='truss_extension'
        enable=(a.variant in 'CD' and reason=='eligible') or (a.variant=='D' and reason=='truss_extension')
        if enable:desired[i,k]=desired[k,i]=True
        reasons[reason]+=1
        pairs.append(dict(i=i,j=k,body_a=bp,body_b=bq,reason=reason,gap_m=gap,
                          baseline=bool(baseline[i,k]),enabled=bool(desired[i,k])))
    if a.variant!='A':
        for p in stage.Traverse():
            rel=UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel()
            if rel:rel.ClearTargets(True)
        stage.GetPrimAtPath('/World/Stem').GetAttribute('physxArticulation:enabledSelfCollisions').Set(True)
        for i,s in enumerate(shapes):
            targets=[Sdf.Path(shapes[k]['path']) for k in range(i+1,len(shapes)) if not desired[i,k]
                     and shapes[k]['body_path']!=s['body_path']]
            if targets:UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(s['path'])).CreateFilteredPairsRel().SetTargets(targets)
        assert np.array_equal(matrix(stage,shapes,bodies,joints),desired), 'authored filter matrix mismatch'
    if a.variant=='B':assert np.array_equal(desired,baseline)
    for path,v in before.items():
        if path.endswith('.physxArticulation:enabledSelfCollisions'):continue
        assert str(stage.GetAttributeAtPath(path).Get())==v,path
    for p in bodies.values():
        p.AddAppliedSchema('PhysxContactReportAPI')
        p.CreateAttribute('physxContactReport:threshold',Sdf.ValueTypeNames.Float).Set(0.)
    a.output.mkdir(parents=True,exist_ok=False)
    stage.GetRootLayer().Export(str(a.output/'scene.usda'))
    config=json.loads((a.source/'config.json').read_text())
    config.update(run_dir=str(a.output.resolve()),duration=a.duration,gui=False,force_target=None,contact_recording=True)
    config['collision_variant']=a.variant
    config['collision_source_sha256']=hashlib.sha256((a.source/'scene.usda').read_bytes()).hexdigest()
    if a.offsets:
        effective=json.loads((a.offsets.parent/'headless-effective.json').read_text())
        config['expected_body_properties']={b['path']:{k:b[k] for k in ('mass_kg','inertia','com','com_orientation')}
                                            for b in effective['bodies']}
    config['scene_sha256']=hashlib.sha256((a.output/'scene.usda').read_bytes()).hexdigest()
    for rel in ('prepare.py','run.py'):
        file=Path(__file__).parent/rel;config['implementation_sha256'][str(file.relative_to(ROOT))]=hashlib.sha256(file.read_bytes()).hexdigest()
    (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    summary=dict(variant=a.variant,bodies=len(bodies),colliders=len(shapes),shapes=dict(Counter(s['shape'] for s in shapes)),
        joints=len(joints),baseline_pairs=int(np.triu(baseline,1).sum()),enabled_pairs=int(np.triu(desired,1).sum()),
        reasons=dict(reasons),offsets_source=str(a.offsets),geometry_note='Capsules/spheres exact; cylinders conservatively bounded as capsules; no geometry edits')
    meshes=[UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
    summary['render_mesh_vertices']=sum(len(m.GetPointsAttr().Get() or []) for m in meshes)
    summary['render_mesh_triangles']=sum(sum(max(int(n)-2,0) for n in (m.GetFaceVertexCountsAttr().Get() or [])) for m in meshes)
    (a.output/'pair-audit.json').write_text(json.dumps(dict(summary=summary,colliders=[s['path'] for s in shapes],pairs=pairs),indent=2)+'\n')
    print(json.dumps(summary),flush=True)


if __name__=='__main__':main()
