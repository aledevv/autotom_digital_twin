"""Summarize physical support controls without promoting headless passes to GUI acceptance."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from pxr import Usd, UsdPhysics
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'src'))
from exporterV2.fruit_diagnostics import attachment_errors


def joint_motion(d,hz):
    """Offline frame gaps for all support joints, using the recorded physics poses."""
    stage=Usd.Stage.Open(str(d/'scene.usda'))
    chunks=sorted(d.glob('headless-trace-*.npz'))
    if not chunks:
        return None
    with np.load(chunks[0]) as z:
        paths=list(z['paths'])
    indices={str(p):i for i,p in enumerate(paths)}
    joints=[]
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Joint):
            j=UsdPhysics.Joint(p)
            a,b=j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets()
            if a and b and str(a[0]) in indices and str(b[0]) in indices and '/TerminalBodies/' not in str(b[0]):
                joints.append(j)
    parents=[indices[str(j.GetBody0Rel().GetTargets()[0])] for j in joints]
    children=[indices[str(j.GetBody1Rel().GetTargets()[0])] for j in joints]
    lp=[];lr=[]
    for side in (0,1):
        lp.append(np.array([list(getattr(j,f'GetLocalPos{side}Attr')().Get()) for j in joints]))
        quats=[getattr(j,f'GetLocalRot{side}Attr')().Get() for j in joints]
        lr.append(np.array([[q.GetReal(),*q.GetImaginary()] for q in quats]))
    gaps=[];angles=[]
    for file in chunks:
        with np.load(file) as z:
            for state in z['state']:
                gap,angle=attachment_errors(state[:,:3],state[:,3:7],parents,children,*lp,*lr)
                gaps.append(gap);angles.append(angle)
    gaps=np.asarray(gaps);angles=np.asarray(angles)
    names=[str(j.GetPath()) for j in joints]
    np.savez_compressed(d/'offline-support-joint-errors.npz',joint_paths=names,
                        time_s=np.arange(1,len(gaps)+1)/hz,gap_m=gaps,relative_frame_angle_rad=angles)
    t,j=np.unravel_index(np.argmax(gaps),gaps.shape)
    exceeded=np.argwhere(gaps>.0005)
    return dict(max_gap_m=float(gaps[t,j]),peak_joint=names[j],peak_time_s=(int(t)+1)/hz,
        first_gap_over_05mm=None if not len(exceeded) else dict(time_s=(int(exceeded[0,0])+1)/hz,
            joint=names[int(exceeded[0,1])]),
        caveat='Relative frame angle of a mobile joint includes intended bending; not itself an error.')


def summarize(d):
    r=json.loads((d/'report.json').read_text())
    c=r['config']
    e=json.loads((d/'headless-effective.json').read_text())
    return dict(case=d.name,configuration={k:c[k] for k in ('method','density','lock_entry','lock_internal','without_fruit','hz','solver','gpu')},
        status=r['status'],simulated_seconds=r['simulated_seconds'],errors=r['errors'],
        first_failure=r['first_failure'],tail=r['tail'],numerical_gate_errors=r['numerical_gate_errors'],
        events=[{'time_s':x['time_s'],'joint':x['joint']} for x in r['events'] if x['kind']=='joint_break'],
        body_count=r['body_count'],expected_dofs=c['expected_articulation_dofs'],actual_dofs=e['articulation']['dofs'],
        runtime_property_errors=e['errors'],support_joint_motion=joint_motion(d,c['hz']),local_artifacts=str(d),
        evidence_sha256={f:hashlib.sha256((d/f).read_bytes()).hexdigest() for f in
            ('scene.usda','config.json','controls.json','report.json','headless-effective.json')})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    cases=[]
    for d in sorted(a.directory.iterdir()):
        if (d/'report.json').exists() and json.loads((d/'report.json').read_text()).get('status')!='running':
            cases.append(summarize(d))
    a.output.write_text(json.dumps({'cases':cases},indent=2)+'\n')
    for c in cases:
        print(c['case'],c['status'],round(c['simulated_seconds'],4),'DOFs',c['actual_dofs'])
