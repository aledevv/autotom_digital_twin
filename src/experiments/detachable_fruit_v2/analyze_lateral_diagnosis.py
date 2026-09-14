"""Summarize bounded lateral experiments and the first 0.35 s of joint motion."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def analyze(root):
    result={'cases':{},'first_window_s':0.35}
    for directory in sorted(root.iterdir()):
        if not (directory/'report.json').exists():continue
        report=json.loads((directory/'report.json').read_text())
        if report['status']=='running':continue
        c={k:report.get(k) for k in ['status','errors','simulated_seconds','first_failure','tail','performance','termination_reason']}
        c['breaks']=[{k:e.get(k) for k in ['time_s','fruit','user_target','continuity_passed']} for e in report.get('events',[])]
        c['scene_sha256']=hashlib.sha256((directory/'scene.usda').read_bytes()).hexdigest()
        c['evidence']=str(directory)
        effective=json.loads((directory/'headless-effective.json').read_text())
        c['effective_errors']=effective['errors']
        c['colliders_enabled']=sum(effective['collision_enabled'].values())
        c['articulation_dofs']=effective['articulation']['dofs']
        c['filters_count']=len(effective['filtered_pairs'])
        meta=json.loads((directory/'headless-dofs.json').read_text())
        dofs=np.concatenate([np.load(f)['positions'] for f in sorted(directory.glob('headless-dofs-*.npz'))])[:,0,:]
        n=min(len(dofs),21)
        c['first_window_lateral_dofs']=[]
        for i,path in enumerate(meta['paths'][0]):
            if '/Vegetative/Branch_' not in path:continue
            lo,hi=np.rad2deg(meta['limits'][0][i]);angles=np.rad2deg(dofs[:n,i])
            c['first_window_lateral_dofs'].append(dict(path=path,dof_index=i,limits_deg=[lo,hi],
                min_deg=float(angles.min()),max_deg=float(angles.max()),last_deg=float(angles[-1]),
                min_margin_to_limit_deg=float(np.minimum(angles-lo,hi-angles).min())))
        files=sorted(directory.glob('headless-trace-*.npz'))
        first=np.load(files[0]);state=np.concatenate([np.load(f)['state'] for f in files])
        body_paths=first['paths'];i=next(i for i,p in enumerate(body_paths) if str(p).endswith('rachis_Link_01'))
        c['attachment_drop_first_window_m']=float(state[0,i,2]-state[n-1,i,2])
        c['comparison_start_time_s']=1/report['runtime_physics_hz']
        c['comparison_end_time_s']=n/report['runtime_physics_hz']
        result['cases'][directory.name]=c
    a=root/'baseline/headless-trace-0000.npz';b=root/'baseline-no-contacts/headless-trace-0000.npz'
    if a.exists() and b.exists():
        result['baseline_contacts_max_state_difference']=float(np.abs(np.load(a)['state']-np.load(b)['state']).max())
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();result=analyze(a.root)
    content=json.dumps(result,indent=2)+'\n'
    if a.output:a.output.write_text(content)
    else:print(content)
