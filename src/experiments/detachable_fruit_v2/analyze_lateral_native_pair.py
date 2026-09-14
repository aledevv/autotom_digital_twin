"""Measure native gesture response without inferring mouse force in newtons."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def analyze(root):
    result={'cases':{}}
    rays=[]
    for name in ['mobile','fixed']:
        directory=root/name
        r=json.loads((directory/'report.json').read_text())
        events=[json.loads(l) for l in (directory/'headless-interaction.jsonl').read_text().splitlines()]
        begin=next(e for e in events if e['event']=='begin');end=next(e for e in events if e['event']=='end')
        moves=[e for e in events if e['event']=='move'];rays.append(np.array([e['direction'] for e in moves]))
        traces=sorted(directory.glob('headless-trace-*.npz'))
        paths=list(np.load(traces[0])['paths']);state=np.concatenate([np.load(f)['state'] for f in traces])
        times=np.load(directory/'headless-metrics.npz')['samples'][:,0]
        i=paths.index(r['force_target']['fruit'])
        support=next(j for j,p in enumerate(paths) if str(p).endswith('rachis_Link_01'))
        start=int(np.argmin(abs(times-begin['time_s'])))
        stop=int(np.argmin(abs(times-end['time_s'])))
        trajectory=state[start:stop+1]
        breaks=[e for e in r['events'] if e.get('fruit')==r['force_target']['fruit']]
        target_delta=np.array(moves[-1]['target_point'])-np.array(begin['point'])
        c={k:r[k] for k in ['status','errors','simulated_seconds','tail','performance']}
        c.update(evidence=str(directory),scene_sha256=hashlib.sha256((directory/'scene.usda').read_bytes()).hexdigest(),
                 native_detachment_passed=bool(breaks),breaks=breaks,selection=r['interaction']['selection'],
                 native_settings=r['interaction']['native_settings'],native_force_newtons=None,
                 gesture_end=end,commanded_target_displacement_m=target_delta.tolist(),
                 fruit_center_end_displacement_m=(state[stop,i,:3]-state[start,i,:3]).tolist(),
                 fruit_center_max_displacement_m=float(np.linalg.norm(trajectory[:,i,:3]-state[start,i,:3],axis=1).max()),
                 support_end_displacement_m=(state[stop,support,:3]-state[start,support,:3]).tolist(),
                 support_max_displacement_m=float(np.linalg.norm(trajectory[:,support,:3]-state[start,support,:3],axis=1).max()),
                 final_fruit_center_displacement_m=(state[-1,i,:3]-state[start,i,:3]).tolist())
        assert begin['rigid_body']==r['force_target']['fruit']
        result['cases'][name]=c
    if rays[0].shape==rays[1].shape:
        result['max_ray_direction_difference']=float(np.abs(rays[0]-rays[1]).max())
    else:result['ray_sample_counts']=[len(r) for r in rays]
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();a.output.write_text(json.dumps(analyze(a.root),indent=2)+'\n')
