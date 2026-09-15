"""Summarize contact evidence and the selected capsule pair's physical separation."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import numpy as np
from pxr import Gf, Usd, UsdGeom

ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'src'))
from exporterV2.plant_state_legacy_backend import _collider_records, _segment_distance
from exporterV2.fruit_diagnostics import rotate


def summarize(directory):
    config=json.loads((directory/'config.json').read_text())
    gui=bool(config.get('gui'));prefix='gui' if gui else 'headless'
    report=json.loads((directory/('gui-report.json' if gui else 'report.json')).read_text())
    result=dict(case=str(directory),variant=config['collision_variant'],status=report['status'],
        seconds=report.get('simulated_seconds'),errors=report.get('errors'),tail=report.get('tail'),
        timing=report.get('timing'),performance=report.get('performance'),events=report.get('events'),
        scene_sha256=config['scene_sha256'])
    counts=Counter();points=0;first=None
    for line in (directory/'contacts.jsonl').open():
        row=json.loads(line)
        for c in row['contacts']:
            if c['points']:
                pair=tuple(sorted((c['body_a'],c['body_b'])))
                counts[pair]+=1;points+=len(c['points'])
                if first is None:first=dict(time_s=row['time_s'],pair=pair)
    result.update(contact_pairs=[dict(bodies=k,reports=v) for k,v in counts.items()],contact_points_reported=points,first_contact=first)
    if config.get('branch_drag'):
        d=config['branch_drag'];stage=Usd.Stage.Open(str(directory/'scene.usda'));cache=UsdGeom.XformCache()
        shapes={s['path']:s for s in _collider_records(stage)}
        endpoints=[]
        for suffix in ('a','b'):
            s=shapes[d['collider_'+suffix]];assert s['shape']=='capsule'
            inv=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(d['body_'+suffix])).GetInverse()
            endpoints.append([np.asarray(inv.Transform(p)) for p in (s['start'],s['end'])])
        chunks=sorted(directory.glob(prefix+'-trace-*.npz'))
        paths=np.load(chunks[0])['paths'].tolist();ids=[paths.index(d['body_'+x]) for x in ('a','b')]
        values=np.concatenate([np.load(p)['state'][:,ids,:7] for p in chunks]);gaps=[]
        for sample in values:
            world=[[Gf.Vec3d(*(sample[k,:3]+rotate(sample[k,3:7],point))) for point in endpoints[k]] for k in (0,1)]
            gaps.append(_segment_distance(*world[0],*world[1])-d['radii_sum'])
        times=np.arange(1,len(gaps)+1)/config['hz'];gaps=np.asarray(gaps)
        np.savez_compressed(directory/'selected-pair-gap.npz',time_s=times,gap_m=gaps)
        result.update(selected_pair_min_gap_mm=float(gaps.min()*1000),
            selected_pair_drag_min_gap_mm=float(gaps[(times>=30)&(times<=40)].min()*1000),
            selected_pair_contact_reports=counts[tuple(sorted((d['body_a'],d['body_b'])))])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('cases',nargs='+',type=Path);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();rows=[summarize(d) for d in a.cases];a.output.write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps([{k:r.get(k) for k in ('case','status','selected_pair_drag_min_gap_mm','selected_pair_contact_reports')} for r in rows]))
