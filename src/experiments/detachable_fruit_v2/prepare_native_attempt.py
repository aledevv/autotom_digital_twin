"""Prepare one of the five bounded native-detachment candidates."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Usd, UsdPhysics
from prepare_lateral_diagnosis import prepare
from prepare_lateral_force_pair import TARGET

ROOT=Path(__file__).resolve().parents[3]
LEDGER=Path(__file__).with_name('native_attempts.json')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('output',type=Path);p.add_argument('--coefficient',type=float,default=10.)
    p.add_argument('--mode',choices=['joint','force'],default='joint')
    p.add_argument('--branch-stiffness-scale',type=float,default=1.)
    p.add_argument('--branch-damping-scale',type=float,default=1.)
    a=p.parse_args()
    if not 0 < a.coefficient <= 100.:p.error('Experimental coefficient must be in (0, 100]')
    if not all(np.isfinite(x) and x>0 for x in (a.branch_stiffness_scale,a.branch_damping_scale)):
        p.error('Branch scales must be finite and positive')
    ledger=json.loads(LEDGER.read_text()) if LEDGER.exists() else dict(max_attempts=5,objective='Stable native tomato detachment on mobile lateral branch',attempts=[])
    if len(ledger['attempts'])>=5:raise ValueError('Five-attempt limit reached; no additional candidate allowed')
    source=ROOT/'artifacts/detachable_fruit_v2/standard-integration/lateral-diagnosis-v2/pgs-minute'
    reference=ROOT/'artifacts/detachable_fruit_v2/gui-freeze-rpl5yne3'
    clips=json.loads(Path(__file__).with_name('successful_native_gestures.json').read_text())
    assert hashlib.sha256((reference/'gui-native-interaction.jsonl').read_bytes()).hexdigest()==clips['source_log_sha256']
    clip=clips['gestures'][0]
    times=np.load(reference/'gui-metrics.npz')['samples'][:,0]
    sample=int(np.argmin(abs(times-clip['begin_time_s'])))
    assert abs(times[sample]-clip['begin_time_s'])<.017
    files=sorted(reference.glob('gui-trace-*.npz'))
    states=np.concatenate([np.load(f)['state'] for f in files]);paths=list(np.load(files[0])['paths'])
    center=states[sample,paths.index(clip['fruit']),:3].tolist()
    events=[dict(relative_time_s=e['time_s']-clip['begin_time_s'],event=e['event'],origin=e['origin'],direction=e['direction'])
            for e in clip['raw_events'] if e['event'] in ('begin','move','release')]
    assert events[0]['event']=='begin' and events[-1]['event']=='release'
    output=a.output.resolve();prepare(source,output,'baseline',60.)
    stage=Usd.Stage.Open(str(output/'scene.usda'))
    changes=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Joint) and '/Vegetative/Branch_' in str(prim.GetPath()):
            for axis in ('rotX','rotY'):
                for field,factor in [('stiffness',a.branch_stiffness_scale),('damping',a.branch_damping_scale)]:
                    attr=prim.GetAttribute(f'drive:{axis}:physics:{field}')
                    old=attr.Get()
                    if factor!=1.:
                        attr.Set(old*factor)
                        changes.append(dict(path=str(attr.GetPath()),old=old,new=attr.Get()))
    if changes:stage.GetRootLayer().Save()
    c=json.loads((output/'config.json').read_text());c.update(interaction='native',force_target=TARGET,force_start=30.,
        mouse_grab_mode=a.mode,mouse_force_coefficient=a.coefficient,
        allow_experimental_native_coefficient=a.coefficient>10.,
        native_recording=dict(source=str(reference),source_log_sha256=clips['source_log_sha256'],
            reference_body_position=center,source_fruit=clip['fruit'],events=events))
    c['scene_sha256']=hashlib.sha256((output/'scene.usda').read_bytes()).hexdigest()
    (output/'config.json').write_text(json.dumps(c,indent=2)+'\n')
    v=json.loads((output/'variant.json').read_text());v['config']=c;v['branch_drive_changes']=changes
    (output/'variant.json').write_text(json.dumps(v,indent=2)+'\n')
    attempt=dict(number=len(ledger['attempts'])+1,mode=a.mode,coefficient=a.coefficient,
                 configuration='PGS/GPU 60 Hz, articulation 32/4, fruit 32/1, 6 N, original mobile branch',
                 evidence=str(output),status='prepared',source_gesture=0)
    attempt.update(branch_stiffness_scale=a.branch_stiffness_scale,branch_damping_scale=a.branch_damping_scale,
                   branch_drive_changes=changes)
    ledger['attempts'].append(attempt);LEDGER.write_text(json.dumps(ledger,indent=2)+'\n')
    print(json.dumps(attempt))


if __name__=='__main__':main()
