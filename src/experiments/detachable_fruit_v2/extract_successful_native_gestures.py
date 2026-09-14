"""Extract successful recorded GUI gestures and compare their ray trajectories."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def extract(source):
    log=source/'gui-native-interaction.jsonl'
    report_path=source/'gui-report.json'
    config=json.loads((source/'config.json').read_text())
    report=json.loads(report_path.read_text())
    events=[json.loads(line) for line in log.read_text().splitlines()]
    result=dict(source=str(source),source_log_sha256=hashlib.sha256(log.read_bytes()).hexdigest(),
                source_report_sha256=hashlib.sha256(report_path.read_bytes()).hexdigest(),
                settings={k:config.get(k) for k in ['solver','gpu','hz','art_position','art_velocity','fruit_position','fruit_velocity','break_force','mouse_grab_mode','mouse_force_coefficient']},
                projection_definition='Intersection of each recorded ray with the plane through the initial hit point, normal to the initial ray. Diagnostic displacement, not native internal target or force.',
                slow_test=dict(displacement_m=.2,duration_s=5.,speed_mps=.04),gestures=[])
    for broken in report['events']:
        if broken.get('kind')!='joint_break' or not broken.get('user_target'):continue
        i=max(i for i,e in enumerate(events) if e['event']=='begin' and e['time_s']<=broken['time_s'])
        begin=events[i]
        if begin.get('body')!=broken['fruit']:raise ValueError('break does not match active pick')
        j=next((j for j in range(i+1,len(events)) if events[j]['event'] in ('release','begin')),len(events))
        clip=events[i:j+1] if j<len(events) and events[j]['event']=='release' else events[i:j]
        normal=np.array(begin['direction']);hit=np.array(begin['point']);projected=[]
        for e in clip:
            if e['time_s']>broken['time_s'] or e['event'] not in ('begin','move'):continue
            origin=np.array(e['origin']);direction=np.array(e['direction']);den=direction@normal
            if abs(den)<1e-6:raise ValueError('ray parallel to diagnostic plane')
            t=(hit-origin)@normal/den
            if t<=0:raise ValueError('diagnostic plane behind ray')
            point=origin+t*direction
            projected.append(dict(time_from_begin_s=e['time_s']-begin['time_s'],displacement_m=(point-hit).tolist()))
        speeds=[]
        for a,b in zip(projected,projected[1:]):
            dt=b['time_from_begin_s']-a['time_from_begin_s']
            if dt>0:speeds.append(float(np.linalg.norm(np.array(b['displacement_m'])-a['displacement_m'])/dt))
        result['gestures'].append(dict(fruit=broken['fruit'],joint=broken['joint'],
            begin_time_s=begin['time_s'],break_time_s=broken['time_s'],
            seconds_to_break=broken['time_s']-begin['time_s'],
            projected_displacement_at_last_prebreak_sample_m=float(np.linalg.norm(projected[-1]['displacement_m'])),
            peak_projected_speed_mps=max(speeds,default=0.),raw_events=clip,projected_samples=projected,
            replay_limit='Earlier interactions changed the source scene before this pick. An isolated retargeted replay is not an exact reproduction of the full original session.'))
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.write_text(json.dumps(extract(a.source),indent=2)+'\n')
