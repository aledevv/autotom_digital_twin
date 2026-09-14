"""Record first breaks and gross support motion for the main-to-latest comparison."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from prepare_main_transition import CASES


def summarize(directory):
    p=Path(directory)
    config=json.loads((p/'config.json').read_text())
    report=json.loads((p/'report.json').read_text())
    effective=json.loads((p/'headless-effective.json').read_text())
    breaks=report.get('events',[])
    first_break=next((e for e in breaks if e['kind']=='joint_break'),None)
    result=dict(case=p.name,factors=CASES[p.name],status=report['status'],
        simulated_s=report['simulated_seconds'],errors=report['errors'],
        first_reported_failure=report['first_failure'],
        first_break={k:first_break.get(k) for k in ('time_s','joint','fruit','support_chain')} if first_break else None,
        break_count=len(breaks),gate=report.get('settling_gate'),
        scene_sha256=config['scene_sha256'],report_sha256=hashlib.sha256((p/'report.json').read_bytes()).hexdigest(),
        config=dict(density=config['density'],coherent_fruit=config['coherent_fruit'],
                    stiffness=config.get('rachis_stiffness_scale',1),damping=config.get('rachis_damping_scale',1),
                    arming=config.get('arm_after_settle',False),break_force=config['break_force']))
    files=sorted(p.glob('headless-trace-*.npz'))
    if files:
        values=[np.load(f) for f in files]
        paths=values[0]['paths'].tolist()
        states=np.concatenate([v['state'] for v in values])
        for v in values:
            v.close()
        support=[paths.index(b['path']) for b in effective['bodies'] if b['role'] in ('truss_rachis','pedicel')]
        xyz=states[:,support,:3].astype(float)
        displacement=np.linalg.norm(xyz-xyz[0],axis=2)
        result['support_max_displacement_from_first_step_m']=float(displacement.max())
        crossing=np.argwhere(displacement>.5)
        result['first_support_excursion_over_0_5m']=None
        if len(crossing):
            frame,body=crossing[0]
            result['first_support_excursion_over_0_5m']=dict(time_s=(int(frame)+1)/config['hz'],
                body=paths[support[int(body)]],displacement_m=float(displacement[frame,body]))
        result['tail_support_position_range_m']=(float(np.linalg.norm(np.ptp(xyz[-5*config['hz']:],axis=0),axis=1).max())
                                                  if len(xyz)>=10*config['hz'] else None)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-root',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=[summarize(args.run_root/name) for name in CASES if (args.run_root/name/'report.json').exists()
            and json.loads((args.run_root/name/'report.json').read_text()).get('status')!='running']
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    for r in result:
        print(r['case'],r['status'],round(r['simulated_s'],3),r['errors'])
