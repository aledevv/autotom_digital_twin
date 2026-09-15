"""Compare observed break events with a preregistered iteration-scaled impulse hypothesis."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('roots',nargs='+',type=Path)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();results=[]
    for root in a.roots:
        for d in sorted(root.iterdir()):
            if not (d/'report.json').exists():continue
            c=json.loads((d/'config.json').read_text());r=json.loads((d/'report.json').read_text())
            eff=json.loads((d/'headless-effective.json').read_text())
            zs=[np.load(f) for f in sorted(d.glob('headless-trace-*.npz'))]
            if not zs:continue
            paths=list(zs[0]['paths']);s=np.concatenate([z['state'] for z in zs]);dt=1/c['hz']
            counts=max(c['art_position'],c['fruit_position']);first=[];peaks={}
            for b in eff['bodies']:
                if b['role']!='fruit':continue
                path=b['path'];v=np.vstack([np.zeros(3),s[:,paths.index(path),7:10]])
                forces=b['mass_kg']*(np.diff(v,axis=0)/dt-[0,0,-9.81])
                norm=np.linalg.norm(forces,axis=1)
                # Compare only the original intact phase for each fruit.
                event=next((e for e in r['events'] if e['fruit']==path),None)
                if event:norm=norm[:round(event['time_s']/dt)]
                multiplier=counts if c['solver']=='TGS' else 1
                crossings=np.flatnonzero(norm*multiplier>c['break_force'])
                if len(crossings):first.append(dict(step=int(crossings[0]+1),fruit=path))
                peaks[path]=dict(step_average_resultant_peak_n=float(max(norm)),scaled_peak_n=float(max(norm)*multiplier))
            first_step=min((e['step'] for e in first),default=None)
            predicted={e['fruit'] for e in first if e['step']==first_step}
            events=r['events'];observed_step=round(min(e['time_s'] for e in events)/dt) if events else None
            observed={e['fruit'] for e in events if round(e['time_s']/dt)==observed_step}
            results.append(dict(case=str(d),status=r['status'],solver=c['solver'],position_iterations=counts,
                observed_step=observed_step,predicted_step=first_step,observed_fruits=sorted(observed),predicted_fruits=sorted(predicted),
                first_event_matches=first_step==observed_step and observed==predicted,peaks=peaks,
                errors=r['errors'],report_sha256=hashlib.sha256((d/'report.json').read_bytes()).hexdigest()))
    a.output.write_text(json.dumps(dict(results=results,limitations=[
        'Assumes zero initial COM velocity and gravity -9.81 Z, as authored in these controls.',
        'Velocity-based estimate includes any other non-gravity forces and is not a constraint-force sensor.',
        'Matching events supports the scaling hypothesis, not source-level proof of an engine bug.']),indent=2)+'\n')
    for r in results:print(Path(r['case']).name,r['observed_step'],r['predicted_step'],r['first_event_matches'])

if __name__=='__main__':main()
