"""Summarize mass-profile controls without interpreting velocity differences as joint force sensors."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('root', type=Path)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    manifest = json.loads((a.root / 'matrix.json').read_text())
    results = {}
    for name, controls in manifest.items():
        d = a.root / name
        if not (d / 'report.json').exists():
            results[name] = {'status': 'not_finished'}
            continue
        r = json.loads((d / 'report.json').read_text())
        eff = json.loads((d / 'headless-effective.json').read_text())
        c = json.loads((d / 'config.json').read_text())
        b = {b['path']: b for b in eff['bodies']}
        mass_errors = {k: abs(b[k]['mass_kg']-v['mass_kg']) for k,v in c['expected_body_properties'].items()}
        trace = [np.load(f) for f in sorted(d.glob('headless-trace-*.npz'))]
        state = np.concatenate([t['state'] for t in trace])
        paths = list(trace[0]['paths'])
        # A fixed first-0.2-second comparison window, not a substep peak measurement.
        estimates = {}
        for path, body in b.items():
            if body['role'] != 'fruit':
                continue
            i = paths.index(path)
            v = state[:12,i,7:10]
            resultant = body['mass_kg'] * (np.diff(v,axis=0)*c['hz'] - [0,0,-9.81])
            estimates[path] = float(np.linalg.norm(resultant,axis=1).max()) if len(resultant) else None
        results[name] = dict(status=r['status'],simulated_seconds=r['simulated_seconds'],
            errors=r['errors'],first_break=next(iter(r['events']),None),events=r['events'],
            total_kg=controls['total_kg'],max_loaded_mass_error_kg=max(mass_errors.values()),
            tail=r.get('tail'),initial_step_average_non_gravity_resultant_estimate_n=estimates,
            report_sha256=hashlib.sha256((d/'report.json').read_bytes()).hexdigest(),
            scene_sha256=c['scene_sha256'])
    result = dict(results=results,limitations=[
        'Mass-only controls intentionally preserve inertia and geometry; not physically calibrated replacements.',
        'Finite-difference resultant is a step-average estimate from COM velocities, not a joint force sensor or bound on internal solver impulses.',
        'A 20-second rest pass does not validate native interaction, whole-plant stability or biological accuracy.'])
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    for name,r in results.items():
        print(name,r['status'],r.get('simulated_seconds'))


if __name__ == '__main__':
    main()
