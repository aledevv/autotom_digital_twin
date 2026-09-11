"""Verify runtime body-property preservation and summarize support ablations."""
import argparse
import hashlib
import json
from pathlib import Path


def summarize(directory):
    config = json.loads((directory/'config.json').read_text())
    report = json.loads((directory/'report.json').read_text())
    effective = json.loads((directory/'headless-effective.json').read_text())
    source = Path(config['source_usd']).parent
    original = json.loads((source/'headless-effective.json').read_text())
    by_path = {b['path']:b for b in original['bodies']}
    preserved = {b['path']:{k:b[k]==by_path[b['path']][k]
                 for k in ('mass_kg','inertia','com','com_orientation')} for b in effective['bodies']}
    if not all(all(v.values()) for v in preserved.values()):
        raise ValueError(f'Runtime body properties changed: {directory}: {preserved}')
    return {'case':directory.name, 'local_artifacts':str(directory),
            'source_sha256':config['source_sha256'], 'scene_sha256':config['scene_sha256'],
            'evidence_sha256':{f:hashlib.sha256((directory/f).read_bytes()).hexdigest()
                for f in ('config.json','report.json','headless-effective.json','ablation.json')},
            'body_count':report['body_count'], 'status':report['status'],
            'simulated_seconds':report['simulated_seconds'], 'errors':report['errors'],
            'first_failure':report['first_failure'],
            'breaks':[{'time_s':e['time_s'],'joint':e['joint']} for e in report['events'] if e['kind']=='joint_break'],
            'tail':report['tail'], 'numerical_gate_errors':report['numerical_gate_errors'],
            'runtime_properties_preserved_exactly':True, 'checked_body_count':len(preserved),
            'articulation':effective['articulation'], 'reset_projection_m':report['max_reset_projection_m']}


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    rows=[summarize(d) for d in sorted(a.directory.iterdir()) if (d/'report.json').exists()]
    a.output.write_text(json.dumps({'cases':rows},indent=2)+'\n')
    for r in rows:
        print(r['case'],r['status'],r['simulated_seconds'],'runtime properties preserved')
