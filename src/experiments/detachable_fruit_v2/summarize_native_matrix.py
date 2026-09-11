"""Create a small reviewable summary; full traces and USDs remain local."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def summarize(directory):
    report = json.loads((directory/'report.json').read_text())
    config = report['config']
    effective = json.loads((directory/'headless-effective.json').read_text())
    trace = np.load(directory/'headless-trace-0000.npz')
    # Startup envelope only: all screening failures end within this first chunk.
    state = trace['state']
    support = np.array(['/TerminalBodies/' not in str(p) for p in trace['paths']])
    envelope = []
    for i, sample in enumerate(state):
        v = np.linalg.norm(sample[support, 7:10], axis=-1)
        w = np.linalg.norm(sample[support, 10:13], axis=-1)
        envelope.append({'step':i+1, 'support_max_linear_mps':float(v.max()),
                         'support_max_angular_radps':float(w.max())})
    return dict(case=directory.name, local_artifacts=str(directory),
                status=report['status'], simulated_seconds=report['simulated_seconds'],
                errors=report['errors'], first_failure=report['first_failure'],
                first_attachment_threshold=report['first_attachment_threshold'],
                break_sequence=[{'time_s':e['time_s'], 'joint':e['joint'], 'fruit':e['fruit']}
                                for e in report['events'] if e['kind']=='joint_break'],
                startup_support_envelope=envelope, final_window_metrics=report['tail'],
                source_sha256=config['source_sha256'], scene_sha256=config['scene_sha256'],
                reference_sha256=config.get('reference_sha256'),
                implementation_sha256=config['implementation_sha256'],
                changes=json.loads((directory/'changes.json').read_text()),
                effective_iterations=effective['solver_iterations'],
                effective_scene=effective['scene'], hz=report['runtime_physics_hz'],
                break_force=config['break_force'], isaac_version=config['isaac_version'],
                report_sha256=hashlib.sha256((directory/'report.json').read_bytes()).hexdigest())


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--matrix-dir', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--compact-output', type=Path, help='Small summary suitable for Git')
    a = p.parse_args()
    cases = [summarize(d) for d in sorted(a.matrix_dir.iterdir())
             if d.is_dir() and (d/'report.json').exists() and (d/'changes.json').exists()]
    a.output.write_text(json.dumps({'cases':cases, 'caveat':
        'Stop on first spontaneous break. Short failure windows are not steady-state measurements; '
        'no later collapse or GUI acceptance can be inferred.'}, indent=2)+'\n')
    if a.compact_output:
        compact = []
        for c in cases:
            d = Path(c['local_artifacts'])
            config = json.loads((d/'config.json').read_text())
            compact.append({k:c[k] for k in ('case', 'status', 'simulated_seconds',
                'break_sequence', 'startup_support_envelope', 'final_window_metrics',
                'source_sha256', 'scene_sha256', 'reference_sha256', 'report_sha256',
                'hz', 'break_force', 'isaac_version')} | {
                'configuration':{k:config[k] for k in ('variant', 'solver', 'gpu', 'art_position',
                    'art_velocity', 'fruit_position', 'fruit_velocity')},
                'local_artifacts':str(d), 'changed_attribute_count':len(c['changes']),
                'evidence_sha256':{f:hashlib.sha256((d/f).read_bytes()).hexdigest()
                    for f in ('config.json', 'changes.json', 'headless-effective.json')}})
        a.compact_output.write_text(json.dumps({'cases':compact}, indent=2)+'\n')
    for c in cases:
        print(c['case'], c['status'], c['simulated_seconds'])
