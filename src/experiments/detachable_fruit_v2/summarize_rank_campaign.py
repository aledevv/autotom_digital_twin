"""Summarize available reports without treating pending cases as passing."""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    rows = []
    for directory in sorted(a.root.iterdir()):
        config_file = directory / 'config.json'
        if not config_file.exists():
            continue
        config = json.loads(config_file.read_text())
        if config.get('preparation_only'):
            continue
        report_file = directory / 'report.json'
        if not report_file.exists():
            rows.append(dict(case=directory.name, status='pending'))
            continue
        report = json.loads(report_file.read_text())
        status = report['status']
        process_file = directory / 'process.json'
        if status == 'running' and process_file.exists():
            status = json.loads(process_file.read_text()).get('status', 'runtime_error')
        errors = report.get('errors')
        if status == 'runtime_error' and not errors:
            log_file = directory / 'isaac.log'
            errors = [line.split('loader failed:', 1)[1].strip() for line in log_file.read_text(errors='replace').splitlines() if 'loader failed:' in line][:1]
        rows.append(dict(case=directory.name, status=status, errors=errors,
            force_target=config.get('force_target'),
            simulated_seconds=report.get('simulated_seconds'), ranks=config['rank_campaign']['ranks'],
            fruit_mass_g_by_rank=config['rank_campaign']['fruit_mass_g_by_rank'],
            events=[{k:e.get(k) for k in ('time_s','fruit','continuity_passed')} for e in report.get('events',[])],
            first_failure=report.get('first_failure'), tail=report.get('tail'),
            report_sha256=hashlib.sha256(report_file.read_bytes()).hexdigest(),
            scene_sha256=config['scene_sha256']))
    a.output.write_text(json.dumps(rows, indent=2)+'\n')
    print([(x['case'],x['status']) for x in rows])


if __name__ == '__main__':
    main()
