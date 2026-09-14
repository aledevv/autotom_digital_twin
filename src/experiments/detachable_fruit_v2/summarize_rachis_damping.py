"""Compare actual rachis translation, independent of reported PhysX velocities."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def summarize(directory):
    p = Path(directory)
    config = json.loads((p/'config.json').read_text())
    report = json.loads((p/'report.json').read_text())
    effective = json.loads((p/'headless-effective.json').read_text())
    chunks = sorted(p.glob('headless-trace-*.npz'))
    arrays = [np.load(c) for c in chunks]
    paths = arrays[0]['paths'].tolist()
    state = np.concatenate([a['state'] for a in arrays])
    for a in arrays:
        a.close()
    selected = [paths.index(b['path']) for b in effective['bodies'] if b['role']=='truss_rachis']
    xyz = state[:,selected,:3].astype(float)
    hz = config['hz']
    final = xyz[-min(len(xyz),5*hz):].mean(axis=0)
    # Reference is first measured physics step; no claim of exact authored-pose drop.
    first = xyz[0]
    delta = np.diff(xyz,axis=0)
    early = xyz[:min(len(xyz),5*hz)]
    return dict(case=p.name, status=report['status'], errors=report['errors'],
        simulated_s=report['simulated_seconds'], damping_scale=config.get('rachis_damping_scale',1.), stiffness_scale=config.get('rachis_stiffness_scale',1.),
        scene_sha256=config['scene_sha256'],
        report_sha256=hashlib.sha256((p/'report.json').read_bytes()).hexdigest(),
        rachis_paths=[paths[i] for i in selected],
        measured_motion_window_s=min(len(xyz)/hz,5.),
        first5s_max_speed_mps=float(np.linalg.norm(delta[:5*hz]*hz,axis=2).max()),
        first5s_max_travel_m=float(np.linalg.norm(delta[:5*hz],axis=2).sum(axis=0).max()),
        max_initial_drop_m=float((first[:,2]-early[:,:,2].min(axis=0)).max()),
        max_vertical_overshoot_m=float(np.maximum(final[:,2]-early[:,:,2].min(axis=0),0).max()) if len(xyz)>=10*hz else None,
        tail5s_max_position_range_m=float(np.linalg.norm(np.ptp(xyz[-5*hz:],axis=0),axis=1).max()) if len(xyz)>=10*hz else None,
        max_final_displacement_m=float(np.linalg.norm(final-first,axis=1).max()) if len(xyz)>=10*hz else None)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('cases',nargs='+')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=[summarize(d) for d in args.cases]
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
