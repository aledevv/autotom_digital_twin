"""Rotate a recorded input gesture to a target truss frame; never edit physics."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import sys
import numpy as np
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src'))
from exporterV2.fruit_diagnostics import rotate


def pose_at(directory, prefix, body, sample):
    for path in sorted(directory.glob(prefix+'-trace-*.npz')):
        z = np.load(path)
        if sample < len(z['state']):
            names = list(z['paths'])
            if body not in names:
                rank = re.search(r'Truss_r(\d+)_', body)[1]
                matches = [x for x in names if re.fullmatch(r'/World/Stem/Truss_r'+rank+r'_.*_rachis_Link_01', x)]
                if len(matches) != 1:
                    raise ValueError(f'Expected one rachis frame for rank {rank}: {matches}')
                body = matches[0]
            return z['state'][sample, names.index(body), :7], dict(path=str(path), body=body, sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        sample -= len(z['state'])
    raise ValueError('Requested pose not recorded')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--target-rest', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    c = json.loads((a.source/'config.json').read_text())
    rec = c['native_recording']
    raw = [json.loads(x) for x in (Path(rec['source'])/'gui-native-interaction.jsonl').read_text().splitlines()]
    begin = next(x for x in raw if x['event']=='begin' and x['body']==rec['source_fruit'])
    source_rank = re.search(r'Truss_r(\d+)_', rec['source_fruit'])[1]
    target_rank = re.search(r'Truss_r(\d+)_', c['force_target'])[1]
    source_pose, source_evidence = pose_at(Path(rec['source']), 'gui', f'/World/Stem/Truss_r{source_rank}_o0_rachis_Link_01', round(begin['time_s']*60)-1)
    target_pose, target_evidence = pose_at(a.target_rest, 'headless', f'/World/Stem/Truss_r{target_rank}_o0_rachis_Link_01', round(c['force_start']*c['hz'])-1)
    def matrix(q):
        q = q.astype(float)/np.linalg.norm(q)
        return np.stack([rotate(q, x) for x in np.eye(3)], axis=1)
    transform = matrix(target_pose[3:7]) @ matrix(source_pose[3:7]).T
    assert np.allclose(transform.T@transform, np.eye(3), atol=1e-6)
    assert np.linalg.det(transform)>0.99999
    center = np.asarray(rec['reference_body_position'])
    for e in rec['events']:
        e['origin'] = (center + transform@(np.asarray(e['origin'])-center)).tolist()
        e['direction'] = (transform@np.asarray(e['direction'])).tolist()
    rec['frame_transform'] = dict(rotation=transform.tolist(), source_pose=source_pose.tolist(), target_pose=target_pose.tolist(), source_evidence=source_evidence, target_evidence=target_evidence,
        description='Rigid rotation of recorded rays around reference fruit center; runtime translation still tracks target initial center.')
    a.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(a.source/'scene.usda', a.output/'scene.usda')
    c['run_dir'] = str(a.output.resolve())
    c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (a.output/'config.json').write_text(json.dumps(c,indent=2)+'\n')


if __name__ == '__main__':
    main()
