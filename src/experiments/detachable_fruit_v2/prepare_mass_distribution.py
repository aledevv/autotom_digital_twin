"""Separate total fruit mass from its spatial profile; diagnostics, not a preset."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
from pxr import Usd, UsdPhysics

ROOT = Path(__file__).resolve().parents[3]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--matrix', choices=('totals', 'localized'), default='totals')
    a = p.parse_args()
    base = ROOT / 'artifacts/detachable_fruit_v2/rank-diagnosis'
    source = base / 'control-60hz'
    reference = base / 'rank6-mass-only'
    bodies = json.loads((source / 'headless-effective.json').read_text())['bodies']
    ref = json.loads((reference / 'headless-effective.json').read_text())['bodies']
    fruits = sorted((b for b in bodies if b['role'] == 'fruit'), key=lambda b: b['path'])
    refs = {b['path']: b for b in ref}
    m10 = np.array([b['mass_kg'] for b in fruits])
    m6 = np.array([refs[b['path']]['mass_kg'] for b in fruits])
    profiles = {
        'profile10-total10': m10,
        'profile6-total6': m6,
        'profile6-total10': m6 * m10.sum() / m6.sum(),
        'profile10-total6': m10 * m6.sum() / m10.sum(),
        'reverse10-total10': m10[::-1],
        'uniform-total10': np.full(8, m10.mean()),
    }
    if a.matrix == 'localized':
        # Sorted path order is 0L,0R,1L,1R,2L,2R,3L,3R.
        expected_suffixes = [f'lat_{i}_{side}_tomato' for i in range(4) for side in ('L','R')]
        assert all(b['path'].endswith(suffix) for b,suffix in zip(fruits,expected_suffixes))
        profiles = {}
        for name, swaps in {
            'swap-proximal-distal-left': [(0,6)],
            'swap-proximal-distal-right': [(1,7)],
            'swap-proximal-distal-both': [(0,6),(1,7)],
            'swap-distal-left-right': [(6,7)],
        }.items():
            masses = m10.copy()
            for i,j in swaps:
                masses[i], masses[j] = masses[j], masses[i]
            assert np.isclose(masses.sum(),m10.sum(),rtol=0,atol=1e-15)
            profiles[name] = masses
    a.output.mkdir(parents=True, exist_ok=False)
    manifest = {}
    for name, masses in profiles.items():
        out = a.output / name
        subprocess.run([sys.executable, str(Path(__file__).with_name('prepare_rank_diagnosis.py')),
                        '--source', str(source), '--output', str(out)], check=True)
        stage = Usd.Stage.Open(str(out / 'scene.usda'))
        config = json.loads((out / 'config.json').read_text())
        changes = []
        for b, mass in zip(fruits, masses):
            attr = UsdPhysics.MassAPI(stage.GetPrimAtPath(b['path'])).GetMassAttr()
            old = attr.Get()
            attr.Set(float(mass))
            config['expected_body_properties'][b['path']]['mass_kg'] = float(mass)
            changes.append({'path': str(attr.GetPath()), 'before': old, 'after': attr.Get()})
        stage.GetRootLayer().Save()
        config.update(record_articulation_dofs=True)
        config['scene_sha256'] = hashlib.sha256((out / 'scene.usda').read_bytes()).hexdigest()
        config['implementation_sha256'][str(Path(__file__).relative_to(ROOT))] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        config['mass_distribution_control'] = dict(name=name, changes=changes,
            total_kg=float(masses.sum()), inertia_policy='unchanged: mass-only causal control, not biological replacement')
        (out / 'config.json').write_text(json.dumps(config, indent=2)+'\n')
        manifest[name] = config['mass_distribution_control']
        # The fresh common source has explicit inertias; this intervention changes only eight mass attributes.
        original = Usd.Stage.Open(str(source / 'scene.usda'))
        for b in fruits:
            assert stage.GetPrimAtPath(b['path']).GetAttribute('physics:centerOfMass').Get() == original.GetPrimAtPath(b['path']).GetAttribute('physics:centerOfMass').Get()
    (a.output / 'matrix.json').write_text(json.dumps(manifest, indent=2)+'\n')


if __name__ == '__main__':
    main()
