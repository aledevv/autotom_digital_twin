"""Read-only construction audit with independent coefficient calculations."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from pxr import Usd, UsdPhysics

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src'))
from exporterV2.drive_experiments import drive_records
from exporterV2.fruit_experiments import audit, value
from exporterV2.plant_state_legacy_backend import _audit_collider_overlaps


def inspect(directory, method):
    case = directory/'tgs-6n-preflight'
    stage = Usd.Stage.Open(str(case/'scene.usda'))
    build = json.loads((directory/'direct/build.json').read_text())
    definitions = {b['id']: b for b in build['branches']}
    effective = json.loads((case/'headless-effective.json').read_text())
    report = json.loads((case/'report.json').read_text())
    masses = {b['path']: b['mass_kg'] for b in effective['bodies']}
    static = audit(stage)
    filtered, active = _audit_collider_overlaps(stage)
    formulas = []
    for row in drive_records(stage):
        if row['axis'] != 'rotX':
            continue
        child = stage.GetPrimAtPath(row['child'])
        branch_id = value(child, 'autotom:branchId')
        if branch_id not in definitions:
            branch_id = next(name for name in sorted(definitions, key=len, reverse=True)
                             if child.GetName().startswith(name+'_Link_'))
        branch = definitions[branch_id]
        parent = definitions[branch['parent']]
        radius, length = 2*branch['radius'], 2*branch['height']
        modulus = branch.get('young_modulus', 3e9 if method == 'main' else 20e9)
        ratio = branch.get('damping_ratio', 7 if method == 'main' else 4)
        mass = masses[row['child']]
        ei = modulus * math.pi * radius**4 / 4
        k = max(ei/length, .001)
        pivot_inertia = mass*(3*radius**2+length**2)/12 + mass*(length/2)**2
        d = 2*ratio*math.sqrt(k*pivot_inertia)
        if row['role'] != 'rachis_internal':
            pr, pl = 2*parent['radius'], 2*parent['height']
            pe = parent.get('young_modulus', (3e9 if method == 'main' else 20e9)
                            if parent.get('physics_profile') == 'truss' else (70e6 if method == 'main' else 30e6))
            pei = pe*math.pi*pr**4/4
            attachment = 1/(.5*pl/pei + .5*length/ei)
            if branch.get('attachment_stiffness_rad') is not None:
                attachment = branch['attachment_stiffness_rad']
            d *= math.sqrt(attachment/k)
            k = attachment
        scale = branch.get('drive_stiffness_scale', 1.)
        k *= scale*math.pi/180
        d *= math.sqrt(scale)*math.pi/180
        formulas.append({**row, 'expected_stiffness': k, 'expected_damping': d,
                         'stiffness_relative_error': abs(row['stiffness']/k-1),
                         'damping_relative_error': abs(row['damping']/d-1),
                         'radius_m': radius, 'length_m': length, 'modulus_pa': modulus,
                         'nominal_damping_ratio': ratio, 'pivot_inertia_for_formula': pivot_inertia})
    inertia_checks = []
    for body in effective['bodies']:
        eig = np.linalg.eigvalsh(np.asarray(body['inertia']).reshape(3,3))
        inertia_checks.append({'body':body['path'], 'eigenvalues':eig.tolist(),
                               'positive':bool(np.all(eig>0)),
                               'triangle_inequality':bool(eig[-1] <= eig[0]+eig[1]+1e-12)})
    return {'static_audit':static, 'formula_checks':formulas, 'inertia_checks':inertia_checks,
            'runtime_mass_errors':effective['errors'],
            'filtered_overlap_count':len(filtered), 'active_conservative_overlaps':active,
            'reset_projection_m':report['max_reset_projection_m'],
            'initialization_events':report['events'],
            'settings':{k:effective[k] for k in ('scene','solver_iterations','gravity_direction','gravity_magnitude_mps2')},
            'caveats':['Overlap analysis is exact for capsules/spheres, conservative for cylinders.',
                       'Drive gains are branch-level values; attachment stiffness uses nominal parent dimensions.',
                       'Damping uses local link pivot inertia, not total moving-subtree inertia.',
                       'Positive inertias and coincident rest frames do not prove runtime stability.']}


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--pair-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    result={m:inspect(a.pair_dir/m,m) for m in ('main','v23')}
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    for m,r in result.items():
        print(m,'static_errors',r['static_audit']['errors'],
              'max_coefficient_relative_error',max(max(x['stiffness_relative_error'],x['damping_relative_error']) for x in r['formula_checks']),
              'active_overlaps',len(r['active_conservative_overlaps']))
