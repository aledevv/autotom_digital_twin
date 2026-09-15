"""Select original direct trusses from the preserved five-truss main fixture.

No v2.3 integration. Preserve retained body geometry/mass/inertia and all drives.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from pxr import Usd, UsdPhysics
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src'))
from exporterV2.fruit_experiments import audit, attachment_records


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--ranks', type=int, nargs='+', required=True)
    p.add_argument('--test', choices=('rest', 'native'), required=True)
    p.add_argument('--target-rank', type=int)
    a = p.parse_args()
    ranks = sorted(set(a.ranks))
    assert ranks and set(ranks) <= set(range(6, 11))
    source = ROOT / 'artifacts/detachable_fruit_v2/multiple-trusses/five-screen'
    reference = ROOT / 'artifacts/detachable_fruit_v2/generic-truss-search/pgs-3n-native'
    a.output.mkdir(parents=True, exist_ok=False)
    stage = Usd.Stage.Open(Usd.Stage.Open(str(source / 'scene.usda')).Flatten())
    before = {str(x.GetPath()): str(x.Get()) for prim in stage.Traverse() for x in prim.GetAttributes()}
    removed = []
    for prim in list(stage.Traverse()):
        if not prim:
            continue
        match = re.search(r'Truss_r(\d+)_', str(prim.GetPath()))
        if match and int(match[1]) not in ranks:
            removed.append(str(prim.GetPath()))
            stage.RemovePrim(prim.GetPath())
    filters = []
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            old = rel.GetTargets()
            new = [x for x in old if stage.GetPrimAtPath(x.GetPrimPath())]
            if new != old:
                assert 'filter' in rel.GetName().lower() or 'collection' in rel.GetName().lower(), (rel, old)
                rel.SetTargets(new)
                filters.append(dict(path=str(rel.GetPath()), before=list(map(str, old)), after=list(map(str, new))))
    scene = stage.GetPrimAtPath('/World/PhysicsScene')
    scene.GetAttribute('physxScene:solverType').Set('PGS')
    scene.GetAttribute('physxScene:enableGPUDynamics').Set(False)
    for r in attachment_records(stage):
        UsdPhysics.Joint(stage.GetPrimAtPath(r['joint'])).GetBreakForceAttr().Set(3.)
    after = {str(x.GetPath()): str(x.Get()) for prim in stage.Traverse() for x in prim.GetAttributes()}
    changes = [dict(path=k, before=before[k], after=v) for k, v in after.items() if before[k] != v]
    allowed = {'physxScene:solverType', 'physxScene:enableGPUDynamics', 'physics:breakForce'}
    assert all(x['path'].split('.')[-1] in allowed for x in changes), changes
    effective = json.loads((source / 'headless-effective.json').read_text())
    retained = [b for b in effective['bodies'] if stage.GetPrimAtPath(b['path'])]
    assert len(retained) == 10 + 20 * len(ranks)
    check = audit(stage)
    assert not check['errors'], check['errors']
    stage.GetRootLayer().Export(str(a.output / 'scene.usda'))
    c = json.loads((source / 'config.json').read_text())
    c.update(run_dir=str(a.output.resolve()), solver='PGS', gpu=False, break_force=3., duration=60.,
             gui=False, gui_real_time=False, force_target=None, interaction='native',
             mouse_grab_mode='force', mouse_force_coefficient=50., allow_experimental_native_coefficient=True,
             native_replay_hold_after_break=True, diagnostic_all_body_speed_limit_m_s=1000.,
             record_articulation_dofs=True, arm_after_settle=False,
             expected_body_properties={b['path']: {k:b[k] for k in ('mass_kg','inertia','com','com_orientation')} for b in retained})
    if a.test == 'native':
        rank = a.target_rank if a.target_rank is not None else ranks[0]
        assert rank in ranks
        target = f'/World/TerminalBodies/Truss_r{rank}_o0_rachis_pedicel_lat_3_L_tomato'
        assert stage.GetPrimAtPath(target)
        c.update(force_target=target, force_start=30., native_recording=json.loads((reference/'config.json').read_text())['native_recording'])
    c['rank_campaign'] = dict(ranks=ranks, test=a.test, source=str(source),
        source_sha256=hashlib.sha256((source/'scene.usda').read_bytes()).hexdigest(),
        source_effective_sha256=hashlib.sha256((source/'headless-effective.json').read_bytes()).hexdigest(),
        removed_roots=removed, filtered_relationships=filters, retained_attribute_changes=changes,
        fruit_mass_g_by_rank={rank: sum(b['mass_kg']*1000 for b in retained if b['role']=='fruit' and f'Truss_r{rank}_' in b['path']) for rank in ranks},
        construction='main-derived; original rank-specific fruit data; support density 20000 kg/m3 remains artificial')
    c['scene_sha256'] = hashlib.sha256((a.output/'scene.usda').read_bytes()).hexdigest()
    for rel in ('src/exporterV2/fruit_diagnostics.py','src/exporterV2/fruit_interaction.py','src/experiments/detachable_fruit_v2/prepare_rank_campaign.py'):
        c['implementation_sha256'][rel] = hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
    for name, data in [('config',c),('audit',check)]:
        (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
    print(a.output)


if __name__ == '__main__':
    main()
