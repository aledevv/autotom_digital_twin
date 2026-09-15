"""Change only fruit break thresholds in a verified prepared scene."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

from pxr import Usd, UsdPhysics

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src'))
from exporterV2.fruit_experiments import attachment_records, audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--threshold', type=float, choices=(3., 4.), required=True)
    parser.add_argument('--test', choices=('rest', 'native'), required=True)
    parser.add_argument('--mouse-coefficient', type=float, choices=(10., 25., 50.))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    scene = args.source / 'scene.usda'
    stage = Usd.Stage.Open(Usd.Stage.Open(str(scene)).Flatten())
    before = {str(a.GetPath()): str(a.Get()) for p in stage.Traverse() for a in p.GetAttributes()}
    records = attachment_records(stage)
    assert records
    for record in records:
        UsdPhysics.Joint(stage.GetPrimAtPath(record['joint'])).GetBreakForceAttr().Set(args.threshold)
    after = {str(a.GetPath()): str(a.Get()) for p in stage.Traverse() for a in p.GetAttributes()}
    changes = [dict(path=k, before=v, after=after[k]) for k, v in before.items() if v != after[k]]
    assert len(changes) == len(records)
    assert all(x['path'].endswith('.physics:breakForce') for x in changes)
    check = audit(stage)
    assert not check['errors'], check['errors']
    stage.GetRootLayer().Export(str(args.output / 'scene.usda'))
    config = json.loads((args.source / 'config.json').read_text())
    if args.mouse_coefficient is not None:
        config['mouse_coefficient_trial'] = dict(before=config.get('mouse_force_coefficient'),
                                               after=args.mouse_coefficient)
        config.update(mouse_force_coefficient=args.mouse_coefficient,
                      allow_experimental_native_coefficient=args.mouse_coefficient > 10.)
    config.update(run_dir=str(args.output.resolve()), break_force=args.threshold,
                  duration=60., gui=False, gui_real_time=False, arm_after_settle=False)
    if args.test == 'rest':
        config.update(force_target=None)
        config.pop('native_recording', None)
    else:
        assert config['interaction'] == 'native' and config['native_replay_hold_after_break']
    config['scene_sha256'] = hashlib.sha256((args.output / 'scene.usda').read_bytes()).hexdigest()
    config['threshold_trial'] = dict(source=str(args.source.resolve()), test=args.test,
        source_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(), changes=changes)
    for rel in ('src/exporterV2/fruit_diagnostics.py', 'src/exporterV2/fruit_interaction.py',
                'src/experiments/detachable_fruit_v2/prepare_break_threshold.py'):
        config['implementation_sha256'][rel] = hashlib.sha256((ROOT / rel).read_bytes()).hexdigest()
    (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')
    (args.output / 'audit.json').write_text(json.dumps(check, indent=2) + '\n')


if __name__ == '__main__':
    main()
