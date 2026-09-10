"""Prepare an audited reduced native-mouse case from a freshly built reference."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'src'))


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--build-dir', type=Path, required=True)
    p.add_argument('--run-dir', type=Path, required=True)
    p.add_argument('--runtime', choices=('main', 'candidate'), default='main')
    p.add_argument('--duration', type=float, default=20)
    p.add_argument('--gui', action='store_true')
    p.add_argument('--break-force', type=float, default=6.)
    p.add_argument('--diagnostic-gui', action='store_true',
                   help='Observe spontaneous breaks in GUI; report remains failed')
    a = p.parse_args()
    if not math.isfinite(a.break_force) or a.break_force <= 0:
        p.error('break force must be finite and positive')
    if a.diagnostic_gui and not a.gui:
        p.error('--diagnostic-gui requires --gui')
    from pxr import Usd, Sdf, UsdPhysics
    from exporterV2.fruit_experiments import prepare_stage, bodies_and_joints
    build = json.loads((a.build_dir/'build.json').read_text())
    source = (a.build_dir/'source.usda').resolve()
    out = a.run_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if (out/'config.json').exists():
        raise ValueError('Use a fresh run directory')
    stage = Usd.Stage.Open(Usd.Stage.Open(str(source)).Flatten())
    bodies, _ = bodies_and_joints(stage)
    # Legacy main has no diagnostic labels. These attributes do not change physics.
    for path, prim in bodies.items():
        if not prim.GetAttribute('autotom:role'):
            role = ('fruit' if '/TerminalBodies/' in path else 'internode' if '/trunk/' in path
                    else 'pedicel' if 'pedicel' in path else 'truss_rachis')
            kind = 'stem' if role == 'internode' else role
            prim.CreateAttribute('autotom:role', Sdf.ValueTypeNames.String).Set(role)
            prim.CreateAttribute('autotom:entityKind', Sdf.ValueTypeNames.String).Set(
                'terminal_body' if role == 'fruit' else 'physical_link')
            prim.CreateAttribute('autotom:branchKind', Sdf.ValueTypeNames.String).Set(kind)
            prim.CreateAttribute('autotom:branchId', Sdf.ValueTypeNames.String).Set(path.split('/')[-2])
    config = dict(scenario='stem-truss', attachment='external', breakable=True, break_force=a.break_force,
                  collisions=True, solver='TGS' if a.runtime == 'main' else 'PGS', gpu=a.runtime == 'main',
                  hz=60, art_position=32, art_velocity=4 if a.runtime == 'main' else 0,
                  fruit_position=32 if a.runtime == 'main' else 255, fruit_velocity=1 if a.runtime == 'main' else 0,
                  stiffness_scale=1., damping_ratio=4., damping_scale=1., duration=a.duration,
                  acceptance='functional', force_target=None, force_start=30., interaction='native',
                  mouse_grab_mode='joint', mouse_force_coefficient=10., record_native_mouse=True,
                  camera_eye=[1.1, 1.1, 1.0], camera_target=[0., 0., .3],
                  run_dir=str(out), source_usd=str(source), source_sha256=sha(source),
                  method=build['method'], selected_truss=build['selected'],
                  input_sha256=build['input_sha256'], current_commit=subprocess.check_output(
                      ['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                  provenance=json.loads((a.build_dir.parent/'provenance.json').read_text()),
                  gui=a.gui, preserve_source_drives=True,
                  observe_spontaneous_breaks=a.diagnostic_gui)
    # prepare_stage applies a damping multiplier relative to 4; 4 means identity
    # even for the reference main whose physical source ratio is 7.
    report = prepare_stage(stage, config)
    assert not report['reanchored_joints'], report['reanchored_joints']
    assert report['body_count'] == len(bodies), 'Fixture unexpectedly lost bodies'
    assert len(report['attachments']) == build['selected']['fruit_count']
    assert not any(float(prim.GetAttribute('autotom:aggregatedLeafVisualMassKg').Get() or 0)
                   for prim in stage.Traverse()), 'Removed leaves still contribute mass'
    if report['errors']:
        raise ValueError(report['errors'])
    stage.GetRootLayer().Export(str(out/'scene.usda'))
    config['scene_sha256'] = sha(out/'scene.usda')
    files = ['src/exporterV2/fruit_experiments.py', 'src/exporterV2/fruit_diagnostics.py',
             'src/exporterV2/native_drag_observer.py', 'src/exporterV2/isaac_app.py',
             str(Path(__file__).resolve().relative_to(ROOT))]
    config['implementation_sha256'] = {f:sha(ROOT/f) for f in files}
    config['isaac_version'] = (Path.home()/'isaacsim/VERSION').read_text().strip()
    (out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    (out/'audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print('[PREPARED]',out,'bodies',report['body_count'],'fruits',len(report['attachments']))


if __name__ == '__main__':
    main()
