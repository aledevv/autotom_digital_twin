"""Fresh main/density2000 native GUI run, no deadline and no automatic kill."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--full-truss', action='store_true', help='Complete main reference truss with eight fruits and full stem')
    parser.add_argument('--source-case', type=Path, help='Prepared and screened scene, including its loaded mass/inertia report')
    parser.add_argument('--disable-native-observer', action='store_true')
    parser.add_argument('--observe-spontaneous-breaks', action='store_true', help='Record spontaneous breaks without ending the manual observation')
    parser.add_argument('--coherent-fruit', action='store_true', help='Input-size fruit; support density defaults to 1000 kg/m3')
    parser.add_argument('--support-density', type=int, choices=(1000,2000,20000), help='Override support density for isolated comparisons')
    parser.add_argument('--arm-after-settle',action='store_true')
    parser.add_argument('--fruit-break-force', type=float)
    parser.add_argument('--rachis-stiffness-scale', type=float, default=1.)
    parser.add_argument('--rachis-damping-scale', type=float, default=1.)
    args = parser.parse_args()
    os.chdir(ROOT)
    out = Path(tempfile.mkdtemp(prefix='gui-freeze-', dir=ROOT/'artifacts/detachable_fruit_v2'))
    subprocess.run([sys.executable, str(Path(__file__).with_name('prepare_support_matrix.py')),
                    '--source-case', (str(args.source_case) if args.source_case else
                                      ('artifacts/detachable_fruit_v2/2026-09-10/native-comparison/main/tgs-6n-preflight'
                                      if args.full_truss else 'artifacts/detachable_fruit_v2/2026-09-11/support-ablation/main-rachis')),
                    '--run-dir', str(out), '--density', str(args.support_density or (1000 if args.coherent_fruit else 2000)), '--gui', '--duration', '60', '--rachis-damping-scale', str(args.rachis_damping_scale), '--rachis-stiffness-scale', str(args.rachis_stiffness_scale),
                    *(['--coherent-fruit'] if args.coherent_fruit else []),
                    *(['--arm-after-settle'] if args.arm_after_settle else []),
                    *(['--fruit-break-force', str(args.fruit_break_force)] if args.fruit_break_force is not None else [])], check=True)
    config_path = out/'config.json'
    config = json.loads(config_path.read_text())
    if args.observe_spontaneous_breaks:
        config['observe_spontaneous_breaks'] = True
        config_path.write_text(json.dumps(config, indent=2)+'\n')
    command = ['/home/alessandro/isaacsim/python.sh', 'src/exporterV2/isaac_app.py',
               '--usd', str(out/'scene.usda'), '--physics-preset', 'flexible',
               '--interactive-physics-hz', str(config['hz']), '--fruit-experiment', str(out/'config.json'),
               '--gui-until-close']
    if args.disable_native_observer:
        command.append('--disable-native-observer')
    manifest = dict(command=command, cwd=str(ROOT), started_unix_s=time.time(),
        git_head=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
        source_hashes={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in [*(ROOT/'src/exporterV2').glob('*.py'),
                                 Path(__file__).resolve(), Path(__file__).with_name('prepare_support_matrix.py').resolve()]})
    (out/'launch.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'RUN_DIR={out}', flush=True)
    print('Close the window or press Ctrl+C. No automatic time limit. If frozen, wait at least 6 seconds for diagnostics.', flush=True)
    with (out/'isaac.log').open('w') as log:
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        while True:
            try:
                code = child.wait()
                break
            except KeyboardInterrupt:
                print('Forwarding Ctrl+C to Isaac; supervisor remains active during cleanup.', flush=True)
                os.killpg(child.pid, signal.SIGINT)
    (out/'process.json').write_text(json.dumps(dict(returncode=code, ended_unix_s=time.time()), indent=2)+'\n')
    print(f'Isaac exit={code}; evidence: {out}', flush=True)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
