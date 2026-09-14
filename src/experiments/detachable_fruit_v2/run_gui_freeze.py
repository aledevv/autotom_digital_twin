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
    parser.add_argument('--disable-native-observer', action='store_true')
    args = parser.parse_args()
    os.chdir(ROOT)
    out = Path(tempfile.mkdtemp(prefix='gui-freeze-', dir=ROOT/'artifacts/detachable_fruit_v2'))
    subprocess.run([sys.executable, str(Path(__file__).with_name('prepare_support_matrix.py')),
                    '--source-case', 'artifacts/detachable_fruit_v2/2026-09-11/support-ablation/main-rachis',
                    '--run-dir', str(out), '--density', '2000', '--gui', '--duration', '60'], check=True)
    command = ['/home/alessandro/isaacsim/python.sh', 'src/exporterV2/isaac_app.py',
               '--usd', str(out/'scene.usda'), '--physics-preset', 'flexible',
               '--interactive-physics-hz', '60', '--fruit-experiment', str(out/'config.json'),
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
