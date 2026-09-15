"""Open a prepared experiment with native input, fresh evidence and manual close."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import time

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('source',type=Path)
    p.add_argument('--real-time', action='store_true', help='Pace GUI physics to wall time; timestep stays unchanged')
    a=p.parse_args();source=a.source.resolve()
    config=json.loads((source/'config.json').read_text())
    scene=source/'scene.usda'
    assert hashlib.sha256(scene.read_bytes()).hexdigest()==config['scene_sha256']
    out=Path(tempfile.mkdtemp(prefix='gui-',dir=source.parent))
    shutil.copyfile(scene,out/'scene.usda')
    config.update(run_dir=str(out),gui=True,duration=60.,observe_spontaneous_breaks=False,
                  force_target=None)
    config['gui_real_time'] = a.real_time
    config['source_implementation_sha256'] = dict(config.get('implementation_sha256', {}))
    implementation = set(config.get('implementation_sha256', {})) | {
        'src/exporterV2/realtime_pacing.py', 'src/exporterV2/fruit_diagnostics.py',
        'src/experiments/detachable_fruit_v2/run_prepared_gui.py'}
    config['implementation_sha256'] = {
        name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
        for name in sorted(implementation) if (ROOT / name).is_file()}
    # GUI validation is driven solely by the user, never by the headless replay.
    config.pop('native_recording', None)
    (out/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    cmd=[str(Path.home()/'isaacsim/python.sh'),str(ROOT/'src/exporterV2/isaac_app.py'),
         '--usd',str(out/'scene.usda'),'--physics-preset','flexible',
         '--interactive-physics-hz',str(config['hz']),'--duration','60',
         '--fruit-experiment',str(out/'config.json'),'--gui-until-close']
    (out/'launch.json').write_text(json.dumps(dict(command=cmd,source=str(source),started_unix_s=time.time()),indent=2)+'\n')
    print('RUN_DIR='+str(out),flush=True)
    with (out/'isaac.log').open('w') as log:
        process=subprocess.Popen(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        code=process.wait()
    (out/'process.json').write_text(json.dumps(dict(returncode=code,ended_unix_s=time.time()),indent=2)+'\n')
    print('Isaac exit='+str(code),flush=True)


if __name__=='__main__':main()
