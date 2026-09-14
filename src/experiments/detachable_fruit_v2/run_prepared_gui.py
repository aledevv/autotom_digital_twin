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
    a=p.parse_args();source=a.source.resolve()
    config=json.loads((source/'config.json').read_text())
    scene=source/'scene.usda'
    assert hashlib.sha256(scene.read_bytes()).hexdigest()==config['scene_sha256']
    out=Path(tempfile.mkdtemp(prefix='gui-',dir=source.parent))
    shutil.copyfile(scene,out/'scene.usda')
    config.update(run_dir=str(out),gui=True,duration=60.,observe_spontaneous_breaks=False,
                  force_target=None)
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
