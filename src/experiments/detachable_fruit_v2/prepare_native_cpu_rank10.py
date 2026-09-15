"""Original rank-10 fruit profile: native TGS/CPU validation, no mass compensation."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
from pxr import Usd

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--case',choices=('rest','native','com'))
    p.add_argument('--mouse-mode',choices=('joint','force'),default='joint')
    p.add_argument('--coefficient',type=float,default=10.)
    a=p.parse_args()
    if not math.isfinite(a.coefficient) or not 0<a.coefficient<=100:
        p.error('coefficient must be finite and in (0,100]')
    a.output.mkdir(parents=True,exist_ok=False)
    source=ROOT/'artifacts/detachable_fruit_v2/mass-distribution-v1/profile10-total10'
    replay_source=ROOT/'artifacts/detachable_fruit_v2/standard-integration/native-attempts/01-joint10/config.json'
    recording=json.loads(replay_source.read_text())['native_recording']
    for name in ([a.case] if a.case else ['rest','native']):
        out=a.output/name
        subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_rank_diagnosis.py')),
                        '--source',str(source),'--output',str(out)],check=True)
        stage=Usd.Stage.Open(str(out/'scene.usda'))
        stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:enableGPUDynamics').Set(False)
        stage.GetRootLayer().Save();c=json.loads((out/'config.json').read_text())
        c.update(gpu=False,duration=60.,mouse_grab_mode=a.mouse_mode,mouse_force_coefficient=a.coefficient,force_target=None,
                 allow_experimental_native_coefficient=a.coefficient>10)
        if name=='native':
            c.update(force_target='/World/TerminalBodies/Truss_r10_o0_rachis_pedicel_lat_3_R_tomato',
                     interaction='native',native_recording=recording,force_start=30.)
        if name=='com':
            c.update(force_target='/World/TerminalBodies/Truss_r10_o0_rachis_pedicel_lat_3_R_tomato',
                     interaction='com',force_direction=[0.,0.,1.],force_start=30.)
        c['scene_sha256']=hashlib.sha256((out/'scene.usda').read_bytes()).hexdigest()
        c['native_cpu_rank10']=dict(source=str(source),only_physics_change='enableGPUDynamics=false',
            replay_config_sha256=hashlib.sha256(replay_source.read_bytes()).hexdigest() if name=='native' else None)
        c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (out/'config.json').write_text(json.dumps(c,indent=2)+'\n')

if __name__=='__main__':main()
