"""Change only TGS external-force scheduling on the hanging-weight and rank-10 fixtures."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from pxr import Sdf, Usd

ROOT=Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    evidence=ROOT/'artifacts/detachable_fruit_v2'
    for name,source in [('hanging-20g',evidence/'break-scaling-probe/tgs32-20g'),
                        ('rank10',evidence/'mass-distribution-v1/profile10-total10')]:
        out=a.output/name
        command=[sys.executable,str(Path(__file__).with_name('prepare_rank_diagnosis.py')),
                 '--source',str(source),'--output',str(out)]
        if name=='hanging-20g':command.append('--no-contacts')
        subprocess.run(command,check=True)
        stage=Usd.Stage.Open(str(out/'scene.usda'));scene=stage.GetPrimAtPath('/World/PhysicsScene')
        attr=scene.CreateAttribute('physxScene:enableExternalForcesEveryIteration',Sdf.ValueTypeNames.Bool,custom=False)
        before=attr.Get();attr.Set(True);stage.GetRootLayer().Save()
        c=json.loads((out/'config.json').read_text())
        c['scene_sha256']=hashlib.sha256((out/'scene.usda').read_bytes()).hexdigest()
        c['force_scheduling_control']=dict(attribute=str(attr.GetPath()),before=before,after=True,
             prediction='Known static load no longer breaks; rank10 should no longer cross the iteration-scaled threshold. Rest screening only.')
        c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (out/'config.json').write_text(json.dumps(c,indent=2)+'\n')

if __name__=='__main__':main()
