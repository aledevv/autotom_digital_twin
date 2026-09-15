"""Analytic hanging-mass control: immobile support, external joint, no contacts/input."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from pxr import Usd, UsdPhysics

ROOT = Path(__file__).resolve().parents[3]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    source=ROOT/'artifacts/detachable_fruit_v2/2026-09-11/support-ablation/main-rigid'
    variants=[('tgs32-10g','TGS',32,.01),('tgs32-20g','TGS',32,.02),
              ('tgs64-10g','TGS',64,.01),('pgs32-20g','PGS',32,.02)]
    for name,solver,iterations,mass in variants:
        out=a.output/name
        subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_rank_diagnosis.py')),
                        '--source',str(source),'--output',str(out),'--no-contacts'],check=True)
        stage=Usd.Stage.Open(str(out/'scene.usda'));c=json.loads((out/'config.json').read_text());changes=[]
        def set_attr(attr,v):
            before=attr.Get();attr.Set(v);changes.append(dict(path=str(attr.GetPath()),before=str(before),after=str(attr.Get())))
        for prim in stage.Traverse():
            if prim.GetName().endswith('_tomato'):
                set_attr(UsdPhysics.MassAPI(prim).GetMassAttr(),mass)
                c['expected_body_properties'][str(prim.GetPath())]['mass_kg']=mass
            for prop in ('physxArticulation:solverPositionIterationCount','physxRigidBody:solverPositionIterationCount'):
                attr=prim.GetAttribute(prop)
                if attr and attr.HasAuthoredValueOpinion():set_attr(attr,iterations)
            if prim.IsA(UsdPhysics.Scene):set_attr(prim.GetAttribute('physxScene:solverType'),solver)
        stage.GetRootLayer().Save()
        c.update(duration=5.,solver=solver,art_position=iterations,fruit_position=iterations,force_target=None,
                 record_articulation_dofs=True)
        c['scene_sha256']=hashlib.sha256((out/'scene.usda').read_bytes()).hexdigest()
        c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        c['break_scaling_probe']=dict(changes=changes,weight_n=mass*9.81,
            prediction='TGS32 20g and TGS64 10g break if weight is multiplied by position iterations; TGS32 10g and PGS32 20g survive.')
        (out/'config.json').write_text(json.dumps(c,indent=2)+'\n')

if __name__=='__main__':main()
