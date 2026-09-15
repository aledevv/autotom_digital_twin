"""Prepare explicit native-runtime candidates without replacing source fruit data."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from pxr import Sdf, Usd
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.fruit_experiments import audit


def properties(stage):
    return {str(a.GetPath()):str(a.Get()) for p in stage.Traverse() for a in p.GetAttributes()}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--solver',choices=('TGS','PGS'),required=True)
    p.add_argument('--device',choices=('cpu','gpu'),required=True)
    p.add_argument('--force-scheduling',choices=('existing','on','off'),default='existing')
    p.add_argument('--test',choices=('rest','hold','native','ramp'),required=True)
    p.add_argument('--target');p.add_argument('--duration',type=float,default=60.)
    p.add_argument('--hold-force',type=float,default=2.)
    p.add_argument('--hold-seconds',type=float,default=10.)
    p.add_argument('--mouse-coefficient',type=float,default=50.)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    stage=Usd.Stage.Open(Usd.Stage.Open(str(a.source/'scene.usda')).Flatten());before=properties(stage)
    c=json.loads((a.source/'config.json').read_text())
    scene=stage.GetPrimAtPath('/World/PhysicsScene')
    scene.GetAttribute('physxScene:solverType').Set(a.solver)
    scene.GetAttribute('physxScene:enableGPUDynamics').Set(a.device=='gpu')
    if a.force_scheduling!='existing':
        scene.CreateAttribute('physxScene:enableExternalForcesEveryIteration',Sdf.ValueTypeNames.Bool,custom=False).Set(a.force_scheduling=='on')
    c.update(solver=a.solver,gpu=a.device=='gpu',duration=a.duration,run_dir=str(a.output.resolve()),gui=False,
             force_target=None,record_articulation_dofs=True,mouse_grab_mode='force',
             mouse_force_coefficient=a.mouse_coefficient,allow_experimental_native_coefficient=a.mouse_coefficient>10)
    for key in ('native_recording','drag_profile','hold_force','hold_seconds'):c.pop(key,None)
    if a.test!='rest':
        fruits=sorted(k for k in c['expected_body_properties'] if k.endswith('_tomato'))
        target=a.target or next((k for k in fruits if k.endswith('_lat_3_R_tomato')),fruits[-1])
        assert target in fruits
        c.update(force_target=target,force_start=30.,force_direction=[0.,0.,1.],interaction='com')
        if a.test=='hold':c.update(drag_profile='hold',hold_force=a.hold_force,hold_seconds=a.hold_seconds)
        if a.test=='native':
            reference=ROOT/'artifacts/detachable_fruit_v2/standard-integration/native-attempts/01-joint10/config.json'
            c.update(interaction='native',native_recording=json.loads(reference.read_text())['native_recording'])
    result=audit(stage);assert not result['errors'],result['errors']
    after=properties(stage);changes=[dict(path=k,before=before.get(k),after=after.get(k)) for k in before.keys()|after.keys() if before.get(k)!=after.get(k)]
    allowed={'physxScene:solverType','physxScene:enableGPUDynamics','physxScene:enableExternalForcesEveryIteration'}
    assert all(x['path'].split('.')[-1] in allowed for x in changes),changes
    stage.GetRootLayer().Export(str(a.output/'scene.usda'))
    c['scene_sha256']=hashlib.sha256((a.output/'scene.usda').read_bytes()).hexdigest()
    c['generic_candidate']=dict(source=str(a.source.resolve()),test=a.test,changes=changes,
        source_sha256=hashlib.sha256((a.source/'scene.usda').read_bytes()).hexdigest(),fruit_data='unchanged')
    c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    for name,value in [('config',c),('audit',result)]:
        (a.output/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')

if __name__=='__main__':main()
