"""Localize TGS break scaling to GPU and/or external-articulation attachment."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.fruit_experiments import attachment_records,joint_frame


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--case',choices=('cpu-articulation','gpu-kinematic'))
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    source=ROOT/'artifacts/detachable_fruit_v2/break-scaling-probe/tgs32-20g'
    for name in ([a.case] if a.case else ['cpu-articulation','gpu-kinematic']):
        out=a.output/name
        subprocess.run([sys.executable,str(Path(__file__).with_name('prepare_rank_diagnosis.py')),
            '--source',str(source),'--output',str(out),'--no-contacts'],check=True)
        stage=Usd.Stage.Open(str(out/'scene.usda'));c=json.loads((out/'config.json').read_text());changes=[]
        if name=='cpu-articulation':
            attr=stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:enableGPUDynamics')
            changes.append(dict(path=str(attr.GetPath()),before=attr.Get(),after=False));attr.Set(False);c['gpu']=False
        else:
            c['diagnostic_external_kinematic_supports']=['/World/FixedProbeSupport']
            record=attachment_records(stage)[0];joint=UsdPhysics.Joint(stage.GetPrimAtPath(record['joint']))
            frame=joint_frame(stage,joint,0)
            changes.append(dict(joint=record['joint'],old_body0=[str(p) for p in joint.GetBody0Rel().GetTargets()],new_body0=['/World/FixedProbeSupport']))
            support=UsdGeom.Xform.Define(stage,'/World/FixedProbeSupport').GetPrim()
            body=UsdPhysics.RigidBodyAPI.Apply(support);body.CreateKinematicEnabledAttr().Set(True)
            mass=UsdPhysics.MassAPI.Apply(support);mass.CreateMassAttr().Set(1.)
            mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0));mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1))
            support.CreateAttribute('autotom:entityKind',Sdf.ValueTypeNames.String).Set('physical_link')
            support.CreateAttribute('autotom:role',Sdf.ValueTypeNames.String).Set('diagnostic_support')
            joint.GetBody0Rel().SetTargets(['/World/FixedProbeSupport'])
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(frame.ExtractTranslation()))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(frame.ExtractRotationQuat()))
        stage.GetRootLayer().Save();c['scene_sha256']=hashlib.sha256((out/'scene.usda').read_bytes()).hexdigest()
        c['break_scope_control']=dict(name=name,changes=changes)
        c['implementation_sha256'][str(Path(__file__).relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        (out/'config.json').write_text(json.dumps(c,indent=2)+'\n')

if __name__=='__main__':main()
