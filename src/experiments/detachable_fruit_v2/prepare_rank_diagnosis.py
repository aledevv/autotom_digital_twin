"""Fresh rank-10 diagnostic controls; preserve loaded mass/inertia when removing contacts."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from pxr import Gf, Usd, UsdPhysics
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.fruit_experiments import audit

def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('--source',type=Path,required=True)
 p.add_argument('--output',type=Path,required=True)
 p.add_argument('--no-contacts',action='store_true')
 p.add_argument('--no-gravity',action='store_true')
 p.add_argument('--fruit-properties-from',type=Path)
 p.add_argument('--reference-rank',type=int,default=6)
 p.add_argument('--target-rank',type=int,default=10)
 p.add_argument('--transfer',choices=('both','mass','inertia'),default='both')
 p.add_argument('--hz',type=int)
 a=p.parse_args()
 if a.hz is not None and a.hz <= 0:
  p.error('--hz must be positive')
 out=a.output.resolve();out.mkdir(parents=True,exist_ok=False)
 stage=Usd.Stage.Open(Usd.Stage.Open(str(a.source/'scene.usda')).Flatten())
 config=json.loads((a.source/'config.json').read_text())
 effective=json.loads((a.source/'headless-effective.json').read_text())
 reference={}
 if a.fruit_properties_from:
  reference={b['path']:b for b in json.loads((a.fruit_properties_from/'headless-effective.json').read_text())['bodies']}
 changes=[]
 def set_attr(attr,value):
  before=attr.Get();attr.Set(value);changes.append(dict(path=str(attr.GetPath()),before=str(before),after=str(value)))
 expected={}
 for b in effective['bodies']:
  b=dict(b)
  api=UsdPhysics.MassAPI(stage.GetPrimAtPath(b['path']))
  if reference and b['role']=='fruit':
   other=reference[b['path'].replace(f'Truss_r{a.target_rank}_',f'Truss_r{a.reference_rank}_')]
   keys=('mass_kg','inertia') if a.transfer=='both' else (('mass_kg',) if a.transfer=='mass' else ('inertia',))
   b.update({k:other[k] for k in keys})
   set_attr(api.CreateMassAttr(),b['mass_kg'])
  mat=np.array(b['inertia']).reshape(3,3)
  assert np.allclose(mat,np.diag(np.diag(mat)),atol=1e-16)
  set_attr(api.CreateDiagonalInertiaAttr(),Gf.Vec3f(*np.diag(mat)))
  q=np.array(b['com_orientation']).reshape(4)
  set_attr(api.CreatePrincipalAxesAttr(),Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:])))
  expected[b['path']]={k:b[k] for k in ('mass_kg','inertia','com','com_orientation')}
 for prim in stage.Traverse():
  if a.hz and prim.IsA(UsdPhysics.Scene):
   set_attr(prim.GetAttribute('physxScene:timeStepsPerSecond'),a.hz)
  if a.no_contacts and prim.HasAPI(UsdPhysics.CollisionAPI):
   set_attr(UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(),False)
  if a.no_gravity and prim.IsA(UsdPhysics.Scene):
   set_attr(UsdPhysics.Scene(prim).CreateGravityMagnitudeAttr(),0.)
 report=audit(stage);assert not report['errors'],report['errors']
 stage.GetRootLayer().Export(str(out/'scene.usda'))
 config.update(run_dir=str(out),gui=False,source_usd=str((a.source/'scene.usda').resolve()),
  source_sha256=hashlib.sha256((a.source/'scene.usda').read_bytes()).hexdigest(),
  scene_sha256=hashlib.sha256((out/'scene.usda').read_bytes()).hexdigest(),
  expected_body_properties=expected,collisions=not a.no_contacts,
  diagnostic_controls=dict(no_contacts=a.no_contacts,no_gravity=a.no_gravity,
   fruit_properties_from=str(a.fruit_properties_from),reference_rank=a.reference_rank,target_rank=a.target_rank,transfer=a.transfer),diagnostic_changes=changes)
 if a.no_gravity:
  config['diagnostic_gravity_magnitude']=0.
 if a.hz:
  config['hz']=a.hz
 config['implementation_sha256'][str(Path(__file__).resolve().relative_to(ROOT))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
 config['source_effective_sha256']=hashlib.sha256((a.source/'headless-effective.json').read_bytes()).hexdigest()
 if a.fruit_properties_from:
  config['reference_effective_sha256']=hashlib.sha256((a.fruit_properties_from/'headless-effective.json').read_bytes()).hexdigest()
 for name,data in [('config',config),('audit',report),('changes',changes)]:
  (out/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
 print(out)
if __name__=='__main__':main()
