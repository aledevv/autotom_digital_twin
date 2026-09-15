"""Installed-PhysX contact callback and hierarchy filtering sanity check."""
from pathlib import Path
import json
import os
import sys

from isaacsim import SimulationApp
app=SimulationApp({'headless':True})
from isaacsim.core.api import World
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools
from omni.physx import get_physx_simulation_interface

output=Path(sys.argv[1]);output.mkdir(parents=True,exist_ok=False)
world=World(stage_units_in_meters=1.)
stage=world.stage
scene=UsdPhysics.Scene.Get(stage,'/physicsScene')
scene.CreateGravityMagnitudeAttr().Set(0.)
phys=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim());phys.CreateEnableGPUDynamicsAttr().Set(False);phys.CreateSolverTypeAttr().Set('PGS')
for group,x in [('allowed',0.),('filtered',1.),('shape_filtered',2.)]:
    for i,dx in enumerate((-.009,.009)):
        body=UsdGeom.Xform.Define(stage,f'/World/{group}/ball{i}')
        body.AddTranslateOp().Set(Gf.Vec3d(x+dx,0,0))
        p=UsdGeom.Sphere.Define(stage,f'/World/{group}/ball{i}/collider')
        p.CreateRadiusAttr().Set(.01)
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim());UsdPhysics.CollisionAPI.Apply(p.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr().Set(.01)
        PhysxSchema.PhysxContactReportAPI.Apply(body.GetPrim()).CreateThresholdAttr().Set(0.)
UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/World/filtered/ball0')).CreateFilteredPairsRel().SetTargets(['/World/filtered/ball1'])
UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/World/shape_filtered/ball0/collider')).CreateFilteredPairsRel().SetTargets(['/World/shape_filtered/ball1/collider'])
events=[]
def contact(headers,data):
    for h in headers:
        events.append(dict(a=str(PhysicsSchemaTools.intToSdfPath(h.actor0)),b=str(PhysicsSchemaTools.intToSdfPath(h.actor1)),
            type=int(h.type),points=[dict(position=list(p.position),separation=float(p.separation),impulse=list(p.impulse))
                for p in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]]))
sub=get_physx_simulation_interface().subscribe_contact_report_events(contact)
world.reset()
for _ in range(10):world.step(render=False)
allowed=[e for e in events if '/allowed/' in e['a'] and e['points']]
filtered=[e for e in events if '/allowed/' not in e['a']]
result=dict(passed=bool(allowed) and not filtered,events=events)
(output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
print('CONTACT_SELFTEST',result['passed'],flush=True)
sub=None
app.close()
os._exit(0 if result['passed'] else 1)
