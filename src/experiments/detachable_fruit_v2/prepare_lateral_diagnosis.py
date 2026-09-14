"""Bounded lateral-support ablations, using immutable source and loaded properties."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
from exporterV2.fruit_experiments import audit, joint_frame


def prepare(source, output, variant, duration=20.):
    output.mkdir(parents=True, exist_ok=False)
    stage = Usd.Stage.Open(Sdf.Layer.OpenAsAnonymous(str(source/'scene.usda')))
    effective = json.loads((source/'headless-effective.json').read_text())
    config = json.loads((source/'config.json').read_text())
    original = Usd.Stage.Open(str(source/'scene.usda'))
    branch_bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and '/Vegetative/Branch_' in str(p.GetPath())]
    joints = [UsdPhysics.Joint(p) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    selected = [j for j in joints if j.GetBody1Rel().GetTargets() and stage.GetPrimAtPath(j.GetBody1Rel().GetTargets()[0]) in branch_bodies]
    assert len(branch_bodies)==4 and len(selected)==4
    changes = []
    expected = {b['path']:{k:b[k] for k in ['mass_kg','inertia','com','com_orientation']} for b in effective['bodies']}
    if variant.startswith('fixed'):
        for j in selected:
            p=j.GetPrim();changes.append(dict(path=str(p.GetPath()),old_type=p.GetTypeName(),new_type='PhysicsFixedJoint'))
            p.SetTypeName('PhysicsFixedJoint')
            for schema in list(p.GetAppliedSchemas()):
                if schema.startswith(('PhysicsDriveAPI:', 'PhysicsLimitAPI:')):p.RemoveAppliedSchema(schema)
            for name in list(p.GetPropertyNames()):
                if name.startswith(('drive:', 'limit:')):p.RemoveProperty(name)
    if variant=='com-frame':
        cache=UsdGeom.XformCache()
        # Save every body, collider and joint world frame before changing references.
        worlds={str(p.GetPath()):cache.GetLocalToWorldTransform(p) for p in stage.Traverse() if p.IsA(UsdGeom.Xformable)}
        anchors={str(j.GetPath()):[joint_frame(stage,j,i) for i in (0,1)] for j in joints}
        for p in branch_bodies:
            path=str(p.GetPath()); com=Gf.Vec3d(*np.asarray(expected[path]['com']).reshape(3))
            old=worlds[path];new=Gf.Matrix4d(old);new.SetTranslateOnly(old.Transform(com))
            parent=cache.GetLocalToWorldTransform(p.GetParent())
            x=UsdGeom.Xformable(p);x.ClearXformOpOrder();x.AddTransformOp().Set(new*parent.GetInverse())
            for child in p.GetChildren():
                if child.IsA(UsdGeom.Xformable):
                    x=UsdGeom.Xformable(child);x.ClearXformOpOrder();x.AddTransformOp().Set(worlds[str(child.GetPath())]*new.GetInverse())
            api=UsdPhysics.MassAPI(p);api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0))
            inertia=np.asarray(expected[path]['inertia']).reshape(3,3)
            assert np.allclose(inertia,np.diag(np.diag(inertia)),atol=1e-12)
            api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*np.diag(inertia)))
            api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1))
            expected[path]['com']=[0.,0.,0.]
            changes.append(dict(path=path,change='body origin moved to physical COM; geometry preserved',old_com=list(com)))
        cache.Clear()
        for j in joints:
            for side in (0,1):
                targets=getattr(j,f'GetBody{side}Rel')().GetTargets()
                if not targets or stage.GetPrimAtPath(targets[0]) not in branch_bodies:
                    continue
                world=cache.GetLocalToWorldTransform(stage.GetPrimAtPath(targets[0])) if targets else Gf.Matrix4d(1)
                local=anchors[str(j.GetPath())][side]*world.GetInverse()
                getattr(j,f'CreateLocalPos{side}Attr')().Set(Gf.Vec3f(local.ExtractTranslation()))
                getattr(j,f'CreateLocalRot{side}Attr')().Set(Gf.Quatf(local.ExtractRotationQuat()))
        for path,old in worlds.items():
            p=stage.GetPrimAtPath(path)
            if p.HasAPI(UsdPhysics.CollisionAPI) or p.IsA(UsdGeom.Gprim):
                assert np.allclose(old,cache.GetLocalToWorldTransform(p),atol=1e-7),path
        for j in joints:
            for i in (0,1):
                delta=np.max(np.abs(np.asarray(anchors[str(j.GetPath())][i])-np.asarray(joint_frame(stage,j,i))))
                assert delta < 2e-6,(str(j.GetPath()),i,delta)
    if variant=='tgs-zero':
        config.update(art_velocity=0,fruit_velocity=0)
    if variant=='pgs':config['solver']='PGS'
    scene=stage.GetPrimAtPath('/World/PhysicsScene')
    scene.GetAttribute('physxScene:solverType').Set(config['solver'])
    stage.GetPrimAtPath('/World/Stem').GetAttribute('physxArticulation:solverVelocityIterationCount').Set(config['art_velocity'])
    for b in effective['bodies']:
        if b['role']=='fruit':stage.GetPrimAtPath(b['path']).GetAttribute('physxRigidBody:solverVelocityIterationCount').Set(config['fruit_velocity'])
    config.update(collisions=not variant.endswith('no-contacts'),duration=duration,gui=False,
                  run_dir=str(output),record_articulation_dofs=True,expected_body_properties=expected)
    result=audit(stage)
    assert not result['errors'],result['errors']
    stage.GetRootLayer().Export(str(output/'scene.usda'))
    config['scene_sha256']=hashlib.sha256((output/'scene.usda').read_bytes()).hexdigest()
    (output/'config.json').write_text(json.dumps(config,indent=2)+'\n')
    # All authored property changes, rather than only intended overrides.
    differences=[]
    for p in original.Traverse():
        new=stage.GetPrimAtPath(p.GetPath())
        if p.GetTypeName()!=new.GetTypeName():differences.append([str(p.GetPath()),'type',p.GetTypeName(),new.GetTypeName()])
        for name in set(p.GetPropertyNames())|set(new.GetPropertyNames()):
            a,b=p.GetAttribute(name),new.GetAttribute(name)
            va,vb=str(a.Get()) if a else None,str(b.Get()) if b else None
            if va!=vb:differences.append([str(p.GetPath()),name,va,vb])
    (output/'variant.json').write_text(json.dumps(dict(variant=variant,source=str(source),changes=changes,property_differences=differences,audit=result,config=config),indent=2)+'\n')
    print(output.name,'prepared',len(differences),'property differences')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('output',type=Path)
    p.add_argument('--variant',required=True,choices=['baseline','fixed','baseline-no-contacts','fixed-no-contacts','com-frame','tgs-zero','pgs'])
    p.add_argument('--duration',type=float,default=20.)
    a=p.parse_args();prepare(a.source.resolve(),a.output.resolve(),a.variant,a.duration)
