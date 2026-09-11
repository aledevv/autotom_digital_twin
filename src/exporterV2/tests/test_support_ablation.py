"""Support ablations preserve retained body properties and attachment rest frames."""
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Sdf
from experiments.detachable_fruit_v2.prepare_support_ablation import reduce_support, properties


def source():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.)
    names = [('root','internode'), ('stem','internode'), ('rachis','truss_rachis'),
             ('pedicel','pedicel'), ('fruit','fruit'), ('other_pedicel','pedicel'), ('other_fruit','fruit')]
    for i, (name, role) in enumerate(names):
        body = UsdGeom.Xform.Define(stage, '/'+name)
        body.AddTranslateOp().Set(Gf.Vec3d(i*.1, 0, 0))
        prim = body.GetPrim()
        prim.CreateAttribute('autotom:role', Sdf.ValueTypeNames.String).Set(role)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr().Set(.01+i*.001)
        mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0))
        shape = UsdGeom.Sphere.Define(stage, '/'+name+'/Collider')
        shape.CreateRadiusAttr().Set(.01)
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    for parent, child in [(None,'root'), ('root','stem'), ('stem','rachis'),
                           ('rachis','pedicel'), ('pedicel','fruit'),
                           ('rachis','other_pedicel'), ('other_pedicel','other_fruit')]:
        j = UsdPhysics.Joint.Define(stage, '/'+child+('/RootFixedJoint' if parent is None else '/Joint'))
        j.CreateBody1Rel().SetTargets(['/'+child])
        if parent:
            j.CreateBody0Rel().SetTargets(['/'+parent])
            a = UsdGeom.Xformable(stage.GetPrimAtPath('/'+parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
            b = UsdGeom.Xformable(stage.GetPrimAtPath('/'+child)).ComputeLocalToWorldTransform(0).ExtractTranslation()
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(b-a))
        if 'fruit' in child:
            j.CreateBreakForceAttr().Set(6.)
            j.CreateExcludeFromArticulationAttr().Set(True)
        else:
            for axis in ('rotX','rotY','rotZ'):
                lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis)
                lim.CreateLowAttr().Set(-30.)
                lim.CreateHighAttr().Set(30.)
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath('/fruit')).CreateFilteredPairsRel().SetTargets(['/pedicel','/other_fruit'])
    return stage


@pytest.mark.parametrize('mode,count', [('rigid',3), ('articulated',3), ('rachis',5)])
def test_preserve_bodies_and_fruit_attachment(mode,count):
    stage = source()
    before = properties(stage)
    manifest = reduce_support(stage, '/fruit', mode)
    after = properties(stage)
    assert len(manifest['retained']) == count
    assert not manifest['audit']['errors']
    for path in manifest['retained']:
        for key, value in before.items():
            if key.startswith(path+'.') and not key.endswith('.physics:filteredPairs'):
                assert after[key] == value
    for key, value in before.items():
        if key.startswith('/fruit/Joint.') or '/Collider.' in key and key in after:
            assert after[key] == value
    parent = stage.GetPrimAtPath('/pedicel/Joint').GetRelationship('physics:body0').GetTargets()
    assert str(parent[0]) == ('/rachis' if mode=='rachis' else '/root')
    assert stage.GetPrimAtPath('/fruit').GetRelationship('physics:filteredPairs').GetTargets() == [Sdf.Path('/pedicel')]
