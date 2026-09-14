"""Density changes cannot silently retune drives or alter fruit/body geometry."""
import numpy as np
import pytest
from pxr import UsdPhysics
from exporterV2.tests.test_support_ablation import source
from exporterV2.fruit_experiments import bodies_and_joints,value
from experiments.detachable_fruit_v2.prepare_support_ablation import properties
from experiments.detachable_fruit_v2.prepare_support_matrix import apply_controls


def effective(stage):
    return {'bodies':[dict(path=p,role=value(b,'autotom:role'),
        mass_kg=UsdPhysics.MassAPI(b).GetMassAttr().Get(),inertia=np.diag([.001,.001,.001]).reshape(9).tolist(),
        com=[[0.,0.,0.]],com_orientation=[[1.,0.,0.,0.]]) for p,b in bodies_and_joints(stage)[0].items()],
        'articulation':{'dofs':10}}


@pytest.mark.parametrize('ratio', [.1,1.,10.])
@pytest.mark.parametrize('lock_entry', [False,True])
def test_density_and_lock_independent(ratio,lock_entry):
    s=source()
    for axis in ('rotX','rotY'):
        drive=UsdPhysics.DriveAPI.Apply(s.GetPrimAtPath('/rachis/Joint'),axis)
        drive.CreateTypeAttr().Set('force')
        drive.CreateStiffnessAttr().Set(10.)
        drive.CreateDampingAttr().Set(.1)
    before=properties(s); e=effective(s)
    result=apply_controls(s,e,ratio,lock_entry)
    after=properties(s)
    assert not result['audit']['errors']
    for p,v in before.items():
        if '/Collider.' in p or p.startswith('/fruit.') or p.startswith('/fruit/Joint.') or 'drive:' in p:
            assert after[p]==v
    assert result['expected_articulation_dofs']==10-2*int(lock_entry)
    for b in e['bodies']:
        factor=ratio if b['role'] in ('truss_rachis','pedicel') else 1.
        assert result['expected_body_properties'][b['path']]['mass_kg']==pytest.approx(b['mass_kg']*factor)
        assert result['expected_body_properties'][b['path']]['inertia']==pytest.approx(np.asarray(b['inertia'])*factor)


def test_no_fruit_preserves_support_load():
    s=source(); before=properties(s);e=effective(s)
    result=apply_controls(s,e,1.,without_fruit=True)
    assert not result['audit']['attachments']
    for b in e['bodies']:
        if b['role']!='fruit':
            assert UsdPhysics.MassAPI(s.GetPrimAtPath(b['path'])).GetMassAttr().Get()==b['mass_kg']
    assert not s.GetPrimAtPath('/fruit')
    assert s.GetPrimAtPath('/pedicel')


def test_coherent_fruit_preserves_mass_and_attachment():
    import math
    from pxr import Gf, Usd, UsdGeom
    from experiments.detachable_fruit_v2.prepare_support_matrix import correct_fruit_scale
    s = Usd.Stage.CreateInMemory()
    fruit = UsdGeom.Xform.Define(s, '/fruit')
    fruit.AddTranslateOp().Set(Gf.Vec3d(.1, .2, .3))
    fruit.AddOrientOp().Set(Gf.Quatf(Gf.Rotation(Gf.Vec3d(0,1,0),45).GetQuat()))
    sphere = UsdGeom.Sphere.Define(s, '/fruit/Sphere')
    sphere.CreateRadiusAttr(.025)
    mass = UsdPhysics.MassAPI.Apply(fruit.GetPrim())
    mass.CreateMassAttr(.008)
    joint = UsdPhysics.FixedJoint.Define(s, '/fruit/Joint')
    joint.CreateBody1Rel().SetTargets(['/fruit'])
    joint.CreateLocalPos1Attr(Gf.Vec3f(0,0,.023))
    joint.CreateBreakForceAttr(6.)
    old_anchor = UsdGeom.XformCache().GetLocalToWorldTransform(fruit.GetPrim()).Transform(Gf.Vec3d(0,0,.023))
    body = dict(path='/fruit', mass_kg=.008)
    result = correct_fruit_scale(s,body)
    r = sphere.GetRadiusAttr().Get()
    assert .008 / (4/3*math.pi*r**3) == pytest.approx(1000.)
    assert mass.GetMassAttr().Get() == pytest.approx(.008)
    assert np.diag(np.asarray(body['inertia']).reshape(3,3)) == pytest.approx([.4*.008*r*r]*3)
    new_anchor = UsdGeom.XformCache().GetLocalToWorldTransform(fruit.GetPrim()).Transform(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
    assert (old_anchor-new_anchor).GetLength() < 1e-7
    assert result['preserved_overlap_m'] == pytest.approx(.002)
    assert joint.GetBreakForceAttr().Get() == 6.


@pytest.mark.parametrize("damping,stiffness", [(4.,1.), (1.,4.), (2.,2.)])
def test_rachis_controls_leave_other_properties_unchanged(damping,stiffness):
    s = source()
    for name in ('rachis', 'pedicel'):
        prim = s.GetPrimAtPath('/'+name+'/Joint')
        for axis in ('rotX','rotY'):
            d = UsdPhysics.DriveAPI.Apply(prim, axis)
            d.CreateTypeAttr('force')
            d.CreateStiffnessAttr(10.)
            d.CreateDampingAttr(.1)
    before = properties(s)
    result = apply_controls(s,effective(s),1.,rachis_damping_scale=damping,rachis_stiffness_scale=stiffness)
    after = properties(s)
    changed = [p for p in before if before[p] != after[p]]
    fields = ([] if damping == 1 else ['damping']) + ([] if stiffness == 1 else ['stiffness'])
    assert set(changed) == {f'/rachis/Joint.drive:{axis}:physics:{field}' for axis in ('rotX','rotY') for field in fields}
    assert float(after['/rachis/Joint.drive:rotX:physics:damping']) == pytest.approx(.1*damping)
    assert float(after['/rachis/Joint.drive:rotX:physics:stiffness']) == pytest.approx(10.*stiffness)
    assert result['expected_articulation_dofs'] == 10


def test_break_force_changes_only_fruit_attachments():
    s = source()
    before = properties(s)
    result = apply_controls(s,effective(s),1.,fruit_break_force=3.)
    after = properties(s)
    assert {k for k in before.keys() | after.keys() if before.get(k) != after.get(k)} == {
        '/fruit/Joint.physics:breakForce', '/other_fruit/Joint.physics:breakForce'}
    assert UsdPhysics.Joint(s.GetPrimAtPath('/fruit/Joint')).GetBreakForceAttr().Get() == 3.
    assert not result['audit']['errors']


def test_settling_gate_authors_unbreakable_fruit_only():
    import math
    s=source()
    before=properties(s)
    apply_controls(s,effective(s),1.,fruit_break_force=6.,arm_after_settle=True)
    after=properties(s)
    assert {k for k in before.keys() | after.keys() if before.get(k) != after.get(k)} <= {
        '/fruit/Joint.physics:breakForce','/other_fruit/Joint.physics:breakForce'}
    assert math.isinf(UsdPhysics.Joint(s.GetPrimAtPath('/fruit/Joint')).GetBreakForceAttr().Get())
