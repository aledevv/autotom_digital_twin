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
