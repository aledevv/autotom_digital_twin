"""Compatibility checks for the opt-in historical truss template."""
import copy
from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from plant_state import load_plant_state
from exporterV2.plant_state_branches import build_truss_branches, apply_checkpoint_physics_policy
from exporterV2.standard_truss import template, standardize, select_fixture, author, column_frame
from exporterV2.fruit_experiments import audit
from exporterV2.cli import build_argument_parser, generate_from_args

ROOT=Path(__file__).resolve().parents[3]

@pytest.fixture(scope='module')
def adapter():
    state=load_plant_state(ROOT/'data/plant_states/plant_state_day_160.json')
    return build_truss_branches(state,include_fruits=True,physical_fruits=True)


def test_mapping_scale_and_policy(adapter):
    before=copy.deepcopy(adapter)
    result=select_fixture(standardize(adapter),'direct')
    assert adapter==before
    assert len(result.terminal_bodies)==40
    assert len(result.standard_truss_replacements)==5
    assert not result.rigid_leaf_visuals
    assert len({b['id'] for b in result.branches})==len(result.branches)
    for group in result.standard_truss_replacements:
        fruits=[b for b in result.terminal_bodies if b['id'].startswith(group['branch_id']+'_')]
        assert len(fruits)==8
        assert sum(b['mass'] for b in fruits)==pytest.approx(.07470853207632899)
        for b in fruits:
            assert (4/3)*np.pi*(b['radius']*2)**3*1000==pytest.approx(b['mass'])
    after=apply_checkpoint_physics_policy(result)
    assert [b for b in after.branches if b.get('standard_truss')]==[b for b in result.branches if b.get('standard_truss')]


def test_template_equivalence_at_reference_pose(adapter):
    data, source=template()
    old=next(b for b in adapter.branches if b.get('truss_component')=='rachis')
    root=copy.deepcopy(old)
    root_matrix=UsdGeom.XformCache().GetLocalToWorldTransform(source.GetPrimAtPath(data['root']))
    root['link_specs'][0]['rest_frame']=column_frame(root_matrix)
    fixture=replace(adapter,branches=tuple(root if b['id']==root['id'] else b for b in adapter.branches))
    standardized=standardize(fixture)
    group=[b for b in standardized.branches if b.get('standard_root')==root['id'].removesuffix('_rachis')]
    first=next(b for b in group if b['truss_component']=='rachis')
    target=Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(target,1.)
    UsdPhysics.SetStageKilogramsPerUnit(target,1.)
    UsdGeom.Xform.Define(target,'/World/Stem')
    joint=UsdPhysics.Joint(source.GetPrimAtPath(data['root']+'/AttachJoint'))
    local=Gf.Matrix4d(1);local.SetRotate(Gf.Quatd(joint.GetLocalRot0Attr().Get()));local.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
    parent_matrix=local.GetInverse()*root_matrix
    parent='/World/Stem/Parent'
    x=UsdGeom.Xform.Define(target,parent);x.AddTransformOp().Set(parent_matrix)
    UsdPhysics.RigidBodyAPI.Apply(x.GetPrim());mass=UsdPhysics.MassAPI.Apply(x.GetPrim());mass.CreateMassAttr().Set(1.);mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0))
    anchor=UsdPhysics.FixedJoint.Define(target,parent+'/RootFixedJoint')
    anchor.CreateBody1Rel().SetTargets([parent])
    anchor.CreateLocalPos0Attr().Set(Gf.Vec3f(parent_matrix.ExtractTranslation()))
    anchor.CreateLocalRot0Attr().Set(Gf.Quatf(parent_matrix.ExtractRotationQuat()))
    registry={first['parent']:([parent]*first['attach_link'],[],None,None)}
    records=author(target,first,group,standardized.terminal_bodies,registry)
    assert len(records)==8
    assert not audit(target)['errors']
    base=first['standard_root']
    for body in data['bodies']:
        oldp=source.GetPrimAtPath(body['path']);newp=target.GetPrimAtPath(body['path'].replace('Truss_r6_o0',base))
        assert newp
        a=UsdGeom.XformCache().GetLocalToWorldTransform(oldp);b=UsdGeom.XformCache().GetLocalToWorldTransform(newp)
        assert np.allclose(a,b,atol=2e-7)
        for oldchild in Usd.PrimRange(oldp):
            newchild=target.GetPrimAtPath(str(oldchild.GetPath()).replace('Truss_r6_o0',base))
            assert oldchild.GetTypeName()==newchild.GetTypeName()
            for attr in oldchild.GetAttributes():
                name=attr.GetName()
                if name.startswith(('drive:','limit:')) or name in ('radius','height','physics:mass','physics:breakForce','physics:excludeFromArticulation'):
                    assert attr.Get()==newchild.GetAttribute(name).Get(),attr.GetPath()


@pytest.mark.parametrize('extra',[['--physics-hz','480'],['--truss-stiffness-scale','0.5'],['--terminal-solver-preset','stabilized']])
def test_conflicting_overrides_rejected(extra):
    args=build_argument_parser().parse_args(['--day','160','--debug-profile','full','--allow-experimental-fruit-physics','--experimental-truss-preset','main-rank6-standard',*extra])
    with pytest.raises(ValueError):generate_from_args(args)


def test_exported_preset_and_runtime_agree(tmp_path):
    output=tmp_path/'standard.usda'
    args=build_argument_parser().parse_args(['--day','160','--debug-profile','full',
        '--allow-experimental-fruit-physics','--experimental-truss-preset','main-rank6-standard',
        '--experimental-truss-fixture','direct','--leaf-shape-backend','legacy','--output',str(output)])
    _,plan,usd,_=generate_from_args(args)
    import json
    config=json.loads(Path(str(usd)+'.standard.json').read_text())
    assert config['hz']==60
    assert len(config['expected_body_properties'])==100
    assert len(plan.adapter.terminal_bodies)==40
    stage=Usd.Stage.Open(str(usd))
    assert not audit(stage)['errors']
    assert stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:solverType').Get()=='TGS'
    assert stage.GetPrimAtPath('/World/PhysicsScene').GetAttribute('physxScene:timeStepsPerSecond').Get()==60


def test_canonical_roll_does_not_invert_template():
    from exporterV2.standard_truss import upright_attachment
    data, source=template()
    original=UsdGeom.XformCache().GetLocalToWorldTransform(source.GetPrimAtPath(data['root']))
    # Same origin and rachis direction, but opposite transverse axes.
    flipped=np.array(original).copy()
    flipped[:2,:3]*=-1
    result=upright_attachment(original,Gf.Matrix4d(*flipped.reshape(-1)))
    assert np.allclose(result,original,atol=1e-7)
    assert np.allclose(result.ExtractTranslation(),original.ExtractTranslation())


def test_upright_attachment_keeps_canonical_direction_and_downward_fruit(adapter):
    data, source=template()
    original=UsdGeom.XformCache().GetLocalToWorldTransform(source.GetPrimAtPath(data['root']))
    from exporterV2.standard_truss import upright_attachment, world_frame
    root=next(b for b in adapter.branches if b.get('truss_component')=='rachis')
    canonical=world_frame(root['link_specs'][0]['rest_frame'])
    target=upright_attachment(original,canonical)
    assert np.allclose(target.ExtractTranslation(),canonical.ExtractTranslation())
    assert np.allclose(target.TransformDir(Gf.Vec3d(0,0,1)),canonical.TransformDir(Gf.Vec3d(0,0,1)),atol=1e-7)
    delta=original.GetInverse()*target
    assert delta.TransformDir(Gf.Vec3d(0,0,-1))[2]<-.99
