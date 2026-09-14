"""Opt-in, immutable main rank-6 truss template in canonical PlantState scenes."""
from __future__ import annotations
import copy
from dataclasses import replace
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

NAME = 'main-rank6-standard'
ASSETS = Path(__file__).with_name('presets')
PREFIX = 'Truss_r6_o0'
RUNTIME = dict(hz=60, solver='TGS', gpu=True, art_position=32, art_velocity=4,
               fruit_position=32, fruit_velocity=1, break_force=6.)


def template():
    from .core.tree_config import GLOBAL_SCALE
    if GLOBAL_SCALE != 2.:
        raise ValueError('Standard truss requires the validated GLOBAL_SCALE=2 conversion')
    data = json.loads((ASSETS/'main_rank6.json').read_text())
    path = ASSETS/'main_rank6.usda'
    if hashlib.sha256(path.read_bytes()).hexdigest() != data['template_sha256']:
        raise ValueError('Main rank-6 template checksum mismatch')
    return data, Usd.Stage.Open(str(path))


def column_frame(matrix):
    a = np.array(matrix).T
    a[:3, 3] /= 2.
    return a.tolist()


def world_frame(frame):
    a = np.array(frame, dtype=float)
    a[:3, 3] *= 2.
    return Gf.Matrix4d(*a.T.reshape(-1))


def upright_attachment(reference, canonical):
    """Keep the canonical origin/axis, but preserve main's roll relative to gravity.

    A cylindrical PlantState axis does not define which side of the historical
    truss should face down. Copying its full frame can invert the fruit layout.
    """
    source = np.array(reference).T
    target = np.array(canonical).T

    def basis(axis):
        axis = axis / np.linalg.norm(axis)
        up = np.array([0., 0., 1.])
        side = np.cross(up, axis)
        if np.linalg.norm(side) < 1e-6:
            raise ValueError('Vertical standard truss needs an explicit roll convention')
        side /= np.linalg.norm(side)
        return np.column_stack((side, np.cross(axis, side), axis))

    rotation = basis(target[:3, 2]) @ basis(source[:3, 2]).T
    target[:3, :3] = rotation @ source[:3, :3]
    return Gf.Matrix4d(*target.T.reshape(-1))


def standardize(adapter):
    """Replace only nondegenerate trusses, retaining immutable source provenance."""
    data, source = template()
    cache = UsdGeom.XformCache()
    reference_frame = cache.GetLocalToWorldTransform(source.GetPrimAtPath(data['root']))
    inverse = reference_frame.GetInverse()
    branches = [b for b in adapter.branches if b.get('truss_component') not in ('rachis', 'pedicel')]
    terminals = []
    replacements = []
    attachments = [a for a in adapter.attachment_map if a.get('kind') not in ('truss_rachis', 'pedicel')]
    for root in (b for b in adapter.branches if b.get('truss_component') == 'rachis'):
        base = root['id'].removesuffix('_rachis')
        canonical_frame = world_frame(root['link_specs'][0]['rest_frame'])
        target_frame = upright_attachment(reference_frame, canonical_frame)
        transform = inverse * target_frame
        original_pedicels = [b for b in adapter.branches if b.get('parent') == root['id']]
        original_fruits = [b for b in adapter.terminal_bodies if b['parent_branch_id'] in {x['id'] for x in original_pedicels}]
        mapping = {}
        for b in data['branches']:
            new = copy.deepcopy(b)
            bid = b['id'].replace(PREFIX, base)
            new.update(id=bid, parent=root['parent'] if b['id'].endswith('_rachis') else root['id'],
                       attach_link=root['attach_link'] if b['id'].endswith('_rachis') else b['attach_link'],
                       kind='truss_rachis' if b['id'].endswith('_rachis') else 'pedicel',
                       truss_component='rachis' if b['id'].endswith('_rachis') else 'pedicel',
                       standard_truss=NAME, standard_root=base, standard_transform=list(np.array(transform).reshape(-1)),
                       density=20000., young_modulus=3.e9, damping_ratio=7., joint_type='d6', attachment_joint_type='d6', visual_axis_id=bid,
                       source_parent_node_id=(root['source_parent_node_id'] if b['id'].endswith('_rachis') else root['link_specs'][0]['canonical_node_id']))
            specs = []
            for i in range(b['n_links']):
                old_path = f"/World/Stem/{b['id']}_Link_{i+1:02d}"
                matrix = cache.GetLocalToWorldTransform(source.GetPrimAtPath(old_path)) * transform
                spec = copy.deepcopy(root['link_specs'][0])
                axis_id = f"{bid}:standard:{i+1}"
                spec.update(id=axis_id, canonical_axis_id=axis_id, length=b['height'], radius=b['radius'],
                            rest_frame=column_frame(matrix), source_rest_frame=root['link_specs'][0]['rest_frame'])
                specs.append(spec)
                mapping[old_path] = f'/World/Stem/{bid}_Link_{i+1:02d}'
            new['link_specs'] = specs
            branches.append(new)
            attachments.append(dict(kind=new['kind'], branch_id=bid, parent_branch_id=new['parent'],
                                    attach_link=new['attach_link'], standardized=True))
        for i, body in enumerate(data['terminal_bodies']):
            old_path = '/World/TerminalBodies/'+body['id']
            eff = next(x for x in data['bodies'] if x['path'] == old_path)
            matrix = cache.GetLocalToWorldTransform(source.GetPrimAtPath(old_path)) * transform
            sphere = UsdGeom.Sphere(source.GetPrimAtPath(old_path+'/Sphere'))
            new = dict(body, id=body['id'].replace(PREFIX,base),
                       parent_branch_id=body['parent_branch_id'].replace(PREFIX,base),
                       radius=sphere.GetRadiusAttr().Get()/2., mass=eff['mass_kg'], physical=True,
                       detachment_enabled=True, exclude_from_articulation=True, break_force=6.,
                       standard_truss=NAME, rest_center=column_frame(matrix)[0:3],
                       canonical_node_id=root['link_specs'][0]['canonical_node_id'],
                       canonical_organ_id=root['link_specs'][0]['canonical_organ_id'],
                       canonical_primitive_id=f'{base}:standard:fruit:{i+1}')
            new['rest_center'] = [float(matrix.ExtractTranslation()[j])/2 for j in range(3)]
            new['rest_frame'] = column_frame(matrix)
            new['source_center'] = list(new['rest_center'])
            terminals.append(new)
            mapping[old_path] = '/World/TerminalBodies/'+new['id']
        replacements.append(dict(branch_id=root['id'], original_fruit_count=len(original_fruits),
                                 original_fruits=original_fruits, original_supports=[root,*original_pedicels],
                                 template=NAME, generated_fruit_count=8, body_mapping=mapping,
                                 orientation_policy='reference_roll_relative_to_gravity',
                                 canonical_attachment_frame=column_frame(canonical_frame),
                                 authored_attachment_frame=column_frame(target_frame)))
    return replace(adapter, branches=tuple(branches), terminal_bodies=tuple(terminals),
                   attachment_map=tuple(attachments), standard_truss_replacements=tuple(replacements))


def author(stage, root_branch, branches, terminals, registry):
    """Copy the historical geometry/joints verbatim, transformed at the attachment."""
    data, source = template()
    base=root_branch['standard_root']
    group=[b for b in branches if b.get('standard_root')==base]
    fruits=[b for b in terminals if b.get('standard_truss') and b['id'].startswith(base+'_')]
    mapping={x['path']: x['path'].replace(PREFIX,base) for x in data['bodies']}
    transform=Gf.Matrix4d(*root_branch['standard_transform'])
    cache=UsdGeom.XformCache()
    parent_path=registry[root_branch['parent']][0][root_branch['attach_link']-1]
    for scope in ('/World/Looks','/World/Materials'):
        if source.GetPrimAtPath(scope):
            target=scope+'_StandardTruss'
            if not stage.GetPrimAtPath(target):
                Sdf.CopySpec(source.GetRootLayer(),scope,stage.GetRootLayer(),target)
    for old,new in mapping.items():
        UsdGeom.Xform.Define(stage,str(Sdf.Path(new).GetParentPath()))
        Sdf.CopySpec(source.GetRootLayer(),old,stage.GetRootLayer(),new)
        prim=stage.GetPrimAtPath(new)
        matrix=cache.GetLocalToWorldTransform(source.GetPrimAtPath(old))*transform
        prim.GetAttribute('xformOp:translate').Set(matrix.ExtractTranslation())
        prim.GetAttribute('xformOp:orient').Set(Gf.Quatf(matrix.ExtractRotationQuat()))
        for child in Usd.PrimRange(prim):
            for rel in child.GetRelationships():
                targets=[]
                for t in rel.GetTargets():
                    text=str(t)
                    match=next((p for p in mapping if text==p or text.startswith(p+'/')),None)
                    if any(text==v or text.startswith(v+'/') for v in mapping.values()): targets.append(t)
                    elif match: targets.append(Sdf.Path(mapping[match]+text[len(match):]))
                    elif text.startswith(('/World/Looks/','/World/Materials/')):
                        targets.append(Sdf.Path(text.replace('/World/Looks/','/World/Looks_StandardTruss/').replace('/World/Materials/','/World/Materials_StandardTruss/')))
                    elif rel.GetName()=='physics:body0' and old==data['root']:
                        targets.append(Sdf.Path(parent_path))
                rel.SetTargets(targets)
        mass=next(b for b in data['bodies'] if b['path']==old)
        api=UsdPhysics.MassAPI(prim)
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*np.diag(np.array(mass['inertia']).reshape(3,3))))
        q=np.array(mass['com_orientation']).reshape(4)
        api.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:])))
    joint=UsdPhysics.Joint(stage.GetPrimAtPath(mapping[data['root']]+'/AttachJoint'))
    child_matrix=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(mapping[data['root']]))
    parent_matrix=UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(parent_path))
    local=child_matrix*parent_matrix.GetInverse()
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(local.ExtractTranslation()))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(local.ExtractRotationQuat()))
    for b in group:
        paths=[f"/World/Stem/{b['id']}_Link_{i+1:02d}" for i in range(b['n_links'])]
        mats=[UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(p)) for p in paths]
        registry[b['id']]=(paths,[m.ExtractTranslation() for m in mats],mats[0].TransformDir(Gf.Vec3d(0,0,1)),Gf.Quatf(mats[0].ExtractRotationQuat()))
        for path,spec in zip(paths,b['link_specs']):
            prim=stage.GetPrimAtPath(path)
            for key,val in {'entityKind':'physical_link','canonicalPrimitiveId':spec['canonical_axis_id'],
                            'branchId':b['id'],'branchKind':b['kind'],'role':b['kind'],
                            'canonicalNodeId':spec['canonical_node_id'],'canonicalOrganId':spec['canonical_organ_id']}.items():
                prim.CreateAttribute('autotom:'+key,Sdf.ValueTypeNames.String).Set(val)
            prim.CreateAttribute('autotom:visualRadius',Sdf.ValueTypeNames.Double).Set(spec['radius']*2)
            prim.CreateAttribute('autotom:standardTruss',Sdf.ValueTypeNames.String).Set(NAME)
    records=[]
    for b in fruits:
        path='/World/TerminalBodies/'+b['id'];prim=stage.GetPrimAtPath(path)
        for key,field in [('canonicalPrimitiveId','canonical_primitive_id'),('canonicalNodeId','canonical_node_id'),('canonicalOrganId','canonical_organ_id')]:
            prim.CreateAttribute('autotom:'+key,Sdf.ValueTypeNames.String).Set(b[field])
        prim.CreateAttribute('autotom:entityKind',Sdf.ValueTypeNames.String).Set('terminal_body')
        prim.CreateAttribute('autotom:standardTruss',Sdf.ValueTypeNames.String).Set(NAME)
        records.append(dict(b,path=path,pos=UsdGeom.XformCache().GetLocalToWorldTransform(prim).ExtractTranslation(),radius=b['radius']*2))
    return records


def select_fixture(adapter, fixture):
    if fixture == 'full':
        return adapter
    by_id={b['id']:b for b in adapter.branches}
    roots=[b for b in adapter.branches if b.get('standard_truss') and b.get('truss_component')=='rachis']
    if fixture=='direct':
        roots=[b for b in roots if by_id[b['parent']]['kind']=='stem' and any(f'Truss_r{r}_' in b['id'] for r in range(6,11))]
        if len(roots)!=5:
            raise ValueError(f'Expected five direct standard trusses ranks 6-10; found {len(roots)}')
    elif fixture=='lateral':
        roots=sorted((b for b in roots if by_id[b['parent']]['kind']=='lateral_branch'),key=lambda b:(int(re.search(r'Truss_r(\d+)_', b['id']).group(1)),b['id']))[:1]
        if not roots: raise ValueError('No lateral truss available')
    else: raise ValueError(f'Unknown fixture {fixture}')
    selected={b['standard_root'] for b in roots}
    keep={b['id'] for b in adapter.branches if b['kind']=='stem' or b.get('standard_root') in selected}
    for root in roots:
        parent=root['parent']
        while parent is not None:
            keep.add(parent);parent=by_id[parent]['parent']
    # Leaves are removed before their visual masses are aggregated onto supports.
    return replace(adapter,branches=tuple(b for b in adapter.branches if b['id'] in keep),
                   terminal_bodies=tuple(b for b in adapter.terminal_bodies if b['parent_branch_id'] in keep),
                   rigid_leaf_visuals=(),approved_collision_filters=(),degenerate_organs=(),
                   attachment_map=tuple(a for a in adapter.attachment_map if a.get('branch_id') in keep),
                   standard_truss_replacements=tuple(r for r in adapter.standard_truss_replacements if r['branch_id'] in keep))


def configure_stage(stage):
    from .fruit_experiments import value
    scene=stage.GetPrimAtPath('/World/PhysicsScene')
    for key,val,kind in [('solverType','TGS',Sdf.ValueTypeNames.Token),('enableGPUDynamics',True,Sdf.ValueTypeNames.Bool),('timeStepsPerSecond',60,Sdf.ValueTypeNames.Int)]:
        scene.CreateAttribute('physxScene:'+key,kind).Set(val)
    root=stage.GetPrimAtPath('/World/Stem')
    for key,val in [('solverPositionIterationCount',32),('solverVelocityIterationCount',4)]:
        root.CreateAttribute('physxArticulation:'+key,Sdf.ValueTypeNames.Int).Set(val)
    root.CreateAttribute('autotom:experimentalTrussPreset',Sdf.ValueTypeNames.String).Set(NAME)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            role=value(prim,'autotom:role',None) or value(prim,'autotom:branchKind','internode')
            if value(prim,'autotom:entityKind')=='terminal_body':
                role='fruit'
                for key,val in [('solverPositionIterationCount',32),('solverVelocityIterationCount',1)]:
                    prim.CreateAttribute('physxRigidBody:'+key,Sdf.ValueTypeNames.Int).Set(val)
            prim.CreateAttribute('autotom:role',Sdf.ValueTypeNames.String).Set(role)


def write_runtime_config(stage, destination):
    from .fruit_experiments import audit
    destination=Path(destination).resolve()
    check=audit(stage)
    if check['errors']: raise ValueError(check['errors'])
    config=dict(RUNTIME,scenario='standard-truss',attachment='external',breakable=True,collisions=True,
                stiffness_scale=1.,damping_ratio=4.,damping_scale=1.,preserve_source_drives=True,
                acceptance='functional',duration=60.,force_target=None,force_start=30.,interaction='native',
                mouse_grab_mode='joint',mouse_force_coefficient=10.,record_native_mouse=True,
                camera_eye=[1.1,1.1,1.0],camera_target=[0.,0.,.3],gui=True,
                run_dir=str(destination.parent/(destination.stem+'-runtime')),
                scene_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
                experimental_truss_preset=NAME,implementation_sha256={})
    config['expected_body_properties']={}
    for prim in stage.Traverse():
        if prim.GetAttribute('autotom:standardTruss').Get()==NAME:
            api=UsdPhysics.MassAPI(prim)
            q=api.GetPrincipalAxesAttr().Get()
            config['expected_body_properties'][str(prim.GetPath())]=dict(mass_kg=api.GetMassAttr().Get(),
                inertia=np.diag(api.GetDiagonalInertiaAttr().Get()).reshape(9).tolist(),
                com=[list(api.GetCenterOfMassAttr().Get())],
                com_orientation=[[q.GetReal(),*q.GetImaginary()]])
    config['template_sha256']=template()[0]['template_sha256']
    Path(str(destination)+'.standard.json').write_text(json.dumps(config,indent=2)+'\n')
