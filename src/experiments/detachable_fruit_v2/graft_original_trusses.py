"""Experimental USD graft: original rank6-10 trusses onto v2.3 attachments.

Uses a freshly exported standard-truss scaffold only for canonical attachment
frames. Replaces every retained truss body with its rank-specific reference;
removes lateral trusses. Vegetative body/joint attributes are preserved.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'src'))
from exporterV2.standard_truss import upright_attachment
from exporterV2.fruit_experiments import audit


def attrs(stage):
    return {str(a.GetPath()):str(a.Get()) for p in stage.Traverse() if '/Vegetative/' in str(p.GetPath()) for a in p.GetAttributes()}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--scaffold',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    reference=ROOT/'artifacts/detachable_fruit_v2/original-rank-campaign/five-rest'
    source=Usd.Stage.Open(str(reference/'scene.usda'))
    stage=Usd.Stage.Open(Usd.Stage.Open(str(a.scaffold)).Flatten())
    assert UsdGeom.GetStageMetersPerUnit(source)==UsdGeom.GetStageMetersPerUnit(stage)
    assert UsdGeom.GetStageUpAxis(source)==UsdGeom.GetStageUpAxis(stage)
    before=attrs(stage);cache=UsdGeom.XformCache();selected=[];old_metadata={};removed=[]
    for prim in stage.Traverse():
        path=str(prim.GetPath())
        if prim.GetAttribute('autotom:standardTruss').Get():
            old_metadata[path]={x.GetName():x.Get() for x in prim.GetAttributes() if x.GetName().startswith('autotom:')}
            if path.endswith('_rachis_Link_01'):
                joint=UsdPhysics.Joint(stage.GetPrimAtPath(path+'/AttachJoint'))
                parent=str(joint.GetBody0Rel().GetTargets()[0])
                if '/Vegetative/trunk/' in parent:
                    rank=int(re.search(r'Truss_r(\d+)_',path)[1]);assert rank in range(6,11)
                    selected.append(dict(rank=rank,path=path,parent=parent,frame=cache.GetLocalToWorldTransform(prim)))
    assert sorted(x['rank'] for x in selected)==list(range(6,11))
    for path in old_metadata:
        removed.append(path);stage.RemovePrim(path)
    for scope in ('Looks','Materials'):
        if source.GetPrimAtPath('/World/'+scope):
            Sdf.CopySpec(source.GetRootLayer(),'/World/'+scope,stage.GetRootLayer(),'/World/OriginalTruss'+scope)
    effective=json.loads((reference/'headless-effective.json').read_text())
    expected={};mappings=[]
    for item in selected:
        rank=item['rank'];prefix=f'Truss_r{rank}_o0';newprefix=item['path'].split('/')[-1].removesuffix('_rachis_Link_01')
        bodies=[b for b in effective['bodies'] if prefix+'_' in b['path']]
        assert len(bodies)==20
        mapping={b['path']:b['path'].replace(prefix,newprefix) for b in bodies}
        root=f'/World/Stem/{prefix}_rachis_Link_01'
        source_frame=cache.GetLocalToWorldTransform(source.GetPrimAtPath(root))
        target_frame=upright_attachment(source_frame,item['frame'])
        transform=source_frame.GetInverse()*target_frame
        for b in bodies:
            old=b['path'];new=mapping[old]
            UsdGeom.Xform.Define(stage,str(Sdf.Path(new).GetParentPath()))
            Sdf.CopySpec(source.GetRootLayer(),old,stage.GetRootLayer(),new)
            prim=stage.GetPrimAtPath(new);matrix=cache.GetLocalToWorldTransform(source.GetPrimAtPath(old))*transform
            prim.GetAttribute('xformOp:translate').Set(matrix.ExtractTranslation())
            prim.GetAttribute('xformOp:orient').Set(Gf.Quatf(matrix.ExtractRotationQuat()))
            for child in Usd.PrimRange(prim):
                for rel in child.GetRelationships():
                    targets=[]
                    for t in rel.GetTargets():
                        text=str(t);match=next((k for k in mapping if text==k or text.startswith(k+'/')),None)
                        if old==root and rel.GetName()=='physics:body0':newt=item['parent']
                        elif match:newt=mapping[match]+text[len(match):]
                        elif text.startswith('/World/Looks/'):newt=text.replace('/World/Looks/','/World/OriginalTrussLooks/')
                        elif text.startswith('/World/Materials/'):newt=text.replace('/World/Materials/','/World/OriginalTrussMaterials/')
                        else:newt=text
                        if stage.GetPrimAtPath(Sdf.Path(newt).GetPrimPath()) or match:targets.append(Sdf.Path(newt))
                    rel.SetTargets(targets)
            for name,value in old_metadata[new].items():
                prim.CreateAttribute(name,Sdf.ValueTypeNames.String).Set(value) if isinstance(value,str) else None
            prim.CreateAttribute('autotom:standardTruss',Sdf.ValueTypeNames.String).Set('main-original-ranks-experiment')
            prim.CreateAttribute('autotom:role',Sdf.ValueTypeNames.String).Set(b['role'])
            mass=UsdPhysics.MassAPI(prim)
            mat=np.asarray(b['inertia']).reshape(3,3);assert np.allclose(mat,np.diag(np.diag(mat)),atol=1e-16)
            mass.CreateMassAttr().Set(b['mass_kg']);mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*np.diag(mat)))
            mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(*np.asarray(b['com']).reshape(3)))
            q=np.asarray(b['com_orientation']).reshape(4);mass.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*q[1:])))
            expected[new]={k:b[k] for k in ('mass_kg','inertia','com','com_orientation')}
        joint=UsdPhysics.Joint(stage.GetPrimAtPath(mapping[root]+'/AttachJoint'))
        local1=Gf.Matrix4d().SetRotate(joint.GetLocalRot1Attr().Get());local1.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos1Attr().Get()))
        current=UsdGeom.XformCache();local0=local1*current.GetLocalToWorldTransform(stage.GetPrimAtPath(mapping[root]))*current.GetLocalToWorldTransform(stage.GetPrimAtPath(item['parent'])).GetInverse()
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(local0.ExtractTranslation()));joint.GetLocalRot0Attr().Set(Gf.Quatf(local0.ExtractRotationQuat()))
        mappings.append(dict(rank=rank,parent=item['parent'],bodies=mapping,transform=np.asarray(transform).tolist()))
    dangling=[]
    for prim in stage.Traverse():
        for rel in prim.GetRelationships():
            old=rel.GetTargets();new=[x for x in old if stage.GetPrimAtPath(x.GetPrimPath())]
            if old!=new:
                assert 'filter' in rel.GetName().lower() or 'collection' in rel.GetName().lower(),str(rel)
                dangling.append(dict(path=str(rel.GetPath()),removed=[str(x) for x in old if x not in new]));rel.SetTargets(new)
    assert attrs(stage)==before, 'Vegetation attributes changed'
    scene=stage.GetPrimAtPath('/World/PhysicsScene')
    scene.GetAttribute('physxScene:solverType').Set('PGS');scene.GetAttribute('physxScene:enableGPUDynamics').Set(False)
    stage.GetPrimAtPath('/World/Stem').GetAttribute('autotom:experimentalTrussPreset').Set('main-original-ranks-experiment')
    check=audit(stage);assert not check['errors'],check['errors']
    stage.GetRootLayer().Export(str(a.output/'scene.usda'))
    config=json.loads((reference/'config.json').read_text());config.update(run_dir=str(a.output.resolve()),force_target=None,record_articulation_dofs=True,expected_body_properties=expected)
    config['scene_sha256']=hashlib.sha256((a.output/'scene.usda').read_bytes()).hexdigest()
    config['v23_graft']=dict(scaffold=str(a.scaffold),scaffold_sha256=hashlib.sha256(a.scaffold.read_bytes()).hexdigest(),source=str(reference),source_sha256=hashlib.sha256((reference/'scene.usda').read_bytes()).hexdigest(),mappings=mappings,removed_standard_bodies=removed,pruned_filters=dangling,vegetative_attributes_unchanged=True)
    for rel in ('src/experiments/detachable_fruit_v2/graft_original_trusses.py','src/exporterV2/fruit_diagnostics.py'):
        config['implementation_sha256'][rel]=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
    (a.output/'config.json').write_text(json.dumps(config,indent=2)+'\n');(a.output/'audit.json').write_text(json.dumps(check,indent=2)+'\n')
    print('Prepared',a.output,'with',len(expected),'original truss bodies')


if __name__=='__main__':main()
