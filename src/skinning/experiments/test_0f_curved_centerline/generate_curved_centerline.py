"""Test 0F asset: curved centerline rest pose + PhysX + UsdSkel."""
import math, os, sys
try:
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdSkel, PhysxSchema, Sdf, Vt
except ImportError:
    print('[ERROR] Use Isaac Sim python.'); sys.exit(1)

SCRIPT_DIR=os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR=os.path.join(SCRIPT_DIR,'output')
OUTPUT_USD=os.path.join(OUTPUT_DIR,'test_0f_curved_centerline.usda')

NUM_LINKS=3; NUM_BONES=NUM_LINKS
LINK_HEIGHT=0.10; LINK_RADIUS=0.012; LINK_MASS=0.05; GAP=0.001; BASE_Z=0.10
JOINT_STIFF=0.0; JOINT_DAMP=0.05; BEND_LIMIT=89.0
TUBE_RADIUS=LINK_RADIUS; RADIAL_SEGMENTS=14; RINGS_PER_LINK=12
BLEND_HALF_WIDTH=LINK_HEIGHT*0.18
REST_ANGLES_DEG=[0.0,20.0,40.0]   # 0 deg = +Y, positive bends toward +X
SHOW_PHYSICS_COLLIDERS=True

def norm(v):
    v=Gf.Vec3d(v); l=v.GetLength()
    if l < 1e-10: raise ValueError('zero vector')
    return v/l

def tangent_from_angle(deg):
    a=math.radians(deg)
    return norm(Gf.Vec3d(math.sin(a),math.cos(a),0.0))

def rotation_from_tangent(t):
    return Gf.Rotation(Gf.Vec3d(0,0,1),norm(t))

def quatf_from_rotation(r):
    q=r.GetQuat(); i=q.GetImaginary()
    return Gf.Quatf(float(q.GetReal()),Gf.Vec3f(float(i[0]),float(i[1]),float(i[2])))

def quatf_from_matrix(m):
    q=m.ExtractRotationQuat(); i=q.GetImaginary()
    return Gf.Quatf(float(q.GetReal()),Gf.Vec3f(float(i[0]),float(i[1]),float(i[2])))

def pose_matrix(p,r):
    m=Gf.Matrix4d(1.0); m.SetTransform(r,p); return m

def build_centerline():
    if len(REST_ANGLES_DEG)!=NUM_LINKS: raise ValueError('REST_ANGLES_DEG size mismatch')
    tangents=[tangent_from_angle(a) for a in REST_ANGLES_DEG]
    rotations=[rotation_from_tangent(t) for t in tangents]
    step=LINK_HEIGHT+GAP
    origins=[Gf.Vec3d(0,0,BASE_Z)]
    for i in range(1,NUM_LINKS):
        origins.append(origins[-1]+tangents[i-1]*step)
    bind=[pose_matrix(origins[i],rotations[i]) for i in range(NUM_LINKS)]
    tip=origins[-1]+tangents[-1]*LINK_HEIGHT
    nodes=list(origins)+[tip]
    arc=[0.0]
    for i in range(1,len(nodes)):
        arc.append(arc[-1]+(nodes[i]-nodes[i-1]).GetLength())
    return dict(tangents=tangents,rotations=rotations,origins=origins,bind=bind,nodes=nodes,arc=arc)

CENTERLINE=build_centerline()

def joint_names():
    out=['Bone0']
    for i in range(1,NUM_BONES): out.append(out[-1]+f'/Bone{i}')
    return out

def bind_skel_transforms():
    return [Gf.Matrix4d(m) for m in CENTERLINE['bind']]

def rest_local_transforms():
    b=bind_skel_transforms(); out=[Gf.Matrix4d(b[0])]
    for i in range(1,NUM_BONES): out.append(b[i]*b[i-1].GetInverse())
    return out

def apply_physx_scene(stage):
    scene=UsdPhysics.Scene.Define(stage,'/World/PhysicsScene')
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1)); scene.CreateGravityMagnitudeAttr().Set(9.81)
    px=PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateSolverTypeAttr().Set('TGS'); px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableCCDAttr().Set(True); px.CreateEnableStabilizationAttr().Set(True)
    px.CreateEnableGPUDynamicsAttr().Set(True); px.CreateBroadphaseTypeAttr().Set('MBP')

def create_link(stage,stem_path,i):
    path=f'{stem_path}/Branch_Link_{i+1:02d}'
    xf=UsdGeom.Xform.Define(stage,path)
    xf.AddTranslateOp().Set(CENTERLINE['origins'][i])
    xf.AddOrientOp().Set(quatf_from_rotation(CENTERLINE['rotations'][i]))
    col=UsdGeom.Cylinder.Define(stage,f'{path}/Collider')
    col.CreateHeightAttr().Set(LINK_HEIGHT); col.CreateRadiusAttr().Set(LINK_RADIUS); col.CreateAxisAttr().Set('Z')
    UsdGeom.Xformable(col.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(0,0,LINK_HEIGHT/2))
    if not SHOW_PHYSICS_COLLIDERS: UsdGeom.Imageable(col.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim()).CreateRigidBodyEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr().Set(LINK_MASS)
    return path

def configure_d6(j):
    for ax in ('transX','transY','transZ'):
        lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),ax); lim.CreateLowAttr().Set(1.0); lim.CreateHighAttr().Set(-1.0)
    for ax in ('rotX','rotY'):
        lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),ax); lim.CreateLowAttr().Set(-BEND_LIMIT); lim.CreateHighAttr().Set(BEND_LIMIT)
        drv=UsdPhysics.DriveAPI.Apply(j.GetPrim(),ax); drv.CreateTypeAttr().Set('force')
        drv.CreateStiffnessAttr().Set(JOINT_STIFF); drv.CreateDampingAttr().Set(JOINT_DAMP); drv.CreateTargetPositionAttr().Set(0.0)
    lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),'rotZ'); lim.CreateLowAttr().Set(1.0); lim.CreateHighAttr().Set(-1.0)

def create_joint(stage,parent,child,parent_i,child_i):
    j=UsdPhysics.Joint.Define(stage,f'{child}/Joint_{parent_i+1:02d}_{child_i+1:02d}')
    j.CreateBody0Rel().SetTargets([Sdf.Path(parent)]); j.CreateBody1Rel().SetTargets([Sdf.Path(child)])
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(0,0,LINK_HEIGHT+GAP)); j.CreateLocalPos1Attr().Set(Gf.Vec3f(0,0,0))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(1,0,0,0))
    # Curved rest pose: make body1's joint frame coincide with body0's frame.
    # Gf row-vector convention: jointWorld = jointLocal * bodyWorld,
    # therefore local1 = parentWorld * inverse(childWorld).
    p=CENTERLINE['bind'][parent_i]; c=CENTERLINE['bind'][child_i]
    j.CreateLocalRot1Attr().Set(quatf_from_matrix(p*c.GetInverse()))
    configure_d6(j)

def build_physics(stage):
    stem_path='/World/Stem'; stem=UsdGeom.Xform.Define(stage,stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem.GetPrim())
    paths=[]
    for i in range(NUM_LINKS):
        p=create_link(stage,stem_path,i)
        if i==0:
            fj=UsdPhysics.FixedJoint.Define(stage,f'{p}/RootFixedJoint'); fj.CreateBody1Rel().SetTargets([Sdf.Path(p)])
        else: create_joint(stage,paths[-1],p,i-1,i)
        paths.append(p)
    art=PhysxSchema.PhysxArticulationAPI.Apply(stem.GetPrim())
    art.CreateSolverPositionIterationCountAttr().Set(32); art.CreateSolverVelocityIterationCountAttr().Set(1)
    art.CreateEnabledSelfCollisionsAttr().Set(False); art.CreateSleepThresholdAttr().Set(0.0)
    return paths

def point_tangent_at_arc(s):
    nodes=CENTERLINE['nodes']; arc=CENTERLINE['arc']; s=max(0.0,min(float(s),arc[-1]))
    seg=len(nodes)-2
    for i in range(len(nodes)-1):
        if s <= arc[i+1]+1e-12: seg=i; break
    p0,p1=nodes[seg],nodes[seg+1]; length=arc[seg+1]-arc[seg]
    u=0.0 if length<1e-12 else (s-arc[seg])/length
    return p0+(p1-p0)*u, norm(p1-p0)

def weights_for_arc(s):
    joints=CENTERLINE['arc'][:-1]
    for child in range(1,NUM_BONES):
        c=joints[child]; lo=c-BLEND_HALF_WIDTH; hi=c+BLEND_HALF_WIDTH
        if lo<=s<=hi:
            u=max(0.0,min(1.0,(s-lo)/(hi-lo))); return child-1,child,1-u,u
    bone=0
    for i in range(1,NUM_BONES):
        if s>=joints[i]: bone=i
        else: break
    return bone,bone,1.0,0.0

def build_tube_data():
    total=CENTERLINE['arc'][-1]; rings=NUM_LINKS*RINGS_PER_LINK+1
    points=[]; ji=[]; jw=[]; up=Gf.Vec3d(0,0,1)
    for r in range(rings):
        s=(r/(rings-1))*total; center,tan=point_tangent_at_arc(s); side=norm(Gf.Cross(up,tan))
        b0,b1,w0,w1=weights_for_arc(s)
        for k in range(RADIAL_SEGMENTS):
            a=2*math.pi*k/RADIAL_SEGMENTS
            p=center+side*(math.cos(a)*TUBE_RADIUS)+up*(math.sin(a)*TUBE_RADIUS)
            points.append(Gf.Vec3f(float(p[0]),float(p[1]),float(p[2])))
            ji.extend([b0,b1]); jw.extend([w0,w1])
    fc=[]; fi=[]
    for r in range(rings-1):
        a=r*RADIAL_SEGMENTS; b=(r+1)*RADIAL_SEGMENTS
        for k in range(RADIAL_SEGMENTS):
            kn=(k+1)%RADIAL_SEGMENTS; v00=a+k; v01=a+kn; v10=b+k; v11=b+kn
            fc.extend([3,3]); fi.extend([v00,v10,v11,v00,v11,v01])
    return points,fc,fi,ji,jw

def build_visual(stage):
    UsdGeom.Xform.Define(stage,'/World/StemVisual')
    UsdSkel.Root.Define(stage,'/World/StemVisual/SkelRoot')
    sk=UsdSkel.Skeleton.Define(stage,'/World/StemVisual/SkelRoot/Skeleton')
    anim=UsdSkel.Animation.Define(stage,'/World/StemVisual/SkelRoot/SkelAnim')
    names=joint_names(); bind=bind_skel_transforms(); rest=rest_local_transforms()
    sk.CreateJointsAttr().Set(Vt.TokenArray(names)); sk.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(bind)); sk.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest))
    anim.CreateJointsAttr().Set(Vt.TokenArray(names))
    ts=[]; rs=[]
    for m in rest:
        t=m.ExtractTranslation(); q=m.ExtractRotationQuat(); qi=q.GetImaginary()
        ts.append(Gf.Vec3f(float(t[0]),float(t[1]),float(t[2])))
        rs.append(Gf.Quatf(float(q.GetReal()),Gf.Vec3f(float(qi[0]),float(qi[1]),float(qi[2]))))
    anim.CreateTranslationsAttr().Set(Vt.Vec3fArray(ts)); anim.CreateRotationsAttr().Set(Vt.QuatfArray(rs)); anim.CreateScalesAttr().Set(Vt.Vec3hArray([Gf.Vec3h(1,1,1)]*NUM_BONES))
    UsdSkel.BindingAPI.Apply(sk.GetPrim()).CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    mesh=UsdGeom.Mesh.Define(stage,'/World/StemVisual/SkelRoot/TubeMesh'); pts,fc,fi,ji,jw=build_tube_data()
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(pts)); mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(fc)); mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none); mesh.CreateDoubleSidedAttr().Set(True); mesh.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(.28,.56,.19)]))
    mb=UsdSkel.BindingAPI.Apply(mesh.GetPrim()); mb.CreateSkeletonRel().SetTargets([sk.GetPath()]); mb.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))
    ip=mb.CreateJointIndicesPrimvar(constant=False,elementSize=2); ip.SetInterpolation(UsdGeom.Tokens.vertex); ip.Set(Vt.IntArray(ji))
    wp=mb.CreateJointWeightsPrimvar(constant=False,elementSize=2); wp.SetInterpolation(UsdGeom.Tokens.vertex); wp.Set(Vt.FloatArray(jw))

def build_ground(stage):
    g=UsdGeom.Mesh.Define(stage,'/World/Ground'); s=.6
    g.CreatePointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(-s,-s,0),Gf.Vec3f(s,-s,0),Gf.Vec3f(s,s,0),Gf.Vec3f(-s,s,0)]))
    g.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4])); g.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([0,1,2,3])); g.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    g.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(.25,.22,.18)])); UsdPhysics.CollisionAPI.Apply(g.GetPrim())

def build_stage(output_path=OUTPUT_USD):
    os.makedirs(os.path.dirname(output_path),exist_ok=True); stage=Usd.Stage.CreateNew(output_path)
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage,1.0); UsdPhysics.SetStageKilogramsPerUnit(stage,1.0)
    world=UsdGeom.Xform.Define(stage,'/World'); stage.SetDefaultPrim(world.GetPrim())
    apply_physx_scene(stage); links=build_physics(stage); build_visual(stage); build_ground(stage); stage.Save()
    print('='*70); print('TEST 0F asset — curved centerline rest pose'); print('[OK]',output_path)
    for i,a in enumerate(REST_ANGLES_DEG):
        p=CENTERLINE['origins'][i]; print(f'  Link/Bone{i}: angle={a:+.1f} deg origin=({p[0]:+.4f},{p[1]:+.4f},{p[2]:+.4f})')
    print('Run: ~/isaacsim/python.sh run_curved_bridge.py'); print('='*70)
    return output_path

if __name__=='__main__': build_stage()
