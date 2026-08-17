"""Test 0F runtime: curved rest centerline + PhysX -> UsdSkel bridge."""
import os, sys, time
from isaacsim import SimulationApp
simulation_app=SimulationApp({'headless':False,'width':1280,'height':720})

import omni.usd
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

SCRIPT_DIR=os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path: sys.path.insert(0,SCRIPT_DIR)
import generate_curved_centerline as asset

NUM_LINKS=asset.NUM_LINKS; OUTPUT_USD=asset.OUTPUT_USD
LINK_PATHS=[f'/World/Stem/Branch_Link_{i+1:02d}' for i in range(NUM_LINKS)]
SKEL_ROOT_PATH='/World/StemVisual/SkelRoot'; ANIM_PATH='/World/StemVisual/SkelRoot/SkelAnim'

def get_world_mats(cache,prims):
    cache.Clear()
    return [Gf.Matrix4d(cache.GetLocalToWorldTransform(p)) for p in prims]

def world_to_joint_local(world_mats,skel_root_world):
    out=[]
    for i,w in enumerate(world_mats):
        if i==0: out.append(w*skel_root_world.GetInverse())
        else: out.append(w*world_mats[i-1].GetInverse())
    return out

def decompose(local_mats):
    ts=[]; rs=[]
    for m in local_mats:
        t=m.ExtractTranslation(); q=m.ExtractRotationQuat(); qi=q.GetImaginary()
        ts.append(Gf.Vec3f(float(t[0]),float(t[1]),float(t[2])))
        rs.append(Gf.Quatf(float(q.GetReal()),Gf.Vec3f(float(qi[0]),float(qi[1]),float(qi[2]))))
    return Vt.Vec3fArray(ts),Vt.QuatfArray(rs)

def translation_error(a,b):
    return float((a.ExtractTranslation()-b.ExtractTranslation()).GetLength())

def rotation_error_deg(a,b):
    rel=a*b.GetInverse(); ang=abs(float(rel.ExtractRotation().GetAngle()))
    return 360.0-ang if ang>180.0 else ang

def sync_skin(cache,link_prims,skel_root_world,ta,ra):
    wm=get_world_mats(cache,link_prims); lm=world_to_joint_local(wm,skel_root_world)
    ts,rs=decompose(lm); ta.Set(ts); ra.Set(rs); return wm,lm

def main():
    print('\n'+'='*74); print('TEST 0F — Curved centerline rest pose + PhysX -> skin'); print('='*74)
    asset.build_stage(OUTPUT_USD)
    ctx=omni.usd.get_context(); ctx.open_stage(OUTPUT_USD)
    if World.instance() is not None: World.instance().clear_instance()
    world=World(stage_units_in_meters=1.0,physics_prim_path='/World/PhysicsScene')
    stage=ctx.get_stage(); link_prims=[stage.GetPrimAtPath(p) for p in LINK_PATHS]
    for path,p in zip(LINK_PATHS,link_prims):
        if not p.IsValid(): raise RuntimeError(f'Missing rigid link: {path}')
    skel_root=stage.GetPrimAtPath(SKEL_ROOT_PATH); anim_prim=stage.GetPrimAtPath(ANIM_PATH)
    if not skel_root.IsValid(): raise RuntimeError('Missing SkelRoot')
    if not anim_prim.IsValid(): raise RuntimeError('Missing SkelAnim')
    anim=UsdSkel.Animation(anim_prim); ta=anim.GetTranslationsAttr(); ra=anim.GetRotationsAttr()
    cache=UsdGeom.XformCache(Usd.TimeCode.Default()); cache.Clear()
    skel_root_world=Gf.Matrix4d(cache.GetLocalToWorldTransform(skel_root))

    # Verify complete rest frames BEFORE PhysX starts.
    actual=world_to_joint_local(get_world_mats(cache,link_prims),skel_root_world)
    expected=asset.rest_local_transforms(); max_t=0.0; max_r=0.0
    print('\n[REST-POSE CHECK]')
    for i,(a,e) in enumerate(zip(actual,expected)):
        te=translation_error(a,e); re=rotation_error_deg(a,e); max_t=max(max_t,te); max_r=max(max_r,re)
        print(f'  Bone{i}: translation={te*1000:.6f} mm  rotation={re:.6f} deg')
    print(f'  max: {max_t*1000:.6f} mm, {max_r:.6f} deg')
    if max_t<1e-5 and max_r<1e-4: print('  [OK] PhysX and Skeleton rest frames coincide.')
    else: print('  [WARNING] Rest-frame mismatch detected — inspect before trusting motion.')

    world.reset()
    # Re-sync after reset in case PhysX normalizes authored transforms.
    sync_skin(cache,link_prims,skel_root_world,ta,ra)

    print('\n[EXPECTED]')
    print('  Start: already curved in XY (top-view shape roughly like an arc).')
    print('  Then: gravity bends the curved chain downward toward -Z.')
    print('  There should be NO startup straightening/snap, freeze, or axial twist.')
    print('  No need to press Play; this script advances World.step().')
    print('='*74)

    step=0; last_log=0; start=time.time()
    while simulation_app.is_running():
        world.step(render=False)
        wm,lm=sync_skin(cache,link_prims,skel_root_world,ta,ra)
        simulation_app.update()
        step+=1
        if step-last_log>=240:
            tip=wm[-1].ExtractTranslation(); q=lm[1].ExtractRotationQuat(); qi=q.GetImaginary()
            print(f'[frame {step:6d}] Link2 origin=({tip[0]:+.3f},{tip[1]:+.3f},{tip[2]:+.3f})  '
                  f'Bone1.local.q=({q.GetReal():+.3f},{qi[0]:+.3f},{qi[1]:+.3f},{qi[2]:+.3f})')
            last_log=step

    elapsed=time.time()-start
    print('\n'+'='*74); print(f'Finished: {step} frames in {elapsed:.1f}s')
    print('GO if: curved rest pose preserved; zero snap; gravity bends downward; skin follows; no twist.')
    print('='*74)
    simulation_app.close()

if __name__=='__main__':
    try: main()
    finally:
        if simulation_app.is_running(): simulation_app.close()
