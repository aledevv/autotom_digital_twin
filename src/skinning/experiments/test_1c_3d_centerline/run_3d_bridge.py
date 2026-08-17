"""run_3d_bridge.py — Test 1C runtime."""

import os
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

import omni.usd
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import generate_3d_centerline as asset

LINK_PATHS = [f"/World/Stem/Branch_Link_{i+1:02d}" for i in range(asset.NUM_LINKS)]
SKEL_ROOT_PATH = "/World/StemVisual/SkelRoot"
ANIM_PATH = "/World/StemVisual/SkelRoot/SkelAnim"


def get_world_mats(cache, prims):
    cache.Clear()
    return [Gf.Matrix4d(cache.GetLocalToWorldTransform(p)) for p in prims]


def world_to_joint_local(world_mats, skel_root_world):
    out = []
    for i, world in enumerate(world_mats):
        if i == 0:
            out.append(world * skel_root_world.GetInverse())
        else:
            out.append(world * world_mats[i - 1].GetInverse())
    return out


def decompose(local_mats):
    ts, rs = [], []
    for m in local_mats:
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        qi = q.GetImaginary()
        ts.append(Gf.Vec3f(float(t[0]), float(t[1]), float(t[2])))
        rs.append(Gf.Quatf(float(q.GetReal()), Gf.Vec3f(float(qi[0]), float(qi[1]), float(qi[2]))))
    return Vt.Vec3fArray(ts), Vt.QuatfArray(rs)


def sync_skin(cache, link_prims, skel_root_world, ta, ra):
    wm = get_world_mats(cache, link_prims)
    lm = world_to_joint_local(wm, skel_root_world)
    ts, rs = decompose(lm)
    ta.Set(ts)
    ra.Set(rs)
    return wm, lm


def translation_error(a, b):
    return float((a.ExtractTranslation() - b.ExtractTranslation()).GetLength())


def rotation_error_deg(a, b):
    rel = a * b.GetInverse()
    angle = abs(float(rel.ExtractRotation().GetAngle()))
    return 360.0 - angle if angle > 180.0 else angle


def main():
    print("\n" + "=" * 78)
    print("TEST 1C — True 3D Centerline + PhysX/Skeleton bridge")
    print("=" * 78)

    asset.build_stage(asset.OUTPUT_USD)
    ctx = omni.usd.get_context()
    ctx.open_stage(asset.OUTPUT_USD)

    if World.instance() is not None:
        World.instance().clear_instance()

    world = World(stage_units_in_meters=1.0, physics_prim_path="/World/PhysicsScene")
    stage = ctx.get_stage()

    link_prims = [stage.GetPrimAtPath(path) for path in LINK_PATHS]
    for path, prim in zip(LINK_PATHS, link_prims):
        if not prim.IsValid():
            raise RuntimeError(f"Missing rigid link: {path}")

    skel_root = stage.GetPrimAtPath(SKEL_ROOT_PATH)
    anim_prim = stage.GetPrimAtPath(ANIM_PATH)
    if not skel_root.IsValid():
        raise RuntimeError("Missing SkelRoot")
    if not anim_prim.IsValid():
        raise RuntimeError("Missing SkelAnimation")

    anim = UsdSkel.Animation(anim_prim)
    ta = anim.GetTranslationsAttr()
    ra = anim.GetRotationsAttr()

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    cache.Clear()
    skel_root_world = Gf.Matrix4d(cache.GetLocalToWorldTransform(skel_root))

    actual = world_to_joint_local(get_world_mats(cache, link_prims), skel_root_world)
    expected = asset.rest_local_transforms()

    print("\n[3D REST-FRAME CHECK]")
    max_t = 0.0
    max_r = 0.0
    for i, (a, e) in enumerate(zip(actual, expected)):
        te = translation_error(a, e)
        re = rotation_error_deg(a, e)
        max_t = max(max_t, te)
        max_r = max(max_r, re)
        print(f"  Bone{i}: translation={te*1000:.6f} mm  rotation={re:.6f} deg")
    print(f"  max: {max_t*1000:.6f} mm, {max_r:.6f} deg")

    if max_t < 1e-5 and max_r < 1e-4:
        print("  [OK] 3D PhysX and Skeleton rest frames coincide.")
    else:
        print("  [WARNING] 3D rest-frame mismatch.")

    world.reset()
    sync_skin(cache, link_prims, skel_root_world, ta, ra)

    print("\n[WHAT TO LOOK FOR]")
    print("  Inspect the branch from multiple camera angles.")
    print("  The rest shape must already vary in X, Y and Z.")
    print("  GO if:")
    print("    [ ] genuine spatial rest shape")
    print("    [ ] no frame-0 snap")
    print("    [ ] smooth centerline and taper")
    print("    [ ] no axial twist")
    print("    [ ] gravity still bends downward")
    print("    [ ] skin remains coherent")
    print("  No need to press Play.")
    print("=" * 78)

    step = 0
    last_log = 0
    start = time.time()

    while simulation_app.is_running():
        world.step(render=False)
        world_mats, _ = sync_skin(cache, link_prims, skel_root_world, ta, ra)
        simulation_app.update()
        step += 1

        if step - last_log >= 240:
            p = world_mats[-1].ExtractTranslation()
            print(f"[frame {step:6d}] last link origin=({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})")
            last_log = step

    elapsed = time.time() - start
    print(f"\nFinished: {step} frames in {elapsed:.1f}s")
    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
