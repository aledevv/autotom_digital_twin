"""
run_smooth_bridge.py — Test 1A runtime

Requires in the same directory:
    generate_curved_centerline.py
    generate_smooth_centerline.py

The bridge below is intentionally the same bridge validated in Test 0F.
"""

import os
import sys
import time

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})

import omni.usd
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

import generate_smooth_centerline as asset


LINK_PATHS = [
    f"/World/Stem/Branch_Link_{i + 1:02d}"
    for i in range(asset.NUM_LINKS)
]

SKEL_ROOT_PATH = "/World/StemVisual/SkelRoot"
ANIM_PATH = "/World/StemVisual/SkelRoot/SkelAnim"


def get_world_mats(cache, prims):
    cache.Clear()
    return [
        Gf.Matrix4d(cache.GetLocalToWorldTransform(p))
        for p in prims
    ]


def world_to_joint_local(link_world, skel_root_world):
    """
    Same validated conversion:
        Bone0 = Link0World * inverse(SkelRootWorld)
        BoneN = LinkNWorld * inverse(LinkN-1World)
    """
    result = []

    for i, world in enumerate(link_world):
        if i == 0:
            local = world * skel_root_world.GetInverse()
        else:
            local = world * link_world[i - 1].GetInverse()

        result.append(local)

    return result


def decompose(local_mats):
    translations = []
    rotations = []

    for m in local_mats:
        t = m.ExtractTranslation()
        q = m.ExtractRotationQuat()
        qi = q.GetImaginary()

        translations.append(
            Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))
        )
        rotations.append(
            Gf.Quatf(
                float(q.GetReal()),
                Gf.Vec3f(float(qi[0]), float(qi[1]), float(qi[2])),
            )
        )

    return Vt.Vec3fArray(translations), Vt.QuatfArray(rotations)


def sync_skin(
    cache,
    link_prims,
    skel_root_world,
    translations_attr,
    rotations_attr,
):
    link_world = get_world_mats(cache, link_prims)
    local_mats = world_to_joint_local(link_world, skel_root_world)

    translations, rotations = decompose(local_mats)

    translations_attr.Set(translations)
    rotations_attr.Set(rotations)

    return link_world, local_mats


def translation_error(a, b):
    return float(
        (
            a.ExtractTranslation()
            - b.ExtractTranslation()
        ).GetLength()
    )


def rotation_error_deg(a, b):
    rel = a * b.GetInverse()
    angle = abs(float(rel.ExtractRotation().GetAngle()))

    if angle > 180.0:
        angle = 360.0 - angle

    return angle


def main():
    print()
    print("=" * 72)
    print("TEST 1A — Smooth Centerline + validated PhysX bridge")
    print("=" * 72)

    asset.build_stage(asset.OUTPUT_USD)

    ctx = omni.usd.get_context()
    ctx.open_stage(asset.OUTPUT_USD)

    if World.instance() is not None:
        World.instance().clear_instance()

    world = World(
        stage_units_in_meters=1.0,
        physics_prim_path="/World/PhysicsScene",
    )

    stage = ctx.get_stage()

    link_prims = [
        stage.GetPrimAtPath(path)
        for path in LINK_PATHS
    ]

    for path, prim in zip(LINK_PATHS, link_prims):
        if not prim.IsValid():
            raise RuntimeError(f"Missing rigid link: {path}")

    skel_root_prim = stage.GetPrimAtPath(SKEL_ROOT_PATH)
    anim_prim = stage.GetPrimAtPath(ANIM_PATH)

    if not skel_root_prim.IsValid():
        raise RuntimeError("Missing SkelRoot")

    if not anim_prim.IsValid():
        raise RuntimeError("Missing SkelAnimation")

    anim = UsdSkel.Animation(anim_prim)
    translations_attr = anim.GetTranslationsAttr()
    rotations_attr = anim.GetRotationsAttr()

    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    cache.Clear()
    skel_root_world = Gf.Matrix4d(
        cache.GetLocalToWorldTransform(skel_root_prim)
    )

    # Verify that changing visual geometry did NOT change rest frames.
    link_world_before = get_world_mats(cache, link_prims)

    actual_rest = world_to_joint_local(
        link_world_before,
        skel_root_world,
    )
    expected_rest = asset.rest_local_transforms()

    print()
    print("[REST-FRAME CHECK]")

    max_t = 0.0
    max_r = 0.0

    for i, (actual, expected) in enumerate(
        zip(actual_rest, expected_rest)
    ):
        te = translation_error(actual, expected)
        re = rotation_error_deg(actual, expected)

        max_t = max(max_t, te)
        max_r = max(max_r, re)

        print(
            f"  Bone{i}: "
            f"translation={te * 1000.0:.6f} mm, "
            f"rotation={re:.6f} deg"
        )

    print(
        f"  max={max_t * 1000.0:.6f} mm, "
        f"{max_r:.6f} deg"
    )

    world.reset()

    # Immediate synchronization after PhysX reset.
    sync_skin(
        cache,
        link_prims,
        skel_root_world,
        translations_attr,
        rotations_attr,
    )

    print()
    print("[WHAT SHOULD BE DIFFERENT FROM 0F]")
    print()
    print("  0F:")
    print("      ______")
    print("            \\______")
    print("                   \\_____")
    print()
    print("  1A:")
    print("      ______")
    print("            `---.__")
    print("                  `---.__")
    print()
    print("The rest shape should have smooth transitions around")
    print("the control points instead of visible geometric knees.")
    print()
    print(f"Physics bones : {asset.NUM_LINKS}")
    print(f"Visual rings  : {len(asset.SMOOTH['positions'])}")
    print()
    print("The mesh still has only the SAME 3 physics bones.")
    print("The extra rings are visual resolution, not extra joints.")
    print()
    print("Then gravity should behave exactly like Test 0F.")
    print("No need to press Play.")
    print("=" * 72)

    step = 0
    last_log = 0
    start = time.time()

    while simulation_app.is_running():
        world.step(render=False)

        link_world, local_mats = sync_skin(
            cache,
            link_prims,
            skel_root_world,
            translations_attr,
            rotations_attr,
        )

        simulation_app.update()
        step += 1

        if step - last_log >= 240:
            p = link_world[-1].ExtractTranslation()

            print(
                f"[frame {step:6d}] "
                f"last link origin="
                f"({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})"
            )

            last_log = step

    elapsed = time.time() - start

    print()
    print("=" * 72)
    print(f"Finished: {step} frames in {elapsed:.1f}s")
    print()
    print("TEST 1A = GO if:")
    print("  [ ] rest tube is visibly smoother than 0F")
    print("  [ ] no frame-0 snap")
    print("  [ ] same slow gravity bending as 0F")
    print("  [ ] skin remains continuous")
    print("  [ ] no axial twist")
    print("=" * 72)

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
