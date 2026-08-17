"""
run_taper_bridge.py — Test 1B runtime

Runs the tapered visual mesh using the same validated PhysX -> Skeleton bridge.

Expected:
    - initial centerline remains smooth
    - branch is visibly thicker at the base and thinner at the tip
    - no frame-0 snap
    - gravity behavior remains the same as Test 1A
    - taper is preserved while the mesh deforms
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

import generate_taper_centerline as asset


LINK_PATHS = [
    f"/World/Stem/Branch_Link_{i + 1:02d}"
    for i in range(asset.NUM_LINKS)
]

SKEL_ROOT_PATH = "/World/StemVisual/SkelRoot"
ANIM_PATH = "/World/StemVisual/SkelRoot/SkelAnim"


def get_world_mats(cache, prims):
    cache.Clear()

    return [
        Gf.Matrix4d(
            cache.GetLocalToWorldTransform(p)
        )
        for p in prims
    ]


def world_to_joint_local(
    link_world_mats,
    skel_root_world,
):
    result = []

    for i, world in enumerate(link_world_mats):
        if i == 0:
            local = (
                world
                * skel_root_world.GetInverse()
            )
        else:
            local = (
                world
                * link_world_mats[i - 1].GetInverse()
            )

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
            Gf.Vec3f(
                float(t[0]),
                float(t[1]),
                float(t[2]),
            )
        )

        rotations.append(
            Gf.Quatf(
                float(q.GetReal()),
                Gf.Vec3f(
                    float(qi[0]),
                    float(qi[1]),
                    float(qi[2]),
                ),
            )
        )

    return (
        Vt.Vec3fArray(translations),
        Vt.QuatfArray(rotations),
    )


def sync_skin(
    cache,
    link_prims,
    skel_root_world,
    translations_attr,
    rotations_attr,
):
    link_world = get_world_mats(
        cache,
        link_prims,
    )

    local_mats = world_to_joint_local(
        link_world,
        skel_root_world,
    )

    translations, rotations = decompose(
        local_mats
    )

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

    angle = abs(
        float(
            rel.ExtractRotation().GetAngle()
        )
    )

    if angle > 180.0:
        angle = 360.0 - angle

    return angle


def main():
    print()
    print("=" * 74)
    print("TEST 1B — Taper + validated PhysX/Skeleton bridge")
    print("=" * 74)

    asset.build_stage(
        asset.OUTPUT_USD
    )

    ctx = omni.usd.get_context()
    ctx.open_stage(
        asset.OUTPUT_USD
    )

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

    for path, prim in zip(
        LINK_PATHS,
        link_prims,
    ):
        if not prim.IsValid():
            raise RuntimeError(
                f"Missing rigid link: {path}"
            )

    skel_root_prim = stage.GetPrimAtPath(
        SKEL_ROOT_PATH
    )
    anim_prim = stage.GetPrimAtPath(
        ANIM_PATH
    )

    if not skel_root_prim.IsValid():
        raise RuntimeError(
            "Missing SkelRoot"
        )

    if not anim_prim.IsValid():
        raise RuntimeError(
            "Missing SkelAnimation"
        )

    anim = UsdSkel.Animation(
        anim_prim
    )

    translations_attr = (
        anim.GetTranslationsAttr()
    )
    rotations_attr = (
        anim.GetRotationsAttr()
    )

    cache = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    )

    cache.Clear()
    skel_root_world = Gf.Matrix4d(
        cache.GetLocalToWorldTransform(
            skel_root_prim
        )
    )

    # Rest-frame check: taper MUST NOT change skeleton rest transforms.
    link_world_before = get_world_mats(
        cache,
        link_prims,
    )

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
        te = translation_error(
            actual,
            expected,
        )
        re = rotation_error_deg(
            actual,
            expected,
        )

        max_t = max(max_t, te)
        max_r = max(max_r, re)

        print(
            f"  Bone{i}: "
            f"{te * 1000.0:.6f} mm, "
            f"{re:.6f} deg"
        )

    print(
        f"  max: "
        f"{max_t * 1000.0:.6f} mm, "
        f"{max_r:.6f} deg"
    )

    world.reset()

    sync_skin(
        cache,
        link_prims,
        skel_root_world,
        translations_attr,
        rotations_attr,
    )

    print()
    print("[WHAT TO LOOK FOR]")
    print()
    print("  BASE                         TIP")
    print("  ██████████ → ███████ → ███ → ██")
    print()
    print(
        f"  base radius = "
        f"{asset.BASE_RADIUS * 1000.0:.2f} mm"
    )
    print(
        f"  tip radius  = "
        f"{asset.TIP_RADIUS * 1000.0:.2f} mm"
    )
    print()
    print("  GO if:")
    print("    [ ] taper is smooth and monotonic")
    print("    [ ] no sudden radius step")
    print("    [ ] same smooth rest centerline as 1A")
    print("    [ ] no frame-0 snap")
    print("    [ ] same gravity behavior as 1A")
    print("    [ ] taper remains coherent during deformation")
    print()
    print("No need to press Play.")
    print("=" * 74)

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
            p = (
                link_world[-1]
                .ExtractTranslation()
            )

            print(
                f"[frame {step:6d}] "
                f"last link origin="
                f"({p[0]:+.3f}, "
                f"{p[1]:+.3f}, "
                f"{p[2]:+.3f})"
            )

            last_log = step

    elapsed = time.time() - start

    print()
    print("=" * 74)
    print(
        f"Finished: {step} frames "
        f"in {elapsed:.1f}s"
    )
    print()
    print("TEST 1B = GO if:")
    print("  [ ] smooth taper visible")
    print("  [ ] physics behavior unchanged")
    print("  [ ] skinning remains stable")
    print("=" * 74)

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
