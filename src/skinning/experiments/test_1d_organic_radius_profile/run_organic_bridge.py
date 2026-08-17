"""
run_organic_bridge.py — Test 1D runtime

Organic radius profile + validated Test 1C PhysX/Skeleton bridge.

Look for:
    - smooth global taper
    - subtle local swelling around internal nodes
    - very light longitudinal radius variation
    - no radius discontinuities
    - no change in physics/skinning behavior
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


SCRIPT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

if SCRIPT_DIR not in sys.path:
    sys.path.insert(
        0,
        SCRIPT_DIR,
    )

import generate_organic_radius as asset


LINK_PATHS = [
    (
        f"/World/Stem/"
        f"Branch_Link_{i + 1:02d}"
    )
    for i in range(asset.NUM_LINKS)
]

SKEL_ROOT_PATH = (
    "/World/StemVisual/SkelRoot"
)

ANIM_PATH = (
    "/World/StemVisual/SkelRoot/SkelAnim"
)


def get_world_mats(cache, prims):
    cache.Clear()

    return [
        Gf.Matrix4d(
            cache.GetLocalToWorldTransform(p)
        )
        for p in prims
    ]


def world_to_joint_local(
    world_mats,
    skel_root_world,
):
    out = []

    for i, world in enumerate(
        world_mats
    ):
        if i == 0:
            local = (
                world
                * skel_root_world.GetInverse()
            )
        else:
            local = (
                world
                * world_mats[
                    i - 1
                ].GetInverse()
            )

        out.append(local)

    return out


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
    translation_attr,
    rotation_attr,
):
    world_mats = get_world_mats(
        cache,
        link_prims,
    )

    local_mats = world_to_joint_local(
        world_mats,
        skel_root_world,
    )

    translations, rotations = (
        decompose(local_mats)
    )

    translation_attr.Set(
        translations
    )
    rotation_attr.Set(
        rotations
    )

    return world_mats, local_mats


def translation_error(a, b):
    return float(
        (
            a.ExtractTranslation()
            - b.ExtractTranslation()
        ).GetLength()
    )


def rotation_error_deg(a, b):
    relative = (
        a * b.GetInverse()
    )

    angle = abs(
        float(
            relative
            .ExtractRotation()
            .GetAngle()
        )
    )

    if angle > 180.0:
        angle = (
            360.0 - angle
        )

    return angle


def main():
    print()
    print("=" * 78)
    print(
        "TEST 1D — Organic Radius Profile "
        "+ validated 3D bridge"
    )
    print("=" * 78)

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
        physics_prim_path=(
            "/World/PhysicsScene"
        ),
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

    skel_root = (
        stage.GetPrimAtPath(
            SKEL_ROOT_PATH
        )
    )

    anim_prim = (
        stage.GetPrimAtPath(
            ANIM_PATH
        )
    )

    if not skel_root.IsValid():
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

    translation_attr = (
        anim.GetTranslationsAttr()
    )
    rotation_attr = (
        anim.GetRotationsAttr()
    )

    cache = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    )

    cache.Clear()

    skel_root_world = Gf.Matrix4d(
        cache.GetLocalToWorldTransform(
            skel_root
        )
    )

    # Radius profile MUST NOT affect skeleton rest frames.
    actual = world_to_joint_local(
        get_world_mats(
            cache,
            link_prims,
        ),
        skel_root_world,
    )

    expected = (
        asset.rest_local_transforms()
    )

    print()
    print("[REST-FRAME CHECK]")

    max_translation = 0.0
    max_rotation = 0.0

    for i, (a, e) in enumerate(
        zip(actual, expected)
    ):
        te = translation_error(a, e)
        re = rotation_error_deg(a, e)

        max_translation = max(
            max_translation,
            te,
        )
        max_rotation = max(
            max_rotation,
            re,
        )

        print(
            f"  Bone{i}: "
            f"{te * 1000.0:.6f} mm, "
            f"{re:.6f} deg"
        )

    print(
        f"  max: "
        f"{max_translation * 1000.0:.6f} mm, "
        f"{max_rotation:.6f} deg"
    )

    world.reset()

    sync_skin(
        cache,
        link_prims,
        skel_root_world,
        translation_attr,
        rotation_attr,
    )

    print()
    print("[WHAT TO LOOK FOR]")
    print()
    print("  Do NOT expect huge bumps.")
    print(
        "  The goal is a subtly less-perfect "
        "tube profile."
    )
    print()
    print("  Inspect especially the internal nodes:")
    print()
    print(
        "       ----====----"
    )
    print(
        "            ^"
    )
    print(
        "       local swelling"
    )
    print()
    print("  GO if:")
    print(
        "    [ ] taper remains smooth"
    )
    print(
        "    [ ] node swellings are subtle/continuous"
    )
    print(
        "    [ ] no visible radius steps"
    )
    print(
        "    [ ] 3D centerline remains correct"
    )
    print(
        "    [ ] no new twist"
    )
    print(
        "    [ ] gravity/skinning behave exactly as 1C"
    )
    print()
    print("No need to press Play.")
    print("=" * 78)

    step = 0
    last_log = 0
    start = time.time()

    while simulation_app.is_running():
        world.step(
            render=False
        )

        world_mats, _ = sync_skin(
            cache,
            link_prims,
            skel_root_world,
            translation_attr,
            rotation_attr,
        )

        simulation_app.update()

        step += 1

        if step - last_log >= 240:
            p = (
                world_mats[-1]
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

    elapsed = (
        time.time() - start
    )

    print()
    print("=" * 78)
    print(
        f"Finished: {step} frames "
        f"in {elapsed:.1f}s"
    )
    print()
    print("TEST 1D = GO if:")
    print(
        "  [ ] organic radius profile visible"
    )
    print(
        "  [ ] no geometry discontinuities"
    )
    print(
        "  [ ] physical behavior unchanged"
    )
    print("=" * 78)

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
