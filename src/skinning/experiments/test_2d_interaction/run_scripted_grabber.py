"""
run_scripted_grabber.py — Test 2D-A

Scripted proof that a skinned visual branch can be manipulated physically
through its invisible rigid/collision representation.

Timeline at 120 physics Hz:
    0..1.0 s    settle while handle stays at initial grab point
    1.0..2.5 s  pull sideways/up
    2.5..4.0 s  sweep in a second direction
    4.0..4.5 s  hold
    4.5 s       RELEASE: disable the external grab joint
    afterwards   free elastic response
"""

import math
import os
import sys

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})


import omni.usd
from isaacsim.core.api import World
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdSkel, Vt


SCRIPT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

if SCRIPT_DIR not in sys.path:
    sys.path.insert(
        0,
        SCRIPT_DIR,
    )


import branch_core_fixed as core
import generate_scripted_grabber as asset


PHYSICS_HZ = 120

SETTLE_END = int(
    1.0 * PHYSICS_HZ
)
PULL_END = int(
    2.5 * PHYSICS_HZ
)
SWEEP_END = int(
    4.0 * PHYSICS_HZ
)
RELEASE_FRAME = int(
    4.5 * PHYSICS_HZ
)


def smoothstep01(x):
    x = max(
        0.0,
        min(
            1.0,
            float(x),
        ),
    )

    return (
        x * x
        * (
            3.0
            - 2.0 * x
        )
    )


def get_world_mats(
    cache,
    prims,
):
    cache.Clear()

    return [
        Gf.Matrix4d(
            cache.GetLocalToWorldTransform(
                prim
            )
        )
        for prim in prims
    ]


def world_to_joint_local(
    world_mats,
    skel_root_world,
):
    result = []

    for index, world in enumerate(
        world_mats
    ):
        if index == 0:
            local = (
                world
                * skel_root_world.GetInverse()
            )
        else:
            local = (
                world
                * world_mats[
                    index - 1
                ].GetInverse()
            )

        result.append(
            local
        )

    return result


def decompose(
    local_mats,
):
    translations = []
    rotations = []

    for matrix in local_mats:
        t = matrix.ExtractTranslation()
        q = matrix.ExtractRotationQuat()
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
        Vt.Vec3fArray(
            translations
        ),
        Vt.QuatfArray(
            rotations
        ),
    )


def make_runtime_branch(
    stage,
    cache,
    branch,
):
    link_prims = [
        stage.GetPrimAtPath(path)
        for path in branch.link_paths
    ]

    skel_root = stage.GetPrimAtPath(
        branch.skel_root_path
    )

    animation = UsdSkel.Animation(
        stage.GetPrimAtPath(
            branch.animation_path
        )
    )

    cache.Clear()

    skel_root_world = Gf.Matrix4d(
        cache.GetLocalToWorldTransform(
            skel_root
        )
    )

    return {
        "branch": branch,
        "link_prims": link_prims,
        "skel_root_world": skel_root_world,
        "translations_attr": (
            animation.GetTranslationsAttr()
        ),
        "rotations_attr": (
            animation.GetRotationsAttr()
        ),
    }


def sync_runtime_branch(
    cache,
    runtime_branch,
):
    world_mats = get_world_mats(
        cache,
        runtime_branch[
            "link_prims"
        ],
    )

    local_mats = world_to_joint_local(
        world_mats,
        runtime_branch[
            "skel_root_world"
        ],
    )

    translations, rotations = decompose(
        local_mats
    )

    runtime_branch[
        "translations_attr"
    ].Set(
        translations
    )

    runtime_branch[
        "rotations_attr"
    ].Set(
        rotations
    )


def target_for_frame(
    frame,
):
    start = Gf.Vec3d(
        asset.GRAB_START_WORLD
    )

    if frame < SETTLE_END:
        return start

    if frame < PULL_END:
        u = smoothstep01(
            (
                frame
                - SETTLE_END
            )
            / float(
                PULL_END
                - SETTLE_END
            )
        )

        return (
            start
            + Gf.Vec3d(
                0.105 * u,
                0.000,
                0.035 * u,
            )
        )

    first_target = (
        start
        + Gf.Vec3d(
            0.105,
            0.000,
            0.035,
        )
    )

    if frame < SWEEP_END:
        u = smoothstep01(
            (
                frame
                - PULL_END
            )
            / float(
                SWEEP_END
                - PULL_END
            )
        )

        # Second direction makes it obvious that the handle is controlling
        # a 3D point, not simply changing one joint angle.
        return (
            first_target
            + Gf.Vec3d(
                -0.025 * u,
                0.080 * u,
                0.020
                * math.sin(
                    math.pi * u
                ),
            )
        )

    return (
        start
        + Gf.Vec3d(
            0.080,
            0.080,
            0.035,
        )
    )


def main():
    print()
    print("=" * 84)
    print(
        "TEST 2D-A — Scripted Physical Grabber"
    )
    print("=" * 84)

    asset.build_stage(
        asset.OUTPUT_USD
    )

    context = omni.usd.get_context()

    context.open_stage(
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

    stage = context.get_stage()

    cache = UsdGeom.XformCache(
        Usd.TimeCode.Default()
    )

    runtimes = [
        make_runtime_branch(
            stage,
            cache,
            branch,
        )
        for branch
        in asset.BRANCHES
    ]

    handle_prim = stage.GetPrimAtPath(
        asset.GRABBER_PATH
    )

    handle_translate = handle_prim.GetAttribute(
        "xformOp:translate"
    )

    if not handle_translate.IsValid():
        raise RuntimeError(
            "Grabber xformOp:translate not found."
        )

    grab_joint = UsdPhysics.Joint(
        stage.GetPrimAtPath(
            asset.GRABBER_JOINT_PATH
        )
    )

    joint_enabled = (
        grab_joint.GetJointEnabledAttr()
    )

    world.reset()

    for runtime in runtimes:
        sync_runtime_branch(
            cache,
            runtime,
        )

    print()
    print("[TIMELINE]")
    print(
        "  0.0 - 1.0 s : settle"
    )
    print(
        "  1.0 - 2.5 s : pull"
    )
    print(
        "  2.5 - 4.0 s : 3D sweep"
    )
    print(
        "  4.0 - 4.5 s : hold"
    )
    print(
        "  4.5 s       : RELEASE"
    )
    print()
    print("[GO CRITERIA]")
    print(
        "  [ ] orange handle moves along scripted path"
    )
    print(
        "  [ ] grabbed lateral link follows physically"
    )
    print(
        "  [ ] neighboring links bend through their D6 joints"
    )
    print(
        "  [ ] visual skin follows the articulation"
    )
    print(
        "  [ ] after release the branch is physically free"
    )
    print(
        "  [ ] no teleport/explosion at grab point"
    )
    print("=" * 84)

    frame = 0
    released = False

    while simulation_app.is_running():
        target = target_for_frame(
            frame
        )

        handle_translate.Set(
            target
        )

        if (
            frame == SETTLE_END
        ):
            print(
                "[GRAB] scripted pull started"
            )

        if (
            frame == PULL_END
        ):
            print(
                "[GRAB] second 3D sweep started"
            )

        if (
            frame >= RELEASE_FRAME
            and not released
        ):
            joint_enabled.Set(
                False
            )

            released = True

            print(
                "[RELEASE] GrabJoint disabled — "
                "branch should now respond freely."
            )

        world.step(
            render=False
        )

        for runtime in runtimes:
            sync_runtime_branch(
                cache,
                runtime,
            )

        simulation_app.update()

        frame += 1

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
