"""
run_junction_visual.py — Test 2C-C runtime.
"""

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
from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt


SCRIPT_DIR = os.path.dirname(
    os.path.abspath(__file__)
)

if SCRIPT_DIR not in sys.path:
    sys.path.insert(
        0,
        SCRIPT_DIR,
    )


import branch_core_fixed as core
import generate_junction_visual as asset


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

        result.append(local)

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

    animation_prim = stage.GetPrimAtPath(
        branch.animation_path
    )

    animation = UsdSkel.Animation(
        animation_prim
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

    (
        translations,
        rotations,
    ) = decompose(
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


def main():
    print()
    print("=" * 84)
    print(
        "TEST 2C-C — Junction Visual Blend"
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
        for branch in asset.BRANCHES
    ]

    world.reset()

    for runtime in runtimes:
        sync_runtime_branch(
            cache,
            runtime,
        )

    print()
    print("[WHAT TO INSPECT]")
    print(
        "  Look closely at the base of the lateral branch."
    )
    print(
        "  It should emerge through a broader, smoother shoulder."
    )
    print()
    print("  GO if:")
    print(
        "    [ ] junction looks less like two tubes crossing"
    )
    print(
        "    [ ] flare is visible but not bulbous/exaggerated"
    )
    print(
        "    [ ] no visual gap opens while the branch bends"
    )
    print(
        "    [ ] physics behaviour is unchanged"
    )
    print()
    print(
        "  For an A/B comparison close this run and execute:"
    )
    print(
        "    ./run_test_2cc_raw.sh"
    )
    print("=" * 84)

    while simulation_app.is_running():
        world.step(
            render=False
        )

        for runtime in runtimes:
            sync_runtime_branch(
                cache,
                runtime,
            )

        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
