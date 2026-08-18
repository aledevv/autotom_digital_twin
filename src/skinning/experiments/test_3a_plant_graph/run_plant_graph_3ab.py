"""
run_plant_graph_3ab.py — Test 3A-B runtime.

Loads the data-driven PlantGraph USD, keeps all branch skeletons synchronized
with PhysX, and enables the already validated SHIFT + LEFT CLICK interaction
against invisible capsule collision proxies.
"""

import os
import sys

from isaacsim import SimulationApp


simulation_app = SimulationApp({
    "headless": False,
    "width": 1280,
    "height": 720,
})


import carb.settings
import omni.kit.app
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


import generate_plant_graph_3ab as asset


def configure_physx_mouse_interaction():
    """
    Apply settings AFTER opening the USD stage.

    Invisible capsule proxies must remain pickable.
    """
    app = omni.kit.app.get_app()

    extension_manager = (
        app.get_extension_manager()
    )

    extension_manager.set_extension_enabled_immediate(
        "omni.physx.ui",
        True,
    )

    extension_manager.set_extension_enabled_immediate(
        "omni.physx.supportui",
        True,
    )

    simulation_app.update()

    settings = (
        carb.settings.get_settings()
    )

    settings.set(
        "/physics/mouseInteractionEnabled",
        True,
    )

    settings.set(
        "/physics/mouseGrab",
        True,
    )

    settings.set(
        "/physics/mouseGrabIgnoreInvisible",
        False,
    )

    settings.set(
        "/physics/forceGrab",
        False,
    )

    settings.set(
        "/physics/pickingForce",
        10.0,
    )

    print()
    print(
        "[MOUSE INTERACTION]"
    )
    print(
        "  enabled          :",
        settings.get(
            "/physics/mouseInteractionEnabled"
        ),
    )
    print(
        "  grab             :",
        settings.get(
            "/physics/mouseGrab"
        ),
    )
    print(
        "  ignore invisible :",
        settings.get(
            "/physics/mouseGrabIgnoreInvisible"
        ),
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
    local_mats = []

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

        local_mats.append(
            local
        )

    return local_mats


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
        stage.GetPrimAtPath(
            path
        )
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
        "name": branch.name,
        "link_prims": link_prims,
        "skel_root_world": (
            skel_root_world
        ),
        "translations_attr": (
            animation.GetTranslationsAttr()
        ),
        "rotations_attr": (
            animation.GetRotationsAttr()
        ),
    }


def sync_runtime_branch(
    cache,
    runtime,
):
    world_mats = get_world_mats(
        cache,
        runtime[
            "link_prims"
        ],
    )

    local_mats = world_to_joint_local(
        world_mats,
        runtime[
            "skel_root_world"
        ],
    )

    translations, rotations = decompose(
        local_mats
    )

    runtime[
        "translations_attr"
    ].Set(
        translations
    )

    runtime[
        "rotations_attr"
    ].Set(
        rotations
    )


def main():
    print()
    print("=" * 88)
    print(
        "TEST 3A-B — Plant Graph"
    )
    print("=" * 88)

    asset.build_stage(
        asset.OUTPUT_USD
    )

    context = (
        omni.usd.get_context()
    )

    context.open_stage(
        asset.OUTPUT_USD
    )

    # Per-stage settings, therefore authored after open_stage().
    configure_physx_mouse_interaction()

    if World.instance() is not None:
        World.instance().clear_instance()

    world = World(
        stage_units_in_meters=1.0,
        physics_prim_path=(
            "/World/PhysicsScene"
        ),
    )

    stage = (
        context.get_stage()
    )

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

    # Defensive re-apply after reset, same proven solution as 2D-B2.
    configure_physx_mouse_interaction()

    for runtime in runtimes:
        sync_runtime_branch(
            cache,
            runtime,
        )

    simulation_app.update()

    print()
    print("=" * 88)
    print("[GO CRITERIA]")
    print()
    print(
        "  [ ] one coherent plant: main + 3 laterals + 1 secondary"
    )
    print(
        "  [ ] all four junctions stay connected"
    )
    print(
        "  [ ] swelling/flare visible at each junction"
    )
    print(
        "  [ ] no branch behaves as a separate falling object"
    )
    print(
        "  [ ] SHIFT + LEFT CLICK can grab any branch"
    )
    print(
        "  [ ] neighboring branches/main react through the articulation"
    )
    print(
        "  [ ] no ground collision / depenetration artifacts"
    )
    print()
    print(
        "Known baseline behavior: initial settling may be fast."
    )
    print(
        "Do not use that alone as a NO-GO criterion for this graph test."
    )
    print("=" * 88)

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
