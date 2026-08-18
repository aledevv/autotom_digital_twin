"""
run_manual_mouse_grab_v2.py — Test 2D-B2

Fixes two issues in 2D-B:

1. NO hidden pre-settle.
   The branch settles visibly in real time.

2. PhysX mouse-interaction settings are applied AFTER the target USD stage
   has been opened, because these settings are per-stage.

Interaction:
    SHIFT + LEFT CLICK + drag on the green branch.

The green skin is visual-only.
Picking is performed against the invisible capsule collision proxies.
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


import branch_core_fixed as core
import generate_manual_mouse_grab_v2aligned as asset


def configure_physx_mouse_interaction():
    """
    IMPORTANT:
    Call only AFTER context.open_stage(...).

    These /physics/mouse* settings belong to the currently opened stage.
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

    # Allow the just-enabled viewport UI extension to initialize.
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

    # Critical for our architecture:
    # capsule collision proxies are deliberately invisible.
    settings.set(
        "/physics/mouseGrabIgnoreInvisible",
        False,
    )

    # False = temporary D6 grab at the raycast point.
    settings.set(
        "/physics/forceGrab",
        False,
    )

    # Slightly stronger default picking response.
    settings.set(
        "/physics/pickingForce",
        10.0,
    )

    print()
    print("[MOUSE INTERACTION — CURRENT STAGE]")
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
    print(
        "  force grab       :",
        settings.get(
            "/physics/forceGrab"
        ),
    )
    print(
        "  picking force    :",
        settings.get(
            "/physics/pickingForce"
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
    print("=" * 84)
    print(
        "TEST 2D-B4 — V2-aligned settling + manual grab"
    )
    print("=" * 84)

    asset.build_stage(
        asset.OUTPUT_USD
    )

    context = (
        omni.usd.get_context()
    )

    # ---------------------------------------------------------------
    # OPEN TARGET STAGE FIRST
    # ---------------------------------------------------------------
    context.open_stage(
        asset.OUTPUT_USD
    )

    # ---------------------------------------------------------------
    # THEN configure per-stage mouse interaction.
    # This ordering is the main Shift+click fix.
    # ---------------------------------------------------------------
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
        for branch
        in asset.BRANCHES
    ]

    world.reset()

    # Re-author after reset as a defensive measure.
    # reset() should not change the stage, but this also prints the values
    # that are actually active immediately before interaction.
    configure_physx_mouse_interaction()

    for runtime_branch in runtimes:
        sync_runtime_branch(
            cache,
            runtime_branch,
        )

    simulation_app.update()

    print()
    print("=" * 84)
    print("[READY]")
    print()
    print(
        "The branch will now settle VISIBLY."
    )
    print(
        "No hidden physics pre-settle is performed."
    )
    print()
    print(
        "Interaction:"
    )
    print(
        "  hold SHIFT"
    )
    print(
        "  + LEFT CLICK on the GREEN branch"
    )
    print(
        "  + drag"
    )
    print()
    print(
        "The green mesh is visual-only;"
    )
    print(
        "the raycast should hit the invisible capsule underneath."
    )
    print()
    print(
        "No gravity ramp is used."
    )
    print(
        "The main stem is rigid, matching V2's default RIGID_TRUNK behavior."
    )
    print("=" * 84)

    while simulation_app.is_running():
        # No hidden settling:
        # every physics pose is presented to the user.
        world.step(
            render=False
        )

        for runtime_branch in runtimes:
            sync_runtime_branch(
                cache,
                runtime_branch,
            )

        simulation_app.update()

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
