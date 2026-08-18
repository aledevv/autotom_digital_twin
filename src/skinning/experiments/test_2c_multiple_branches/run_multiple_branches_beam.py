"""
run_multiple_branches.py — Test 2C-B runtime

Drives four independent UsdSkel skeletons from one branching PhysX
articulation:
    MainStem + 3 lateral branches.
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


import branch_core_fixed as core
import generate_multiple_branches_beam as asset


def get_world_mats(
    cache,
    prims,
):
    cache.Clear()

    return [
        Gf.Matrix4d(
            cache
            .GetLocalToWorldTransform(
                prim
            )
        )
        for prim
        in prims
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
                * skel_root_world
                .GetInverse()
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

    for matrix in (
        local_mats
    ):
        translation = (
            matrix
            .ExtractTranslation()
        )

        quaternion = (
            matrix
            .ExtractRotationQuat()
        )

        imaginary = (
            quaternion
            .GetImaginary()
        )

        translations.append(
            Gf.Vec3f(
                float(
                    translation[0]
                ),
                float(
                    translation[1]
                ),
                float(
                    translation[2]
                ),
            )
        )

        rotations.append(
            Gf.Quatf(
                float(
                    quaternion.GetReal()
                ),
                Gf.Vec3f(
                    float(
                        imaginary[0]
                    ),
                    float(
                        imaginary[1]
                    ),
                    float(
                        imaginary[2]
                    ),
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


def translation_error(
    a,
    b,
):
    return float(
        (
            a.ExtractTranslation()
            - b.ExtractTranslation()
        ).GetLength()
    )


def rotation_error_deg(
    a,
    b,
):
    relative = (
        a
        * b.GetInverse()
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


def make_runtime_branch(
    stage,
    cache,
    branch,
):
    link_prims = [
        stage.GetPrimAtPath(path)
        for path
        in branch.link_paths
    ]

    for path, prim in zip(
        branch.link_paths,
        link_prims,
    ):
        if not prim.IsValid():
            raise RuntimeError(
                f"{branch.name}: "
                f"missing link {path}"
            )

    skel_root = (
        stage.GetPrimAtPath(
            branch.skel_root_path
        )
    )

    animation_prim = (
        stage.GetPrimAtPath(
            branch.animation_path
        )
    )

    if not skel_root.IsValid():
        raise RuntimeError(
            f"{branch.name}: "
            "missing SkelRoot"
        )

    if not animation_prim.IsValid():
        raise RuntimeError(
            f"{branch.name}: "
            "missing SkelAnimation"
        )

    animation = (
        UsdSkel.Animation(
            animation_prim
        )
    )

    cache.Clear()

    skel_root_world = (
        Gf.Matrix4d(
            cache
            .GetLocalToWorldTransform(
                skel_root
            )
        )
    )

    return {
        "branch": branch,
        "link_prims": (
            link_prims
        ),
        "skel_root_world": (
            skel_root_world
        ),
        "translations_attr": (
            animation
            .GetTranslationsAttr()
        ),
        "rotations_attr": (
            animation
            .GetRotationsAttr()
        ),
    }


def sync_runtime_branch(
    cache,
    runtime_branch,
):
    world_mats = (
        get_world_mats(
            cache,
            runtime_branch[
                "link_prims"
            ],
        )
    )

    local_mats = (
        world_to_joint_local(
            world_mats,
            runtime_branch[
                "skel_root_world"
            ],
        )
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

    return (
        world_mats,
        local_mats,
    )


def check_rest_frames(
    cache,
    runtime_branch,
):
    branch = (
        runtime_branch[
            "branch"
        ]
    )

    actual = (
        world_to_joint_local(
            get_world_mats(
                cache,
                runtime_branch[
                    "link_prims"
                ],
            ),
            runtime_branch[
                "skel_root_world"
            ],
        )
    )

    expected = (
        core.rest_local_transforms(
            branch.physics
        )
    )

    max_translation = 0.0
    max_rotation = 0.0

    for actual_m, expected_m in zip(
        actual,
        expected,
    ):
        max_translation = max(
            max_translation,
            translation_error(
                actual_m,
                expected_m,
            ),
        )

        max_rotation = max(
            max_rotation,
            rotation_error_deg(
                actual_m,
                expected_m,
            ),
        )

    status = (
        "OK"
        if (
            max_translation
            < 1e-5
            and max_rotation
            < 1e-4
        )
        else "WARNING"
    )

    print(
        f"[REST CHECK — "
        f"{branch.name:<10}] "
        f"maxT="
        f"{max_translation * 1000.0:.6f} mm  "
        f"maxR="
        f"{max_rotation:.6f} deg  "
        f"[{status}]"
    )


def branch_tip(
    branch,
    world_mats,
):
    return (
        world_mats[-1]
        .Transform(
            Gf.Vec3d(
                0.0,
                0.0,
                float(
                    branch
                    .physics[
                        "lengths"
                    ][-1]
                ),
            )
        )
    )


def main():
    print()
    print("=" * 84)
    print(
        "TEST 2C-B2 — Multiple Branches / Beam Physics"
    )
    print("=" * 84)

    asset.build_stage(
        asset.OUTPUT_USD
    )

    context = (
        omni.usd.get_context()
    )

    context.open_stage(
        asset.OUTPUT_USD
    )

    if (
        World.instance()
        is not None
    ):
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

    cache = (
        UsdGeom.XformCache(
            Usd.TimeCode.Default()
        )
    )

    runtime_branches = [
        make_runtime_branch(
            stage,
            cache,
            branch,
        )
        for branch
        in asset.BRANCHES
    ]

    print()

    for runtime_branch in (
        runtime_branches
    ):
        check_rest_frames(
            cache,
            runtime_branch,
        )

    world.reset()

    for runtime_branch in (
        runtime_branches
    ):
        sync_runtime_branch(
            cache,
            runtime_branch,
        )

    print()
    print("=" * 84)
    print("[WHAT 2C-B MUST PROVE]")
    print()
    print(
        "  Main stem carries THREE independent lateral branches."
    )
    print(
        "  There is no physical ground in this test."
    )
    print()
    print("  GO if:")
    print(
        "    [ ] main stem remains stable"
    )
    print(
        "    [ ] all 3 lateral roots stay attached"
    )
    print(
        "    [ ] each lateral follows parent motion"
    )
    print(
        "    [ ] laterals bend independently"
    )
    print(
        "    [ ] no junction explodes or separates"
    )
    print(
        "    [ ] no branch is repelled by the visual ground"
    )
    print(
        "    [ ] all 4 skins follow their physics"
    )
    print()
    print("No need to press Play.")
    print("=" * 84)

    step = 0
    last_log = 0
    start = time.time()

    while (
        simulation_app.is_running()
    ):
        world.step(
            render=False
        )

        world_by_name = {}

        for runtime_branch in (
            runtime_branches
        ):
            world_mats, _ = (
                sync_runtime_branch(
                    cache,
                    runtime_branch,
                )
            )

            branch = (
                runtime_branch[
                    "branch"
                ]
            )

            world_by_name[
                branch.name
            ] = world_mats

        simulation_app.update()

        step += 1

        if (
            step - last_log
            >= 240
        ):
            print(
                f"[frame {step:6d}]"
            )

            for branch in (
                asset.BRANCHES
            ):
                tip = branch_tip(
                    branch,
                    world_by_name[
                        branch.name
                    ],
                )

                root = (
                    world_by_name[
                        branch.name
                    ][0]
                    .ExtractTranslation()
                )

                print(
                    f"  {branch.name:<10} "
                    f"root="
                    f"({root[0]:+.3f},"
                    f"{root[1]:+.3f},"
                    f"{root[2]:+.3f}) "
                    f"tip="
                    f"({tip[0]:+.3f},"
                    f"{tip[1]:+.3f},"
                    f"{tip[2]:+.3f})"
                )

            last_log = step

    elapsed = (
        time.time()
        - start
    )

    print()
    print("=" * 84)
    print(
        f"Finished: "
        f"{step} frames "
        f"in {elapsed:.1f}s"
    )
    print("=" * 84)

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
