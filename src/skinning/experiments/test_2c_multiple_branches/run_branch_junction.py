"""
run_branch_junction.py — Test 2C-A runtime

Synchronizes TWO independent UsdSkel branches driven by ONE branching PhysX
articulation tree.

The main-stem and lateral-branch skeletons are intentionally separate.
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


import branch_core as core
import generate_branch_junction as asset


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

        result.append(local)

    return result


def decompose(
    local_mats,
):
    translations = []
    rotations = []

    for matrix in local_mats:
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
                float(translation[0]),
                float(translation[1]),
                float(translation[2]),
            )
        )

        rotations.append(
            Gf.Quatf(
                float(
                    quaternion
                    .GetReal()
                ),
                Gf.Vec3f(
                    float(imaginary[0]),
                    float(imaginary[1]),
                    float(imaginary[2]),
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
        for path in branch.link_paths
    ]

    for path, prim in zip(
        branch.link_paths,
        link_prims,
    ):
        if not prim.IsValid():
            raise RuntimeError(
                f"{branch.name}: "
                f"missing rigid link {path}"
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
        "link_prims": link_prims,
        "skel_root_world": skel_root_world,
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
    world_mats = get_world_mats(
        cache,
        runtime_branch[
            "link_prims"
        ],
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

    print(
        f"[REST CHECK — {branch.name}]"
    )

    for index, (
        actual_matrix,
        expected_matrix,
    ) in enumerate(
        zip(
            actual,
            expected,
        )
    ):
        translation_err = (
            translation_error(
                actual_matrix,
                expected_matrix,
            )
        )

        rotation_err = (
            rotation_error_deg(
                actual_matrix,
                expected_matrix,
            )
        )

        max_translation = max(
            max_translation,
            translation_err,
        )

        max_rotation = max(
            max_rotation,
            rotation_err,
        )

        print(
            f"  Bone{index:02d}: "
            f"{translation_err * 1000.0:.6f} mm, "
            f"{rotation_err:.6f} deg"
        )

    print(
        f"  max: "
        f"{max_translation * 1000.0:.6f} mm, "
        f"{max_rotation:.6f} deg"
    )

    if (
        max_translation < 1e-5
        and max_rotation < 1e-4
    ):
        print("  [OK]")
    else:
        print("  [WARNING]")

    print()


def main():
    print()
    print("=" * 82)
    print(
        "TEST 2C-A — Single Branch Junction"
    )
    print("=" * 82)

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

    main_runtime = (
        make_runtime_branch(
            stage,
            cache,
            asset.MAIN,
        )
    )

    lateral_runtime = (
        make_runtime_branch(
            stage,
            cache,
            asset.LATERAL,
        )
    )

    print()
    check_rest_frames(
        cache,
        main_runtime,
    )
    check_rest_frames(
        cache,
        lateral_runtime,
    )

    world.reset()

    sync_runtime_branch(
        cache,
        main_runtime,
    )

    sync_runtime_branch(
        cache,
        lateral_runtime,
    )

    print("=" * 82)
    print("[WHAT 2C-A MUST PROVE]")
    print()
    print(
        "  One PhysX articulation contains a branching topology:"
    )
    print()
    print(
        "    MainStem"
    )
    print(
        "       |"
    )
    print(
        "       +---- D6 junction ---- LateralBranch"
    )
    print()
    print("  GO if:")
    print(
        "    [ ] main stem remains stable"
    )
    print(
        "    [ ] lateral root stays attached to the main stem"
    )
    print(
        "    [ ] lateral branch follows parent motion"
    )
    print(
        "    [ ] lateral branch can bend through its own D6 joints"
    )
    print(
        "    [ ] no explosion / separation at junction"
    )
    print(
        "    [ ] both independent SkelRoots follow their PhysX links"
    )
    print()
    print(
        "  EXPECTED:"
    )
    print(
        "    visual meshes overlap/intersect around junction."
    )
    print(
        "    We are NOT solving the seamless junction surface yet."
    )
    print()
    print("No need to press Play.")
    print("=" * 82)

    step = 0
    last_log = 0
    start = time.time()

    while (
        simulation_app.is_running()
    ):
        world.step(
            render=False
        )

        (
            main_world,
            _,
        ) = sync_runtime_branch(
            cache,
            main_runtime,
        )

        (
            lateral_world,
            _,
        ) = sync_runtime_branch(
            cache,
            lateral_runtime,
        )

        simulation_app.update()

        step += 1

        if (
            step - last_log
            >= 240
        ):
            main_tip = (
                main_world[-1]
                .Transform(
                    Gf.Vec3d(
                        0.0,
                        0.0,
                        float(
                            asset.MAIN
                            .physics[
                                "lengths"
                            ][-1]
                        ),
                    )
                )
            )

            lateral_root = (
                lateral_world[0]
                .ExtractTranslation()
            )

            lateral_tip = (
                lateral_world[-1]
                .Transform(
                    Gf.Vec3d(
                        0.0,
                        0.0,
                        float(
                            asset.LATERAL
                            .physics[
                                "lengths"
                            ][-1]
                        ),
                    )
                )
            )

            print(
                f"[frame {step:6d}] "
                f"mainTip="
                f"({main_tip[0]:+.3f},"
                f"{main_tip[1]:+.3f},"
                f"{main_tip[2]:+.3f})  "
                f"lateralRoot="
                f"({lateral_root[0]:+.3f},"
                f"{lateral_root[1]:+.3f},"
                f"{lateral_root[2]:+.3f})  "
                f"lateralTip="
                f"({lateral_tip[0]:+.3f},"
                f"{lateral_tip[1]:+.3f},"
                f"{lateral_tip[2]:+.3f})"
            )

            last_log = step

    elapsed = (
        time.time() - start
    )

    print()
    print("=" * 82)
    print(
        f"Finished: "
        f"{step} frames "
        f"in {elapsed:.1f}s"
    )
    print("=" * 82)

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    finally:
        if simulation_app.is_running():
            simulation_app.close()
