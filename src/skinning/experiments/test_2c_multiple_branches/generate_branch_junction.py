"""
generate_branch_junction.py — Test 2C-A

Single bifurcation / branch-junction test.

Topology:
    World
      PlantPhysics (ONE PhysX articulation)
        MainStem
          Link_01 -- D6 -- Link_02 -- ...
                         |
                         +-- Junction D6 --> LateralBranch Link_01 -- ...

Visual:
    MainStem and LateralBranch currently use SEPARATE SkelRoots / meshes.
    They intentionally overlap at the junction.

This test validates topology and attachment mechanics, NOT final junction
surface blending.
"""

import os
from pathlib import Path

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

import branch_core as core


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_USD = str(
    OUTPUT_DIR / "test_2ca_single_junction.usda"
)


# ============================================================================
# 1. MAIN STEM
# ============================================================================

MAIN_SPEC = core.BranchSpec(
    control_points=(
        (0.000, 0.000, 0.055),
        (0.000, 0.080, 0.120),
        (0.010, 0.165, 0.200),
        (0.020, 0.250, 0.270),
        (0.040, 0.335, 0.325),
    ),
    physics_links=5,
    samples_per_control_segment=18,
    radial_segments=14,
    radius=core.RadiusProfile(
        base_radius=0.015,
        tip_radius=0.010,
        taper_start=0.04,
        taper_end=0.96,
        swell_fractions=(0.55,),
        swell_amplitude=0.10,
        micro_variation_amplitude=0.015,
        micro_variation_cycles=1.8,
    ),
    linear_density_kg_per_m=0.28,
    collider_radius_scale=0.90,
    colliders_per_link=2,
    collider_length_scale=0.92,
    joint_stiffness=0.0,
    joint_damping=0.05,
    bend_limit_deg=55.0,
    skin_blend_fraction=0.32,
    show_physics_colliders=True,
)

MAIN = core.make_branch_data(
    "MainStem",
    MAIN_SPEC,
)


# ============================================================================
# 2. JUNCTION LOCATION
# ============================================================================

# Deliberately attach inside a main-stem rigid link, not at a physics node.
# This tests an arbitrary junction position.
JUNCTION_FRACTION = 0.58

JUNCTION_ARC = (
    MAIN.centerline["total_length"]
    * JUNCTION_FRACTION
)

JUNCTION_WORLD = core.point_at_arc(
    MAIN.centerline,
    JUNCTION_ARC,
)

JUNCTION_PARENT_LINK = (
    core.physics_link_for_arc(
        MAIN.physics,
        JUNCTION_ARC,
    )
)

MAIN_RADIUS_AT_JUNCTION = (
    core.radius_for_arc(
        MAIN.spec,
        MAIN.centerline,
        JUNCTION_ARC,
    )
)


# ============================================================================
# 3. LATERAL BRANCH
# ============================================================================

# The branch starts EXACTLY at the parent centerline junction.
j = JUNCTION_WORLD

LATERAL_SPEC = core.BranchSpec(
    control_points=(
        (
            float(j[0]),
            float(j[1]),
            float(j[2]),
        ),
        (
            float(j[0] + 0.065),
            float(j[1] + 0.030),
            float(j[2] + 0.040),
        ),
        (
            float(j[0] + 0.135),
            float(j[1] + 0.065),
            float(j[2] + 0.035),
        ),
        (
            float(j[0] + 0.205),
            float(j[1] + 0.105),
            float(j[2] + 0.010),
        ),
    ),
    physics_links=3,
    samples_per_control_segment=18,
    radial_segments=14,
    radius=core.RadiusProfile(
        base_radius=MAIN_RADIUS_AT_JUNCTION * 0.82,
        tip_radius=MAIN_RADIUS_AT_JUNCTION * 0.48,
        taper_start=0.03,
        taper_end=0.95,
        swell_fractions=(0.10,),
        swell_amplitude=0.08,
        micro_variation_amplitude=0.012,
        micro_variation_cycles=1.6,
    ),
    linear_density_kg_per_m=0.20,
    collider_radius_scale=0.90,
    colliders_per_link=2,
    collider_length_scale=0.92,
    joint_stiffness=0.0,
    joint_damping=0.055,
    bend_limit_deg=55.0,
    skin_blend_fraction=0.32,
    show_physics_colliders=True,
)

LATERAL = core.make_branch_data(
    "LateralBranch",
    LATERAL_SPEC,
)


BRANCHES = {
    "MainStem": MAIN,
    "LateralBranch": LATERAL,
}


# ============================================================================
# 4. STAGE HELPERS
# ============================================================================

def build_ground(stage):
    ground = UsdGeom.Mesh.Define(
        stage,
        "/World/Ground",
    )

    size = 0.8

    ground.CreatePointsAttr().Set([
        Gf.Vec3f(-size, -size, 0.0),
        Gf.Vec3f(size, -size, 0.0),
        Gf.Vec3f(size, size, 0.0),
        Gf.Vec3f(-size, size, 0.0),
    ])

    ground.CreateFaceVertexCountsAttr().Set(
        [4]
    )
    ground.CreateFaceVertexIndicesAttr().Set(
        [0, 1, 2, 3]
    )
    ground.CreateSubdivisionSchemeAttr().Set(
        UsdGeom.Tokens.none
    )
    ground.CreateDisplayColorAttr().Set([
        Gf.Vec3f(
            0.25,
            0.22,
            0.18,
        )
    ])

    UsdPhysics.CollisionAPI.Apply(
        ground.GetPrim()
    )


def validate_junction():
    core.validate_branch_connectivity(
        MAIN
    )
    core.validate_branch_connectivity(
        LATERAL
    )

    child_root = (
        LATERAL
        .physics[
            "origins"
        ][0]
    )

    root_error = float(
        (
            child_root
            - JUNCTION_WORLD
        ).GetLength()
    )

    if root_error > 1e-8:
        raise RuntimeError(
            "Lateral root does not coincide "
            f"with junction: {root_error}"
        )

    return root_error


# ============================================================================
# 5. BUILD
# ============================================================================

def build_stage(
    output_path=OUTPUT_USD,
):
    junction_root_error = (
        validate_junction()
    )

    os.makedirs(
        os.path.dirname(
            output_path
        ),
        exist_ok=True,
    )

    stage = Usd.Stage.CreateNew(
        output_path
    )

    UsdGeom.SetStageUpAxis(
        stage,
        UsdGeom.Tokens.z,
    )

    UsdGeom.SetStageMetersPerUnit(
        stage,
        1.0,
    )

    UsdPhysics.SetStageKilogramsPerUnit(
        stage,
        1.0,
    )

    world = UsdGeom.Xform.Define(
        stage,
        "/World",
    )

    stage.SetDefaultPrim(
        world.GetPrim()
    )

    core.apply_physx_scene(
        stage
    )

    # One common articulation root for the whole branching tree.
    plant_physics = UsdGeom.Xform.Define(
        stage,
        "/World/PlantPhysics",
    )

    UsdGeom.Xform.Define(
        stage,
        "/World/PlantVisual",
    )

    core.build_branch_physics(
        stage,
        MAIN,
    )

    core.build_branch_physics(
        stage,
        LATERAL,
    )

    # Main stem is the only world-anchored root.
    core.create_world_anchor(
        stage,
        MAIN.link_paths[0],
    )

    # The lateral root is connected to an arbitrary point of the main stem.
    junction_info = (
        core.create_junction_joint(
            stage,
            parent_branch=MAIN,
            parent_link_index=JUNCTION_PARENT_LINK,
            child_branch=LATERAL,
            attachment_world=JUNCTION_WORLD,
            stiffness=0.0,
            damping=0.065,
            bend_limit_deg=45.0,
        )
    )

    articulation = (
        PhysxSchema
        .PhysxArticulationAPI
        .Apply(
            plant_physics
            .GetPrim()
        )
    )

    articulation.CreateSolverPositionIterationCountAttr().Set(
        32
    )
    articulation.CreateSolverVelocityIterationCountAttr().Set(
        1
    )
    articulation.CreateEnabledSelfCollisionsAttr().Set(
        False
    )
    articulation.CreateSleepThresholdAttr().Set(
        0.0
    )

    core.build_branch_visual(
        stage,
        MAIN,
        color=(
            0.25,
            0.55,
            0.18,
        ),
    )

    core.build_branch_visual(
        stage,
        LATERAL,
        color=(
            0.42,
            0.68,
            0.20,
        ),
    )

    build_ground(
        stage
    )

    stage.Save()

    parent_local = (
        junction_info[
            "parent_local"
        ]
        .ExtractTranslation()
    )

    child_local = (
        junction_info[
            "child_local"
        ]
        .ExtractTranslation()
    )

    print("=" * 82)
    print("TEST 2C-A — SINGLE BRANCH JUNCTION")
    print("=" * 82)
    print(f"[OK] {output_path}")
    print()
    print("TOPOLOGY:")
    print(
        f"  MainStem       : "
        f"{MAIN.spec.physics_links} links / "
        f"{MAIN.spec.physics_links - 1} internal D6"
    )
    print(
        f"  LateralBranch  : "
        f"{LATERAL.spec.physics_links} links / "
        f"{LATERAL.spec.physics_links - 1} internal D6"
    )
    print("  Junction       : 1 cross-branch D6")
    print(
        f"  TOTAL          : "
        f"{MAIN.spec.physics_links + LATERAL.spec.physics_links} rigid links, "
        f"{(MAIN.spec.physics_links - 1) + (LATERAL.spec.physics_links - 1) + 1} D6"
    )
    print()
    print("JUNCTION:")
    print(
        f"  fraction along main : "
        f"{JUNCTION_FRACTION:.2f}"
    )
    print(
        f"  main parent link    : "
        f"{JUNCTION_PARENT_LINK + 1}"
    )
    print(
        "  world position      : "
        f"({JUNCTION_WORLD[0]:+.4f}, "
        f"{JUNCTION_WORLD[1]:+.4f}, "
        f"{JUNCTION_WORLD[2]:+.4f})"
    )
    print(
        f"  lateral root error  : "
        f"{junction_root_error * 1000.0:.9f} mm"
    )
    print(
        "  parent local pos    : "
        f"({parent_local[0]:+.4f}, "
        f"{parent_local[1]:+.4f}, "
        f"{parent_local[2]:+.4f})"
    )
    print(
        "  child local pos     : "
        f"({child_local[0]:+.6f}, "
        f"{child_local[1]:+.6f}, "
        f"{child_local[2]:+.6f})"
    )
    print()
    print("IMPORTANT:")
    print(
        "  The two visual meshes are intentionally separate in 2C-A."
    )
    print(
        "  Some overlap at the junction is EXPECTED."
    )
    print(
        "  Seamless visual junction blending is deferred to 2C-C."
    )
    print("=" * 82)

    return output_path


if __name__ == "__main__":
    build_stage()
