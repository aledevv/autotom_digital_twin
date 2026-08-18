"""
generate_junction_visual.py — Test 2C-C

Isolated suspended branch junction.

Physics is intentionally kept independent from the visual blend:
    - same compound capsules
    - same D6 topology
    - beam-based stiffness/damping
    - no ground collision

Visual-only additions:
    1. local node swelling on MainStem around the attachment
    2. smooth root flare on LateralBranch
"""

import math
import os
from pathlib import Path

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

import branch_core_fixed as core
import junction_visual as jvis


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"

BLEND_ENABLED = (
    os.environ.get(
        "JUNCTION_BLEND",
        "1",
    )
    != "0"
)

OUTPUT_USD = str(
    OUTPUT_DIR
    / (
        "test_2cc_junction_blended.usda"
        if BLEND_ENABLED
        else "test_2cc_junction_raw.usda"
    )
)


# ============================================================================
# PHYSICS SETTINGS
# ============================================================================

ROOT_HEIGHT = 0.68

YOUNG_MODULUS_PA = 70.0e6
DAMPING_RATIO = 0.8
BEND_LIMIT_DEG = 30.0


def beam_drive_params(
    radius,
    link_length,
    linear_density,
):
    mass = (
        linear_density
        * link_length
    )

    second_moment = (
        math.pi
        * radius**4
        / 4.0
    )

    k_rad = (
        YOUNG_MODULUS_PA
        * second_moment
        / link_length
    )

    j_center = (
        mass
        * (
            3.0 * radius**2
            + link_length**2
        )
        / 12.0
    )

    j_pivot = (
        j_center
        + mass
        * (
            link_length / 2.0
        )**2
    )

    d_rad = (
        2.0
        * DAMPING_RATIO
        * math.sqrt(
            k_rad * j_pivot
        )
    )

    rad_to_deg = (
        math.pi / 180.0
    )

    return (
        k_rad * rad_to_deg,
        d_rad * rad_to_deg,
    )


# ============================================================================
# MAIN STEM
# ============================================================================

MAIN_TOTAL_APPROX = 0.38
MAIN_LINKS = 5

MAIN_K, MAIN_D = beam_drive_params(
    radius=0.014,
    link_length=(
        MAIN_TOTAL_APPROX
        / MAIN_LINKS
    ),
    linear_density=0.27,
)

MAIN_SPEC = core.BranchSpec(
    control_points=(
        (0.000, 0.000, ROOT_HEIGHT),
        (0.004, 0.080, ROOT_HEIGHT + 0.060),
        (0.010, 0.160, ROOT_HEIGHT + 0.125),
        (0.018, 0.245, ROOT_HEIGHT + 0.180),
        (0.030, 0.330, ROOT_HEIGHT + 0.220),
    ),
    physics_links=MAIN_LINKS,
    samples_per_control_segment=22,
    radial_segments=18,
    radius=core.RadiusProfile(
        base_radius=0.015,
        tip_radius=0.0105,
        taper_start=0.04,
        taper_end=0.96,
        swell_fractions=(),
        swell_amplitude=0.0,
        micro_variation_amplitude=0.010,
        micro_variation_cycles=1.6,
    ),
    linear_density_kg_per_m=0.27,
    collider_radius_scale=0.90,
    colliders_per_link=2,
    collider_length_scale=0.92,
    joint_stiffness=MAIN_K,
    joint_damping=MAIN_D,
    bend_limit_deg=BEND_LIMIT_DEG,
    skin_blend_fraction=0.32,
    show_physics_colliders=False,
)

MAIN = core.make_branch_data(
    "MainStem",
    MAIN_SPEC,
)


# ============================================================================
# JUNCTION
# ============================================================================

JUNCTION_FRACTION = 0.56

JUNCTION_ARC = (
    MAIN.centerline[
        "total_length"
    ]
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

PARENT_RADIUS = core.radius_for_arc(
    MAIN.spec,
    MAIN.centerline,
    JUNCTION_ARC,
)


# ============================================================================
# LATERAL
# ============================================================================

j = JUNCTION_WORLD

# Fairly clear side branch so the shoulder is easy to inspect.
direction = Gf.Vec3d(
    0.78,
    0.20,
    0.59,
).GetNormalized()

side = Gf.Vec3d(
    -0.20,
    0.78,
    0.0,
).GetNormalized()

LATERAL_LENGTH = 0.245
LATERAL_LINKS = 3

p0 = Gf.Vec3d(j)
p1 = (
    p0
    + direction * 0.078
    + side * 0.006
)
p2 = (
    p0
    + direction * 0.160
    - side * 0.004
    + Gf.Vec3d(
        0.0,
        0.0,
        -0.008,
    )
)
p3 = (
    p0
    + direction * LATERAL_LENGTH
    + Gf.Vec3d(
        0.0,
        0.0,
        -0.018,
    )
)

LATERAL_BASE_RADIUS = (
    PARENT_RADIUS
    * 0.72
)

LATERAL_TIP_RADIUS = (
    PARENT_RADIUS
    * 0.42
)

LATERAL_K, LATERAL_D = (
    beam_drive_params(
        radius=(
            0.65
            * LATERAL_BASE_RADIUS
            + 0.35
            * LATERAL_TIP_RADIUS
        ),
        link_length=(
            LATERAL_LENGTH
            / LATERAL_LINKS
        ),
        linear_density=0.19,
    )
)

LATERAL_SPEC = core.BranchSpec(
    control_points=(
        tuple(p0),
        tuple(p1),
        tuple(p2),
        tuple(p3),
    ),
    physics_links=LATERAL_LINKS,
    samples_per_control_segment=24,
    radial_segments=18,
    radius=core.RadiusProfile(
        base_radius=LATERAL_BASE_RADIUS,
        tip_radius=LATERAL_TIP_RADIUS,
        taper_start=0.03,
        taper_end=0.95,
        swell_fractions=(),
        swell_amplitude=0.0,
        micro_variation_amplitude=0.008,
        micro_variation_cycles=1.4,
    ),
    linear_density_kg_per_m=0.19,
    collider_radius_scale=0.90,
    colliders_per_link=2,
    collider_length_scale=0.92,
    joint_stiffness=LATERAL_K,
    joint_damping=LATERAL_D,
    bend_limit_deg=BEND_LIMIT_DEG,
    skin_blend_fraction=0.32,
    show_physics_colliders=False,
)

LATERAL = core.make_branch_data(
    "LateralBranch",
    LATERAL_SPEC,
)

BRANCHES = [
    MAIN,
    LATERAL,
]


# ============================================================================
# VISUAL BLEND SETTINGS
# ============================================================================

PARENT_BULGES = ()

CHILD_FLARE = None

if BLEND_ENABLED:
    # Local parent node swelling centered on the attachment.
    PARENT_BULGES = (
        jvis.VisualBulge(
            center_fraction=JUNCTION_FRACTION,
            amplitude=0.20,
            sigma_fraction=0.045,
        ),
    )

    # Only affects the first ~5 cm of the LateralBranch visual mesh.
    CHILD_FLARE = jvis.RootFlare(
        parent_radius=PARENT_RADIUS,
        flare_length=max(
            0.050,
            PARENT_RADIUS * 3.2,
        ),
        root_parent_fraction=0.95,
        shoulder_amplitude=0.20,
        shoulder_center_fraction=0.38,
        shoulder_sigma_fraction=0.20,
    )


# ============================================================================
# VISUAL-ONLY GROUND
# ============================================================================

def build_visual_ground(
    stage,
):
    ground = UsdGeom.Mesh.Define(
        stage,
        "/World/VisualGround",
    )

    size = 0.75

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
            0.18,
            0.16,
            0.14,
        )
    ])

    # INTENTIONALLY no CollisionAPI.


# ============================================================================
# BUILD
# ============================================================================

def build_stage(
    output_path=OUTPUT_USD,
):
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

    plant_physics = (
        UsdGeom.Xform.Define(
            stage,
            "/World/PlantPhysics",
        )
    )

    UsdPhysics.ArticulationRootAPI.Apply(
        plant_physics.GetPrim()
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

    core.create_world_anchor(
        stage,
        MAIN.link_paths[0],
    )

    junction_info = (
        core.create_junction_joint(
            stage,
            parent_branch=MAIN,
            parent_link_index=JUNCTION_PARENT_LINK,
            child_branch=LATERAL,
            attachment_world=JUNCTION_WORLD,
            stiffness=None,
            damping=None,
            bend_limit_deg=BEND_LIMIT_DEG,
        )
    )

    articulation = (
        PhysxSchema
        .PhysxArticulationAPI
        .Apply(
            plant_physics.GetPrim()
        )
    )

    articulation.CreateSolverPositionIterationCountAttr().Set(
        32
    )

    articulation.CreateSolverVelocityIterationCountAttr().Set(
        4
    )

    articulation.CreateEnabledSelfCollisionsAttr().Set(
        False
    )

    articulation.CreateSleepThresholdAttr().Set(
        0.0
    )

    color = (
        0.30,
        0.58,
        0.20,
    )

    jvis.build_branch_visual(
        stage,
        MAIN,
        color=color,
        bulges=PARENT_BULGES,
        root_flare=None,
    )

    jvis.build_branch_visual(
        stage,
        LATERAL,
        color=color,
        bulges=(),
        root_flare=CHILD_FLARE,
    )

    build_visual_ground(
        stage
    )

    stage.Save()

    raw_child_root_radius = (
        core.radius_for_arc(
            LATERAL.spec,
            LATERAL.centerline,
            0.0,
        )
    )

    visual_child_root_radius = (
        jvis.visual_radius_for_arc(
            LATERAL,
            0.0,
            root_flare=CHILD_FLARE,
        )
    )

    raw_parent_radius = (
        core.radius_for_arc(
            MAIN.spec,
            MAIN.centerline,
            JUNCTION_ARC,
        )
    )

    visual_parent_radius = (
        jvis.visual_radius_for_arc(
            MAIN,
            JUNCTION_ARC,
            bulges=PARENT_BULGES,
        )
    )

    print("=" * 84)
    print(
        "TEST 2C-C — JUNCTION VISUAL BLEND"
    )
    print("=" * 84)
    print(f"[OK] {output_path}")
    print()
    print(
        f"  blend enabled       : "
        f"{BLEND_ENABLED}"
    )
    print(
        f"  ground collision    : OFF"
    )
    print(
        f"  physics colliders   : hidden"
    )
    print()
    print("VISUAL-ONLY CHANGE:")
    print(
        f"  parent physical r   : "
        f"{raw_parent_radius * 1000.0:.2f} mm"
    )
    print(
        f"  parent visual r     : "
        f"{visual_parent_radius * 1000.0:.2f} mm"
    )
    print(
        f"  child physical r0   : "
        f"{raw_child_root_radius * 1000.0:.2f} mm"
    )
    print(
        f"  child visual r0     : "
        f"{visual_child_root_radius * 1000.0:.2f} mm"
    )

    if CHILD_FLARE is not None:
        print(
            f"  child flare length  : "
            f"{CHILD_FLARE.flare_length * 1000.0:.1f} mm"
        )

    print()
    print("PHYSICS:")
    print(
        f"  Main K/D            : "
        f"{MAIN_K:.6f} / "
        f"{MAIN_D:.6f}"
    )
    print(
        f"  Lateral K/D         : "
        f"{LATERAL_K:.6f} / "
        f"{LATERAL_D:.6f}"
    )
    print(
        f"  bend limit          : "
        f"+/- {BEND_LIMIT_DEG:.1f} deg"
    )
    print()
    print(
        "IMPORTANT: visual flare does NOT modify collision radii."
    )
    print(
        "The two branches still use separate SkelRoots."
    )
    print(
        "The goal is a visually softer overlap, not a true boolean-union mesh."
    )
    print("=" * 84)

    return output_path


if __name__ == "__main__":
    build_stage()
