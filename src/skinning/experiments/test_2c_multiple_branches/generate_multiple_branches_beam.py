"""
generate_multiple_branches.py — Test 2C-B

Multiple lateral branches attached to one suspended main stem.

Validated foundations kept from Test 2C-A FIXED:
    - one UsdPhysics articulation tree
    - exporter-style child-oriented attachment D6 frames
    - collision filtering at parent/child junctions
    - compound capsule collision proxies
    - separate UsdSkel skeleton per branch
    - PhysX -> SkelAnimation runtime bridge

NEW in 2C-B:
    - three lateral branches
    - three independent attachment points
    - different 3D branch directions
    - one articulation with multiple children

Ground is VISUAL ONLY. No ground collision is authored in this test.
"""

import math
import os
from pathlib import Path

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

import branch_core_fixed as core

# ============================================================================
# BEAM-BASED DRIVE PARAMETERS
# ============================================================================

YOUNG_MODULUS_PA = 70.0e6
DAMPING_RATIO = 0.8
BEND_LIMIT_DEG = 30.0


def beam_drive_params(
    radius,
    link_length,
    linear_density,
):
    """
    Same mechanical model used by exporterV2/core/tree_config.py.

        I = pi r^4 / 4
        K = E I / L
        D = 2 zeta sqrt(K J)

    Isaac rotational drives use values per degree, so both K and D
    are multiplied by pi / 180.
    """
    radius = float(radius)
    link_length = float(link_length)
    linear_density = float(linear_density)

    if radius <= 0.0:
        raise ValueError("radius must be positive")
    if link_length <= 0.0:
        raise ValueError("link_length must be positive")
    if linear_density <= 0.0:
        raise ValueError("linear_density must be positive")

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

    # Solid cylinder, transverse rotation around one end.
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


def representative_branch_drive(
    base_radius,
    tip_radius,
    total_length,
    physics_links,
    linear_density,
):
    # Use a representative radius slightly biased toward the thicker base.
    radius = (
        0.65 * base_radius
        + 0.35 * tip_radius
    )

    link_length = (
        total_length
        / physics_links
    )

    return beam_drive_params(
        radius,
        link_length,
        linear_density,
    )



SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"
OUTPUT_USD = str(
    OUTPUT_DIR / "test_2cb2_multiple_branches_beam_physics.usda"
)


# ============================================================================
# CONFIG
# ============================================================================

ROOT_HEIGHT = 0.55


MAIN_K, MAIN_D = representative_branch_drive(
    base_radius=0.016,
    tip_radius=0.010,
    total_length=0.53,
    physics_links=6,
    linear_density=0.28,
)

# Main stem is intentionally suspended well above the visual ground.
MAIN_SPEC = core.BranchSpec(
    control_points=(
        (0.000, 0.000, ROOT_HEIGHT),
        (0.005, 0.085, ROOT_HEIGHT + 0.070),
        (0.012, 0.175, ROOT_HEIGHT + 0.145),
        (0.022, 0.265, ROOT_HEIGHT + 0.205),
        (0.035, 0.355, ROOT_HEIGHT + 0.250),
        (0.050, 0.440, ROOT_HEIGHT + 0.280),
    ),
    physics_links=6,
    samples_per_control_segment=18,
    radial_segments=14,
    radius=core.RadiusProfile(
        base_radius=0.016,
        tip_radius=0.010,
        taper_start=0.04,
        taper_end=0.96,
        swell_fractions=(0.30, 0.58, 0.80),
        swell_amplitude=0.08,
        swell_sigma_fraction=0.035,
        micro_variation_amplitude=0.012,
        micro_variation_cycles=2.0,
    ),
    linear_density_kg_per_m=0.28,
    collider_radius_scale=0.90,
    colliders_per_link=2,
    collider_length_scale=0.92,
    joint_stiffness=MAIN_K,
    joint_damping=MAIN_D,
    bend_limit_deg=BEND_LIMIT_DEG,
    skin_blend_fraction=0.32,
    show_physics_colliders=True,
)

MAIN = core.make_branch_data(
    "MainStem",
    MAIN_SPEC,
)


# ============================================================================
# LATERAL BRANCH FACTORY
# ============================================================================

def make_lateral(
    name,
    junction_fraction,
    azimuth_deg,
    elevation_deg,
    length=0.24,
    links=3,
):
    """
    Build one lateral branch from an arbitrary arc position of MAIN.

    Direction convention:
        azimuth   = rotation around world Z
        elevation = angle above horizontal

    The child starts exactly at the main-stem centerline junction.
    Collision filtering handles the intentional geometric overlap there.
    """
    junction_arc = (
        MAIN.centerline["total_length"]
        * junction_fraction
    )

    junction_world = core.point_at_arc(
        MAIN.centerline,
        junction_arc,
    )

    parent_link_index = (
        core.physics_link_for_arc(
            MAIN.physics,
            junction_arc,
        )
    )

    main_radius = core.radius_for_arc(
        MAIN.spec,
        MAIN.centerline,
        junction_arc,
    )

    az = math.radians(
        azimuth_deg
    )
    el = math.radians(
        elevation_deg
    )

    direction = Gf.Vec3d(
        math.cos(el) * math.cos(az),
        math.cos(el) * math.sin(az),
        math.sin(el),
    ).GetNormalized()

    # Add a small second direction perturbation so the branch is not a
    # perfectly straight line. This remains deterministic.
    side = Gf.Vec3d(
        -math.sin(az),
        math.cos(az),
        0.0,
    )

    p0 = Gf.Vec3d(
        junction_world
    )

    p1 = (
        p0
        + direction * (length * 0.34)
        + side * 0.008
    )

    p2 = (
        p0
        + direction * (length * 0.68)
        - side * 0.006
        + Gf.Vec3d(0.0, 0.0, -0.006)
    )

    p3 = (
        p0
        + direction * length
        + Gf.Vec3d(0.0, 0.0, -0.016)
    )

    lateral_k, lateral_d = representative_branch_drive(
        base_radius=main_radius * 0.78,
        tip_radius=main_radius * 0.44,
        total_length=length,
        physics_links=links,
        linear_density=0.19,
    )

    spec = core.BranchSpec(
        control_points=(
            tuple(p0),
            tuple(p1),
            tuple(p2),
            tuple(p3),
        ),
        physics_links=links,
        samples_per_control_segment=18,
        radial_segments=14,
        radius=core.RadiusProfile(
            base_radius=main_radius * 0.78,
            tip_radius=main_radius * 0.44,
            taper_start=0.03,
            taper_end=0.95,
            swell_fractions=(0.10,),
            swell_amplitude=0.06,
            swell_sigma_fraction=0.035,
            micro_variation_amplitude=0.010,
            micro_variation_cycles=1.5,
        ),
        linear_density_kg_per_m=0.19,
        collider_radius_scale=0.90,
        colliders_per_link=2,
        collider_length_scale=0.92,
        joint_stiffness=lateral_k,
        joint_damping=lateral_d,
        bend_limit_deg=BEND_LIMIT_DEG,
        skin_blend_fraction=0.32,
        show_physics_colliders=True,
    )

    branch = core.make_branch_data(
        name,
        spec,
    )

    return {
        "branch": branch,
        "junction_fraction": junction_fraction,
        "junction_arc": junction_arc,
        "junction_world": junction_world,
        "parent_link_index": parent_link_index,
        "azimuth_deg": azimuth_deg,
        "elevation_deg": elevation_deg,
    }


# Three distinct junctions and three distinct spatial directions.
LATERAL_RECORDS = [
    make_lateral(
        "Lateral_01",
        junction_fraction=0.30,
        azimuth_deg=10.0,
        elevation_deg=24.0,
        length=0.235,
        links=3,
    ),
    make_lateral(
        "Lateral_02",
        junction_fraction=0.56,
        azimuth_deg=145.0,
        elevation_deg=18.0,
        length=0.250,
        links=3,
    ),
    make_lateral(
        "Lateral_03",
        junction_fraction=0.79,
        azimuth_deg=265.0,
        elevation_deg=30.0,
        length=0.220,
        links=3,
    ),
]

LATERALS = [
    record["branch"]
    for record in LATERAL_RECORDS
]

BRANCHES = [
    MAIN,
    *LATERALS,
]


# ============================================================================
# VALIDATION
# ============================================================================

def validate_topology():
    core.validate_branch_connectivity(
        MAIN
    )

    for record in (
        LATERAL_RECORDS
    ):
        branch = record[
            "branch"
        ]

        core.validate_branch_connectivity(
            branch
        )

        root = (
            branch
            .physics[
                "origins"
            ][0]
        )

        error = float(
            (
                root
                - record[
                    "junction_world"
                ]
            ).GetLength()
        )

        if error > 1e-8:
            raise RuntimeError(
                f"{branch.name}: "
                f"root/junction mismatch "
                f"{error}"
            )


# ============================================================================
# VISUAL-ONLY GROUND
# ============================================================================

def build_visual_ground(
    stage,
):
    """
    Debug/reference plane ONLY.
    Deliberately no UsdPhysics.CollisionAPI.
    """
    ground = UsdGeom.Mesh.Define(
        stage,
        "/World/VisualGround",
    )

    size = 0.75

    ground.CreatePointsAttr().Set([
        Gf.Vec3f(
            -size,
            -size,
            0.0,
        ),
        Gf.Vec3f(
            size,
            -size,
            0.0,
        ),
        Gf.Vec3f(
            size,
            size,
            0.0,
        ),
        Gf.Vec3f(
            -size,
            size,
            0.0,
        ),
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


# ============================================================================
# BUILD
# ============================================================================

def build_stage(
    output_path=OUTPUT_USD,
):
    validate_topology()

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

    # Correct articulation root, validated in 2C-A fixed.
    UsdPhysics.ArticulationRootAPI.Apply(
        plant_physics.GetPrim()
    )

    UsdGeom.Xform.Define(
        stage,
        "/World/PlantVisual",
    )

    # ------------------------------------------------------------------
    # Physics
    # ------------------------------------------------------------------

    core.build_branch_physics(
        stage,
        MAIN,
    )

    for branch in LATERALS:
        core.build_branch_physics(
            stage,
            branch,
        )

    # Only MAIN is fixed to world.
    core.create_world_anchor(
        stage,
        MAIN.link_paths[0],
    )

    junction_infos = []

    for record in (
        LATERAL_RECORDS
    ):
        info = (
            core.create_junction_joint(
                stage,
                parent_branch=MAIN,
                parent_link_index=record[
                    "parent_link_index"
                ],
                child_branch=record[
                    "branch"
                ],
                attachment_world=record[
                    "junction_world"
                ],
                stiffness=None,
                damping=None,
                bend_limit_deg=BEND_LIMIT_DEG,
            )
        )

        junction_infos.append(
            info
        )

    # One articulation for the complete branching tree.
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
        1
    )
    articulation.CreateEnabledSelfCollisionsAttr().Set(
        False
    )
    articulation.CreateSleepThresholdAttr().Set(
        0.0
    )

    # ------------------------------------------------------------------
    # Visuals: one independent skeleton per branch
    # ------------------------------------------------------------------

    core.build_branch_visual(
        stage,
        MAIN,
        color=(
            0.22,
            0.52,
            0.16,
        ),
    )

    lateral_colors = (
        (0.42, 0.67, 0.18),
        (0.36, 0.62, 0.20),
        (0.47, 0.70, 0.23),
    )

    for branch, color in zip(
        LATERALS,
        lateral_colors,
    ):
        core.build_branch_visual(
            stage,
            branch,
            color=color,
        )

    build_visual_ground(
        stage
    )

    stage.Save()

    total_links = sum(
        branch.spec.physics_links
        for branch in BRANCHES
    )

    internal_d6 = sum(
        branch.spec.physics_links - 1
        for branch in BRANCHES
    )

    junction_d6 = len(
        LATERALS
    )

    print("=" * 84)
    print(
        "TEST 2C-B2 — MULTIPLE BRANCHES WITH BEAM PHYSICS"
    )
    print("=" * 84)
    print(f"[OK] {output_path}")
    print()
    print("SUSPENDED TEST:")
    print(
        f"  root height       : "
        f"{ROOT_HEIGHT:.3f} m"
    )
    print(
        "  ground collision  : OFF"
    )
    print()
    print("BEAM PHYSICS:")
    print(
        f"  E                 : "
        f"{YOUNG_MODULUS_PA / 1e6:.1f} MPa"
    )
    print(
        f"  damping ratio     : "
        f"{DAMPING_RATIO:.2f}"
    )
    print(
        f"  bend limit        : "
        f"+/- {BEND_LIMIT_DEG:.1f} deg"
    )
    print(
        f"  Main K / D        : "
        f"{MAIN_K:.6f} Nm/deg / "
        f"{MAIN_D:.6f} Nms/deg"
    )
    for branch in LATERALS:
        print(
            f"  {branch.name} K / D  : "
            f"{branch.spec.joint_stiffness:.6f} / "
            f"{branch.spec.joint_damping:.6f}"
        )
    print()
    print("TOPOLOGY:")
    print(
        f"  MainStem          : "
        f"{MAIN.spec.physics_links} links"
    )

    for record in (
        LATERAL_RECORDS
    ):
        branch = record[
            "branch"
        ]

        print(
            f"  {branch.name:<17}: "
            f"{branch.spec.physics_links} links, "
            f"main fraction={record['junction_fraction']:.2f}, "
            f"parent Link_{record['parent_link_index'] + 1:02d}, "
            f"az={record['azimuth_deg']:+.1f}°, "
            f"el={record['elevation_deg']:+.1f}°"
        )

    print()
    print(
        f"  total rigid links : "
        f"{total_links}"
    )
    print(
        f"  internal D6       : "
        f"{internal_d6}"
    )
    print(
        f"  junction D6       : "
        f"{junction_d6}"
    )
    print(
        f"  total D6          : "
        f"{internal_d6 + junction_d6}"
    )
    print(
        f"  skeletons         : "
        f"{len(BRANCHES)}"
    )

    print()
    print("JUNCTION FILTERING:")

    for record, info in zip(
        LATERAL_RECORDS,
        junction_infos,
    ):
        filtered = ", ".join(
            f"Main Link_{index + 1:02d}"
            for index in info[
                "filtered_parent_indices"
            ]
        )

        print(
            f"  {record['branch'].name}: "
            f"{filtered}"
        )

    print()
    print("EXPECTED:")
    print(
        "  Visual meshes may intersect at each junction."
    )
    print(
        "  Seamless visual junction blending is NOT part of 2C-B."
    )
    print("=" * 84)

    return output_path


if __name__ == "__main__":
    build_stage()
