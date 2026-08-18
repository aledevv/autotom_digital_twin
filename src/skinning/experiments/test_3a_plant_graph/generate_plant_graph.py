"""
generate_plant_graph.py — Test 3A-A

First data-driven plant test.

The plant topology is specified only through GRAPH_NODES.
The build code does not contain special-case variables for Lateral_01,
Lateral_02, etc.

Validated foundations reused:
    - Test 2D-B2 effective physics
    - smooth centerline
    - Parallel Transport Frames
    - variable radius / taper
    - compound invisible capsule proxy
    - D6 articulation
    - exporter-style junction frames + collision filtering
    - parent swelling + child root flare
    - UsdSkel skinning

Ground is VISUAL ONLY.
"""

import os
from pathlib import Path

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

import branch_core_fixed as core
import junction_visual as jvis
from plant_graph import (
    PlantBranchNode,
    PlantGraph,
    PlantPhysicsProfile,
)


SCRIPT_DIR = Path(__file__).resolve().parent
OUTPUT_DIR = SCRIPT_DIR / "output"

OUTPUT_USD = str(
    OUTPUT_DIR
    / "test_3aa_plant_graph.usda"
)


# ============================================================================
# BASELINE = APPROVED TEST 2D-B2
# ============================================================================

PHYSICS_PROFILE = PlantPhysicsProfile(
    young_modulus_pa=1.5e6,
    damping_ratio=4.0,
    bend_limit_deg=40.0,
)


# ============================================================================
# GRAPH DATA
# ============================================================================

ROOT_HEIGHT = 0.62

GRAPH_NODES = (
    PlantBranchNode(
        name="MainStem",
        parent=None,
        control_points=(
            (
                0.000,
                0.000,
                ROOT_HEIGHT,
            ),
            (
                0.004,
                0.082,
                ROOT_HEIGHT + 0.060,
            ),
            (
                0.011,
                0.166,
                ROOT_HEIGHT + 0.126,
            ),
            (
                0.020,
                0.252,
                ROOT_HEIGHT + 0.184,
            ),
            (
                0.032,
                0.338,
                ROOT_HEIGHT + 0.226,
            ),
        ),
        physics_links=5,
        samples_per_control_segment=22,
        radial_segments=18,
        base_radius=0.015,
        tip_radius=0.0105,
        linear_density_kg_per_m=0.27,
        micro_variation_amplitude=0.010,
        micro_variation_cycles=1.6,
    ),

    PlantBranchNode(
        name="Lateral_01",
        parent="MainStem",
        attach_fraction=0.30,
        azimuth_deg=15.0,
        tilt_deg=61.0,
        length=0.205,
        curvature_side=0.010,
        curvature_vertical=-0.010,
        physics_links=3,
        base_radius=0.0108,
        tip_radius=0.0062,
        linear_density_kg_per_m=0.18,
    ),

    PlantBranchNode(
        name="Lateral_02",
        parent="MainStem",
        attach_fraction=0.55,
        azimuth_deg=145.0,
        tilt_deg=58.0,
        length=0.235,
        curvature_side=-0.012,
        curvature_vertical=-0.015,
        physics_links=3,
        base_radius=0.0103,
        tip_radius=0.0058,
        linear_density_kg_per_m=0.19,
    ),

    PlantBranchNode(
        name="Lateral_03",
        parent="MainStem",
        attach_fraction=0.78,
        azimuth_deg=265.0,
        tilt_deg=63.0,
        length=0.215,
        curvature_side=0.011,
        curvature_vertical=-0.012,
        physics_links=3,
        base_radius=0.0094,
        tip_radius=0.0052,
        linear_density_kg_per_m=0.17,
    ),
)


PLANT_GRAPH = PlantGraph(
    GRAPH_NODES,
    physics_profile=PHYSICS_PROFILE,
)

RESOLVED_NODES = (
    PLANT_GRAPH.resolve()
)

BRANCHES = [
    node.branch
    for node in RESOLVED_NODES
]

NODE_BY_NAME = {
    node.config.name: node
    for node in RESOLVED_NODES
}


# ============================================================================
# VISUAL JUNCTION PROFILES
# ============================================================================

def collect_visual_profiles():
    """
    Every child contributes:
        - one VisualBulge to its parent
        - one RootFlare to itself
    """
    parent_bulges = {
        node.config.name: []
        for node in RESOLVED_NODES
    }

    child_flare = {
        node.config.name: None
        for node in RESOLVED_NODES
    }

    for node in RESOLVED_NODES:
        if node.parent_name is None:
            continue

        parent_bulges[
            node.parent_name
        ].append(
            jvis.VisualBulge(
                center_fraction=(
                    node.config.attach_fraction
                ),
                amplitude=0.18,
                sigma_fraction=0.040,
            )
        )

        parent_radius = float(
            node.parent_radius
        )

        child_flare[
            node.config.name
        ] = jvis.RootFlare(
            parent_radius=parent_radius,
            flare_length=max(
                0.042,
                parent_radius * 3.2,
            ),
            root_parent_fraction=0.94,
            shoulder_amplitude=0.18,
            shoulder_center_fraction=0.38,
            shoulder_sigma_fraction=0.20,
        )

    return (
        parent_bulges,
        child_flare,
    )


PARENT_BULGES, CHILD_FLARES = (
    collect_visual_profiles()
)


# ============================================================================
# VISUAL GROUND
# ============================================================================

def build_visual_ground(stage):
    ground = UsdGeom.Mesh.Define(
        stage,
        "/World/VisualGround",
    )

    size = 0.85

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

    # Intentionally NO CollisionAPI.


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

    # Preserve approved 2D-B2 scene rate.
    physx_scene_api = (
        PhysxSchema.PhysxSceneAPI(
            stage.GetPrimAtPath(
                "/World/PhysicsScene"
            )
        )
    )

    physx_scene_api.GetTimeStepsPerSecondAttr().Set(
        240
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

    # ------------------------------------------------------------------
    # All branch physics from graph.
    # ------------------------------------------------------------------
    for node in RESOLVED_NODES:
        core.build_branch_physics(
            stage,
            node.branch,
        )

    # Exactly one world anchor: first rigid link of graph root.
    root_node = NODE_BY_NAME[
        PLANT_GRAPH.root_name()
    ]

    core.create_world_anchor(
        stage,
        root_node.branch.link_paths[0],
    )

    # ------------------------------------------------------------------
    # All graph edges become D6 junctions.
    # ------------------------------------------------------------------
    junction_count = 0

    for node in RESOLVED_NODES:
        if node.parent_name is None:
            continue

        parent_node = NODE_BY_NAME[
            node.parent_name
        ]

        core.create_junction_joint(
            stage,
            parent_branch=(
                parent_node.branch
            ),
            parent_link_index=(
                node.parent_link_index
            ),
            child_branch=node.branch,
            attachment_world=(
                node.attachment_world
            ),
            stiffness=None,
            damping=None,
            bend_limit_deg=(
                PHYSICS_PROFILE
                .bend_limit_deg
            ),
        )

        junction_count += 1

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

    # ------------------------------------------------------------------
    # Visual meshes from graph.
    # ------------------------------------------------------------------
    branch_color = (
        0.30,
        0.58,
        0.20,
    )

    for node in RESOLVED_NODES:
        jvis.build_branch_visual(
            stage,
            node.branch,
            color=branch_color,
            bulges=tuple(
                PARENT_BULGES[
                    node.config.name
                ]
            ),
            root_flare=(
                CHILD_FLARES[
                    node.config.name
                ]
            ),
        )

    build_visual_ground(
        stage
    )

    stage.Save()

    total_links = sum(
        node.branch.spec.physics_links
        for node in RESOLVED_NODES
    )

    internal_joints = sum(
        max(
            0,
            node.branch.spec.physics_links
            - 1,
        )
        for node in RESOLVED_NODES
    )

    print("=" * 88)
    print(
        "TEST 3A-A — DATA-DRIVEN PLANT GRAPH"
    )
    print("=" * 88)
    print(
        f"[OK] {output_path}"
    )
    print()
    print("GRAPH:")

    for node in RESOLVED_NODES:
        if node.parent_name is None:
            print(
                f"  {node.config.name}  [ROOT]"
            )
        else:
            print(
                f"  {node.parent_name}"
                f" --({node.config.attach_fraction:.2f})--> "
                f"{node.config.name}"
            )

    print()
    print("GENERATED:")
    print(
        f"  branches           : "
        f"{len(RESOLVED_NODES)}"
    )
    print(
        f"  rigid bodies       : "
        f"{total_links}"
    )
    print(
        f"  internal D6        : "
        f"{internal_joints}"
    )
    print(
        f"  junction D6        : "
        f"{junction_count}"
    )
    print(
        f"  total D6           : "
        f"{internal_joints + junction_count}"
    )
    print(
        f"  skeletons          : "
        f"{len(RESOLVED_NODES)}"
    )
    print()
    print("BASELINE PHYSICS = 2D-B2:")
    print(
        f"  E                  : "
        f"{PHYSICS_PROFILE.young_modulus_pa / 1e6:.2f} MPa"
    )
    print(
        f"  damping ratio      : "
        f"{PHYSICS_PROFILE.damping_ratio:.2f}"
    )
    print(
        f"  bend limit         : "
        f"+/- {PHYSICS_PROFILE.bend_limit_deg:.1f} deg"
    )
    print(
        "  physics Hz         : 240"
    )
    print(
        "  solver             : 32 / 4"
    )
    print(
        "  ground collision   : OFF"
    )
    print(
        "  capsule visibility : OFF"
    )
    print()
    print(
        "IMPORTANT: no lateral branch is special-cased in build_stage()."
    )
    print(
        "Adding/removing a branch from GRAPH_NODES changes the generated plant."
    )
    print("=" * 88)

    return output_path


if __name__ == "__main__":
    build_stage()
