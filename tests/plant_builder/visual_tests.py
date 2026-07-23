"""
visual_tests.py

Progressive visual test suite for PlantBuilder.
Each test builds a more complex plant and opens it in Isaac Sim.

Usage:
    ./run_experiment.sh 1    # Just a trunk
    ./run_experiment.sh 2    # Trunk + 1 branch
    ./run_experiment.sh 3    # Trunk + branch with extensions
    ./run_experiment.sh 4    # Trunk + branch + subbranch
    ./run_experiment.sh 5    # Full tree: multiple branches & subbranches
"""

import os
import sys

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(script_dir))
sys.path.insert(0, project_root)

from src.plant_model.plant_builder import PlantBuilder

OUTPUT = os.path.join(project_root, "data", "usd_models", "builder_visual_test.usda")


# =============================================================================
#  PhysX helpers
# =============================================================================
def setup_physx(stage, stem_path):
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableStabilizationAttr().Set(True)

    art = PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(stem_path))
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)


# =============================================================================
#  Test scenarios
# =============================================================================

def test_1_trunk_only(builder):
    """TEST 1: Simple vertical trunk — 5 internodes."""
    print("\n🧪 TEST 1: Trunk only (5 internodes)")
    prev = builder.create_root("T01", radius=0.10, length=0.4)
    for i in range(2, 6):
        prev = builder.add_internode(prev, f"T{i:02d}", radius=0.10, length=0.4)


def test_2_trunk_and_branch(builder):
    """TEST 2: Trunk + 1 lateral branch."""
    print("\n🧪 TEST 2: Trunk + 1 branch")
    t1 = builder.create_root("T01", radius=0.10, length=0.5)
    t2 = builder.add_internode(t1, "T02", radius=0.10, length=0.5)
    t3 = builder.add_internode(t2, "T03", radius=0.09, length=0.5)

    builder.add_lateral_branch(t2, "B01", radius=0.04, length=0.3,
                               z_offset_ratio=0.8, tilt_angle=45,
                               rot_around_parent=90)


def test_3_branch_with_extension(builder):
    """TEST 3: Trunk + branch extended with internodes."""
    print("\n🧪 TEST 3: Branch extended with internodes")
    t1 = builder.create_root("T01", radius=0.10, length=0.5)
    t2 = builder.add_internode(t1, "T02", radius=0.10, length=0.5)
    t3 = builder.add_internode(t2, "T03", radius=0.09, length=0.5)

    b1 = builder.add_lateral_branch(t2, "B01", radius=0.04, length=0.25,
                                    z_offset_ratio=0.8, tilt_angle=45,
                                    rot_around_parent=90)
    b2 = builder.add_internode(b1, "B02", radius=0.04, length=0.25)
    b3 = builder.add_internode(b2, "B03", radius=0.03, length=0.20)


def test_4_subbranch(builder):
    """TEST 4: Trunk → Branch → Subbranch."""
    print("\n🧪 TEST 4: Subbranch off a branch")
    t1 = builder.create_root("T01", radius=0.10, length=0.5)
    t2 = builder.add_internode(t1, "T02", radius=0.10, length=0.5)
    t3 = builder.add_internode(t2, "T03", radius=0.09, length=0.5)

    b1 = builder.add_lateral_branch(t2, "B01", radius=0.04, length=0.25,
                                    z_offset_ratio=0.8, tilt_angle=45,
                                    rot_around_parent=90)
    b2 = builder.add_internode(b1, "B02", radius=0.04, length=0.25)

    # Subbranch off the first branch segment
    builder.add_lateral_branch(b1, "SB01", radius=0.02, length=0.15,
                               z_offset_ratio=0.5, tilt_angle=30,
                               rot_around_parent=90)


def test_5_full_tree(builder):
    """TEST 5: Full tree — tall trunk, 3 branches at different heights, subbranches."""
    print("\n🧪 TEST 5: Full tree with branches & subbranches")

    # Trunk: 8 internodes
    prev = builder.create_root("T01", radius=0.10, length=0.3)
    for i in range(2, 9):
        r = max(0.06, 0.10 - i * 0.005)
        prev = builder.add_internode(prev, f"T{i:02d}", radius=r, length=0.3)

    # Branch A off T03, going East
    bA1 = builder.add_lateral_branch("T03", "BA01", radius=0.04, length=0.20,
                                     z_offset_ratio=0.8, tilt_angle=50,
                                     rot_around_parent=0)
    bA2 = builder.add_internode(bA1, "BA02", radius=0.04, length=0.20)
    bA3 = builder.add_internode(bA2, "BA03", radius=0.03, length=0.15)

    # Branch B off T05, going West
    bB1 = builder.add_lateral_branch("T05", "BB01", radius=0.04, length=0.20,
                                     z_offset_ratio=0.8, tilt_angle=55,
                                     rot_around_parent=180)
    bB2 = builder.add_internode(bB1, "BB02", radius=0.04, length=0.20)

    # Branch C off T07, going North
    bC1 = builder.add_lateral_branch("T07", "BC01", radius=0.04, length=0.20,
                                     z_offset_ratio=0.8, tilt_angle=45,
                                     rot_around_parent=90)
    bC2 = builder.add_internode(bC1, "BC02", radius=0.03, length=0.15)

    # Subbranch off Branch A, segment 2
    sA1 = builder.add_lateral_branch(bA2, "SA01", radius=0.02, length=0.12,
                                     z_offset_ratio=0.5, tilt_angle=35,
                                     rot_around_parent=90)
    builder.add_internode(sA1, "SA02", radius=0.02, length=0.10)

    # Subbranch off Branch B, segment 1
    builder.add_lateral_branch(bB1, "SB01", radius=0.02, length=0.12,
                               z_offset_ratio=0.6, tilt_angle=40,
                               rot_around_parent=-90)

    # Subbranch off Branch C, segment 1 — two opposite subbranches
    builder.add_lateral_branch(bC1, "SC01", radius=0.02, length=0.10,
                               z_offset_ratio=0.5, tilt_angle=30,
                               rot_around_parent=90)
    builder.add_lateral_branch(bC1, "SC02", radius=0.02, length=0.10,
                               z_offset_ratio=0.5, tilt_angle=30,
                               rot_around_parent=-90)


def test_6_flexible_tree(builder):
    """TEST 6: Tall flexible tree — long branches that bend under gravity."""
    print("\n🧪 TEST 6: Flexible tree (bending test)")

    # ── Trunk: 10 stiff segments ──────────────────────────────────────
    # Very high stiffness so the trunk barely moves
    TRUNK_STIFF = 800.0
    TRUNK_DAMP  = 2.0

    prev = builder.create_root("T01", radius=0.12, length=0.25, mass=2.0)
    for i in range(2, 11):
        r = max(0.07, 0.12 - i * 0.005)
        prev = builder.add_internode(prev, f"T{i:02d}", radius=r, length=0.25,
                                     mass=1.5, stiffness=TRUNK_STIFF,
                                     damping=TRUNK_DAMP)

    # ── Branch stiffness tiers ────────────────────────────────────────
    # Branches: moderate — they should sag visibly but hold shape
    BR_STIFF_BASE = 50_000.0   # base attachment (strong)
    BR_DAMP_BASE  = 2_000.0
    BR_STIFF_INT  = 150.0      # internal joints (flexible)
    BR_DAMP_INT   = 30.0

    # Subbranches: softer — they droop nicely
    SB_STIFF_BASE = 20_000.0
    SB_DAMP_BASE  = 1_000.0
    SB_STIFF_INT  = 80.0
    SB_DAMP_INT   = 20.0

    # ── Branch A: off T03, 5 segments going East ─────────────────────
    bA = builder.add_lateral_branch("T03", "BA01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.8, tilt_angle=55,
                                    rot_around_parent=0, mass=0.3,
                                    stiffness=BR_STIFF_BASE, damping=BR_DAMP_BASE)
    for i in range(2, 6):
        r = max(0.025, 0.04 - i * 0.003)
        bA = builder.add_internode(bA, f"BA{i:02d}", radius=r, length=0.18,
                                   mass=0.2, stiffness=BR_STIFF_INT,
                                   damping=BR_DAMP_INT)

    # ── Branch B: off T05, 5 segments going West ─────────────────────
    bB = builder.add_lateral_branch("T05", "BB01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.8, tilt_angle=50,
                                    rot_around_parent=180, mass=0.3,
                                    stiffness=BR_STIFF_BASE, damping=BR_DAMP_BASE)
    for i in range(2, 6):
        r = max(0.025, 0.04 - i * 0.003)
        bB = builder.add_internode(bB, f"BB{i:02d}", radius=r, length=0.18,
                                   mass=0.2, stiffness=BR_STIFF_INT,
                                   damping=BR_DAMP_INT)

    # ── Branch C: off T07, 4 segments going North ────────────────────
    bC = builder.add_lateral_branch("T07", "BC01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.8, tilt_angle=45,
                                    rot_around_parent=90, mass=0.3,
                                    stiffness=BR_STIFF_BASE, damping=BR_DAMP_BASE)
    for i in range(2, 5):
        r = max(0.025, 0.04 - i * 0.003)
        bC = builder.add_internode(bC, f"BC{i:02d}", radius=r, length=0.18,
                                   mass=0.2, stiffness=BR_STIFF_INT,
                                   damping=BR_DAMP_INT)

    # ── Branch D: off T08, 4 segments going South ────────────────────
    bD = builder.add_lateral_branch("T08", "BD01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.5, tilt_angle=50,
                                    rot_around_parent=270, mass=0.3,
                                    stiffness=BR_STIFF_BASE, damping=BR_DAMP_BASE)
    for i in range(2, 5):
        r = max(0.025, 0.04 - i * 0.003)
        bD = builder.add_internode(bD, f"BD{i:02d}", radius=r, length=0.18,
                                   mass=0.2, stiffness=BR_STIFF_INT,
                                   damping=BR_DAMP_INT)

    # ── Subbranches (soft, droopy) ────────────────────────────────────
    # Sub off Branch A, segment 3 — two opposite
    sA1 = builder.add_lateral_branch("BA03", "SA01", radius=0.02, length=0.12,
                                     z_offset_ratio=0.5, tilt_angle=40,
                                     rot_around_parent=90, mass=0.1,
                                     stiffness=SB_STIFF_BASE, damping=SB_DAMP_BASE)
    for i in range(2, 4):
        sA1 = builder.add_internode(sA1, f"SA0{i}", radius=0.015, length=0.10,
                                    mass=0.08, stiffness=SB_STIFF_INT,
                                    damping=SB_DAMP_INT)

    sA2 = builder.add_lateral_branch("BA03", "SA04", radius=0.02, length=0.12,
                                     z_offset_ratio=0.5, tilt_angle=40,
                                     rot_around_parent=-90, mass=0.1,
                                     stiffness=SB_STIFF_BASE, damping=SB_DAMP_BASE)
    builder.add_internode(sA2, "SA05", radius=0.015, length=0.10,
                          mass=0.08, stiffness=SB_STIFF_INT, damping=SB_DAMP_INT)

    # Sub off Branch B, segment 2
    sB1 = builder.add_lateral_branch("BB02", "SB01", radius=0.02, length=0.12,
                                     z_offset_ratio=0.6, tilt_angle=35,
                                     rot_around_parent=90, mass=0.1,
                                     stiffness=SB_STIFF_BASE, damping=SB_DAMP_BASE)
    for i in range(2, 4):
        sB1 = builder.add_internode(sB1, f"SB0{i}", radius=0.015, length=0.10,
                                    mass=0.08, stiffness=SB_STIFF_INT,
                                    damping=SB_DAMP_INT)

    # Sub off Branch C, segment 2
    sC1 = builder.add_lateral_branch("BC02", "SC01", radius=0.02, length=0.10,
                                     z_offset_ratio=0.5, tilt_angle=30,
                                     rot_around_parent=0, mass=0.1,
                                     stiffness=SB_STIFF_BASE, damping=SB_DAMP_BASE)
    builder.add_internode(sC1, "SC02", radius=0.015, length=0.10,
                          mass=0.08, stiffness=SB_STIFF_INT, damping=SB_DAMP_INT)

    # Sub off Branch D, segment 2 — two opposite
    sD1 = builder.add_lateral_branch("BD02", "SD01", radius=0.02, length=0.10,
                                     z_offset_ratio=0.5, tilt_angle=35,
                                     rot_around_parent=90, mass=0.1,
                                     stiffness=SB_STIFF_BASE, damping=SB_DAMP_BASE)
    builder.add_internode(sD1, "SD02", radius=0.015, length=0.10,
                          mass=0.08, stiffness=SB_STIFF_INT, damping=SB_DAMP_INT)

    sD2 = builder.add_lateral_branch("BD02", "SD03", radius=0.02, length=0.10,
                                     z_offset_ratio=0.5, tilt_angle=35,
                                     rot_around_parent=-90, mass=0.1,
                                     stiffness=SB_STIFF_BASE, damping=SB_DAMP_BASE)
    builder.add_internode(sD2, "SD04", radius=0.015, length=0.10,
                          mass=0.08, stiffness=SB_STIFF_INT, damping=SB_DAMP_INT)

    # ── Sub-Subbranches (thinnest, very flexible, 4 segments deep) ──────
    SSB_STIFF = 0.001
    SSB_DAMP  = 0.00002
    
    # Off sA1 (SA03) - Long, thin branch with 4 segments
    ssA = builder.add_lateral_branch("SA03", "SSA01", radius=0.005, length=0.04,
                                      z_offset_ratio=0.5, tilt_angle=45,
                                      rot_around_parent=90, mass=0.02,
                                      stiffness=SSB_STIFF, damping=SSB_DAMP)
    for i in range(2, 5):
        r = max(0.002, 0.005 - (i - 1) * 0.001)
        ssA = builder.add_internode(ssA, f"SSA0{i}", radius=r, length=0.04,
                                    mass=0.01, stiffness=SSB_STIFF/(i), damping=SSB_DAMP/(i))


def test_7_tree_with_leaves(builder):
    """TEST 7: Branches with leaves attached at tips and along segments."""
    print("\n🧪 TEST 7: Tree with leaves")

    # ── Trunk ─────────────────────────────────────────────────────────
    prev = builder.create_root("T01", radius=0.10, length=0.4, mass=2.0)
    for i in range(2, 7):
        r = max(0.06, 0.10 - i * 0.006)
        prev = builder.add_internode(prev, f"T{i:02d}", radius=r, length=0.35,
                                     mass=1.5, stiffness=500_000, damping=100)

    # ── Branch A: off T03, East, 4 segments ──────────────────────────
    bA = builder.add_lateral_branch("T03", "BA01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.8, tilt_angle=50,
                                    rot_around_parent=0, mass=0.3,
                                    stiffness=50_000, damping=2_000)
    for i in range(2, 5):
        bA = builder.add_internode(bA, f"BA{i:02d}", radius=0.03, length=0.15,
                                   mass=0.2, stiffness=200, damping=30)

    # Leaves along Branch A
    builder.add_leaf("BA02", "LA01", leaf_length=0.06, leaf_width=0.03,
                     z_offset_ratio=0.7, tilt_angle=65, rot_around_parent=90)
    builder.add_leaf("BA02", "LA02", leaf_length=0.06, leaf_width=0.03,
                     z_offset_ratio=0.7, tilt_angle=65, rot_around_parent=-90)
    builder.add_leaf("BA03", "LA03", leaf_length=0.07, leaf_width=0.04,
                     z_offset_ratio=0.6, tilt_angle=60, rot_around_parent=90)
    builder.add_leaf("BA03", "LA04", leaf_length=0.07, leaf_width=0.04,
                     z_offset_ratio=0.6, tilt_angle=60, rot_around_parent=-90)
    # Terminal leaf at tip of branch
    builder.add_leaf("BA04", "LA05", leaf_length=0.08, leaf_width=0.05,
                     z_offset_ratio=1.0, tilt_angle=45, rot_around_parent=0)

    # ── Branch B: off T05, West, 3 segments ──────────────────────────
    bB = builder.add_lateral_branch("T05", "BB01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.8, tilt_angle=55,
                                    rot_around_parent=180, mass=0.3,
                                    stiffness=50_000, damping=2_000)
    for i in range(2, 4):
        bB = builder.add_internode(bB, f"BB{i:02d}", radius=0.03, length=0.15,
                                   mass=0.2, stiffness=200, damping=30)

    # Leaves along Branch B
    builder.add_leaf("BB01", "LB01", leaf_length=0.06, leaf_width=0.03,
                     z_offset_ratio=0.7, tilt_angle=65, rot_around_parent=90)
    builder.add_leaf("BB01", "LB02", leaf_length=0.06, leaf_width=0.03,
                     z_offset_ratio=0.7, tilt_angle=65, rot_around_parent=-90)
    builder.add_leaf("BB02", "LB03", leaf_length=0.07, leaf_width=0.04,
                     z_offset_ratio=0.6, tilt_angle=60, rot_around_parent=0)
    # Terminal leaf
    builder.add_leaf("BB03", "LB04", leaf_length=0.08, leaf_width=0.05,
                     z_offset_ratio=1.0, tilt_angle=45, rot_around_parent=0)

    # ── Leaves directly on trunk (top) ────────────────────────────────
    builder.add_leaf("T06", "LT01", leaf_length=0.07, leaf_width=0.04,
                     z_offset_ratio=0.8, tilt_angle=55, rot_around_parent=45)
    builder.add_leaf("T06", "LT02", leaf_length=0.07, leaf_width=0.04,
                     z_offset_ratio=0.8, tilt_angle=55, rot_around_parent=225)


def test_8_compound_leaf(builder):
    """TEST 8: Articulated compound leaf (two alternatives for physics stability)."""
    print("\n🧪 TEST 8: Articulated compound leaf")

    # ── Trunk: 4 stiff segments ───────────────────────────────────────
    prev = builder.create_root("T01", radius=0.10, length=0.4, mass=2.0)
    for i in range(2, 5):
        r = max(0.06, 0.10 - i * 0.006)
        prev = builder.add_internode(prev, f"T{i:02d}", radius=r, length=0.4,
                                     mass=1.5, stiffness=500_000, damping=100)

    # ── Leaf A: relatively thin components (Aspect ratio ~18) ────────
    # 4 segments, 15cm long, 8mm radius
    LA_len = 0.15
    LA_rad = 0.008
    rachis_A = builder.add_lateral_branch("T02", "LA01", radius=LA_rad, length=LA_len,
                                          z_offset_ratio=0.8, tilt_angle=60,
                                          rot_around_parent=0, mass=0.03,
                                          stiffness=10, damping=3)
    
    for i in range(2, 5):
        r = max(0.004, LA_rad - i * 0.001)
        rachis_A = builder.add_internode(rachis_A, f"LA{i:02d}", radius=r, length=LA_len,
                                         mass=0.02, stiffness=0.1, damping=0.1)
        # Lateral leaflets at each node
        builder.add_leaf(f"LA{i-1:02d}", f"LeafA_{i}a", leaf_length=0.08, leaf_width=0.04,
                         z_offset_ratio=0.9, tilt_angle=70, rot_around_parent=90)
        builder.add_leaf(f"LA{i-1:02d}", f"LeafA_{i}b", leaf_length=0.08, leaf_width=0.04,
                         z_offset_ratio=0.9, tilt_angle=70, rot_around_parent=-90)

    # Terminal leaflet
    builder.add_leaf("LA04", "LeafA_term", leaf_length=0.1, leaf_width=0.05,
                     z_offset_ratio=1.0, tilt_angle=20, rot_around_parent=0)


    # ── Leaf B: more segments, shorter (squared cylinders) ───────────
    # 10 segments, 4cm long, 2cm radius (Aspect ratio = 2)
    LB_len = 0.04
    LB_rad = 0.02
    rachis_B = builder.add_lateral_branch("T03", "LB01", radius=LB_rad, length=LB_len,
                                          z_offset_ratio=0.8, tilt_angle=60,
                                          rot_around_parent=180, mass=0.04,
                                          stiffness=2_000, damping=100)

    for i in range(2, 11):
        r = max(0.008, LB_rad - i * 0.001)
        # Progressively decrease stiffness for the branch with more segments
        # from 800 down to ~50
        stiff = max(200, 4000 - (i - 2) * 80)
        damp = max(50, 200 - (i - 2) * 8)
        rachis_B = builder.add_internode(rachis_B, f"LB{i:02d}", radius=r, length=LB_len,
                                         mass=0.03, stiffness=stiff, damping=damp)
        
        # Add lateral leaflets every 3 segments (to match approx spacing of Leaf A)
        if i % 3 == 0:
            builder.add_leaf(f"LB{i-1:02d}", f"LeafB_{i}a", leaf_length=0.08, leaf_width=0.04,
                             z_offset_ratio=0.8, tilt_angle=70, rot_around_parent=90)
            builder.add_leaf(f"LB{i-1:02d}", f"LeafB_{i}b", leaf_length=0.08, leaf_width=0.04,
                             z_offset_ratio=0.8, tilt_angle=70, rot_around_parent=-90)

    # Terminal leaflet
    builder.add_leaf("LB10", "LeafB_term", leaf_length=0.1, leaf_width=0.05,
                     z_offset_ratio=1.0, tilt_angle=20, rot_around_parent=0)


    # ── Leaf C: many segments, thin and squared ────────────────────────
    # 25 segments, 1.6cm long, 8mm radius (Aspect ratio = 2)
    LC_len = 0.032
    LC_rad = 0.008
    rachis_C = builder.add_lateral_branch("T04", "LC01", radius=LC_rad, length=LC_len,
                                          z_offset_ratio=0.8, tilt_angle=60,
                                          rot_around_parent=90, mass=0.01,
                                          stiffness=1, damping=0)

    for i in range(2, 8):
        r = max(0.002, LC_rad - i * 0.0002)
        # Progressively decrease stiffness, scaled for 25 segments
        stiff = 0.001
        damp = 0.0001
        rachis_C = builder.add_internode(rachis_C, f"LC{i:02d}", radius=r, length=LC_len,
                                         mass=0.005, stiffness=stiff, damping=damp)
        
        # Add lateral leaflets every 6 segments
        if i % 2 == 0:
            builder.add_leaf(f"LC{i-1:02d}", f"LeafC_{i}a", leaf_length=0.04, leaf_width=0.02,
                             z_offset_ratio=0.8, tilt_angle=70, rot_around_parent=90, stiffness=0.0005)
            builder.add_leaf(f"LC{i-1:02d}", f"LeafC_{i}b", leaf_length=0.04, leaf_width=0.02,
                             z_offset_ratio=0.8, tilt_angle=70, rot_around_parent=-90, stiffness=0.0005)

    # Terminal leaflet
    builder.add_leaf("LC07", "LeafC_term", leaf_length=0.04, leaf_width=0.02,
                     z_offset_ratio=1.0, tilt_angle=20, rot_around_parent=0)


def test_9_tomato_truss(builder):
    """TEST 9: Branch with an articulated tomato truss bearing fruits."""
    print("\n🧪 TEST 9: Tomato truss")

    # ── Trunk: 6 stiff segments ───────────────────────────────────────
    prev = builder.create_root("T01", radius=0.10, length=0.35, mass=2.0)
    for i in range(2, 7):
        r = max(0.06, 0.10 - i * 0.006)
        prev = builder.add_internode(prev, f"T{i:02d}", radius=r, length=0.35,
                                     mass=1.5, stiffness=500_000, damping=100)

    # ── Branch off T03, East, 3 segments ──────────────────────────────
    bA = builder.add_lateral_branch("T03", "BA01", radius=0.04, length=0.18,
                                    z_offset_ratio=0.8, tilt_angle=50,
                                    rot_around_parent=0, mass=0.3,
                                    stiffness=50_000, damping=2_000)
    for i in range(2, 4):
        bA = builder.add_internode(bA, f"BA{i:02d}", radius=0.03, length=0.15,
                                   mass=0.2, stiffness=200, damping=30)

    # ── Truss rachis off the branch (BA02), 4 segments ────────────────
    rachis_ids = builder.add_truss_rachis(
        "BA02", "TR1",
        n_segments=4,
        rachis_radius=0.015,
        rachis_seg_length=0.09,
        z_offset_ratio=0.8,
        tilt_angle=45.0,
        rot_around_parent=90,
        mass=0.17,
        stiffness_base=0.5, damping_base=0.12,
        stiffness_int=0.1, damping_int=0.03,
    )

    break_force = 500.0

    # ── Attach 4 fruits, alternating sides ────────────────────────────
    builder.add_fruit(rachis_ids[0], "F01", fruit_radius=0.054,
                      pedicel_length=0.050, pedicel_radius=0.01, lateral_angle=90, is_ripe=False,
                      mass=0.13, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)
    builder.add_fruit(rachis_ids[1], "F02", fruit_radius=0.060,
                      pedicel_length=0.050, pedicel_radius=0.01, lateral_angle=-90, is_ripe=False,
                      mass=0.17, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)
    builder.add_fruit(rachis_ids[2], "F03", fruit_radius=0.066,
                      pedicel_length=0.050, pedicel_radius=0.01, lateral_angle=90, is_ripe=True,
                      mass=0.20, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)
    # Terminal fruit — at the tip of the last rachis segment
    builder.add_fruit(rachis_ids[3], "F04", fruit_radius=0.075,
                      pedicel_length=0.045, pedicel_radius=0.01, lateral_angle=0, is_ripe=True,
                      mass=0.27, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)

    # ── A second truss off T05 for more visual interest ───────────────
    rachis2 = builder.add_truss_rachis(
        "T05", "TR2",
        n_segments=3,
        rachis_radius=0.015,
        rachis_seg_length=0.09,
        z_offset_ratio=0.7,
        tilt_angle=50.0,
        rot_around_parent=180,
        mass=0.17,
        stiffness_base=0.5, damping_base=0.12,
        stiffness_int=0.1, damping_int=0.03,
    )
    builder.add_fruit(rachis2[0], "F05", fruit_radius=0.045,
                      pedicel_length=0.045, pedicel_radius=0.01, lateral_angle=90, is_ripe=False,
                      mass=0.10, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)
    builder.add_fruit(rachis2[1], "F06", fruit_radius=0.054,
                      pedicel_length=0.045, pedicel_radius=0.01, lateral_angle=-90, is_ripe=False,
                      mass=0.13, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)
    builder.add_fruit(rachis2[2], "F07", fruit_radius=0.060,
                      pedicel_length=0.038, pedicel_radius=0.01, lateral_angle=0, is_ripe=True,
                      mass=0.17, stiffness=0.025, damping=0.006, collisions=True, break_force=break_force)


def test_10_scaled_small_dimensions(builder):
    """TEST 10: Real small dimensions with artificial mass for stability over many segments."""
    import math
    print("\n🧪 TEST 10: Real small dimensions (10 segments, physics stabilized)")

    BAKED_SCALE = 1.0

    # To prevent 'Invalid PhysX transform' due to float32 underflow in the inertia tensor,
    # we MUST artificially inflate the mass of tiny millimetric objects. 
    def get_mass(radius, length):
        vol = math.pi * (radius ** 2) * length
        return max(vol * 500.0, 0.05)

    real_trunk_rad = 0.01
    real_trunk_len = 0.04
    
    t_rad = real_trunk_rad * BAKED_SCALE
    t_len = real_trunk_len * BAKED_SCALE
    
    # Trunk is relatively heavy, stiffness can be around 1000
    prev = builder.create_root("T01", radius=t_rad, length=t_len, mass=get_mass(t_rad, t_len))
    for i in range(2, 5):
        r = max(0.006, real_trunk_rad - i * 0.0006) * BAKED_SCALE
        prev = builder.add_internode(prev, f"T{i:02d}", radius=r, length=t_len,
                                     mass=get_mass(r, t_len), 
                                     stiffness=1000.0, damping=100.0)

    # ── Branch B: 10 segments, 4mm long, 2mm radius ───────────
    real_LB_len = 0.004
    real_LB_rad = 0.002
    
    lb_rad = real_LB_rad * BAKED_SCALE
    lb_len = real_LB_len * BAKED_SCALE
    
    # 10 segments of 0.05kg = 0.5kg total at ~4cm length.
    # We need a solid stiffness at the base (e.g. 50.0) to hold up 0.5kg.
    stiff_base = 50.0
    damp_base = 5.0

    rachis_B = builder.add_lateral_branch("T02", "LB01", 
                                          radius=lb_rad, length=lb_len,
                                          z_offset_ratio=0.8, tilt_angle=60,
                                          rot_around_parent=180, mass=get_mass(lb_rad, lb_len),
                                          stiffness=stiff_base, damping=damp_base)

    for i in range(2, 11):  # Full 10 segments!
        r = max(0.0008, real_LB_rad - i * 0.0001) * BAKED_SCALE
        stiff = max(1.0, stiff_base - (i * 4.5))
        damp = max(0.1, damp_base - (i * 0.45))
        
        rachis_B = builder.add_internode(rachis_B, f"LB{i:02d}", 
                                         radius=r, length=lb_len,
                                         mass=get_mass(r, lb_len), stiffness=stiff, damping=damp)
        
        # Add small subbranches to each segment (every 3 segments)
        if i % 3 == 0:
            sub_len = 0.008 * BAKED_SCALE
            sub_rad = 0.001 * BAKED_SCALE
            
            builder.add_lateral_branch(f"LB{i-1:02d}", f"SubB_{i}a", 
                                       radius=sub_rad, length=sub_len,
                                       z_offset_ratio=0.8, tilt_angle=70, rot_around_parent=90,
                                       mass=get_mass(sub_rad, sub_len),
                                       stiffness=2.0, damping=0.2)
            builder.add_lateral_branch(f"LB{i-1:02d}", f"SubB_{i}b", 
                                       radius=sub_rad, length=sub_len,
                                       z_offset_ratio=0.8, tilt_angle=70, rot_around_parent=-90,
                                       mass=get_mass(sub_rad, sub_len),
                                       stiffness=2.0, damping=0.2)

    # Terminal subbranch
    term_len = 0.01 * BAKED_SCALE
    term_rad = 0.001 * BAKED_SCALE
    builder.add_lateral_branch("LB10", "SubB_term", 
                               radius=term_rad, length=term_len,
                               z_offset_ratio=1.0, tilt_angle=20, rot_around_parent=0,
                               mass=get_mass(term_rad, term_len),
                               stiffness=2.0, damping=0.2)

    # For extremely small objects, PhysX needs a very tiny contact offset
    for prim in builder.stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collider = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            if prim.IsA(UsdGeom.Cylinder):
                cyl = UsdGeom.Cylinder(prim)
                rad = cyl.GetRadiusAttr().Get()
                if rad:
                    collider.CreateContactOffsetAttr().Set(rad * 0.05)
                    collider.CreateRestOffsetAttr().Set(rad * 0.01)

TESTS = {
    1: test_1_trunk_only,
    2: test_2_trunk_and_branch,
    3: test_3_branch_with_extension,
    4: test_4_subbranch,
    5: test_5_full_tree,
    6: test_6_flexible_tree,
    7: test_7_tree_with_leaves,
    8: test_8_compound_leaf,
    9: test_9_tomato_truss,
    10: test_10_scaled_small_dimensions,
}


# =============================================================================
#  Main
# =============================================================================
def main():
    # Parse test number from CLI
    test_num = 1
    for arg in sys.argv[1:]:
        if arg.isdigit():
            test_num = int(arg)
            break

    if test_num not in TESTS:
        print(f"Unknown test {test_num}. Available: {sorted(TESTS.keys())}")
        simulation_app.close()
        sys.exit(1)

    test_fn = TESTS[test_num]
    print(f"\n{'='*50}")
    print(f"  PlantBuilder Visual Test {test_num}/{len(TESTS)}")
    print(f"  {test_fn.__doc__}")
    print(f"{'='*50}")

    if os.path.exists(OUTPUT):
        os.remove(OUTPUT)

    stage = Usd.Stage.CreateNew(OUTPUT)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    builder = PlantBuilder(stage, "/World/Stem")
    test_fn(builder)

    setup_physx(stage, "/World/Stem")
    stage.GetRootLayer().Save()
    print(f"\n[OK] USD saved → {OUTPUT}")

    omni.usd.get_context().open_stage(OUTPUT)
    w = World(stage_units_in_meters=1.0)
    w.reset()
    print("[OK] Simulation running — close window to exit.\n")

    while simulation_app.is_running():
        w.step(render=True)

    print("Done.")
    simulation_app.close()


if __name__ == "__main__":
    main()
