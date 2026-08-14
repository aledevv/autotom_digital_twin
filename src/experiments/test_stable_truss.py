import os
import sys

from isaacsim import SimulationApp
# Start Isaac Sim before importing omni/USD modules
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World

# Ensure paths so we can import exporterV2
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from exporterV2.core.tree_config import (
    TrussPhysicsConfig, PhysicsRuntimeConfig, OutputConfig
)
from exporterV2.core.usd.stage import build_stage


def setup_physx(stage, stem_path):
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    # ── TEST DIAGNOSTICO: 240 Hz (Step 5) ──
    px.CreateTimeStepsPerSecondAttr().Set(240)
    px.CreateEnableStabilizationAttr().Set(True)

    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(stem_path))
    # EXTREME solver precision for the main tree
    art.CreateSolverPositionIterationCountAttr().Set(128)
    art.CreateSolverVelocityIterationCountAttr().Set(1)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)

    # ── TEST DIAGNOSTICO: SOLVER ITERATIONS SUI POMODORI ──
    for tomato_path in ["/World/TerminalBodies/tomato_1", "/World/TerminalBodies/tomato_2"]:
        prim = stage.GetPrimAtPath(tomato_path)
        if prim and prim.IsValid():
            api = PhysxSchema.PhysxRigidBodyAPI(prim)
            if not api:
                api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateSolverPositionIterationCountAttr().Set(256) # 256 for maximum precision!
            api.CreateSolverVelocityIterationCountAttr().Set(1)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)

    # ── TEST DIAGNOSTICO: ELIMINARE COLLISIONI LOCALI (Step 2) ──
    from exporterV2.core.usd.collision import add_collision_filter
    tomato_1 = "/World/TerminalBodies/tomato_1"
    add_collision_filter(stage, tomato_1, "/World/Stem/pedicel_1_Link_01")
    for i in range(1, 5):
        add_collision_filter(stage, tomato_1, f"/World/Stem/rachis_1_Link_{i:02d}")
        
    tomato_2 = "/World/TerminalBodies/tomato_2"
    add_collision_filter(stage, tomato_2, "/World/Stem/pedicel_2_Link_01")
    for i in range(1, 5):
        add_collision_filter(stage, tomato_2, f"/World/Stem/rachis_1_Link_{i:02d}")


def main():
    # ── 1. EXPERIMENTAL DETACHMENT CONFIG OVERRIDES ──
    TrussPhysicsConfig.TOMATO_DETACHMENT_ENABLED = True
    TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N = 200.0
    TrussPhysicsConfig.TOMATO_DETACHMENT_EXCLUDE_FROM_ARTICULATION = True
    
    # ── TEST DIAGNOSTICO: MASS RATIO (Step 4) ──
    # Pushing density to 20000.0 to make pedicel mass ~10g, achieving a 5:1 mass ratio 
    # with the 50g tomato. This is highly optimal for PhysX.
    TrussPhysicsConfig.PLANT_DENSITY = 20000.0  
    
    # ── 2. MASS RATIO STABILIZATION ──
    # The default pedicel mass is ~0.3g for a 1mm radius pedicel.
    # Attaching a ~130g tomato (excluded from articulation) to a 0.3g articulation link 
    # causes the PhysX TGS solver to explode due to the extreme mass ratio (~400:1).
    # Using thicker branches (e.g. 1cm radius) naturally raises the mass to ~50g,
    # stabilizing the solver without needing artificial density inflation!

    branches = [
        # Rigid Trunk
        {
            "id": "trunk", 
            "parent": None, 
            "attach_link": None,
            "n_links": 3, 
            "radius": 0.05, 
            "height": 0.15,
            "tilt": 0.0, 
            "rot": 0.0, 
            "joint_type": "fixed",
        },
        # Rachis (thin, realistic from day 50 CSV)
        {
            "id": "rachis_1", 
            "parent": "trunk", 
            "attach_link": 2,
            "n_links": 4, 
            "radius": 0.0015,  # Pre-scale (scaled to 3mm radius)
            "height": 0.045,
            "tilt": 45.0, 
            "rot": 90.0, 
            "physics_profile": "truss",
            # -- Hardcoded physics to compensate for extreme thinness --
            "young_modulus": 50.0e7,
            "damping_ratio": 5.0,
            "drive_stiffness_scale": 50.0,
        },
        # Pedicel 1 (attached to rachis link 2)
        {
            "id": "pedicel_1", 
            "parent": "rachis_1", 
            "attach_link": 2,
            "n_links": 2,
            "radius": 0.001,  # Pre-scale (scaled to 2mm radius)
            "height": 0.02,
            "tilt": 45.0, 
            "rot": 90.0, 
            "physics_profile": "truss",
            # -- Hardcoded physics to compensate for extreme thinness --
            "young_modulus": 50.0e7,
            "damping_ratio": 5.0,
            "drive_stiffness_scale": 50.0,
            "kind": "pedicel"
        },
        # Pedicel 2 (attached to rachis link 4)
        {
            "id": "pedicel_2", 
            "parent": "rachis_1", 
            "attach_link": 4,
            "n_links": 2,
            "radius": 0.001,  # Pre-scale (scaled to 2mm radius)
            "height": 0.02,
            "tilt": 45.0, 
            "rot": -90.0, 
            "physics_profile": "truss",
            # -- Hardcoded physics to compensate for extreme thinness --
            "young_modulus": 50.0e7,
            "damping_ratio": 5.0,
            "drive_stiffness_scale": 50.0,
            "kind": "pedicel"
        }
    ]

    terminal_bodies = [
        {
            "id": "tomato_1", 
            "shape": "sphere",
            "parent_branch_id": "pedicel_1",
            "mass": 0.05,  # ~50g (realistic for day 50 early fruit)
            "radius": 0.013, # Pre-scale from day 50 CSV
            "maturation": 0.6,
        },
        {
            "id": "tomato_2", 
            "shape": "sphere",
            "parent_branch_id": "pedicel_2",
            "mass": 0.02,  # ~20g
            "radius": 0.010, # Pre-scale from day 50 CSV
            "maturation": 0.1,
        }
    ]

    output_path = os.path.join(project_root, "data", "usd_models", "test_stable_truss.usda")
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    print("\n--- Building V2 Detachable Truss ---")
    OutputConfig.STEP_1_VERBOSE = True
    
    stage, stem_path = build_stage(
        output_path,
        branches=branches,
        terminal_bodies=terminal_bodies
    )

    setup_physx(stage, stem_path)
    stage.GetRootLayer().Save()

    print(f"\n[OK] USD saved → {output_path}")

    # Launch Simulation
    omni.usd.get_context().open_stage(output_path)
    w = World(stage_units_in_meters=1.0)
    w.reset()

    print("\n[INFO] Simulation running — close window to exit.")
    print("      To test detachment, use Shift+Click to drag the tomatoes.")

    while simulation_app.is_running():
        w.step(render=True)

    simulation_app.close()

if __name__ == "__main__":
    main()
