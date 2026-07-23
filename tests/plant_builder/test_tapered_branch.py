import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(os.path.dirname(SCRIPT_DIR))
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics, Gf, PhysxSchema
from isaacsim.core.api import World

from plant_model.plant_builder import PlantBuilder

def setup_physx(stage, stem_path):
    sc = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    sc.CreateGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    sc.CreateGravityMagnitudeAttr().Set(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(sc.GetPrim())
    px.CreateSolverTypeAttr().Set("TGS")
    px.CreateTimeStepsPerSecondAttr().Set(120)
    px.CreateEnableStabilizationAttr().Set(True)

    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(stem_path))
    art.CreateSolverPositionIterationCountAttr().Set(64)
    art.CreateSolverVelocityIterationCountAttr().Set(8)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)

def create_tapered_test(out_path, test_type="all"):
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    stem_path = "/World/Stem"
    builder = PlantBuilder(stage, stem_path, global_scale=1.0)
    builder.create_root("Trunk_00", radius=0.1, length=1.0, mass=50.0)
    
    if test_type.startswith("stress_"):
        num_seg = int(test_type.split("_")[1])
        L = 0.5 / num_seg
        D = 2 * 0.05
        print(f"\n--- Stress Test: {num_seg} segments, L={L:.4f}, D={D:.4f}, L/D={L/D:.4f} ---")
        branch_id = builder.add_branch(
            parent_id="Trunk_00", base_id=f"Stress_{num_seg}",
            total_length=0.5, start_radius=0.05, end_radius=0.05,
            num_segments=num_seg, z_offset_ratio=0.5, tilt_angle=90.0,
            rot_around_parent=0.0, stiffness_base=50000.0, stiffness_tip=50000.0,
            max_bend_angle=15.0
        )
        builder.add_fruit(branch_id, f"Fruit_Stress_{num_seg}", fruit_radius=0.05, mass=0.2)
        
    elif test_type == "smooth":
        print("\n--- Visual Smoothness Test (25 seg) ---")
        smooth_id = builder.add_branch(
            parent_id="Trunk_00", base_id="SmoothBranch",
            total_length=1.0, start_radius=0.04, end_radius=0.01,
            num_segments=10, z_offset_ratio=0.9, tilt_angle=45.0,
            rot_around_parent=180.0, stiffness_base=20000.0, stiffness_tip=5000.0,
            max_bend_angle=10.0
        )
        builder.add_fruit(smooth_id, "Fruit_Smooth", fruit_radius=0.08, mass=0.3)
        
    setup_physx(stage, stem_path)
    stage.GetRootLayer().Save()
    print(f"\n[OK] Saved {out_path}")

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--test", default="smooth", help="Test type: smooth, stress_5, stress_10, stress_20, stress_50")
    args = parser.parse_args()
    
    out_path = os.path.join(PROJECT_ROOT, "output", f"test_{args.test}.usda")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    if os.path.exists(out_path):
        os.remove(out_path)
        
    create_tapered_test(out_path, args.test)
    omni.usd.get_context().open_stage(out_path)
    
    world = World(stage_units_in_meters=1.0)
    world.reset()
    
    # Run for 200 frames to see if it explodes
    for i in range(200):
        world.step(render=False)
        
    print(f"\n[OK] Test {args.test} completed 200 frames without crashing app.")
    simulation_app.close()

if __name__ == "__main__":
    main()
