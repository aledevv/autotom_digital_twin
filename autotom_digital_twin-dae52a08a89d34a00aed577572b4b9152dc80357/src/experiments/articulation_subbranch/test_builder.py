"""
test_builder.py

Minimal test: trunk only (root + 2 internodes).
Once this works we add branches / subbranches.
"""

import os, sys
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf
import omni.usd
from isaacsim.core.api import World

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
sys.path.insert(0, project_root)

from src.plant_model.plant_builder import PlantBuilder

OUTPUT = os.path.join(project_root, "data", "usd_models",
                      "test_builder_articulation.usda")

# ── helpers (PhysX scene / articulation config) ──────────────────────────
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

# ── main ─────────────────────────────────────────────────────────────────
def main():
    if os.path.exists(OUTPUT):
        os.remove(OUTPUT)

    stage = Usd.Stage.CreateNew(OUTPUT)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # ── Build plant ──────────────────────────────────────────────────
    builder = PlantBuilder(stage, "/World/Stem")

    # Trunk: 3 internodes
    t1 = builder.create_root("Trunk_01", radius=0.10, length=0.5)
    t2 = builder.add_internode(t1, "Trunk_02", radius=0.10, length=0.5)
    t3 = builder.add_internode(t2, "Trunk_03", radius=0.09, length=0.5)

    # Branch off the second trunk segment
    b1 = builder.add_lateral_branch(t2, "Branch_01",
                                    radius=0.04, length=0.3,
                                    z_offset_ratio=0.8,
                                    tilt_angle=45,
                                    rot_around_parent=90)
    # Extend that branch
    b2 = builder.add_internode(b1, "Branch_02", radius=0.04, length=0.25)

    # ── PhysX config & save ──────────────────────────────────────────
    setup_physx(stage, "/World/Stem")
    stage.GetRootLayer().Save()
    print(f"\n[OK] USD saved → {OUTPUT}")

    # ── Open in Isaac Sim ────────────────────────────────────────────
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
