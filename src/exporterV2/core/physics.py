"""
physics.py - PhysX Configuration

Shared PhysX scene and articulation settings for Isaac Sim simulations.
"""

from pxr import UsdPhysics, PhysxSchema, Gf


def apply_physx_scene_settings(stage) -> None:
    """
    Configure PhysicsScene for stiff articulation drives.
    
    Settings optimized for plant stems with high stiffness joints.
    """
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(480)
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateEnableGPUDynamicsAttr().Set(True)
    physx.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    """
    Configure articulation iteration counts for mixed stiffness levels.
    
    Higher iteration counts needed for stable simulation of stiff joints.
    """
    prim = stage.GetPrimAtPath(stem_path)
    art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    art.CreateSolverPositionIterationCountAttr().Set(255)
    art.CreateSolverVelocityIterationCountAttr().Set(32)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)
