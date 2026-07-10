"""
physx_utils.py

Pure functions to inject PhysX configuration into an already-built USD stage.
No SimulationApp here — must be called from an entry-point that already
bootstrapped the Isaac Sim runtime (e.g. mainV2.py).
"""

from pxr import UsdPhysics, PhysxSchema, Gf


def apply_physx_scene_settings(stage) -> None:
    """Creates and configures the PhysicsScene for rigid drive stability."""
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    scene_api = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    scene_api.CreateSolverTypeAttr().Set("TGS")
    scene_api.CreateTimeStepsPerSecondAttr().Set(120)
    scene_api.CreateEnableCCDAttr().Set(True)
    scene_api.CreateEnableStabilizationAttr().Set(True)
    scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    scene_api.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    """Configures solver iteration counts on the ArticulationRoot for stability."""
    stem_prim = stage.GetPrimAtPath(stem_path)
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    art_api.CreateSolverPositionIterationCountAttr().Set(240)
    art_api.CreateSolverVelocityIterationCountAttr().Set(16)
    art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    art_api.CreateSleepThresholdAttr().Set(0.0)