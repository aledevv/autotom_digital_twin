"""
physx_utils.py

Funzioni pure per iniettare la configurazione PhysX in uno stage USD
gia' costruito. NON contiene SimulationApp: va chiamato da un entry-point
che ha gia' fatto il bootstrap (es. main_v2.py).
"""

from pxr import UsdPhysics, PhysxSchema, Gf


def apply_physx_scene_settings(stage) -> None:
    """Crea/configura la PhysicsScene con parametri PhysX adatti a drive rigidi."""
    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx_scene_api = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx_scene_api.CreateSolverTypeAttr().Set("TGS")
    physx_scene_api.CreateTimeStepsPerSecondAttr().Set(120)
    physx_scene_api.CreateEnableCCDAttr().Set(True)
    physx_scene_api.CreateEnableStabilizationAttr().Set(True)
    physx_scene_api.CreateEnableGPUDynamicsAttr().Set(True)
    physx_scene_api.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(stage, stem_path: str) -> None:
    """Configura le iterazioni del solver sull'ArticulationRoot per stabilita' con drive rigidi."""
    stem_prim = stage.GetPrimAtPath(stem_path)
    physx_art_api = PhysxSchema.PhysxArticulationAPI.Apply(stem_prim)
    physx_art_api.CreateSolverPositionIterationCountAttr().Set(240)
    physx_art_api.CreateSolverVelocityIterationCountAttr().Set(16)
    physx_art_api.CreateEnabledSelfCollisionsAttr().Set(False)
    physx_art_api.CreateSleepThresholdAttr().Set(0.0)