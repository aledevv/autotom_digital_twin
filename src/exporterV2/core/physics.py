"""
physics.py - PhysX Configuration

Shared PhysX scene and articulation settings for Isaac Sim simulations.
"""

from pxr import UsdPhysics, PhysxSchema, Gf

from .tree_config import PhysicsRuntimeConfig


def apply_physx_scene_settings(
    stage,
    physics_hz: float | None = None,
    enable_gpu_dynamics: bool | None = None,
) -> None:
    """
    Configure PhysicsScene for stiff articulation drives.
    
    Settings optimized for plant stems with high stiffness joints.
    """
    if physics_hz is None:
        physics_hz = PhysicsRuntimeConfig.PHYSICS_HZ
    if enable_gpu_dynamics is None:
        enable_gpu_dynamics = PhysicsRuntimeConfig.ENABLE_GPU_DYNAMICS
    if physics_hz <= 0:
        raise ValueError("physics_hz must be positive")

    scene_path = "/World/PhysicsScene"
    usd_scene = UsdPhysics.Scene.Define(stage, scene_path)
    usd_scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    usd_scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(usd_scene.GetPrim())
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(int(physics_hz))
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateEnableGPUDynamicsAttr().Set(enable_gpu_dynamics)
    physx.CreateBroadphaseTypeAttr().Set("MBP")


def apply_physx_articulation_settings(
    stage,
    stem_path: str,
    solver_position_iterations: int | None = None,
    solver_velocity_iterations: int | None = None,
) -> None:
    """
    Configure articulation iteration counts for TGS.

    The cantilever benchmark shows that the previous 255/32 setting can make
    low-load D6 joints timestep-dependent or fully locked at high update rates.
    """
    if solver_position_iterations is None:
        solver_position_iterations = PhysicsRuntimeConfig.SOLVER_POSITION_ITERATIONS
    if solver_velocity_iterations is None:
        solver_velocity_iterations = PhysicsRuntimeConfig.SOLVER_VELOCITY_ITERATIONS
    if not 1 <= solver_position_iterations <= 255:
        raise ValueError("solver_position_iterations must be in [1, 255]")
    if not 1 <= solver_velocity_iterations <= 255:
        raise ValueError("solver_velocity_iterations must be in [1, 255]")
    prim = stage.GetPrimAtPath(stem_path)
    art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    art.CreateSolverPositionIterationCountAttr().Set(solver_position_iterations)
    art.CreateSolverVelocityIterationCountAttr().Set(solver_velocity_iterations)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)


def apply_physx_rigid_body_solver_settings(
    stage,
    body_path: str,
    solver_position_iterations: int | None = None,
    solver_velocity_iterations: int | None = None,
) -> None:
    """Configure solver precision for a rigid body outside the articulation."""
    if solver_position_iterations is None:
        solver_position_iterations = (
            PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_POSITION_ITERATIONS
        )
    if solver_velocity_iterations is None:
        solver_velocity_iterations = (
            PhysicsRuntimeConfig.TERMINAL_BODY_SOLVER_VELOCITY_ITERATIONS
        )
    if not 1 <= solver_position_iterations <= 256:
        raise ValueError("solver_position_iterations must be in [1, 256]")
    if not 1 <= solver_velocity_iterations <= 255:
        raise ValueError("solver_velocity_iterations must be in [1, 255]")

    prim = stage.GetPrimAtPath(body_path)
    if not prim or not prim.IsValid():
        raise ValueError(f"Rigid body prim does not exist: {body_path}")

    rigid_body = PhysxSchema.PhysxRigidBodyAPI(prim)
    if not rigid_body:
        rigid_body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rigid_body.CreateSolverPositionIterationCountAttr().Set(
        solver_position_iterations
    )
    rigid_body.CreateSolverVelocityIterationCountAttr().Set(
        solver_velocity_iterations
    )
