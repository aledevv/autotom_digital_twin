"""
compare_isaac_sim.py - Compare Baseline vs Thin Link Lock in Isaac Sim

Loads both USD files side-by-side in Isaac Sim for visual comparison.

Run with: ~/isaacsim/python.sh src/exporterV2/core/optimizations/tests/10_thin_link_lock/compare_isaac_sim.py
"""

import os
from omni.isaac.kit import SimulationApp

# Start Isaac Sim
simulation_app = SimulationApp({"headless": False})

import omni
from pxr import Gf, Usd, UsdGeom
import carb

def load_and_position_usd(stage, usd_path, x_offset=0.0):
    """
    Load a USD file and position it at x_offset.
    
    Args:
        stage: USD stage to load into
        usd_path: Path to USD file
        x_offset: X offset for positioning
    
    Returns:
        Prim path of loaded USD
    """
    filename = os.path.basename(usd_path).replace(".usda", "")
    prim_path = f"/World/{filename}"
    
    # Add reference to USD
    prim = stage.DefinePrim(prim_path)
    prim.GetReferences().AddReference(usd_path)
    
    # Apply transform
    xformable = UsdGeom.Xformable(prim)
    xformable.AddTranslateOp().Set(Gf.Vec3d(x_offset, 0.0, 0.0))
    
    carb.log_info(f"Loaded {filename} at x={x_offset}")
    return prim_path


def setup_camera(stage):
    """Setup camera to view both plants.

    Uses omni.kit.viewport.utility (Isaac Sim 4.x API).
    The SetActiveCamera kit command was removed in recent Isaac Sim versions.
    Camera setup is non-critical — the user can navigate manually if this fails.
    """
    camera_path = "/World/Camera"
    camera = UsdGeom.Camera.Define(stage, camera_path)

    # Position camera to see both plants side-by-side
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, -3.0, 1.5))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(20.0, 0.0, 0.0))

    # Activate camera via viewport utility (Isaac Sim 4.x)
    try:
        import omni.kit.viewport.utility as vpu
        viewport = vpu.get_active_viewport()
        if viewport is not None:
            viewport.set_active_camera(camera_path)
            carb.log_info(f"Active camera set to {camera_path}")
    except Exception as exc:
        carb.log_warn(f"Could not set active camera (non-critical): {exc}")
        carb.log_warn("Navigate to the camera manually in the Viewport menu.")


def add_ground_plane(stage):
    """Add ground plane for reference."""
    plane_path = "/World/GroundPlane"
    plane = UsdGeom.Mesh.Define(stage, plane_path)
    
    # Simple quad
    plane.CreatePointsAttr([
        Gf.Vec3f(-5, -5, 0),
        Gf.Vec3f(5, -5, 0),
        Gf.Vec3f(5, 5, 0),
        Gf.Vec3f(-5, 5, 0),
    ])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    
    # Add collision
    from pxr import UsdPhysics
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())


def main():
    """Main comparison script."""
    print("="*70)
    print("  Isaac Sim - Thin Link Lock Comparison")
    print("="*70)
    
    # Get USD paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    usd_dir = os.path.join(script_dir, "usd_output")
    
    baseline_path = os.path.join(usd_dir, "baseline.usda")
    locked_path = os.path.join(usd_dir, "thin_link_lock.usda")
    
    # Check files exist
    if not os.path.exists(baseline_path):
        carb.log_error(f"Baseline USD not found: {baseline_path}")
        carb.log_error("Run generate_comparison_usd.py first!")
        simulation_app.close()
        return 1
    
    if not os.path.exists(locked_path):
        carb.log_error(f"Locked USD not found: {locked_path}")
        carb.log_error("Run generate_comparison_usd.py first!")
        simulation_app.close()
        return 1
    
    # Create stage
    print("\n[Step 1] Creating stage...")
    stage = omni.usd.get_context().get_stage()
    
    # Setup physics scene
    print("[Step 2] Setting up physics...")
    from pxr import UsdPhysics, PhysxSchema
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    
    # PhysX settings
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx_scene.CreateEnableCCDAttr().Set(True)
    physx_scene.CreateEnableStabilizationAttr().Set(True)
    physx_scene.CreateEnableGPUDynamicsAttr().Set(False)
    physx_scene.CreateBroadphaseTypeAttr().Set("MBP")
    physx_scene.CreateSolverTypeAttr().Set("TGS")
    
    # Add ground
    print("[Step 3] Adding ground plane...")
    add_ground_plane(stage)
    
    # Load both USD files side-by-side
    print("[Step 4] Loading USD files...")
    baseline_prim = load_and_position_usd(stage, baseline_path, x_offset=-1.0)
    locked_prim = load_and_position_usd(stage, locked_path, x_offset=1.0)
    
    print(f"  ✓ Baseline at x=-1.0m")
    print(f"  ✓ Thin Link Lock at x=+1.0m")
    
    # Setup camera
    print("[Step 5] Setting up camera...")
    setup_camera(stage)
    
    # Instructions
    print("\n" + "="*70)
    print("  ✓ Comparison Scene Ready!")
    print("="*70)
    print("\nScene Layout:")
    print("  Left  (x=-1.0): Baseline - All links articulated (D6)")
    print("  Right (x=+1.0): Thin Link Lock - Thin links fixed (Fixed)")
    print("\nInstructions:")
    print("  1. Press PLAY button to start simulation")
    print("  2. Observe thin link stability:")
    print("     - Baseline: Thin links might wobble, oscillate or jitter.")
    print("     - Thin Link Lock: Thin links remain rigid and stable.")
    print("  3. Apply external force (optional):")
    print("     - Select a thin link")
    print("     - Use Force/Torque tools")
    print("  4. Compare stability and performance")
    print("\nExpected Results:")
    print("  • Visual: Geometrically identical")
    print("  • Physics: Thin link lock more stable, no physics solver issues on thin links.")
    print("  • Performance: Thin link lock slightly faster (fewer active DOF)")
    
    print("\nPress Ctrl+C in terminal to exit when done.")
    
    # Keep simulation running
    try:
        while simulation_app.is_running():
            simulation_app.update()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    simulation_app.close()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
