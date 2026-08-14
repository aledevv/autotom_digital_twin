"""
demo_fixed_joint.py - Demo for fixed joint attachment to cylinder tips

Creates a USD with cylinders and spheres attached with fixed joints.
Tests the create_fixed_joint_to_tip function.

Run with:
    uv run python src/exporterV2/demos/demo_fixed_joint.py
"""

import os
import sys
from pathlib import Path

# Add parent directories to path for imports
script_dir = Path(__file__).parent
src_dir = script_dir.parents[1]
sys.path.insert(0, str(src_dir))

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
from exporterV2.core.usd.geometry import create_rigid_segment, create_sphere_rigid_body
from exporterV2.core.usd.joints import create_fixed_joint_to_tip, anchor_link_to_world


def create_demo_usd():
    """Create demo USD with cylinders and spheres attached via fixed joints."""
    
    # Output path
    project_root = script_dir.parents[2]
    output_dir = project_root / "data" / "usd_models"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "fixed_joint_demo.usda"
    
    print("\n" + "="*80)
    print("  Fixed Joint Attachment Demo - Creating USD")
    print("="*80)
    
    # Create stage
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Create World xform with ArticulationRootAPI
    world_path = "/World"
    world_prim = UsdGeom.Xform.Define(stage, world_path)
    stage.SetDefaultPrim(world_prim.GetPrim())
    
    # Create Stem container with ArticulationRootAPI
    stem_path = f"{world_path}/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())
    
    print(f"\nCreating test structures...")
    
    # Test 1: Single vertical pedicel with sphere at tip
    print(f"\n[Test 1] Vertical pedicel + sphere")
    pedicel1_height = 0.15  # 15cm
    pedicel1_radius = 0.005  # 5mm
    sphere1_radius = 0.03   # 3cm
    
    # Calculate mass (cylinder: rho * pi * r^2 * h)
    rho = 1000.0  # kg/m^3
    import math
    pedicel1_mass = rho * math.pi * (pedicel1_radius ** 2) * pedicel1_height
    sphere1_mass = (4.0/3.0) * math.pi * (sphere1_radius ** 3) * rho
    
    pedicel1_path = create_rigid_segment(
        stage, stem_path, "Pedicel1",
        pedicel1_radius, pedicel1_height,
        Gf.Vec3d(0.0, 0.0, 0.0),  # Base at origin
        pedicel1_mass
    )
    
    # Anchor pedicel to world
    anchor_link_to_world(stage, pedicel1_path)
    
    # Create sphere at tip (sphere center = pedicel_height + sphere_radius)
    sphere1_pos = Gf.Vec3d(0.0, 0.0, pedicel1_height + sphere1_radius)
    sphere1_path = create_sphere_rigid_body(
        stage, stem_path, "Sphere1",
        sphere1_radius, sphere1_pos, sphere1_mass
    )
    
    # Attach sphere to pedicel tip with fixed joint
    create_fixed_joint_to_tip(
        stage, pedicel1_path, sphere1_path,
        parent_height=pedicel1_height,
        child_offset=sphere1_radius  # Sphere center offset from tip
    )
    
    print(f"  ✓ Pedicel: h={pedicel1_height}m, r={pedicel1_radius}m")
    print(f"  ✓ Sphere: r={sphere1_radius}m at z={pedicel1_height + sphere1_radius}m")
    print(f"  ✓ Fixed joint created")
    
    # Test 2: Tilted pedicel with sphere
    print(f"\n[Test 2] Tilted pedicel + sphere")
    pedicel2_height = 0.12
    pedicel2_radius = 0.004
    sphere2_radius = 0.025
    
    pedicel2_mass = rho * math.pi * (pedicel2_radius ** 2) * pedicel2_height
    sphere2_mass = (4.0/3.0) * math.pi * (sphere2_radius ** 3) * rho
    
    # Tilted 45° from vertical
    tilt_rad = math.radians(45.0)
    quat = Gf.Quatf(math.cos(tilt_rad/2), math.sin(tilt_rad/2), 0, 0)
    
    # Position pedicel base offset from first one
    pedicel2_base = Gf.Vec3d(0.15, 0.0, 0.0)
    pedicel2_path = create_rigid_segment(
        stage, stem_path, "Pedicel2",
        pedicel2_radius, pedicel2_height,
        pedicel2_base,
        pedicel2_mass,
        orientation=quat
    )
    
    anchor_link_to_world(stage, pedicel2_path)
    
    # Calculate sphere position (tip of tilted pedicel)
    # Direction: rotated (0,0,1) by quat
    from pxr import Gf
    direction = Gf.Vec3d(0, 0, 1)
    rotation = Gf.Rotation(Gf.Quatd(quat))
    tilted_dir = rotation.TransformDir(direction)
    
    sphere2_pos = pedicel2_base + tilted_dir * (pedicel2_height + sphere2_radius)
    sphere2_path = create_sphere_rigid_body(
        stage, stem_path, "Sphere2",
        sphere2_radius, sphere2_pos, sphere2_mass,
        orientation=quat
    )
    
    create_fixed_joint_to_tip(
        stage, pedicel2_path, sphere2_path,
        parent_height=pedicel2_height,
        child_offset=sphere2_radius
    )
    
    print(f"  ✓ Pedicel: h={pedicel2_height}m, r={pedicel2_radius}m, tilt=45°")
    print(f"  ✓ Sphere: r={sphere2_radius}m")
    print(f"  ✓ Fixed joint created")
    
    # Test 3: Multiple spheres at different sizes
    print(f"\n[Test 3] Multiple pedicels with varying sphere sizes")
    
    sphere_configs = [
        {"r": 0.02, "x": 0.3, "color": "small"},
        {"r": 0.035, "x": 0.45, "color": "large"},
        {"r": 0.015, "x": 0.6, "color": "tiny"},
    ]
    
    for i, config in enumerate(sphere_configs):
        pedicel_height = 0.10
        pedicel_radius = 0.003
        sphere_radius = config["r"]
        
        pedicel_mass = rho * math.pi * (pedicel_radius ** 2) * pedicel_height
        sphere_mass = (4.0/3.0) * math.pi * (sphere_radius ** 3) * rho
        
        pedicel_base = Gf.Vec3d(config["x"], 0.0, 0.0)
        pedicel_path = create_rigid_segment(
            stage, stem_path, f"Pedicel3_{i}",
            pedicel_radius, pedicel_height,
            pedicel_base, pedicel_mass
        )
        
        anchor_link_to_world(stage, pedicel_path)
        
        sphere_pos = pedicel_base + Gf.Vec3d(0, 0, pedicel_height + sphere_radius)
        sphere_path = create_sphere_rigid_body(
            stage, stem_path, f"Sphere3_{i}",
            sphere_radius, sphere_pos, sphere_mass
        )
        
        create_fixed_joint_to_tip(
            stage, pedicel_path, sphere_path,
            parent_height=pedicel_height,
            child_offset=sphere_radius
        )
        
        print(f"  ✓ Pedicel {i}: sphere r={sphere_radius}m ({config['color']})")
    
    # Add ground plane for reference
    print(f"\nCreating ground plane...")
    ground_path = f"{world_path}/GroundPlane"
    ground_xform = UsdGeom.Xform.Define(stage, ground_path)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    
    plane = UsdGeom.Mesh.Define(stage, f"{ground_path}/Mesh")
    plane.GetPointsAttr().Set([
        Gf.Vec3f(-0.5, -0.5, 0), Gf.Vec3f(1.0, -0.5, 0),
        Gf.Vec3f(1.0, 0.5, 0), Gf.Vec3f(-0.5, 0.5, 0)
    ])
    plane.GetFaceVertexCountsAttr().Set([4])
    plane.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    plane.GetSubdivisionSchemeAttr().Set("none")
    
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    print(f"  ✓ Ground plane")
    
    # Save stage
    stage.GetRootLayer().Save()
    
    print(f"\n✓ USD saved to: {output_path}")
    print("\n" + "="*80)
    print("  Demo Complete - Load in Isaac Sim to test")
    print("="*80)
    print(f"\nExpected behavior:")
    print(f"  - Multiple pedicels (cylinders) anchored to world")
    print(f"  - Spheres rigidly attached to pedicel tips with FixedJoint")
    print(f"  - Press PLAY: structure should remain rigid (no sphere movement relative to pedicels)")
    print(f"  - Spheres should NOT separate from pedicels under gravity")
    print(f"  - All joints visible in Physics Inspector as FixedJoint type")
    
    return str(output_path)


if __name__ == "__main__":
    try:
        output_path = create_demo_usd()
        sys.exit(0)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
