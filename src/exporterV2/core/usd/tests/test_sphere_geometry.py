"""
test_sphere_geometry.py - Unit test for sphere geometry creation

Tests the create_sphere_rigid_body function to ensure correct USD structure.
"""

import os
import sys
from pathlib import Path
from pxr import Usd, UsdGeom, UsdPhysics, Gf

# Add parent directories to path
script_dir = Path(__file__).parent
core_dir = script_dir.parent
sys.path.insert(0, str(core_dir))

from geometry import create_sphere_rigid_body


def test_sphere_creation():
    """Test basic sphere creation with rigid body."""
    print("\n" + "="*80)
    print("TEST: Sphere Geometry Creation")
    print("="*80)
    
    # Create temporary USD stage
    output_dir = script_dir / "usd_output"
    output_dir.mkdir(exist_ok=True)
    output_path = output_dir / "test_sphere.usda"
    
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    # Create world xform
    world_path = "/World"
    UsdGeom.Xform.Define(stage, world_path)
    
    # Test sphere creation
    radius = 0.05  # 5cm
    world_pos = Gf.Vec3d(0.0, 0.0, 1.0)
    mass = 0.1  # 100g
    
    sphere_path = create_sphere_rigid_body(
        stage,
        world_path,
        "TestSphere",
        radius,
        world_pos,
        mass
    )
    
    print(f"\n✓ Created sphere at: {sphere_path}")
    
    # Verify sphere prim exists
    sphere_prim = stage.GetPrimAtPath(sphere_path)
    assert sphere_prim.IsValid(), "Sphere prim should exist"
    print(f"✓ Sphere prim valid: {sphere_path}")
    
    # Verify sphere geometry
    sphere_geom_path = f"{sphere_path}/Sphere"
    sphere_geom_prim = stage.GetPrimAtPath(sphere_geom_path)
    assert sphere_geom_prim.IsValid(), "Sphere geometry should exist"
    
    sphere_geom = UsdGeom.Sphere(sphere_geom_prim)
    actual_radius = sphere_geom.GetRadiusAttr().Get()
    assert abs(actual_radius - radius) < 1e-6, f"Radius mismatch: {actual_radius} vs {radius}"
    print(f"✓ Sphere radius correct: {actual_radius}m")
    
    # Verify RigidBodyAPI applied
    assert sphere_prim.HasAPI(UsdPhysics.RigidBodyAPI), "Should have RigidBodyAPI"
    print(f"✓ RigidBodyAPI applied")
    
    # Verify MassAPI and mass value
    assert sphere_prim.HasAPI(UsdPhysics.MassAPI), "Should have MassAPI"
    mass_api = UsdPhysics.MassAPI(sphere_prim)
    actual_mass = mass_api.GetMassAttr().Get()
    assert abs(actual_mass - mass) < 1e-6, f"Mass mismatch: {actual_mass} vs {mass}"
    print(f"✓ Mass correct: {actual_mass}kg")
    
    # Verify COM at center
    com = mass_api.GetCenterOfMassAttr().Get()
    assert com == Gf.Vec3f(0.0, 0.0, 0.0), f"COM should be at center, got {com}"
    print(f"✓ Center of mass at origin: {com}")
    
    # Verify CollisionAPI on geometry
    assert sphere_geom_prim.HasAPI(UsdPhysics.CollisionAPI), "Geometry should have CollisionAPI"
    print(f"✓ CollisionAPI applied to geometry")
    
    # Verify transform
    xform = UsdGeom.Xform(sphere_prim)
    translate_op = xform.GetOrderedXformOps()[0]
    actual_pos = translate_op.Get()
    assert actual_pos == world_pos, f"Position mismatch: {actual_pos} vs {world_pos}"
    print(f"✓ Position correct: {actual_pos}")
    
    # Save stage
    stage.GetRootLayer().Save()
    print(f"\n✓ USD saved to: {output_path}")
    
    print("\n" + "="*80)
    print("TEST PASSED: All sphere geometry checks successful")
    print("="*80)
    
    return True


def test_sphere_with_orientation():
    """Test sphere creation with custom orientation."""
    print("\n" + "="*80)
    print("TEST: Sphere with Orientation")
    print("="*80)
    
    output_dir = script_dir / "usd_output"
    output_path = output_dir / "test_sphere_orientation.usda"
    
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    world_path = "/World"
    UsdGeom.Xform.Define(stage, world_path)
    
    # Create sphere with 45° rotation around Z
    import math
    angle_rad = math.radians(45.0)
    quat = Gf.Quatf(math.cos(angle_rad/2), 0, 0, math.sin(angle_rad/2))
    
    sphere_path = create_sphere_rigid_body(
        stage,
        world_path,
        "RotatedSphere",
        0.03,
        Gf.Vec3d(0.5, 0.5, 0.5),
        0.05,
        orientation=quat
    )
    
    print(f"✓ Created rotated sphere at: {sphere_path}")
    
    # Verify orientation
    sphere_prim = stage.GetPrimAtPath(sphere_path)
    xform = UsdGeom.Xform(sphere_prim)
    xform_ops = xform.GetOrderedXformOps()
    
    # Should have translate and orient ops
    assert len(xform_ops) >= 2, "Should have translate and orient ops"
    
    orient_op = xform_ops[1]
    assert orient_op.GetOpType() == UsdGeom.XformOp.TypeOrient, "Second op should be orient"
    
    actual_quat = orient_op.Get()
    # Compare quaternion components (allow small numerical error)
    for i in range(4):
        assert abs(actual_quat[i] - quat[i]) < 1e-5, f"Quaternion component {i} mismatch"
    
    print(f"✓ Orientation correct: {actual_quat}")
    
    stage.GetRootLayer().Save()
    print(f"✓ USD saved to: {output_path}")
    
    print("\n" + "="*80)
    print("TEST PASSED: Sphere with orientation successful")
    print("="*80)
    
    return True


if __name__ == "__main__":
    try:
        test_sphere_creation()
        test_sphere_with_orientation()
        
        print("\n" + "="*80)
        print("ALL TESTS PASSED ✓")
        print("="*80)
        
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
