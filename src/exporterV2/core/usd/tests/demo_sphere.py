"""
demo_sphere.py - Demo script for sphere geometry

Creates a simple USD with spheres to test in Isaac Sim.

Run with:
    python -m exporterV2.core.usd.tests.demo_sphere
"""

import os
import sys
from pathlib import Path

# Add parent directories to path for imports
script_dir = Path(__file__).parent
core_dir = script_dir.parent
exporterV2_dir = core_dir.parent
src_dir = exporterV2_dir.parent
sys.path.insert(0, str(src_dir))

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
from exporterV2.core.usd.geometry import create_sphere_rigid_body


def create_demo_usd():
    """Create demo USD with spheres for Isaac Sim testing."""
    
    # Output path
    project_root = src_dir.parent
    output_dir = project_root / "data" / "usd_models"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "sphere_demo.usda"
    
    print("\n" + "="*80)
    print("  Sphere Geometry Demo - Creating USD")
    print("="*80)
    
    # Create stage
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    
    # Create World xform
    world_path = "/World"
    world_prim = UsdGeom.Xform.Define(stage, world_path)
    stage.SetDefaultPrim(world_prim.GetPrim())
    
    # Create Stem container
    stem_path = f"{world_path}/Stem"
    UsdGeom.Xform.Define(stage, stem_path)
    
    print(f"\nCreating spheres...")
    
    # Create multiple spheres at different positions
    spheres = [
        {"name": "Sphere_Small", "radius": 0.02, "pos": (0.0, 0.0, 0.5), "mass": 0.01},
        {"name": "Sphere_Medium", "radius": 0.04, "pos": (0.1, 0.0, 0.5), "mass": 0.05},
        {"name": "Sphere_Large", "radius": 0.06, "pos": (0.2, 0.0, 0.5), "mass": 0.15},
        {"name": "Sphere_Elevated", "radius": 0.03, "pos": (0.0, 0.1, 1.0), "mass": 0.03},
    ]
    
    for sphere_def in spheres:
        sphere_path = create_sphere_rigid_body(
            stage,
            stem_path,
            sphere_def["name"],
            sphere_def["radius"],
            Gf.Vec3d(*sphere_def["pos"]),
            sphere_def["mass"]
        )
        print(f"  ✓ {sphere_def['name']}: r={sphere_def['radius']}m, "
              f"pos={sphere_def['pos']}, mass={sphere_def['mass']}kg")
    
    # Add a ground plane for reference
    print(f"\nCreating ground plane...")
    ground_path = f"{world_path}/GroundPlane"
    ground_xform = UsdGeom.Xform.Define(stage, ground_path)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
    
    plane = UsdGeom.Mesh.Define(stage, f"{ground_path}/Mesh")
    # Simple 2x2m plane
    plane.GetPointsAttr().Set([
        Gf.Vec3f(-1, -1, 0), Gf.Vec3f(1, -1, 0),
        Gf.Vec3f(1, 1, 0), Gf.Vec3f(-1, 1, 0)
    ])
    plane.GetFaceVertexCountsAttr().Set([4])
    plane.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    plane.GetSubdivisionSchemeAttr().Set("none")
    
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    print(f"  ✓ Ground plane at z=-0.1m")
    
    # Save stage
    stage.GetRootLayer().Save()
    
    print(f"\n✓ USD saved to: {output_path}")
    print("\n" + "="*80)
    print("  Demo Complete - Load in Isaac Sim to test")
    print("="*80)
    print(f"\nLoad command:")
    print(f"  Open Isaac Sim → File → Open → {output_path}")
    print(f"\nExpected behavior:")
    print(f"  - 4 spheres visible at different heights")
    print(f"  - All spheres have collision enabled (visible in Physics Inspector)")
    print(f"  - Ground plane for reference")
    print(f"  - Press PLAY to see spheres fall with gravity")
    
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
