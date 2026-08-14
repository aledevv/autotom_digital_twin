"""
demo_complete_truss.py - Complete truss demo with tomatoes

Generates a USD with complete truss: rachis + pedicels + tomatoes (spheres).
Tests the full truss pipeline including fixed joint attachment of tomatoes.

Run with:
    uv run python src/exporterV2/demos/demo_complete_truss.py
"""

import os
import sys
from pathlib import Path

# Add parent directories to path
script_dir = Path(__file__).parent
src_dir = script_dir.parents[1]
sys.path.insert(0, str(src_dir))

from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
import math

from exporterV2.adapters.groimp_csv.truss_builder import truss_to_complete_config

# Import core USD functions
from exporterV2.core.usd.stage import setup_base_stage, build_chain
from exporterV2.core.usd.geometry import create_rigid_segment, create_sphere_rigid_body
from exporterV2.core.usd.joints import create_fixed_joint_to_tip, anchor_link_to_world
from exporterV2.core.usd.collision import validate_truss_geometry, print_geometry_validation_report
from exporterV2.core.tree_config import GLOBAL_SCALE, GAP, compute_mass, calculate_physics_params, scaled


def build_truss_with_tomatoes(
    stage,
    stem_path: str,
    truss_dict: dict,
    trunk_branch: dict,
    rank: int,
):
    """
    Build complete truss with rachis, pedicels, and tomatoes.
    
    Args:
        stage: USD stage
        stem_path: Path to Stem container
        truss_dict: Truss configuration dict
        trunk_branch: Trunk branch definition
        rank: Truss rank (for generation)
    
    Returns:
        List of tomato sphere paths
    """
    # Generate truss configuration
    branches, tomatoes = truss_to_complete_config(
        truss_dict,
        parent_trunk_id=trunk_branch["id"],
        rank=rank
    )
    
    print(f"\nBuilding truss with {len(branches)} branches and {len(tomatoes)} tomatoes...")
    
    # Build trunk first
    print(f"\n[Trunk] {trunk_branch['n_links']} links")
    trunk_paths, trunk_bases = build_chain(
        stage, stem_path, trunk_branch,
        Gf.Vec3d(0, 0, 0),  # Start at origin
        Gf.Vec3d(0, 0, 1),  # Vertical
        is_root=True,
        locked_joints=True  # Use locked joints for demo stability
    )
    
    # Build truss branches
    branch_registry = {trunk_branch["id"]: (trunk_paths, trunk_bases, Gf.Vec3d(0, 0, 1), Gf.Quatf(1, 0, 0, 0))}
    
    for b in branches:
        bid = b["id"]
        parent_id = b["parent"]
        attach_idx = b["attach_link"] - 1  # Convert to 0-based
        
        parent_paths, parent_bases, parent_axis, parent_orientation = branch_registry[parent_id]
        parent_def = trunk_branch if parent_id == trunk_branch["id"] else next(x for x in branches if x["id"] == parent_id)
        
        p_h_world = scaled(parent_def["height"])
        p_r_world = scaled(parent_def["radius"])
        gap_world = scaled(GAP)
        
        # Calculate branch orientation
        tilt_deg = b["tilt"]
        rot_deg = b["rot"]
        
        rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
        rot_tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_deg)
        branch_rot_in_parent_frame = rot_tilt * rot_z
        
        parent_rot = Gf.Rotation(Gf.Quatd(parent_orientation))
        combined = branch_rot_in_parent_frame * parent_rot
        chain_axis_raw = combined.TransformDir(Gf.Vec3d(0, 0, 1))
        chain_axis = Gf.Vec3d(*chain_axis_raw).GetNormalized()
        chain_orientation = Gf.Quatf(combined.GetQuat())
        
        # Calculate radial offset (coaxial branches have no offset)
        if tilt_deg == 0.0 and rot_deg == 0.0:
            radial_distance = 0.0
        else:
            radial_distance = p_r_world / 2.0
        
        base_offset_local = Gf.Vec3d(0.0, radial_distance, p_h_world + gap_world)
        rot_z_local = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_deg)
        offset_in_parent_frame = rot_z_local.TransformDir(base_offset_local)
        offset_in_world = parent_rot.TransformDir(offset_in_parent_frame)
        
        attach_base = parent_bases[attach_idx]
        start_pos = attach_base + offset_in_world
        
        local_pos0 = Gf.Vec3f(offset_in_parent_frame[0], offset_in_parent_frame[1], offset_in_parent_frame[2])
        local_rot0 = Gf.Quatf(branch_rot_in_parent_frame.GetQuat())
        
        link_paths, link_bases = build_chain(
            stage, stem_path, b,
            start_pos, chain_axis,
            is_root=False,
            parent_link_path=parent_paths[attach_idx],
            attachment_local_pos0=local_pos0,
            attachment_local_rot0=local_rot0,
            chain_orientation=chain_orientation,
            locked_joints=True
        )
        
        branch_registry[bid] = (link_paths, link_bases, chain_axis, chain_orientation)
        
        print(f"  ✓ {bid}: {len(link_paths)} links")
    
    # Attach tomatoes to pedicel tips
    print(f"\nAttaching {len(tomatoes)} tomatoes...")
    tomato_paths = []
    
    for tomato_def in tomatoes:
        pedicel_id = tomato_def["pedicel_id"]
        tomato_radius = scaled(tomato_def["radius"])
        tomato_mass = tomato_def["mass"]
        
        # Get pedicel info from registry
        pedicel_paths, pedicel_bases, pedicel_axis, pedicel_orientation = branch_registry[pedicel_id]
        pedicel_link_path = pedicel_paths[0]  # Pedicels have only 1 link
        pedicel_base = pedicel_bases[0]
        
        # Get pedicel branch definition
        pedicel_def = next(b for b in branches if b["id"] == pedicel_id)
        pedicel_height = scaled(pedicel_def["height"])
        
        # Calculate tomato position: pedicel tip + radius along pedicel axis
        tomato_pos = pedicel_base + pedicel_axis * (pedicel_height + tomato_radius)
        
        # Create sphere
        tomato_path = create_sphere_rigid_body(
            stage, stem_path,
            tomato_def["id"],
            tomato_radius,
            tomato_pos,
            tomato_mass,
            orientation=pedicel_orientation
        )
        
        # Attach with fixed joint
        create_fixed_joint_to_tip(
            stage,
            pedicel_link_path,
            tomato_path,
            parent_height=pedicel_height,
            child_offset=tomato_radius
        )
        
        tomato_paths.append(tomato_path)
        
        maturation = tomato_def["maturation"]
        state = "ripe" if maturation > 0.5 else "unripe"
        print(f"  ✓ {tomato_def['id']}: r={tomato_radius:.4f}m, {state}")
    
    # Validate geometry (check for intersections)
    print(f"\nValidating geometry...")
    warnings = validate_truss_geometry(
        tomatoes,
        branch_registry,
        branches,
        margin=0.001  # 1mm safety margin
    )
    
    if warnings:
        print_geometry_validation_report(warnings)
    else:
        print(f"  ✓ No geometry intersections detected")
    
    return tomato_paths


def create_demo_usd():
    """Create complete truss demo USD."""
    
    # Output path
    project_root = script_dir.parents[2]
    output_dir = project_root / "data" / "usd_models"
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "truss_complete_demo.usda"
    
    print("\n" + "="*80)
    print("  Complete Truss Demo - Creating USD")
    print("="*80)
    
    # Create stage
    stage, stem_path = setup_base_stage(str(output_path))
    
    # Define trunk (simple, 5 links)
    trunk_branch = {
        "id": "trunk",
        "parent": None,
        "attach_link": None,
        "n_links": 5,
        "radius": 0.01,  # 1cm radius (scaled to 2cm)
        "height": 0.20,  # 20cm per link (scaled to 40cm)
        "tilt": 0.0,
        "rot": 0.0,
    }
    
    # Define truss (attached at rank 3)
    truss_dict = {
        "rachis_length": 0.15,  # 15cm total
        "rachis_radius": 0.002,  # 2mm
        "n_fruits": 7,  # 7 tomatoes
        "pedicel_length": 0.01,  # 1cm pedicels
        "pedicel_angle": 90.0,  # Perpendicular
        "parent_rank": 3,
        "tilt_deg": 60.0,  # Drooping downward
        "azimuth_deg": 90.0,  # Side orientation
        # Tomato parameters (radii in meters, will be scaled by GLOBAL_SCALE=2)
        # 0.015m radius → 3cm world radius → 6cm diameter
        "tomato_radii": [0.012, 0.015, 0.014, 0.016, 0.013, 0.015, 0.017],  # 2.4-3.4cm world radius
        "maturation": [0.0, 0.0, 0.5, 1.0, 0.0, 1.0, 0.5],  # Mix of ripe/unripe
    }
    
    # Build truss
    tomato_paths = build_truss_with_tomatoes(
        stage,
        stem_path,
        truss_dict,
        trunk_branch,
        rank=3
    )
    
    # Add ground plane
    print(f"\nCreating ground plane...")
    world_path = "/World"
    ground_path = f"{world_path}/GroundPlane"
    ground_xform = UsdGeom.Xform.Define(stage, ground_path)
    ground_xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.1))
    
    plane = UsdGeom.Mesh.Define(stage, f"{ground_path}/Mesh")
    plane.GetPointsAttr().Set([
        Gf.Vec3f(-1, -1, 0), Gf.Vec3f(1, -1, 0),
        Gf.Vec3f(1, 1, 0), Gf.Vec3f(-1, 1, 0)
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
    print(f"\nStructure:")
    print(f"  - Trunk: 5 links (vertical)")
    print(f"  - Truss rachis: 4 links (drooping at 60°)")
    print(f"  - Pedicels: 6 lateral + 1 terminal (alternating ±90°)")
    print(f"  - Tomatoes: 7 spheres (varying sizes, mix of ripe/unripe)")
    print(f"\nExpected behavior:")
    print(f"  - Complete truss structure visible")
    print(f"  - Tomatoes rigidly attached to pedicel tips")
    print(f"  - Press PLAY: structure should remain stable")
    print(f"  - All fixed joints visible in Physics Inspector")
    
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
