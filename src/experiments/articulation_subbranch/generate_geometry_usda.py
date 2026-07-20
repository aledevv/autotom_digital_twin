"""
generate_geometry_usda.py

Generates a tree structure (main stem + sub-branches) using OpenUSD Xforms and Cylinders.
Physics APIs are intentionally omitted. This script acts as a generalized geometric variant
of the articulation generator, with dynamic segment scaling up to a maximum segment count.
"""

import os
import random
import math
from pxr import Usd, UsdGeom, Gf

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================

GLOBAL_SCALE = 1.0

class Config:
    """Geometric settings for the generation."""
    MAX_STEM_SEGMENTS = 50
    MAX_BRANCH_SEGMENTS = 20
    
    BASE_SEGMENT_LENGTH = 0.20 * GLOBAL_SCALE
    STEM_RADIUS = 0.10 * GLOBAL_SCALE
    BRANCH_RADIUS = 0.04 * GLOBAL_SCALE
    GAP = 0.001 * GLOBAL_SCALE

def get_output_usd_path() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "generated_geometry.usda")

def setup_base_stage(path: str) -> tuple:
    stage = Usd.Stage.CreateNew(path)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    stem_path = "/World/Stem"
    UsdGeom.Xform.Define(stage, stem_path)
    return stage, stem_path

def compute_target_length(total_length: float, max_segments: int, default_length: float) -> float:
    """Calculates target segment length so that max_segments is never exceeded."""
    if total_length <= 0:
        return default_length
    return max(total_length / max_segments, default_length)

def calculate_segments(total_length: float, target_length: float) -> int:
    """Returns number of segments needed."""
    if total_length <= 0:
        return 1
    return max(1, round(total_length / target_length))

def create_visual_segment(stage: Usd.Stage, parent_path: str, name: str,
                          radius: float, height: float, 
                          world_pos: Gf.Vec3d, orientation: Gf.Quatf) -> str:
    """Creates a visual-only Xform with a nested Cylinder."""
    link_path = f"{parent_path}/{name}"
    
    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(world_pos)
    xform.AddOrientOp().Set(orientation)
    
    # Define visual cylinder
    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(radius)
    cylinder.GetHeightAttr().Set(height)
    cylinder.GetAxisAttr().Set("Z")
    
    # Offset cylinder so its base aligns with the Xform origin
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
    
    return link_path

def generate_stem(stage: Usd.Stage, stem_path: str, total_length: float) -> list[dict]:
    """Generates main stem segments based on randomized length."""
    target_length = compute_target_length(total_length, Config.MAX_STEM_SEGMENTS, Config.BASE_SEGMENT_LENGTH)
    n_segments = calculate_segments(total_length, target_length)
    
    # Adjust segment height so exactly n_segments cover total_length
    segment_height = (total_length - (n_segments - 1) * Config.GAP) / n_segments if n_segments > 1 else total_length
    
    print(f"[INFO] Building stem with {n_segments} segments (Length: {total_length:.2f}m, Target Seg Len: {target_length:.2f}m)")
    
    segments_info = []
    
    for i in range(n_segments):
        seg_name = f"Seg_{i+1:02d}"
        base_z = i * (segment_height + Config.GAP)
        
        pos = Gf.Vec3d(0.0, 0.0, base_z)
        rot = Gf.Quatf(1.0, 0.0, 0.0, 0.0) # Identity
        
        link_path = create_visual_segment(stage, stem_path, seg_name, Config.STEM_RADIUS, segment_height, pos, rot)
        
        segments_info.append({
            "path": link_path,
            "index": i + 1,
            "base_pos": pos,
            "height": segment_height
        })
        
    return segments_info

def generate_branch(stage: Usd.Stage, stem_path: str, parent_seg: dict, branch_name: str, 
                    total_length: float, tilt_angle_deg: float, rot_around_trunk_deg: float,
                    z_offset_ratio: float = 0.5):
    """Generates branch segments based on randomized length, attached to a specific stem segment."""
    target_length = compute_target_length(total_length, Config.MAX_BRANCH_SEGMENTS, Config.BASE_SEGMENT_LENGTH)
    n_segments = calculate_segments(total_length, target_length)
    
    segment_height = (total_length - (n_segments - 1) * Config.GAP) / n_segments if n_segments > 1 else total_length
    
    print(f"[INFO] Attaching {branch_name} to {parent_seg['path']} with {n_segments} segments (Length: {total_length:.2f}m)")
    
    branch_base_path = f"{stem_path}/Branches"
    if not stage.GetPrimAtPath(branch_base_path):
        UsdGeom.Xform.Define(stage, branch_base_path)
    
    branch_path = f"{branch_base_path}/{branch_name}"
    UsdGeom.Xform.Define(stage, branch_path)
    
    # Calculate attachment world position
    parent_pos = parent_seg["base_pos"]
    
    # Base attachment logic (adapted from original)
    total_distance = Config.STEM_RADIUS / 2.0
    base_pos0 = Gf.Vec3d(0.0, total_distance, z_offset_ratio * parent_seg["height"])
    
    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot_around_trunk_deg)
    rotated_pos0 = rot_z.TransformDir(base_pos0)
    
    rot_total = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_angle_deg) * rot_z
    branch_world_base_pos = parent_pos + rotated_pos0
    
    for i in range(n_segments):
        seg_name = f"Seg_{i+1:02d}"
        
        # Local distance along branch direction
        z_distance = i * (segment_height + Config.GAP)
        
        link_world_pos = branch_world_base_pos + rot_total.TransformDir(Gf.Vec3d(0.0, 0.0, z_distance))
        
        create_visual_segment(
            stage, branch_path, seg_name, Config.BRANCH_RADIUS, segment_height, 
            link_world_pos, Gf.Quatf(rot_total.GetQuat())
        )

def check_collision(candidate, existing, radius, height, min_dist):
    cx = radius * math.cos(math.radians(candidate[1]))
    cy = radius * math.sin(math.radians(candidate[1]))
    cz = candidate[0] * height
    
    for ext in existing:
        ex = radius * math.cos(math.radians(ext[1]))
        ey = radius * math.sin(math.radians(ext[1]))
        ez = ext[0] * height
        dist = math.sqrt((cx - ex)**2 + (cy - ey)**2 + (cz - ez)**2)
        if dist < min_dist:
            return False
    return True

def main():
    output_path = get_output_usd_path()
    # Remove file if exists so CreateNew doesn't fail
    if os.path.exists(output_path):
        os.remove(output_path)
        
    stage, stem_path = setup_base_stage(output_path)
    
    # Randomize stem length (e.g. between 1 and 15 meters)
    total_stem_length = random.uniform(1.0, 15.0)
    stem_segments = generate_stem(stage, stem_path, total_stem_length)
    
    # Define random branches
    n_branches = random.randint(20, 50)
    
    segment_branches = {}
    branch_count = 0
    MAX_PER_INTERNODE = 7
    MIN_DIST = Config.BRANCH_RADIUS * 2.5
    
    for _ in range(n_branches):
        if len(stem_segments) > 1:
            parent_idx = random.randint(1, len(stem_segments) - 1)
        else:
            parent_idx = 0
            
        if parent_idx not in segment_branches:
            segment_branches[parent_idx] = []
            
        if len(segment_branches[parent_idx]) >= MAX_PER_INTERNODE:
            continue
            
        parent_seg = stem_segments[parent_idx]
        
        valid_spawn = False
        attempts = 0
        while not valid_spawn and attempts < 20:
            z_ratio = random.uniform(0.1, 0.9)
            rot = random.uniform(0.0, 360.0)
            if check_collision((z_ratio, rot), segment_branches[parent_idx], Config.STEM_RADIUS, parent_seg["height"], MIN_DIST):
                valid_spawn = True
                segment_branches[parent_idx].append((z_ratio, rot))
            attempts += 1
            
        if not valid_spawn:
            continue
            
        branch_length = random.uniform(0.5, 5.0)
        tilt = random.uniform(30.0, 75.0)
        
        branch_count += 1
        generate_branch(
            stage=stage,
            stem_path=stem_path,
            parent_seg=parent_seg,
            branch_name=f"Branch_{branch_count:02d}",
            total_length=branch_length,
            tilt_angle_deg=tilt,
            rot_around_trunk_deg=rot,
            z_offset_ratio=z_ratio
        )
        
    stage.GetRootLayer().Save()
    print(f"[OK] Geometry USD saved to {output_path}")

if __name__ == "__main__":
    main()
