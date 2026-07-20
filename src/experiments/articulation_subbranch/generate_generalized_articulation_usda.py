"""
generate_generalized_articulation_usda.py

Generates an articulated trunk structure (main stem + sub-branches) using OpenUSD cylinders.
Includes RigidBody, Collision, ArticulationRoot, and flexible D6 Joints.
Supports randomized branch generation and segment limitation logic.
"""

import os
import random
import math
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

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

class PhysicsConfig:
    """Physical behavior settings for the trunk joints and bodies."""
    LINK_MASS = 1.0 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 20.0
    STIFFNESS = 50000.0 * (GLOBAL_SCALE ** 5)
    DAMPING = 5000.0 * (GLOBAL_SCALE ** 5)

class BranchPhysicsConfig:
    """Physical settings for secondary branches."""
    MASS = 0.2 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 30.0
    
    scale5 = GLOBAL_SCALE ** 5
    STIFFNESS_XY = 300.0 * scale5
    DAMPING_XY = 50.0 * scale5
    STIFFNESS_Z = 200.0 * scale5
    DAMPING_Z = 50.0 * scale5
    
    BASE_STIFFNESS = 184000.0 * scale5
    BASE_DAMPING = 5000.0 * scale5


# ==============================================================================
# PATH & STAGE HELPERS
# ==============================================================================

def get_output_usd_path() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "generated_generalized_articulation.usda")

def setup_base_stage(path: str) -> tuple:
    stage = Usd.Stage.CreateNew(path)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    stem_path = "/World/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    # Apply Articulation Root
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())
    return stage, stem_path

# ==============================================================================
# MATH & GEOMETRY HELPERS
# ==============================================================================

def compute_target_length(total_length: float, max_segments: int, default_length: float) -> float:
    if total_length <= 0:
        return default_length
    return max(total_length / max_segments, default_length)

def calculate_segments(total_length: float, target_length: float) -> int:
    if total_length <= 0:
        return 1
    return max(1, round(total_length / target_length))

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

# ==============================================================================
# PHYSICS SETUP HELPERS
# ==============================================================================

def configure_joint_drives(joint: UsdPhysics.Joint, stiff_xy: float, damp_xy: float, stiff_z: float, damp_z: float, bend_limit_deg: float, lock_z: bool = False):
    for axis in ["transX", "transY", "transZ"]:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)
        
    for axis in ["rotX", "rotY"]:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(-bend_limit_deg)
        limit.CreateHighAttr().Set(bend_limit_deg)
        
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(stiff_xy)
        drive.CreateDampingAttr().Set(damp_xy)
        drive.CreateTargetPositionAttr().Set(0.0)
        
    limit_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    if lock_z:
        limit_z.CreateLowAttr().Set(1.0)
        limit_z.CreateHighAttr().Set(-1.0)
    else:
        drive_z = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "rotZ")
        drive_z.CreateTypeAttr().Set("force")
        drive_z.CreateStiffnessAttr().Set(stiff_z)
        drive_z.CreateDampingAttr().Set(damp_z)
        drive_z.CreateTargetPositionAttr().Set(0.0)

def create_d6_bending_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str, height: float):
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])
    
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, height + Config.GAP))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    configure_joint_drives(
        joint=joint,
        stiff_xy=PhysicsConfig.STIFFNESS,
        damp_xy=PhysicsConfig.DAMPING,
        stiff_z=0.0,
        damp_z=0.0,
        bend_limit_deg=PhysicsConfig.BEND_LIMIT_DEG,
        lock_z=True
    )

def anchor_link_to_world(stage: Usd.Stage, link_path: str):
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])

def create_rigid_segment(stage: Usd.Stage, parent_path: str, name: str,
                         radius: float, height: float, 
                         world_pos: Gf.Vec3d, orientation: Gf.Quatf, mass: float) -> str:
    link_path = f"{parent_path}/{name}"
    
    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(world_pos)
    xform.AddOrientOp().Set(orientation)
    
    # Physics Rigidbody
    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)
    
    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(radius)
    cylinder.GetHeightAttr().Set(height)
    cylinder.GetAxisAttr().Set("Z")
    
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
    
    # Collision
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    
    return link_path

# ==============================================================================
# GENERATORS
# ==============================================================================

def generate_stem(stage: Usd.Stage, stem_path: str, total_length: float) -> list[dict]:
    target_length = compute_target_length(total_length, Config.MAX_STEM_SEGMENTS, Config.BASE_SEGMENT_LENGTH)
    n_segments = calculate_segments(total_length, target_length)
    
    segment_height = (total_length - (n_segments - 1) * Config.GAP) / n_segments if n_segments > 1 else total_length
    print(f"[INFO] Building stem with {n_segments} segments (Length: {total_length:.2f}m, Target Seg Len: {target_length:.2f}m)")
    
    segments_info = []
    previous_link_path = None
    
    for i in range(n_segments):
        seg_name = f"Seg_{i+1:02d}"
        base_z = i * (segment_height + Config.GAP)
        
        pos = Gf.Vec3d(0.0, 0.0, base_z)
        rot = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        
        link_path = create_rigid_segment(stage, stem_path, seg_name, Config.STEM_RADIUS, segment_height, pos, rot, PhysicsConfig.LINK_MASS)
        
        if previous_link_path is None:
            anchor_link_to_world(stage, link_path)
        else:
            joint_name = f"Joint_{i:02d}_{i+1:02d}"
            create_d6_bending_joint(stage, previous_link_path, link_path, joint_name, segment_height)
            
        previous_link_path = link_path
        
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
    target_length = compute_target_length(total_length, Config.MAX_BRANCH_SEGMENTS, Config.BASE_SEGMENT_LENGTH)
    n_segments = calculate_segments(total_length, target_length)
    
    segment_height = (total_length - (n_segments - 1) * Config.GAP) / n_segments if n_segments > 1 else total_length
    print(f"[INFO] Attaching {branch_name} to {parent_seg['path']} with {n_segments} segments (Length: {total_length:.2f}m)")
    
    branch_base_path = f"{stem_path}/Branches"
    if not stage.GetPrimAtPath(branch_base_path):
        UsdGeom.Xform.Define(stage, branch_base_path)
    
    branch_path = f"{branch_base_path}/{branch_name}"
    UsdGeom.Xform.Define(stage, branch_path)
    
    parent_pos = parent_seg["base_pos"]
    total_distance = Config.STEM_RADIUS / 2.0
    
    # Calculate relative pos for joint
    relative_z_pos = z_offset_ratio * parent_seg["height"]
    base_pos0 = Gf.Vec3d(0.0, total_distance, relative_z_pos)
    
    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot_around_trunk_deg)
    rotated_pos0 = rot_z.TransformDir(base_pos0)
    rot_total = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_angle_deg) * rot_z
    
    branch_world_base_pos = parent_pos + rotated_pos0
    
    previous_link_path = parent_seg['path']
    
    for i in range(n_segments):
        seg_name = f"Seg_{i+1:02d}"
        z_distance = i * (segment_height + Config.GAP)
        
        link_world_pos = branch_world_base_pos + rot_total.TransformDir(Gf.Vec3d(0.0, 0.0, z_distance))
        
        current_link_path = create_rigid_segment(
            stage, branch_path, seg_name, Config.BRANCH_RADIUS, segment_height, 
            link_world_pos, Gf.Quatf(rot_total.GetQuat()), BranchPhysicsConfig.MASS
        )
        
        # Joint creation
        joint_path = f"{current_link_path}/Joint_{i:02d}"
        joint = UsdPhysics.Joint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(previous_link_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(current_link_path)])
        
        if i == 0:
            # Base joint connecting to the trunk
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rotated_pos0[0], rotated_pos0[1], relative_z_pos))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(rot_total.GetQuat()))
            
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0)) # Base of the child cylinder
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            
            stiff_xy, damp_xy = BranchPhysicsConfig.BASE_STIFFNESS, BranchPhysicsConfig.BASE_DAMPING
            stiff_z, damp_z = BranchPhysicsConfig.BASE_STIFFNESS, BranchPhysicsConfig.BASE_DAMPING
        else:
            # Internal branch joints
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, segment_height + Config.GAP))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            
            stiff_xy, damp_xy = BranchPhysicsConfig.STIFFNESS_XY, BranchPhysicsConfig.DAMPING_XY
            stiff_z, damp_z = BranchPhysicsConfig.STIFFNESS_Z, BranchPhysicsConfig.DAMPING_Z

        configure_joint_drives(
            joint=joint,
            stiff_xy=stiff_xy,
            damp_xy=damp_xy,
            stiff_z=stiff_z,
            damp_z=damp_z,
            bend_limit_deg=BranchPhysicsConfig.BEND_LIMIT_DEG,
            lock_z=False
        )

        previous_link_path = current_link_path


def build_stage(output_path: str):
    if os.path.exists(output_path):
        os.remove(output_path)
        
    stage, stem_path = setup_base_stage(output_path)
    
    total_stem_length = random.uniform(1.0, 15.0)
    stem_segments = generate_stem(stage, stem_path, total_stem_length)
    
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
        
    return stage, stem_path


def main():
    output_path = get_output_usd_path()
    stage, stem_path = build_stage(output_path)
    stage.GetRootLayer().Save()
    print(f"[OK] Generalized Articulation USD saved to {output_path}")

if __name__ == "__main__":
    main()
