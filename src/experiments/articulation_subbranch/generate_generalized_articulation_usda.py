"""
generate_generalized_articulation_usda.py

Generates an articulated trunk structure (main stem + sub-branches) using OpenUSD cylinders.
Includes RigidBody, Collision, ArticulationRoot, and flexible D6 Joints.
Supports randomized branch generation and segment limitation logic.
"""

import os
import random
import math
import csv
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================

GLOBAL_SCALE = 1.0

class Config:
    """Geometric settings for the generation."""
    # PhysX restricts articulations to a maximum of 64 links.
    # To avoid 'Invalid PhysX transform' errors, we must ensure:
    # Stem segments + (Branches * Branch segments) <= 64
    MAX_STEM_SEGMENTS = 25
    MAX_BRANCH_SEGMENTS = 6
    
    BASE_SEGMENT_LENGTH = 0.20 * GLOBAL_SCALE
    STEM_RADIUS = 0.10 * GLOBAL_SCALE
    BRANCH_RADIUS = 0.04 * GLOBAL_SCALE
    GAP = 0.001 * GLOBAL_SCALE

class GenerationConfig:
    """Randomization and procedural generation settings."""
    MIN_STEM_LENGTH = 3.0
    MAX_STEM_LENGTH = 15.0
    
    MIN_BRANCHES = 3
    MAX_BRANCHES = 25
    
    MIN_BRANCH_LENGTH = 0.7
    MAX_BRANCH_LENGTH = 3.0
    
    MIN_TILT_ANGLE = 30.0
    MAX_TILT_ANGLE = 75.0
    
    MIN_Z_RATIO = 0.1
    MAX_Z_RATIO = 0.9
    
    MAX_PER_INTERNODE = 4
    MAX_PLACEMENT_ATTEMPTS = 20

class PhysicsConfig:
    """Physical behavior settings for the trunk joints and bodies."""
    LINK_MASS = 1.0 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 20.0
    STIFFNESS = 500000.0 * (GLOBAL_SCALE ** 5)
    DAMPING = 50.0 * (GLOBAL_SCALE ** 5)

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

def generate_stem(stage: Usd.Stage, stem_path: str, segment_lengths: list[float]) -> list[dict]:
    n_segments = len(segment_lengths)
    total_length = sum(segment_lengths)
    print(f"[INFO] Building physical stem with {n_segments} physical internodes (Total Length: {total_length:.2f}m)")
    
    segments_info = []
    previous_link_path = None
    current_z = 0.0
    
    for i, segment_height in enumerate(segment_lengths):
        seg_name = f"Internode_{i+1:02d}"
        base_z = current_z
        
        pos = Gf.Vec3d(0.0, 0.0, base_z)
        rot = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        
        link_path = create_rigid_segment(stage, stem_path, seg_name, Config.STEM_RADIUS, segment_height, pos, rot, PhysicsConfig.LINK_MASS)
        
        if previous_link_path is None:
            anchor_link_to_world(stage, link_path)
        else:
            joint_name = f"Joint_{i:02d}_{i+1:02d}"
            prev_seg_height = segments_info[-1]["height"]
            create_d6_bending_joint(stage, previous_link_path, link_path, joint_name, prev_seg_height)
            
        previous_link_path = link_path
        
        segments_info.append({
            "path": link_path,
            "index": i + 1,
            "base_pos": pos,
            "height": segment_height
        })
        
        current_z += segment_height + Config.GAP
        
    return segments_info

def generate_branch(stage: Usd.Stage, stem_path: str, parent_seg: dict, branch_name: str, 
                    total_length: float, tilt_angle_deg: float, rot_around_trunk_deg: float,
                    z_offset_ratio: float = 0.5):
    target_length = compute_target_length(total_length, Config.MAX_BRANCH_SEGMENTS, Config.BASE_SEGMENT_LENGTH)
    n_segments = calculate_segments(total_length, target_length)
    
    segment_height = (total_length - (n_segments - 1) * Config.GAP) / n_segments if n_segments > 1 else total_length
    print(f"[INFO] Attaching {branch_name} to {parent_seg['path']} with {n_segments} physical branch internodes (Length: {total_length:.2f}m)")
    
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
        seg_name = f"Branch_Internode_{i+1:02d}"
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
    # 1. Generate biological internodes (testing the logic)
    num_internodes = random.randint(15, 60)
    biological_internodes = []
    
    csv_data = []
    for i in range(num_internodes):
        ilength = random.uniform(0.05, 0.25)
        iid = f"Internode_{i:02d}"
        biological_internodes.append({"id": iid, "length": ilength})
        
        csv_data.append({
            "id": iid,
            "organ_class": "Internode",
            "parent_id": f"Internode_{i-1:02d}" if i > 0 else "",
            "length": round(ilength, 4),
            "width_m": round(Config.STEM_RADIUS * 2, 4),
            "parent_segment_idx": "",
            "z_offset_ratio": "",
            "tilt_angle": "",
            "rot_angle": ""
        })
    
    # We must calculate segment lengths exactly like build_stage_from_csv_data will do,
    # so we can build the USD in this function. But actually, it's cleaner to just call 
    # build_stage_from_csv_data and pass it the generated csv_data!
    # Wait, the prompt says "build_stage" generates USD. Let's just generate the CSV here,
    # and use the determinisitic builder to actually build the stage.
    
    n_branches = random.randint(GenerationConfig.MIN_BRANCHES, GenerationConfig.MAX_BRANCHES)
    segment_branches = {}
    branch_count = 0
    MIN_DIST = Config.BRANCH_RADIUS * 2.5
    
    for _ in range(n_branches):
        if num_internodes > 1:
            parent_idx = random.randint(1, num_internodes - 1)
        else:
            parent_idx = 0
            
        if parent_idx not in segment_branches:
            segment_branches[parent_idx] = []
            
        if len(segment_branches[parent_idx]) >= GenerationConfig.MAX_PER_INTERNODE:
            continue
            
        # The segment_branches tracks (z_ratio, rot) to avoid collision. 
        # But wait, now Z ratio is within the INTERNODE.
        # Let's assume branches attach exactly at the top of the biological internode (z_ratio=1.0)
        # So we only randomize rotation.
        valid_spawn = False
        attempts = 0
        z_ratio = 1.0 # Biological branches usually attach at the node (top of internode)
        rot = 0.0
        while not valid_spawn and attempts < GenerationConfig.MAX_PLACEMENT_ATTEMPTS:
            rot = random.uniform(0.0, 360.0)
            if check_collision((z_ratio, rot), segment_branches[parent_idx], Config.STEM_RADIUS, biological_internodes[parent_idx]["length"], MIN_DIST):
                valid_spawn = True
                segment_branches[parent_idx].append((z_ratio, rot))
            attempts += 1
            
        if not valid_spawn:
            continue
            
        branch_length = random.uniform(GenerationConfig.MIN_BRANCH_LENGTH, GenerationConfig.MAX_BRANCH_LENGTH)
        tilt = random.uniform(GenerationConfig.MIN_TILT_ANGLE, GenerationConfig.MAX_TILT_ANGLE)
        
        branch_count += 1
        
        csv_data.append({
            "id": f"Branch_{branch_count:02d}",
            "organ_class": "Branch",
            "parent_id": biological_internodes[parent_idx]["id"],
            "length": round(branch_length, 4),
            "width_m": round(Config.BRANCH_RADIUS * 2, 4),
            "parent_segment_idx": "",
            "z_offset_ratio": round(z_ratio, 4),
            "tilt_angle": round(tilt, 4),
            "rot_angle": round(rot, 4)
        })
        
    # Now that we generated the purely biological CSV, we call our deterministic builder!
    # Wait, the previous logic returned `stage, stem_path, csv_data`.
    # I can just re-use `build_stage_from_csv_data` here directly so the output matches!
    stage, stem_path = build_stage_from_csv_data(output_path, csv_data)
    
    return stage, stem_path, csv_data


def build_stage_from_csv_data(output_path: str, csv_data: list):
    """Builds the USD stage deterministically using the parameters from csv_data."""
    if os.path.exists(output_path):
        os.remove(output_path)
        
    stage, stem_path = setup_base_stage(output_path)
    
    internode_rows = [row for row in csv_data if row["organ_class"] == "Internode"]
    branch_rows = [row for row in csv_data if row["organ_class"] == "Branch"]
    
    n_internodes = len(internode_rows)
    max_segments = Config.MAX_STEM_SEGMENTS
    
    physical_segment_lengths = []
    internode_mapping = {} # id -> (physical_segment_idx, height_within_segment)
    
    if n_internodes > max_segments:
        # Group internodes into max_segments
        group_size = n_internodes / max_segments
        current_physical_idx = 0
        current_physical_length = 0.0
        
        for i, i_row in enumerate(internode_rows):
            target_group = min(int(i / group_size), max_segments - 1)
            
            if target_group > current_physical_idx:
                physical_segment_lengths.append(current_physical_length)
                current_physical_idx = target_group
                current_physical_length = 0.0
            
            length = float(i_row["length"])
            current_physical_length += length
            
            # The node attaches at the top of the biological internode
            internode_mapping[i_row["id"]] = (current_physical_idx, current_physical_length)
            
        physical_segment_lengths.append(current_physical_length)
    else:
        for i, i_row in enumerate(internode_rows):
            length = float(i_row["length"])
            physical_segment_lengths.append(length)
            internode_mapping[i_row["id"]] = (i, length)
            
    # Now build the stem with these specific physical segment lengths
    stem_segments = generate_stem(stage, stem_path, physical_segment_lengths)
    
    # Build the branches using precise accumulated Z-offset ratios
    for row in branch_rows:
        parent_id = row["parent_id"]
        if parent_id not in internode_mapping:
            continue
            
        physical_seg_idx, abs_height = internode_mapping[parent_id]
        parent_seg = stem_segments[physical_seg_idx]
        physical_length = physical_segment_lengths[physical_seg_idx]
        
        # Calculate exactly where it should be on the merged physical cylinder
        z_ratio = abs_height / physical_length if physical_length > 0 else 0.0
        
        rot = float(row["rot_angle"])
        tilt = float(row["tilt_angle"])
        branch_length = float(row["length"])
        branch_name = row["id"]
        
        generate_branch(
            stage=stage,
            stem_path=stem_path,
            parent_seg=parent_seg,
            branch_name=branch_name,
            total_length=branch_length,
            tilt_angle_deg=tilt,
            rot_around_trunk_deg=rot,
            z_offset_ratio=z_ratio
        )
            
    return stage, stem_path


def save_config_csv(output_path: str, csv_data: list):
    csv_path = output_path.replace(".usda", "_config.csv")
    with open(csv_path, "w", newline='') as f:
        writer = csv.DictWriter(f, fieldnames=["id", "organ_class", "parent_id", "length", "width_m", "parent_segment_idx", "z_offset_ratio", "tilt_angle", "rot_angle"])
        writer.writeheader()
        writer.writerows(csv_data)
    print(f"[OK] Config CSV saved to {csv_path}")

def main():
    output_path = get_output_usd_path()
    stage, stem_path, csv_data = build_stage(output_path)
    stage.GetRootLayer().Save()
    print(f"[OK] Generalized Articulation USD saved to {output_path}")
    save_config_csv(output_path, csv_data)

if __name__ == "__main__":
    main()
