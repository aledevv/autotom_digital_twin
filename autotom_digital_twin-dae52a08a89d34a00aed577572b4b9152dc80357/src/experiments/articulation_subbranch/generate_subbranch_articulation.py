import os
import random
import math
import csv
import sys
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from src.plant_model.usd_helpers import _make_leaf

GLOBAL_SCALE = 1.0

class Config:
    MAX_STEM_SEGMENTS = 25
    MAX_BRANCH_SEGMENTS = 6
    BASE_SEGMENT_LENGTH = 0.20 * GLOBAL_SCALE
    STEM_RADIUS = 0.10 * GLOBAL_SCALE
    BRANCH_RADIUS = 0.04 * GLOBAL_SCALE
    GAP = 0.001 * GLOBAL_SCALE

class PhysicsConfig:
    LINK_MASS = 1.0 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 20.0
    STIFFNESS = 500000.0 * (GLOBAL_SCALE ** 5)
    DAMPING = 50.0 * (GLOBAL_SCALE ** 5)

class BranchPhysicsConfig:
    MASS = 0.2 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 30.0
    scale5 = GLOBAL_SCALE ** 5
    STIFFNESS_XY = 300.0 * scale5
    DAMPING_XY = 50.0 * scale5
    STIFFNESS_Z = 200.0 * scale5
    DAMPING_Z = 50.0 * scale5
    BASE_STIFFNESS = 184000.0 * scale5
    BASE_DAMPING = 5000.0 * scale5

class MockKey:
    def __init__(self, rank, order, organ_index):
        self.rank = rank
        self.order = order
        self.organ_index = organ_index

class MockLeafNode:
    def __init__(self, rot_angle: float, tilt_angle: float, rank: int):
        self.key = MockKey(rank=rank, order=1, organ_index=rank)
        self.ccw_orientation = rot_angle
        self.angle_petiole = tilt_angle
        self.length_petiole = 0.08
        self.diameter_petiole = 0.01
        self.rachis_length = 0.2
        self.blades_nr = 3
        self.leaf_area_m2blades = [0.005, 0.01, 0.015]
        self.leaf_segments_length = [self.rachis_length * 0.6]
        self.leaf_inclination_segments = [50.0]
        self.area_blades_total = 0.03

def get_output_usd_path() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "generated_subbranch_articulation.usda")

def setup_base_stage(path: str) -> tuple:
    stage = Usd.Stage.CreateNew(path)
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    stem_path = "/World/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())
    return stage, stem_path

def compute_target_length(total_length: float, max_segments: int, default_length: float) -> float:
    if total_length <= 0: return default_length
    return max(total_length / max_segments, default_length)

def calculate_segments(total_length: float, target_length: float) -> int:
    if total_length <= 0: return 1
    return max(1, round(total_length / target_length))

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
    
    configure_joint_drives(joint, PhysicsConfig.STIFFNESS, PhysicsConfig.DAMPING, 0.0, 0.0, PhysicsConfig.BEND_LIMIT_DEG, True)

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
    
    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)
    
    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(radius)
    cylinder.GetHeightAttr().Set(height)
    cylinder.GetAxisAttr().Set("Z")
    
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    
    return link_path

def generate_stem(stage: Usd.Stage, stem_path: str, segment_lengths: list[float]) -> list[dict]:
    n_segments = len(segment_lengths)
    total_length = sum(segment_lengths)
    print(f"[INFO] Building physical stem with {n_segments} physical internodes (Total Length: {total_length:.2f}m)")
    
    segments_info = []
    previous_link_path = None
    current_z = 0.0
    
    for i, segment_height in enumerate(segment_lengths):
        seg_name = f"Internode_{i+1:02d}"
        pos = Gf.Vec3d(0.0, 0.0, current_z)
        rot = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        
        link_path = create_rigid_segment(stage, stem_path, seg_name, Config.STEM_RADIUS, segment_height, pos, rot, PhysicsConfig.LINK_MASS)
        
        if previous_link_path is None:
            anchor_link_to_world(stage, link_path)
        else:
            joint_name = f"Joint_{i:02d}_{i+1:02d}"
            create_d6_bending_joint(stage, previous_link_path, link_path, joint_name, segments_info[-1]["height"])
            
        previous_link_path = link_path
        
        segments_info.append({
            "path": link_path,
            "index": i + 1,
            "base_pos": pos,
            "height": segment_height,
            "global_rot": Gf.Rotation(rot),
            "radius": Config.STEM_RADIUS
        })
        
        current_z += segment_height + Config.GAP
        
    return segments_info

def generate_lateral_appendage(stage: Usd.Stage, base_path: str, parent_seg: dict, appendage_name: str, 
                    total_length: float, tilt_angle_deg: float, rot_around_parent_deg: float,
                    z_offset_ratio: float = 0.5):
    target_length = compute_target_length(total_length, Config.MAX_BRANCH_SEGMENTS, Config.BASE_SEGMENT_LENGTH)
    n_segments = calculate_segments(total_length, target_length)
    
    segment_height = (total_length - (n_segments - 1) * Config.GAP) / n_segments if n_segments > 1 else total_length
    print(f"[INFO] Attaching {appendage_name} to {parent_seg['path']} with {n_segments} segments (Length: {total_length:.2f}m)")
    
    appendages_base_path = f"{base_path}/Branches"
    if not stage.GetPrimAtPath(appendages_base_path):
        UsdGeom.Xform.Define(stage, appendages_base_path)
    
    appendage_path = f"{appendages_base_path}/{appendage_name}"
    UsdGeom.Xform.Define(stage, appendage_path)
    
    parent_pos = parent_seg["base_pos"]
    parent_rot = parent_seg.get("global_rot", Gf.Rotation(Gf.Quatf(1.0, 0.0, 0.0, 0.0)))
    parent_radius = parent_seg.get("radius", Config.STEM_RADIUS)
    
    relative_z_pos = z_offset_ratio * parent_seg["height"]
    local_offset_base = Gf.Vec3d(0.0, parent_radius, relative_z_pos)
    
    local_rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot_around_parent_deg)
    local_tilt = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_angle_deg)
    
    sub_rot_local = local_tilt * local_rot_z
    local_pos0 = local_rot_z.TransformDir(local_offset_base)
    
    sub_rot_total = sub_rot_local * parent_rot
    branch_world_base_pos = parent_pos + parent_rot.TransformDir(local_pos0)
    
    previous_link_path = parent_seg['path']
    appendage_segments_info = []
    
    for i in range(n_segments):
        seg_name = f"Segment_{i+1:02d}"
        z_distance = i * (segment_height + Config.GAP)
        
        link_world_pos = branch_world_base_pos + sub_rot_total.TransformDir(Gf.Vec3d(0.0, 0.0, z_distance))
        
        current_link_path = create_rigid_segment(
            stage, appendage_path, seg_name, Config.BRANCH_RADIUS, segment_height, 
            link_world_pos, Gf.Quatf(sub_rot_total.GetQuat()), BranchPhysicsConfig.MASS
        )
        
        joint_path = f"{current_link_path}/Joint_{i:02d}"
        joint = UsdPhysics.Joint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(previous_link_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(current_link_path)])
        
        if i == 0:
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(local_pos0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(sub_rot_local.GetQuat()))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            stiff_xy, damp_xy = BranchPhysicsConfig.BASE_STIFFNESS, BranchPhysicsConfig.BASE_DAMPING
            stiff_z, damp_z = BranchPhysicsConfig.BASE_STIFFNESS, BranchPhysicsConfig.BASE_DAMPING
        else:
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, segment_height + Config.GAP))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            stiff_xy, damp_xy = BranchPhysicsConfig.STIFFNESS_XY, BranchPhysicsConfig.DAMPING_XY
            stiff_z, damp_z = BranchPhysicsConfig.STIFFNESS_Z, BranchPhysicsConfig.DAMPING_Z

        configure_joint_drives(joint, stiff_xy, damp_xy, stiff_z, damp_z, BranchPhysicsConfig.BEND_LIMIT_DEG, False)

        previous_link_path = current_link_path
        appendage_segments_info.append({
            "path": current_link_path,
            "index": i + 1,
            "base_pos": link_world_pos,
            "height": segment_height,
            "global_rot": sub_rot_total,
            "radius": Config.BRANCH_RADIUS
        })
        
    return appendage_segments_info

def generate_leaf(stage: Usd.Stage, stem_path: str, parent_seg: dict, leaf_name: str,
                  length: float, width: float, tilt_angle_deg: float, rot_around_parent_deg: float,
                  z_offset_ratio: float, leaf_rank: int):
    print(f"[INFO] Attaching Leaf {leaf_name} to {parent_seg['path']} (visual only, static).")
    
    leaf_path = f"{parent_seg['path']}/{leaf_name}"
    rametto_xform = UsdGeom.Xform.Define(stage, leaf_path)
    
    relative_z_pos = z_offset_ratio * parent_seg["height"]
    parent_radius = parent_seg.get("radius", Config.STEM_RADIUS)
    
    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot_around_parent_deg)
    base_pos0 = Gf.Vec3d(parent_radius, 0.0, relative_z_pos)
    local_pos = rot_z.TransformDir(base_pos0)
    
    rametto_xform.ClearXformOpOrder()
    rametto_xform.AddTranslateOp().Set(local_pos)
    
    scale_factor = length / 0.15
    rametto_xform.AddScaleOp().Set(Gf.Vec3f(scale_factor, scale_factor, scale_factor))
    
    mock_node = MockLeafNode(rot_around_parent_deg, tilt_angle_deg, leaf_rank)
    materials = {"leaf": None, "pedicel": None}
    
    _make_leaf(stage, leaf_path, mock_node, 0.0, materials)

def get_segment_and_offset(segments: list, total_length: float, z_ratio_total: float):
    target_z = total_length * z_ratio_total
    current_z = 0.0
    for seg in segments:
        if current_z + seg["height"] >= target_z or seg == segments[-1]:
            offset_within_seg = max(0.0, target_z - current_z)
            return seg, offset_within_seg
        current_z += seg["height"] + Config.GAP
    return segments[-1], segments[-1]["height"]

def build_stage_from_csv_data(output_path: str, csv_data: list):
    if os.path.exists(output_path):
        os.remove(output_path)
        
    stage, stem_path = setup_base_stage(output_path)
    
    internode_rows = [row for row in csv_data if row.get("organ_class") == "Internode"]
    lateral_rows = [row for row in csv_data if row.get("organ_class") in ["Branch", "Subbranch"]]
    leaf_rows = [row for row in csv_data if row.get("organ_class") == "Leaf"]
    
    physical_mapping = {}
    
    # --- STEM ---
    physical_segment_lengths = [float(row["length"]) for row in internode_rows]
    stem_segments = generate_stem(stage, stem_path, physical_segment_lengths)
    
    for i, row in enumerate(internode_rows):
        physical_mapping[row["id"]] = (stem_segments[i], float(row["length"]))
            
    # --- BRANCHES AND SUBBRANCHES ---
    processed_laterals = {}
    
    for row in lateral_rows:
        parent_id = row["parent_id"]
        
        if parent_id in physical_mapping:
            # Trunk parent
            parent_seg, abs_height = physical_mapping[parent_id]
            z_ratio = abs_height / parent_seg["height"] if parent_seg["height"] > 0 else 0.0
        elif parent_id in processed_laterals:
            # Lateral parent
            parent_row = processed_laterals[parent_id]
            parent_segments = parent_row["_segments"]
            parent_length = parent_row["_length"]
            z_ratio_total = float(row.get("z_offset_ratio") or 0.5)
            
            parent_seg, abs_height = get_segment_and_offset(parent_segments, parent_length, z_ratio_total)
            z_ratio = abs_height / parent_seg["height"] if parent_seg["height"] > 0 else 0.0
        else:
            print(f"[WARN] Parent {parent_id} not found for {row['id']}")
            continue
            
        total_length = float(row["length"])
        appendage_name = row["id"]
        
        branch_segments = generate_lateral_appendage(
            stage=stage,
            base_path=stem_path,
            parent_seg=parent_seg,
            appendage_name=appendage_name,
            total_length=total_length,
            tilt_angle_deg=float(row["tilt_angle"]),
            rot_around_parent_deg=float(row["rot_angle"]),
            z_offset_ratio=z_ratio
        )
        
        row["_segments"] = branch_segments
        row["_length"] = total_length
        processed_laterals[row["id"]] = row
        
    # --- LEAVES ---
    leaf_rank = 1
    for row in leaf_rows:
        parent_id = row["parent_id"]
        
        if parent_id in physical_mapping:
            parent_seg, abs_height = physical_mapping[parent_id]
            z_ratio = abs_height / parent_seg["height"] if parent_seg["height"] > 0 else 0.0
        elif parent_id in processed_laterals:
            parent_row = processed_laterals[parent_id]
            parent_segments = parent_row["_segments"]
            parent_length = parent_row["_length"]
            z_ratio_total = float(row.get("z_offset_ratio") or 0.5)
            parent_seg, abs_height = get_segment_and_offset(parent_segments, parent_length, z_ratio_total)
            z_ratio = abs_height / parent_seg["height"] if parent_seg["height"] > 0 else 0.0
        else:
            continue
        
        generate_leaf(
            stage=stage,
            stem_path=stem_path,
            parent_seg=parent_seg,
            leaf_name=row["id"],
            length=float(row.get("length", 0) or 0),
            width=float(row.get("width_m", 0) or 0.01),
            tilt_angle_deg=float(row.get("tilt_angle", 0) or 0),
            rot_around_parent_deg=float(row.get("rot_angle", 0) or 0),
            z_offset_ratio=z_ratio,
            leaf_rank=leaf_rank
        )
        leaf_rank += 1
            
    return stage, stem_path

def build_stage(output_path: str):
    csv_data = [
        {"id": "Internode_00", "organ_class": "Internode", "parent_id": "", "length": 2.0, "width_m": Config.STEM_RADIUS * 2, "tilt_angle": 0.0, "rot_angle": 0.0},
        {"id": "Branch_01", "organ_class": "Branch", "parent_id": "Internode_00", "length": 1.5, "width_m": Config.BRANCH_RADIUS * 2, "z_offset_ratio": 1.0, "tilt_angle": 45.0, "rot_angle": 90.0},
        {"id": "Subbranch_01_01", "organ_class": "Subbranch", "parent_id": "Branch_01", "length": 0.8, "width_m": Config.BRANCH_RADIUS * 2, "z_offset_ratio": 0.5, "tilt_angle": 30.0, "rot_angle": 90.0},
        {"id": "Subbranch_01_02", "organ_class": "Subbranch", "parent_id": "Branch_01", "length": 0.8, "width_m": Config.BRANCH_RADIUS * 2, "z_offset_ratio": 0.5, "tilt_angle": 30.0, "rot_angle": -90.0}
    ]
    stage, stem_path = build_stage_from_csv_data(output_path, csv_data)
    return stage, stem_path, csv_data

def save_config_csv(output_path: str, csv_data: list):
    csv_path = output_path.replace(".usda", "_config.csv")
    with open(csv_path, "w", newline='') as f:
        # filter out internal keys
        clean_data = [{k: v for k, v in row.items() if not k.startswith('_')} for row in csv_data]
        writer = csv.DictWriter(f, fieldnames=["id", "organ_class", "parent_id", "length", "width_m", "parent_segment_idx", "z_offset_ratio", "tilt_angle", "rot_angle"])
        writer.writeheader()
        writer.writerows(clean_data)
    print(f"[OK] Config CSV saved to {csv_path}")

def main():
    output_path = get_output_usd_path()
    stage, stem_path, csv_data = build_stage(output_path)
    stage.GetRootLayer().Save()
    print(f"[OK] Generalized Articulation USD saved to {output_path}")
    save_config_csv(output_path, csv_data)

if __name__ == "__main__":
    main()
