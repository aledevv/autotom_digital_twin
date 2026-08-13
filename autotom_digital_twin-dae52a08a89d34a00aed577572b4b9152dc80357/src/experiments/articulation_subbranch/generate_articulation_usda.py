"""
generate_articulation_usda.py

Generates an articulated trunk structure using OpenUSD cylinders.
Includes RigidBody, Collision, ArticulationRoot, and flexible D6 Joints.
"""

import os
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================

# Global scale factor (1.0 = 1 meter)
GLOBAL_SCALE = 1

class TrunkConfig:
    """Geometric settings for the main trunk."""
    N_LINKS = 10
    HEIGHT = 0.20 * GLOBAL_SCALE
    RADIUS = 0.10 * GLOBAL_SCALE
    GAP = 0.001 * GLOBAL_SCALE

class PhysicsConfig:
    """Physical behavior settings for the trunk joints and bodies."""
    LINK_MASS = 1.0 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 20.0
    STIFFNESS = 50000.0 * (GLOBAL_SCALE ** 5)
    DAMPING = 5000.0 * (GLOBAL_SCALE ** 5)

class BranchConfig:
    """Geometric and physical settings for secondary branches."""
    RADIUS = 0.04 * GLOBAL_SCALE
    HEIGHT = 0.15 * GLOBAL_SCALE
    GAP = 0.001 * GLOBAL_SCALE
    MASS = 0.2 * (GLOBAL_SCALE ** 3)
    BEND_LIMIT_DEG = 30.0
    
    # Rotational spring parameters (stiffness and damping)
    scale5 = GLOBAL_SCALE ** 5
    STIFFNESS_XY = 300.0 * scale5
    DAMPING_XY = 50.0 * scale5
    STIFFNESS_Z = 200.0 * scale5
    DAMPING_Z = 50.0 * scale5
    
    # Base attachment joint parameters (much stiffer to secure branch to trunk)
    BASE_STIFFNESS = 184000.0 * scale5
    BASE_DAMPING = 5000.0 * scale5


# ==============================================================================
# PATH & STAGE HELPERS
# ==============================================================================

def get_output_usd_path() -> str:
    """Calculates and ensures the absolute path for the output USD file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "generated_subbranch.usda")


def setup_base_stage(path: str) -> tuple:
    """Initializes the USD Stage, sets Z-up axis, and prepares the ArticulationRoot. (NO PhysX HERE)"""
    stage = Usd.Stage.CreateNew(path)
    
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    stem_path = "/World/Stem"
    stem_prim = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem_prim.GetPrim())
    
    return stage, stem_path


# ==============================================================================
# PHYSICS & GEOMETRY CREATION HELPERS
# ==============================================================================

def create_rigid_body_link(stage: Usd.Stage, parent_path: str, index: int, base_z: float) -> str:
    """Creates a Link Xform acting as a RigidBody, and nested Cylinder geometry with Collisions."""
    link_path = f"{parent_path}/Link{index:02d}"
    
    # 1. Define the spatial Xform container for this link
    xform_prim = UsdGeom.Xform.Define(stage, link_path)
    xform_prim.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, base_z))
    
    # 2. Apply Physics Rigid Body and Mass APIs
    UsdPhysics.RigidBodyAPI.Apply(xform_prim.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform_prim.GetPrim())
    mass_api.CreateMassAttr().Set(PhysicsConfig.LINK_MASS)
    
    # 3. Define the physical Cylinder geometry inside the Xform
    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(TrunkConfig.RADIUS)
    cylinder.GetHeightAttr().Set(TrunkConfig.HEIGHT)
    
    # Offset cylinder upward so its base aligns with the Xform origin
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TrunkConfig.HEIGHT / 2.0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    
    return link_path


def anchor_link_to_world(stage: Usd.Stage, link_path: str):
    """Adds a Fixed Joint to lock the first link firmly in the world coordinate space."""
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def configure_joint_drives(joint: UsdPhysics.Joint, stiff_xy: float, damp_xy: float, stiff_z: float, damp_z: float, bend_limit_deg: float, lock_z: bool = False):
    """Configures translational locking and angular drives for a D6 joint."""
    # 1. Lock all translations
    for axis in ["transX", "transY", "transZ"]:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)
        
    # 2. Configure swing (rotX and rotY)
    for axis in ["rotX", "rotY"]:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(-bend_limit_deg)
        limit.CreateHighAttr().Set(bend_limit_deg)
        
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(stiff_xy)
        drive.CreateDampingAttr().Set(damp_xy)
        drive.CreateTargetPositionAttr().Set(0.0)
        
    # 3. Configure twist (rotZ)
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


def create_d6_bending_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str):
    """Creates a custom D6 Joint that locks translations and allows spring-loaded angular bending."""
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])
    
    # Anchor sits at the top of the parent and base of the child
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, TrunkConfig.HEIGHT + TrunkConfig.GAP))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    
    # Set matching identity rotations
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


def create_sub_branch(stage, parent_link_path, branch_name, n_links=3, tilt_angle_deg=45.0, rot_around_trunk_deg=0.0):
    """Creates a parametric secondary branch with physics and geometry avoiding nested transforms."""
    # 1. Calculate attachment point on the trunk
    parent_prim = stage.GetPrimAtPath(parent_link_path)
    parent_xform = UsdGeom.Xformable(parent_prim)
    parent_translation = parent_xform.GetLocalTransformation().ExtractTranslation()
    
    # Slightly intersect trunk to ensure solid visual attachment
    total_distance = TrunkConfig.RADIUS / 2.0
    base_pos0 = Gf.Vec3d(0.0, total_distance, TrunkConfig.HEIGHT / 2.0)
    
    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot_around_trunk_deg)
    rotated_pos0 = rot_z.TransformDir(base_pos0)
    
    # Apply outward radial tilt first, then rotate around the trunk azimuth
    rot_total = rot_z * Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_angle_deg)
    branch_world_base_pos = parent_translation + rotated_pos0
    
    # 2. Create the branch container
    UsdGeom.Xform.Define(stage, "/World/Stem/Branches")
    branch_base_path = f"/World/Stem/Branches/{branch_name}"
    branch_base = UsdGeom.Xform.Define(stage, branch_base_path)
    branch_base.ClearXformOpOrder()
    
    previous_link_path = parent_link_path
    
    for i in range(n_links):
        link_index = i + 1
        current_link_path = f"{branch_base_path}/Link_{link_index:02d}"
        
        # Link Xform is placed directly at its center of mass
        link_xform = UsdGeom.Xform.Define(stage, current_link_path)
        link_xform.ClearXformOpOrder()
        
        z_center_distance = (i * (BranchConfig.HEIGHT + BranchConfig.GAP)) + (BranchConfig.HEIGHT / 2.0)
        link_world_pos = branch_world_base_pos + rot_total.TransformDir(Gf.Vec3d(0.0, 0.0, z_center_distance))
        
        link_xform.AddTranslateOp().Set(link_world_pos)
        link_xform.AddOrientOp().Set(Gf.Quatf(rot_total.GetQuat()))
        
        UsdPhysics.RigidBodyAPI.Apply(link_xform.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(link_xform.GetPrim())
        mass_api.CreateMassAttr().Set(BranchConfig.MASS)
        
        # Cylinder geometry
        cylinder_path = f"{current_link_path}/Cylinder"
        cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
        cylinder.GetRadiusAttr().Set(BranchConfig.RADIUS)
        cylinder.GetHeightAttr().Set(BranchConfig.HEIGHT)
        cylinder.GetAxisAttr().Set("Z")
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        
        # 3. Create Kinematic Joint
        joint_path = f"{current_link_path}/Joint_{branch_name}_{i:02d}"
        joint = UsdPhysics.Joint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([previous_link_path])
        joint.CreateBody1Rel().SetTargets([current_link_path])
        
        if i == 0:
            # Base joint connecting to the trunk
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rotated_pos0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(rot_total.GetQuat()))
            
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -BranchConfig.HEIGHT / 2.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            
            stiff_xy, damp_xy = BranchConfig.BASE_STIFFNESS, BranchConfig.BASE_DAMPING
            stiff_z, damp_z = BranchConfig.BASE_STIFFNESS, BranchConfig.BASE_DAMPING
        else:
            # Internal branch joints
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, BranchConfig.HEIGHT / 2.0 + BranchConfig.GAP))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -BranchConfig.HEIGHT / 2.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            
            stiff_xy, damp_xy = BranchConfig.STIFFNESS_XY, BranchConfig.DAMPING_XY
            stiff_z, damp_z = BranchConfig.STIFFNESS_Z, BranchConfig.DAMPING_Z

        configure_joint_drives(
            joint=joint,
            stiff_xy=stiff_xy,
            damp_xy=damp_xy,
            stiff_z=stiff_z,
            damp_z=damp_z,
            bend_limit_deg=BranchConfig.BEND_LIMIT_DEG,
            lock_z=False
        )

        previous_link_path = current_link_path


def build_stage(output_path: str) -> Usd.Stage:
    """Creates the entire stage (trunk + branches) and returns it, WITHOUT saving it.
    (Similar to what main does but useful to import this as a module)"""

    stage, stem_parent_path = setup_base_stage(output_path)

    # Build trunk (articulated links and joints)
    trunk_links = {}
    previous_link_path = None
    for i in range(TrunkConfig.N_LINKS):
        link_index = i + 1
        current_base_z = i * (TrunkConfig.HEIGHT + TrunkConfig.GAP)
        current_link_path = create_rigid_body_link(stage, stem_parent_path, link_index, current_base_z)
        trunk_links[link_index] = current_link_path

        if previous_link_path is None:
            anchor_link_to_world(stage, current_link_path)
        else:
            joint_name = f"Joint_{link_index-1:02d}_{link_index:02d}"
            create_d6_bending_joint(stage, previous_link_path, current_link_path, joint_name)
        previous_link_path = current_link_path

    # Creates defined branches
    branches_to_create = [
        {"parent_idx": 4, "name": "Branch_Lower_Left", "links": 7, "tilt": 45.0, "rot": 0.0},
        {"parent_idx": 7, "name": "Branch_Mid_Right", "links": 5, "tilt": 30.0, "rot": 90.0},
        {"parent_idx": 10, "name": "Branch_Upper_Left", "links": 3, "tilt": 60.0, "rot": -45.0},
    ]
    for b in branches_to_create:
        parent_path = trunk_links.get(b["parent_idx"])
        if parent_path:
            create_sub_branch(stage=stage, parent_link_path=parent_path, branch_name=b["name"],
                               n_links=b["links"], tilt_angle_deg=b["tilt"], rot_around_trunk_deg=b["rot"])

    return stage, stem_parent_path


# ==============================================================================
# MAIN EXECUTION PIPELINE
# ==============================================================================

def main():
    output_path = get_output_usd_path()
    stage, stem_parent_path = setup_base_stage(output_path)
    
    print(f"[INFO] Building articulated trunk chain with {TrunkConfig.N_LINKS} links...")
    
    # Dictionary to save trunk link paths for branching
    trunk_links = {}
    previous_link_path = None
    
    # Generate Main Trunk
    for i in range(TrunkConfig.N_LINKS):
        link_index = i + 1
        current_base_z = i * (TrunkConfig.HEIGHT + TrunkConfig.GAP)
        
        current_link_path = create_rigid_body_link(stage, stem_parent_path, link_index, current_base_z)
        trunk_links[link_index] = current_link_path
        
        if previous_link_path is None:
            anchor_link_to_world(stage, current_link_path)
        else:
            joint_name = f"Joint_{link_index-1:02d}_{link_index:02d}"
            create_d6_bending_joint(stage, previous_link_path, current_link_path, joint_name)
            
        previous_link_path = current_link_path

    # ==========================================================================
    # --- ADD PARAMETRIC SECONDARY BRANCHES ---
    # ==========================================================================
    branches_to_create = [
        {"parent_idx": 4, "name": "Branch_Lower_Left", "links": 7, "tilt": 45.0, "rot": 0.0},
        # {"parent_idx": 7, "name": "Branch_Upper_Right", "links": 4, "tilt": 50.0, "rot": 180.0},
    ]
    
    for b in branches_to_create:
        parent_path = trunk_links.get(b["parent_idx"])
        if parent_path:
            print(f"[INFO] Attaching branch {b['name']} to trunk link {b['parent_idx']}...")
            create_sub_branch(
                stage=stage,
                parent_link_path=parent_path,
                branch_name=b["name"],
                n_links=b["links"],
                tilt_angle_deg=b["tilt"],
                rot_around_trunk_deg=b["rot"]
            )

    stage.GetRootLayer().Save()
    print("[OK] Stage saved with integrated secondary branches.")


if __name__ == "__main__":
    main()