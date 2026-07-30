"""
generate_tomato_branch_usda.py

Generates an articulated trunk and a single branch structure using OpenUSD cylinders.
Uses original hardcoded stiffness/damping values for initial validation.
"""

import os
import math
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================

# Global scale factor (1.0 = 1 meter)
GLOBAL_SCALE = 5.0

class TrunkConfig:
    """Geometric settings for the main trunk."""
    N_LINKS = 3
    HEIGHT = 0.05 * GLOBAL_SCALE  # 5 cm unscaled
    RADIUS = 0.01 * GLOBAL_SCALE  # 1 cm unscaled
    GAP = 0.001 * GLOBAL_SCALE

class PhysicsConfig:
    """Physical behavior settings for the trunk joints and bodies."""
    BEND_LIMIT_DEG = 20.0

class BranchConfig:
    """Geometric and physical settings for secondary branches."""
    RADIUS = 0.005 * GLOBAL_SCALE  # 0.5 cm unscaled
    HEIGHT = 0.05 * GLOBAL_SCALE   # 5 cm unscaled
    GAP = 0.001 * GLOBAL_SCALE
    BEND_LIMIT_DEG = 30.0

class BioConfig:
    """Biological material properties for a tomato plant."""
    
    # Modulo di elasticità per fusti erbacei/rampicanti (circa 150 MPa)
    YOUNG_MODULUS = 8.0e7  # Pa [Piu floscio: 8.0e7, medio: 1.5e8, rigido: 2.5e8]
    
    # Rapporto di smorzamento (Sottosmorzato per permettere il "traballamento")
    DAMPING_RATIO = 0.15   # Adimensionale (Un valore tra 0.05 -> molto smorzato, 0.3 -> troppo rigido)
    
    # Densità (Le piante erbacee sono quasi il 90-95% acqua)
    PLANT_DENSITY = 1000.0 # kg/m^3

def calculate_physics_params(radius: float, length: float, mass: float) -> tuple[float, float]:
    """
    Calculates joint Stiffness (K) and Damping (D) based on biological material properties.
    radius: scaled radius of the link
    length: scaled length of the link
    mass: scaled mass of the link (or mass to support)
    """
    # Moment of inertia for a solid cylinder: I = (pi * r^4) / 4
    I = (math.pi * (radius ** 4)) / 4.0
    
    # Bending stiffness: K = (E * I) / L
    K = (BioConfig.YOUNG_MODULUS * I) / length
    
    # Damping: D = 2 * zeta * sqrt(K * M)
    D = 2.0 * BioConfig.DAMPING_RATIO * math.sqrt(K * mass)
    
    return K, D

def compute_mass(radius: float, height: float) -> float:
    """Computes mass based on cylindrical volume and plant density."""
    volume = math.pi * (radius ** 2) * height
    return BioConfig.PLANT_DENSITY * volume


# ==============================================================================
# PATH & STAGE HELPERS
# ==============================================================================

def get_output_usd_path() -> str:
    """Calculates and ensures the absolute path for the output USD file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "generated_tomato_branch.usda")


def setup_base_stage(path: str) -> tuple:
    """Initializes the USD Stage, sets Z-up axis, and prepares the ArticulationRoot."""
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
    
    xform_prim = UsdGeom.Xform.Define(stage, link_path)
    xform_prim.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, base_z))
    
    UsdPhysics.RigidBodyAPI.Apply(xform_prim.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform_prim.GetPrim())
    mass = compute_mass(TrunkConfig.RADIUS, TrunkConfig.HEIGHT)
    mass_api.CreateMassAttr().Set(mass)
    
    cylinder_path = f"{link_path}/Cylinder"
    cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
    cylinder.GetRadiusAttr().Set(TrunkConfig.RADIUS)
    cylinder.GetHeightAttr().Set(TrunkConfig.HEIGHT)
    
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


def create_d6_bending_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str):
    """Creates a custom D6 Joint that locks translations and allows spring-loaded angular bending."""
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])
    
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, TrunkConfig.HEIGHT + TrunkConfig.GAP))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    mass = compute_mass(TrunkConfig.RADIUS, TrunkConfig.HEIGHT)
    stiff, damp = calculate_physics_params(TrunkConfig.RADIUS, TrunkConfig.HEIGHT, mass)
    
    configure_joint_drives(
        joint=joint,
        stiff_xy=stiff,
        damp_xy=damp,
        stiff_z=0.0,
        damp_z=0.0,
        bend_limit_deg=PhysicsConfig.BEND_LIMIT_DEG,
        lock_z=True
    )


def create_sub_branch(stage, parent_link_path, branch_name, n_links=3, tilt_angle_deg=45.0, rot_around_trunk_deg=0.0):
    """Creates a parametric secondary branch with physics and geometry."""
    parent_prim = stage.GetPrimAtPath(parent_link_path)
    parent_xform = UsdGeom.Xformable(parent_prim)
    parent_translation = parent_xform.GetLocalTransformation().ExtractTranslation()
    
    total_distance = TrunkConfig.RADIUS / 2.0
    base_pos0 = Gf.Vec3d(0.0, total_distance, TrunkConfig.HEIGHT / 2.0)
    
    rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), rot_around_trunk_deg)
    rotated_pos0 = rot_z.TransformDir(base_pos0)
    
    rot_total = rot_z * Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -tilt_angle_deg)
    branch_world_base_pos = parent_translation + rotated_pos0
    
    UsdGeom.Xform.Define(stage, "/World/Stem/Branches")
    branch_base_path = f"/World/Stem/Branches/{branch_name}"
    branch_base = UsdGeom.Xform.Define(stage, branch_base_path)
    branch_base.ClearXformOpOrder()
    
    previous_link_path = parent_link_path
    
    for i in range(n_links):
        link_index = i + 1
        current_link_path = f"{branch_base_path}/Link_{link_index:02d}"
        
        link_xform = UsdGeom.Xform.Define(stage, current_link_path)
        link_xform.ClearXformOpOrder()
        
        z_center_distance = (i * (BranchConfig.HEIGHT + BranchConfig.GAP)) + (BranchConfig.HEIGHT / 2.0)
        link_world_pos = branch_world_base_pos + rot_total.TransformDir(Gf.Vec3d(0.0, 0.0, z_center_distance))
        
        link_xform.AddTranslateOp().Set(link_world_pos)
        link_xform.AddOrientOp().Set(Gf.Quatf(rot_total.GetQuat()))
        
        UsdPhysics.RigidBodyAPI.Apply(link_xform.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(link_xform.GetPrim())
        mass = compute_mass(BranchConfig.RADIUS, BranchConfig.HEIGHT)
        mass_api.CreateMassAttr().Set(mass)
        
        cylinder_path = f"{current_link_path}/Cylinder"
        cylinder = UsdGeom.Cylinder.Define(stage, cylinder_path)
        cylinder.GetRadiusAttr().Set(BranchConfig.RADIUS)
        cylinder.GetHeightAttr().Set(BranchConfig.HEIGHT)
        cylinder.GetAxisAttr().Set("Z")
        UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
        
        joint_path = f"{current_link_path}/Joint_{branch_name}_{i:02d}"
        joint = UsdPhysics.Joint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([previous_link_path])
        joint.CreateBody1Rel().SetTargets([current_link_path])
        
        if i == 0:
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(rotated_pos0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(rot_total.GetQuat()))
            
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -BranchConfig.HEIGHT / 2.0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            
            stiff_xy, damp_xy = calculate_physics_params(BranchConfig.RADIUS, BranchConfig.HEIGHT, mass)
            # Make the base attachment joint a bit stiffer to avoid sagging at the trunk connection
            stiff_xy *= 2.0
            stiff_z, damp_z = stiff_xy, damp_xy
        else:
            joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, BranchConfig.HEIGHT / 2.0 + BranchConfig.GAP))
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -BranchConfig.HEIGHT / 2.0))
            joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, 0, 0, 0))
            joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
            
            stiff_xy, damp_xy = calculate_physics_params(BranchConfig.RADIUS, BranchConfig.HEIGHT, mass)
            stiff_z, damp_z = stiff_xy, damp_xy

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
    """Creates the entire stage (trunk + branches)."""
    stage, stem_parent_path = setup_base_stage(output_path)

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

    # Only create a single branch on link 2
    branches_to_create = [
        {"parent_idx": 2, "name": "Branch_1", "links": 8, "tilt": 45.0, "rot": 0.0},
    ]
    for b in branches_to_create:
        parent_path = trunk_links.get(b["parent_idx"])
        if parent_path:
            create_sub_branch(stage=stage, parent_link_path=parent_path, branch_name=b["name"],
                               n_links=b["links"], tilt_angle_deg=b["tilt"], rot_around_trunk_deg=b["rot"])

    return stage, stem_parent_path


def main():
    output_path = get_output_usd_path()
    stage, stem_parent_path = setup_base_stage(output_path)
    
    print(f"[INFO] Building articulated trunk chain with {TrunkConfig.N_LINKS} links...")
    
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

    branches_to_create = [
        {"parent_idx": 2, "name": "Branch_1", "links": 8, "tilt": 45.0, "rot": 0.0},
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
