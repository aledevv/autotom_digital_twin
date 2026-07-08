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

class TrunkConfig:
    """Geometric settings for the main trunk."""
    N_LINKS = 10       # Number of consecutive trunk segments
    HEIGHT = 0.20      # Height of each single segment (meters)
    RADIUS = 0.10      # Radius of each segment (meters)
    GAP = 0.001         # Distance between adjacent segments (meters)

class PhysicsConfig:
    """Physical behavior settings for the joints and bodies."""
    LINK_MASS = 1.0          # Mass of each segment in kg
    BEND_LIMIT_DEG = 20.0    # Maximum angular bending limit for rotX and rotY
    STIFFNESS = 50000.0      # Spring stiffness coefficient to straighten the branch
    DAMPING = 5000.0         # Spring damping coefficient to prevent infinite oscillation


# ==============================================================================
# PATH & STAGE HELPERS
# ==============================================================================

def get_output_usd_path() -> str:
    """Calculates and ensures the absolute path for the output USD file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Ascend 3 levels to match the repository structure ('data/usd_models/')
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "generated_subbranch.usda")


def setup_base_stage(path: str) -> tuple:
    """Initializes the USD Stage, sets Z-up axis, and prepares the ArticulationRoot."""
    stage = Usd.Stage.CreateNew(path)
    
    # Create the mandatory faked Root Prim required by the loader script
    world_prim = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world_prim.GetPrim())
    
    # Isaac Sim works with Z-up axis conventions
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    
    # Create the articulation root container for optimal physics solvers
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
    
    # Offset the cylinder upwards by half height so its bottom aligns with Xform origin
    cylinder.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TrunkConfig.HEIGHT / 2.0))
    
    # 4. Enable Collisions on the geometry
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    
    return link_path


def anchor_link_to_world(stage: Usd.Stage, link_path: str):
    """Adds a Fixed Joint to lock the first link firmly in the world coordinate space."""
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    
    # Body0 left empty automatically targets the static World background
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def create_d6_bending_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str):
    """Creates a custom D6 Joint that locks translations and allows spring-loaded angular bending."""
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)
    
    # Connect the parent and child bodies
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])
    
    # Set local anchor positions (Joint sits at the top of the parent and base of the child)
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, TrunkConfig.HEIGHT + TrunkConfig.GAP))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    
    # Set matching identity rotations
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    
    # --- CONFIGURE D6 DEGREES OF FREEDOM ---
    
    # 1. Lock all translations (low > high is the USD standard syntax to completely lock an axis)
    for axis in ["transX", "transY", "transZ"]:
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(1.0)
        limit.CreateHighAttr().Set(-1.0)
        
    # 2. Lock torsional/twist rotation around the Z axis
    limit_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    limit_z.CreateLowAttr().Set(1.0)
    limit_z.CreateHighAttr().Set(-1.0)

    # 3. Limit and apply spring-dampers (Drive) to swing rotations (X and Y axes)
    for axis in ["rotX", "rotY"]:
        # Set soft limit angular range
        limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr().Set(-PhysicsConfig.BEND_LIMIT_DEG)
        limit.CreateHighAttr().Set(PhysicsConfig.BEND_LIMIT_DEG)
        
        # Apply the rotational spring drive to restore position back to 0.0 degrees
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(PhysicsConfig.STIFFNESS)
        drive.CreateDampingAttr().Set(PhysicsConfig.DAMPING)
        drive.CreateTargetPositionAttr().Set(0.0)


# ==============================================================================
# MAIN EXECUTION PIPELINE
# ==============================================================================

def main():
    output_path = get_output_usd_path()
    stage, stem_parent_path = setup_base_stage(output_path)
    
    print(f"[INFO] Building articulated trunk chain with {TrunkConfig.N_LINKS} links...")
    
    previous_link_path = None
    
    for i in range(TrunkConfig.N_LINKS):
        link_index = i + 1
        # Calculate current base Z height incrementing by height and gap at each step
        current_base_z = i * (TrunkConfig.HEIGHT + TrunkConfig.GAP)
        
        # Step A: Instantiate geometry and body parameters
        current_link_path = create_rigid_body_link(stage, stem_parent_path, link_index, current_base_z)
        
        # Step B: Instantiate kinematic constraints (Joints)
        if previous_link_path is None:
            # Anchor the very first link to the global world origin
            anchor_link_to_world(stage, current_link_path)
        else:
            # Wire up a flexible D6 joint linking the current segment to the previous one
            joint_name = f"Joint_{link_index-1:02d}_{link_index:02d}"
            create_d6_bending_joint(stage, previous_link_path, current_link_path, joint_name)
            
        previous_link_path = current_link_path

    # Save out the complete self-contained USD stage
    stage.GetRootLayer().Save()
    print(f"[OK] Articulated trunk exported successfully to: {output_path}")


if __name__ == "__main__":
    main()