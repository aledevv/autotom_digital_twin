"""
generate_cantilever_usda.py

Generates a single articulated trunk chain for a Cantilever Bending Test.
Based on Euler-Bernoulli beam theory.
"""

import os
import math
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# ==============================================================================
# CONFIGURATION & CONSTANTS
# ==============================================================================

# Lavoriamo in metri reali!
GLOBAL_SCALE = 1.0

class TrunkConfig:
    N_LINKS = 10
    HEIGHT = 0.015 * GLOBAL_SCALE  # 1.5 cm per link -> Totale = 15 cm
    RADIUS = 0.005 * GLOBAL_SCALE  # 5 mm raggio
    GAP = 0.0001 * GLOBAL_SCALE

class PhysicsConfig:
    BEND_LIMIT_DEG = 30.0

class BioConfig:
    YOUNG_MODULUS = 1.5e8  # 150 MPa (da benchmark 1)
    DAMPING_RATIO = 0.2
    PLANT_DENSITY = 1000.0

def calculate_physics_params(radius: float, length: float, mass: float) -> tuple[float, float]:
    I = (math.pi * (radius ** 4)) / 4.0
    K = (BioConfig.YOUNG_MODULUS * I) / length
    D = 2.0 * BioConfig.DAMPING_RATIO * math.sqrt(K * mass)
    return K, D

def compute_mass(radius: float, height: float) -> float:
    volume = math.pi * (radius ** 2) * height
    return BioConfig.PLANT_DENSITY * volume


# ==============================================================================
# PATH & STAGE HELPERS
# ==============================================================================

def get_output_usd_path() -> str:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "cantilever_benchmark.usda")


def setup_base_stage(path: str) -> tuple:
    # Controlla se il layer USD esiste già nella memoria della sessione Python
    existing_layer = Sdf.Layer.Find(path)
    if existing_layer:
        # Se esiste già in memoria, svuotalo per sovrascriverlo pulito
        existing_layer.Clear()
        stage = Usd.Stage.Open(existing_layer)
    else:
        # Se c'è un file su disco ma non in memoria, rimuovilo prima di creare il nuovo
        if os.path.exists(path):
            try:
                os.remove(path)
            except Exception:
                pass
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
    link_path = f"{parent_path}/Trunk_{index:02d}"
    
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
    joint_path = f"{link_path}/RootFixedJoint"
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def configure_joint_drives(joint: UsdPhysics.Joint, stiff: float, damp: float, bend_limit_deg: float):
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
        drive.CreateStiffnessAttr().Set(stiff)
        drive.CreateDampingAttr().Set(damp)
        drive.CreateTargetPositionAttr().Set(0.0)
        
    limit_z = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    limit_z.CreateLowAttr().Set(1.0)
    limit_z.CreateHighAttr().Set(-1.0)


def create_d6_bending_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str):
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
    
    configure_joint_drives(joint, stiff, damp, PhysicsConfig.BEND_LIMIT_DEG)


def build_stage(output_path: str) -> tuple[Usd.Stage, str]:
    stage, stem_parent_path = setup_base_stage(output_path)

    previous_link_path = None
    for i in range(TrunkConfig.N_LINKS):
        link_index = i + 1
        current_base_z = i * (TrunkConfig.HEIGHT + TrunkConfig.GAP)
        current_link_path = create_rigid_body_link(stage, stem_parent_path, link_index, current_base_z)

        if previous_link_path is None:
            anchor_link_to_world(stage, current_link_path)
        else:
            joint_name = f"Joint_{link_index-1:02d}_{link_index:02d}"
            create_d6_bending_joint(stage, previous_link_path, current_link_path, joint_name)
        previous_link_path = current_link_path

    return stage, stem_parent_path


def main():
    output_path = get_output_usd_path()
    stage, stem_parent_path = build_stage(output_path)
    stage.GetRootLayer().Save()
    print(f"[OK] Stage saved at: {output_path}")

if __name__ == "__main__":
    main()
