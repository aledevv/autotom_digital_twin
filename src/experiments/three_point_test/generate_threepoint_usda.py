"""
generate_threepoint_usda.py

Generates a horizontal articulated beam for a Three-Point Bending Test in Isaac Sim.

Design:
  - 20 links, each 1.5 cm, total span L = 30 cm along the world X axis.
  - Beam lies horizontally; gravity (-Z) acts as the transverse load.
  - Two SIMPLE SUPPORTS at the first and last link (D6 joint: translations locked,
    bending rotations FREE). This is NOT a fixed/clamped joint.
  - D6 bending joints between consecutive links with drive type = "acceleration".
  - SDR = L / D = 0.30 / 0.010 = 30 ≥ 20  ✅  (Anisimov et al. 2025 criterion)

Key difference from cantilever_test:
  1. Beam is HORIZONTAL (along X), not vertical (along Z).
  2. Supports are SIMPLE (free rotation), not clamped (fixed joint).
  3. Drive type is "acceleration", not "force".
  4. 20 links instead of 10 (for SDR requirement).

References:
  [1] Anisimov et al. (2025), Methods and Protocols 8(2), 32.
  [2] Shtein et al. (2020), Plants 9(6), 678.
"""

import os
import math
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf

# ==============================================================================
# CONFIGURATION
# ==============================================================================

from threepoint_config import TrunkConfig, PhysicsConfig, BioConfig


# ==============================================================================
# PHYSICS HELPERS
# ==============================================================================

def compute_mass(radius: float, height: float) -> float:
    """Volume of one cylindrical link × density."""
    return BioConfig.PLANT_DENSITY * math.pi * radius**2 * height


def calculate_physics_params(radius: float, link_length: float, mass: float) -> tuple[float, float]:
    """
    K_θ = E·I / L_link  [N·m/rad]  — rotational drive stiffness for one D6 joint.
    D   = 2 · ζ · √(K·m)           — damping coefficient.

    Returns:
        (stiffness, damping)
    """
    I = (math.pi * radius**4) / 4.0
    K = (BioConfig.YOUNG_MODULUS * I) / link_length
    D = 2.0 * BioConfig.DAMPING_RATIO * math.sqrt(K * mass)
    return K, D


# ==============================================================================
# PATH & STAGE HELPERS
# ==============================================================================

def get_output_usd_path() -> str:
    script_dir   = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    output_dir   = os.path.join(project_root, "data", "usd_models")
    os.makedirs(output_dir, exist_ok=True)
    return os.path.join(output_dir, "threepoint_benchmark.usda")


def setup_base_stage(path: str) -> tuple:
    existing_layer = Sdf.Layer.Find(path)
    if existing_layer:
        existing_layer.Clear()
        stage = Usd.Stage.Open(existing_layer)
    else:
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
# GEOMETRY & JOINT CREATION
# ==============================================================================

def create_rigid_body_link(stage: Usd.Stage, parent_path: str, index: int, x_pos: float) -> str:
    """
    Create one cylindrical rigid body link at world X position x_pos.
    The cylinder axis is along X (horizontal beam), rendered as a short rod.

    Args:
        index: 1-based link number (used for naming: Link_01 … Link_20)
        x_pos: X coordinate of the link's Xform origin in world space [m]
    """
    link_path = f"{parent_path}/Link_{index:02d}"

    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.AddTranslateOp().Set(Gf.Vec3d(x_pos, 0.0, 0.0))

    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(compute_mass(TrunkConfig.RADIUS, TrunkConfig.HEIGHT))

    # Cylinder with axis along X (horizontal beam direction)
    cyl_path = f"{link_path}/Cylinder"
    cylinder  = UsdGeom.Cylinder.Define(stage, cyl_path)
    cylinder.GetRadiusAttr().Set(TrunkConfig.RADIUS)
    cylinder.GetHeightAttr().Set(TrunkConfig.HEIGHT)
    cylinder.GetAxisAttr().Set(UsdGeom.Tokens.x)
    # Center the cylinder: its midpoint is at X = HEIGHT/2 relative to the link origin
    cylinder.AddTranslateOp().Set(Gf.Vec3d(TrunkConfig.HEIGHT / 2.0, 0.0, 0.0))
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())

    return link_path


def create_simple_support(stage: Usd.Stage, stem_path: str, link_path: str, name: str, world_x: float):
    """
    Create a SIMPLE SUPPORT (pin joint) between the world and a link.

    Constraints:
      - transX, transY, transZ: LOCKED  (the support does not translate)
      - rotY, rotZ             : FREE   (beam can rotate → bending allowed)
      - rotX                   : LOCKED (no torsion around beam axis)

    This is the key difference from a FixedJoint (clamped support), which
    would change the denominator in the deflection formula from 48 to 192.

    IMPORTANT: the joint must be a child of the link prim (body1), NOT inside
    a separate Xform under the ArticulationRoot. Placing it outside the link
    (but inside the articulation) leaves body0 ambiguous and causes PhysX to
    produce NaN transforms. Pattern confirmed from cantilever_benchmark.usda.
    """
    # Joint lives inside the link prim — same pattern as cantilever RootFixedJoint
    joint_path = f"{link_path}/{name}"

    joint = UsdPhysics.Joint.Define(stage, joint_path)
    # body0 = world (not set → implicit world anchor); body1 = link
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])

    # World-frame attachment point: where in the world the pin is located.
    # For body0 (world), localPos0 is in world coords.
    # For body1 (link), localPos1 is in link-local coords (link origin = (0,0,0)).
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(world_x, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    # Lock all translations (support stays in place)
    for axis in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)    # low > high → locked in PhysX convention
        lim.CreateHighAttr().Set(-1.0)

    # Free bending rotations: rotY (bending in XZ plane) and rotZ (bending in XY plane)
    for axis in ["rotY", "rotZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(-180.0)
        lim.CreateHighAttr().Set(180.0)

    # Lock axial torsion (rotX = rotation around beam axis)
    lim_rx = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotX")
    lim_rx.CreateLowAttr().Set(1.0)
    lim_rx.CreateHighAttr().Set(-1.0)


def create_d6_bending_joint(stage: Usd.Stage, parent_link: str, child_link: str, name: str):
    """
    Create a D6 bending joint between two consecutive horizontal links.

    Drive axes: rotY, rotZ  (bending in XZ and XY planes)
    Locked    : transX/Y/Z, rotX  (rigid in translation and torsion)
    Drive type: "acceleration"  (NOT "force", to avoid mass-scaling artifacts)
    """
    joint_path = f"{child_link}/{name}"
    joint = UsdPhysics.Joint.Define(stage, joint_path)

    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_link)])

    # Joint connects at the right end of parent (x = HEIGHT) and left end of child (x = 0).
    # LocalPos0 = HEIGHT, not HEIGHT+GAP: the gap is a collision clearance, not a joint offset.
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(TrunkConfig.HEIGHT, 0.0, 0.0))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    mass   = compute_mass(TrunkConfig.RADIUS, TrunkConfig.HEIGHT)
    stiff, damp = calculate_physics_params(TrunkConfig.RADIUS, TrunkConfig.HEIGHT, mass)

    # Lock translations
    for axis in ["transX", "transY", "transZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    # Bending drives on rotY (XZ plane) and rotZ (XY plane) — "acceleration" type
    for axis in ["rotY", "rotZ"]:
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
        lim.CreateLowAttr().Set(-PhysicsConfig.BEND_LIMIT_DEG)
        lim.CreateHighAttr().Set(PhysicsConfig.BEND_LIMIT_DEG)

        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
        drive.CreateTypeAttr().Set("acceleration")   # ← critical: NOT "force"
        drive.CreateStiffnessAttr().Set(stiff)
        drive.CreateDampingAttr().Set(damp)
        drive.CreateTargetPositionAttr().Set(0.0)

    # Lock axial torsion
    lim_rx = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotX")
    lim_rx.CreateLowAttr().Set(1.0)
    lim_rx.CreateHighAttr().Set(-1.0)


# ==============================================================================
# STAGE BUILDER
# ==============================================================================

def build_stage(output_path: str) -> tuple[Usd.Stage, str]:
    stage, stem_path = setup_base_stage(output_path)

    step = TrunkConfig.HEIGHT + TrunkConfig.GAP
    link_paths = []

    # Create all links along X axis
    for i in range(TrunkConfig.N_LINKS):
        x_pos = i * step
        link_path = create_rigid_body_link(stage, stem_path, i + 1, x_pos)
        link_paths.append(link_path)

    # Simple support at first link (x=0)
    create_simple_support(stage, stem_path, link_paths[0],  "SupportA", world_x=0.0)
    # Simple support at last link
    last_x = (TrunkConfig.N_LINKS - 1) * step
    create_simple_support(stage, stem_path, link_paths[-1], "SupportB", world_x=last_x)

    # D6 bending joints between consecutive links (NOT at the support links)
    for i in range(1, TrunkConfig.N_LINKS):
        joint_name = f"Joint_{i:02d}_{i+1:02d}"
        create_d6_bending_joint(stage, link_paths[i - 1], link_paths[i], joint_name)

    return stage, stem_path


def main():
    output_path = get_output_usd_path()
    stage, _ = build_stage(output_path)
    stage.GetRootLayer().Save()

    span = TrunkConfig.total_span()
    sdr  = span / (2.0 * TrunkConfig.RADIUS)
    center_idx = TrunkConfig.center_link_index()
    print(f"[OK] Stage saved at: {output_path}")
    print(f"     N_LINKS={TrunkConfig.N_LINKS}, span={span*100:.1f} cm, "
          f"SDR={sdr:.1f}, center link=Link_{center_idx+1:02d}")
    print(f"     E={BioConfig.YOUNG_MODULUS/1e6:.0f} MPa, "
          f"drive='acceleration'")


if __name__ == "__main__":
    main()
