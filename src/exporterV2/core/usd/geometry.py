"""
geometry.py - USD Geometry Creation

Creates rigid body links with cylindrical collision shapes.
"""

from pxr import Usd, UsdGeom, Gf, UsdPhysics


def create_rigid_segment(
    stage,
    stem_path: str,
    link_name: str,
    radius: float,
    height: float,
    world_pos: Gf.Vec3d,
    mass: float,
    orientation: Gf.Quatf = None,
) -> str:
    """
    Create one rigid cylinder link directly under stem_path.

    Args:
        stage: USD stage
        stem_path: Parent path for all links
        link_name: Name for this link
        radius: Cylinder radius [m]
        height: Cylinder height [m]
        world_pos: World-space position
        mass: Link mass [kg]
        orientation: Optional world-space orientation (None = identity)

    Returns:
        USD path of the created link
    """
    link_path = f"{stem_path}/{link_name}"

    xform = UsdGeom.Xform.Define(stage, link_path)
    xform.AddTranslateOp().Set(world_pos)
    if orientation is not None:
        xform.AddOrientOp().Set(orientation)

    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)
    # Set COM to cylinder's geometric center to avoid spurious torques
    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, height / 2.0))

    cyl = UsdGeom.Cylinder.Define(stage, f"{link_path}/Cylinder")
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))

    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    return link_path
