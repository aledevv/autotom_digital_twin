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
    collision_enabled: bool = True,
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
    mass_api.CreateDiagonalInertiaAttr().Set(
        Gf.Vec3f(
            mass * (3.0 * radius * radius + height * height) / 12.0,
            mass * (3.0 * radius * radius + height * height) / 12.0,
            0.5 * mass * radius * radius,
        )
    )

    cyl = UsdGeom.Cylinder.Define(stage, f"{link_path}/Cylinder")
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))

    if collision_enabled:
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

    return link_path


def create_sphere_rigid_body(
    stage,
    parent_path: str,
    sphere_name: str,
    radius: float,
    world_pos: Gf.Vec3d,
    mass: float,
    orientation: Gf.Quatf = None,
    rigid_body_enabled: bool = True,
    collision_enabled: bool = True,
    visible: bool = True,
    kinematic_enabled: bool = False,
) -> str:
    """
    Create one rigid sphere body directly under parent_path.
    
    Used for tomatoes and other spherical objects that need physics.
    The sphere is a rigid body with collision enabled.

    Args:
        stage: USD stage
        parent_path: Parent path for the sphere (e.g., /World/Stem)
        sphere_name: Name for this sphere (e.g., "Tomato_01")
        radius: Sphere radius [m]
        world_pos: World-space position (center of sphere)
        mass: Sphere mass [kg]
        orientation: Optional world-space orientation (None = identity)

    Returns:
        USD path of the created sphere
    """
    sphere_path = f"{parent_path}/{sphere_name}"

    xform = UsdGeom.Xform.Define(stage, sphere_path)
    xform.AddTranslateOp().Set(world_pos)
    if orientation is not None:
        xform.AddOrientOp().Set(orientation)

    rigid_body = UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    rigid_body.CreateRigidBodyEnabledAttr().Set(rigid_body_enabled)
    rigid_body.CreateKinematicEnabledAttr().Set(kinematic_enabled)
    mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
    mass_api.CreateMassAttr().Set(mass)
    # Sphere COM is at its center (no offset needed)
    mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))

    sphere = UsdGeom.Sphere.Define(stage, f"{sphere_path}/Sphere")
    sphere.GetRadiusAttr().Set(radius)
    # Sphere doesn't need translation offset (already centered at xform position)

    collision = UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    collision.CreateCollisionEnabledAttr().Set(collision_enabled)
    if not visible:
        xform.MakeInvisible()

    return sphere_path


def create_attached_sphere_collider(
    stage,
    parent_body_path: str,
    sphere_name: str,
    radius: float,
    local_center: Gf.Vec3f,
) -> str:
    """Create tomato geometry as part of an existing compound rigid body."""
    attached_path = f"{parent_body_path}/{sphere_name}_Attached"
    xform = UsdGeom.Xform.Define(stage, attached_path)
    xform.AddTranslateOp().Set(local_center)
    sphere = UsdGeom.Sphere.Define(stage, f"{attached_path}/Sphere")
    sphere.GetRadiusAttr().Set(radius)
    collision = UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    collision.CreateCollisionEnabledAttr().Set(True)
    return attached_path
