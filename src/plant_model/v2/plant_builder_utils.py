from pxr import Gf, UsdPhysics
import math

def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    """Convert a double-precision quaternion to float-precision,
    using the explicit (real, i, j, k) constructor to avoid any
    ambiguity in the Python bindings."""
    imag = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()),
                    float(imag[0]), float(imag[1]), float(imag[2]))


def _auto_mass(radius: float, length: float, density: float, mass_floor: float = 0.1) -> float:
    """Mass from cylinder volume + caps × density, with a minimum floor for thin segments."""
    cyl = math.pi * (radius ** 2) * length
    caps = (4.0 / 3.0) * math.pi * (radius ** 3)
    mass = density * (cyl + caps)
    return max(mass, mass_floor)


def _beam_stiffness(radius: float, length: float, youngs_modulus: float, beam_factor: float = math.pi / 4.0) -> float:
    """Euler-Bernoulli cantilever stiffness E*I/l for a circular cross-section."""
    return beam_factor * youngs_modulus * (radius ** 4) / max(length, 1e-4)


def _linear_damping(stiffness: float, damping_ratio: float) -> float:
    """Linear damping proportional to stiffness."""
    return damping_ratio * stiffness


def _configure_drives(joint, stiff_xy, damp_xy, stiff_z, damp_z, lock_z: bool = False, max_bend_angle: float = 60.0):
    prim = joint.GetPrim()
    for ax in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    # Reintroduce angular limits to prevent the solver from exploding on short/thick branches.
    # We use a max_bend_angle (e.g., 60 deg) to keep the joint within a safe cone.
    # The drive stiffness is responsible for holding the rest pose.
    for ax in ("rotX", "rotY"):
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr().Set(-max_bend_angle)
        lim.CreateHighAttr().Set(max_bend_angle)
        
        drv = UsdPhysics.DriveAPI.Apply(prim, ax)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiff_xy)
        drv.CreateDampingAttr().Set(damp_xy)
        drv.CreateTargetPositionAttr().Set(0.0)

    if lock_z:
        lim_z = UsdPhysics.LimitAPI.Apply(prim, "rotZ")
        lim_z.CreateLowAttr().Set(1.0)
        lim_z.CreateHighAttr().Set(-1.0) # lock it by setting low > high
    else:
        lim_z = UsdPhysics.LimitAPI.Apply(prim, "rotZ")
        lim_z.CreateLowAttr().Set(-max_bend_angle)
        lim_z.CreateHighAttr().Set(max_bend_angle)

        
    drv_z = UsdPhysics.DriveAPI.Apply(prim, "rotZ")
    drv_z.CreateTypeAttr().Set("force")
    drv_z.CreateStiffnessAttr().Set(stiff_z)
    drv_z.CreateDampingAttr().Set(damp_z)
    drv_z.CreateTargetPositionAttr().Set(0.0)