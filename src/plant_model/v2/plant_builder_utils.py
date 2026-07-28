from pxr import Gf, UsdPhysics
import math

def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    """Convert a double-precision quaternion to float-precision,
    using the explicit (real, i, j, k) constructor to avoid any
    ambiguity in the Python bindings."""
    imag = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()),
                    float(imag[0]), float(imag[1]), float(imag[2]))


# plant_builder_utils.py

def _auto_mass(radius: float, length: float, density: float, mass_floor: float = 0.005) -> float:
    """Mass from cylinder volume × density, with a minimum floor for thin segments."""
    volume = math.pi * (radius ** 2) * length
    return max(volume * density, mass_floor)

def _critical_damping(stiffness: float, mass: float) -> float:
    return 2.0 * math.sqrt(stiffness * mass)
    

def _configure_drives(joint, stiff_xy, damp_xy, stiff_z, damp_z,
                       bend_limit, lock_z, twist_limit=15.0):
    prim = joint.GetPrim()
    for ax in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    for ax in ("rotX", "rotY"):
        lim = UsdPhysics.LimitAPI.Apply(prim, ax)
        lim.CreateLowAttr().Set(-bend_limit)
        lim.CreateHighAttr().Set(bend_limit)
        drv = UsdPhysics.DriveAPI.Apply(prim, ax)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(stiff_xy)
        drv.CreateDampingAttr().Set(damp_xy)
        drv.CreateTargetPositionAttr().Set(0.0)

    lim_z = UsdPhysics.LimitAPI.Apply(prim, "rotZ")
    if lock_z:
        # PhysX convention: low > high → DOF is locked (same as transX/Y/Z above)
        lim_z.CreateLowAttr().Set(1.0)
        lim_z.CreateHighAttr().Set(-1.0)
    else:
        lim_z.CreateLowAttr().Set(-twist_limit)
        lim_z.CreateHighAttr().Set(twist_limit)
        drv_z = UsdPhysics.DriveAPI.Apply(prim, "rotZ")
        drv_z.CreateTypeAttr().Set("force")
        drv_z.CreateStiffnessAttr().Set(stiff_z)
        drv_z.CreateDampingAttr().Set(damp_z)
        drv_z.CreateTargetPositionAttr().Set(0.0)