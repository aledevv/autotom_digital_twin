from pxr import Gf, UsdPhysics

def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    """Convert a double-precision quaternion to float-precision,
    using the explicit (real, i, j, k) constructor to avoid any
    ambiguity in the Python bindings."""
    imag = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()),
                    float(imag[0]), float(imag[1]), float(imag[2]))


def _configure_drives(joint, stiff_xy, damp_xy, stiff_z, damp_z, bend_limit, lock_z):
    """Set up D6 joint limits and drives (translation locked, rotation limited)."""
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
        lim_z.CreateLowAttr().Set(1.0)
        lim_z.CreateHighAttr().Set(-1.0)
    else:
        # PhysX requires explicit low/high on every LimitAPI instance;
        # without them the solver sees an uninitialised "double pyramid" and errors.
        lim_z.CreateLowAttr().Set(-bend_limit)
        lim_z.CreateHighAttr().Set(bend_limit)
        drv_z = UsdPhysics.DriveAPI.Apply(prim, "rotZ")
        drv_z.CreateTypeAttr().Set("force")
        drv_z.CreateStiffnessAttr().Set(stiff_z)
        drv_z.CreateDampingAttr().Set(damp_z)
        drv_z.CreateTargetPositionAttr().Set(0.0)