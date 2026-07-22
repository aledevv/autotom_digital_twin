from pxr import Usd, UsdGeom, UsdPhysics, Sdf
import math

stage = Usd.Stage.Open("data/usd_models/builder_visual_test.usda")

# Check root scale
stem = stage.GetPrimAtPath("/World/Stem")
xform = UsdGeom.Xform(stem)
scale = xform.GetOrderedXformOps()[0].Get() if xform.GetOrderedXformOps() else None
print(f"Stem Scale: {scale}")

for path in ["/World/Stem/T01", "/World/Stem/LB01/Cylinder", "/World/Stem/LB01", "/World/Stem/LB01/Joint"]:
    prim = stage.GetPrimAtPath(path)
    if not prim: continue
    print(f"\n--- {path} ---")
    if prim.IsA(UsdGeom.Cylinder):
        cyl = UsdGeom.Cylinder(prim)
        print(f"Radius: {cyl.GetRadiusAttr().Get()}")
        print(f"Height: {cyl.GetHeightAttr().Get()}")
    if prim.HasAPI(UsdPhysics.MassAPI):
        mass_api = UsdPhysics.MassAPI(prim)
        print(f"Mass: {mass_api.GetMassAttr().Get()}")
        print(f"Density: {mass_api.GetDensityAttr().Get()}")
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        col_api = UsdPhysics.CollisionAPI(prim)
        print("Has Collision API")
    if prim.IsA(UsdPhysics.Joint):
        drive_x = UsdPhysics.DriveAPI(prim, "rotX")
        if drive_x:
            print(f"Stiffness rotX: {drive_x.GetStiffnessAttr().Get()}")
            print(f"Damping rotX: {drive_x.GetDampingAttr().Get()}")
        
