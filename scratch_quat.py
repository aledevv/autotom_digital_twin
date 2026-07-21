from pxr import Gf
rot = Gf.Rotation(Gf.Vec3d(1,0,0), 45)
quatd = rot.GetQuat()
quatf = Gf.Quatf(quatd)
print(f"quatd: {quatd}")
print(f"quatf: {quatf}")
print(f"quatf.GetReal(): {quatf.GetReal()}")
