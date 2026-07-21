import math
from pxr import Gf

rot_z = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), 90.0)
tilt = Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), -45.0)

# Try tilt * rot_z
rot1 = tilt * rot_z
v1 = rot1.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))

# Try rot_z * tilt
rot2 = rot_z * tilt
v2 = rot2.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))

print("rot1 (tilt * rot_z) applied to Z-axis:", v1)
print("rot2 (rot_z * tilt) applied to Z-axis:", v2)
