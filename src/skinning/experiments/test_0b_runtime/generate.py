"""
generate.py  —  Test 0B: UsdSkel runtime

Genera il file USDA con la posa neutra (tutti i bones a 0°, tubo dritto).
La posa viene poi animata a runtime da run.py.

Uso:
    ~/isaacsim/python.sh generate.py
"""

import math
import os
import sys

try:
    from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt
except ImportError:
    print("[ERROR] pxr non trovato. Usa ~/isaacsim/python.sh generate.py")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
OUTPUT_USD = os.path.join(OUTPUT_DIR, "test_0b_runtime.usda")

# Geometria
BONE_LENGTH     = 0.10   # 10 cm per bone
TUBE_RADIUS     = 0.012  # 12 mm raggio
RADIAL_SEGMENTS = 12
RINGS_PER_BONE  = 12     # più rings rispetto 0A per animazione più smooth

# Posa neutra (dritta) — run.py animerà i bones a runtime
BONE_ANGLES_DEG = [0.0, 0.0, 0.0]
NUM_BONES       = len(BONE_ANGLES_DEG)


# ─────────────────────────────────────────────────────────────────────────────
# MATEMATICA BONES  (identica a test_0a_static)
# ─────────────────────────────────────────────────────────────────────────────

def rot_x(deg: float) -> Gf.Rotation:
    return Gf.Rotation(Gf.Vec3d(1, 0, 0), deg)


def compute_bone_world_transforms() -> list[Gf.Matrix4d]:
    xforms: list[Gf.Matrix4d] = []
    pos = Gf.Vec3d(0, 0, 0)
    for angle in BONE_ANGLES_DEG:
        r = rot_x(angle)
        xforms.append(Gf.Matrix4d(r, pos))
        pos = pos + r.TransformDir(Gf.Vec3d(0, 0, BONE_LENGTH))
    return xforms


def compute_rest_transforms(world_xforms: list[Gf.Matrix4d]) -> list[Gf.Matrix4d]:
    rest: list[Gf.Matrix4d] = []
    for i, xf in enumerate(world_xforms):
        if i == 0:
            rest.append(xf)
        else:
            rest.append(world_xforms[i - 1].GetInverse() * xf)
    return rest


def mat_to_quatf(m: Gf.Matrix4d) -> Gf.Quatf:
    q  = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return Gf.Quatf(float(q.GetReal()),
                    Gf.Vec3f(float(im[0]), float(im[1]), float(im[2])))


def mat_to_trans(m: Gf.Matrix4d) -> Gf.Vec3f:
    t = m.ExtractTranslation()
    return Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))


# ─────────────────────────────────────────────────────────────────────────────
# MESH
# ─────────────────────────────────────────────────────────────────────────────

def build_tube_mesh(world_xforms: list[Gf.Matrix4d]):
    """Tubo continuo con blend lineare lungo il bone (stessa logica di test_0a)."""
    total_rings = NUM_BONES * RINGS_PER_BONE + 1
    points, normals, j_indices, j_weights = [], [], [], []

    for ring_idx in range(total_rings):
        t    = ring_idx / (total_rings - 1) * NUM_BONES
        seg  = min(int(t), NUM_BONES - 1)
        frac = t - seg

        xf      = world_xforms[seg]
        rot     = xf.ExtractRotation()
        origin  = xf.ExtractTranslation()
        z_world = rot.TransformDir(Gf.Vec3d(0, 0, 1))
        x_world = rot.TransformDir(Gf.Vec3d(1, 0, 0))
        y_world = rot.TransformDir(Gf.Vec3d(0, 1, 0))
        center  = origin + z_world * (frac * BONE_LENGTH)

        bone_a = seg
        bone_b = min(seg + 1, NUM_BONES - 1)
        w_a    = 1.0 - frac
        w_b    = frac
        if bone_a == bone_b:
            w_a, w_b = 1.0, 0.0

        for s in range(RADIAL_SEGMENTS):
            angle = 2.0 * math.pi * s / RADIAL_SEGMENTS
            n = x_world * math.cos(angle) + y_world * math.sin(angle)
            p = center + n * TUBE_RADIUS
            points.append( Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
            normals.append(Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])))
            j_indices.extend([bone_a, bone_b])
            j_weights.extend([w_a, w_b])

    face_counts, face_indices = [], []
    for r in range(total_rings - 1):
        b0 = r * RADIAL_SEGMENTS
        b1 = (r + 1) * RADIAL_SEGMENTS
        for s in range(RADIAL_SEGMENTS):
            s1 = (s + 1) % RADIAL_SEGMENTS
            v0, v1 = b0 + s, b0 + s1
            v2, v3 = b1 + s1, b1 + s
            face_counts.extend([3, 3])
            face_indices.extend([v0, v3, v2, v0, v2, v1])

    return points, face_counts, face_indices, normals, j_indices, j_weights


# ─────────────────────────────────────────────────────────────────────────────
# BUILD STAGE
# ─────────────────────────────────────────────────────────────────────────────

def build_stage(output_path: str = OUTPUT_USD) -> str:
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    stage = Usd.Stage.CreateNew(output_path)
    stage.SetMetadata("metersPerUnit", 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    skel_root = UsdSkel.Root.Define(stage, "/World/SkelRoot")
    skeleton  = UsdSkel.Skeleton.Define(stage, "/World/SkelRoot/Skeleton")

    joint_names = ["Bone0", "Bone0/Bone1", "Bone0/Bone1/Bone2"]
    skeleton.CreateJointsAttr().Set(Vt.TokenArray(joint_names))

    world_xforms = compute_bone_world_transforms()
    rest_xforms  = compute_rest_transforms(world_xforms)

    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(world_xforms))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest_xforms))

    # SkelAnimation: posa neutra iniziale — run.py la aggiornerà a runtime
    anim = UsdSkel.Animation.Define(stage, "/World/SkelRoot/SkelAnim")
    anim.CreateJointsAttr().Set(Vt.TokenArray(joint_names))
    anim.CreateTranslationsAttr().Set(
        Vt.Vec3fArray([mat_to_trans(xf) for xf in rest_xforms]))
    anim.CreateRotationsAttr().Set(
        Vt.QuatfArray([mat_to_quatf(xf) for xf in rest_xforms]))
    anim.CreateScalesAttr().Set(
        Vt.Vec3hArray([Gf.Vec3h(1, 1, 1)] * len(joint_names)))

    skel_binding = UsdSkel.BindingAPI.Apply(skeleton.GetPrim())
    skel_binding.CreateAnimationSourceRel().SetTargets([anim.GetPath()])

    # Mesh
    mesh = UsdGeom.Mesh.Define(stage, "/World/SkelRoot/TubeMesh")
    pts, fc, fi, nrm, ji, jw = build_tube_mesh(world_xforms)
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(pts))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.CreateNormalsAttr().Set(Vt.Vec3fArray(nrm))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray([Gf.Vec3f(0.28, 0.56, 0.19)]))

    mb = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    mb.CreateSkeletonRel().SetTargets([skeleton.GetPath()])
    mb.CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    mb.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))
    mb.CreateJointIndicesPrimvar(constant=False, elementSize=2).Set(Vt.IntArray(ji))
    mb.CreateJointWeightsPrimvar(constant=False, elementSize=2).Set(Vt.FloatArray(jw))

    # Ground plane
    gnd = UsdGeom.Mesh.Define(stage, "/World/Ground")
    s = 0.4
    gnd.CreatePointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-s, -s, -0.001), Gf.Vec3f(s, -s, -0.001),
        Gf.Vec3f( s,  s, -0.001), Gf.Vec3f(-s,  s, -0.001),
    ]))
    gnd.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    gnd.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    gnd.CreateDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.25, 0.22, 0.18)]))
    gnd.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)

    stage.Save()
    return output_path


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  Test 0B — generate.py (posa neutra per animazione runtime)")
    print(f"  Bones: {NUM_BONES}, tutti a 0° (tubo dritto)")
    print(f"  Raggio: {TUBE_RADIUS*1000:.0f} mm, "
          f"bone length: {BONE_LENGTH*100:.0f} cm, "
          f"rings/bone: {RINGS_PER_BONE}")
    print("=" * 60)

    path = build_stage()
    print(f"\n[OK] USDA: {path}")
