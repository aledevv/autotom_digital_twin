"""
generate.py  —  Test 0A: UsdSkel statico

Genera il file USDA:
  - SkelRoot
  - Skeleton: 3 bones (Bone0 → Bone1 → Bone2) inclinati a 0°/25°/45° attorno X
  - SkelAnimation: posa statica (uguale alla rest pose)
  - TubeMesh: tubo continuo skinnato ai 3 bones

Non richiede Isaac Sim. Usa solo pxr.

Uso:
    uv run generate.py
    oppure
    ~/isaacsim/python.sh generate.py
"""

import math
import os
import sys

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdSkel, Vt
except ImportError:
    print("[ERROR] pxr non trovato. Usa ~/isaacsim/python.sh generate.py")
    sys.exit(1)

# ─────────────────────────────────────────────────────────────────────────────
# CONFIG
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR  = os.path.join(SCRIPT_DIR, "output")
OUTPUT_USD  = os.path.join(OUTPUT_DIR, "test_0a_static.usda")

BONE_LENGTH      = 0.10   # 10 cm per bone
TUBE_RADIUS      = 0.012  # 12 mm raggio
RADIAL_SEGMENTS  = 12     # vertici per anello
RINGS_PER_BONE   = 8      # anelli longitudinali per bone (più = più smooth)

# Rotazioni cumulative attorno X (gradi) per ogni bone
BONE_ANGLES_DEG = [0.0, 25.0, 45.0]
NUM_BONES       = len(BONE_ANGLES_DEG)


# ─────────────────────────────────────────────────────────────────────────────
# MATEMATICA BONES
# ─────────────────────────────────────────────────────────────────────────────

def rot_x(deg: float) -> Gf.Rotation:
    return Gf.Rotation(Gf.Vec3d(1, 0, 0), deg)


def compute_bone_world_transforms() -> list[Gf.Matrix4d]:
    """
    Ritorna la world-space Matrix4d per ogni bone.
    Bone0 è all'origine, allineato lungo +Z.
    Ogni bone nasce alla tip del precedente con rotazione cumulativa attorno X.
    """
    xforms: list[Gf.Matrix4d] = []
    pos = Gf.Vec3d(0, 0, 0)

    for i, angle in enumerate(BONE_ANGLES_DEG):
        rot = rot_x(angle)
        xforms.append(Gf.Matrix4d(rot, pos))

        # Tip del bone corrente → origine del prossimo
        tip_local = Gf.Vec3d(0, 0, BONE_LENGTH)
        pos = pos + rot.TransformDir(tip_local)

    return xforms


def compute_rest_transforms(world_xforms: list[Gf.Matrix4d]) -> list[Gf.Matrix4d]:
    """
    Local transforms per UsdSkel restTransforms.
    local_i = parent_world_inv * child_world
    """
    rest: list[Gf.Matrix4d] = []
    for i, xf in enumerate(world_xforms):
        if i == 0:
            # Il parent è SkelRoot (identità)
            rest.append(xf)
        else:
            local = world_xforms[i - 1].GetInverse() * xf
            rest.append(local)
    return rest


def mat_to_quatf(m: Gf.Matrix4d) -> Gf.Quatf:
    q = m.ExtractRotationQuat()
    im = q.GetImaginary()
    return Gf.Quatf(float(q.GetReal()),
                    Gf.Vec3f(float(im[0]), float(im[1]), float(im[2])))


def mat_to_trans(m: Gf.Matrix4d) -> Gf.Vec3f:
    t = m.ExtractTranslation()
    return Gf.Vec3f(float(t[0]), float(t[1]), float(t[2]))


# ─────────────────────────────────────────────────────────────────────────────
# GENERAZIONE MESH
# ─────────────────────────────────────────────────────────────────────────────

def build_tube_mesh(world_xforms: list[Gf.Matrix4d]):
    """
    Tubo continuo lungo la centerline piecewise-linear dei bones.

    La centerline per il segmento i va dal punto origine di world_xforms[i]
    lungo l'asse +Z locale per BONE_LENGTH.

    I joint weights blendano linearmente:
      vertice al parametro t ∈ [i, i+1] → w(bone_i) = 1-frac, w(bone_{i+1}) = frac
    """
    # Numero totale di anelli = NUM_BONES*RINGS_PER_BONE + 1 (cap finale)
    total_rings = NUM_BONES * RINGS_PER_BONE + 1

    points:       list[Gf.Vec3f] = []
    normals:      list[Gf.Vec3f] = []
    j_indices:    list[int]      = []
    j_weights:    list[float]    = []

    for ring_idx in range(total_rings):
        # Parametro globale t ∈ [0, NUM_BONES]
        t = ring_idx / (total_rings - 1) * NUM_BONES

        # Segmento corrente e frazione dentro di esso
        seg   = min(int(t), NUM_BONES - 1)
        frac  = t - seg

        # Posizione e rotazione del ring lungo la centerline del segmento
        xf    = world_xforms[seg]
        rot   = xf.ExtractRotation()          # Gf.Rotation
        origin = xf.ExtractTranslation()

        # Pos along segment
        z_world = rot.TransformDir(Gf.Vec3d(0, 0, 1))
        ring_center = origin + z_world * (frac * BONE_LENGTH)

        # Assi tangenziali per il ring
        x_world = rot.TransformDir(Gf.Vec3d(1, 0, 0))
        y_world = rot.TransformDir(Gf.Vec3d(0, 1, 0))

        # ── Joint weights ────────────────────────────────────────────────────
        # STRATEGIA ATTUALE (Test 0A): blend lineare lungo l'intero segmento.
        #   w(bone_i) = 1 - frac,  w(bone_{i+1}) = frac,  con frac ∈ [0, 1]
        #
        # Questo è sufficiente per verificare che lo skinning funzioni, ma non è
        # il comportamento desiderato per la pianta reale.
        #
        # STRATEGIA FUTURA (pianta): blend-zone localizzata vicino al D6.
        #   - bone_i al 100% per la maggior parte del segmento
        #   - blend solo in una finestra [1-α, 1] vicino al giunto
        #   Esempio con BLEND_FRAC = 0.25:
        #     frac < 0.75  → w_a = 1.0, w_b = 0.0  (zona rigida)
        #     frac ≥ 0.75  → blend smooth verso bone_{i+1}
        #   Questo riduce il candy-wrapper artifact a bone count elevato.
        #   Da implementare e comparare in Test 0B o nel passaggio a Phase 1.
        bone_a = seg
        bone_b = min(seg + 1, NUM_BONES - 1)
        w_a    = 1.0 - frac
        w_b    = frac
        if bone_a == bone_b:
            w_a, w_b = 1.0, 0.0

        for s in range(RADIAL_SEGMENTS):
            angle   = 2.0 * math.pi * s / RADIAL_SEGMENTS
            cos_a   = math.cos(angle)
            sin_a   = math.sin(angle)

            n = x_world * cos_a + y_world * sin_a  # normale radiale
            p = ring_center + n * TUBE_RADIUS

            points.append( Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])))
            normals.append(Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])))
            j_indices.extend([bone_a, bone_b])
            j_weights.extend([w_a,    w_b])

    # Connetti gli anelli con triangoli
    face_counts:  list[int] = []
    face_indices: list[int] = []

    for r in range(total_rings - 1):
        base0 = r       * RADIAL_SEGMENTS
        base1 = (r + 1) * RADIAL_SEGMENTS
        for s in range(RADIAL_SEGMENTS):
            s1 = (s + 1) % RADIAL_SEGMENTS
            v0, v1 = base0 + s,  base0 + s1
            v2, v3 = base1 + s1, base1 + s
            face_counts.extend([3, 3])
            face_indices.extend([v0, v3, v2,
                                  v0, v2, v1])

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

    # ── SkelRoot ────────────────────────────────────────────────────────────
    skel_root = UsdSkel.Root.Define(stage, "/World/SkelRoot")

    # ── Skeleton ─────────────────────────────────────────────────────────────
    skeleton = UsdSkel.Skeleton.Define(stage, "/World/SkelRoot/Skeleton")

    joint_names = ["Bone0", "Bone0/Bone1", "Bone0/Bone1/Bone2"]
    skeleton.CreateJointsAttr().Set(Vt.TokenArray(joint_names))

    world_xforms = compute_bone_world_transforms()
    rest_xforms  = compute_rest_transforms(world_xforms)

    skeleton.CreateBindTransformsAttr().Set(Vt.Matrix4dArray(world_xforms))
    skeleton.CreateRestTransformsAttr().Set(Vt.Matrix4dArray(rest_xforms))

    # ── SkelAnimation (posa statica = rest pose) ─────────────────────────────
    anim = UsdSkel.Animation.Define(stage, "/World/SkelRoot/SkelAnim")
    anim.CreateJointsAttr().Set(Vt.TokenArray(joint_names))
    anim.CreateTranslationsAttr().Set(
        Vt.Vec3fArray([mat_to_trans(xf) for xf in rest_xforms]))
    anim.CreateRotationsAttr().Set(
        Vt.QuatfArray([mat_to_quatf(xf) for xf in rest_xforms]))
    anim.CreateScalesAttr().Set(
        Vt.Vec3hArray([Gf.Vec3h(1, 1, 1)] * len(joint_names)))

    # Binding Skeleton → SkelAnimation
    skel_binding = UsdSkel.BindingAPI.Apply(skeleton.GetPrim())
    skel_binding.CreateAnimationSourceRel().SetTargets([anim.GetPath()])

    # ── TubeMesh ──────────────────────────────────────────────────────────────
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
        Vt.Vec3fArray([Gf.Vec3f(0.28, 0.56, 0.19)]))  # verde fusto pomodoro

    # SkelBindingAPI sulla mesh
    mb = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    mb.CreateSkeletonRel().SetTargets([skeleton.GetPath()])
    mb.CreateAnimationSourceRel().SetTargets([anim.GetPath()])
    mb.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1.0))

    idx_pv = mb.CreateJointIndicesPrimvar(constant=False, elementSize=2)
    idx_pv.Set(Vt.IntArray(ji))

    wgt_pv = mb.CreateJointWeightsPrimvar(constant=False, elementSize=2)
    wgt_pv.Set(Vt.FloatArray(jw))

    # ── Ground plane (riferimento visivo) ─────────────────────────────────────
    gnd = UsdGeom.Mesh.Define(stage, "/World/Ground")
    s = 0.4
    gnd.CreatePointsAttr().Set(Vt.Vec3fArray([
        Gf.Vec3f(-s, -s, -0.001), Gf.Vec3f(s, -s, -0.001),
        Gf.Vec3f( s,  s, -0.001), Gf.Vec3f(-s, s, -0.001),
    ]))
    gnd.CreateFaceVertexCountsAttr().Set(Vt.IntArray([4]))
    gnd.CreateFaceVertexIndicesAttr().Set(Vt.IntArray([0, 1, 2, 3]))
    gnd.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray([Gf.Vec3f(0.25, 0.22, 0.18)]))
    gnd.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)

    stage.Save()
    return output_path


# ─────────────────────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  Test 0A — generate.py")
    print(f"  Bones: {NUM_BONES}, angoli: {BONE_ANGLES_DEG} deg")
    print(f"  Raggio tubo: {TUBE_RADIUS*1000:.0f} mm, "
          f"lunghezza bone: {BONE_LENGTH*100:.0f} cm")
    print("=" * 60)

    world_xforms = compute_bone_world_transforms()
    for i, xf in enumerate(world_xforms):
        t = xf.ExtractTranslation()
        print(f"  Bone{i} world pos: ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})")

    path = build_stage()
    print(f"\n[OK] USDA: {path}")
    print("     Apri con: usdview output/test_0a_static.usda")
