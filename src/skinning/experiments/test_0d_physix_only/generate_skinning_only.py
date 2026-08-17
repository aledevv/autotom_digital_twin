"""
generate_skinning_only.py — Test 0D: UsdSkel standalone

Obiettivo:
  - NESSUNA fisica
  - NESSUN bridge PhysX -> Skeleton
  - Skeleton di 3 bones con la stessa convenzione geometrica della catena PhysX
  - TubeMesh indipendente, skinnata
  - Animazione manuale: dritta -> piegata -> dritta

Se questo test funziona, il passo successivo è sostituire l'animazione manuale
con le pose lette dai rigid links PhysX.
"""

import math
import os
import sys

try:
    from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt
except ImportError:
    print("[ERROR] pxr non trovato. Usa ~/isaacsim/python.sh generate_skinning_only.py")
    sys.exit(1)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
OUTPUT_USD = os.path.join(OUTPUT_DIR, "test_0d_skinning_only.usda")

# Stessa geometria/convenzione del checkpoint PhysX
NUM_BONES = 3
LINK_HEIGHT = 0.10
GAP = 0.001
TUBE_RADIUS = 0.012
BASE_Z = 0.10

# Mesh
RADIAL_SEGMENTS = 12
RINGS_PER_LINK = 10

# Una piccola zona di blend attorno ad ogni joint.
# Fuori da questa zona la mesh segue rigidamente una sola bone.
BLEND_HALF_WIDTH = LINK_HEIGHT * 0.15

# Test manuale
FPS = 60.0
BEND_DEG = 30.0


def _quat_x(deg: float) -> Gf.Quatf:
    a = math.radians(deg) * 0.5
    return Gf.Quatf(
        math.cos(a),
        Gf.Vec3f(math.sin(a), 0.0, 0.0),
    )


def _matrix_rot_trans(rot_deg_x: float, t: Gf.Vec3d) -> Gf.Matrix4d:
    m = Gf.Matrix4d(1.0)
    m.SetTransform(
        Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), rot_deg_x),
        t,
    )
    return m


def _joint_names():
    names = ["Bone0"]
    for i in range(1, NUM_BONES):
        names.append(names[-1] + f"/Bone{i}")
    return names


def _rest_local_transforms():
    """
    Skeleton locale:
      Bone0:
        translation = (0, 0, BASE_Z)
        rotation    = -90° X  -> asse locale +Z diventa world +Y

      Bone1..:
        translation = (0, 0, LINK_HEIGHT + GAP)
        rotation    = identity

    Quindi la catena delle bones usa la stessa convenzione dei rigid links:
    origine della bone = BASE del segmento.
    """
    step = LINK_HEIGHT + GAP

    out = [
        _matrix_rot_trans(
            -90.0,
            Gf.Vec3d(0.0, 0.0, BASE_Z),
        )
    ]

    for _ in range(1, NUM_BONES):
        out.append(
            _matrix_rot_trans(
                0.0,
                Gf.Vec3d(0.0, 0.0, step),
            )
        )

    return out


def _bind_world_transforms():
    """
    Pose concatenata a bind time nello spazio dello SkelRoot.

    Le origini delle bones coincidono con le basi dei corrispondenti
    rigid link del checkpoint PhysX:
        bone 0 -> (0, 0.000, BASE_Z)
        bone 1 -> (0, 0.101, BASE_Z)
        bone 2 -> (0, 0.202, BASE_Z)
    """
    step = LINK_HEIGHT + GAP

    return [
        _matrix_rot_trans(
            -90.0,
            Gf.Vec3d(0.0, i * step, BASE_Z),
        )
        for i in range(NUM_BONES)
    ]


def _weights_for_y(y: float):
    """
    Due influenze max per vertice.

    Lontano dai joint:
      peso 1.0 sulla bone del segmento.

    Attorno a ciascun joint:
      blend lineare parent -> child.

    Questo mantiene i tratti quasi rigidi ma elimina lo spigolo netto
    esattamente sul joint.
    """
    step = LINK_HEIGHT + GAP

    for child in range(1, NUM_BONES):
        joint_y = child * step
        lo = joint_y - BLEND_HALF_WIDTH
        hi = joint_y + BLEND_HALF_WIDTH

        if lo <= y <= hi:
            u = (y - lo) / (hi - lo)
            u = max(0.0, min(1.0, u))
            return child - 1, child, 1.0 - u, u

    # Fuori dalle zone di blend: bone del tratto corrente.
    bone = 0
    for i in range(1, NUM_BONES):
        if y >= i * step:
            bone = i
        else:
            break

    return bone, bone, 1.0, 0.0


def build_tube_mesh_data():
    """
    Tubo rettilineo in bind pose, già nello spazio dello SkelRoot.

    La centerline è:
        (0, y, BASE_Z)

    e coincide con la catena PhysX del checkpoint precedente.
    """
    total_length = (NUM_BONES - 1) * (LINK_HEIGHT + GAP) + LINK_HEIGHT
    total_rings = NUM_BONES * RINGS_PER_LINK + 1

    points = []
    joint_indices = []
    joint_weights = []

    for ring in range(total_rings):
        u = ring / (total_rings - 1)
        y = u * total_length

        b0, b1, w0, w1 = _weights_for_y(y)

        for s in range(RADIAL_SEGMENTS):
            a = 2.0 * math.pi * s / RADIAL_SEGMENTS

            # Tubo lungo Y -> sezione nel piano XZ.
            x = math.cos(a) * TUBE_RADIUS
            z = BASE_Z + math.sin(a) * TUBE_RADIUS

            points.append(Gf.Vec3f(float(x), float(y), float(z)))
            joint_indices.extend([b0, b1])
            joint_weights.extend([w0, w1])

    face_counts = []
    face_indices = []

    for r in range(total_rings - 1):
        row0 = r * RADIAL_SEGMENTS
        row1 = (r + 1) * RADIAL_SEGMENTS

        for s in range(RADIAL_SEGMENTS):
            sn = (s + 1) % RADIAL_SEGMENTS

            v00 = row0 + s
            v01 = row0 + sn
            v10 = row1 + s
            v11 = row1 + sn

            face_counts.extend([3, 3])
            face_indices.extend([
                v00, v10, v11,
                v00, v11, v01,
            ])

    return (
        points,
        face_counts,
        face_indices,
        joint_indices,
        joint_weights,
    )


def build_skinning(stage):
    skel_root = UsdSkel.Root.Define(stage, "/World/StemVisual/SkelRoot")
    skeleton = UsdSkel.Skeleton.Define(
        stage,
        "/World/StemVisual/SkelRoot/Skeleton",
    )
    anim = UsdSkel.Animation.Define(
        stage,
        "/World/StemVisual/SkelRoot/SkelAnim",
    )

    names = _joint_names()
    rest_local = _rest_local_transforms()
    bind_world = _bind_world_transforms()

    # Skeleton topology + bind/rest pose
    skeleton.CreateJointsAttr().Set(Vt.TokenArray(names))
    skeleton.CreateRestTransformsAttr().Set(
        Vt.Matrix4dArray(rest_local)
    )
    skeleton.CreateBindTransformsAttr().Set(
        Vt.Matrix4dArray(bind_world)
    )

    # Animation usa lo stesso joint order.
    anim.CreateJointsAttr().Set(Vt.TokenArray(names))

    step = LINK_HEIGHT + GAP

    translations = [
        Gf.Vec3f(0.0, 0.0, BASE_Z),
    ] + [
        Gf.Vec3f(0.0, 0.0, step)
        for _ in range(1, NUM_BONES)
    ]

    anim.CreateTranslationsAttr().Set(
        Vt.Vec3fArray(translations)
    )
    anim.CreateScalesAttr().Set(
        Vt.Vec3hArray(
            [Gf.Vec3h(1.0, 1.0, 1.0) for _ in range(NUM_BONES)]
        )
    )

    # Rotazioni locali:
    # Bone0 mantiene -90° X per allineare +Z locale con +Y world.
    # Bone1/Bone2 vengono piegate manualmente per validare lo skinning.
    straight = [_quat_x(-90.0)] + [
        _quat_x(0.0) for _ in range(1, NUM_BONES)
    ]

    bent = [_quat_x(-90.0)]
    for i in range(1, NUM_BONES):
        # Curvatura progressiva, locale, semplice da riconoscere visivamente.
        bent.append(_quat_x(BEND_DEG))

    rot_attr = anim.CreateRotationsAttr()
    rot_attr.Set(Vt.QuatfArray(straight), Usd.TimeCode(0.0))
    rot_attr.Set(Vt.QuatfArray(bent), Usd.TimeCode(FPS))
    rot_attr.Set(Vt.QuatfArray(straight), Usd.TimeCode(2.0 * FPS))

    # Binding Animation -> Skeleton.
    skel_binding = UsdSkel.BindingAPI.Apply(skeleton.GetPrim())
    skel_binding.CreateAnimationSourceRel().SetTargets(
        [anim.GetPrim().GetPath()]
    )

    # TubeMesh
    mesh = UsdGeom.Mesh.Define(
        stage,
        "/World/StemVisual/SkelRoot/TubeMesh",
    )

    points, fc, fi, ji, jw = build_tube_mesh_data()

    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(fc))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(fi))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
    mesh.CreateOrientationAttr().Set(UsdGeom.Tokens.rightHanded)
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray([Gf.Vec3f(0.28, 0.56, 0.19)])
    )

    # Binding Mesh -> Skeleton + skin influences.
    mesh_binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    mesh_binding.CreateSkeletonRel().SetTargets(
        [skeleton.GetPrim().GetPath()]
    )

    # I punti della mesh sono già nello stesso bind space dello Skeleton.
    mesh_binding.CreateGeomBindTransformAttr().Set(
        Gf.Matrix4d(1.0)
    )

    indices_pv = mesh_binding.CreateJointIndicesPrimvar(
        constant=False,
        elementSize=2,
    )
    indices_pv.SetInterpolation(UsdGeom.Tokens.vertex)
    indices_pv.Set(Vt.IntArray(ji))

    weights_pv = mesh_binding.CreateJointWeightsPrimvar(
        constant=False,
        elementSize=2,
    )
    weights_pv.SetInterpolation(UsdGeom.Tokens.vertex)
    weights_pv.Set(Vt.FloatArray(jw))

    return skel_root, skeleton, anim, mesh


def build_stage(output_path=OUTPUT_USD):
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    stage = Usd.Stage.CreateNew(output_path)

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    stage.SetTimeCodesPerSecond(FPS)
    stage.SetFramesPerSecond(FPS)
    stage.SetStartTimeCode(0.0)
    stage.SetEndTimeCode(2.0 * FPS)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    # Nessuna fisica in Test 0D.
    UsdGeom.Xform.Define(stage, "/World/StemVisual")
    build_skinning(stage)

    stage.Save()

    print("=" * 65)
    print("Test 0D — UsdSkel standalone")
    print("=" * 65)
    print(f"[OK] {output_path}")
    print()
    print("Expected:")
    print("  frame   0: tubo perfettamente dritto lungo +Y")
    print(f"  frame  {int(FPS)}: tubo piegato (~{BEND_DEG:.0f}° per bone figlia)")
    print(f"  frame {int(2*FPS)}: tubo nuovamente dritto")
    print()
    print("Se frame 0 NON è dritto, fermarsi:")
    print("  bind/rest/geomBind/coordinate space sono incoerenti.")
    print()
    print("Se frame 0 è dritto e frame 60 si piega:")
    print("  GO -> il prossimo test può essere PhysX -> SkelAnimation.")
    print("=" * 65)

    return output_path


if __name__ == "__main__":
    build_stage()
