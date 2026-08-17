"""
generate_bridge.py — Test 0E: PhysX + UsdSkel, senza bridge runtime

Questo file costruisce SOLO l'asset USD:
  - catena PhysX funzionante (3 rigid links + D6)
  - collider cilindrici invisibili
  - Skeleton + TubeMesh skinnata funzionanti
  - SkelAnimation inizializzata nella rest pose

NON contiene il bridge runtime.
Per eseguire il test completo usare:
    ~/isaacsim/python.sh run_bridge.py

Convenzione condivisa PhysX / Skeleton:
  - origine link = origine bone = BASE del segmento
  - asse locale del segmento = +Z
  - root ruotata -90° X -> ramo iniziale lungo +Y world
  - distanza fra origini consecutive = LINK_HEIGHT + GAP
"""

import math
import os
import sys

try:
    from pxr import (
        Gf,
        Usd,
        UsdGeom,
        UsdPhysics,
        UsdSkel,
        PhysxSchema,
        Sdf,
        Vt,
    )
except ImportError:
    print("[ERROR] pxr non trovato. Usa ~/isaacsim/python.sh generate_bridge.py")
    sys.exit(1)


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")
OUTPUT_USD = os.path.join(OUTPUT_DIR, "test_0e_physx_to_skin.usda")

# ---------------------------------------------------------------------------
# CONFIG condivisa
# ---------------------------------------------------------------------------

NUM_LINKS = 3
NUM_BONES = NUM_LINKS

LINK_HEIGHT = 0.10
LINK_RADIUS = 0.012
LINK_MASS = 0.05
GAP = 0.001
BASE_Z = 0.10

JOINT_STIFF = 0.0
JOINT_DAMP = 0.05
BEND_LIMIT = 89.0

TUBE_RADIUS = LINK_RADIUS
RADIAL_SEGMENTS = 12
RINGS_PER_LINK = 10
BLEND_HALF_WIDTH = LINK_HEIGHT * 0.15


# ---------------------------------------------------------------------------
# GENERIC HELPERS
# ---------------------------------------------------------------------------

def quat_x(deg: float) -> Gf.Quatf:
    a = math.radians(deg) * 0.5
    return Gf.Quatf(
        math.cos(a),
        Gf.Vec3f(math.sin(a), 0.0, 0.0),
    )


def matrix_rot_trans(rot_deg_x: float, t: Gf.Vec3d) -> Gf.Matrix4d:
    m = Gf.Matrix4d(1.0)
    m.SetTransform(
        Gf.Rotation(Gf.Vec3d(1.0, 0.0, 0.0), rot_deg_x),
        t,
    )
    return m


def joint_names():
    names = ["Bone0"]
    for i in range(1, NUM_BONES):
        names.append(names[-1] + f"/Bone{i}")
    return names


# ---------------------------------------------------------------------------
# PHYSX — invariato rispetto al checkpoint physics-only
# ---------------------------------------------------------------------------

def apply_physx_scene(stage) -> str:
    scene_path = "/World/PhysicsScene"
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    physx = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physx.CreateSolverTypeAttr().Set("TGS")
    physx.CreateTimeStepsPerSecondAttr().Set(120)
    physx.CreateEnableCCDAttr().Set(True)
    physx.CreateEnableStabilizationAttr().Set(True)
    physx.CreateEnableGPUDynamicsAttr().Set(True)
    physx.CreateBroadphaseTypeAttr().Set("MBP")
    return scene_path


def create_rigid_segment(stage, stem_path, name, world_pos, orientation):
    """
    Il Cylinder è SOLO collider/debug physics.
    Lo rendiamo invisibile: la visualizzazione finale è TubeMesh.
    """
    link_path = f"{stem_path}/{name}"
    xf = UsdGeom.Xform.Define(stage, link_path)

    xf.AddTranslateOp().Set(
        Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
    )
    xf.AddOrientOp().Set(orientation)

    collider = UsdGeom.Cylinder.Define(stage, f"{link_path}/Collider")
    collider.CreateHeightAttr().Set(LINK_HEIGHT)
    collider.CreateRadiusAttr().Set(LINK_RADIUS)
    collider.CreateAxisAttr().Set("Z")

    # Link origin = BASE; collider centrato a metà segmento.
    collider_xf = UsdGeom.Xformable(collider.GetPrim())
    collider_xf.AddTranslateOp().Set(
        Gf.Vec3d(0.0, 0.0, LINK_HEIGHT / 2.0)
    )

    UsdGeom.Imageable(collider.GetPrim()).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())

    rb = UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
    rb.CreateRigidBodyEnabledAttr().Set(True)

    mass = UsdPhysics.MassAPI.Apply(xf.GetPrim())
    mass.CreateMassAttr().Set(LINK_MASS)

    return link_path


def anchor_link_to_world(stage, link_path: str) -> None:
    joint = UsdPhysics.FixedJoint.Define(
        stage,
        f"{link_path}/RootFixedJoint",
    )
    # body0 non impostato = world
    joint.CreateBody1Rel().SetTargets([Sdf.Path(link_path)])


def configure_joint(joint) -> None:
    """
    D6:
      - translations locked
      - rotX / rotY limitati ±BEND_LIMIT e guidati con damping
      - rotZ locked
    """
    # low > high -> locked
    for ax in ("transX", "transY", "transZ"):
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), ax)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)

    for ax in ("rotX", "rotY"):
        lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), ax)
        lim.CreateLowAttr().Set(-BEND_LIMIT)
        lim.CreateHighAttr().Set(BEND_LIMIT)

        drv = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), ax)
        drv.CreateTypeAttr().Set("force")
        drv.CreateStiffnessAttr().Set(JOINT_STIFF)
        drv.CreateDampingAttr().Set(JOINT_DAMP)
        drv.CreateTargetPositionAttr().Set(0.0)

    lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), "rotZ")
    lim.CreateLowAttr().Set(1.0)
    lim.CreateHighAttr().Set(-1.0)


def create_internal_joint(stage, parent_path, child_path, index) -> None:
    joint = UsdPhysics.Joint.Define(
        stage,
        f"{child_path}/Joint_{index:02d}_{index + 1:02d}",
    )
    joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_path)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(child_path)])

    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(0.0, 0.0, LINK_HEIGHT + GAP)
    )
    joint.CreateLocalPos1Attr().Set(
        Gf.Vec3f(0.0, 0.0, 0.0)
    )
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    configure_joint(joint)


def apply_articulation_settings(stage, stem_path: str) -> None:
    prim = stage.GetPrimAtPath(stem_path)
    art = PhysxSchema.PhysxArticulationAPI.Apply(prim)
    art.CreateSolverPositionIterationCountAttr().Set(32)
    art.CreateSolverVelocityIterationCountAttr().Set(1)
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSleepThresholdAttr().Set(0.0)


def build_physics_chain(stage):
    stem_path = "/World/Stem"
    stem = UsdGeom.Xform.Define(stage, stem_path)
    UsdPhysics.ArticulationRootAPI.Apply(stem.GetPrim())

    # +Z local -> +Y world.
    orientation = quat_x(-90.0)
    step = LINK_HEIGHT + GAP

    paths = []
    for i in range(NUM_LINKS):
        world_pos = Gf.Vec3d(
            0.0,
            i * step,
            BASE_Z,
        )

        path = create_rigid_segment(
            stage,
            stem_path,
            f"Branch_Link_{i + 1:02d}",
            world_pos,
            orientation,
        )

        if i == 0:
            anchor_link_to_world(stage, path)
        else:
            create_internal_joint(
                stage,
                paths[-1],
                path,
                i,
            )

        paths.append(path)

    apply_articulation_settings(stage, stem_path)
    return paths


# ---------------------------------------------------------------------------
# USDSKEL — invariato concettualmente rispetto al Test 0D
# ---------------------------------------------------------------------------

def rest_local_transforms():
    step = LINK_HEIGHT + GAP

    out = [
        matrix_rot_trans(
            -90.0,
            Gf.Vec3d(0.0, 0.0, BASE_Z),
        )
    ]

    for _ in range(1, NUM_BONES):
        out.append(
            matrix_rot_trans(
                0.0,
                Gf.Vec3d(0.0, 0.0, step),
            )
        )

    return out


def bind_skel_transforms():
    step = LINK_HEIGHT + GAP

    return [
        matrix_rot_trans(
            -90.0,
            Gf.Vec3d(0.0, i * step, BASE_Z),
        )
        for i in range(NUM_BONES)
    ]


def weights_for_y(y: float):
    step = LINK_HEIGHT + GAP

    for child in range(1, NUM_BONES):
        joint_y = child * step
        lo = joint_y - BLEND_HALF_WIDTH
        hi = joint_y + BLEND_HALF_WIDTH

        if lo <= y <= hi:
            u = (y - lo) / (hi - lo)
            u = max(0.0, min(1.0, u))
            return child - 1, child, 1.0 - u, u

    bone = 0
    for i in range(1, NUM_BONES):
        if y >= i * step:
            bone = i
        else:
            break

    return bone, bone, 1.0, 0.0


def build_tube_mesh_data():
    total_length = (
        (NUM_BONES - 1) * (LINK_HEIGHT + GAP)
        + LINK_HEIGHT
    )
    total_rings = NUM_BONES * RINGS_PER_LINK + 1

    points = []
    joint_indices = []
    joint_weights = []

    for ring in range(total_rings):
        u = ring / (total_rings - 1)
        y = u * total_length

        b0, b1, w0, w1 = weights_for_y(y)

        for s in range(RADIAL_SEGMENTS):
            a = 2.0 * math.pi * s / RADIAL_SEGMENTS
            x = math.cos(a) * TUBE_RADIUS
            z = BASE_Z + math.sin(a) * TUBE_RADIUS

            points.append(
                Gf.Vec3f(float(x), float(y), float(z))
            )
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


def build_visual_skin(stage):
    UsdGeom.Xform.Define(stage, "/World/StemVisual")

    skel_root = UsdSkel.Root.Define(
        stage,
        "/World/StemVisual/SkelRoot",
    )
    skeleton = UsdSkel.Skeleton.Define(
        stage,
        "/World/StemVisual/SkelRoot/Skeleton",
    )
    anim = UsdSkel.Animation.Define(
        stage,
        "/World/StemVisual/SkelRoot/SkelAnim",
    )

    names = joint_names()
    rest_local = rest_local_transforms()
    bind_xforms = bind_skel_transforms()

    skeleton.CreateJointsAttr().Set(
        Vt.TokenArray(names)
    )
    skeleton.CreateRestTransformsAttr().Set(
        Vt.Matrix4dArray(rest_local)
    )
    skeleton.CreateBindTransformsAttr().Set(
        Vt.Matrix4dArray(bind_xforms)
    )

    anim.CreateJointsAttr().Set(
        Vt.TokenArray(names)
    )

    step = LINK_HEIGHT + GAP

    # Rest pose iniziale, nessuna animazione temporale authored.
    anim.CreateTranslationsAttr().Set(
        Vt.Vec3fArray(
            [Gf.Vec3f(0.0, 0.0, BASE_Z)]
            + [
                Gf.Vec3f(0.0, 0.0, step)
                for _ in range(1, NUM_BONES)
            ]
        )
    )

    anim.CreateRotationsAttr().Set(
        Vt.QuatfArray(
            [quat_x(-90.0)]
            + [
                quat_x(0.0)
                for _ in range(1, NUM_BONES)
            ]
        )
    )

    anim.CreateScalesAttr().Set(
        Vt.Vec3hArray(
            [
                Gf.Vec3h(1.0, 1.0, 1.0)
                for _ in range(NUM_BONES)
            ]
        )
    )

    skel_binding = UsdSkel.BindingAPI.Apply(
        skeleton.GetPrim()
    )
    skel_binding.CreateAnimationSourceRel().SetTargets(
        [anim.GetPrim().GetPath()]
    )

    mesh = UsdGeom.Mesh.Define(
        stage,
        "/World/StemVisual/SkelRoot/TubeMesh",
    )

    points, fc, fi, ji, jw = build_tube_mesh_data()

    mesh.CreatePointsAttr().Set(
        Vt.Vec3fArray(points)
    )
    mesh.CreateFaceVertexCountsAttr().Set(
        Vt.IntArray(fc)
    )
    mesh.CreateFaceVertexIndicesAttr().Set(
        Vt.IntArray(fi)
    )
    mesh.CreateSubdivisionSchemeAttr().Set(
        UsdGeom.Tokens.none
    )
    mesh.CreateOrientationAttr().Set(
        UsdGeom.Tokens.rightHanded
    )
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray(
            [Gf.Vec3f(0.28, 0.56, 0.19)]
        )
    )

    mesh_binding = UsdSkel.BindingAPI.Apply(
        mesh.GetPrim()
    )
    mesh_binding.CreateSkeletonRel().SetTargets(
        [skeleton.GetPrim().GetPath()]
    )
    mesh_binding.CreateGeomBindTransformAttr().Set(
        Gf.Matrix4d(1.0)
    )

    indices_pv = mesh_binding.CreateJointIndicesPrimvar(
        constant=False,
        elementSize=2,
    )
    indices_pv.SetInterpolation(
        UsdGeom.Tokens.vertex
    )
    indices_pv.Set(
        Vt.IntArray(ji)
    )

    weights_pv = mesh_binding.CreateJointWeightsPrimvar(
        constant=False,
        elementSize=2,
    )
    weights_pv.SetInterpolation(
        UsdGeom.Tokens.vertex
    )
    weights_pv.Set(
        Vt.FloatArray(jw)
    )

    return anim.GetPrim().GetPath()


# ---------------------------------------------------------------------------
# GROUND + STAGE
# ---------------------------------------------------------------------------

def build_ground(stage):
    gnd = UsdGeom.Mesh.Define(stage, "/World/Ground")
    s = 0.6

    gnd.CreatePointsAttr().Set(
        Vt.Vec3fArray([
            Gf.Vec3f(-s, -s, 0.0),
            Gf.Vec3f( s, -s, 0.0),
            Gf.Vec3f( s,  s, 0.0),
            Gf.Vec3f(-s,  s, 0.0),
        ])
    )
    gnd.CreateFaceVertexCountsAttr().Set(
        Vt.IntArray([4])
    )
    gnd.CreateFaceVertexIndicesAttr().Set(
        Vt.IntArray([0, 1, 2, 3])
    )
    gnd.CreateDisplayColorAttr().Set(
        Vt.Vec3fArray(
            [Gf.Vec3f(0.25, 0.22, 0.18)]
        )
    )
    gnd.CreateSubdivisionSchemeAttr().Set(
        UsdGeom.Tokens.none
    )
    UsdPhysics.CollisionAPI.Apply(gnd.GetPrim())


def build_stage(output_path=OUTPUT_USD):
    os.makedirs(
        os.path.dirname(output_path),
        exist_ok=True,
    )

    stage = Usd.Stage.CreateNew(output_path)

    UsdGeom.SetStageUpAxis(
        stage,
        UsdGeom.Tokens.z,
    )
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    apply_physx_scene(stage)
    link_paths = build_physics_chain(stage)
    anim_path = build_visual_skin(stage)
    build_ground(stage)

    stage.Save()

    print("=" * 68)
    print("Test 0E asset — PhysX + UsdSkel")
    print("=" * 68)
    print(f"[OK] {output_path}")
    print()
    print("PhysX links:")
    for p in link_paths:
        print(f"  {p}")
    print()
    print(f"SkelAnimation: {anim_path}")
    print("Collider physics: invisibili")
    print("Visuale: solo TubeMesh")
    print()
    print("Questo USDA da solo resta nella rest pose.")
    print("Il movimento della skin viene applicato da run_bridge.py.")
    print("=" * 68)

    return output_path


if __name__ == "__main__":
    build_stage()
