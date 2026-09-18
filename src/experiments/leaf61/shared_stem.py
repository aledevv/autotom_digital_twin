"""Connect three accepted branch fixtures to one elastically hinged stem."""

import numpy as np

from model import rotation

STEM_LENGTH = 0.5
STEM_MASS = 0.06
STEM_STIFFNESS = 3.0
STEM_DAMPING = 0.25


def connect(stage, prefixes, shifts, fixed=False):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    for prefix, shift in zip(prefixes, shifts):
        stage.RemovePrim(prefix + "/FixedRoot")
        stage.RemovePrim(prefix + "/Anchor")
        stage.RemovePrim(prefix + "/Support")
        branch_joint = UsdPhysics.Joint.Get(stage, prefix + "/BranchRoot")
        branch_joint.CreateBody0Rel().SetTargets(["/World/Stem"])
        branch_joint.CreateLocalPos0Attr().Set(
            Gf.Vec3f(*(shift + [0, 0, 0.2 - STEM_LENGTH / 2]))
        )
    stem = UsdGeom.Xform.Define(stage, "/World/Stem")
    stem.AddTranslateOp().Set(Gf.Vec3d(0, 0, STEM_LENGTH / 2))
    UsdPhysics.RigidBodyAPI.Apply(stem.GetPrim())
    UsdPhysics.MassAPI.Apply(stem.GetPrim()).CreateMassAttr().Set(STEM_MASS)
    shape = UsdGeom.Cube.Define(stage, "/World/Stem/Collider")
    shape.CreateSizeAttr().Set(1)
    shape.AddScaleOp().Set(Gf.Vec3f(0.014, 0.014, STEM_LENGTH))
    shape.CreateDisplayColorAttr().Set([Gf.Vec3f(0.18, 0.35, 0.06)])
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim()).CreateContactOffsetAttr().Set(
        0.0005
    )
    anchor = UsdGeom.Xform.Define(stage, "/World/StemAnchor")
    UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
    mass = UsdPhysics.MassAPI.Apply(anchor.GetPrim())
    mass.CreateMassAttr().Set(0.1)
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.0001))
    root = UsdPhysics.FixedJoint.Define(stage, "/World/StemRoot")
    root.CreateBody1Rel().SetTargets([anchor.GetPath()])
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr().Set(False)
    art.CreateSolverPositionIterationCountAttr().Set(32)
    art.CreateSolverVelocityIterationCountAttr().Set(4)
    art.CreateSleepThresholdAttr().Set(0)
    if fixed:
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/StemHinge")
    else:
        joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/StemHinge")
        joint.CreateAxisAttr().Set("X")
        joint.CreateLowerLimitAttr().Set(-35)
        joint.CreateUpperLimitAttr().Set(35)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.CreateTypeAttr().Set("force")
        drive.CreateStiffnessAttr().Set(STEM_STIFFNESS * np.pi / 180)
        drive.CreateDampingAttr().Set(STEM_DAMPING * np.pi / 180)
        drive.CreateTargetPositionAttr().Set(0)
    joint.CreateBody0Rel().SetTargets([anchor.GetPath()])
    joint.CreateBody1Rel().SetTargets([stem.GetPath()])
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, -STEM_LENGTH / 2))
    return shifts + np.array([0, 0, 0.2 - STEM_LENGTH / 2])


def attachment_frames(stem_pose, local_roots):
    R = rotation(stem_pose[None, [6, 3, 4, 5]])[0]
    roots = stem_pose[:3] + local_roots @ R.T
    base_error = float(np.linalg.norm(stem_pose[:3] - STEM_LENGTH / 2 * R[:, 2]))
    return roots, base_error
