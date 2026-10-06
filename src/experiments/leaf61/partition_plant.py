"""Partition only the world-fixed trunk component; retain every compliant joint."""

import numpy as np


def partition_fixed_trunk(stage, split_branches=False):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    fixed, roots = [], set()
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        j = UsdPhysics.FixedJoint(prim)
        if not j.GetJointEnabledAttr().Get():
            continue
        a, b = j.GetBody0Rel().GetTargets(), j.GetBody1Rel().GetTargets()
        if not a and b:
            roots.add(str(b[0]))
            fixed.append((prim, None, str(b[0])))
        elif a and b:
            fixed.append((prim, str(a[0]), str(b[0])))
    component = set(roots)
    while True:
        before = len(component)
        for _, a, b in fixed:
            if a in component or b in component:
                component.add(b)
                if a:
                    component.add(a)
        if len(component) == before:
            break
    if len(component) < 2:
        raise RuntimeError("No multi-body world-fixed component to partition")
    # All previous roots describe the single connected plant. Re-root exactly
    # its fixed-to-world component, never detach a compliant native joint.
    removed_roots = []
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            removed_roots.append(str(prim.GetPath()))
            prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if prim.HasAPI(PhysxSchema.PhysxArticulationAPI):
                prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
    disabled = []
    for prim, a, b in fixed:
        if b in component and (a is None or a in component):
            UsdPhysics.FixedJoint(prim).GetJointEnabledAttr().Set(False)
            disabled.append(str(prim.GetPath()))
    cache = UsdGeom.XformCache()
    anchors = []
    mass_before = sum(
        float(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0)
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    )
    if split_branches:
        # Factor the shared infinite-stiffness boundary, NOT any moving branch.
        # Each outgoing elastic joint gets an identical world-fixed copy of its
        # parent frame. Split only that already-fixed parent's mass/COM/inertia.
        outgoing = {}
        for prim in list(stage.Traverse()):
            if not prim.IsA(UsdPhysics.Joint):
                continue
            joint = UsdPhysics.Joint(prim)
            if not joint.GetJointEnabledAttr().Get():
                continue
            a, b = joint.GetBody0Rel().GetTargets(), joint.GetBody1Rel().GetTargets()
            if a and b and str(a[0]) in component and str(b[0]) not in component:
                outgoing.setdefault(str(a[0]), []).append(joint)
        for parent, joints in outgoing.items():
            source = stage.GetPrimAtPath(parent)
            source_mass = UsdPhysics.MassAPI(source)
            old_mass = float(source_mass.GetMassAttr().Get())
            share = old_mass / (len(joints) + 1)
            source_mass.GetMassAttr().Set(share)
            matrix = cache.GetLocalToWorldTransform(source)
            inertia = source_mass.GetDiagonalInertiaAttr().Get()
            if inertia and min(inertia) > 0:
                source_mass.GetDiagonalInertiaAttr().Set(inertia / (len(joints) + 1))
            for joint in joints:
                body = f"/World/PartitionFixedFrames/Frame{len(anchors):02d}"
                anchor = UsdGeom.Xform.Define(stage, body)
                anchor.AddTranslateOp().Set(matrix.ExtractTranslation())
                anchor.AddOrientOp().Set(Gf.Quatf(matrix.ExtractRotation().GetQuat()))
                UsdPhysics.RigidBodyAPI.Apply(anchor.GetPrim())
                mass = UsdPhysics.MassAPI.Apply(anchor.GetPrim())
                mass.CreateMassAttr().Set(share)
                mass.CreateCenterOfMassAttr().Set(
                    source_mass.GetCenterOfMassAttr().Get()
                )
                if inertia and min(inertia) > 0:
                    mass.CreateDiagonalInertiaAttr().Set(inertia / (len(joints) + 1))
                PhysxSchema.PhysxRigidBodyAPI.Apply(anchor.GetPrim())
                child = joint.GetBody1Rel().GetTargets()[0]
                if not joint.GetCollisionEnabledAttr().Get():
                    UsdPhysics.FilteredPairsAPI.Apply(
                        source
                    ).CreateFilteredPairsRel().AddTarget(child)
                joint.CreateBody0Rel().SetTargets([body])
                anchors.append(
                    {
                        "body": body,
                        "original_fixed_parent": parent,
                        "joint": str(joint.GetPath()),
                        "mass_kg": share,
                    }
                )
                component.add(body)
        cache.Clear()
    mass_after = sum(
        float(UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0)
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    )
    if not np.isclose(mass_before, mass_after, rtol=1e-7, atol=1e-9):
        raise RuntimeError("Fixed boundary factorization changed total mass")
    created = []
    for i, body in enumerate(sorted(component)):
        matrix = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body))
        q = matrix.ExtractRotation().GetQuat()
        p = matrix.ExtractTranslation()
        path = f"/World/PartitionAnchors/Anchor{i:02d}"
        joint = UsdPhysics.FixedJoint.Define(stage, path)
        joint.CreateBody1Rel().SetTargets([body])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*p))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(q))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1))
        UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
        PhysxSchema.PhysxArticulationAPI.Apply(
            joint.GetPrim()
        ).CreateEnabledSelfCollisionsAttr().Set(True)
        created.append(
            {"joint": path, "body": body, "world_position": np.array(p).tolist()}
        )
    return {
        "mode": "world-fixed-component",
        "removed_articulation_roots": removed_roots,
        "disabled_fixed_joints": disabled,
        "independent_roots": created,
        "compliant_joints_unchanged": True,
        "added_rigid_bodies": len(anchors),
        "fixed_frame_copies": anchors,
        "mass_before_kg": mass_before,
        "mass_after_kg": mass_after,
        "split_branches": split_branches,
        "note": "Equivalent only because these native trunk bodies were already rigidly fixed to world; do not use to freeze a flexible trunk.",
    }
