"""Illustrative compliant lamina: seven curved collider strips and tapered drives."""

import numpy as np


def strip_mesh(points, faces, lo, hi, center, thickness):
    """Clip native 3-D triangles, then thicken for convex cooking (no flattening)."""
    vertices, triangles = [], []
    surface_area = 0.0
    for face in faces:
        poly = list(points[face])
        for bound, sign in ((lo, 1), (hi, -1)):
            clipped = []
            for a, b in zip(poly, poly[1:] + poly[:1]):
                ain, bin_ = (
                    sign * (a[0] - bound) >= -1e-12,
                    sign * (b[0] - bound) >= -1e-12,
                )
                if ain:
                    clipped.append(a)
                if ain != bin_:
                    clipped.append(a + (b - a) * ((bound - a[0]) / (b[0] - a[0])))
            poly = clipped
        if len(poly) < 3:
            continue
        poly = np.asarray(poly)
        for j in range(1, len(poly) - 1):
            surface_area += (
                np.linalg.norm(np.cross(poly[j] - poly[0], poly[j + 1] - poly[0])) / 2
            )
        offset, count = len(vertices), len(poly)
        vertices.extend(poly - center - [0, 0, thickness / 2])
        vertices.extend(poly - center + [0, 0, thickness / 2])
        for j in range(1, count - 1):
            triangles.extend(
                [
                    (offset, offset + j + 1, offset + j),
                    (offset + count, offset + count + j, offset + count + j + 1),
                ]
            )
        for j in range(count):
            k = (j + 1) % count
            triangles.extend(
                [
                    (offset + j, offset + k, offset + count + k),
                    (offset + j, offset + count + k, offset + count + j),
                ]
            )
    if surface_area <= 0:
        raise ValueError("Empty curved collider strip")
    return np.asarray(vertices), np.asarray(triangles, dtype=np.int32), surface_area


def setup(points, c, count=7):
    ds = (c.length - c.fixed_length) / count
    centers = np.array(
        [[(min(float(points[:, 0].min()), 0.0) + c.fixed_length) / 2, 0, c.height]]
        + [[c.fixed_length + (j + 0.5) * ds, 0, c.height] for j in range(count)]
    )
    u = np.interp(points[:, 0], centers[1:, 0], np.arange(1, count + 1))
    low = np.floor(u).astype(np.int32)
    indices = np.column_stack((low, np.minimum(low + 1, count)))
    fraction = u - low
    return centers, indices, np.column_stack((1 - fraction, fraction))


def build_soft(world, c, points, faces, prefix, count=7):
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdSkel, Vt

    from model import area

    stage = world.stage
    context = world.get_physics_context()
    context.enable_gpu_dynamics(False)
    context.set_broadphase_type("MBP")
    context.set_solver_type("PGS")
    centers, indices, weights = setup(points, c, count)
    ds = (c.length - c.fixed_length) / count
    mass = float(area(points, faces).sum() * c.thickness * c.density)
    strips = [
        strip_mesh(
            points,
            faces,
            min(float(points[:, 0].min()), 0.0) if j == 0 else c.fixed_length + j * ds,
            c.fixed_length + (j + 1) * ds,
            centers[j + 1],
            c.thickness,
        )
        for j in range(count)
    ]
    masses = mass * np.array([s[2] for s in strips]) / sum(s[2] for s in strips)
    # Root bend stiffness is 8x lower than the original; the tip is 60x lower.
    stiffness = np.geomspace(0.003, 0.0004, count)
    for j, center in enumerate(centers):
        path = prefix + ("/Petiole" if j == 0 else f"/Link{j - 1}")
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*center))
        body.AddOrientOp().Set(Gf.Quatf(1))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr().Set(
            0.01 if j == 0 else float(masses[j - 1])
        )
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
        rb.CreateSolverPositionIterationCountAttr().Set(c.iterations)
        rb.CreateSleepThresholdAttr().Set(0)
        if j:
            cp, cf, _ = strips[j - 1]
            collider = UsdGeom.Mesh.Define(stage, path + "/Collider")
            collider.CreatePointsAttr().Set(
                Vt.Vec3fArray.FromNumpy(cp.astype(np.float32))
            )
            collider.CreateFaceVertexCountsAttr().Set([3] * len(cf))
            collider.CreateFaceVertexIndicesAttr().Set(cf.ravel().tolist())
            UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(
                collider.GetPrim()
            ).CreateApproximationAttr().Set("convexHull")
            coll = PhysxSchema.PhysxCollisionAPI.Apply(collider.GetPrim())
            coll.CreateContactOffsetAttr().Set(0.0003)
            coll.CreateRestOffsetAttr().Set(0)
            UsdGeom.Imageable(body.GetPrim()).MakeInvisible()
    fixed = UsdPhysics.FixedJoint.Define(stage, prefix + "/FixedPetiole")
    fixed.CreateBody1Rel().SetTargets([prefix + "/Petiole"])
    fixed.CreateLocalPos0Attr().Set(Gf.Vec3f(*centers[0]))
    for j in range(count):
        joint = UsdPhysics.Joint.Define(stage, prefix + f"/Joint{j}")
        joint.CreateBody0Rel().SetTargets(
            [prefix + ("/Petiole" if j == 0 else f"/Link{j - 1}")]
        )
        joint.CreateBody1Rel().SetTargets([prefix + f"/Link{j}"])
        anchor = np.array([c.fixed_length + j * ds, 0, c.height])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(anchor - centers[j])))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(*(anchor - centers[j + 1])))
        joint.CreateCollisionEnabledAttr().Set(False)
        for axis in ("transX", "transY", "transZ", "rotZ"):
            lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            lim.CreateLowAttr().Set(1)
            lim.CreateHighAttr().Set(-1)
        inertia = sum(masses[k] * ((k - j + 0.5) * ds) ** 2 for k in range(j, count))
        for axis, scale in [("rotY", 1.0), ("rotX", 0.35)]:
            k = float(stiffness[j] * scale)
            lim = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            lim.CreateLowAttr().Set(-55)
            lim.CreateHighAttr().Set(55)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(k * np.pi / 180)
            drive.CreateDampingAttr().Set(2 * 1.2 * np.sqrt(k * inertia) * np.pi / 180)
            drive.CreateTargetPositionAttr().Set(0)
    UsdSkel.Root.Define(stage, prefix + "/Leaf")
    mesh = UsdGeom.Mesh.Define(stage, prefix + "/Leaf/Mesh")
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    mesh.CreateFaceVertexCountsAttr().Set([3] * len(faces))
    mesh.CreateFaceVertexIndicesAttr().Set(faces.ravel().tolist())
    mesh.CreateSubdivisionSchemeAttr().Set("none")
    mesh.CreateDoubleSidedAttr().Set(True)
    mesh.CreateDisplayColorAttr().Set([Gf.Vec3f(0.12, 0.42, 0.05)])
    skeleton = UsdSkel.Skeleton.Define(stage, prefix + "/Leaf/Skeleton")
    anim = UsdSkel.Animation.Define(stage, prefix + "/Leaf/Animation")
    names = [f"bone{j}" for j in range(count + 1)]
    binds = Vt.Matrix4dArray(
        [Gf.Matrix4d().SetTranslate(Gf.Vec3d(*v)) for v in centers]
    )
    skeleton.CreateJointsAttr().Set(names)
    skeleton.CreateBindTransformsAttr().Set(binds)
    skeleton.CreateRestTransformsAttr().Set(binds)
    anim.CreateJointsAttr().Set(names)
    anim.CreateScalesAttr().Set(Vt.Vec3hArray([Gf.Vec3h(1)] * (count + 1)))
    UsdSkel.BindingAPI.Apply(skeleton.GetPrim()).CreateAnimationSourceRel().SetTargets(
        [anim.GetPath()]
    )
    binding = UsdSkel.BindingAPI.Apply(mesh.GetPrim())
    binding.CreateSkeletonRel().SetTargets([skeleton.GetPath()])
    binding.CreateGeomBindTransformAttr().Set(Gf.Matrix4d(1))
    binding.CreateJointIndicesPrimvar(False, 2).Set(indices.ravel().tolist())
    binding.CreateJointWeightsPrimvar(False, 2).Set(weights.ravel().tolist())
    return (
        None,
        None,
        anim,
        (centers, indices, weights, None),
        {
            "mass_kg": mass,
            "bend_stiffness_Nm_rad": stiffness.tolist(),
            "segment_masses_kg": masses.tolist(),
            "torsion_ratio": 0.35,
            "damping_ratio": 1.2,
        },
    )
