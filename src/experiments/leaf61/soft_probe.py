"""A controlled branch-shaped capsule bends the first blade from underneath."""

import numpy as np

from model import material_target, ramp, rotation, skin


class ContactProbe:
    def __init__(self, stage, row, rows, native):
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        self.path = "/World/BranchContactProbe"
        body = UsdGeom.Xform.Define(stage, self.path)
        body.AddTranslateOp().Set(Gf.Vec3d(0, 0, -2))
        self.frame = np.asarray(row["initial_frame"])
        q = Gf.Matrix3d(*self.frame.T.ravel().tolist()).ExtractRotation().GetQuat()
        self.q = np.array([*q.GetImaginary(), q.GetReal()])
        body.AddOrientOp().Set(Gf.Quatf(q))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateKinematicEnabledAttr().Set(
            True
        )
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr().Set(0.01)
        capsule = UsdGeom.Capsule.Define(stage, self.path + "/Collider")
        capsule.CreateAxisAttr().Set("Y")
        capsule.CreateHeightAttr().Set(0.075)
        capsule.CreateRadiusAttr().Set(0.003)
        capsule.CreateDisplayColorAttr().Set([Gf.Vec3f(0.3, 0.2, 0.08)])
        self.enabled = UsdPhysics.CollisionAPI.Apply(
            capsule.GetPrim()
        ).CreateCollisionEnabledAttr(False)
        PhysxSchema.PhysxCollisionAPI.Apply(
            capsule.GetPrim()
        ).CreateContactOffsetAttr().Set(0.0003)
        PhysxSchema.PhysxContactReportAPI.Apply(
            body.GetPrim()
        ).CreateThresholdAttr().Set(0)
        targets = [p.GetPath() for p in native] + [
            r["prefix"] + f"/Link{j}" for r in rows[1:] for j in range(r["segments"])
        ]
        UsdPhysics.FilteredPairsAPI.Apply(
            body.GetPrim()
        ).CreateFilteredPairsRel().SetTargets(targets)
        self.visual = UsdGeom.Imageable(body.GetPrim())
        self.visual.MakeInvisible()
        self.start_time = 8.0
        self.started = False

    def initialize(self, world):
        self.view = world.physics_sim_view.create_rigid_body_view(self.path)
        self.ids = np.array([0], dtype=np.int32)

    def repeat(self, t):
        self.enabled.Set(False)
        self.visual.MakeInvisible()
        self.started = False
        self.start_time = t + 2

    def update(self, t, read, row):
        if t < self.start_time:
            return
        if not self.started:
            poses = read()
            h = row["host_id"]
            hr = rotation(poses[h : h + 1, [6, 3, 4, 5]])[0]
            # Use matrix form to reconstruct the actual pre-contact leaf.
            bp = np.vstack(
                [poses[h, :3] + hr @ row["host_offset"], poses[row["ids"], :3]]
            )
            from pxr import Gf

            q = (
                Gf.Matrix3d(*(hr @ row["host_frame"]).T.ravel().tolist())
                .ExtractRotation()
                .GetQuat()
            )
            bq = np.vstack(
                [[q.GetReal(), *q.GetImaginary()], poses[row["ids"]][:, [6, 3, 4, 5]]]
            )
            p = skin(
                row["points"], row["centers"], row["indices"], row["weights"], bp, bq
            )
            c = row["config"]
            x = c["fixed_length"] + 0.65 * (c["length"] - c["fixed_length"])
            self.target = material_target(row["points"], p, row["faces"], x, 0)
            self.normal = hr @ row["host_frame"][:, 2]
            self.frame = hr @ row["host_frame"]
            q = Gf.Matrix3d(*self.frame.T.ravel().tolist()).ExtractRotation().GetQuat()
            self.q = np.array([*q.GetImaginary(), q.GetReal()])
            start = np.array(
                [[*(self.target - 0.02 * self.normal), *self.q]], dtype=np.float32
            )
            self.view.set_transforms(start, self.ids)
            self.enabled.Set(True)
            self.visual.MakeVisible()
            self.started = True
        dt = t - self.start_time
        fraction = (
            ramp(dt / 1.5) if dt < 1.5 else (1.0 if dt < 2 else 1 - ramp((dt - 2) / 2))
        )
        center = self.target + (-0.02 + 0.032 * fraction) * self.normal
        self.view.set_kinematic_targets(
            np.array([[*center, *self.q]], dtype=np.float32), self.ids
        )
