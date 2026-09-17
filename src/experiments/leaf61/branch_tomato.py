"""One dynamic tomato and PhysX contact evidence for the coupled branch test."""

import numpy as np


class BranchTomato:
    def __init__(self, stage, prefix="/World"):
        from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

        self.stage = stage
        self.prefix = prefix
        self.path = prefix + "/Tomato"
        body = UsdGeom.Xform.Define(stage, self.path)
        body.AddTranslateOp().Set(Gf.Vec3d(1, 0, 1))
        UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
        UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr().Set(0.02)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(body.GetPrim())
        self.gravity = rb.CreateDisableGravityAttr(True)
        rb.CreateEnableCCDAttr().Set(True)
        rb.CreateSleepThresholdAttr().Set(0)
        sphere = UsdGeom.Sphere.Define(stage, prefix + "/Tomato/Collider")
        sphere.CreateRadiusAttr().Set(0.02)
        sphere.CreateDisplayColorAttr().Set([Gf.Vec3f(0.8, 0.025, 0.015)])
        self.collision = UsdPhysics.CollisionAPI.Apply(
            sphere.GetPrim()
        ).CreateCollisionEnabledAttr(False)
        shape = PhysxSchema.PhysxCollisionAPI.Apply(sphere.GetPrim())
        shape.CreateContactOffsetAttr().Set(0.0005)
        shape.CreateRestOffsetAttr().Set(0)
        calyx = UsdGeom.Sphere.Define(stage, prefix + "/Tomato/Calyx")
        calyx.CreateRadiusAttr().Set(0.0056)
        calyx.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.02))
        calyx.AddScaleOp().Set(Gf.Vec3f(1, 1, 0.3))
        calyx.CreateDisplayColorAttr().Set([Gf.Vec3f(0.1, 0.3, 0.025)])
        self.visual = UsdGeom.Imageable(body.GetPrim())
        self.visual.MakeInvisible()
        PhysxSchema.PhysxContactReportAPI.Apply(
            body.GetPrim()
        ).CreateThresholdAttr().Set(0)
        floor = UsdGeom.Cube.Define(stage, prefix + "/Floor")
        floor.CreateSizeAttr().Set(1)
        floor.AddTranslateOp().Set(Gf.Vec3d(0.1, 0.1, -0.015))
        floor.AddScaleOp().Set(Gf.Vec3f(0.8, 0.8, 0.02))
        floor.CreateDisplayColorAttr().Set([Gf.Vec3f(0.22, 0.22, 0.22)])
        UsdPhysics.CollisionAPI.Apply(floor.GetPrim())
        self.reset_stats()

    def reset_stats(self):
        self.recording = True
        self.released = False
        self.removed = False
        self.cleanup_reason = None
        self.events = []
        self.traces = []
        self.time = 0.0
        self.release_position = None
        self.fall_distance = 0.0

    def initialize(self, world):
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        self.view = world.physics_sim_view.create_rigid_body_view(self.path)
        self.ids = np.array([0], dtype=np.int32)

        def contact(headers, details):
            if not self.recording or not self.released or self.removed:
                return
            for h in headers:
                names = [
                    str(PhysicsSchemaTools.intToSdfPath(v))
                    for v in (h.actor0, h.actor1)
                ]
                if self.path not in names:
                    continue
                leaves = [
                    x
                    for x in names
                    if x.startswith(self.prefix + "/Leaves/") and "/Link" in x
                ]
                if not leaves:
                    continue
                for d in details[
                    h.contact_data_offset : h.contact_data_offset + h.num_contact_data
                ]:
                    impulse = float(np.linalg.norm(d.impulse))
                    if impulse > 0:
                        self.events.append(
                            {
                                "time_s": self.time,
                                "leaf_body": leaves[0],
                                "impulse_Ns": impulse,
                                "separation_m": float(d.separation),
                            }
                        )

        self.subscription = (
            get_physx_simulation_interface().subscribe_contact_report_events(contact)
        )

    def remove(self):
        self.gravity.Set(True)
        self.collision.Set(False)
        self.view.set_velocities(np.zeros((1, 6), np.float32), self.ids)
        self.view.set_transforms(
            np.array([[1, 0, 1, 0, 0, 0, 1]], np.float32), self.ids
        )
        self.visual.MakeInvisible()
        self.removed = True

    def drop(self, target):
        point = np.array(target) + [0, 0, 0.08]
        self.view.set_transforms(np.array([[*point, 0, 0, 0, 1]], np.float32), self.ids)
        self.view.set_velocities(np.zeros((1, 6), np.float32), self.ids)
        self.gravity.Set(False)
        self.collision.Set(True)
        self.visual.MakeVisible()
        self.release_position = point
        self.released = True
        self.removed = False

    def sample(self, t, record, pose=None):
        # The parent validates the complete pose batch, including all tomatoes.
        if pose is not None and not record and (not self.released or self.removed):
            return
        v = np.asarray(self.view.get_transforms())[0].copy() if pose is None else pose
        if pose is None and not np.isfinite(v).all():
            raise RuntimeError("Nonfinite tomato pose")
        if self.released and not self.removed:
            self.fall_distance = max(
                self.fall_distance, float(self.release_position[2] - v[2])
            )
        if record:
            self.traces.append([t, *v.tolist()])
        if self.released and not self.removed and v[2] < -0.2:
            self.cleanup_reason = "Below scene after leaving the floor"
            self.remove()

    def summary(self):
        return {
            "released": self.released,
            "removed": self.removed,
            "cleanup_reason": self.cleanup_reason,
            "fall_distance_m": self.fall_distance,
            "release_position_m": self.release_position.tolist()
            if self.release_position is not None
            else None,
            "contact_events": self.events,
            "maximum_contact_penetration_m": max(
                [max(0, -e["separation_m"]) for e in self.events], default=0
            ),
            "contacted_leaves": sorted(
                {e["leaf_body"].split("/")[-2] for e in self.events}
            ),
        }
