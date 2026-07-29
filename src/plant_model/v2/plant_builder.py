import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf

from plant_model.v2.config import PLANT_ROOT_PATH, GLOBAL_SCALE
from plant_model.v2.constants import STEM_COLOR, LEAF_COLOR
from plant_model.v2.plant_builder_utils import (
    _quatd_to_quatf, _configure_drives, _auto_mass, _critical_damping,
)

IDENTITY_QUATF = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
IDENTITY_ROT = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0.0)


class PlantBuilder:
    def __init__(self, stage: Usd.Stage, base_path: str = PLANT_ROOT_PATH, scale: float = GLOBAL_SCALE):
        self.stage = stage
        self.base_path = base_path
        self._segments: dict[str, dict] = {}
        self.scale = scale

        if not self.stage.GetPrimAtPath(self.base_path):
            UsdGeom.Xform.Define(self.stage, self.base_path)

    # ------------------------------------------------------------------ #
    # VISUAL
    # ------------------------------------------------------------------ #

    def _make_visual(self, path: str, radius: float, height: float, color: tuple) -> Usd.Prim:
        """Simple colored cylinder. TODO: replace with skinned mesh binding."""
        radius_scaled = radius * self.scale
        height_scaled = height * self.scale

        cyl = UsdGeom.Cylinder.Define(self.stage, f"{path}/Cylinder")
        cyl.GetRadiusAttr().Set(radius_scaled)
        cyl.GetHeightAttr().Set(height_scaled)
        cyl.GetAxisAttr().Set("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height_scaled / 2.0))
        cyl.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])
        return cyl.GetPrim()

    # ------------------------------------------------------------------ #
    # PUBLIC API
    # ------------------------------------------------------------------ #

    def add_main_stem_segments(
        self,
        base_id: str,
        segments: list[dict],
        color: tuple = STEM_COLOR,
        mass_per_segment: float = 1.0,
        physics: bool = False,
    ) -> dict[tuple[int, int], str]:
        """
        Builds the main trunk chain (order=0), stacked vertically.
        Petioles must be attached separately via add_leaf, anchored to a
        specific point on the trunk.
        """
        trunk_segments = [s for s in segments if s["order"] == 0]
        if len(trunk_segments) != len(segments):
            skipped = len(segments) - len(trunk_segments)
            print(f"[WARN] add_main_stem_segments: ignoring {skipped} non-trunk "
                  f"(order>0) segments.")

        order_rank_to_id: dict[tuple[int, int], str] = {}
        current_z = 0.0

        for seg in sorted(trunk_segments, key=lambda s: s["rank"]):
            seg_id = f"Internode_o{seg['order']}_r{seg['rank']}"
            order_rank_to_id[(seg["order"], seg["rank"])] = seg_id
            path = f"{self.base_path}/{seg_id}"

            xform = UsdGeom.Xform.Define(self.stage, path)
            xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, current_z))
            xform.AddOrientOp().Set(IDENTITY_QUATF)

            self._make_visual(path, seg["radius"], seg["length"], color)

            if physics:
                UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
                UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr().Set(mass_per_segment)
                fj = UsdPhysics.FixedJoint.Define(self.stage, f"{path}/FixedJoint")
                fj.CreateBody1Rel().SetTargets([Sdf.Path(path)])

            scaled_length = seg["length"] * self.scale

            self._segments[seg_id] = dict(
                path=path,
                global_rot=IDENTITY_ROT,
                radius=seg["radius"] * self.scale,
                height=scaled_length,
                base_pos=Gf.Vec3d(0, 0, current_z),
            )

            current_z += scaled_length

        return order_rank_to_id

    def add_leaf(
        self,
        parent_id: str,
        base_id: str,
        total_length: float,
        radius_start: float,
        radius_end: float,
        z_offset_ratio: float = 1.0,
        tilt_angle: float = 60.0,
        rot_around_parent: float = 0.0,
        num_petiole_segments: int = 3,
        physics: bool = False,
        stiffness_base: float = 5000.0,
        stiffness_tip: float = 1000.0,
        damping_ratio: float = 0.7,
        max_bend_angle: float = 10.0,
        twist_limit: float = 15.0,
        density: float = 200.0,
        color: tuple = LEAF_COLOR,
    ) -> str:
        """Build an articulated petiole chain (stem + leaves, no blades).

        Builds a chain of `num_petiole_segments` D6-jointed cylinder segments
        representing the petiole. The first segment branches off the parent
        stem via add_lateral_branch; subsequent segments extend tip-to-tip
        via add_internode.
        """
        if num_petiole_segments <= 0:
            raise ValueError("num_petiole_segments must be > 0")

        # Container Xform so the leaf appears as a named group
        group_path = f"{self.base_path}/{base_id}"
        UsdGeom.Xform.Define(self.stage, group_path)

        segment_len = total_length / num_petiole_segments
        current_parent = parent_id

        for i in range(num_petiole_segments):
            t = i / max(1, num_petiole_segments - 1) if num_petiole_segments > 1 else 0.0
            r = radius_start + t * (radius_end - radius_start)
            stiff = stiffness_base + t * (stiffness_tip - stiffness_base)
            seg_id = f"{base_id}/Seg_{i:02d}"

            if i == 0:
                self.add_lateral_branch(
                    parent_id=current_parent, id=seg_id,
                    radius=r, length=segment_len,
                    z_offset_ratio=z_offset_ratio, tilt_angle=tilt_angle,
                    rot_around_parent=rot_around_parent,
                    density=density, stiffness=stiff, damping_ratio=damping_ratio,
                    max_bend_angle=max_bend_angle, twist_limit=twist_limit,
                    color=color, physics=physics,
                )
            else:
                self.add_internode(
                    parent_id=current_parent, id=seg_id,
                    radius=r, length=segment_len,
                    density=density, stiffness=stiff, damping_ratio=damping_ratio,
                    max_bend_angle=max_bend_angle,
                    color=color, physics=physics,
                )
            current_parent = seg_id

        return current_parent

    def add_lateral_branch(
        self,
        parent_id: str,
        id: str,
        radius: float,
        length: float,
        z_offset_ratio: float,
        tilt_angle: float,
        rot_around_parent: float,
        density: float = 200.0,
        stiffness: float = 5000.0,
        damping_ratio: float = 0.7,
        max_bend_angle: float = 30.0,
        twist_limit: float = 15.0,
        color: tuple = STEM_COLOR,
        physics: bool = False,
    ) -> str:
        """
        Attach a new segment to the surface of a parent cylinder, offset
        laterally and tilted away from the parent's axis. Used as the
        first segment of a petiole chain.

        z_offset_ratio : 0..1, where along the parent's height to attach.
        tilt_angle      : degrees away from the parent's axis.
        rot_around_parent : degrees around the parent's axis (azimuth).
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        if id in self._segments:
            raise ValueError(f"Segment id '{id}' already exists!")

        p = self._segments[parent_id]
        rel_z = z_offset_ratio * p["height"]

        rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_parent)
        tilt_r = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_angle)

        sub_rot_local = tilt_r * rot_z
        sub_rot_total = sub_rot_local * p["global_rot"]

        axis_offset = Gf.Vec3d(0.0, 0.0, rel_z)
        world_pos = p["base_pos"] + p["global_rot"].TransformDir(axis_offset)
        local_pos0 = Gf.Vec3f(0.0, 0.0, float(rel_z))

        orient_qf = _quatd_to_quatf(sub_rot_total.GetQuat())
        path = f"{self.base_path}/{id}"

        xform = UsdGeom.Xform.Define(self.stage, path)
        xform.AddTranslateOp().Set(world_pos)
        xform.AddOrientOp().Set(orient_qf)

        self._make_visual(path, radius, length, color)

        scaled_length = length * self.scale
        scaled_radius = radius * self.scale

        mass = _auto_mass(scaled_radius, scaled_length, density)
        damping = damping_ratio * _critical_damping(stiffness, mass)

        if physics:
            UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
            UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr().Set(mass)

            cyl_prim = self.stage.GetPrimAtPath(f"{path}/Cylinder")
            UsdPhysics.CollisionAPI.Apply(cyl_prim)

            jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
            jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
            jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            jnt.CreateLocalPos0Attr().Set(local_pos0)
            jnt.CreateLocalRot0Attr().Set(_quatd_to_quatf(sub_rot_local.GetQuat()))
            jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)
            _configure_drives(jnt, stiffness, damping, stiffness, damping,
                              bend_limit=max_bend_angle, lock_z=False,
                              twist_limit=twist_limit)

        self._segments[id] = dict(
            path=path,
            global_rot=sub_rot_total,
            radius=scaled_radius,
            height=scaled_length,
            base_pos=world_pos,
        )
        return id

    def add_internode(
        self,
        parent_id: str,
        id: str,
        radius: float,
        length: float,
        density: float = 200.0,
        stiffness: float = 300.0,
        damping_ratio: float = 0.7,
        max_bend_angle: float = 20.0,
        color: tuple = STEM_COLOR,
        physics: bool = False,
    ) -> str:
        """
        Extend a chain by adding a segment in the same direction as its
        parent (straight, tip-to-tip). Used for segments after the first
        one in a petiole chain.
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        if id in self._segments:
            raise ValueError(f"Segment id '{id}' already exists!")

        p = self._segments[parent_id]

        offset_z = p["height"]
        world_pos = p["base_pos"] + p["global_rot"].TransformDir(Gf.Vec3d(0, 0, offset_z))
        orient_qf = _quatd_to_quatf(p["global_rot"].GetQuat())

        path = f"{self.base_path}/{id}"
        xform = UsdGeom.Xform.Define(self.stage, path)
        xform.AddTranslateOp().Set(world_pos)
        xform.AddOrientOp().Set(orient_qf)

        self._make_visual(path, radius, length, color)

        scaled_length = length * self.scale
        scaled_radius = radius * self.scale

        mass = _auto_mass(scaled_radius, scaled_length, density)
        damping = damping_ratio * _critical_damping(stiffness, mass)

        if physics:
            UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
            UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr().Set(mass)

            cyl_prim = self.stage.GetPrimAtPath(f"{path}/Cylinder")
            UsdPhysics.CollisionAPI.Apply(cyl_prim)

            jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
            jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
            jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, offset_z))
            jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            jnt.CreateLocalRot0Attr().Set(IDENTITY_QUATF)
            jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)
            _configure_drives(jnt, stiffness, damping, 0, 0,
                              bend_limit=max_bend_angle, lock_z=True)

        self._segments[id] = dict(
            path=path,
            global_rot=p["global_rot"],
            radius=scaled_radius,
            height=scaled_length,
            base_pos=world_pos,
        )
        return id
