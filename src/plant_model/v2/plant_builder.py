from plant_model.v2.plant_builder_utils import _critical_damping
import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
VERSION_DIR  = os.path.dirname(SCRIPT_DIR)      
SRC_DIR      = os.path.dirname(VERSION_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
from plant_model.v2.config import PLANT_ROOT_PATH, GLOBAL_SCALE
from plant_model.v2.constants import STEM_COLOR, LEAF_COLOR
from plant_model.v2.plant_builder_utils import _quatd_to_quatf, _configure_drives, _auto_mass

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
    # VISUAL (placeholder only — will be replaced by skinned mesh later)
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
    # PHYSICS (optional, disabled by default for the main stem)
    # ------------------------------------------------------------------ #
    def _apply_physics(self, prim: Usd.Prim, mass: float, anchor_to_world: bool = False):
        """Rigid body + mass. If anchor_to_world, add a single FixedJoint."""
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr().Set(mass)

        if anchor_to_world:
            fj = UsdPhysics.FixedJoint.Define(self.stage, f"{prim.GetPath()}/FixedJoint")
            fj.CreateBody1Rel().SetTargets([prim.GetPath()])

        # --- FUTURE: per-internode articulation ---
        # If we ever split the stem into real joints, add D6 joints here
        # between consecutive internode ranks, reusing the same
        # stiffness/damping profile approach as add_lateral_branch.
        # For now the stem is a single rigid stick, no internal joints.

    # ------------------------------------------------------------------ #
    # PUBLIC API
    # ------------------------------------------------------------------ #
    # plant_builder.py

    # plant_builder.py

    def add_main_stem_segments(
        self,
        base_id: str,
        segments: list[dict],
        color: tuple = STEM_COLOR,
        mass_per_segment: float = 1.0,
        physics: bool = False,
    ) -> dict[tuple[int, int], str]:
        """
        Builds ONLY the main trunk chain (order=0), stacked vertically.
        Lateral branches (order>0) must be attached separately via
        add_lateral_branch/add_internode chains, anchored to a specific
        point on the trunk — NOT stacked here.
        """
        trunk_segments = [s for s in segments if s["order"] == 0]
        if len(trunk_segments) != len(segments):
            skipped = len(segments) - len(trunk_segments)
            print(f"[WARN] add_main_stem_segments: ignoring {skipped} non-trunk "
                f"(order>0) segments — build those via add_lateral_branch instead.")

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
                path=path, depth=0, global_rot=IDENTITY_ROT,
                radius=seg["radius"] * self.scale, height=scaled_length,
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
        num_segments: int = 2,
        physics: bool = False,
        stiffness_base: float = 5000.0,
        stiffness_tip: float = 1000.0,
        damping_ratio: float = 0.7,
        max_bend_angle: float = 10.0,
        twist_limit: float = 15.0,
        density: float = 200.0,
        color: tuple = LEAF_COLOR,
    ) -> str:
        if num_segments <= 0:
            raise ValueError("num_segments must be > 0")

        # Create a group Xform so the leaf appears as a named container
        # in the stage hierarchy with its segments nested underneath.
        group_path = f"{self.base_path}/{base_id}"
        UsdGeom.Xform.Define(self.stage, group_path)

        segment_len = total_length / num_segments
        current_parent = parent_id
        seg_ids = []

        for i in range(num_segments):
            t = i / max(1, num_segments - 1) if num_segments > 1 else 0.0
            r = radius_start + t * (radius_end - radius_start)
            stiff = stiffness_base + t * (stiffness_tip - stiffness_base)

            # f"{base_id}/Seg_{i:02d}" → path becomes base_path/{base_id}/Seg_{i:02d}
            # which is nested under the group Xform created above.
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

            seg_ids.append(seg_id)
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
        first segment of a leaf or branch chain.

        z_offset_ratio : 0..1, where along the parent's height to attach.
        tilt_angle      : degrees away from the parent's axis.
        rot_around_parent : degrees around the parent's axis (azimuth).
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        if id in self._segments:
            raise ValueError(f"Segment id '{id}' already exists!")

        p = self._segments[parent_id]

        parent_radius = p["radius"]
        rel_z = z_offset_ratio * p["height"]

        rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_parent)
        tilt_r = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_angle)

        sub_rot_local = tilt_r * rot_z
        sub_rot_total = sub_rot_local * p["global_rot"]

        # world_pos: branch Xform placed on the parent's central axis at rel_z.
        # V1 always branches from (0, 0, tip_z) — the stem center — so the petiole
        # axis passes through the stem axis. Using parent_radius here was shifting
        # the branch base sideways to the surface of the cylinder (wrong visually).
        axis_offset = Gf.Vec3d(0.0, 0.0, rel_z)
        world_pos = p["base_pos"] + p["global_rot"].TransformDir(axis_offset)

        # local_pos0 for the physics joint: pivot on the parent axis at rel_z.
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
            path=path, depth=p.get("depth", 0) + 1, global_rot=sub_rot_total,
            radius=scaled_radius, height=scaled_length, base_pos=world_pos,
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
        one in a tapering chain (leaf, branch).
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
            path=path, depth=p.get("depth", 0) + 1, global_rot=p["global_rot"],
            radius=scaled_radius, height=scaled_length, base_pos=world_pos,
        )
        return id