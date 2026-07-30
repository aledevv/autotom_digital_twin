import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf, PhysxSchema

from plant_model.v2.config import PLANT_ROOT_PATH, GLOBAL_SCALE
from plant_model.v2.constants import STEM_COLOR, LEAF_COLOR
from plant_model.v2.plant_builder_utils import (
    _quatd_to_quatf, _configure_drives, _auto_mass, _beam_stiffness, _linear_damping,
)

IDENTITY_QUATF = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
IDENTITY_ROT = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0.0)

# Root prim that holds all collision group prims
_COLL_GROUPS_ROOT = "/World/CollisionGroups"


class PlantBuilder:
    def __init__(self, stage: Usd.Stage, base_path: str = PLANT_ROOT_PATH, scale: float = GLOBAL_SCALE):
        self.stage = stage
        self.base_path = base_path
        self._segments: dict[str, dict] = {}
        self.scale = scale

        # Path of the global tree collision group
        self._tree_coll_group_path: str | None = None

        if not self.stage.GetPrimAtPath(self.base_path):
            UsdGeom.Xform.Define(self.stage, self.base_path)

    # ------------------------------------------------------------------ #
    # COLLISION GROUP HELPERS
    # ------------------------------------------------------------------ #

    def _ensure_coll_groups_root(self) -> None:
        if not self.stage.GetPrimAtPath(_COLL_GROUPS_ROOT):
            UsdGeom.Scope.Define(self.stage, _COLL_GROUPS_ROOT)

    def _create_collision_group(self, group_path: str) -> UsdPhysics.CollisionGroup:
        """Define a CollisionGroup prim, creating parent scopes as needed."""
        self._ensure_coll_groups_root()
        grp = UsdPhysics.CollisionGroup.Define(self.stage, group_path)
        return grp

    def _apply_collision(self, prim: Usd.Prim, contact_offset: float = 0.001, rest_offset: float = 0.0) -> None:
        """Apply CollisionAPI and PhysxCollisionAPI with custom offsets."""
        UsdPhysics.CollisionAPI.Apply(prim)
        
        # Apply Convex Decomposition
        mesh_coll = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_coll.CreateApproximationAttr().Set("convexDecomposition")
        
        physx_coll = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx_coll.CreateContactOffsetAttr().Set(contact_offset)
        physx_coll.CreateRestOffsetAttr().Set(rest_offset)

    def _add_collider_to_group(self, group_path: str, collider_path: str) -> None:
        """Add a collider prim to an existing CollisionGroup.

        PhysX reads membership from a UsdGeom CollectionAPI named 'colliders'
        on the CollisionGroup prim. GetCollidersCollectionAPI() does not exist
        in IsaacSim bindings — write the relationship directly instead.
        """
        grp_prim = self.stage.GetPrimAtPath(group_path)
        if not grp_prim:
            raise RuntimeError(f"CollisionGroup not found: {group_path}")
        coll_api = Usd.CollectionAPI.Apply(grp_prim, "colliders")
        coll_api.GetIncludesRel().AddTarget(Sdf.Path(collider_path))
        print(f"[COLLGROUP] {collider_path} -> {group_path}")

    def _filter_groups(self, group_path_a: str, group_path_b: str) -> None:
        """
        Make group A filter out collisions with group B, and vice-versa.
        PhysX requires the physics:filteredGroups rel on both sides to be symmetric.
        Written as a raw USD relationship to avoid version-specific API differences.
        """
        for src, tgt in ((group_path_a, group_path_b), (group_path_b, group_path_a)):
            src_prim = self.stage.GetPrimAtPath(src)
            if not src_prim:
                continue
            rel = src_prim.GetRelationship("physics:filteredGroups")
            if not rel:
                rel = src_prim.CreateRelationship("physics:filteredGroups", custom=False)
            rel.AddTarget(Sdf.Path(tgt))
            print(f"[COLLGROUP] filter: {src} x {tgt}")

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

        # Create the global tree collision group once
        if physics:
            tree_grp_path = f"{_COLL_GROUPS_ROOT}/Tree"
            self._create_collision_group(tree_grp_path)
            self._tree_coll_group_path = tree_grp_path
            # Branches don't collide with each other
            self._filter_groups(tree_grp_path, tree_grp_path)

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

                cyl_path = f"{path}/Cylinder"
                self._apply_collision(self.stage.GetPrimAtPath(cyl_path))
                self._add_collider_to_group(self._tree_coll_group_path, cyl_path)

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

    def add_articulated_branch(
        self,
        parent_id: str,
        base_id: str,
        total_length: float,
        radius_start: float,
        radius_end: float,
        z_offset_ratio: float,
        tilt_angle: float,
        rot_around_parent: float,
        num_segments: int = 5,
        physics: bool = True,
        youngs_modulus: float = 1.0e8,
        damping_ratio: float = 0.2,
        max_bend_angle: float = 45.0,
        twist_limit: float = 5.0,
        density: float = 800.0,
        color: tuple = LEAF_COLOR,
        branch_collision: bool = True,
    ) -> str:
        """Build an articulated branch chain.

        Builds a chain of `num_segments` D6-jointed cylinder segments.
        The first segment branches off the parent stem via add_lateral_branch;
        subsequent segments extend tip-to-tip via add_internode.

        Collision grouping (when physics=True):
          - A single global group is created for the entire tree.
          - All segments belong to it -> no self-collisions.
        """
        if num_segments <= 0:
            raise ValueError("num_segments must be > 0")

        # Container Xform so the leaf appears as a named group
        group_path = f"{self.base_path}/{base_id}"
        UsdGeom.Xform.Define(self.stage, group_path)

        segment_len = total_length / num_segments
        current_parent = parent_id

        for i in range(num_segments):
            t = i / max(1, num_segments - 1) if num_segments > 1 else 0.0
            r = radius_start + t * (radius_end - radius_start)
            seg_id = f"{base_id}/Seg_{i:02d}"
            
            if i == 0:
                self.add_lateral_branch(
                    parent_id=current_parent, id=seg_id,
                    radius=r, length=segment_len,
                    z_offset_ratio=z_offset_ratio, tilt_angle=tilt_angle,
                    rot_around_parent=rot_around_parent,
                    density=density, youngs_modulus=youngs_modulus, damping_ratio=damping_ratio,
                    max_bend_angle=max_bend_angle,
                    color=color, physics=physics, petiole_collision=branch_collision,
                    coll_group_path=self._tree_coll_group_path
                )
            else:
                self.add_internode(
                    parent_id=current_parent, id=seg_id,
                    radius=r, length=segment_len,
                    density=wood_density, youngs_modulus=youngs_modulus, damping_ratio=damping_ratio,
                    max_bend_angle=max_bend_angle,
                    color=color, physics=physics,
                    coll_group_path=self._tree_coll_group_path,
                    petiole_collision=petiole_collision,
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
        density: float = 800.0,
        youngs_modulus: float = 1.0e9,
        damping_ratio: float = 0.1,
        max_bend_angle: float = 60.0,
        color: tuple = STEM_COLOR,
        physics: bool = False,
        coll_group_path: str | None = None,
        petiole_collision: bool = True,
    ) -> str:
        """
        Attach a new segment to the surface of a parent cylinder, offset
        laterally and tilted away from the parent's axis. Used as the
        first segment of a petiole chain.

        z_offset_ratio    : 0..1, where along the parent's height to attach.
        tilt_angle        : degrees away from the parent's axis.
        rot_around_parent : degrees around the parent's axis (azimuth).
        coll_group_path   : if set, register this segment's cylinder in that group.
        petiole_collision : if False, skip CollisionAPI on this segment's cylinder.
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
        stiffness = _beam_stiffness(scaled_radius, scaled_length, youngs_modulus)
        damping = _linear_damping(stiffness, damping_ratio)

        if physics:
            UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
            UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr().Set(mass)

            # Increase solver iterations for stability of thin/light segments
            from pxr import PhysxSchema
            physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
            physx_rb.CreateSolverPositionIterationCountAttr().Set(32)
            physx_rb.CreateSolverVelocityIterationCountAttr().Set(16)

            cyl_path = f"{path}/Cylinder"
            cyl_prim = self.stage.GetPrimAtPath(cyl_path)
            if petiole_collision:
                self._apply_collision(cyl_prim)
                if coll_group_path:
                    self._add_collider_to_group(coll_group_path, cyl_path)
                
                # Filter collision with parent cylinder explicitly
                parent_cyl_path = f"{p['path']}/Cylinder"
                if self.stage.GetPrimAtPath(parent_cyl_path):
                    filt = UsdPhysics.FilteredPairsAPI.Apply(cyl_prim)
                    filt.CreateFilteredPairsRel().AddTarget(Sdf.Path(parent_cyl_path))

            print(f"[PHYSICS] lateral {id}: mass={mass:.4f}kg  stiff={stiffness:.2f}  "
                  f"damp={damping:.4f}  collision={petiole_collision}")

            jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
            jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
            jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            jnt.CreateLocalPos0Attr().Set(local_pos0)
            jnt.CreateLocalRot0Attr().Set(_quatd_to_quatf(sub_rot_local.GetQuat()))
            jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)
            _configure_drives(jnt, stiffness, damping, stiffness, damping, lock_z=False, max_bend_angle=max_bend_angle)

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
        density: float = 800.0,
        youngs_modulus: float = 1.0e9,
        damping_ratio: float = 0.1,
        max_bend_angle: float = 60.0,
        color: tuple = STEM_COLOR,
        physics: bool = False,
        coll_group_path: str | None = None,
        petiole_collision: bool = True,
    ) -> str:
        """
        Extend a chain by adding a segment in the same direction as its
        parent (straight, tip-to-tip). Used for segments after the first
        one in a petiole chain.

        coll_group_path   : if set, register this segment's cylinder in that group.
        petiole_collision : if False, skip CollisionAPI on this segment's cylinder.
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
        stiffness = _beam_stiffness(scaled_radius, scaled_length, youngs_modulus)
        damping = _linear_damping(stiffness, damping_ratio)

        if physics:
            UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
            UsdPhysics.MassAPI.Apply(xform.GetPrim()).CreateMassAttr().Set(mass)
            
            # Increase solver iterations for stability of thin/light segments
            from pxr import PhysxSchema
            physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(xform.GetPrim())
            physx_rb.CreateSolverPositionIterationCountAttr().Set(32)
            physx_rb.CreateSolverVelocityIterationCountAttr().Set(16)

            cyl_path = f"{path}/Cylinder"
            cyl_prim = self.stage.GetPrimAtPath(cyl_path)
            if petiole_collision:
                self._apply_collision(cyl_prim)
                if coll_group_path:
                    self._add_collider_to_group(coll_group_path, cyl_path)
                
                # Filter collision with parent cylinder explicitly
                parent_cyl_path = f"{p['path']}/Cylinder"
                if self.stage.GetPrimAtPath(parent_cyl_path):
                    filt = UsdPhysics.FilteredPairsAPI.Apply(cyl_prim)
                    filt.CreateFilteredPairsRel().AddTarget(Sdf.Path(parent_cyl_path))

            print(f"[PHYSICS] internode {id}: mass={mass:.4f}kg  stiff={stiffness:.2f}  "
                  f"damp={damping:.4f}  collision={petiole_collision}")

            jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
            jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
            jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])
            jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, offset_z))
            jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
            jnt.CreateLocalRot0Attr().Set(IDENTITY_QUATF)
            jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)
            _configure_drives(jnt, stiffness, damping, 0, 0, lock_z=True, max_bend_angle=max_bend_angle)

        self._segments[id] = dict(
            path=path,
            global_rot=p["global_rot"],
            radius=scaled_radius,
            height=scaled_length,
            base_pos=world_pos,
        )
        return id

