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
                path=path,
                global_rot=IDENTITY_ROT,
                radius=seg["radius"] * self.scale,
                height=scaled_length,
                base_pos=Gf.Vec3d(0, 0, current_z),
            )

            current_z += scaled_length

        return order_rank_to_id

    # ------------------------------------------------------------------ #
    # LEAF BLADE HELPERS
    # ------------------------------------------------------------------ #

    def _make_leaf_mesh(self, path: str, half_width: float, length: float) -> UsdGeom.Mesh:
        """Create a 16-point ovate leaf blade mesh in the local XY plane.

        The blade base is at the local origin; tip is at (0, length, 0).
        The mesh is visual-only (no CollisionAPI applied here).
        """
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        pts = [Gf.Vec3f(0, 0, 0)]  # base attachment point
        n_side = 7
        for i in range(1, n_side):
            t = i / n_side
            x = half_width * math.sin(math.pi * t) * (1.2 - 0.4 * t)
            pts.append(Gf.Vec3f(x, t * length, 0))
        pts.append(Gf.Vec3f(0, length, 0))  # tip
        for i in range(n_side - 1, 0, -1):
            t = i / n_side
            x = half_width * math.sin(math.pi * t) * (1.2 - 0.4 * t)
            pts.append(Gf.Vec3f(-x, t * length, 0))
        mesh.GetPointsAttr().Set(pts)
        n_tri = len(pts) - 2
        mesh.GetFaceVertexCountsAttr().Set([3] * n_tri)
        idx = []
        for i in range(1, len(pts) - 1):
            idx.extend([0, i, i + 1])
        mesh.GetFaceVertexIndicesAttr().Set(idx)
        mesh.GetSubdivisionSchemeAttr().Set("none")
        mesh.GetDisplayColorAttr().Set([Gf.Vec3f(*LEAF_COLOR)])
        return mesh

    def _attach_compound_blades(
        self,
        base_id: str,
        segment_len: float,
        num_segments: int,
        petiole_length: float,
        blades_nr: int,
        area_array: list[float],
        seg_len_array: list[float],
        incl_array: list[float],
        petiolule_length: float = 0.01,
        blade_inclination_override: float | None = 50.0,
        blade_collision: bool = False,
    ) -> None:
        """Attach static blade meshes to the existing rachis segment chain.

        Lateral leaflet pairs are placed along the rachis at positions derived
        from leaf_segments_length (CSV). A terminal leaflet is placed at the
        rachis tip. All blades are static USD Mesh children of the nearest
        rachis segment Xform — no extra rigid bodies or joints.

        Parameters
        ----------
        base_id       : leaf base ID, used to look up segment paths.
        segment_len   : length of each rachis segment (scaled).
        num_segments  : total number of rachis segments.
        petiole_length: petiole portion of the chain (scaled), blades start after this.
        blades_nr     : total leaflet count (pairs + 1 terminal) from CSV.
        area_array    : per-leaflet area (m²) from leaf_area_m2blades.
        seg_len_array : rachis inter-leaflet distances from leaf_segments_length.
        incl_array    : insertion angles (deg) from leaf_inclination_segments.
        petiolule_length : visual-only petiolule cylinder length (scaled).
        blade_inclination_override : if set, overrides incl_array for all leaflets.
        blade_collision : apply CollisionAPI to blade meshes.
        """
        pairs = blades_nr - 1
        if pairs <= 0 and blades_nr <= 0:
            return

        scale = self.scale

        def _seg_for_distance(d_scaled: float) -> tuple[str, float]:
            """Return (segment_id, local_z_ratio) for a scaled distance along the chain."""
            seg_idx = min(int(d_scaled / segment_len), num_segments - 1)
            local_d = d_scaled - seg_idx * segment_len
            ratio = min(max(local_d / segment_len, 0.0), 1.0)
            return f"{base_id}/Seg_{seg_idx:02d}", ratio

        def _blade_orient_quat(rot_around_z_deg: float, insertion_deg: float) -> Gf.Quatf:
            """Quaternion: azimuth around segment Z, then tilt by insertion angle around X.

            The blade mesh grows in +Y from its local origin. The resulting
            orientation places the blade so that +Y aligns with the leaflet
            growth direction (lateral out from rachis or along rachis for terminal).
            """
            from plant_model.v2.plant_builder_utils import _quatd_to_quatf
            rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_z_deg)
            # insertion_deg: 90=perpendicular to rachis (straight out), 0=along rachis
            tilt = Gf.Rotation(Gf.Vec3d(1, 0, 0), -(90.0 - insertion_deg))
            combined = tilt * rot_z
            return _quatd_to_quatf(combined.GetQuat())

        # current_d tracks distance along the full chain (petiole + rachis)
        current_d = petiole_length * scale

        # ── Lateral leaflet pairs ──────────────────────────────────────────
        for j in range(pairs):
            area = area_array[j] if j < len(area_array) else 0.0
            if area <= 0:
                area = 1e-4
            lat_area = area / 2.0
            lat_length = math.sqrt(lat_area / 0.6) * scale
            lat_width  = lat_length * 0.6

            insertion = (
                blade_inclination_override
                if blade_inclination_override is not None
                else (incl_array[j] if j < len(incl_array) else 90.0)
            )

            target_seg_id, ratio = _seg_for_distance(current_d)
            seg_path = f"{self.base_path}/{target_seg_id}"
            seg_data = self._segments.get(target_seg_id)
            if seg_data is None:
                print(f"[WARN] _attach_compound_blades: segment '{target_seg_id}' not found, skipping pair {j}")
                dist_to_next = seg_len_array[j] * scale if j < len(seg_len_array) else segment_len
                current_d += dist_to_next
                continue

            local_z = ratio * seg_data["height"]
            pet_len = petiolule_length * scale

            for side_label, az_deg in (("R", 90.0), ("L", -90.0)):
                blade_xf_path = f"{seg_path}/Lat{j}{side_label}"
                xf = UsdGeom.Xform.Define(self.stage, blade_xf_path)
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, local_z))
                xf.AddOrientOp().Set(_blade_orient_quat(az_deg, insertion))

                # Visual-only petiolule cylinder
                pet_prim = UsdGeom.Cylinder.Define(self.stage, f"{blade_xf_path}/Petiolule")
                pet_prim.GetRadiusAttr().Set(seg_data["radius"] * 0.4)
                pet_prim.GetHeightAttr().Set(pet_len)
                pet_prim.GetAxisAttr().Set("Z")
                pet_prim.AddTranslateOp().Set(Gf.Vec3d(0, 0, pet_len / 2.0))
                pet_prim.GetDisplayColorAttr().Set([Gf.Vec3f(*LEAF_COLOR)])

                # Blade mesh: local origin at petiolule tip, grows in +Y
                blade_xf2 = UsdGeom.Xform.Define(self.stage, f"{blade_xf_path}/BladeXform")
                blade_xf2.ClearXformOpOrder()
                blade_xf2.AddTranslateOp().Set(Gf.Vec3d(0, 0, pet_len))
                # Rotate so blade +Y aligns with petiolule +Z (growth direction)
                rot90x = Gf.Quatf(0.7071068, 0.7071068, 0.0, 0.0)
                blade_xf2.AddOrientOp().Set(rot90x)

                mesh = self._make_leaf_mesh(
                    f"{blade_xf_path}/BladeXform/Mesh",
                    lat_width / 2.0, lat_length
                )
                if blade_collision:
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(
                        mesh.GetPrim()).GetApproximationAttr().Set("convexHull")

            dist_to_next = (
                seg_len_array[j] * scale if j < len(seg_len_array)
                else segment_len
            )
            current_d += dist_to_next

        # ── Terminal leaflet ───────────────────────────────────────────────
        if blades_nr > 0 and len(area_array) > 0:
            term_area   = area_array[-1]
            term_length = math.sqrt(max(term_area, 1e-6) / 0.6) * scale
            term_width  = term_length * 0.6

            tip_seg_id = f"{base_id}/Seg_{num_segments - 1:02d}"
            tip_seg_path = f"{self.base_path}/{tip_seg_id}"
            tip_seg_data = self._segments.get(tip_seg_id)

            if tip_seg_data is not None:
                pet_len = petiolule_length * scale
                tip_local_z = tip_seg_data["height"]  # tip of last segment

                term_xf_path = f"{tip_seg_path}/Terminal"
                xf = UsdGeom.Xform.Define(self.stage, term_xf_path)
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, tip_local_z))
                # 0° insertion = blade grows straight along rachis (+Z → +Y after 90x rot)
                xf.AddOrientOp().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

                pet_prim = UsdGeom.Cylinder.Define(self.stage, f"{term_xf_path}/Petiolule")
                pet_prim.GetRadiusAttr().Set(tip_seg_data["radius"] * 0.4)
                pet_prim.GetHeightAttr().Set(pet_len)
                pet_prim.GetAxisAttr().Set("Z")
                pet_prim.AddTranslateOp().Set(Gf.Vec3d(0, 0, pet_len / 2.0))
                pet_prim.GetDisplayColorAttr().Set([Gf.Vec3f(*LEAF_COLOR)])

                blade_xf2 = UsdGeom.Xform.Define(self.stage, f"{term_xf_path}/BladeXform")
                blade_xf2.ClearXformOpOrder()
                blade_xf2.AddTranslateOp().Set(Gf.Vec3d(0, 0, pet_len))
                rot90x = Gf.Quatf(0.7071068, 0.7071068, 0.0, 0.0)
                blade_xf2.AddOrientOp().Set(rot90x)

                mesh = self._make_leaf_mesh(
                    f"{term_xf_path}/BladeXform/Mesh",
                    term_width / 2.0, term_length
                )
                if blade_collision:
                    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
                    UsdPhysics.MeshCollisionAPI.Apply(
                        mesh.GetPrim()).GetApproximationAttr().Set("convexHull")

                print(f"   🍃 Terminal leaflet {term_length/scale*100:.1f}cm × {term_width/scale*100:.1f}cm")

        print(f"   🍀 Attached {pairs} lateral pairs + terminal to '{base_id}'")

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
        # ── Compound leaf blades ──────────────────────────────────────────
        blade_enabled: bool = True,
        blades_nr: int = 0,
        area_array: list[float] | None = None,
        seg_len_array: list[float] | None = None,
        incl_array: list[float] | None = None,
        petiolule_length: float = 0.01,
        blade_inclination_override: float | None = 50.0,
        blade_collision: bool = False,
    ) -> str:
        """Build an articulated petiole/rachis chain and attach compound leaf blades.

        The rachis is a chain of `num_petiole_segments` D6-jointed cylinder
        segments. After the chain is built, `_attach_compound_blades` places
        static blade meshes (lateral pairs + terminal) driven by CSV data.

        Parameters
        ----------
        num_petiole_segments : number of articulation segments (1 = rigid).
        blade_enabled        : if False, skip blade attachment entirely.
        blades_nr            : total leaflet count from CSV (blades_nr field).
        area_array           : per-leaflet blade area (m²) list.
        seg_len_array        : inter-leaflet rachis distances (m).
        incl_array           : leaflet insertion angles (deg).
        petiolule_length     : visual-only petiolule cylinder length (m, unscaled).
        blade_inclination_override : override for all lateral insertion angles.
        blade_collision      : apply CollisionAPI to blade meshes.
        """
        if num_petiole_segments <= 0:
            raise ValueError("num_petiole_segments must be > 0")

        # Create a group Xform so the leaf appears as a named container
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

        # ── Attach compound leaf blades ────────────────────────────────────
        if blade_enabled and blades_nr > 0 and area_array:
            seg_data_0 = self._segments.get(f"{base_id}/Seg_00")
            scaled_seg_len = seg_data_0["height"] if seg_data_0 else segment_len * self.scale
            # petiole_length (unscaled) determines where along the chain blades begin
            # We approximate it as 0 (blades start from segment 0 of the rachis).
            # The caller should pass the actual petiole length if known.
            self._attach_compound_blades(
                base_id=base_id,
                segment_len=scaled_seg_len,
                num_segments=num_petiole_segments,
                petiole_length=0.0,  # blades distributed across the whole chain
                blades_nr=blades_nr,
                area_array=area_array,
                seg_len_array=seg_len_array or [],
                incl_array=incl_array or [],
                petiolule_length=petiolule_length,
                blade_inclination_override=blade_inclination_override,
                blade_collision=blade_collision,
            )

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
        rel_z = z_offset_ratio * p["height"]

        rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_parent)
        tilt_r = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_angle)

        sub_rot_local = tilt_r * rot_z
        sub_rot_total = sub_rot_local * p["global_rot"]

        # world_pos: branch Xform placed on the parent's central axis at rel_z.
        # Branching from the stem center (not the cylinder surface) so the
        # petiole axis passes through the stem axis — matching v1 behaviour.
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
            path=path,
            global_rot=p["global_rot"],
            radius=scaled_radius,
            height=scaled_length,
            base_pos=world_pos,
        )
        return id
