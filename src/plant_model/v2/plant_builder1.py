"""
plant_builder.py

A high-level API for building articulated plant structures in OpenUSD.
Provides primitive operations (create_root, add_internode, add_lateral_branch)
that hide all the quaternion math, joint configuration, and PhysX boilerplate.

Usage:
    builder = PlantBuilder(stage, "/World/Stem")
    root = builder.create_root("Trunk_01", radius=0.1, length=0.5)
    seg2 = builder.add_internode(root, "Trunk_02", radius=0.1, length=0.5)
    br1  = builder.add_lateral_branch(seg2, "Branch_01", radius=0.04, length=0.3,
                                      z_offset_ratio=0.8, tilt_angle=45, rot_around_parent=90)
"""

from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_MAX_BEND_ANGLE
from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_DENSITY
from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_DAMPING_TIP
from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_DAMPING_BASE
from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_STIFFNESS_TIP
from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_STIFFNESS_BASE
from plant_model.v2.builder_constants1 import LEAFLET_PETIOLULE_SEGMENTS
from plant_model.v2.builder_constants1 import COMPOUND_LEAF_MAX_BEND_ANGLE
from plant_model.v2.builder_constants1 import COMPOUND_LEAF_DENSITY
from plant_model.v2.builder_constants1 import COMPOUND_LEAF_DAMPING_TIP
from plant_model.v2.builder_constants1 import COMPOUND_LEAF_DAMPING_BASE
from plant_model.v2.builder_constants1 import COMPOUND_LEAF_STIFFNESS_TIP
from plant_model.v2.builder_constants1 import COMPOUND_LEAF_STIFFNESS_BASE
import math
from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf
from .builder_constants1 import (
    PLANT_COLOR, LEAF_COLOR, FRUIT_RIPE, FRUIT_YOUNG, GAP,
    MAX_ARTICULATION_DEPTH, DEPTH_WARNING_THRESHOLD, ASPECT_RATIO_WARNING,
    TRUNK_STIFFNESS, TRUNK_DAMPING, BRANCH_STIFFNESS, BRANCH_DAMPING,
    LATERAL_STIFFNESS, LATERAL_DAMPING,
)

# =============================================================================
# HELPER: safe Quatd -> Quatf conversion
# =============================================================================
def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    """Convert a double-precision quaternion to float-precision,
    using the explicit (real, i, j, k) constructor to avoid any
    ambiguity in the Python bindings."""
    imag = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()),
                    float(imag[0]), float(imag[1]), float(imag[2]))


IDENTITY_QUATF = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
IDENTITY_ROT = Gf.Rotation(Gf.Vec3d(0, 0, 1), 0.0)


class PlantBuilder:
    """Builds articulated cylinder-based plant structures in a USD stage.

    Every segment is tracked internally so that subsequent calls can
    reference any previously created segment as a parent.
    """

    def __init__(self, stage: Usd.Stage, base_path: str = "/World/Stem", global_scale: float = 1.0):
        self.stage = stage
        self.base_path = base_path
        self.global_scale = global_scale

        # id -> { path, depth, global_rot (Gf.Rotation), radius, height, base_pos (Gf.Vec3d) }
        self._segments: dict[str, dict] = {}

        # Ensure the ArticulationRoot Xform exists
        if not self.stage.GetPrimAtPath(self.base_path):
            xform = UsdGeom.Xform.Define(self.stage, self.base_path)
            UsdPhysics.ArticulationRootAPI.Apply(xform.GetPrim())

        print(f"🌱 [PlantBuilder] Initialized — ArticulationRoot at {self.base_path} (scale={self.global_scale}x)")

    # --------------------------------------------------------------------- #
    #  SECURITY CHECKS
    # --------------------------------------------------------------------- #
    def _check(self, parent_id: str, new_id: str,
               radius: float, length: float) -> int:
        """Run safety validations, return the depth of the new segment."""
        depth = 0
        if parent_id and parent_id in self._segments:
            depth = self._segments[parent_id]["depth"] + 1

        if depth >= MAX_ARTICULATION_DEPTH:
            raise ValueError(
                f"🚫 [PlantBuilder] Cannot add '{new_id}': depth {depth} "
                f"exceeds PhysX articulation limit of {MAX_ARTICULATION_DEPTH} links!")
        if depth > DEPTH_WARNING_THRESHOLD:
            print(f"⚠️  [PlantBuilder] '{new_id}' depth={depth} — "
                  f"approaching PhysX limit ({MAX_ARTICULATION_DEPTH}). Consider simplifying the chain.")

        aspect = length / radius if radius > 0 else 0
        if aspect > ASPECT_RATIO_WARNING:
            print(f"⚠️  [PlantBuilder] '{new_id}' aspect ratio = {aspect:.1f} "
                  f"(length/radius). Thin segments cause physics jittering.")

        if new_id in self._segments:
            raise ValueError(
                f"🚫 [PlantBuilder] Segment ID '{new_id}' already exists!")

        return depth

    # --------------------------------------------------------------------- #
    #  LOW-LEVEL USD HELPERS  (match the working code exactly)
    # --------------------------------------------------------------------- #
    def _make_cylinder(self, path: str, radius: float, height: float,
                       world_pos: Gf.Vec3d, orient_quatf: Gf.Quatf,
                       mass: float, color: tuple = None) -> str:
        """Create a rigid-body cylinder at the given world position/orientation."""
        xform = UsdGeom.Xform.Define(self.stage, path)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(world_pos)
        xform.AddOrientOp().Set(orient_quatf)          # ← Quatf directly

        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass_api.CreateMassAttr().Set(mass)

        cyl_path = f"{path}/Cylinder"
        cyl = UsdGeom.Cylinder.Define(self.stage, cyl_path)
        cyl.GetRadiusAttr().Set(radius)
        cyl.GetHeightAttr().Set(height)
        cyl.GetAxisAttr().Set("Z")
        cyl.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height / 2.0))
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())

        if color is not None:
            cyl.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

        return path

    def _configure_drives(self, joint, stiff_xy, damp_xy,
                          stiff_z, damp_z, bend_limit, lock_z):
        """Set up D6 joint limits and drives (translation locked, rotation limited)."""
        prim = joint.GetPrim()
        for ax in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(prim, ax)
            lim.CreateLowAttr().Set(1.0)        # low > high ⇒ locked
            lim.CreateHighAttr().Set(-1.0)

        for ax in ("rotX", "rotY"):
            lim = UsdPhysics.LimitAPI.Apply(prim, ax)
            lim.CreateLowAttr().Set(-bend_limit)
            lim.CreateHighAttr().Set(bend_limit)
            drv = UsdPhysics.DriveAPI.Apply(prim, ax)
            drv.CreateTypeAttr().Set("force")
            drv.CreateStiffnessAttr().Set(stiff_xy)
            drv.CreateDampingAttr().Set(damp_xy)
            drv.CreateTargetPositionAttr().Set(0.0)

        lim_z = UsdPhysics.LimitAPI.Apply(prim, "rotZ")
        if lock_z:
            lim_z.CreateLowAttr().Set(1.0)
            lim_z.CreateHighAttr().Set(-1.0)
        else:
            drv_z = UsdPhysics.DriveAPI.Apply(prim, "rotZ")
            drv_z.CreateTypeAttr().Set("force")
            drv_z.CreateStiffnessAttr().Set(stiff_z)
            drv_z.CreateDampingAttr().Set(damp_z)
            drv_z.CreateTargetPositionAttr().Set(0.0)

    # --------------------------------------------------------------------- #
    #  PRIMITIVE ACTIONS
    # --------------------------------------------------------------------- #
    def create_root(self, id: str, radius: float, length: float,
                    mass: float = 1.0) -> str:
        """Create the very first segment, anchored to the world with a FixedJoint."""
        depth = self._check("", id, radius, length)

        path = f"{self.base_path}/{id}"
        pos = Gf.Vec3d(0, 0, 0)

        self._make_cylinder(path, radius, length, pos, IDENTITY_QUATF, mass, color=PLANT_COLOR)

        # Anchor to the world (Body0 = None = world)
        fj = UsdPhysics.FixedJoint.Define(self.stage, f"{path}/FixedJoint")
        fj.CreateBody1Rel().SetTargets([Sdf.Path(path)])

        self._segments[id] = dict(
            path=path, depth=depth, global_rot=IDENTITY_ROT,
            radius=radius, height=length, base_pos=pos,
        )
        print(f"🌱 [PlantBuilder] Root '{id}'  r={radius}  L={length}")
        return id

    def add_internode(self, parent_id: str, id: str,
                      radius: float, length: float,
                      mass: float = 1.0,
                      stiffness: float | None = None,
                      damping: float | None = None,
                      color: tuple = PLANT_COLOR) -> str:
        """Extend a branch by adding a segment in the same direction as its parent."""
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        depth = self._check(parent_id, id, radius, length)
        p = self._segments[parent_id]

        # Auto-tune stiffness
        if stiffness is None or damping is None:
            if p["depth"] == 0:          # first generation (trunk)
                stiffness = TRUNK_STIFFNESS
                damping = TRUNK_DAMPING
            else:                        # higher-order branch
                stiffness = BRANCH_STIFFNESS
                damping = BRANCH_DAMPING
            print(f"   ✨ Auto-tuned internode '{id}' → "
                  f"stiffness={stiffness}, damping={damping}")

        # World position: end of parent + gap, along parent's Z axis
        offset_z = p["height"] + GAP
        world_pos = p["base_pos"] + p["global_rot"].TransformDir(
            Gf.Vec3d(0, 0, offset_z))

        orient_qf = _quatd_to_quatf(p["global_rot"].GetQuat())
        path = f"{self.base_path}/{id}"
        self._make_cylinder(path, radius, length, world_pos, orient_qf, mass, color=color)

        # D6 joint: same frame, just offset along local Z
        jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
        jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
        jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, offset_z))
        jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        jnt.CreateLocalRot0Attr().Set(IDENTITY_QUATF)
        jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)
        self._configure_drives(jnt, stiffness, damping, 0, 0,
                               bend_limit=20.0, lock_z=True)

        self._segments[id] = dict(
            path=path, depth=depth, global_rot=p["global_rot"],
            radius=radius, height=length, base_pos=world_pos,
        )
        return id

    def add_lateral_branch(self, parent_id: str, id: str,
                           radius: float, length: float,
                           z_offset_ratio: float,
                           tilt_angle: float,
                           rot_around_parent: float,
                           mass: float = 0.2,
                           stiffness: float | None = None,
                           damping: float | None = None,
                           color: tuple = PLANT_COLOR) -> str:
        """Attach a new branch segment to the surface of a parent cylinder.

        Parameters
        ----------
        z_offset_ratio : 0..1 — where along the parent's height to attach.
        tilt_angle : degrees away from the parent's axis.
        rot_around_parent : degrees around the parent's axis (azimuth).
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        depth = self._check(parent_id, id, radius, length)
        p = self._segments[parent_id]

        if stiffness is None or damping is None:
            stiffness = 184_000.0
            damping = 5_000.0
            print(f"   ✨ Auto-tuned lateral '{id}' → "
                  f"stiffness={stiffness}, damping={damping}")

        # --- geometry math (matches generate_generalized_articulation_usda) ---
        parent_radius = p["radius"]
        rel_z = z_offset_ratio * p["height"]
        local_offset = Gf.Vec3d(0.0, parent_radius, rel_z)

        rot_z   = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_parent)
        tilt_r  = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_angle)

        sub_rot_local = tilt_r * rot_z                    # local frame rotation
        local_pos0    = rot_z.TransformDir(local_offset)  # offset in parent frame

        sub_rot_total = sub_rot_local * p["global_rot"]   # world orientation
        world_pos     = p["base_pos"] + p["global_rot"].TransformDir(local_pos0)

        orient_qf = _quatd_to_quatf(sub_rot_total.GetQuat())
        path = f"{self.base_path}/{id}"
        self._make_cylinder(path, radius, length, world_pos, orient_qf, mass, color)

        # D6 joint — base attachment
        jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
        jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
        jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])

        jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(local_pos0[0]), float(local_pos0[1]), float(local_pos0[2])))
        jnt.CreateLocalRot0Attr().Set(
            _quatd_to_quatf(sub_rot_local.GetQuat()))
        jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)

        self._configure_drives(jnt, stiffness, damping, stiffness, damping,
                               bend_limit=30.0, lock_z=False)

        self._segments[id] = dict(
            path=path, depth=depth, global_rot=sub_rot_total,
            radius=radius, height=length, base_pos=world_pos,
        )
        return id

    def add_branch(self, parent_id: str, base_id: str,
                           total_length: float, start_radius: float, end_radius: float,
                           num_segments: int,
                           z_offset_ratio: float = 1.0,
                           tilt_angle: float = 45.0,
                           rot_around_parent: float = 0.0,
                           stiffness_base: float = 10000.0,
                           stiffness_tip: float = 1000.0,
                           damping_base: float = 1000.0,
                           damping_tip: float = 100.0,
                           density: float = 500.0,
                           max_bend_angle: float = 30.0,
                           color: tuple = PLANT_COLOR ) -> str:
        """Procedurally generate a multi-segment, tapering branch.
        Returns the ID of the last segment (the tip).
        """
        if num_segments <= 0:
            raise ValueError("num_segments must be > 0")
            
        segment_len = total_length / num_segments
        current_parent = parent_id
        
        for i in range(num_segments):
            # Tapering logic
            t = i / max(1, num_segments - 1) if num_segments > 1 else 0.0
            r = start_radius + t * (end_radius - start_radius)
            stiff = stiffness_base + t * (stiffness_tip - stiffness_base)
            damp = damping_base + t * (damping_tip - damping_base)
            
            # Safety checks
            if r < 0.0005:
                print(f"⚠️  [PlantBuilder] Tapered branch '{base_id}' segment {i} radius {r:.4f} is too small! PhysX may become unstable.")
                print(f"ℹ️  [PlantBuilder] Automatically clamped radius to 0.0005.")
                r = 0.0005
                
            if segment_len < 2 * r:
                print(f"⚠️  [PlantBuilder] Tapered branch '{base_id}' segment {i} has L/R ratio < 2 (L={segment_len:.4f}, D={2*r:.4f}). PhysX may jitter.")
                
            # Physics Mass
            mass = math.pi * (r**2) * segment_len * density
            mass = max(mass, 0.05) # Minimum 50g mass for stability
            
            seg_id = f"{base_id}_{i:02d}"
            
            if i == 0:
                # First segment attaches laterally to the main parent
                self.add_lateral_branch(
                    parent_id=current_parent,
                    id=seg_id,
                    radius=r,
                    length=segment_len,
                    z_offset_ratio=z_offset_ratio,
                    tilt_angle=tilt_angle,
                    rot_around_parent=rot_around_parent,
                    mass=mass,
                    stiffness=stiff,
                    damping=damp,
                    color=color
                )
            else:
                # Subsequent segments attach straight to the previous segment
                self.add_internode(
                    parent_id=current_parent,
                    id=seg_id,
                    radius=r,
                    length=segment_len,
                    mass=mass,
                    stiffness=stiff,
                    damping=damp,
                    color=color
                )
                
            # Apply max bend limit
            jnt_path = f"{self.base_path}/{seg_id}/Joint"
            jnt = UsdPhysics.Joint(self.stage.GetPrimAtPath(jnt_path))
            if jnt:
                self._configure_drives(jnt, stiff, damp, stiff, damp, bend_limit=max_bend_angle, lock_z=False)
                    
            current_parent = seg_id
            
        return current_parent

    # ----------------------------------------------------------------- #
    #  LEAF
    # ----------------------------------------------------------------- #
    def _make_leaf_mesh(self, path: str, half_width: float, length: float, color: tuple = LEAF_COLOR):
        """Create a simple ovate leaf blade mesh (16-point smooth shape)."""
        mesh = UsdGeom.Mesh.Define(self.stage, path)
        pts = [Gf.Vec3f(0, 0, 0)]  # base
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
        mesh.GetDisplayColorAttr().Set([Gf.Vec3f(*color)])

        return mesh

    def attach_blade(self, parent_id: str, id: str,
                     leaf_length: float = 0.08, leaf_width: float = 0.04,
                     z_offset_ratio: float = 1.0,
                     tilt_angle: float = 0.0,
                     rot_around_parent: float = 0.0,
                     extra_rot_y: float = 0.0,
                     enable_collision: bool = True) -> str:
        """Attach a static leaf blade mesh to an existing segment.
        This does NOT create a new physics body or joint. It moves with the parent.

        Rotation order (applied right-to-left on the mesh Y-axis):
          1. rot_around_parent: azimuth around parent Z (which lateral side)
          2. tilt_angle:        tilt around X (negative = up, 0 = perpendicular out, +90 = along Z)
          3. extra_rot_y:       additional spin around Y (use 90 to face outward after tilt)
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        p = self._segments[parent_id]
        
        rel_z = z_offset_ratio * p["height"]
        
        rot_z  = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_parent)
        tilt_r = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_angle)
        rot_y  = Gf.Rotation(Gf.Vec3d(0, 1, 0), extra_rot_y)
        sub_rot_local = rot_y * tilt_r * rot_z
        rot_q = _quatd_to_quatf(sub_rot_local.GetQuat())
        
        # Offset slightly out from the center in the rotated Y direction
        offset_dir = rot_z.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
        local_offset = Gf.Vec3d(offset_dir[0] * p["radius"], offset_dir[1] * p["radius"], rel_z)

        blade_path = f"{p['path']}/{id}"
        blade_xform = UsdGeom.Xform.Define(self.stage, blade_path)
        blade_xform.AddTranslateOp().Set(local_offset)
        blade_xform.AddOrientOp().Set(rot_q)

        mesh_path = f"{blade_path}/Mesh"
        mesh = self._make_leaf_mesh(mesh_path, leaf_width / 2.0, leaf_length)

        if enable_collision:
            UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).GetApproximationAttr().Set("convexHull")
            
        
        return blade_path

    def add_compound_leaf(self, parent_id: str, base_id: str,
                          petiole_length: float, rachis_length: float,
                          start_radius: float, end_radius: float,
                          num_segments: int,
                          blade_area_array: list[float],
                          seg_len_array: list[float],
                          incl_array: list[float],
                          z_offset_ratio: float = 1.0,
                          tilt_angle: float = 60.0,
                          lateral_tilt_angle: float = 70.0,
                          rot_around_parent: float = 0.0) -> str:
        """Procedurally generate a physically articulated compound leaf.

        Rachis is built with add_branch. Each lateral leaflet is an articulated
        petiolule (add_branch, 1 seg) with a static blade (attach_blade) at its
        tip — the rack (rastrelliera) pattern. The terminal leaflet is static on the
        last rachis segment, oriented along the rachis direction.
        """
        import math
        
        total_length = petiole_length + rachis_length
        segment_len = total_length / max(num_segments, 1)


        # 1. Generate articulated rachis
        tip_id = self.add_branch(
            parent_id=parent_id,
            base_id=base_id,
            total_length=total_length,
            start_radius=start_radius,
            end_radius=end_radius,
            num_segments=num_segments,
            z_offset_ratio=z_offset_ratio,
            tilt_angle=tilt_angle,
            rot_around_parent=rot_around_parent,
            stiffness_base=COMPOUND_LEAF_STIFFNESS_BASE,  # Leaves are very soft
            stiffness_tip=COMPOUND_LEAF_STIFFNESS_TIP,
            damping_base=COMPOUND_LEAF_DAMPING_BASE,
            damping_tip=COMPOUND_LEAF_DAMPING_TIP,
            density=COMPOUND_LEAF_DENSITY,
            max_bend_angle=COMPOUND_LEAF_MAX_BEND_ANGLE
        )
        
        n_blades = len(blade_area_array)
        if n_blades == 0:
            return tip_id

        pairs = n_blades - 1
        petiolule_len = max(segment_len * 0.5, 0.005)
        petiolule_rad = max(end_radius * 0.4, 0.001)

        def _seg_for_distance(d: float) -> tuple[str, float]:
            """Return (segment_id, local_z_ratio) for a given distance along the leaf."""
            seg_idx = min(int(d / segment_len), num_segments - 1)
            local_d = d - seg_idx * segment_len
            ratio   = min(max(local_d / segment_len, 0.0), 1.0)
            return f"{base_id}_{seg_idx:02d}", ratio

        # 2. Lateral leaflets — rack (rastrelliera) pattern:
        #    each lateral is an articulated petiolule (add_branch, 1 seg) + static blade (attach_blade)
        current_d = petiole_length
        for j in range(pairs):
            area       = blade_area_array[j]
            lat_area   = area / 2.0        # each leaflet = pair_area / 2
            lat_length = math.sqrt(lat_area / 0.6) if lat_area > 0 else 0.001
            lat_width  = lat_length * 0.6

            # insertion_angle from CSV: 90° = perpendicular to rachis (stick straight out)
            insertion = incl_array[j] if j < len(incl_array) else 90.0
            target_seg, ratio = _seg_for_distance(current_d)

            for side_label, rot_sign in (("R", 90.0), ("L", -90.0)):
                pet_id = f"{base_id}_Lat{j}{side_label}"
                self.add_branch(
                    parent_id=target_seg,
                    base_id=pet_id,
                    total_length=petiolule_len,
                    start_radius=petiolule_rad,
                    end_radius=petiolule_rad * 0.7,
                    num_segments=LEAFLET_PETIOLULE_SEGMENTS,
                    z_offset_ratio=ratio,
                    tilt_angle=lateral_tilt_angle,
                    rot_around_parent=rot_sign,
                    stiffness_base=LEAFLET_PETIOLULE_STIFFNESS_BASE,
                    stiffness_tip=LEAFLET_PETIOLULE_STIFFNESS_TIP,
                    damping_base=LEAFLET_PETIOLULE_DAMPING_BASE,
                    damping_tip=LEAFLET_PETIOLULE_DAMPING_TIP,
                    density=LEAFLET_PETIOLULE_DENSITY,
                    max_bend_angle=LEAFLET_PETIOLULE_MAX_BEND_ANGLE
                )
                # tilt_angle=-90 → Gf.Rotation(X, +90°) → blade Y aligns with petiolule Z (growth dir)
                # extra_rot_y=90 → spin 90° around Y so the flat face faces outward
                self.attach_blade(
                    parent_id=f"{pet_id}_{1:02d}",
                    id="Blade",
                    leaf_length=max(lat_length, 0.001),
                    leaf_width=max(lat_width,   0.001),
                    z_offset_ratio=1.0,
                    tilt_angle=-90.0,
                    rot_around_parent=0.0,
                    extra_rot_y=90.0
                )

            dist_to_next = seg_len_array[j] if j < len(seg_len_array) else (rachis_length / max(pairs, 1))
            current_d += dist_to_next

        # 3. Terminal leaflet — static blade on the last rachis segment, growing along rachis
        #    tilt_angle=-90 → blade Y aligns with segment Z (rachis growth direction)
        tip_seg_id  = f"{base_id}_{num_segments - 1:02d}"
        term_area   = blade_area_array[-1]
        term_length = math.sqrt(term_area / 0.6) if term_area > 0 else 0.001
        term_width  = term_length * 0.6
        self.attach_blade(
            parent_id=tip_seg_id,
            id="Blade_Terminal",
            leaf_length=max(term_length, 0.001),
            leaf_width=max(term_width,   0.001),
            z_offset_ratio=1.0,
            tilt_angle=-90.0,
            rot_around_parent=0.0,
            extra_rot_y=0.0
        )

        return tip_id

    def add_leaf(self, parent_id: str, id: str,
                 leaf_length: float = 0.08, leaf_width: float = 0.04,
                 petiole_length: float | None = None,
                 petiole_radius: float | None = None,
                 z_offset_ratio: float = 1.0,
                 tilt_angle: float = 60.0,
                 rot_around_parent: float = 0.0,
                 mass: float = 0.02,
                 stiffness: float = 0.0002,
                 damping: float = 0.00006) -> str:
        """Attach a leaf to a parent segment.

        The petiole is the articulated rigid body (connected to the parent
        via a D6 joint). The leaf blade is static child geometry on the
        petiole — so the joint is branch ↔ petiole, and the whole leaf
        (petiole + blade) swings as one piece.

        Parameters
        ----------
        leaf_length / leaf_width : size of the ovate blade.
        petiole_length : length of the stem connecting blade to branch.
                         Defaults to leaf_length * 0.4 if not given.
        petiole_radius : radius of petiole cylinder.
                         Defaults to parent_radius * 0.15 if not given.
        z_offset_ratio : 0..1 — where along parent to attach.
        tilt_angle : degrees away from parent axis.
        rot_around_parent : azimuth around parent axis.
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        depth = self._check(parent_id, id, 0.01, leaf_length)
        p = self._segments[parent_id]

        parent_radius = p["radius"]
        pet_len = petiole_length if petiole_length else leaf_length * 0.4
        pet_rad = petiole_radius if petiole_radius else max(parent_radius * 0.15, 0.003)

        # ── Compute attachment point & orientation (same math as lateral) ──
        rel_z = z_offset_ratio * p["height"]
        local_offset = Gf.Vec3d(0.0, parent_radius, rel_z)

        rot_z  = Gf.Rotation(Gf.Vec3d(0, 0, 1), rot_around_parent)
        tilt_r = Gf.Rotation(Gf.Vec3d(1, 0, 0), -tilt_angle)

        sub_rot_local = tilt_r * rot_z
        local_pos0    = rot_z.TransformDir(local_offset)
        sub_rot_total = sub_rot_local * p["global_rot"]
        world_pos     = p["base_pos"] + p["global_rot"].TransformDir(local_pos0)

        orient_qf = _quatd_to_quatf(sub_rot_total.GetQuat())

        # ── Petiole: the articulated rigid body ───────────────────────────
        path = f"{self.base_path}/{id}"
        xform = UsdGeom.Xform.Define(self.stage, path)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(world_pos)
        xform.AddOrientOp().Set(orient_qf)

        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        mass_api = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass_api.CreateMassAttr().Set(mass)

        # Petiole cylinder (axis=Z, local frame: grows along petiole's Z)
        pet = UsdGeom.Cylinder.Define(self.stage, f"{path}/Petiole")
        pet.GetRadiusAttr().Set(pet_rad)
        pet.GetHeightAttr().Set(pet_len)
        pet.GetAxisAttr().Set("Z")
        pet.AddTranslateOp().Set(Gf.Vec3d(0, 0, pet_len / 2.0))
        UsdPhysics.CollisionAPI.Apply(pet.GetPrim())

        # ── Blade: static child mesh at the petiole tip ───────────────────
        # The blade grows in local +Y starting at the petiole tip (Z = pet_len).
        # We place a sub-xform to position & rotate the blade at the tip.
        blade_xf = UsdGeom.Xform.Define(self.stage, f"{path}/BladeXform")
        blade_xf.ClearXformOpOrder()
        blade_xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, pet_len))
        # Rotate so blade's +Y (growth dir) aligns with petiole +Z
        rot_90_x = Gf.Quatf(0.7071068, 0.7071068, 0.0, 0.0)  # 90° around X
        blade_xf.AddOrientOp().Set(rot_90_x)

        mesh = self._make_leaf_mesh(f"{path}/BladeXform/Blade",
                                    leaf_width / 2.0, leaf_length)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(
            mesh.GetPrim()).GetApproximationAttr().Set("convexHull")

        # ── D6 joint: parent branch ↔ petiole ────────────────────────────
        jnt = UsdPhysics.Joint.Define(self.stage, f"{path}/Joint")
        jnt.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
        jnt.CreateBody1Rel().SetTargets([Sdf.Path(path)])

        jnt.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(local_pos0[0]), float(local_pos0[1]), float(local_pos0[2])))
        jnt.CreateLocalRot0Attr().Set(
            _quatd_to_quatf(sub_rot_local.GetQuat()))
        jnt.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        jnt.CreateLocalRot1Attr().Set(IDENTITY_QUATF)

        self._configure_drives(jnt, stiffness, damping, stiffness, damping,
                               bend_limit=45.0, lock_z=False)

        self._segments[id] = dict(
            path=path, depth=depth, global_rot=sub_rot_total,
            radius=pet_rad, height=pet_len, base_pos=world_pos,
        )
        print(f"   🍃 Leaf '{id}' petiole={pet_len*100:.1f}cm  "
              f"blade={leaf_length*100:.0f}×{leaf_width*100:.0f}cm")
        return id

    # ----------------------------------------------------------------- #
    #  TRUSS RACHIS
    # ----------------------------------------------------------------- #
    def add_truss_rachis(self, parent_id: str, base_id: str,
                         n_segments: int = 4,
                         rachis_radius: float = 0.005,
                         rachis_seg_length: float = 0.03,
                         z_offset_ratio: float = 0.8,
                         tilt_angle: float = 45.0,
                         rot_around_parent: float = 0.0,
                         mass: float = 0.05,
                         stiffness_base: float = 0.05,
                         damping_base: float = 0.01,
                         stiffness_int: float = 0.02,
                         damping_int: float = 0.005) -> list[str]:
        """Build an articulated rachis (peduncle) chain for a tomato truss.

        The first segment attaches laterally to the parent (like a branch).
        Subsequent segments extend straight tip-to-tip (like internodes),
        so the rachis starts straight and bends only under gravity and
        the weight of attached fruits.

        Parameters
        ----------
        parent_id : which segment to attach the rachis to.
        base_id : ID prefix — segments will be named base_id_01, base_id_02, …
        n_segments : how many rachis segments.
        rachis_radius / rachis_seg_length : cylinder dimensions.
        z_offset_ratio, tilt_angle, rot_around_parent : attachment geometry.
        stiffness_base / damping_base : D6 drive for the base attachment.
        stiffness_int / damping_int : D6 drive for internal joints.

        Returns
        -------
        List of segment IDs (use these to attach fruits).
        """
        if n_segments < 1:
            raise ValueError("n_segments must be >= 1")

        seg_ids: list[str] = []

        # ── First segment: lateral attachment to parent ───────────────────
        first_id = f"{base_id}_01"
        self.add_lateral_branch(
            parent_id, first_id,
            radius=rachis_radius, length=rachis_seg_length,
            z_offset_ratio=z_offset_ratio, tilt_angle=tilt_angle,
            rot_around_parent=rot_around_parent,
            mass=mass, stiffness=stiffness_base, damping=damping_base,
        )
        seg_ids.append(first_id)

        # ── Subsequent segments: straight tip-to-tip (like internodes) ────
        # They connect at the top of the previous segment without any
        # lateral offset, so the rachis starts perfectly straight.
        # Bending happens only through physics (gravity + soft joints).
        prev_id = first_id
        for i in range(2, n_segments + 1):
            seg_id = f"{base_id}_{i:02d}"
            self.add_internode(
                prev_id, seg_id,
                radius=rachis_radius, length=rachis_seg_length,
                mass=mass * 0.5, stiffness=stiffness_int, damping=damping_int,
            )
            seg_ids.append(seg_id)
            prev_id = seg_id

        print(f"   🍇 Truss rachis '{base_id}' — {n_segments} segments")
        return seg_ids

    # ----------------------------------------------------------------- #
    #  FRUIT
    # ----------------------------------------------------------------- #
    def add_fruit(self, parent_id: str, id: str,
                  fruit_radius: float = 0.015,
                  pedicel_length: float = 0.015,
                  pedicel_radius: float | None = None,
                  lateral_angle: float = 90.0,
                  mass: float | None = None,
                  is_ripe: bool = True,
                  stiffness: float = 0.001,
                  damping: float = 0.0001,
                  collisions: bool = False,
                  break_force: float | None = None) -> str:
        """Attach a tomato fruit sphere to a rachis segment via D6 joint.

        The fruit assembly = pedicel cylinder + sphere, both as children
        of a single rigid body Xform. The pedicel extends laterally from
        the parent rachis, with the sphere at its tip.

        The D6 joint between the rachis segment and the pedicel allows
        the fruit to swing up and down under gravity.

        Parameters
        ----------
        parent_id : rachis segment to attach to.
        id : unique identifier for this fruit.
        fruit_radius : radius of the fruit sphere (meters).
        pedicel_length : length of the connecting stem (meters).
        pedicel_radius : radius of pedicel cylinder. Defaults to parent radius * 0.6.
        lateral_angle : degrees around the parent axis for pedicel direction.
                        Use ±90 for alternating sides.
        mass : fruit mass (kg). Auto-computed from sphere volume and
               tomato density (~1050 kg/m³) if not specified.
        is_ripe : controls sphere color (red vs green).
        stiffness / damping : D6 drive parameters for the pedicel joint.
        """
        if parent_id not in self._segments:
            raise KeyError(f"Parent '{parent_id}' not found!")
        if id in self._segments:
            raise ValueError(f"🚫 [PlantBuilder] Segment ID '{id}' already exists!")

        p = self._segments[parent_id]
        ped_rad = pedicel_radius if pedicel_radius else max(p["radius"] * 0.6, 0.002)

        # Auto-compute mass from sphere volume (tomato density ≈ 1050 kg/m³)
        if mass is None:
            mass = (4.0 / 3.0) * math.pi * fruit_radius ** 3 * 1050.0

        # ── Compute attachment point (at tip of parent, laterally offset) ──
        rel_z = 0.8 * p["height"]  # attach near tip of rachis segment
        local_offset = Gf.Vec3d(0.0, p["radius"], rel_z)

        rot_z  = Gf.Rotation(Gf.Vec3d(0, 0, 1), lateral_angle)
        tilt_r = Gf.Rotation(Gf.Vec3d(1, 0, 0), -90.0)  # pedicel points outward

        sub_rot_local = tilt_r * rot_z
        local_pos0    = rot_z.TransformDir(local_offset)
        sub_rot_total = sub_rot_local * p["global_rot"]
        world_pos     = p["base_pos"] + p["global_rot"].TransformDir(local_pos0)

        orient_qf = _quatd_to_quatf(sub_rot_total.GetQuat())

        ped_len_half = pedicel_length / 2.0

        # ── 1. Pedicel Base Rigid Body ────────────────────────────────────
        base_path = f"{self.base_path}/{id}_base"
        base_xform = UsdGeom.Xform.Define(self.stage, base_path)
        base_xform.ClearXformOpOrder()
        base_xform.AddTranslateOp().Set(world_pos)
        base_xform.AddOrientOp().Set(orient_qf)

        UsdPhysics.RigidBodyAPI.Apply(base_xform.GetPrim())
        mass_api_base = UsdPhysics.MassAPI.Apply(base_xform.GetPrim())
        # Enforce a minimum stable mass for tiny intermediate segments (15g)
        mass_api_base.CreateMassAttr().Set(max(mass * 0.2, 0.015))

        ped_base = UsdGeom.Cylinder.Define(self.stage, f"{base_path}/PedicelBase")
        ped_base.GetRadiusAttr().Set(ped_rad)
        ped_base.GetHeightAttr().Set(ped_len_half)
        ped_base.GetAxisAttr().Set("Z")
        ped_base.AddTranslateOp().Set(Gf.Vec3d(0, 0, ped_len_half / 2.0))
        ped_base_col = UsdPhysics.CollisionAPI.Apply(ped_base.GetPrim())
        ped_base_col.CreateCollisionEnabledAttr().Set(False)

        # Joint: Rachis ↔ PedicelBase
        jnt1 = UsdPhysics.Joint.Define(self.stage, f"{base_path}/Joint")
        jnt1.CreateBody0Rel().SetTargets([Sdf.Path(p["path"])])
        jnt1.CreateBody1Rel().SetTargets([Sdf.Path(base_path)])

        jnt1.CreateLocalPos0Attr().Set(Gf.Vec3f(
            float(local_pos0[0]), float(local_pos0[1]), float(local_pos0[2])))
        jnt1.CreateLocalRot0Attr().Set(
            _quatd_to_quatf(sub_rot_local.GetQuat()))
        jnt1.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        jnt1.CreateLocalRot1Attr().Set(IDENTITY_QUATF)

        # Base attachment is a bit stiffer so it sticks out initially
        self._configure_drives(jnt1, stiffness * 5, damping * 5, stiffness * 5, damping * 5,
                               bend_limit=30.0, lock_z=False)

        # Collision filter: no self-collision between base and parent rachis
        filt1 = UsdPhysics.FilteredPairsAPI.Apply(base_xform.GetPrim())
        filt1.GetFilteredPairsRel().SetTargets([Sdf.Path(p["path"])])

        # ── 2. Pedicel Tip Rigid Body ────────────────────────────────────
        tip_world_pos = world_pos + sub_rot_total.TransformDir(Gf.Vec3d(0, 0, ped_len_half))

        tip_path = f"{self.base_path}/{id}_tip"
        tip_xform = UsdGeom.Xform.Define(self.stage, tip_path)
        tip_xform.ClearXformOpOrder()
        tip_xform.AddTranslateOp().Set(tip_world_pos)
        tip_xform.AddOrientOp().Set(orient_qf)

        UsdPhysics.RigidBodyAPI.Apply(tip_xform.GetPrim())
        mass_api_tip = UsdPhysics.MassAPI.Apply(tip_xform.GetPrim())
        mass_api_tip.CreateMassAttr().Set(max(mass * 0.1, 0.015))

        ped_tip = UsdGeom.Cylinder.Define(self.stage, f"{tip_path}/PedicelTip")
        ped_tip.GetRadiusAttr().Set(ped_rad)
        ped_tip.GetHeightAttr().Set(ped_len_half)
        ped_tip.GetAxisAttr().Set("Z")
        ped_tip.AddTranslateOp().Set(Gf.Vec3d(0, 0, ped_len_half / 2.0))
        ped_tip_col = UsdPhysics.CollisionAPI.Apply(ped_tip.GetPrim())
        ped_tip_col.CreateCollisionEnabledAttr().Set(False)

        # Joint: PedicelBase ↔ PedicelTip (elbow)
        jnt2 = UsdPhysics.Joint.Define(self.stage, f"{tip_path}/Joint")
        jnt2.CreateBody0Rel().SetTargets([Sdf.Path(base_path)])
        jnt2.CreateBody1Rel().SetTargets([Sdf.Path(tip_path)])

        jnt2.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, ped_len_half))
        jnt2.CreateLocalRot0Attr().Set(IDENTITY_QUATF)
        jnt2.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        jnt2.CreateLocalRot1Attr().Set(IDENTITY_QUATF)

        # Elbow is soft and bends easily to drop the tomato
        self._configure_drives(jnt2, stiffness, damping, stiffness, damping,
                               bend_limit=120.0, lock_z=False)

        # Collision filter: no self-collision between tip and base
        filt2 = UsdPhysics.FilteredPairsAPI.Apply(tip_xform.GetPrim())
        filt2.GetFilteredPairsRel().SetTargets([Sdf.Path(base_path)])

        # ── 3. Tomato Sphere Rigid Body ───────────────────────────────────
        path = f"{self.base_path}/{id}"
        xform = UsdGeom.Xform.Define(self.stage, path)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(tip_world_pos)
        xform.AddOrientOp().Set(orient_qf)

        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        mass_api_sph = UsdPhysics.MassAPI.Apply(xform.GetPrim())
        mass_api_sph.CreateMassAttr().Set(mass * 0.9)

        sph = UsdGeom.Sphere.Define(self.stage, f"{path}/Sphere")
        sph.GetRadiusAttr().Set(fruit_radius)
        sph.AddTranslateOp().Set(Gf.Vec3d(0, 0, ped_len_half + fruit_radius))
        sph_col = UsdPhysics.CollisionAPI.Apply(sph.GetPrim())
        sph_col.CreateCollisionEnabledAttr().Set(collisions)

        color = Gf.Vec3f(FRUIT_RIPE) if is_ripe else Gf.Vec3f(FRUIT_YOUNG)
        sph.GetDisplayColorAttr().Set([color])

        # Fixed Joint: PedicelTip ↔ TomatoSphere (with break force)
        jnt3 = UsdPhysics.FixedJoint.Define(self.stage, f"{path}/FixedJoint")
        jnt3.CreateBody0Rel().SetTargets([Sdf.Path(tip_path)])
        jnt3.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        jnt3.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, ped_len_half))
        jnt3.CreateLocalRot0Attr().Set(IDENTITY_QUATF)
        jnt3.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, ped_len_half))
        jnt3.CreateLocalRot1Attr().Set(IDENTITY_QUATF)

        # IMPORTANT: Exclude from articulation so it can be dynamically broken!
        jnt3.CreateExcludeFromArticulationAttr().Set(True)

        if break_force is not None:
            jnt3.CreateBreakForceAttr().Set(break_force)
            jnt3.CreateBreakTorqueAttr().Set(break_force * 0.1)

        filt3 = UsdPhysics.FilteredPairsAPI.Apply(xform.GetPrim())
        filt3.GetFilteredPairsRel().SetTargets([Sdf.Path(tip_path)])

        depth = p.get("depth", 0) + 1
        self._segments[id] = dict(
            path=path, depth=depth, global_rot=sub_rot_total,
            radius=fruit_radius, height=pedicel_length + fruit_radius * 2,
            base_pos=world_pos,
        )

        state = "🔴 ripe" if is_ripe else "🟢 unripe"
        print(f"   🍅 Fruit '{id}' r={fruit_radius*100:.1f}cm "
              f"m={mass*1000:.0f}g {state}")
        return id
