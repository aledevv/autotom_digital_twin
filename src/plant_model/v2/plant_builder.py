import os
import sys
import argparse

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
VERSION_DIR  = os.path.dirname(SCRIPT_DIR)      
SRC_DIR      = os.path.dirname(VERSION_DIR)
PROJECT_ROOT = os.path.dirname(SRC_DIR)

if SRC_DIR not in sys.path:
    sys.path.insert(0, SRC_DIR)

from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
from plant_model.v2.config import PLANT_ROOT_PATH, GLOBAL_SCALE


IDENTITY_QUATF = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

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
    def add_main_stem(
        self,
        id: str,
        total_length: float,
        radius: float,
        color: tuple = (0.35, 0.62, 0.20),
        mass: float = 1.0,
        physics: bool = False,
        max_segments: int = 1,
    ) -> str:
        """
        Create the main stem as a SINGLE rigid stick (no internal joints).
        physics=False -> pure static visual geometry (fastest for iteration).
        physics=True  -> rigid body anchored to world (still no bending).
        """
        if id in self._segments:
            raise ValueError(f"Segment id '{id}' already exists")

        # FOR NOW STEM IS A STICK, NOT ARTICULATED (max_segments > 1 not implemented yet)
        if max_segments > 1:
            print(f"⚠️ [PlantBuilder] Max segments {max_segments} > 1, but stem is created as a single rigid stick. Value of max_segments has been set to 1.")
            max_segments = 1  # force

        path = f"{self.base_path}/{id}"
        xform = UsdGeom.Xform.Define(self.stage, path)
        xform.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
        xform.AddOrientOp().Set(IDENTITY_QUATF)

        self._make_visual(path, radius, total_length, color)

        if physics:
            scaled_mass = mass * (self.scale ** 3)   # Scale mass by volume
            self._apply_physics(xform.GetPrim(), scaled_mass, anchor_to_world=True)

        self._segments[id] = dict(
            path=path,
            length=total_length,
            radius=radius,
            physics_enabled=physics,
        )
        return id