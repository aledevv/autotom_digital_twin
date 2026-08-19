"""Runtime synchronization for persisted vegetative SkelRoots."""

from dataclasses import dataclass
from time import perf_counter
from typing import List

from pxr import Gf, Usd, UsdGeom, UsdSkel, Vt

from .schema import (
    ANIMATION_REL,
    BRANCH_ID_ATTR,
    PHYSICS_LINKS_REL,
    VISUAL_AXIS_ID_ATTR,
)


@dataclass
class _RuntimeBranch:
    name: str
    link_prims: list
    skel_root_prim: object
    translations_attr: object
    rotations_attr: object
    mesh_vertex_count: int = 0


def _quatf_from_matrix(matrix: Gf.Matrix4d) -> Gf.Quatf:
    quat = matrix.ExtractRotationQuat()
    imag = quat.GetImaginary()
    return Gf.Quatf(float(quat.GetReal()), Gf.Vec3f(*imag))


class SkinningRuntime:
    """Synchronize UsdSkel animation attributes from PhysX rigid links."""

    def __init__(self, stage: Usd.Stage, branches: List[_RuntimeBranch]):
        self.stage = stage
        self.branches = branches
        self._cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    @classmethod
    def discover(cls, stage: Usd.Stage) -> "SkinningRuntime":
        """Reconstruct and validate runtime state from authored relations."""
        branches = []
        for prim in stage.Traverse():
            if not prim.IsA(UsdSkel.Root):
                continue
            links_rel = prim.GetRelationship(PHYSICS_LINKS_REL)
            animation_rel = prim.GetRelationship(ANIMATION_REL)
            if not links_rel or not animation_rel:
                continue
            link_targets = links_rel.GetTargets()
            animation_targets = animation_rel.GetTargets()
            name_attr = prim.GetAttribute(VISUAL_AXIS_ID_ATTR)
            if not name_attr or not name_attr.HasAuthoredValue():
                name_attr = prim.GetAttribute(BRANCH_ID_ATTR)
            name = name_attr.Get() if name_attr else prim.GetPath().pathString
            if not link_targets:
                raise ValueError(f"SkelRoot '{name}' has no physics link targets")
            if len(animation_targets) != 1:
                raise ValueError(f"SkelRoot '{name}' must target exactly one SkelAnimation")

            link_prims = [stage.GetPrimAtPath(path) for path in link_targets]
            if any(not link.IsValid() for link in link_prims):
                raise ValueError(f"SkelRoot '{name}' references a missing physics link")
            animation = UsdSkel.Animation(stage.GetPrimAtPath(animation_targets[0]))
            if not animation or not animation.GetPrim().IsValid():
                raise ValueError(f"SkelRoot '{name}' references an invalid SkelAnimation")
            joints = animation.GetJointsAttr().Get() or []
            if len(joints) != len(link_prims):
                raise ValueError(
                    f"SkelRoot '{name}' has {len(joints)} bones but {len(link_prims)} links"
                )
            translations_attr = animation.GetTranslationsAttr()
            rotations_attr = animation.GetRotationsAttr()
            if not translations_attr or not rotations_attr:
                raise ValueError(f"SkelRoot '{name}' is missing animation transforms")
            translations = translations_attr.Get() or []
            rotations = rotations_attr.Get() or []
            if len(translations) != len(link_prims) or len(rotations) != len(link_prims):
                raise ValueError(
                    f"SkelRoot '{name}' animation transform counts do not match its links"
                )

            skeletons = [
                UsdSkel.Skeleton(descendant)
                for descendant in Usd.PrimRange(prim)
                if descendant.IsA(UsdSkel.Skeleton)
            ]
            if len(skeletons) != 1:
                raise ValueError(f"SkelRoot '{name}' must contain exactly one Skeleton")
            skeleton_joints = skeletons[0].GetJointsAttr().Get() or []
            if len(skeleton_joints) != len(link_prims):
                raise ValueError(
                    f"SkelRoot '{name}' Skeleton bone count does not match its links"
                )

            mesh_vertex_count = 0
            for descendant in Usd.PrimRange(prim):
                if not descendant.IsA(UsdGeom.Mesh):
                    continue
                points = UsdGeom.Mesh(descendant).GetPointsAttr().Get() or []
                mesh_vertex_count += len(points)

            branches.append(_RuntimeBranch(
                name=name,
                link_prims=link_prims,
                skel_root_prim=prim,
                translations_attr=translations_attr,
                rotations_attr=rotations_attr,
                mesh_vertex_count=mesh_vertex_count,
            ))
        return cls(stage, branches)

    @property
    def branch_count(self) -> int:
        return len(self.branches)

    @property
    def bone_count(self) -> int:
        return sum(len(branch.link_prims) for branch in self.branches)

    @property
    def single_bone_branch_count(self) -> int:
        return sum(1 for branch in self.branches if len(branch.link_prims) == 1)

    @property
    def multi_bone_branch_count(self) -> int:
        return self.branch_count - self.single_bone_branch_count

    @property
    def mesh_vertex_count(self) -> int:
        return sum(branch.mesh_vertex_count for branch in self.branches)

    @property
    def animation_attribute_writes_per_sync(self) -> int:
        # One translations + one rotations write per visual axis.
        return self.branch_count * 2

    def stats(self) -> dict:
        """Return compact structural stats useful for performance diagnostics."""
        return {
            "visual_axes": self.branch_count,
            "bones": self.bone_count,
            "single_bone_axes": self.single_bone_branch_count,
            "multi_bone_axes": self.multi_bone_branch_count,
            "mesh_vertices": self.mesh_vertex_count,
            "usd_attr_writes_per_sync": self.animation_attribute_writes_per_sync,
        }

    def _sync_impl(self, *, profiled: bool) -> dict | None:
        """Internal sync implementation; optionally return a detailed timing split."""
        if profiled:
            profile = {
                "cache_clear_s": 0.0,
                "xform_reads_s": 0.0,
                "local_matrices_s": 0.0,
                "decompose_s": 0.0,
                "usd_writes_s": 0.0,
                "total_s": 0.0,
            }
            total_start = perf_counter()
            start = perf_counter()
            self._cache.Clear()
            profile["cache_clear_s"] += perf_counter() - start
        else:
            profile = None
            self._cache.Clear()

        for runtime in self.branches:
            if profiled:
                start = perf_counter()
            world = [
                Gf.Matrix4d(self._cache.GetLocalToWorldTransform(prim))
                for prim in runtime.link_prims
            ]
            root_world = Gf.Matrix4d(
                self._cache.GetLocalToWorldTransform(runtime.skel_root_prim)
            )
            if profiled:
                profile["xform_reads_s"] += perf_counter() - start
                start = perf_counter()

            root_inverse = root_world.GetInverse()
            local = [world[0] * root_inverse]
            local.extend(
                world[index] * world[index - 1].GetInverse()
                for index in range(1, len(world))
            )
            if profiled:
                profile["local_matrices_s"] += perf_counter() - start
                start = perf_counter()

            translations = []
            rotations = []
            for matrix in local:
                translation = matrix.ExtractTranslation()
                translations.append(Gf.Vec3f(*translation))
                rotations.append(_quatf_from_matrix(matrix))
            if profiled:
                profile["decompose_s"] += perf_counter() - start
                start = perf_counter()

            runtime.translations_attr.Set(Vt.Vec3fArray(translations))
            runtime.rotations_attr.Set(Vt.QuatfArray(rotations))
            if profiled:
                profile["usd_writes_s"] += perf_counter() - start

        if profiled:
            profile["total_s"] = perf_counter() - total_start
            return profile
        return None

    def sync(self) -> None:
        """Write current rigid-link transforms into every SkelAnimation."""
        self._sync_impl(profiled=False)

    def sync_profiled(self) -> dict:
        """Run one sync and return detailed timing information in seconds."""
        return self._sync_impl(profiled=True)


def configure_physx_mouse_interaction(simulation_app) -> None:
    """Enable grabbing of invisible capsule proxies after stage open/reset."""
    import carb.settings
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("omni.physx.ui", True)
    manager.set_extension_enabled_immediate("omni.physx.supportui", True)
    simulation_app.update()
    settings = carb.settings.get_settings()
    settings.set("/physics/mouseInteractionEnabled", True)
    settings.set("/physics/mouseGrab", True)
    settings.set("/physics/mouseGrabIgnoreInvisible", False)
    settings.set("/physics/forceGrab", False)
    settings.set("/physics/pickingForce", 10.0)
