"""Discovery and validation of detachable fruit metadata in a USD stage."""

import numpy as np
from pxr import Usd, UsdPhysics

from .state import FruitRuntimeData


class TomatoPlantConfigurationError(ValueError):
    pass


def _required_value(prim, name: str):
    attr = prim.GetAttribute(name)
    value = attr.Get() if attr else None
    if value is None:
        raise TomatoPlantConfigurationError(f"prim '{prim.GetPath()}': missing '{name}'")
    return value


def _single_target(prim, name: str) -> str:
    rel = prim.GetRelationship(name)
    targets = rel.GetTargets() if rel else []
    if len(targets) != 1:
        raise TomatoPlantConfigurationError(
            f"prim '{prim.GetPath()}': relationship '{name}' must have one target"
        )
    return str(targets[0])


class PlantMetadataParser:
    SUPPORTED_SCHEMA_VERSIONS = {"1.0"}

    def parse(self, stage, root_prim_path: str) -> list[FruitRuntimeData]:
        root = stage.GetPrimAtPath(root_prim_path)
        if not root:
            raise TomatoPlantConfigurationError(f"plant root '{root_prim_path}' does not exist")

        version = _required_value(root, "tomatoPlant:schemaVersion")
        if version not in self.SUPPORTED_SCHEMA_VERSIONS:
            raise TomatoPlantConfigurationError(f"unsupported tomatoPlant schema version {version!r}")
        if not bool(_required_value(root, "tomatoPlant:enabled")):
            return []

        fruits = []
        ids = set()
        for prim in Usd.PrimRange(root):
            if not bool(prim.GetAttribute("tomatoPlant:detachable").Get()):
                continue
            fruit = self._parse_fruit(stage, prim, root_prim_path)
            if fruit.fruit_id in ids:
                raise TomatoPlantConfigurationError(f"duplicate fruit id '{fruit.fruit_id}'")
            ids.add(fruit.fruit_id)
            fruits.append(fruit)
        return fruits

    def _parse_fruit(self, stage, prim, root_prim_path: str) -> FruitRuntimeData:
        fruit_id = str(_required_value(prim, "tomatoPlant:id"))
        attachment_body_path = _single_target(prim, "tomatoPlant:attachmentBody")
        detached_path = _single_target(prim, "tomatoPlant:detachedBody")
        attached_path = str(prim.GetPath())

        attachment_body_prim = stage.GetPrimAtPath(attachment_body_path)
        detached_prim = stage.GetPrimAtPath(detached_path)
        if not attachment_body_prim or not attachment_body_prim.HasAPI(UsdPhysics.RigidBodyAPI):
            raise TomatoPlantConfigurationError(
                f"fruit '{fruit_id}': attachment body '{attachment_body_path}' is not rigid"
            )
        if not detached_prim:
            raise TomatoPlantConfigurationError(
                f"fruit '{fruit_id}': detached body '{detached_path}' does not exist"
            )
        if attached_path == detached_path:
            raise TomatoPlantConfigurationError(f"fruit '{fruit_id}': attached and detached paths are equal")
        if detached_path.startswith(f"{root_prim_path}/"):
            raise TomatoPlantConfigurationError(
                f"fruit '{fruit_id}': detached body must be outside the articulation root"
            )
        detached_rigid_body = UsdPhysics.RigidBodyAPI(detached_prim)
        if detached_rigid_body.GetRigidBodyEnabledAttr().Get() is not True:
            raise TomatoPlantConfigurationError(
                f"fruit '{fruit_id}': detached proxy must be a live rigid body"
            )
        if detached_rigid_body.GetKinematicEnabledAttr().Get() is not True:
            raise TomatoPlantConfigurationError(
                f"fruit '{fruit_id}': detached proxy must initially be kinematic"
            )

        model = str(_required_value(prim, "tomatoPlant:detachmentModel"))
        force_threshold = float(_required_value(prim, "tomatoPlant:forceThreshold"))
        torque_threshold = float(_required_value(prim, "tomatoPlant:torqueThreshold"))
        force_exponent = float(_required_value(prim, "tomatoPlant:forceExponent"))
        torque_exponent = float(_required_value(prim, "tomatoPlant:torqueExponent"))
        minimum_duration = float(_required_value(prim, "tomatoPlant:minimumBreakDuration"))
        fruit_mass = float(_required_value(prim, "tomatoPlant:mass"))
        fruit_radius = float(_required_value(prim, "tomatoPlant:radius"))
        local_center = _required_value(prim, "tomatoPlant:localCenter")
        if model not in {"force", "force_torque"}:
            raise TomatoPlantConfigurationError(f"fruit '{fruit_id}': invalid model '{model}'")
        positive = {
            "forceThreshold": force_threshold,
            "forceExponent": force_exponent,
            "minimumBreakDuration": minimum_duration,
            "mass": fruit_mass,
            "radius": fruit_radius,
        }
        if model == "force_torque":
            positive.update(torqueThreshold=torque_threshold, torqueExponent=torque_exponent)
        for name, value in positive.items():
            if value <= 0.0:
                raise TomatoPlantConfigurationError(f"fruit '{fruit_id}': {name} must be positive")

        return FruitRuntimeData(
            fruit_id=fruit_id,
            attached_prim_path=attached_path,
            detached_prim_path=detached_path,
            attachment_body_path=attachment_body_path,
            fruit_mass=fruit_mass,
            fruit_radius=fruit_radius,
            local_center=np.asarray(local_center, dtype=float),
            model=model,
            force_threshold=force_threshold,
            torque_threshold=torque_threshold,
            force_exponent=force_exponent,
            torque_exponent=torque_exponent,
            minimum_break_duration=minimum_duration,
        )
