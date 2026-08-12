"""Instantaneous detachment criteria."""

from abc import ABC, abstractmethod

import numpy as np

from .state import FruitRuntimeData


class DetachmentModel(ABC):
    @abstractmethod
    def evaluate(
        self,
        fruit: FruitRuntimeData,
        force: np.ndarray,
        torque: np.ndarray,
        dt: float,
    ) -> float:
        """Return normalized instantaneous damage."""


class ForceThresholdModel(DetachmentModel):
    def evaluate(self, fruit, force, torque, dt):
        del torque, dt
        return float(np.linalg.norm(force) / fruit.force_threshold)


class CombinedForceTorqueModel(DetachmentModel):
    def evaluate(self, fruit, force, torque, dt):
        del dt
        force_term = (np.linalg.norm(force) / fruit.force_threshold) ** fruit.force_exponent
        torque_term = (np.linalg.norm(torque) / fruit.torque_threshold) ** fruit.torque_exponent
        return float(force_term + torque_term)


def create_detachment_model(name: str) -> DetachmentModel:
    if name == "force":
        return ForceThresholdModel()
    if name == "force_torque":
        return CombinedForceTorqueModel()
    raise ValueError(f"Unsupported tomato detachment model: {name!r}")
