"""Data objects shared by the detachment runtime and future RL adapters."""

from dataclasses import dataclass, field
from enum import Enum

import numpy as np


class FruitState(Enum):
    ATTACHED = 0
    DETACH_PENDING = 1
    DETACHED = 2


@dataclass(frozen=True)
class JointWrench:
    force: np.ndarray
    torque: np.ndarray


@dataclass
class FruitRuntimeData:
    fruit_id: str
    attached_prim_path: str
    detached_prim_path: str
    attachment_body_path: str
    fruit_mass: float
    fruit_radius: float
    local_center: np.ndarray
    model: str
    force_threshold: float
    torque_threshold: float
    force_exponent: float
    torque_exponent: float
    minimum_break_duration: float
    state: FruitState = FruitState.ATTACHED
    last_force: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    last_torque: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    damage: float = 0.0
    overload_time: float = 0.0


@dataclass(frozen=True)
class DetachmentEvent:
    fruit_id: str
    force: float
    torque: float
    damage: float
    attached_prim_path: str
    detached_prim_path: str
