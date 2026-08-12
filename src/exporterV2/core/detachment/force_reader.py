"""Joint reaction wrench readers."""

from abc import ABC, abstractmethod
import numpy as np

from .state import JointWrench


class JointWrenchReader(ABC):
    @abstractmethod
    def initialize(self, fruits) -> None:
        pass

    @abstractmethod
    def read(self, joint_path: str) -> JointWrench:
        pass

    def read_many(self, joint_paths: list[str]) -> dict[str, JointWrench]:
        return {path: self.read(path) for path in joint_paths}

    def refresh(self, fruits) -> None:
        self._articulation.initialize()
        self.initialize(fruits)


class IsaacArticulationWrenchReader(JointWrenchReader):
    """Read incoming-joint wrenches in the PhysX articulation link frame."""

    def __init__(self, articulation):
        self._articulation = articulation
        self._row_by_path = {}

    def bind_articulation(self, articulation) -> None:
        self._articulation = articulation

    def initialize(self, fruits) -> None:
        self._row_by_path = {}
        for fruit in fruits:
            body_name = fruit.attachment_body_path.rsplit("/", 1)[-1]
            if body_name not in self._articulation.body_names:
                raise RuntimeError(
                    f"Attached body '{body_name}' was not found in articulation body_names"
                )
            # Measured wrench row N is the incoming joint of articulation body N.
            self._row_by_path[fruit.attachment_body_path] = self._articulation.get_body_index(
                body_name
            )

    def read(self, joint_path: str) -> JointWrench:
        measured = np.asarray(self._articulation.get_measured_joint_forces(), dtype=float)
        return self._read_from_array(joint_path, measured)

    def read_many(self, joint_paths: list[str]) -> dict[str, JointWrench]:
        # One tensor read per physics step avoids one GPU synchronization and
        # full articulation copy for every tomato.
        if not joint_paths:
            return {}
        measured = np.asarray(self._articulation.get_measured_joint_forces(), dtype=float)
        return {
            path: self._read_from_array(path, measured)
            for path in joint_paths
        }

    def _read_from_array(self, joint_path: str, measured: np.ndarray) -> JointWrench:
        row = self._row_by_path[joint_path]
        wrench = measured[0, row] if measured.ndim == 3 else measured[row]
        return JointWrench(force=wrench[:3].copy(), torque=wrench[3:].copy())
