"""Live compound-body tomato representation swap."""

from abc import ABC, abstractmethod

import numpy as np


class DetachmentBackend(ABC):
    @abstractmethod
    def initialize(self, fruits) -> None:
        pass

    @abstractmethod
    def detach_batch(self, fruits) -> None:
        pass

    @abstractmethod
    def reset(self, fruits) -> None:
        pass


def _parallel_axis(point: np.ndarray, mass: float) -> np.ndarray:
    point = np.asarray(point, dtype=float)
    return mass * ((point @ point) * np.eye(3) - np.outer(point, point))


class IsaacSwapBackend(DetachmentBackend):
    """Detach fruit without changing articulation topology or rebuilding World."""

    def __init__(self, stage, articulation, world, root_prim_path):
        del world, root_prim_path
        self._stage = stage
        self._articulation = articulation
        self.last_snapshot_by_fruit = {}
        self._initial_mass_state = None

    @property
    def articulation(self):
        return self._articulation

    def initialize(self, fruits) -> None:
        del fruits
        positions, orientations = self._articulation.get_body_coms()
        self._initial_mass_state = (
            np.asarray(self._articulation.get_body_masses(), dtype=float).copy(),
            np.asarray(positions, dtype=float).copy(),
            np.asarray(orientations, dtype=float).copy(),
            np.asarray(self._articulation.get_body_inertias(), dtype=float).copy(),
        )

    @staticmethod
    def _set_collision(stage, prim_path: str, enabled: bool) -> None:
        stage.GetPrimAtPath(f"{prim_path}/Sphere").GetAttribute(
            "physics:collisionEnabled"
        ).Set(enabled)

    @staticmethod
    def _set_visible(stage, prim_path: str, visible: bool) -> None:
        from pxr import UsdGeom

        imageable = UsdGeom.Imageable(stage.GetPrimAtPath(prim_path))
        imageable.MakeVisible() if visible else imageable.MakeInvisible()

    def detach_batch(self, fruits) -> None:
        from isaacsim.core.prims import RigidPrim
        from isaacsim.core.utils.rotations import quat_to_rot_matrix

        masses = np.asarray(self._articulation.get_body_masses(), dtype=float)
        com_positions, com_orientations = self._articulation.get_body_coms()
        com_positions = np.asarray(com_positions, dtype=float)
        com_orientations = np.asarray(com_orientations, dtype=float)
        inertias = np.asarray(self._articulation.get_body_inertias(), dtype=float)
        snapshots = {}

        for fruit in fruits:
            body_name = fruit.attachment_body_path.rsplit("/", 1)[-1]
            body_index = self._articulation.get_body_index(body_name)
            parent = RigidPrim(fruit.attachment_body_path)
            parent.initialize()
            parent_positions, parent_orientations = parent.get_world_poses()
            parent_velocities = parent.get_velocities()

            rotation = quat_to_rot_matrix(parent_orientations[0])
            world_offset = rotation @ fruit.local_center
            fruit_position = parent_positions[0] + world_offset
            fruit_linear_velocity = (
                parent_velocities[0, :3]
                + np.cross(parent_velocities[0, 3:], world_offset)
            )
            fruit_velocity = np.concatenate(
                [fruit_linear_velocity, parent_velocities[0, 3:]]
            )
            snapshots[fruit.fruit_id] = {
                "positions": fruit_position.reshape(1, 3),
                "orientations": parent_orientations.copy(),
                "velocities": fruit_velocity.reshape(1, 6),
            }

            old_mass = float(masses[0, body_index])
            new_mass = old_mass - fruit.fruit_mass
            if new_mass <= 0.0:
                raise RuntimeError(f"fruit '{fruit.fruit_id}' would leave non-positive body mass")
            old_com = com_positions[0, body_index].copy()
            new_com = (
                old_mass * old_com - fruit.fruit_mass * fruit.local_center
            ) / new_mass
            old_inertia = inertias[0, body_index].reshape(3, 3)
            sphere_inertia = (
                0.4 * fruit.fruit_mass * fruit.fruit_radius**2 * np.eye(3)
            )
            inertia_origin = old_inertia + _parallel_axis(old_com, old_mass)
            new_inertia = (
                inertia_origin
                - sphere_inertia
                - _parallel_axis(fruit.local_center, fruit.fruit_mass)
                - _parallel_axis(new_com, new_mass)
            )
            masses[0, body_index] = new_mass
            com_positions[0, body_index] = new_com
            inertias[0, body_index] = new_inertia.reshape(9)

            self._set_collision(self._stage, fruit.attached_prim_path, False)
            self._set_visible(self._stage, fruit.attached_prim_path, False)

        self._articulation.set_body_masses(masses)
        self._articulation.set_body_coms(com_positions, com_orientations)
        self._articulation.set_body_inertias(inertias)

        for fruit in fruits:
            snapshot = snapshots[fruit.fruit_id]
            detached = RigidPrim(fruit.detached_prim_path)
            detached.initialize()
            detached.set_world_poses(snapshot["positions"], snapshot["orientations"])
            detached_prim = self._stage.GetPrimAtPath(fruit.detached_prim_path)
            detached_prim.GetAttribute("physics:kinematicEnabled").Set(False)
            self._set_collision(self._stage, fruit.detached_prim_path, True)
            self._set_visible(self._stage, fruit.detached_prim_path, True)
            detached.set_velocities(snapshot["velocities"])

        self.last_snapshot_by_fruit = snapshots

    def reset(self, fruits) -> None:
        from isaacsim.core.prims import RigidPrim

        masses, com_positions, com_orientations, inertias = self._initial_mass_state
        self._articulation.set_body_masses(masses.copy())
        self._articulation.set_body_coms(
            com_positions.copy(), com_orientations.copy()
        )
        self._articulation.set_body_inertias(inertias.copy())
        for fruit in fruits:
            detached_prim = self._stage.GetPrimAtPath(fruit.detached_prim_path)
            detached_prim.GetAttribute("physics:kinematicEnabled").Set(True)
            self._set_collision(self._stage, fruit.detached_prim_path, False)
            self._set_visible(self._stage, fruit.detached_prim_path, False)
            detached = RigidPrim(fruit.detached_prim_path)
            detached.initialize()
            detached.set_velocities(np.zeros((1, 6), dtype=np.float32))
            self._set_collision(self._stage, fruit.attached_prim_path, True)
            self._set_visible(self._stage, fruit.attached_prim_path, True)
