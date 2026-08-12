"""Generic per-physics-step tomato detachment orchestrator."""

import numpy as np

from .backend import IsaacSwapBackend
from .force_reader import IsaacArticulationWrenchReader
from .metadata import PlantMetadataParser
from .models import create_detachment_model
from .state import DetachmentEvent, FruitState


class TomatoPlantRuntime:
    def __init__(
        self,
        root_prim_path: str = "/World/Stem",
        *,
        stage=None,
        articulation=None,
        world=None,
        wrench_reader=None,
        backend=None,
        metadata_parser=None,
        debug: bool | None = None,
        sensor_hz: float | None = None,
    ):
        self.root_prim_path = root_prim_path
        self.stage = stage
        self.articulation = articulation
        self.world = world
        self._reader = wrench_reader
        self._backend = backend
        self._parser = metadata_parser or PlantMetadataParser()
        self.fruits = []
        self._models = {}
        self.debug = debug
        self._debug_elapsed = 0.0
        self._debug_interval = 0.5
        self.sensor_hz = sensor_hz
        self._sensor_elapsed = 0.0

    def initialize(self):
        if self.stage is None:
            import omni.usd

            self.stage = omni.usd.get_context().get_stage()
        print(f"[TomatoPlant] discovering metadata under {self.root_prim_path}", flush=True)
        self.fruits = self._parser.parse(self.stage, self.root_prim_path)
        if self.debug is None:
            self.debug = False
            get_prim = getattr(self.stage, "GetPrimAtPath", None)
            if get_prim is not None:
                debug_attr = get_prim(self.root_prim_path).GetAttribute(
                    "tomatoPlant:debug"
                )
                self.debug = bool(debug_attr.Get()) if debug_attr else False
        print(f"[TomatoPlant] discovered {len(self.fruits)} detachable fruits", flush=True)
        if self._reader is None:
            self._reader = IsaacArticulationWrenchReader(self.articulation)
        if self._backend is None:
            if self.world is None:
                raise ValueError("world is required when using the Isaac swap backend")
            self._backend = IsaacSwapBackend(
                self.stage,
                self.articulation,
                self.world,
                self.root_prim_path,
            )
        self._models = {fruit.fruit_id: create_detachment_model(fruit.model) for fruit in self.fruits}
        print("[TomatoPlant] mapping articulation joint wrenches", flush=True)
        self._reader.initialize(self.fruits)
        print("[TomatoPlant] wrench mapping ready", flush=True)
        self._backend.initialize(self.fruits)
        print(f"[TomatoPlant] initialized {self.root_prim_path}: {len(self.fruits)} fruits")
        return self

    def step(self, dt: float) -> list[DetachmentEvent]:
        if dt <= 0.0:
            raise ValueError(f"dt must be positive, got {dt}")
        self._sensor_elapsed += dt
        if self.sensor_hz is not None:
            if self.sensor_hz <= 0.0:
                raise ValueError(f"sensor_hz must be positive, got {self.sensor_hz}")
            sensor_interval = 1.0 / self.sensor_hz
            if self._sensor_elapsed + 1e-12 < sensor_interval:
                return []
            sample_dt = self._sensor_elapsed
            self._sensor_elapsed %= sensor_interval
        else:
            sample_dt = dt

        attached_fruits = [
            fruit for fruit in self.fruits if fruit.state is FruitState.ATTACHED
        ]
        joint_paths = [fruit.attachment_body_path for fruit in attached_fruits]
        read_many = getattr(self._reader, "read_many", None)
        if read_many is None:
            wrenches = {path: self._reader.read(path) for path in joint_paths}
        else:
            wrenches = read_many(joint_paths)
        pending = []
        for fruit in attached_fruits:
            wrench = wrenches[fruit.attachment_body_path]
            fruit.last_force = np.asarray(wrench.force, dtype=float)
            fruit.last_torque = np.asarray(wrench.torque, dtype=float)
            fruit.damage = self._models[fruit.fruit_id].evaluate(
                fruit, fruit.last_force, fruit.last_torque, sample_dt
            )
            fruit.overload_time = (
                fruit.overload_time + sample_dt if fruit.damage >= 1.0 else 0.0
            )
            if fruit.overload_time + 1e-12 >= fruit.minimum_break_duration:
                fruit.state = FruitState.DETACH_PENDING
                pending.append(fruit)

        self._debug_elapsed += sample_dt
        if self.debug and attached_fruits and self._debug_elapsed >= self._debug_interval:
            self._debug_elapsed = 0.0
            hottest = max(attached_fruits, key=lambda fruit: fruit.damage)
            print(
                f"[TomatoPlant] LOAD {hottest.fruit_id} "
                f"F={np.linalg.norm(hottest.last_force):.3f}N "
                f"M={np.linalg.norm(hottest.last_torque):.4f}Nm "
                f"D={hottest.damage:.3f} overload={hottest.overload_time:.4f}s",
                flush=True,
            )

        if not pending:
            return []

        self._backend.detach_batch(pending)
        events = []
        for fruit in pending:
            fruit.state = FruitState.DETACHED
            event = DetachmentEvent(
                fruit_id=fruit.fruit_id,
                force=float(np.linalg.norm(fruit.last_force)),
                torque=float(np.linalg.norm(fruit.last_torque)),
                damage=fruit.damage,
                attached_prim_path=fruit.attached_prim_path,
                detached_prim_path=fruit.detached_prim_path,
            )
            events.append(event)
            print(
                f"[TomatoPlant] DETACH {event.fruit_id} "
                f"F={event.force:.4f}N M={event.torque:.4f}Nm D={event.damage:.3f}"
            )
        return events

    def reset(self) -> None:
        self._backend.reset(self.fruits)
        for fruit in self.fruits:
            fruit.state = FruitState.ATTACHED
            fruit.last_force = np.zeros(3, dtype=float)
            fruit.last_torque = np.zeros(3, dtype=float)
            fruit.damage = 0.0
            fruit.overload_time = 0.0
        self._reader.refresh(self.fruits)
        self._debug_elapsed = 0.0
        self._sensor_elapsed = 0.0

    def get_fruits(self):
        return tuple(self.fruits)

    def get_detached_fruits(self):
        return tuple(fruit for fruit in self.fruits if fruit.state is FruitState.DETACHED)

    def get_debug_state(self):
        return [
            {
                "id": fruit.fruit_id,
                "state": fruit.state.name,
                "force": fruit.last_force.tolist(),
                "torque": fruit.last_torque.tolist(),
                "force_norm": float(np.linalg.norm(fruit.last_force)),
                "torque_norm": float(np.linalg.norm(fruit.last_torque)),
                "damage": fruit.damage,
                "overload_time": fruit.overload_time,
            }
            for fruit in self.fruits
        ]

    def shutdown(self) -> None:
        self.fruits.clear()
        self._models.clear()
