# Tomato Plant Detachment Runtime

## 1. Objective

Implement a reusable tomato-plant simulation architecture for **NVIDIA Isaac Sim 4.5** based on:

```text
USD asset
    +
USD metadata
    +
generic Python runtime
```

The USD file must contain the physical plant model and all plant-specific detachment parameters.

The Python runtime must contain only generic behavior and must not hard-code:

* tomato prim paths;
* peduncle joint paths;
* force thresholds;
* torque thresholds;
* number of tomatoes;
* plant topology.

The runtime must discover these from USD metadata.

The primary design objective is:

> Preserve the tomato as part of the reduced-coordinate plant articulation while it is attached, and only change its physical representation when a detachment event occurs.

Do **not** use a `FixedJoint` with `Exclude From Articulation` as the default attached representation.

---

# 2. Main architectural principle

The simulation consists of three layers:

```text
┌──────────────────────────────┐
│ USD ASSET                    │
│                              │
│ Geometry                     │
│ Colliders                    │
│ Mass/inertia                 │
│ D6 joints                    │
│ Articulation                 │
│ Tomato links                 │
│ Detachment metadata          │
└──────────────┬───────────────┘
               │ discovered by
               ▼
┌──────────────────────────────┐
│ GENERIC PLANT RUNTIME        │
│                              │
│ metadata parser              │
│ force/torque reader          │
│ detachment state machine     │
│ detachment backend           │
└──────────────┬───────────────┘
               │ exposes state
               ▼
┌──────────────────────────────┐
│ ISAAC SIM / ISAAC LAB TASK   │
│                              │
│ robot                        │
│ observations                 │
│ rewards                      │
│ RL task                      │
└──────────────────────────────┘
```

The robot task must not contain tomato detachment implementation details.

The desired external API is approximately:

```python
plant = TomatoPlantRuntime(
    root_prim_path="/World/Plant"
)

plant.initialize()

# every physics step
events = plant.step(dt)
```

The runtime should return events such as:

```python
[
    DetachmentEvent(
        fruit_id="tomato_03",
        force=2.73,
        torque=0.08,
        damage=1.06,
    )
]
```

---

# 3. USD organization

Use a predictable hierarchy.

Example:

```text
/Plant
│
├── Physics
│   │
│   ├── StemRoot
│   ├── Branch_01
│   ├── Branch_02
│   ├── Peduncle_01
│   ├── TomatoAttached_01
│   │
│   ├── Peduncle_02
│   └── TomatoAttached_02
│
├── DetachedBodies
│   │
│   ├── TomatoDetached_01
│   └── TomatoDetached_02
│
└── Visuals
```

The exact hierarchy is not mandatory.

The runtime must rely primarily on metadata rather than naming conventions.

---

# 4. Attached tomato representation

Each attached tomato must be a normal rigid link belonging to the plant articulation.

Example:

```text
Branch
   │
   D6
   │
Peduncle
   │
   Fixed/D6 articulation joint
   │
TomatoAttached
```

The tomato mass and inertia must therefore contribute to the dynamics of the plant before detachment.

The final joint must **not** use:

```text
Exclude From Articulation = true
```

for the default implementation.

Do not use PhysX native `breakForce` as the primary detachment criterion.

The runtime determines detachment explicitly from measured forces/torques.

Isaac Sim 4.5 provides articulation-state outputs including measured joint efforts, forces and torques.

---

# 5. Detached tomato representation

For every detachable tomato, author an independent rigid-body representation outside the plant articulation.

Example:

```text
/Plant/DetachedBodies/TomatoDetached_01
```

At initialization:

```text
visibility              = false
physics:rigidBodyEnabled = false
```

Isaac Sim 4.5 supports setting `physics:rigidBodyEnabled` programmatically.

The detached object must have the same:

* visual geometry;
* collision geometry;
* mass;
* center of mass;
* inertia tensor;
* physics material;

as the attached tomato unless explicitly configured otherwise.

Do not simulate both representations simultaneously.

---

# 6. USD metadata schema

Use namespaced custom USD attributes.

Prefix all attributes with:

```text
tomatoPlant:
```

Custom namespaced USD attributes are an appropriate mechanism for exposing behavior parameters to Python logic; Isaac Sim 4.5 itself uses this pattern for modular behavior scripting.

## 6.1 Plant root metadata

On:

```text
/Plant
```

create:

```text
tomatoPlant:schemaVersion       string = "1.0"
tomatoPlant:enabled             bool   = true
tomatoPlant:runtimeType         string = "detachablePlant"
```

Optional:

```text
tomatoPlant:debug               bool = false
```

---

# 7. Fruit metadata

Put the primary metadata on the attached tomato prim.

Example:

```text
/Plant/Physics/TomatoAttached_01
```

Required:

```text
tomatoPlant:detachable = true

tomatoPlant:id = "tomato_01"
```

References:

```text
tomatoPlant:attachmentJoint
    = "/Plant/Physics/PeduncleTomatoJoint_01"

tomatoPlant:detachedBody
    = "/Plant/DetachedBodies/TomatoDetached_01"
```

Prefer USD relationships instead of strings when practical.

Conceptually:

```text
rel tomatoPlant:attachmentJoint = <...>
rel tomatoPlant:detachedBody    = <...>
```

---

# 8. Detachment model metadata

Version 1 must support at least three models:

```text
force
force_torque
custom
```

Metadata:

```text
tomatoPlant:detachmentModel = "force_torque"
```

Parameters:

```text
tomatoPlant:forceThreshold  = 2.5
tomatoPlant:torqueThreshold = 0.12
```

Units:

```text
forceThreshold  -> Newton
torqueThreshold -> Newton metre
```

Also store:

```text
tomatoPlant:forceExponent  = 2.0
tomatoPlant:torqueExponent = 2.0
```

The default combined damage function is:

[
D =
\left(
\frac{|F|}{F_c}
\right)^n
+
\left(
\frac{|M|}{M_c}
\right)^m
]

Detach when:

[
D \geq 1
]

where:

```text
Fc = forceThreshold
Mc = torqueThreshold
n  = forceExponent
m  = torqueExponent
```

For `force` mode:

[
D = \frac{|F|}{F_c}
]

and detach when:

[
D \geq 1
]

---

# 9. Optional directional force model

Design the API so that a later version can distinguish:

```text
tension
compression
shear
bending
torsion
```

Do not require this for V1.

Future metadata might contain:

```text
tomatoPlant:tensionThreshold
tomatoPlant:shearThreshold
tomatoPlant:bendingThreshold
tomatoPlant:torsionThreshold
```

Do not hard-code the assumption that only force magnitude will always be used.

---

# 10. Fruit runtime state

Each fruit has exactly one state:

```python
class FruitState(Enum):
    ATTACHED = 0
    DETACH_PENDING = 1
    DETACHED = 2
```

Normal transition:

```text
ATTACHED
    │
    │ detachment criterion
    ▼
DETACH_PENDING
    │
    │ topology/state transition
    ▼
DETACHED
```

No automatic transition from:

```text
DETACHED -> ATTACHED
```

during an episode.

Reattachment can be added later if required.

---

# 11. Runtime object model

Implement approximately:

```python
class TomatoPlantRuntime:

    def __init__(self, root_prim_path: str):
        ...

    def initialize(self):
        ...

    def step(self, dt: float):
        ...

    def reset(self):
        ...

    def shutdown(self):
        ...

    def get_fruits(self):
        ...

    def get_detached_fruits(self):
        ...
```

Internal representation:

```python
@dataclass
class FruitRuntimeData:

    fruit_id: str

    attached_prim_path: str
    detached_prim_path: str
    attachment_joint_path: str

    model: str

    force_threshold: float
    torque_threshold: float

    force_exponent: float
    torque_exponent: float

    state: FruitState

    last_force: np.ndarray
    last_torque: np.ndarray

    damage: float
```

---

# 12. Metadata discovery

At `initialize()`:

1. Get the plant root prim.
2. Verify:

```text
tomatoPlant:schemaVersion
```

3. Traverse descendants.
4. Find every prim for which:

```text
tomatoPlant:detachable == true
```

5. Parse metadata.
6. Resolve:

   * attached body;
   * attachment joint;
   * detached body.
7. Validate configuration.
8. Build `FruitRuntimeData`.
9. Disable all detached rigid bodies.
10. Initialize physics interfaces.

The runtime must support:

```text
0 tomatoes
1 tomato
N tomatoes
```

without source-code modifications.

---

# 13. Validation

Initialization must fail early with a useful error if:

* referenced joint does not exist;
* detached body does not exist;
* duplicate fruit IDs exist;
* thresholds are invalid;
* attached and detached paths are equal;
* attachment joint does not connect the expected fruit;
* detached body accidentally belongs to the articulation;
* detached body is enabled at initialization.

Example exception:

```text
TomatoPlantConfigurationError:
fruit 'tomato_03':
detached body '/Plant/DetachedBodies/TomatoDetached_03'
does not exist.
```

Do not silently ignore malformed fruit metadata.

---

# 14. Physics-step integration

The runtime must execute once per **physics step**, not once per rendered frame.

Isaac Sim 4.5 supports physics-step callbacks for this purpose.

Example integration:

```python
world.add_physics_callback(
    "tomato_plant_runtime",
    plant.on_physics_step,
)
```

The public API should not unnecessarily depend on `World`.

Provide:

```python
plant.step(dt)
```

as the core implementation and make the callback adapter thin:

```python
def on_physics_step(self, dt):
    self.step(dt)
```

This makes later Isaac Lab integration easier.

---

# 15. Per-step algorithm

For every attached fruit:

```python
def step(dt):

    events = []

    for fruit in attached_fruits:

        wrench = force_reader.read(
            fruit.attachment_joint_path
        )

        F = wrench.force
        M = wrench.torque

        D = detachment_model.evaluate(
            fruit,
            F,
            M,
        )

        fruit.last_force = F
        fruit.last_torque = M
        fruit.damage = D

        if D >= 1.0:
            fruit.state = DETACH_PENDING
```

After evaluating **all** fruits:

```python
for fruit in pending_fruits:
    event = detachment_backend.detach(fruit)
    events.append(event)
```

Do not modify articulation topology in the middle of iterating through force measurements.

---

# 16. Force-reader abstraction

Do not couple `TomatoPlantRuntime` directly to a specific Isaac API.

Create:

```python
class JointWrenchReader(ABC):

    def initialize(self, plant):
        ...

    def read(self, joint_path) -> JointWrench:
        ...
```

Data class:

```python
@dataclass
class JointWrench:
    force: np.ndarray   # shape (3,)
    torque: np.ndarray  # shape (3,)
```

Initial implementation:

```python
Isaac45ArticulationWrenchReader
```

Use Isaac Sim 4.5 articulation force/torque APIs or the corresponding articulation-state interface.

Isaac Sim 4.5's Articulation State node explicitly exposes measured joint forces and torques.

Keep coordinate-frame handling explicit.

The reader must document whether the wrench is expressed in:

```text
parent joint frame
child joint frame
world frame
```

and convert to one canonical runtime convention.

Recommended canonical representation:

```text
attachment-joint local frame
```

---

# 17. Detachment-model abstraction

Implement:

```python
class DetachmentModel(ABC):

    def evaluate(
        self,
        fruit,
        force,
        torque,
        dt,
    ) -> float:
        ...
```

Implementations:

```python
ForceThresholdModel
CombinedForceTorqueModel
```

Factory:

```python
model = DetachmentModelFactory.create(
    fruit.metadata
)
```

No model selection logic should appear in the main physics loop.

---

# 18. Detachment backend abstraction

This is critical.

Implement:

```python
class DetachmentBackend(ABC):

    def detach(
        self,
        plant,
        fruit,
    ) -> DetachmentEvent:
        ...
```

The rest of the project must not care how topology is modified.

Provide at least:

```python
ArticulationRebuildBackend
```

and optionally later:

```python
ExternalJointBackend
```

This abstraction is required because detaching a link from a reduced-coordinate articulation is fundamentally different from breaking an ordinary maximal-coordinate joint.

---

# 19. Correctness-first detachment algorithm

The preferred backend should preserve the correct attached dynamics and perform the topology transition only at detachment.

Before changing anything, snapshot the state.

Required state:

```python
@dataclass
class RigidBodyState:

    position: np.ndarray
    orientation: np.ndarray

    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
```

For the fruit:

```python
fruit_state = read_body_state(
    attached_tomato
)
```

Also snapshot any articulation state required for rebuilding the remaining plant.

---

# 20. Important topology constraint

Do **not** implement detachment as simply:

```python
attached_tomato.set_visibility(False)
detached_tomato.enable()
```

This is incorrect.

The attached tomato would still exist as an articulation link and therefore could continue affecting the articulation dynamics.

Similarly, visibility alone affects rendering, not the physical topology.

The detachment backend must ensure that after detachment:

```text
remaining plant articulation
```

no longer contains the detached fruit as a dynamic link.

---

# 21. Articulation rebuild backend

Implement the topology-changing backend conservatively.

Conceptual sequence:

```text
physics step N
      │
      ▼
detect detachment
      │
      ▼
snapshot physics state
      │
      ▼
temporarily stop / safely suspend affected physics structure
      │
      ▼
remove detached fruit from articulation topology
      │
      ▼
reinitialize affected articulation representation
      │
      ▼
restore plant state
      │
      ▼
activate detached rigid body
      │
      ▼
restore fruit pose + velocity
      │
      ▼
physics step N+1
```

Exact Isaac Sim 4.5 calls must be isolated inside:

```python
ArticulationRebuildBackend
```

Do not scatter USD/PhysX topology manipulation throughout the runtime.

---

# 22. State transfer

Immediately before detachment read:

```text
p_A = attached tomato world position
q_A = attached tomato world orientation

v_A = attached tomato linear velocity
w_A = attached tomato angular velocity
```

Then initialize the detached rigid body with:

```text
p_B = p_A
q_B = q_A
v_B = v_A
w_B = w_A
```

The detached rigid body must therefore inherit momentum from the attached fruit.

Conceptually:

```text
before

branch──────●
            ↑
          tomato
            velocity v


after

branch

             ●
             → v
```

The detached tomato must not start with zero velocity.

---

# 23. Detached rigid-body activation order

Recommended order:

```python
# snapshot
state = read_attached_state()

# perform articulation topology update
remove_from_articulation(...)

# configure detached representation while disabled
set_world_pose(detached, state.pose)
set_velocity(detached, state.velocity)

# visual transition
hide(attached_visual)
show(detached_visual)

# enable physics last
enable_rigid_body(detached)
```

Avoid temporarily enabling two overlapping tomato colliders at the same pose.

Isaac Sim 4.5 provides programmatic rigid-body enable/disable support through `physics:rigidBodyEnabled`.

---

# 24. Multiple fruits

The architecture must support multiple independently detachable fruits.

Example:

```text
Plant

Branch A ─ tomato_01
Branch B ─ tomato_02
Branch C ─ tomato_03
```

Each fruit has independent:

```text
state
thresholds
joint
detached body
```

However, note:

> Removing an articulation link changes the articulation topology.

Therefore multiple simultaneous detachment requests should be batched when possible.

Example:

```python
pending = [
    tomato_02,
    tomato_07,
    tomato_09,
]

backend.detach_batch(pending)
```

Prefer one topology rebuild per physics step rather than one rebuild for every fruit.

---

# 25. Reset behavior

`reset()` must restore the original USD-defined attached configuration.

Required result:

```text
all fruits:
    state = ATTACHED

attached fruit physics:
    active as articulation links

detached rigid bodies:
    rigidBodyEnabled = false
    visible = false
```

All runtime accumulated values must reset:

```text
damage = 0
last_force = 0
last_torque = 0
```

Reset must be deterministic.

---

# 26. Debugging interface

Optional but recommended.

Expose:

```python
plant.get_debug_state()
```

For every fruit:

```python
{
    "id": "tomato_03",
    "state": "ATTACHED",

    "force": [Fx, Fy, Fz],
    "torque": [Mx, My, Mz],

    "force_norm": ...,
    "torque_norm": ...,

    "damage": ...,
}
```

Optional viewport debugging may display:

```text
Tomato 03

F = 1.83 N
M = 0.067 Nm
D = 0.61
ATTACHED
```

Debugging must be disableable.

---

# 27. Suggested source-tree layout

Implement:

```text
tomato_plant/
│
├── __init__.py
│
├── runtime.py
│
├── metadata.py
│
├── models.py
│
├── force_reader.py
│
├── state.py
│
├── events.py
│
│
├── backends/
│   ├── __init__.py
│   ├── base.py
│   └── articulation_rebuild.py
│
└── tests/
    ├── test_metadata.py
    ├── test_detachment_models.py
    ├── test_state_machine.py
    ├── test_single_fruit.py
    ├── test_multiple_fruits.py
    └── test_state_transfer.py
```

Do not create one giant `tomato_plant.py`.

---

# 28. Core classes

Target roughly:

```text
TomatoPlantRuntime
│
├── PlantMetadataParser
│
├── JointWrenchReader
│
├── DetachmentModel
│
└── DetachmentBackend
```

Dependency direction:

```text
               TomatoPlantRuntime
                  /     |     \
                 /      |      \
                ▼       ▼       ▼
          metadata   models   backend
                                │
                                ▼
                         Isaac Sim 4.5
```

Only the backend and wrench reader should strongly depend on Isaac-specific physics APIs.

---

# 29. Portability requirement

The following must be possible:

```python
plant = TomatoPlantRuntime(
    "/World/CompletelyDifferentPlant"
)
```

provided that the new plant follows the metadata schema.

No code changes should be necessary.

For example, this must **not** exist:

```python
if path == "/World/Plant/Tomato_01":
    threshold = 2.5
```

Nor:

```python
TOMATO_PATHS = [
    "/World/Plant/Tomato1",
    "/World/Plant/Tomato2",
]
```

All such information belongs in USD.

---

# 30. Isaac Lab compatibility

Do not directly couple the physics model to:

* reward functions;
* observations;
* policy;
* actions;
* episode termination.

The plant runtime should work independently:

```python
events = plant.step(dt)
```

An Isaac Lab task can then consume:

```python
events
```

or:

```python
plant.fruit_states
```

For example:

```python
reward += detached_this_step * harvest_reward
```

The plant runtime must not know that reinforcement learning is occurring.

---

# 31. Future vectorization

V1 may support a single plant in normal Isaac Sim Python.

However, design data structures so that a later implementation can replace:

```python
for fruit in fruits:
```

with vectorized arrays/tensors.

Do not unnecessarily embed Python object references inside the physics computation.

Prefer indexed data such as:

```text
fruit index
joint index
body index
```

after initialization.

---

# 32. Performance requirement

No USD hierarchy traversal inside the physics loop.

Do discovery only during:

```text
initialize()
reset/rebuild if required
```

During normal steps use cached references/indices.

Bad:

```python
def step():
    stage.Traverse()
```

Good:

```python
def initialize():
    self._fruits = discover_fruits()
```

followed by:

```python
def step():
    wrench_reader.read_cached(...)
```

---

# 33. Minimum V1 detachment model

Start simple.

Implement:

```python
class CombinedForceTorqueModel:

    def evaluate(self, fruit, force, torque, dt):

        F = np.linalg.norm(force)
        M = np.linalg.norm(torque)

        force_term = (
            F / fruit.force_threshold
        ) ** fruit.force_exponent

        torque_term = (
            M / fruit.torque_threshold
        ) ** fruit.torque_exponent

        return force_term + torque_term
```

Detach when:

```python
damage >= 1.0
```

Do not implement fatigue/damage accumulation in V1 unless required.

---

# 34. Optional persistence criterion

Architect the code so that V2 can require a threshold to remain exceeded for a minimum duration:

```text
tomatoPlant:minimumBreakDuration = 0.020
```

For example:

```python
if instantaneous_damage >= 1:
    overload_time += dt
else:
    overload_time = 0
```

Detach when:

```python
overload_time >= minimum_break_duration
```

This may later help prevent single-step numerical spikes from causing false detachment.

Do not make it mandatory in the first implementation.

---

# 35. Logging

Log:

```text
plant initialization
number of detachable fruits
fruit IDs
metadata validation failures
detachment events
topology rebuild failures
```

Example:

```text
[TomatoPlant] initialized /World/Plant
[TomatoPlant] discovered 7 detachable fruits

[TomatoPlant]
DETACH tomato_03
F=2.74 N
M=0.083 Nm
D=1.12
```

Avoid logging every physics step by default.

---

# 36. Acceptance test A — asset discovery

Given a USD containing three tagged fruits:

```text
tomato_01
tomato_02
tomato_03
```

the runtime must automatically discover all three.

Expected:

```python
len(plant.fruits) == 3
```

No fruit paths may be supplied from Python.

---

# 37. Acceptance test B — no premature detachment

Apply forces below threshold.

Expected:

```text
state = ATTACHED
```

for at least several seconds of simulation.

No detached rigid body may become active.

---

# 38. Acceptance test C — threshold detachment

Apply a controlled load that causes:

[
D > 1
]

Expected:

```text
ATTACHED
   ↓
DETACHED
```

exactly once.

A second event must not be emitted in following frames.

---

# 39. Acceptance test D — velocity continuity

Immediately before detachment record:

```text
v_before
w_before
```

Immediately after activation of the free tomato record:

```text
v_after
w_after
```

Require approximately:

[
v_{after} \approx v_{before}
]

and:

[
\omega_{after} \approx \omega_{before}
]

within a documented numerical tolerance.

---

# 40. Acceptance test E — pose continuity

The visual tomato must not visibly teleport at detachment.

Require:

[
|p_{after}-p_{before}| < \epsilon_p
]

and orientation error:

[
\Delta\theta < \epsilon_R
]

Use tolerances appropriate to simulation timestep and scene scale.

---

# 41. Acceptance test F — attached physics equivalence

This is a critical test.

Create:

```text
Plant A
```

using the original desired all-in-articulation model.

Create:

```text
Plant B
```

using the new metadata/runtime asset, but with no detachment occurring.

Run identical cantilever/free-oscillation tests.

Compare:

```text
tip displacement
oscillation frequency
damping
tomato trajectory
```

Before detachment, Plant B should reproduce Plant A within numerical tolerance.

This validates the main reason for using this architecture.

---

# 42. Acceptance test G — detached mass removal

After detachment, verify that the old attached fruit no longer contributes dynamically to the plant.

A simple test:

1. Let the plant reach rest with tomato attached.
2. Measure static branch deflection.
3. Trigger tomato detachment.
4. Let the plant settle.
5. Measure branch deflection again.

Expected:

```text
deflection_after_detachment
<
deflection_with_attached_tomato
```

The difference should correspond to removal of the fruit load.

This test catches an incorrect implementation where only the tomato visualization was swapped.

---

# 43. Acceptance test H — reset

After detachment call:

```python
plant.reset()
```

Expected:

```text
fruit attached again
detached proxy disabled
damage = 0
no stale velocity
original articulation restored
```

Repeat:

```text
detach → reset
```

at least 100 times without accumulating invalid physics state.

---

# 44. Acceptance test I — multiple fruit

With at least three fruits:

```text
tomato_01
tomato_02
tomato_03
```

detach only `tomato_02`.

Expected:

```text
01 ATTACHED
02 DETACHED
03 ATTACHED
```

The plant must remain stable and the other fruit metadata/state must be unchanged.

---

# 45. Do not optimize prematurely

Correctness order:

```text
1. Correct attached physics
2. Correct force measurement
3. Correct state transition
4. Correct topology after detachment
5. Correct momentum transfer
6. Reliable reset
7. Multiple fruit
8. Performance/vectorization
9. Isaac Lab optimization
```

Do not compromise items 1–5 merely to avoid a topology rebuild.

---

# 46. Explicit non-goals for V1

Do not implement yet:

* biological fracture propagation;
* plastic deformation of the peduncle;
* fatigue accumulation;
* reattachment;
* tearing geometry;
* GPU-vectorized detachment;
* distributed breakage along arbitrary branches;
* arbitrary articulation fracture.

Keep the first implementation focused specifically on detachable fruits.

---

# 47. Important design warning

Do not assume that:

```text
pre-created detached tomato
+
hide attached tomato
```

alone is sufficient.

The purpose of the architecture is not merely visual detachment.

The physical requirement is:

```text
before:

Plant articulation mass
=
branch mass
+
peduncle mass
+
fruit mass


after:

Plant articulation mass
=
branch mass
+
peduncle mass

Free rigid-body mass
=
fruit mass
```

This invariant must hold.

---

# 48. Implementation strategy for Codex

Implement in phases.

## Phase 1

Implement:

```text
metadata parser
data classes
state machine
detachment models
unit tests
```

without manipulating PhysX topology.

## Phase 2

Implement:

```text
Isaac45ArticulationWrenchReader
```

and verify measured force/torque.

## Phase 3

Implement a standalone one-fruit test scene.

Verify:

```text
F/M reading
threshold detection
event generation
```

## Phase 4

Implement:

```text
ArticulationRebuildBackend
```

for one leaf tomato.

Verify topology and state transfer.

## Phase 5

Add:

```text
multiple tomatoes
batch detachment
reset
```

## Phase 6

Only after the standalone Isaac Sim implementation passes tests, add an Isaac Lab adapter.

---

# 49. Required documentation

Add:

```text
README.md
```

covering:

```text
Architecture
USD metadata schema
How to author a detachable fruit
Runtime API
Example
Known limitations
Isaac Sim version
```

Explicitly state:

```text
Target simulator: NVIDIA Isaac Sim 4.5
```

Do not silently use APIs introduced in later Isaac Sim releases.

---

# 50. Final architectural invariant

The implementation is correct only if these statements remain true:

### While attached

```text
The tomato is physically part of the same articulation
that provides the desired plant dynamics.
```

### At detachment

```text
A generic runtime detects a physically meaningful
force/torque criterion.
```

### After detachment

```text
The fruit is an independent rigid body and no longer
contributes mass or constraints to the plant articulation.
```

### Portability

```text
Plant-specific physics parameters live in USD metadata.

Generic behavior lives in Python.

Robot-task-specific logic lives outside both.
```

Target conceptual interface:

```python
plant = TomatoPlantRuntime("/World/Plant")
plant.initialize()

# simulation loop
events = plant.step(dt)
```

The same runtime should work with another correctly authored plant USD without source-code changes.
