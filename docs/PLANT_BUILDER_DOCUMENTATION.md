# PlantBuilder: Modular USD Articulation API for Isaac Sim

## 1. Overview & Architecture

`PlantBuilder` is a high-level Python library designed to procedurally construct physically articulated plant models in OpenUSD (Pixar USD) for simulation within **Nvidia Isaac Sim (PhysX)**.

Instead of writing custom, low-level USD scene manipulation code for every unique plant structure, `PlantBuilder` abstracts geometric alignment, parent-child coordinate frame transformations, PhysX D6 joint configurations, and physical drive parameter tuning into four elementary primitive operations:

1. `create_root`: Initializes the trunk base and anchors it to the simulation world via a `FixedJoint`.
2. `add_internode`: Sequentially extends an existing branch or stem along its local longitudinal axis ($Z$-axis).
3. `add_lateral_branch`: Attaches a lateral branch at an arbitrary height, tilt angle, and azimuth around any parent cylinder surface.
4. `add_leaf`: Attaches an articulated leaf structure comprising a rigid-body petiole and child blade mesh.

```
                   World Stage (/World)
                             │
                  [PhysxArticulationAPI]
                      /World/Stem
                             │
              ┌──────────────┴──────────────┐
       Trunk Segment (T01)          FixedJoint (to World)
              │ (Internode D6 Joint)
       Trunk Segment (T02)
       ┌──────┴────────────────────────────┐
  Lateral Branch (B01)            Trunk Segment (T03)
  ┌────┴───────────────┐
Branch Segment (B02)  Subbranch (SB01)
                             │
                      Petiole / Leaf (LA01)
```

---

## 2. Mathematical & Kinematic Foundations

### 2.1 Coordinate Frame Transformations

Every plant segment $i$ maintains a local coordinate system centered at its base, where the cylinder axis points along local $+Z$. 

The global orientation of a segment is tracked as a `pxr.Gf.Rotation` object (`global_rot`). When attaching a lateral branch or leaf to a parent segment $P$:

1. **Surface Base Position ($P_{world}$)**:
   A parent cylinder has radius $R_P$ and length $L_P$. Given a relative vertical attachment ratio $\eta_z \in [0, 1]$, tilt angle $\theta_{tilt}$ (degrees away from parent axis), and azimuth $\phi_{az}$ (degrees rotation around parent axis):

   $$\mathbf{o}_{local} = \begin{bmatrix} 0 \\ R_P \\ \eta_z \cdot L_P \end{bmatrix}$$

   $$\mathbf{R}_{z}(\phi_{az}) = \text{RotationAroundZ}(\phi_{az})$$
   $$\mathbf{R}_{x}(\theta_{tilt}) = \text{RotationAroundX}(-\theta_{tilt})$$

   $$\mathbf{R}_{local} = \mathbf{R}_{x}(\theta_{tilt}) \cdot \mathbf{R}_{z}(\phi_{az})$$
   $$\mathbf{p}_{parent\_frame} = \mathbf{R}_{z}(\phi_{az}) \cdot \mathbf{o}_{local}$$

   $$\mathbf{P}_{world} = \mathbf{P}_{parent\_base} + \mathbf{R}_{parent\_global} \cdot \mathbf{p}_{parent\_frame}$$

2. **Global Orientation ($\mathbf{R}_{global}$)**:
   $$\mathbf{R}_{global} = \mathbf{R}_{local} \cdot \mathbf{R}_{parent\_global}$$

### 2.2 Quaternion Precision & USD Schema Compatibility

To avoid precision mismatches between Pixar USD double-precision calculations (`Gf.Quatd`) and Isaac Sim PhysX single-precision attributes (`Gf.Quatf`), `PlantBuilder` utilizes an explicit double-to-float quaternion converter (`_quatd_to_quatf`):

```python
def _quatd_to_quatf(qd: Gf.Quatd) -> Gf.Quatf:
    imag = qd.GetImaginary()
    return Gf.Quatf(float(qd.GetReal()), float(imag[0]), float(imag[1]), float(imag[2]))
```

---

## 3. PhysX Joint Configuration & Impedance Tuning

### 3.1 D6 Joint Setup

All segment-to-segment connections are defined using 6-DOF generic joints (`UsdPhysics.Joint` / D6 Joints).
- **Linear Degrees of Freedom**: Locked (`transX`, `transY`, `transZ` low limit > high limit).
- **Angular Degrees of Freedom**:
  - `rotX` and `rotY`: Limited by angular bend thresholds (`bend_limit`) and driven by PD force controllers (`UsdPhysics.DriveAPI`).
  - `rotZ`: Locked for straight internodes to prevent torsional spinning, or driven for lateral branches and leaves.

### 3.2 Dynamic Stiffness & Damping Tiers

Biological plant tissue exhibits varying degrees of compliance depending on structural hierarchy:

| Hierarchy Level | Segment Type | Typical Mass (kg) | Stiffness ($N\cdot m/rad$) | Damping ($N\cdot m\cdot s/rad$) | Lock Z |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Trunk (Base)** | Primary Internode | 1.0 – 2.0 | 500,000.0 – 800,000.0 | 50.0 – 200.0 | Yes |
| **Trunk (Upper)** | High Internode | 0.5 – 1.0 | 300.0 – 50,000.0 | 30.0 – 50.0 | Yes |
| **Primary Branch** | Base Joint | 0.2 – 0.3 | 50,000.0 | 2,000.0 | No |
| **Secondary Branch**| Subbranch Joint | 0.08 – 0.1 | 80.0 – 150.0 | 20.0 – 30.0 | No |
| **Leaf Petiole** | Petiole D6 Joint | 0.01 – 0.02 | 0.0002 | 0.00006 | No |

> **Crucial Physics Insight**: For extremely small visual organs like leaf petioles ($m \approx 10-20\text{g}$, length $\approx 2-3\text{cm}$), standard joint stiffness values ($> 1.0$) cause immediate numerical instability or rigid lockup. Stiffness of $2 \times 10^{-4}$ and damping of $6 \times 10^{-5}$ allow realistic gravitational drooping and wind-response fluttering without exploding the TGS solver.

---

## 4. API Reference

### `PlantBuilder(stage: Usd.Stage, base_path: str = "/World/Stem", global_scale: float = 1.0)`
Instantiates the plant builder context on a given USD stage. Automatically applies `PhysxSchema.PhysxArticulationAPI` to `base_path`.

### `create_root(id: str, radius: float, length: float, mass: float = 1.0) -> str`
Creates the root cylinder segment and anchors it to the world using a `UsdPhysics.FixedJoint`.

### `add_internode(parent_id: str, id: str, radius: float, length: float, mass: float = 1.0, stiffness: float = None, damping: float = None) -> str`
Appends a sequential stem segment at the top tip of `parent_id`.

### `add_lateral_branch(parent_id: str, id: str, radius: float, length: float, z_offset_ratio: float, tilt_angle: float, rot_around_parent: float, mass: float = 0.2, stiffness: float = None, damping: float = None) -> str`
Creates a lateral branch attached to the outer cylinder surface of `parent_id`.

### `add_leaf(parent_id: str, id: str, leaf_length: float = 0.08, leaf_width: float = 0.04, petiole_length: float = None, petiole_radius: float = None, z_offset_ratio: float = 1.0, tilt_angle: float = 60.0, rot_around_parent: float = 0.0, mass: float = 0.02, stiffness: float = 0.0002, damping: float = 0.00006) -> str`
Creates an articulated leaf assembly consisting of an articulated rigid-body petiole cylinder and a child 16-point ovate leaf blade mesh.

---

## 5. Security & Safety Validation

`PlantBuilder` enforces strict automated safety checks during graph construction:

1. **Max Link Depth Guard**: PhysX Articulation solver TGS limits maximal link depth to 64. If `depth >= 64`, `PlantBuilder` raises a `ValueError`. Warnings are logged above depth 50.
2. **Slenderness Ratio Check**: If aspect ratio $\frac{L}{R} > 25$, a console warning is emitted indicating potential PhysX solver jittering.
3. **ID Uniqueness**: Duplicate segment identifiers raise an immediate error before stage mutation.

---

## 6. Test Suite & Verification

The suite includes both automated headless tests (`tests/plant_builder/run_tests.py`) and visual interactive scenarios (`tests/plant_builder/visual_tests.py`):

```bash
# Run headless structural & mathematical unit tests
./run_builder_tests.sh

# Run visual progressive test scenarios (GUI)
./run_experiment.sh 1   # Test 1: Trunk only
./run_experiment.sh 3   # Test 3: Extended branches
./run_experiment.sh 5   # Test 5: Complex multi-tier tree
./run_experiment.sh 6   # Test 6: Flexible bending dynamics
./run_experiment.sh 7   # Test 7: Articulated leaves with soft petioles
```
