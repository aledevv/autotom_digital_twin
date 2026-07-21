# Procedural Articulation Framework for Botanical Physics Simulation in Isaac Sim

## 1. Introduction & Motivation

Simulating realistic biological plants within modern physics engines—such as Nvidia PhysX integrated into Isaac Sim—presents a fundamental trade-off between geometric fidelity, kinematic complexity, and numerical stability. Traditional approaches to digital twin creation often rely on ad-hoc, hardcoded USD (Universal Scene Description) generation scripts or direct static mesh imports. While static models provide accurate visual representations, they lack physical compliance, rendering them unsuitable for robotic manipulation, agricultural harvesting scenarios, or aerodynamic interaction studies where plant bodies bend and deform under external contact and gravity.

Conversely, manually specifying multi-body dynamics for hundreds of organic stem segments and leaf petioles leads to highly brittle implementations. Manual transform calculations are prone to frame alignment errors, joint interpenetrations, and numerical explosion in the physics solver due to poorly tuned joint impedance parameters.

To resolve these challenges, a generalized, declarative framework—termed **`PlantBuilder`**—was formulated and implemented. This framework abstracts low-level USD scene graph operations and PhysX D6 joint configurations into a set of recursive, biologically inspired primitive operations. By decoupling spatial kinematics, mass property distribution, and joint drive compliance from specific plant topographies, this architecture establishes a scalable foundation for procedural plant modeling applicable to tomatoes (*Solanum lycopersicum*), woody crops, and general botanical structures.

---

## 2. Kinematic Topology & Transform Mechanics

A plant is modeled as an articulated structure governed by a directed acyclic tree topology $\mathcal{G} = (\mathcal{V}, \mathcal{E})$, where vertices $v_i \in \mathcal{V}$ represent rigid cylindrical internode segments or visual organs (petioles/leaf blades), and directed edges $e_{ij} \in \mathcal{E}$ represent 6-DOF physical joints.

```
                         [ World Frame: /World ]
                                    │ FixedJoint
                            [ Root Internode: T01 ]
                                    │ D6 Joint (axial offset)
                            [ Trunk Internode: T02 ]
                                    │
           ┌────────────────────────┴────────────────────────┐
   D6 Joint (surface attach)                         D6 Joint (axial offset)
   [ Lateral Branch: B01 ]                           [ Trunk Internode: T03 ]
           │ D6 Joint                                        │ D6 Joint
   [ Branch Internode: B02 ]                         [ Leaf Petiole: L01 ]
           │ D6 Joint                                        │ Static Child Mesh
   [ Subbranch: SB01 ]                               [ Leaf Blade Mesh ]
```

### 2.1 Forward Kinematics of Surface Attachment

Unlike standard robotic manipulators connected strictly tip-to-base, plant structures exhibit lateral branching where secondary stems originate from arbitrary positions along the outer boundary of a parent cylinder.

Let a parent segment $P$ be defined by length $L_P$, radius $R_P$, world position $\mathbf{P}_{base} \in \mathbb{R}^3$, and orientation matrix $\mathbf{R}_{P} \in S O(3)$. A lateral organ (branch or petiole) attached at relative height $\eta_z \in [0, 1]$, tilt angle $\theta \in [-\pi, \pi]$, and azimuthal angle $\phi \in [0, 2\pi)$ undergoes the following spatial transformation:

1. **Local Offset Construction**:
   $$\mathbf{o}_{local} = \begin{bmatrix} 0 \\ R_P \\ \eta_z L_P \end{bmatrix}$$

2. **Azimuthal & Insertion Angle Rotations**:
   $$\mathbf{R}_z(\phi) = \begin{bmatrix} \cos\phi & -\sin\phi & 0 \\ \sin\phi & \cos\phi & 0 \\ 0 & 0 & 1 \end{bmatrix}, \quad \mathbf{R}_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos(-\theta) & -\sin(-\theta) \\ 0 & \sin(-\theta) & \cos(-\theta) \end{bmatrix}$$

   $$\mathbf{R}_{local} = \mathbf{R}_x(\theta) \mathbf{R}_z(\phi)$$

3. **Parent Frame Relative Position**:
   $$\mathbf{p}_{parent} = \mathbf{R}_z(\phi) \mathbf{o}_{local}$$

4. **Global Pose Computation**:
   $$\mathbf{W}_{child} = \mathbf{P}_{base} + \mathbf{R}_P \mathbf{p}_{parent}$$
   $$\mathbf{R}_{child} = \mathbf{R}_{local} \mathbf{R}_P$$

To maintain strict compatibility with Isaac Sim's single-precision PhysX API while preserving double-precision transform integrity in Pixar USD, all quaternions $\mathbf{q} \in S^3$ are constructed using double-precision rotation algebra (`Gf.Rotation`) and explicitly cast to single-precision representation (`Gf.Quatf`) prior to stage composition.

---

## 3. Physical Articulation Dynamics & Impedance Tuning

### 3.1 Joint Formulation & Limit Constraints

Inter-segment connections utilize 6-DOF D6 joints (`UsdPhysics.Joint`). Translational degrees of freedom ($\text{transX}, \text{transY}, \text{transZ}$) are locked by setting lower bounds greater than upper bounds ($\text{low} > \text{high}$), enforcing strict rigid link attachment.

Angular motion is governed by proportional-derivative (PD) drives (`UsdPhysics.DriveAPI`) operating on orthogonal rotational axes ($\text{rotX}, \text{rotY}, \text{rotZ}$):

$$\tau = -K_p (\theta - \theta_{target}) - K_d \dot{\theta}$$

where $K_p$ represents angular stiffness ($N\cdot m/\text{rad}$), $K_d$ denotes angular damping ($N\cdot m\cdot s/\text{rad}$), and $\theta_{target} = 0$ maintains structural equilibrium under zero-load conditions.

### 3.2 Multi-Tier Impedance Parameterization

A critical discovery in simulating biological compliance is that uniform joint parameters fail across multi-scale organ hierarchies. High stiffness on small distal organs causes numerical stiffness in the solver, leading to explosive instabilities or rigid locking, whereas low stiffness on primary trunks leads to structural collapse under self-weight.

To achieve physical fidelity across all scales, a multi-tier impedance parameterization was derived:

```
                            [ IMPEDANCE SPECTRUM ]

     High Stiffness                                          Low Stiffness
  (Rigid Load-Bearing)                                   (High Compliance)
   ───► 500,000 N·m/rad       50,000 N·m/rad        150 N·m/rad      0.0002 N·m/rad ───►
      ┌──────────────┐     ┌──────────────┐     ┌────────────┐     ┌────────────┐
      │ Primary      │     │ Lateral      │     │ Secondary  │     │ Leaf       │
      │ Trunk Base   │     │ Branch Base  │     │ Internodes │     │ Petioles   │
      └──────────────┘     └──────────────┘     └────────────┘     └────────────┘
```

1. **Primary Trunk Tier**: High stiffness ($K_p \approx 5 \times 10^5 \text{ N}\cdot\text{m/rad}, K_d \approx 50 \text{ N}\cdot\text{m s/rad}$) anchors the main vertical axis, counteracting cumulative gravitational moment.
2. **Lateral Branch Attachment Tier**: Moderate stiffness ($K_p \approx 5 \times 10^4 \text{ N}\cdot\text{m/rad}, K_d \approx 2 \times 10^3 \text{ N}\cdot\text{m s/rad}$) permits realistic cantilever sagging under branch self-weight.
3. **Distal Branch Internode Tier**: Soft stiffness ($K_p \approx 80 - 150 \text{ N}\cdot\text{m/rad}, K_d \approx 20 - 30 \text{ N}\cdot\text{m s/rad}$) allows continuous bending curves along extended multi-segment branches.
4. **Articulated Leaf Petiole Tier**: Ultra-low impedance ($K_p \approx 2 \times 10^{-4} \text{ N}\cdot\text{m/rad}, K_d \approx 6 \times 10^{-5} \text{ N}\cdot\text{m s/rad}$) applied directly at the branch-to-petiole D6 joint. Because leaf mass is tiny ($m \approx 10-20\text{g}$), this micro-impedance allows leaves to swing and droop naturally under gravity without causing PhysX TGS solver jittering.

---

## 4. Structural Safety Guards & Solver Optimization

To prevent simulation crashes in Isaac Sim, automated security checks are embedded into the creation graph:

* **Kinematic Chain Depth Guard**: PhysX Temporal Gauss-Seidel (TGS) solver enforces a strict maximum articulation depth of 64 links. The builder tracks graph depth $d(v)$ and halts execution if $d \ge 64$, emitting warnings when approaching the threshold ($d > 50$).
* **Slenderness Ratio Constraint**: Extremely thin cylinders ($\frac{L}{R} > 25$) exhibit poor inertia tensor conditioning. The system logs aspect ratio warnings to recommend geometric simplification.
* **Solver Configuration**: The generated USD stage configures the TGS solver with 64 position iterations and 8 velocity iterations per timestep ($f_{sim} = 120\text{ Hz}$), guaranteeing constraint convergence across deep articulation trees.

---

## 5. Experimental Validation

The framework was validated through a suite of progressive visual test scenarios integrated into Isaac Sim GUI execution (`visual_tests.py`).

| Test Scenario | Topological Description | Validated Behavior |
| :--- | :--- | :--- |
| **Test 1: Trunk Only** | 5 sequential trunk internodes | Base FixedJoint anchoring & axial alignment under gravity. |
| **Test 3: Branch Extension** | Trunk + branch extended with 3 internodes | Surface attachment math and multi-segment cantilever deflection. |
| **Test 5: Full Tree Topology** | 8 trunk internodes, 3 lateral branches, 4 subbranches | Complex multi-tier articulation hierarchy stability under TGS solver. |
| **Test 6: Flexible Bending Dynamics** | 10 trunk internodes, 4 long branches (18 segments total), 6 subbranches | Gradual drooping profiles across stiffness tiers without solver jitter. |
| **Test 7: Articulated Leaf Petioles** | Trunk + 2 lateral branches + 11 leaves (petiole + 16-point ovate blade mesh) | Branch-to-petiole D6 joint articulation and micro-impedance response. |

```
                       [ EXPERIMENTAL RESULTS SUMMARY ]

   Simple Trunk           Branched Hierarchy            Full Tree + Leaves
   (Tests 1 - 2)            (Tests 3 - 5)                 (Tests 6 - 7)
       │                         │                              │
  ┌────┴────┐               ┌────┴────┐                    ┌────┴────┐
  │ Fixed   │               │ Branch  │                    │ Micro-  │
  │ Anchor  │               │ Sagging │                    │ Drooping│
  └─────────┘               └─────────┘                    └─────────┘
```

The progressive testing suite confirms that the proposed procedural formulation eliminates ad-hoc script maintenance while providing a robust, physics-compliant framework for simulating complex plant mechanics in Isaac Sim.
