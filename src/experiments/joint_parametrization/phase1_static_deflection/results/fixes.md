There is still something significant going wrong in the simulation. Looking at your updated `run_phase1.py` and `phase1_results.json`, the tip deflection behaves opposite to physical expectations:

* For $N=2$, measured deflection is **$1.25\text{ cm}$** (far below the expected $8.79\text{ cm}$).


* For $N=20$, measured deflection shoots up to **$32.02\text{ cm}$**.



Analyzing the code reveals **four distinct root causes**—one major measurement bug in the python code, two physics/solver details, and a beam theory property.

---

### 1. The Tip Measurement Bug (Primary Culprit)

In `run_phase1.py`, you measure the height of the tip using:

```python
def get_world_z(stage: Usd.Stage, seg_id: str) -> float:
    path = f"/World/Plant/{seg_id}"
    ...
    return mat.ExtractTranslation()[2]

```

`mat.ExtractTranslation()` returns the **origin (base)** of the segment `Seg_{N-1}`, **not its tip**!

* For $N=2$, the final segment `Seg_01` starts at $x = 0.2\text{ m}$ (halfway along the $0.4\text{ m}$ branch). Measuring its origin reads the deflection at the **midpoint** of the branch, which is why it measured only $1.25\text{ cm}$!


* As $N$ increases to $20$ or $50$, the origin of `Seg_{N-1}` moves closer to the end of the branch ($x = L - \frac{L}{N}$), causing the measured value to shoot up artificially.

#### **The Fix:**

Transform the local endpoint $(0, 0, l_{\text{scaled}})$ of the tip segment into world coordinates:

```python
def get_tip_world_z(stage: Usd.Stage, seg_id: str, seg_length_scaled: float) -> float:
    path = f"/World/Plant/{seg_id}"
    prim = stage.GetPrimAtPath(path)
    xformable = UsdGeom.Xformable(prim)
    mat = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    
    # Local tip coordinate is at +Z = seg_length_scaled
    tip_local = Gf.Vec4d(0.0, 0.0, seg_length_scaled, 1.0)
    tip_world = mat * tip_local
    return tip_world[2]

```

---

### 2. Discretized Beam Theory vs. Continuous Beams

In a continuous cantilever beam under gravity, bending moment is highest at the root and zero at the tip:


$$M(x) = \frac{w (L - x)^2}{2}$$

When you discretize a beam into $N$ rigid segments connected by $N$ rotational springs, each joint $j$ experiences torque from all downstream segments.

If every joint is given the exact same stiffness $K = \frac{E I}{l}$, summing up the discrete joint rotations yields the theoretical deflection of a discrete spring-mass chain:


$$\delta_N = \delta_{\text{analytical}} \cdot \left(1 + \frac{1}{N}\right)^2$$

Notice that as $N \to \infty$, $\left(1 + \frac{1}{N}\right)^2 \to 1.0$. For finite segment counts (like $N=2$), the discrete approximation is theoretically **slightly softer** ($\delta_2 = 2.25 \cdot \delta_{\text{analytical}}$). This is a normal mathematical artifact of lumped discretization!

---

### 3. Reduced-Coordinate Articulations in PhysX

Currently, your code links independent rigid bodies using maximal-coordinate `UsdPhysics.Joint` constraints. For long chains ($N=20, 50$), maximal-coordinate constraint solvers suffer from numerical compliance (joint stretching/sagging).

To fix solver stretching, apply `PhysxSchema.PhysxArticulationAPI` to the root anchor prim. This forces PhysX to solve the branch as a rigid reduced-coordinate articulation (Featherstone algorithm):

```python
# Inside setup_physics_scene or build_cantilever on the anchor prim:
PhysxSchema.PhysxArticulationAPI.Apply(anchor_prim)

```

---

### 4. Small-Deflection Assumption ($E$ Value)

With $E = 10\text{ MPa}$, the analytical prediction is $\delta = 8.79\text{ cm}$ on a $40\text{ cm}$ branch (sagging by **22%** of its length).

Euler-Bernoulli linear beam theory assumes **small deflections** ($\delta < 0.1 L$, or $< 4\text{ cm}$). When a beam sags by 22%, geometric non-linearities distort the small-angle math.

Setting $E = 50\text{ MPa}$ ($5.0 \times 10^7\text{ Pa}$) brings the analytical sag down to $\approx 1.76\text{ cm}$, placing the experiment strictly inside the linear Euler-Bernoulli regime.

---

### Recommended Script Adjustments

Here is the updated configuration to test in `run_phase1.py`:

1. Set `E = 50.0e6` (50 MPa).
2. Update `get_world_z` to calculate `get_tip_world_z` using segment length.
3. Apply `PhysxSchema.PhysxArticulationAPI.Apply(anchor_prim)` to the anchor.

Run this updated setup and check if the measured tip deflections converge cleanly toward the analytical baseline as $N$ increases!

# expected behavior
With the parameters set to $E = 50\text{ MPa}$ ($5.0 \times 10^7\text{ Pa}$) and the tip Z-position measured correctly at the physical tip of segment $Seg_{N-1}$, the continuous analytical prediction for tip deflection is:

$$\delta_{\text{EB}} = \frac{\rho \cdot g \cdot L^4}{2 \cdot E \cdot r^2} = \frac{700 \times 9.81 \times (0.4)^4}{2 \times (50 \times 10^6) \times (0.01)^2} = 0.01759\text{ m} \approx 1.759\text{ cm}$$

---

### The Discrete Lumped-Parameter Convergence Formula

When a continuous cantilever beam under uniform self-weight is modeled as a chain of $N$ rigid links connected by torsional springs ($K = \frac{E \cdot I \cdot N}{L}$), the exact theoretical deflection of the discrete rigid-body chain $\delta_N$ follows:

$$\delta_N = \delta_{\text{EB}} \cdot \left(1 + \frac{1}{N}\right)^2$$

As $N \to \infty$, $\left(1 + \frac{1}{N}\right)^2 \to 1.0$, and the discrete chain converges to the continuous Euler-Bernoulli solution ($\delta_{\text{EB}}$).

---

### Expected Results Summary Table

Below are the numerical values your simulation should produce across sweeps of $N$:

| $N$ | Segment Length $l$ | $K_{\text{real}}$ ($\text{N}\cdot\text{m}/\text{rad}$) | $K_{\text{sim}}$ ($S^4$ scaled) | Expected Tip Deflection ($\text{cm}$) | Euler-Bernoulli Baseline ($\text{cm}$) | Expected Error vs. $\delta_{\text{EB}}$ |
| --- | --- | --- | --- | --- | --- | --- |
| **2** | $0.200\text{ m}$ | $1.9635$ | $19,635$ | **$3.958\text{ cm}$** | $1.759\text{ cm}$ | $+125.0\%$ |
| **3** | $0.133\text{ m}$ | $2.9452$ | $29,452$ | **$3.127\text{ cm}$** | $1.759\text{ cm}$ | $+77.8\%$ |
| **5** | $0.080\text{ m}$ | $4.9087$ | $49,087$ | **$2.533\text{ cm}$** | $1.759\text{ cm}$ | $+44.0\%$ |
| **10** | $0.040\text{ m}$ | $9.8175$ | $98,175$ | **$2.128\text{ cm}$** | $1.759\text{ cm}$ | $+21.0\%$ |
| **20** | $0.020\text{ m}$ | $19.6350$ | $196,350$ | **$1.939\text{ cm}$** | $1.759\text{ cm}$ | $+10.2\%$ |
| **50** | $0.008\text{ m}$ | $49.0874$ | $490,874$ | **$1.830\text{ cm}$** | $1.759\text{ cm}$ | $+4.0\%$ |
| $\infty$ | $0.000\text{ m}$ | $\infty$ | $\infty$ | **$1.759\text{ cm}$** | $1.759\text{ cm}$ | $0.0\%$ |

---

### Key Behaviors to Look For in Your Next Run

1. **Monotonic Convergence:** Deflection must decrease smoothly as $N$ grows ($3.96\text{ cm} \to 3.13\text{ cm} \to 2.53\text{ cm} \to \dots \to 1.83\text{ cm}$).
2. **Asymptotic Convergence:** The deflection curve will level off near $1.76\text{ cm}$ for high segment counts ($N \ge 20$).
3. **No Explosions or Sudden Spikes:** With `PhysxArticulationAPI` enabled, high segment counts ($N=20, 50$) will remain stable and will not stretch or explode.

---

### Scientific Relevance for Your Thesis

The fact that small $N$ values (like $N=2$) deflect more than the continuous theoretical baseline is **not a bug**—it is a mathematically proven property of lumped-parameter spatial discretization:


$$\text{Error}(N) = \left(1 + \frac{1}{N}\right)^2 - 1 = \frac{2}{N} + \frac{1}{N^2}$$

Including this analytical relationship in your thesis proves that your digital twin correctly captures both physical beam behavior and spatial discretization limits.