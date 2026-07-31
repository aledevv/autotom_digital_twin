Good news first: the physics core is implemented correctly and matches the paper. Here's the breakdown.

## What matches the paper correctly

**1. The governing equation.** `threepoint_theory.py` uses exactly Anisimov's eq. (1)/(2):
δ = FL³/(48EI), I = π(D⁴−d⁴)/64 (solid case: πR⁴/4). This is the right formula whether you call it "inverse" or "straight" TPBT — the paper explicitly confirms (§3.2.1, Fig. 3b) that iTPBT and sTPBT give statistically identical E values, since they're mechanically the same test with the roles of "moving part" and "load sensor" swapped. So simulating a classical force-at-center / simply-supported-ends setup (which is what `run_threepoint.py` actually does) is a legitimate stand-in for the inverse device — you're not simulating the literal mechanism, but you don't need to.

**2. Boundary conditions.** `create_simple_support` locks translation and locks axial torsion but frees rotY/rotZ — that's a genuine pin/simple support, not a clamp. This is the critical detail the paper stresses (§1: shear/support type matters, and a clamped support would change the denominator 48→192). Good.

**3. SDR criterion.** With your current geometry, SDR = L/(2R) = 28.7 cm / 1 cm ≈ 28.7 ≥ 20 — satisfies the Fok & Smart (1993) criterion the paper cites (ref [15]/[37]) to avoid shear-induced underestimation of E.

**4. Discretization scheme.** Modeling the continuous beam as N rigid links joined by torsional springs with stiffness K = EI/l_link is a standard pseudo-rigid-body approximation of Euler–Bernoulli bending. It converges to the continuum solution as N_LINKS increases — the `sim/theory` ratio printed at the end of `run_threepoint.py` is exactly the right sanity check for this. Just make sure you actually look at that ratio (and ideally re-run at N=10/20/40 to confirm convergence) before trusting E derived from real tomato data.

## Issues I found

**1. Stale docstring numbers (real bug, values are wrong).** The header of `run_threepoint.py` claims:
```
E=35 MPa  → δ ≈ 4.51 mm
E=150 MPa → δ ≈ 1.05 mm
```
I recomputed with your actual `TrunkConfig` (N=20, span L=28.69 cm, R=5 mm):

| E | kB (theory) | δ at F=0.5 N |
|---|---|---|
| 35 MPa | 34.92 N/m | **14.32 mm** |
| 150 MPa | 149.66 N/m | **3.34 mm** |

That's ~3× off from the comment. The formula and code are self-consistent (`_print_geometry_summary()` will print the *correct* 14.3 mm at runtime), it's just the docstring that's out of date — probably left over from an earlier geometry (shorter span or bigger radius). Not dangerous, but worth fixing so you don't misjudge results against a stale expectation.

**2. That 14 mm deflection is a real concern, not just cosmetic.** 14.3 mm over a 28.7 cm span is ~5% of span — getting into the range where small-angle/linear assumptions (both Euler-Bernoulli and your per-joint torsional-spring linearization) start to lose accuracy, especially for the 35 MPa (primary-tissue) case, which is the regime most relevant to a young tomato stem. Two options: cap `FORCE_STEPS_N` lower for soft-tissue runs, or use `N_LINEAR_POINTS` to only regress the first few (small-deflection) steps rather than all 10.

**3. Force isn't applied at exact midspan.** `center_link_index() = N_LINKS // 2 = 10` puts the load at x = 15.10 cm, while true midspan is 14.35 cm — a 2.6% span offset (artifact of using an even N_LINKS, so there's no link exactly at center). Small, probably negligible, but if you want it exact, use an odd N_LINKS or split the force across the two central links.

**4. Duplicated geometry constants.** `plot_threepoint.py` re-hardcodes `N_LINKS`, `HEIGHT`, `GAP`, `RADIUS` instead of importing `TrunkConfig` from `generate_threepoint_usda.py`. This is exactly the failure mode that produced bug #1 above — if you ever tune the geometry for real tomato dimensions, it's easy to update one file and forget the other, and your theory overlay in the plot will silently be wrong. Worth importing `TrunkConfig` directly.

**5. Settling time vs. damping.** `DAMPING_RATIO = 0.2` is deliberately underdamped, and each force step only holds for 120 steps (0.25 s @ 480 Hz) before you sample. For the softer E regimes (which have lower natural frequency), 0.25 s may not be enough for the oscillation to settle, which would add noise to your "settled" F–δ points. Worth logging a few extra steps and eyeballing that the transient panel (panel 1 in the plot) actually looks flat before each sample point, not still ringing.

## Bottom line
The mechanics and equations are right, and the setup is honestly quite careful (correct free-rotation supports, correct I, correct SDR check, kB-slope-based E extraction like the paper recommends over single-point estimates). The main things to fix before trusting numbers on real tomato stem geometry: correct the stale docstring, reconsider force range so deflections stay small relative to span, and centralize the geometry constants so plot and sim can't drift apart.

# Fixinign suggestions

Here are concrete patches for each issue, file by file.

## 1. Fix the stale docstring (`run_threepoint.py`)

Rather than hardcoding numbers that drift, compute them from `TrunkConfig`/`BioConfig` at import time so the docstring can never go stale again:

```python
# Replace the static docstring block with a runtime-computed one.
# Keep the prose docstring, but drop the hardcoded "Expected deflection" lines
# and let _print_geometry_summary() (which already exists) be the source of truth —
# it already prints δ theo for the last force step. Just extend it to also
# print δ at each reference E:

def _print_geometry_summary():
    L   = TrunkConfig.total_span()
    I   = second_moment_of_area(TrunkConfig.RADIUS)
    SDR = span_diameter_ratio(L, TrunkConfig.RADIUS)

    print("\n\033[1;36m=== Three-Point Bending Test ===\033[0m")
    print(f"  N_LINKS  = {TrunkConfig.N_LINKS}")
    print(f"  Span L   = {L*100:.1f} cm")
    print(f"  Radius   = {TrunkConfig.RADIUS*1000:.1f} mm")
    print(f"  SDR      = {SDR:.1f}  {'✅' if SDR >= 20 else '⚠️  < 20'}")
    print(f"  I        = {I:.3e} m⁴")

    for label, E in [("35 MPa (primary)", 3.5e7), ("150 MPa (mature)", 1.5e8)]:
        kB_theo = structural_stiffness(E, L, I)
        d_theo  = theoretical_deflection(FORCE_STEPS_N[-1], L, E, I)
        print(f"  --- {label}: kB={kB_theo:.4f} N/m, "
              f"δ(F={FORCE_STEPS_N[-1]}N)={d_theo*1000:.2f} mm")
    print()
```
This way the "expected deflection" line is always derived from the live geometry, and you can delete the two stale lines from the module docstring at the top entirely (or replace them with "see `_print_geometry_summary()` output at runtime").

## 2. Keep deflections small (`run_threepoint.py`)

Two changes: cap the force range so max deflection stays in the small-angle regime, and add a runtime guard so you notice if it doesn't.

```python
# Instead of a fixed force list going all the way to 0.5 N regardless of E,
# scale the force range to target a max deflection-to-span ratio (e.g. 2%).

TARGET_MAX_DEFLECTION_RATIO = 0.02   # δ_max / L, keeps small-angle assumption valid

def build_force_steps(E_estimate: float, n_steps: int = 10) -> list[float]:
    """Auto-scale the force ramp so max deflection stays within
    TARGET_MAX_DEFLECTION_RATIO of the span, for the given E."""
    L = TrunkConfig.total_span()
    I = second_moment_of_area(TrunkConfig.RADIUS)
    delta_max = TARGET_MAX_DEFLECTION_RATIO * L
    F_max = structural_stiffness(E_estimate, L, I) * delta_max
    return [round(F_max * (i + 1) / n_steps, 5) for i in range(n_steps)]

FORCE_STEPS_N = build_force_steps(BioConfig.YOUNG_MODULUS)
```

For E=35 MPa this gives F_max ≈ 34.92 × (0.02×0.2869) ≈ 0.020 N (vs. 0.5 N before), keeping δ_max ≈ 5.7 mm — still comfortably linear. It also means you don't need to hand-tune the force list every time you change geometry or target tissue.

Add a guard in `run_simulation_test` right after each sample, so you get an explicit warning instead of silently trusting a large-deflection point:

```python
        if delta_m > TARGET_MAX_DEFLECTION_RATIO * TrunkConfig.total_span() * 1.5:
            warn(f"  δ={delta_mm:.2f} mm exceeds small-deflection budget "
                 f"— consider lowering FORCE_STEPS_N or excluding this point from the fit.")
```

## 3. Center the applied force exactly (`generate_threepoint_usda.py` + `run_threepoint.py`)

Cleanest fix: use an odd `N_LINKS` so there's a genuine center link.

```python
class TrunkConfig:
    N_LINKS  = 21   # odd → exact center link exists (was 20)
    ...
    @classmethod
    def center_link_index(cls) -> int:
        """0-based index of the central link (exact center for odd N_LINKS)."""
        assert cls.N_LINKS % 2 == 1, "N_LINKS should be odd for an exact center link"
        return cls.N_LINKS // 2
```

If you'd rather keep N=20 (e.g. because you've already calibrated joint spacing against real tomato internode length), split the load across the two center-adjacent links instead of moving to odd N:

```python
# In run_simulation_test(), instead of a single center_prim:
center_idx = TrunkConfig.center_link_index()          # 10
left_path  = f"/World/Stem/Link_{center_idx:02d}"     # Link_10
right_path = f"/World/Stem/Link_{center_idx+1:02d}"   # Link_11
left_prim, right_prim = RigidPrim(left_path), RigidPrim(right_path)
left_prim.initialize(); right_prim.initialize()

# ... in the force loop, apply half the force to each:
half_force_vec = (FORCE_DIRECTION * force_val / 2.0).reshape(1, 3)
left_prim.apply_forces(forces=half_force_vec, is_global=True)
right_prim.apply_forces(forces=half_force_vec, is_global=True)
# and average their Z positions for the deflection measurement
```

The odd-N_LINKS route is simpler and I'd default to that unless you have a specific reason to keep 20.

## 4. Stop duplicating geometry constants (`plot_threepoint.py`)

Import directly instead of redefining:

```python
# Replace this block:
# N_LINKS = 20
# HEIGHT  = 0.015
# GAP     = 0.0001
# RADIUS  = 0.005
# L       = (N_LINKS - 1) * (HEIGHT + GAP)
# I       = second_moment_of_area(RADIUS)
# SDR     = span_diameter_ratio(L, RADIUS)

from generate_threepoint_usda import TrunkConfig, BioConfig

L      = TrunkConfig.total_span()
I      = second_moment_of_area(TrunkConfig.RADIUS)
SDR    = span_diameter_ratio(L, TrunkConfig.RADIUS)
RADIUS = TrunkConfig.RADIUS   # keep for the title string that formats radius in mm
```

Note: `generate_threepoint_usda.py` imports from `pxr`/`omni.usd` at module scope only inside functions that need Isaac Sim (`build_stage` etc.) — but check that the top-level `from pxr import ...` doesn't fire on plain import. Looking at your file, the `from pxr import Usd, UsdGeom, Gf, UsdPhysics, Sdf` line is at module top level, so importing `generate_threepoint_usda` from `plot_threepoint.py` (which explicitly says it "works without Isaac Sim") will fail outside the Isaac Sim Python environment. Two options:
- Move the `pxr` import inside `build_stage()`/the USD-writing functions so the module is importable standalone, or
- Just duplicate `TrunkConfig` as a plain dataclass in a third shared file (e.g. `threepoint_config.py`) with zero Isaac Sim deps, imported by both `generate_threepoint_usda.py` and `plot_threepoint.py`.

The second option is cleaner given your stated goal of `plot_threepoint.py` running outside Isaac Sim — I'd pull `TrunkConfig`, `BioConfig`, and `PhysicsConfig` into a new `threepoint_config.py` alongside `threepoint_theory.py`, and have both `generate_threepoint_usda.py` and `plot_threepoint.py` import from there.

## 5. Settle before sampling, don't just count steps (`run_threepoint.py`)

Replace the fixed 120-step hold with a velocity-based settling check:

```python
SETTLE_VELOCITY_THRESHOLD = 1e-4   # m/s, "at rest" cutoff
MAX_HOLD_STEPS = 400               # safety cap so a non-converging case doesn't hang

def hold_until_settled(my_world, center_prim, force_vec, render):
    prev_z = None
    for step in range(MAX_HOLD_STEPS):
        center_prim.apply_forces(forces=force_vec, is_global=True)
        my_world.step(render=render)
        pos, _ = center_prim.get_world_poses()
        z = float(np.squeeze(pos)[2])
        if prev_z is not None:
            vz = abs(z - prev_z) / my_world.get_physics_dt()
            if vz < SETTLE_VELOCITY_THRESHOLD and step > 20:  # min steps to avoid false-early-exit
                return step + 1
        prev_z = z
    warn("Did not settle within MAX_HOLD_STEPS — result may be noisy.")
    return MAX_HOLD_STEPS
```

Then in the force loop, replace the fixed `for hold_i in range(STEP_HOLD_SIM_STEPS):` with a call to `hold_until_settled(...)` and log how many steps it actually took — that number itself is diagnostic (if it's consistently hitting `MAX_HOLD_STEPS`, your damping ratio is too low for that stiffness regime).

---

**Suggested order to apply these:** #4 first (shared config module — makes the other fixes easier to keep in sync), then #3 (geometry), then #1 (docstring, now trivial since it reads from the shared config), then #2 and #5 (simulation behavior). Want me to just write out the full patched files so you can diff them directly, or is this enough to apply by hand?