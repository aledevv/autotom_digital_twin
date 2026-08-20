# Segmented Organic Branch Visuals

## Purpose

Exporter V2 separates the **physical branch discretization** from the **visible branch surface**. PhysX continues to simulate articulated rigid links, D6/fixed joints, and hidden capsule collision proxies, while the visible stem/branch surface is generated procedurally from a smooth radius profile.

The implementation supports two important visual representations:

- `skinned`: continuous UsdSkel deformation, kept as a high-quality/reference mode.
- `segmented`: realtime organic geometry, where each PhysX link owns one rigid piece of the smooth tube mesh.

The segmented representation was introduced after profiling showed that runtime UsdSkel evaluation, rather than the organic mesh topology itself, was the dominant performance cost on a full plant.

## Runtime rationale

Development measurements on a day-40 plant were approximately:

| Configuration | Observed FPS |
| --- | ---: |
| Legacy cylinder visuals | 18–20 |
| Static smooth organic mesh | 19–20 |
| Full UsdSkel skinning | ~8 |
| Shared/global Skeleton experiment | ~10 |
| Segmented organic visuals | ~20 |

These numbers are hardware- and scene-dependent, but they established an important architectural result: **the smooth tube geometry itself was inexpensive enough for realtime use, while runtime skeletal deformation was not**.

The production realtime path therefore authors rigid organic pieces directly under PhysX links. No `SkelRoot`, `Skeleton`, `SkelAnimation`, skin weights, or per-frame PhysX-to-skeleton synchronization is required in `segmented` mode.

## Data flow

```text
GroIMP branch definitions
        |
        v
resolve_vegetative_graph()
        |
        +--> BranchData / physics links / joint frames
        |
        v
build_visual_axes()
        |
        +--> VisualAxisData
        |    - botanical visual segments
        |    - physical link intervals
        |    - radius profile
        |    - attachment arcs
        |
        v
author_segmented_visual_axis()
        |
        +--> one rigid organic mesh per PhysX link
        |
        +--> local overlap at internal joints
        |
        +--> terminal leaf-junction dressing when applicable
```

## Continuous visual profile

The visible tube is parameterized by arc length `s` along a visual axis. For a straight rest-pose axis with origin `p0` and unit direction `a`, the centerline is

$$
\mathbf{c}(s)=\mathbf{p}_0+s\mathbf{a}.
$$

A ring vertex at angle `theta` is generated from two orthonormal directions `n` and `b` perpendicular to the axis:

$$
\mathbf{v}(s,\theta)=\mathbf{c}(s)+r(s)\left(\mathbf{n}\cos\theta+\mathbf{b}\sin\theta\right).
$$

`VisualProfile.radial_segments` controls the angular resolution. The default is 14 radial samples. Axial samples are primarily generated from metric spacing (`axial_spacing_m`) and are augmented locally around taper regions, botanical radius transitions, and attachment points.

### Segment taper

A `VisualSegment` can specify a start radius and an optional distal `end_radius`. The radius is interpolated with a smoothstep function rather than linearly:

$$
h(t)=3t^2-2t^3,\qquad 0\leq t\leq 1.
$$

For a segment starting at arc `s0` with length `L`, start radius `r0`, and end radius `r1`:

$$
t=\frac{s-s_0}{L},
$$

$$
r(s)=r_0+(r_1-r_0)h(t).
$$

This is used, for example, to make young petiolules progressively thinner toward their distal end.

### Smooth transitions between botanical segments

Consecutive source segments can have different nominal radii. Instead of creating a hard step at their boundary, the renderer defines a transition window around the boundary and blends the two radii with the same smoothstep function. The transition half-width is bounded by both a metric profile value and a fraction of the neighboring segment lengths:

$$
w=\min\left(w_{profile},\alpha L_{prev},\alpha L_{next}\right).
$$

The current profile uses `radius_transition_half_width_m = 0.025` and `radius_transition_max_fraction = 0.45`. This prevents short organs from receiving an unrealistically long transition while allowing structural stems to change diameter smoothly.

### Junction swelling

Attachment regions can receive a local Gaussian radius multiplier. If `sj` is an attachment arc, a simplified form is

$$
r'(s)=r(s)\left[1+A\exp\left(-\frac{1}{2}\left(\frac{s-s_j}{\sigma}\right)^2\right)\right].
$$

This provides a mild organic shoulder around branch insertion points without modifying physics or collision geometry.

## Segmented realtime representation

The continuous visual profile is sampled exactly as above, but the resulting surface is divided according to PhysX link intervals. For link `i`, with physical arc interval `[si, ei]`, only the corresponding rings are authored under that rigid body's USD path. Points are converted from world rest-pose coordinates into the link's local rest frame:

$$
\mathbf{v}_{local}=\mathbf{T}_{link,rest}^{-1}\mathbf{v}_{world}.
$$

At runtime PhysX updates the rigid link transform, and the visual mesh follows through standard USD transform inheritance. No deformation runtime is involved.

Conceptually:

```text
PhysX Link 01
└── OrganicVisual_01

PhysX Link 02
└── OrganicVisual_02

PhysX Link 03
└── OrganicVisual_03
```

### Joint overlap / tongue

A purely rigid split can expose a crack when adjacent links rotate. The segmented representation therefore lets the parent visual piece extend slightly beyond its physical end into the next piece. The overlap length is bounded by a global maximum and fractions of the adjacent link lengths:

$$
L_{ov}=\min\left(L_{max},\beta L_i,\beta L_{i+1}\right).
$$

The overlap radius is reduced along the extension so it nests inside the following piece. This does not affect collisions because the organic tube is visual-only; the physics proxy remains the capsule geometry authored by the branch physics backend.

## Terminal lateral-branch treatment

A visually problematic case occurs when a lateral structural branch ends exactly where a leaf petiole begins. A naive implementation produces a flat truncated parent surface with a separate child tube attached on top. The current implementation treats one terminal petiole as the **real continuation** of the lateral branch.

The selection rule is implemented in `adapter.py`: a vegetative petiole attached to the final physical link with `attach_frac >= 0.95` can be marked as the centered terminal continuation. The chosen petiole is placed on the parent centerline (`radial_distance = 0`) instead of receiving the normal lateral offset.

`BranchData` now carries explicit state:

```python
centered_terminal: bool = False
centered_terminal_host: bool = False
```

This replaced temporary mutation-based flags in the source dictionaries and keeps the visual-junction state attached to resolved branch data.

### Contact geometry

For the centered petiole, the first visual segment extends a short distance backward into the host branch. The buried root is narrower at its deepest point and smoothly returns to the normal petiole radius. This overlap hides small gaps caused by the two rigid surfaces meeting at an angle.

The host branch receives a shallow rounded closure at the terminal ring. The centered petiole penetrates that closure visually, so the junction reads as solid tissue rather than an open tube.

The host terminal taper is derived from the **actual visual petiole radius**, not simply from the raw CSV radius. This distinction matters because the leaf axis may have a root flare. The relevant ratio is therefore based on

```python
parent_radius = _visual_radius(parent_axis, parent_axis.total_length)
child_contact_radius = _visual_radius(child_axis, 0.0)
scale = child_contact_radius / parent_radius
```

rather than `child["radius"] / parent["radius"]`.

The match is intentionally conservative and clamped: the parent should not expand beyond its existing terminal diameter solely to satisfy the visual junction.

## Small visual terminal fork

The terminal leaf junction also receives a small **visual-only young twig**. This element is deliberately short and thin so that its rigid motion is plausible. It has no independent physics, collider, joint, or skeleton; it is authored under the terminal rigid link of the structural branch.

The twig centerline is a quadratic Bézier curve:

$$
\mathbf{B}(t)=(1-t)^2\mathbf{P}_0+2(1-t)t\mathbf{P}_1+t^2\mathbf{P}_2,\qquad t\in[0,1].
$$

`P0` begins slightly inside the parent, `P1` preserves a short forward tendency, and `P2` defines the final young-shoot direction. The surface is generated by sweeping the same ring construction used for the main organic tube around this curve.

The young twig radius tapers strongly from root to tip using two smooth regions: root-to-shoulder and shoulder-to-tip. A small procedural leaf is placed at the distal end.

### Deterministic azimuth variation

If every fake twig used the same orientation, the plant would look procedural and repetitive. The implementation therefore varies the twig azimuth around the parent axis. The real terminal petiole defines a reference radial direction. The opposite radial direction is rotated around the structural branch axis by a deterministic pseudo-random angle.

A stable hash of the parent and child IDs is mapped to

$$
\phi\in[-75^\circ,+75^\circ].
$$

The varied radial vector is

$$
\mathbf{r}_{var}=R_{\mathbf{a}}(\phi)\left(-\mathbf{r}_{petiole}\right),
$$

where `a` is the lateral-branch axis and `R_a(phi)` is a rotation around that axis. The final young-shoot direction combines forward, radial, and upward components before normalization. Because the hash is deterministic, exporting the same plant topology produces the same visual orientation on every run.

Truss and tomato terminal geometry are intentionally excluded from this terminal-fork dressing.

## High-quality UsdSkel mode

The original continuous `skinned` mode remains in the codebase and uses the same visual tube profile. Each visual axis is associated with a USD Skeleton and animated from the corresponding PhysX link transforms. The continuous deformation is visually superior during strong bending, but full-plant tests showed a substantial runtime cost in Isaac Sim 4.5.

The skinned representation should therefore be understood as a **high-quality/reference rendering mode**, while segmented organic geometry is the realtime mode.

## Main implementation files

- `core/skinning/model.py` — visual/physics data models and explicit centered-terminal flags.
- `core/skinning/adapter.py` — graph resolution, terminal petiole selection, and centerline attachment.
- `core/skinning/axis.py` — construction of continuous visual axes from physical branch members.
- `core/skinning/mesh.py` — common continuous radius profile, sampling, tube generation, shared plain-mesh helpers, and UsdSkel authoring.
- `core/skinning/visual_modes.py` — segmented/static/rigid visual representations, per-link organic mesh extraction, overlaps, and terminal host closure.
- `core/skinning/terminal_fork.py` — leaf-only decorative young twig, Bézier geometry, and deterministic azimuth variation.
- `core/skinning/builder.py` — backend orchestration and terminal visual-radius matching.
- `core/skinning/runtime.py` — PhysX-to-UsdSkel synchronization used only by runtime-skinned modes.

## Running

Realtime organic visuals are the default when the skinned backend is selected:

```bash
./run_mainV2.sh --day 40
```

Equivalent explicit invocation:

```bash
./run_mainV2.sh \
  --day 40 \
  --branch-backend skinned \
  --skinning-visual-mode segmented
```

Continuous UsdSkel reference:

```bash
./run_mainV2.sh \
  --day 40 \
  --branch-backend skinned \
  --skinning-visual-mode skinned
```

## Design invariant

The central invariant is that **visual improvements must not redefine the physical plant**. The segmented mesh, overlap tongues, terminal dome, centered contact dressing, and fake young twig are rendering constructs. The validated PhysX rigid bodies, joints, collision proxies, masses, and mechanical parameters remain the source of simulation behavior.
