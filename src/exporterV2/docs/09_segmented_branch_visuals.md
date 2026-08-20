# Vegetative Visual Modes

Exporter V2 separates vegetative physics from visible organic surfaces. PhysX always owns the rigid links, joints, and collision proxies. A selected visual mode determines how the same resolved plant is rendered.

## Supported Modes

| Mode | USD representation | Intended use |
| --- | --- | --- |
| `segmented` | One local organic mesh per rigid link | Current full-plant default |
| `skinned` | Continuous tube, per-axis UsdSkel, runtime synchronization | High-quality deformation reference |
| `static` | Continuous world-space mesh | Geometry-cost benchmark |
| `rigid-single` | Continuous local mesh on one-link axes; other axes use the skinned path | Focused runtime diagnostic |

The legacy branch backend remains separately available and authors cylinder visuals together with its physical chains.

Development profiling on the same day-40 plant found that static and segmented organic geometry performed close to the legacy visual baseline, while full runtime UsdSkel evaluation was substantially heavier. Measurements depend on hardware and renderer, but this result is why `segmented` remains the realtime default and `skinned` remains available rather than being removed.

## Shared Visual Profile

`build_visual_axes()` groups resolved vegetative branches into continuous botanical axes. Each `VisualAxisData` records:

- source segments and their radii;
- physical link intervals and rest transforms;
- metric sample spacing;
- taper and radius-transition settings;
- attachment arcs used for local junction swelling.

For straight rest-pose centerline origin `p0`, axis `a`, and arc length `s`:

$$
\mathbf{c}(s)=\mathbf{p}_0+s\mathbf{a}.
$$

A ring point uses radius `r(s)` and an orthonormal frame `n`, `b`:

$$
\mathbf{v}(s,\theta)=\mathbf{c}(s)+r(s)
\left(\mathbf{n}\cos\theta+\mathbf{b}\sin\theta\right).
$$

Botanical taper, smooth radius transitions, petiolule root treatment, and attachment swelling are calculated once in `mesh.py` and reused by all organic representations.

## Segmented Representation

For each physical link, `visual_segmented.py` selects the rings inside that link's arc interval and converts points into the link's rest-local frame. Standard USD transform inheritance then makes the mesh follow PhysX without a skeleton runtime.

```text
Rigid link
├── collision proxy
└── OrganicVisual_NN
```

A short tapered tongue extends a parent visual into the next link to hide cracks during joint rotation. The tongue is visual-only and does not add collision shapes, rigid bodies, joints, or prim-level physics.

## Terminal Leaf Junctions

A vegetative petiole attached near the end of a structural lateral can be marked as its centered terminal continuation. The rendered host tip narrows toward the actual petiole contact radius, while the petiole root extends slightly inside the host. This keeps the junction visually closed without changing attachment physics.

`terminal_fork.py` can add a small secondary young shoot and leaf under the host's final rigid link. Its azimuth is deterministic from branch identifiers. Truss and tomato terminals are excluded from this dressing.

Leaf blades are separate double-sided meshes with a shared leaf material. Their geometry and material do not alter leaf collision proxies or branch topology.

## Materials

Visual meshes bind shared materials from `core/usd/materials.py`:

- `/World/Looks/TomatoStem` for vegetative organic surfaces;
- `/World/Looks/TomatoLeaf` for leaf blades;
- maturation buckets below `/World/Looks/TomatoFruit` for tomatoes.

Realtime generation uses `UsdPreviewSurface`. Optional OmniSurface presets remain available for isolated look development and are not selected by the full-plant default.

## Implementation Map

- `builder.py`: visual-mode selection and orchestration.
- `adapter.py`: vegetative graph resolution and centered-terminal classification.
- `axis.py`: continuous visual-axis construction.
- `mesh.py`: shared profile, tube samples, plain mesh authoring, and `skinned` mode.
- `visual_segmented.py`: per-link organic meshes and overlap tongues.
- `visual_static.py`: static continuous benchmark.
- `visual_rigid.py`: one-link direct attachment mode.
- `visual_modes.py`: compatibility re-exports.
- `terminal_fork.py`: terminal leaf-only dressing.
- `leaf_blade.py`: procedural double-sided leaf blades.
- `runtime.py`: PhysX-to-UsdSkel synchronization.

## Running

```bash
# Current default
./run_mainV2.sh --day 40

# Explicit modes
./run_mainV2.sh --day 40 --branch-backend skinned --skinning-visual-mode segmented
./run_mainV2.sh --day 40 --branch-backend skinned --skinning-visual-mode skinned
./run_mainV2.sh --day 40 --branch-backend skinned --skinning-visual-mode static
./run_mainV2.sh --day 40 --branch-backend skinned --skinning-visual-mode rigid-single

# Preserved legacy backend
./run_mainV2.sh --day 40 --branch-backend legacy
```

The invariant across these modes is physical equivalence: visual implementation changes must not redefine rigid bodies, joints, collision proxies, masses, detachment, or filters.
