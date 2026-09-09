# Realistic leaflet shapes in PlantState V2

The `--day` PlantState pipeline uses **Gaussian** leaflet outlines by default.
`i3` selects the alternative role-conditioned generator; `legacy` explicitly
selects the previous Merlice profile for regression/debugging. All three use
the same plant supports, placement, materials, and physics. The static demo
without `--day` retains its previous behavior.

## Run and configure

From the repository root:

```bash
./run_mainV2.sh --day 50
./run_mainV2.sh --day 50 --leaf-shape-backend i3
./run_mainV2.sh --day 50 --leaf-shape-backend legacy
./run_mainV2.sh --day 50 --leaf-shape-seed 123
./run_debugV2.sh --day 50 --organ leaves --leaf-shape-backend gaussian
```

The central configuration is `src/exporterV2/profiles/leaf_shape.yaml`. Edit
`backend` there or pass `--leaf-shape-config /absolute/path/leaves.yaml`.
Overrides use this precedence: CLI backend/seed, selected YAML, default YAML.
A custom YAML can override just a subset, for example:

```yaml
backend: i3
seed: 42
timeout_seconds: 600
generator:
  neighbours: 5
```

The complete default file includes the model and bank settings for both
methods. Relative resource paths resolve against the submodule's
`src/tomato_leaf_generator/resources`, not the shell working directory.
The same flags work with `uv run python -m exporterV2 --day 50`.
`--generate-only` produces and audits the USD without starting Isaac Sim.
Use distinct `--output` paths to retain comparisons between methods.

## Runtime setup

Initialize the external repository if necessary:

```bash
git submodule update --init --recursive
```

AutoTom remains on Python 3.14. `triangle==20250106` has no Python 3.14 wheel,
and the external package declares Python 3.12 support. A dedicated uv project
under `leaf_shapes/runtime` locks Python 3.12-compatible NumPy and Triangle.
The main command automatically prepares/reuses its environment. The first
run needs network access to download missing dependencies/interpreter; once
prepared, `UV_OFFLINE=1 ./run_mainV2.sh --day 50` works without package access.
No manual environment activation or change to Isaac's packages is needed.

The worker imports the production `create_leaf_shape_generator` API directly
from the submodule, through one centralized bootstrap. It uses only the
shape APIs, so unrelated external image-processing and deformation packages
are not installed. It does not import `pxr` or start `SimulationApp`.

For standalone scripts that generate inside **Isaac's Python**, Triangle is
an explicit separate environment requirement:

```bash
"$HOME/isaacsim/python.sh" -m pip install triangle
```

Installing Triangle alone does not bootstrap Isaac's USD extensions. The
production worker avoids that issue: shape generation is separate from USD
authoring and Isaac loading.

## Individual and reproducible shapes

Each leaflet receives a stable 63-bit seed from SHA-256 of the global seed,
plant ID, canonical structural axis ID and explicit role. The hash policy is
versioned as `autotom-leaf-seed-v1`; Python's randomized `hash()` is not used.
The backend is excluded from that hash so matching leaflets receive matching
seeds in Gaussian/I3 comparisons.

The same input snapshot, configuration, resources and locked runtime produce
the same set on every run, regardless of request order. Changing the global
seed gives a new set. Leaflets have individual seeds, rather than sharing
one copied outline; mathematical uniqueness of all possible draws is not
guaranteed. Cross-snapshot identity is only preserved if structural IDs remain
the same; day-to-day biological tracking is outside this integration.

The worker loads the model/bank once and generates all requested leaflets in
one process. Temporary JSON files carry contours, triangles and provenance;
they are removed after the export. There is no persistent geometry cache.
Failures are explicit, including the failing role/seed/identity when available.
Timeout or interruption terminates the worker process group. There is no
automatic Gaussian-to-I3 or realistic-to-legacy fallback.

## Geometry and attachment

Both generators provide canonical CCW contours with biological landmarks
base `(0,0)` and tip `(0,1)`. The full 1024-point boundary is triangulated with
Triangle option `p`, preserving boundary segments without simplification,
midrib insertion or extra refinement. The original generated contours are
never mirrored based on left/right role.

The existing V2 path computes physical blade length from petiolule length.
That length scales both contour dimensions uniformly; the learned outline
determines its own width. Existing length clamps and global scale semantics
are retained in each exporter path. This does not introduce an area-matching
rescaling or a fixed 6 cm production length.

The blade retains static arch, tip sag and fold parameters from `leaf_blade`.
The approximate fold uses `abs(x)/max(abs(x))` and clamped normalized
longitudinal position. These are visual parameters, not newly validated
biological measurements. Triangulation and deformation remain separate.

The biological origin is transformed to the petiolule tip in its existing
local attachment frame. It is a semantic landmark, not a fixed vertex index;
sampled contours need not contain that exact point. Face winding is adjusted
to the existing side/forward basis without reflecting the shape. Blade meshes
remain visual-only and use the existing tomato leaf material.

Each blade records `autotom:leafShapeBackend`, `leafletRole`, `leafShapeSeed`,
`leafShapeStructuralId`, `leafShapeProvider`, `leafShapeProvenance`, and local
`leafShapeBase`/`leafShapeTip`. The USD default prim and JSON manifest retain
the configuration, complete per-leaf provenance, contour hashes and runtime
versions. Model/bank checksums are included in generator provenance. Generation
wall time is printed separately so it cannot change reproducible USD/manifest
contents.

## Validation and follow-up

Run focused checks with:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run pytest -q src/exporterV2/tests/test_real_leaf_shapes.py
```

Tests cover both generators/all roles, separate-process determinism, distinct
sampled contours, configuration precedence, failures/timeouts, local attachment,
materials, physical petiolules, four visual modes and unchanged non-blade prims.
Compare full-plant outputs with identical snapshots, seed, physics and camera.
Generated artifacts should remain local; mesh counts or file size do not
establish runtime FPS. Final GUI review must inspect the base connection,
orientation, width/taper and static fold for both methods.

TODOs for separate work:

- Move leaflet mesh/deformation orchestration out of `core/skinning`, potentially
  into `core/leaf_geometry`; the isolated generation code already lives separately.
- Add a constrained midrib before refining the longitudinal fold model.
- Replace visual static curvature with the future fitted empirical provider.
- Clean up external package installation/bootstrap after this integration.
- Measure controlled FPS before considering contour simplification or refinement.
