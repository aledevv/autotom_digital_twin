# Experimental main rank-6 standard truss

`main-rank6-standard` is an opt-in v2.3 integration of the historical reference,
not a validated full-plant preset. Default PlantState construction is unchanged.

## Current gate

Template equivalence at the reference pose and export audits pass. The reduced
v2.3 scene (full canonical stem plus direct trusses 6–10, 110 bodies / 40 fruits)
FAILS its first rest screening: five fruit breaks are recorded at 0.20000001 s.
The report's first failing fruit is rank 10, lat_3_L. Stop here: no lateral/full
plant simulation or manual GUI acceptance has been performed for this integration.
No solver retuning or geometry adjustment follows this failure.

The older main-derived five-truss reference remains separately preserved at
`b5272ee` and subsequent manual-test checkpoints; do not confuse it with this
PlantState-anchored integration. Loaded solver/scene properties and corresponding
truss masses, inertia and COMs match the older reference. Root attachment frames,
positions and the PlantState stem differ; root rachis tilt is 72 degrees from
vertical in the main reference versus 75 degrees in the PlantState fixture; the underlying dynamic cause has not
been established. Physical attributes of the final export match the screened USD.

## Reproduce the export

From the repository root:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache ./run_mainV2.sh \
  --day 160 --debug-profile full --allow-experimental-fruit-physics \
  --experimental-truss-preset main-rank6-standard \
  --experimental-truss-fixture direct --generate-only \
  --output /tmp/v23-standard-direct.usda
```

`--experimental-truss-fixture` accepts `direct`, `lateral`, `full` (default).
These select scenes before vegetation mass aggregation. `lateral` selects the
first truss by numeric rank and ID, retaining its full ancestor chain; it has
not passed its simulation gate. `full` retains vegetation and all nondegenerate
standardized trusses, and has not passed its simulation gate either.

The generator writes the USD, manifest and `<usd>.standard.json`. The regular
launcher passes this resolved configuration to the native mouse monitor; GUI
runs have no automatic time deadline. Each launch from this sidecar writes to
a fresh `<usd-stem>-runs/run-*` directory. `--headless --duration 20` requests a
screening run when simulation is appropriate. An unexpected physical failure
still stops the monitor. Do not infer stability from successful USD export.

## Template and compatibility

`main_rank6.usda` is a small, checked-in historical truss fixture with its joint,
collider and visual definitions. `main_rank6.json` pins its checksum, source USD
hash, branch definitions and loaded body properties. Neither generation nor tests
require the archived main checkout or local experiment USDs.

The template is re-expressed at the first canonical rachis frame and attached
to the original PlantState parent link. Internal relative body frames and joint
properties are retained. The exporter bypasses the skinned branch constructor
only for tagged standard trusses. Dedicated count/pose checks coexist with the
unchanged vegetation checks. No runtime joint deletion/reconstruction is used.

The preset has four rachis segments, eight pedicels and eight fruits. It replaces
original fruit counts, masses and dimensions and original internal truss geometry;
full source definitions and generated body mappings are recorded in manifest
metadata. Input PlantState is never mutated. Degenerate trusses remain excluded.
The existing v2.3 GLOBAL_SCALE=2 conversion is explicit and checked; sphere
radii are stored at half the final world radius in adapter definitions, while
the baked historical USD uses metres directly. Fruit mass remains 74.708532 g
per truss and spherical density 1000 kg/m3; support density is the artificial
main reference value 20000 kg/m3.

Resolved runtime: TGS/GPU, 60 Hz, articulation 32/4, fruit 32/1, native joint
mouse coefficient 10, excluded fruit FixedJoints break at 6 N from startup.
Conflicting cadence, drive, locking and terminal solver overrides are rejected.
Runtime-side expected mass/inertia/COM checks cover all 100 standard bodies in
the direct fixture. This is limited to day 160 and remains explicitly experimental.

Verification: 65 tests in the initial targeted/regression batch passed; the final
six preset tests also passed after adding the export/runtime consistency case.
Wrapper generate-only and shell syntax checks pass. Heavy evidence is local under
`artifacts/detachable_fruit_v2/standard-integration/`; compact report is adjacent.
