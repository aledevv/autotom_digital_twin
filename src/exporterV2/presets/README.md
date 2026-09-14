# Experimental main rank-6 standard truss

`main-rank6-standard` is an opt-in v2.3 integration of the historical reference,
not a validated full-plant preset. Default PlantState construction is unchanged.

## Current decision (2026-09-14)

The user chose to retain direct-stem trusses only and stop the lateral-truss search.
Use the `direct` fixture as the current reference. The `lateral` and `full` options
remain diagnostic, not accepted configurations; this decision does not alter
exporter defaults or implement a new full-plant exclusion filter.
See the [experiment history and final decision](../../experiments/detachable_fruit_v2/DECISION_AND_HISTORY.md).

## Recorded integration gate

Template equivalence and export audits pass. The initial reduced integration
(full canonical stem plus direct trusses 6–10, 110 bodies / 40 fruits) failed at
0.20 s with five spontaneous breaks. The canonical attachment's transverse axes
inverted the reference template: fruit centers were above their attachments.

The corrected transformation retains the canonical origin and longitudinal axis,
but preserves the reference roll relative to gravity. All 40 fruit centers now
lie below their attachments. Internal geometry and physics are unchanged.
The corrected scene passes 20 s of functional headless screening with no break
events. Strict numerical velocity thresholds still produce advisories; this is
not final stability acceptance. Corrected GUI review is positive: four user-targeted breaks on four trusses,
no reported errors, 31.02 steady FPS. The session lasted 22.95 simulated seconds
(real-time factor 0.519), so the full 60 s manual cycle remains pending. A separate 60 s headless
confirmation passed without errors or break events.

The older main-derived five-truss reference remains separately preserved at
`b5272ee`. The v2.3 parent stem, attachment positions and inclination still differ
from that reference. See the adjacent compact report for before/after evidence.

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
first truss by numeric rank and ID, retaining its full ancestor chain. The
selected `Truss_r5_o0_g421757` failed screening after 0.333 s with a spontaneous
lat_0_L fruit break, without mouse input. Subsequent lateral GUI trials and bounded controls failed acceptance; see the
linked final decision. The cause remains undetermined. `full` retains vegetation and all nondegenerate
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

The template uses the first canonical rachis origin and axis, with main roll
relative to gravity, and is attached
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
nine preset tests also passed including direct/lateral export/runtime consistency and orientation regression cases.
Wrapper generate-only and shell syntax checks pass. Heavy evidence is local under
`artifacts/detachable_fruit_v2/standard-integration/`; compact report is adjacent.
