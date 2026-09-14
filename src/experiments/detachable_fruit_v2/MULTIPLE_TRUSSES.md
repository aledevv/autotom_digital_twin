# Multiple direct trusses on the main reference stem

2026-09-14. User accepted the corrected-size full rank-6 truss, then requested
more trusses on the same tree to investigate scaling of instability.

The archived main builder regenerates the source USD from day-160 input. Keep
the complete original stem, selected direct trusses, their pedicels and fruits;
remove other vegetation before building. No cloning or reanchoring. Main and
groPy branches remain unchanged.

Reference settings: TGS/GPU, 60 Hz, articulation 32/4, fruit 32/1, native joint
mouse coefficient 10, break force 6 N from startup, original drives and support
density 20000 kg/m3. Fruit mass is unchanged; spherical dimensions and inertia
are corrected to 1000 kg/m3, preserving each attachment. Support density is an
artificial reference, not a biological estimate.

| Scene | Direct ranks | Bodies | Fruits | Headless rest outcome |
| --- | --- | --- | --- | --- |
| Two trusses | 6, 7 | 50 | 16 | Passed 20 simulated seconds, no breaks/errors |
| Five trusses | 6, 7, 8, 9, 10 | 110 | 40 | Spontaneous break at 0.20000001 s; screening stopped |

The first failing joint is rank 10, pedicel lat_3_R, fruit fixed attachment.
This is an actual JOINT_BREAK without applied input, not a process freeze.
Isaac exited 1 after the failed screening; no longer-duration collapse was tested.
Both static audits reported no errors. The 30 retained bodies in the two-truss
scene match the accepted GUI's loaded masses, inertias and COMs (rtol 1e-5,
atol 1e-12). Exact selections, hashes and first failure are in
`multiple_trusses_results.json`; heavy evidence is local under
`artifacts/detachable_fruit_v2/multiple-trusses/`.

Increasing this selection changes load, contact opportunities and coupling as
well as body count. Ranks 9 and 10 both retain main's original attachment to
trunk link 10. Neither observation establishes the cause of failure. A useful
next isolated comparison is rank 10 alone versus the five-truss scene before
attributing the result to number alone.

The two-truss GUI was opened in `gui-freeze-zwh4aey8`, without an automatic
closing deadline. Manual interaction acceptance and GUI FPS are pending.

Repeat the manual test from the repository root (fresh evidence directory):

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --source-case artifacts/detachable_fruit_v2/multiple-trusses/two-screen \
  --coherent-fruit --support-density 20000 --fruit-break-force 6
```

Builder `--truss-count 2` retains two consecutive direct ranks from `--rank 6`;
`--truss-count 0` retains all remaining direct ranks. Scene preparation now
supports `--coherent-fruit` before screening. GUI launcher `--source-case`
accepts another screened reference case with its loaded mass/inertia report.

## Isolated rank-10 follow-up

The user approved the isolated comparison. Rebuilt rank 10 alone on its original
full stem, retaining all eight fruits: 30 bodies, unchanged runtime and drives.
Static audit passed. The same lat_3_R fruit joint broke spontaneously at
0.20000001 simulated seconds, exactly the first-failure time of the five-truss
case. Screening stopped there (Isaac exit 1); this does not establish a GUI crash.

Compared all attributes and relationships on 118 retained stem/rank-10 prims
against the five-truss USD. The only difference is the absent collision-filter
relationship to removed rank 9; the trunk filter remains. No retained physical
property or attachment was changed.

The other four trusses are therefore unnecessary for this first failure. This
does not exclude additional multi-truss problems later. Next diagnosis should
compare rank 10 against the accepted rank 6, including their geometry, joints,
loads and attachment to the stem, before changing parameters. No failing GUI
was launched and no physical model correction was applied in this follow-up.

## Five trusses at 120 Hz

Follow-up after the rank diagnosis: change only cadence to 120 Hz, keeping the
original fruit masses and corrected geometry, TGS/GPU, iterations 32/4 and 32/1,
6 N break force and original drives. All 110 loaded bodies retain masses,
inertias and COMs relative to the five-truss 60 Hz case (rtol 1e-5, atol 1e-12).
The fresh headless case `five-120hz` passes 20 simulated seconds with no breaks
or functional errors; Isaac exits 0. Strict velocity advisories remain diagnostic.

GUI opened as `gui-freeze-4owvne26`. The launcher now reads physics Hz from the
prepared configuration instead of hardcoding 60; rendering remains scheduled
at 60 per simulated second. No automatic manual-session deadline. User feedback
and full-cycle GUI performance acceptance are pending.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --source-case artifacts/detachable_fruit_v2/multiple-trusses/five-120hz \
  --coherent-fruit --support-density 20000 --fruit-break-force 6
```

### Observed GUI outcome (same launch)

The manual run ended at 5.391667 simulated seconds / 18.43 loop wall seconds.
Three JOINT_BREAK events were associated with the selected native drag (rank 8
lat_3_R, rank 8 lat_3_L, rank 7 lat_3_L). Then rank 10 lat_0_R broke outside the
selected fruit drag; the monitor stopped with physical_or_validation_error.
Do not call this a spontaneous-at-rest event: interaction had already occurred.
User assessment and the precise relationship to the last gesture remain pending.

Measured mean GUI FPS was 17.55; after the 5 simulated-second cutoff, 17.39 FPS
with a very short remaining window (about 0.39 simulated seconds). Real-time
factor 0.2925. This is an early failed interaction trial, not a completed minute
or a reliable long-duration steady-performance estimate. It fails the current
functional/performance acceptance. Details: `five_trusses_120hz_gui.json`.
120 Hz alone is therefore not an accepted solution despite the successful rest
screen. The window shutdown was initiated by the monitor, not established as a
GPU freeze. No further solver tuning was applied in this trial.
