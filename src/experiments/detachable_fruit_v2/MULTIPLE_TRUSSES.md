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
