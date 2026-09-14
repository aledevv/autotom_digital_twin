# Final bounded position-iteration comparison

2026-09-14. User requested one final solver test before returning to the main
reference as the basis for v2.3. Native interaction is unchanged.

Use isolated rank 10, eight fruits, full original stem, TGS/GPU at 60 Hz,
original fruit masses and corrected geometry, support density 20000 kg/m3,
original drives/limits, break threshold 6 N. Increase only position iterations
for the articulation and external fruit bodies; velocity counts stay 4 and 1.
Each scene starts fresh; stop screening at the first spontaneous break.

| Position iterations (articulation / fruit) | First recorded break | Outcome |
| --- | --- | --- |
| 32 / 32, existing control | 0.20000001 s, lat_3_R | Failed |
| 64 / 64 | 0.11666667 s, lat_3_R and lat_3_L | Failed |
| 128 / 128 | 0.01666667 s, lat_0_R and lat_0_L | Failed |

Loaded settings and all expected mass/inertia/COM checks pass. Frame audits
pass. Times are completed-step event attribution; console callbacks can carry
the preceding step time. These are JOINT_BREAK failures, not observed GUI
freezes. Neither candidate proceeds to five trusses or manual GUI. No further
solver search is performed. Increasing position iterations is not a remedy
for this configuration and does not monotonically delay the first break.

Code change: `prepare_rank_diagnosis.py --position-iterations 64|128` prepares
fresh variants and records exact modified attributes plus configuration and
hashes. Results: `last_solver_results.json`. Heavy logs, USD and traces remain
in `artifacts/detachable_fruit_v2/last-solver-test/`.

## Boundary for the main-to-v2.3 fallback

These recent test scenes ALREADY use the archived main builder and CSV adapter,
with fruit size corrected and original source masses. Thus copying that builder
alone into v2.3 cannot be represented as a demonstrated fix for rank 10 or the
whole plant. The positive manual reference is the rank-6 full truss with native
detachment; its record and launch are in MULTIPLE_TRUSSES.md. Preserve it as a
reference, along with the custom-controller checkpoint, without changing main
or groPy.

For a subsequent integration, compare v2.3 PlantState-derived branch definitions
to this main reference at the adapter/builder boundary, then validate the same
rank-6 fixture before adding other trusses. Do not silently replace fruit masses
with rank-6 values or assert whole-plant stability. No adapter migration is
implemented in this final solver test.
