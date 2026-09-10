# Iteration count, 60 Hz and rendered performance — 2026-09-10

The user revised the validation duration to **60 seconds**, including manual
Shift+click in the final GUI review. The performance target is approximately
20 rendered FPS or higher. A 60 Hz physics rate specifies the integration
timestep; it does not by itself guarantee 60 physics steps per wall second.
The report therefore records both FPS and simulated-time/wall-time ratio.

This investigation preserves the full day-160 source, all 72 fruits, their
masses and geometry, collision filters, and native 6 N breakage from startup.
PGS is retained, with articulation counts 32/0 and fruit velocity iterations 0.
The first changed control is fruit position iterations (32 to 64), followed by
frequency (480 to 60 Hz). CPU is a separate execution comparison at equal
settings. These are experimental configurations, not accepted production presets.

Local results are under `artifacts/detachable_fruit_v2/2026-09-10/`. The source
is the archived `2026-09-09/baseline-full/source.usda`; its SHA-256 is
`94fe77b9378ff6cacb4e7bde4723bcb1bc51b7281c95d61f670d536f651f332c`.

The user then clarified the minimum requirement: attached fruit at rest,
correct interactive detachment, and at least 20 GUI FPS. Angular error alone
is advisory. Follow-up runs explicitly use `--acceptance functional`; earlier
strict failures are retained unchanged. Final visual/interaction approval
still belongs to the user.

## Measurement

- The same monitor samples every physics step at both frequencies.
- Final-ten-second maxima retain the existing velocity, excursion, attachment
  translation and angular thresholds. Final per-fruit frame errors are saved
  separately to distinguish a lasting offset from settling motion.
- Headless throughput is physics steps per wall second; it is **not** GUI FPS.
- GUI FPS uses the entire interval between rendered frames, including physics,
  monitoring and rendering. The report includes overall and post-warmup FPS
  (after five simulated seconds), fifth-percentile frame FPS, viewport
  resolution, camera and renderer configuration. It also saves frame timings.
- The previous 480 Hz / 32-iteration run supplies a physics baseline. Its
  historical timing is not a same-session performance benchmark, because the
  monitor's COM calculation has since been corrected. Same-session 60 Hz cases
  provide the controlled iteration-cost comparison.

## Completed 20-second screens

All cases completed with zero native break events. Counts below are fruit
position iterations; the articulation remains 32/0. Angles and gaps are maxima
over the final ten seconds, not whole-run startup peaks.

| Backend | Hz | Fruit iterations | Tail angle (deg) | Tail gap (mm) | Headless steps/wall s |
|---|---:|---:|---:|---:|---:|
| GPU, historical baseline | 480 | 32 | 2.463 | 0.402 | not comparable |
| GPU | 480 | 64 | 1.154 | 0.194 | 12.91 |
| GPU | 60 | 32 | 80.181 | 29.189 | 21.29 |
| GPU | 60 | 64 | 39.214 | 11.918 | 12.82 |
| CPU | 480 | 64 | 0.334 | 0.048 | 75.98 |
| CPU | 60 | 64 | 9.656 | 2.345 | 80.57 |
| CPU | 60 | 255 | 5.315 | 1.185 | 52.49 |

The controlled GPU iteration comparison documents a reduction of the angular
residual. Waiting alone does not explain it: final errors at 20 seconds are
2.447 degrees for the old 32-count case and 1.152 degrees for the new 64-count
case. At 60 Hz, even the 255-count CPU screen fails the attachment tolerances.
Backend changes affect both residual and cost; their cause is not established
by this comparison. Both backend configurations retain MBP broadphase; the
runtime `enableGPUDynamics` flag is checked against each requested configuration.

No screen passes every original numerical gate. The 480 Hz CPU case passes
attachment and pose-derived motion gates but fails the raw PhysX velocity
gates (0.01328 m/s and 0.63637 rad/s versus pose-derived 0.00438 m/s and
0.01850 rad/s). This discrepancy must remain explicit; a solver velocity alone
is not a measurement of visible trembling. The one-minute runs examine whether
residuals and actual pose changes persist.

The CPU 480 Hz throughput corresponds to approximately 9.5 batches of eight
physics steps per wall second before rendering, using the current GUI loop.
It therefore does not demonstrate the requested 20 FPS. CPU 60 Hz leaves more
rendering budget, but its physics has not passed. No GUI FPS has been measured.

## One-minute follow-up

CPU 480 Hz / 64 completed 60 simulated seconds with zero breaks. In the
final ten seconds its attachment error is 0.338 degrees / 0.0421 mm and its
position excursion is 0.0186 mm. Pose-derived speeds are 0.145 mm/s and
0.00128 rad/s. Raw solver velocities remain above the old strict gates, so
the original strict result remains failed. Headless throughput is 77.33 steps/s.

The fresh CPU 60 Hz / 255 run completed 60 simulated seconds with zero breaks
and passed the functional headless policy. Its final-ten-second maximum is
5.235 degrees / 1.076 mm; position excursion is 1.997 mm. Pose-derived speeds
are 0.298 mm/s and 0.0204 rad/s. Raw solver velocities still exceed their
original tolerances. Headless throughput is 49.69 steps/s, with a real-time
factor of 0.828. This pass does not establish GUI FPS or visual acceptance.

## Automatic detachment at 60 Hz

| Mass-ratio target | Break time (s) | Applied ramp force (N) | Continuity |
|---|---:|---:|---|
| min | 32.650 | 6.360 | True |
| median | 32.483 | 5.960 | True |
| max | 32.467 | 5.920 | True |

Each fresh run completed 60 simulated seconds. The ramp starts at 30 seconds;
each target detached with a native JOINT_BREAK event, no other joint broke,
and the one-step continuity check passed. The force in this table is the
externally applied ramp, not a measurement of the joint reaction force.
Numerical advisories remain in the reports. Detached bodies are excluded from
support settling metrics, but their free-fall motion remains in the traces.

GUI FPS and manual interaction acceptance are still pending.
