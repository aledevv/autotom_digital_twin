# Detachable fruit stability experiments

This opt-in investigation targets the complete day-160 ExporterV2 plant. The
normal fruit-free launcher is unchanged. A headless pass is only a prerequisite
for a three-minute GUI review, including the user's Shift+click interactions.

The original state is preserved on GitHub by annotated tag
`checkpoint/v2-before-detachable-fruit-2026-09-09`, resolving to
`ac8793ea683998eab8d7318d6e30182535ce5543`. Work takes place on
`experiment/v2-detachable-fruit-stability`.

The first instrumented comparison is documented in [RESULTS.md](RESULTS.md).

## Run a case

From the repository root, with the project environment and Isaac Sim installed:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_experiment.py \
  --run-dir /tmp/autotom-fruit-v2/example-full
```

Use a fresh directory for each case. `--prepare-only` performs export, ablation,
and the USD audit without launching Isaac. `--source-usd PATH` reuses an existing
full physical-fruit source for exact paired comparisons. `--input PATH` records
the associated PlantState and its SHA-256; it defaults to day 160.

Scenarios: `full`, `no-fruit`, `single`, `truss`, `trusses`. The reduced cases
select the highest **authored** fruit/pedicel mass ratio by default; `--fruit`
selects an exact fruit USD path. The selected truss's root or pedicel joint is
reattached to the original fixed root at the same world rest frame. Removed
bodies, reanchored joints, retained masses, and collision filters are audited.
Removing vegetative bodies removes their aggregated leaf mass as well.

`--no-collisions` disables contacts after cooking while preserving and checking
effective runtime masses and inertias. Cross it with `--unbreakable` for the
collision/breakage 2x2 comparison. `--attachment internal --unbreakable` is
restricted to `single` and `truss`: it is a diagnostic control, not a detachable
solution. No articulation is rebuilt during simulation.

Solver controls: `--solver TGS|PGS`, `--cpu`, `--art-position`, `--art-velocity`,
`--fruit-position`, `--fruit-velocity`. Zero velocity iterations are allowed for
the documented TGS comparison. Elastic controls are `--damping-scale 1|2|4|7`
and `--stiffness-scale 1|0.5|0.25`; dimensions, masses, and the 6 N final break
threshold are preserved. All cases use four simulation/task workers, recorded
in their runtime configuration. Use `--hz 480` for the shared GUI/headless gate.
Stiffness changes preserve the requested damping ratio by scaling the damping
coefficient with the square root of stiffness. The loader restores the requested
solver before PhysX initialization: Isaac's `World` otherwise changes PGS to TGS.
Runtime solver/GPU/frequency mismatches and scene hash mismatches fail the run.
The damping multiplier is relative to the baseline ratio 4. The mutually
exclusive `--damping-ratio` option reproduces early cases that used absolute
ratios; configuration files record both the resulting ratio and multiplier.

Prepared cases can be run sequentially with `run_batch.py DIR [DIR ...]`.
It launches one fresh Isaac process per case, preserves each exit code and log,
and continues after a physical gate failure. It refuses existing reports.

## Measurement and acceptance

The monitor reads poses and linear/angular velocities through a batched PhysX
view at every physics step. It records terminal constraint frame errors,
PhysX break events, per-role peaks, root drift, the first failed attachment, and
full state traces in compressed five-second chunks. The scalar-first quaternion
and column order are embedded in each NPZ file. Sample times are in the companion
metrics NPZ. Broken fruit remains in traces but is excluded from settling gates.
The latest monitor also records per-body peaks and independent COM/angular
finite differences; the same velocity limits apply to those measurements.

Inertia eigenvalues and effective mass properties are checked at startup and
after changing the collision state. Infinite authored solver limits are encoded
as strings in JSON; nonfinite simulated state is a failure.

Screening lasts 20 simulated seconds. Candidates run for 120 seconds; the best
candidate needs three independent starts and a 180-second run. Over the final
10 seconds, gates are 5 mm/s linear speed, 0.05 rad/s angular speed, 1 mm position
excursion, 0.5 mm terminal-frame separation, and 0.5 degrees frame rotation.
Excursion is conservatively bounded by the trajectory bounding-box diagonal.
Reported solver velocities should also be inspected alongside actual pose
changes: these are separate measurements, not interchangeable evidence of
visible jitter.
`recheck_pose_velocities.py DIR [DIR ...]` independently recomputes the last
ten seconds from saved poses and the effective COM frames. It writes a separate
`pose-velocity-check.json` and preserves the original runtime report.

For controlled detachment, add `--duration 120 --force-target min|median|max`.
At 30 seconds (configurable using `--force-start`), apply a global downward force
at the fruit center of mass, ramping from 0 to 12 N in five seconds. Stop applying
force when the target joint breaks. Unexpected breaks fail the automated case.
Trace files retain the pre/post-break pose and velocity for continuity review.
Breaking before stimulation or failing to break within the ramp also fails.

Only after a headless candidate passes, open the **same prepared scene**:

```bash
~/isaacsim/python.sh src/exporterV2/isaac_app.py \
  --usd /tmp/autotom-fruit-v2/CANDIDATE/scene.usda \
  --physics-preset flexible --interactive-physics-hz 480 --duration 180 \
  --fruit-experiment /tmp/autotom-fruit-v2/CANDIDATE/config.json
```

GUI rendering runs at 60 Hz with eight explicitly sampled physics steps per
render. Shift+click uses the existing PhysX mouse-grab configuration. Natural
break events during manual interaction are logged, not classified as automated
test failures. GUI output is separate from headless output, and ends with
`awaiting_user_review`; it never assigns the user's acceptance automatically.
Both simulated and wall time are reported. Keep an unstimulated configuration
for manual review; do not reuse an automated force-ramp configuration.

All generated scenes, manifests, traces, reports and figures remain local under
the chosen run directory. The runner records input/source/scene hashes,
preparation code hashes, executed monitor code hashes, and Isaac version.
Completed investigations can be archived under the Git-ignored directory
`artifacts/detachable_fruit_v2/`; raw results are never committed with the code.
