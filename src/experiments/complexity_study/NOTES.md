# Complexity study – method notes and known limitations

## Measured hard limit: 255 links per articulation (2026-10-06)

Isaac Sim 4.5 / PhysX 5, RTX 4080 Laptop, day-160 PlantState stages built with `core/usd/body_merge.py` to an exact body count (fixed petiolule links merged, so the physics is otherwise identical), 0.5 s headless at 480 Hz:

| Articulation links | GPU dynamics | CPU dynamics |
|---|---|---|
| 216 (production default) | runs | – |
| 246 / 255 | runs | runs (255) |
| 256 / 257 / 266 / 347 | fails: `PhysX error: GPU ... fail to launch kernel`, CUDA error 700; sometimes appears as a hang on stage open | segmentation fault on stage open (256, 347) |

- The limit is on **links (bodies) per articulation, not D6 joints**: the lock-only stage with 134 D6 but 347 bodies fails, while 40 D6 / 246 bodies runs.
- This is very likely the "~250 joints" observed with the legacy pipeline, where joints ≈ links − 1. The `budget_config.yaml` comment and `OPTIMIZATION_README.md` describe it as instability or "melting"; what we measured is a hard failure at 256 links.
- Memory is not the cause (12 GB VRAM, failure at a few hundred links).
- Enforced in code:
  - `MAX_ARTICULATION_LINKS = 255` (`plant_state_merge_techniques.py`);
  - the export refuses more than 255 links with an explicit error instead of crashing Isaac;
  - with `--optimizer-techniques full`, the optimizer merges fixed-joint links (`fixed_link_merge`, no fidelity cost) until the plant fits.
- Consequence: the step-1 "links vs DOF" pair (347 vs 216 bodies at 412 DOF) cannot run. Use 255 vs 216 bodies at 206 D6 instead (`--physical-petiolules --joint-budget 206 --optimizer-techniques full`).

Plan: `~/.claude/plans/hi-i-have-created-parallel-quokka.md`.
Static profiler: `src/exporterV2/complexity_profile.py`.

## The 220/230 D6 joint threshold

What the repository says (2026-10-06):

- `JOINT_TARGET = 220` and `JOINT_WARNING_MAX = 230` (`src/exporterV2/plant_state_adapter.py:36-37`, mirrored in `plant_state_legacy_backend.py:1641`). They were introduced in commit `da6e53a` (Phase J, 2026-08-21) without a dedicated measurement. They are a safety margin under the "~250 joints per articulation" value in `core/optimizations/docs/OPTIMIZATION_README.md:61` and `notion_pages/7_Comprehensive_Optimization_Report.md:5`. That document describes 250 as an **empirical observation on our hardware** (solver instability, "melting", crashes), not as a documented PhysX limit.
- `core/optimizations/docs/RESEARCH_VALIDATION.md:9` cites a hard limit of 64 links per PhysX articulation. The repository's own stages contradict it: PlantState day 50 has 133 links in one articulation, legacy day 100 has 165, and both load and run. The claim is from older PhysX/Unity documentation and must not be cited for this setup without checking against the PhysX 5 / Isaac Sim 4.5 docs.
- **What `--optimize` actually did (corrected 2026-10-06)**: every `--debug-profile` value is in `INCREMENTAL_PROFILES`, so day-based exports always take `export_incremental_checkpoint` in `plant_state_legacy_backend.py`. The `--optimize` / `JOINT_TARGET` branch in `plant_state_adapter.py:1186` is never reached, and `--optimize` was **silently ignored**. The effective gate was the hard 230 check in the backend, unless `--allow-over-budget` was given.
- **Now**: `--joint-budget N` (and `--optimize`, which equals `--joint-budget 220`) runs `BudgetOptimizer` on the PlantState branches before the 220/230 checks, with only the joint-locking techniques (see below).
- Memory is an unlikely cause: one articulation with about 150 links is very small compared with the 12 GB of VRAM of the test machine. The study measures RSS/VRAM against DOF to confirm or refute this, and pushes the model above 230 to show which limit appears first: cost (FPS < 20), stability, or memory.

## Legacy vs PlantState starting point

Static profile (`complexity_profile.py`):

| Stage | Pipeline | Bodies | D6 | DOF | Pos/vel iters | Colliders | Visible triangles |
|---|---|---|---|---|---|---|---|
| `tree_v2_day_100.usda` | legacy BRANCHES | 165 | 164 | 328 | 64/8 | 165 cylinders | 0 |
| `tree_v2_day_50.usda` | PlantState | 133 | 123 | 246 | 32/4 | 266 capsules | 239,514 |

Phase J measured legacy 4.4-6.5× slower than PlantState at day 50 at the same physics rate. Part of the gap can come from settings (2× iterations) and not from topology. A cost law fitted on the legacy budget sweep alone may therefore not transfer to the PlantState plant. Hence:

1. Benchmark both pipelines with the same solver settings.
2. Fit the cost model with `pipeline` as a factor.
3. Use the legacy sweep in absolute terms only if the slopes agree. Otherwise report it as a relative trend only.

## Self-collisions

`enabledSelfCollisions = False` on the articulation excludes every link-link pair, so the idle plant has **0 candidate collision pairs** (35,112 raw shape pairs at day 50). The 473 authored FilteredPairs only matter if self-collisions are turned on. Collider cost therefore appears only during interaction with external bodies (mouse/gripper, fruits, ground). That is why the interactive benchmark includes a scripted drag scenario.

## PlantState joint counts by day (exporter run 2026-10-06)

| Day | Profile | Physical links | D6 |
|---|---|---|---|
| 50 | truss-supports | 133 | 123 |
| 80 | truss-supports | 216 | 206 |
| 160 | truss-supports | 216 | 206 |
| 160 | truss-supports + `--physical-petiolules` | 347 | 337 |

- Days 80 and 160 have the same topology (organ counts, links, D6) but different sizes; `test_day_160_is_structurally_mature_but_not_a_copy_of_day_80` checks larger fruit radii at day 160. Model complexity (level B) is identical, mass and geometry are not, so they are one point on a cost-vs-DOF plot but two for fidelity.
- Day 160 with physical petiolules (337 D6) is the natural over-budget start point for an optimizer sweep on the PlantState pipeline.

## BudgetOptimizer on PlantState

Status: **step 1 (joint locking) and step 2 (link merging) implemented.** CLI: `--joint-budget N --optimizer-techniques lock|full --merge-stiffness-policy load|tip|blend|series|keep`. Defaults: technique set `lock` (until merged stages are validated in the GUI), compliance policy `blend` (since 2026-10-08).

### Step 1 – joint locking (`src/exporterV2/plant_state_optimization.py`)

- Techniques: `thin_link_lock`, `petiole_lock`, and `pedicel_lock`, a new `PedicelLockTechnique` that is the pedicel stage of `truss_static` without the static rachis curve.
- Each locked branch also gets `attachment_joint_type = "fixed"`. Otherwise the USD keeps one attachment D6 per branch while the budget counts zero. The export audit checks authored vs expected D6.
- `petiole_lock` now also matches `kind == "petiolule"`: 25 of 131 PlantState petiolules are named `..._rachis_terminal_...` and were missed by the name test.
- The report is in the manifest: `physics.joint_budget.optimization`, `metadata.joint_budget`.
- Day 160 with physical petiolules, measured:

| Budget | D6 | Bodies | Applied |
|---|---|---|---|
| none | 337 | 347 | – |
| ≤ 330 … 206 | 206 | 347 | all 131 petiolules |
| 150 | 150 | 347 | + 7 trusses' pedicels |
| 134 | 134 | 347 | + all 9 trusses (floor of step 1) |
| 133 | error | | below the locking floor |

- Granularity: petiole lock is all-or-nothing (−131), pedicel lock goes one truss (−8) per pass.
- ~~Links vs DOF experiment: 347 vs 216 bodies~~. The 347-body stage exceeds the 255-link limit (see top). Lock-only exports with more than 255 links are now refused.

### Step 2 – link merging

Files:
- `src/exporterV2/plant_state_merge_techniques.py`: PlantState versions of `lateral_reduce`, `stem_collapse`, `truss_static` and `leaf_branch_reduce`. They never change `n_links`, lengths or poses; they only mark `merged_links`.
- `src/exporterV2/core/usd/body_merge.py`: `merge_rigid_links`, which runs on the built stage **after** the normal export audit.
- `PlantStateBudgetOptimizer` in `plant_state_optimization.py`: counts D6 and bodies after merges, and gives the structural lower bound (one D6 per lateral, leaf and truss).

Differences from the legacy techniques:

| Technique | Legacy BRANCHES | PlantState |
|---|---|---|
| `leaf_branch_reduce` | replaces petiole and rachis with one straight pre-bent segment | petiole and rachis links become one body; the exact GroIMP shape is kept |
| `lateral_reduce` | removes a segment and rescales | merges links, balanced (joins the adjacent body groups with the fewest links) |
| `stem_collapse` | collapses to 3 segments with collision-checked child remapping | merges stem links to 3 bodies; children follow exactly, no remap needed |
| `truss_static` | pedicels locked, rachis rebuilt as a 5-segment static curve, root stiffness ×0.40 | pedicels locked, rachis links merged into link 1; root stiffness from the policy below |

Merge audit (raises on failure):
- every mesh and collider keeps its world transform (1e-6);
- total mass and plant centre of mass are unchanged;
- bodies and D6/Fixed joints drop by exactly the merged links;
- no relationship targets a removed body;
- the D6 count matches the optimizer prediction.

Compliance policies, where `1/K_U' = 1/K_U + w/K_J`:
- `keep`: w = 0, merged parts are rigid (legacy behaviour);
- `series`: w = 1;
- `load`: w = (M_J·d_J)/(M_U·d_U), the rest-pose gravity moment times the arm to the downstream centre of mass, clamped to [0, 1].

At day 160 with budget 40, the `load` weights are min 0.005, median 0.17, max 0.59, so `series` over-transfers compliance about 6× at the median.

Day 160 with physical petiolules, `--optimizer-techniques full`:

| Budget | D6 | Bodies | Techniques applied |
|---|---|---|---|
| none | 337 | 347 | – |
| 200 | 200 | 341 | petiole lock, 6 lateral merges |
| 120 | 116 | 322 | + stem, pedicels, rigid rachides |
| 60 | 60 | 266 | + 5 leaves |
| 40 | 40 | 246 | + all 28 leaves (structural lower bound) |
| 39 | error | | below the lower bound |

Locked petiolules and pedicels are still separate bodies on fixed joints. Merging fixed-joint links costs no fidelity: a fixed articulation joint is already rigid. If the study shows that bodies cost runtime, a `fixed_link_merge` technique is the next cheap win.

## Headless results so far (2026-10-06, 480 Hz, 5 s, `isaac_app.py --headless`)

Start: day 160 with physical petiolules (337 D6). Optimized to 40 D6 / 246 bodies with `--optimizer-techniques full`, compared across the three compliance policies. All runs passed the stability checks.

| Policy | Max displacement | Max endpoint displacement | Sag internode | Sag petiole | Sag truss rachis |
|---|---|---|---|---|---|
| series | 0.201 m | 0.214 m | 0.368 | 2.555 | 4.099 |
| load | 0.077 m | 0.082 m | 0.122 | 0.853 | 1.366 |
| keep | 0.039 m | 0.041 m | 0.056 | 0.400 | 0.629 |

- Sag ratios are measured on the survivor link's own length, so compare them only across variants of the same body.
- Wall time for 5 s simulated (2400 steps) was about 2.5–3.75 min for these 246-body stages. The production default (216 bodies, 206 D6) took 186 s. These are not clean timings: other jobs were running.
- The unoptimized 347-body stages never ran (255-link limit, see top).

## Fidelity vs cost from the production default (2026-10-08)

Reference: `d160_truss-supports` (production default: 216 bodies, 206 D6, petiolules visual-only). Variants come from the same plant with `--joint-budget N --optimizer-techniques full`. `g206` is `--physical-petiolules --joint-budget 206 --optimizer-techniques full`: 255 bodies, also 206 D6.

Tool: `src/experiments/complexity_study/fidelity_bench.py`.
- `run` settles the plant 5 s under gravity at 480 Hz, headless, and records the world position of every visual mesh centre.
- `compare` matches all 558 meshes by a key that survives merging.
- Re-running the reference gives exactly 0 mm error and 138.9 s vs 138.4 s wall time, so PhysX is deterministic here and every millimetre below is a real model difference.
- Reference displacement under gravity: median 15.2 mm, max 53.3 mm.

| Variant | Bodies | D6 | Wall time for 2400 steps | Speed-up | Mean error | p95 | Max | Median error / ref sag | Stability |
|---|---|---|---|---|---|---|---|---|---|
| reference | 216 | 206 | 138.9 s | 1.00× | 0 | 0 | 0 | 0 | passed |
| b120 `load` | 191 | 116 | 102.6 s | 1.35× | 2.5 mm | 6.8 mm | 10.2 mm | 0.16 | passed |
| b60 `load` | 135 | 60 | 65.8 s | 2.11× | 2.8 mm | 6.9 mm | 10.3 mm | 0.18 | passed |
| b40 `load` | 115 | 40 | 52.9 s | 2.63× | 3.0 mm | 7.1 mm | 10.7 mm | 0.18 | passed |
| b40 `keep` | 115 | 40 | 52.8 s | 2.63× | 7.7 mm | 19.6 mm | 30.6 mm | 0.50 | passed |
| b40 `series` | 115 | 40 | 52.3 s | 2.66× | 37.6 mm | 75.4 mm | 96.1 mm | 2.30 | passed |
| g206 (physical petiolules) | 255 | 206 | 152.9 s | 0.91× | 1.7 mm | 7.5 mm | 12.2 mm | 0.07 | passed |

Wall time is pure `world.step(render=False)` at 480 Hz on an RTX 4080 Laptop / i9-13900HK with GPU dynamics and TGS 32/4.

Reading:
- **`load` is the right compliance policy.** It is about 2.5× more accurate than `keep` (the legacy behaviour: merged parts become rigid) and 12× more accurate than `series` *for statics*. The push test later showed it is too stiff dynamically; `blend` is the default since 2026-10-08.
- Most of the error appears already at b120, with pedicels locked and laterals/stem merged. Going from 120 to 40 D6 adds only 0.5 mm of mean error but nearly halves the time.
- **Cost follows DOF more than bodies.**
  - At 206 D6, 216 → 255 bodies costs +10% time (138.9 → 152.9 s).
  - 206 → 40 D6 (with 216 → 115 bodies) gives 2.6× faster.
  - A first rough model (not yet the proper fit): time is roughly linear in D6, and bodies add about 0.15 s per 1000 steps per body (14.0 s / 2.4 / 39).
- The headless validation in `isaac_app.py` reads every body pose at every step, so its steps/s (13.7 → 32.3) include Python overhead. Use the `fidelity_bench.py` wall times or `cost_bench.py` for the cost model.

## Dynamic fidelity: lateral push (2026-10-08)

Tool: `push_bench.py`.
- All stages are placed side by side in one scene and receive identical pushes.
- They settle 3 s at 60 Hz (the interactive regime), then three probe organs get a horizontal force tangential to the stem for 0.2 s: the most cantilevered lateral tip, truss rachis tip and leaf blade.
- The force equals the weight of the plant part downstream of the probe: 0.36 / 0.03 / 0.10 N.
- The response is recorded for 4 s.
- `--gui` shows the same test with an overlay (`push_overlay.py`): force vectors and labels, contact discs, a caption per plant, and a live panel with displacement and a plot. It was approved visually on 2026-10-08.
- Data: `results/push_day160.json`. Figures: `figures/push_trajectories.png`, `figures/cost_vs_fidelity.png`.
- Consistency checks:
  - the reference result is identical in the 2-plant and the 7-plant scenes;
  - the leaf probe in b120/b60 equals the reference, because leaves are only merged at b40.

Peak displacement (mm) / free-response frequency (Hz):

| Variant | Lateral tip | Truss tip | Leaf blade |
|---|---|---|---|
| reference (206 D6) | 91.7 / 2.03 | 8.6 / 1.98 | 146.1 / 3.28 |
| b120 load | 56.6 / 2.46 | 8.2 / 2.35 | 146.1 / 3.28 |
| b60 load | 56.8 / 2.46 | 5.5 / 2.35 | 146.1 / 3.28 |
| b40 load | 56.8 / 2.46 | 5.5 / 2.36 | 83.5 / 3.52 |
| b40 keep | 30.7 / 3.19 | 2.6 / 3.41 | 43.8 / 4.89 |
| b40 series | 131.8 / 1.27 | 17.5 / 1.28 | 200.7 / 1.95 |
| g206 (255 bodies) | 87.3 / 1.96 | 12.9 / 1.97 | 149.0 / 2.88 |

Reading:
- **Statics and dynamics disagree on the best policy.**
  - `load` is calibrated on rest-pose gravity moments. It matches the gravity settle (3 mm) but is **too stiff for a tip push**: about 60% of the reference peak and +20% frequency.
  - `series` is too soft: about 145% of the peak, −37% frequency.
  - `keep` is far too stiff: about 33% of the peak.
  - Mean dynamic peak error: load 14% (b120) → 39% (b40), series 62%, keep 69%.
- For a point load P at the tip, joint j gives θ_j = P·d_j/K_j, so the exact single-joint equivalent is `1/K_eq = Σ (d_j/d_0)² / K_j`. A `tip` policy with `w = (d_J/d_U)²` is the natural next candidate; a blend of `load` and `tip` could cover both load cases.
- The reference truss tip does not return to zero (≈ −5 mm residual). The 3 s settle at 60 Hz is not fully at rest before the push. Lengthen `--settle` before using residuals as a metric.

## Tip and blend compliance policies (2026-10-08)

New `--merge-stiffness-policy` values in `core/usd/body_merge.py`:
- `tip`: w = (d_J / d_U)², where d is the distance to the farthest link endpoint downstream of the removed joint. Exact single-joint equivalent for a point load at the tip.
- `blend`: the mean of the `load` and `tip` weights.

Median weights at b40: load 0.17, tip 0.35.

The push was re-run with `--settle 6`; the reference residuals drop to 4.1 / 0.6 / 0.5 mm. The reference is identical across the three push runs, which are merged in `results/push_day160.json`. Static results are in `results/fidelity_day160.json`.

| Policy | Budget (D6) | Static mean error | Dynamic mean peak error | Dynamic frequency error |
|---|---|---|---|---|
| load | 120 / 60 / 40 | 2.5 / 2.8 / 3.0 mm | 22.9 / 29.0 / 43.3 % | 12.0 / 12.0 / 14.5 % |
| tip | 120 / 60 / 40 | 8.1 / 8.9 / 9.4 mm | 7.2 / 12.3 / 18.0 % | 7.3 / 7.5 / 11.0 % |
| blend | 120 / 40 | 4.4 / 5.1 mm | 13.0 / 28.6 % | **0.2 / 1.3 %** |
| keep | 40 | 7.7 mm | 70.2 % | 57.7 % |
| series | 40 | 37.6 mm | 33.0 % | 38.3 % |

Reading:
- `load`, `tip` and `blend` are all on the Pareto front between static and dynamic fidelity. `keep` (legacy) and `series` are dominated.
- `blend` almost exactly matches the reference oscillation frequency, so it is the most natural choice for interactive use. `load` is best for static poses (e.g. perception datasets), `tip` for contact or push tasks.
- Timing noise: identical topologies (same budget, different policy) took 102.6 / 104.5 / 126.3 s at b120, i.e. up to 20% run-to-run variation. `figures/cost_vs_fidelity.png` uses the median wall time per budget. The cost model must use repeated, isolated runs (`cost_bench.py`).

## Next steps (resume here)

1. Decide the default `--optimizer-techniques` (still `lock`). The compliance policy default is now `blend`.
2. Interactive GUI benchmark (`interactive_bench.py`, plan step 3): 60 Hz with RTX rendering, idle and with a scripted drag. The headless 480 Hz numbers above are not the interactive frame time.
3. Cost model fit (`cost_bench.py` with N repeats in fresh processes, then `analyze.py`) on more points: days 50/80/160, budgets 40–206, solver iterations 8/16/32/64, CPU-PGS vs GPU-TGS.
4. Update `core/optimizations/docs/RESEARCH_VALIDATION.md` (64-link claim) and the `budget_config.yaml` comment ("~250 joints") with the measured 255-link limit.
5. After a 256+ link crash, CUDA can stay unusable for a while (`cudaErrorInitializationError`). Check with a 0.5 s probe before a batch.
