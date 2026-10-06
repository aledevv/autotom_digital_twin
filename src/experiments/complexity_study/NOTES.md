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

Status: **step 1 (joint locking) and step 2 (link merging) implemented.** CLI: `--joint-budget N --optimizer-techniques lock|full --merge-stiffness-policy load|series|keep`. The default technique set is `lock` until merged stages are validated in the GUI.

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
- `load` (default): w = (M_J·d_J)/(M_U·d_U), the rest-pose gravity moment times the arm to the downstream centre of mass, clamped to [0, 1].

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

## Next steps (resume here)

1. Fidelity batch from the **production default** as reference (`d160_truss-supports`, 216 bodies / 206 D6):
   - `--joint-budget 120|60|40 --optimizer-techniques full` with `load`;
   - 40 with `keep` and `series`;
   - `--physical-petiolules --joint-budget 206 --optimizer-techniques full` (255 vs 216 bodies at equal D6).

   Run them one at a time. After a 256+ link crash, CUDA can stay unusable for a while (`cudaErrorInitializationError`); check with a 0.5 s probe first.
2. Per-organ fidelity metric (tip error vs reference after settling), instead of the maxima in the stability report. This is the plan's `fidelity_bench.py`.
3. Interactive GUI benchmark (`interactive_bench.py`) and the cost-model fit (`analyze.py`), as in the plan.
4. Decide the default `--optimizer-techniques` (still `lock`) once merged stages pass a GUI review.
5. Update `core/optimizations/docs/RESEARCH_VALIDATION.md` (64-link claim) and the `budget_config.yaml` comment ("~250 joints") with the measured 255-link limit.
