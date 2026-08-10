# Optimization Tests

Test suite for the joint-budget optimization pipeline.

> **TL;DR — run everything at once:**
> ```bash
> cd ~/isaacsim/autotom_digital_twin
> uv run pytest src/exporterV2/core/optimizations/tests/ \
>     --ignore=src/exporterV2/core/optimizations/tests/visual_validation \
>     -v
> ```
> ✅ Expected: **92 passed, 0 failed** (≈2 s)

---

## Two runners — which one to use?

| Runner | When to use |
|--------|-------------|
| `uv run python …` | Pure-Python scripts (pytest, matplotlib) — NO Isaac Sim |
| `~/isaacsim/python.sh …` | Scripts that `import omni` / `SimulationApp` |

> ⚠️ Never run `~/isaacsim/python.sh` on a `test_*.py` file — it doesn't have `pytest`.

---

## Test structure

```
tests/
├── 1_infrastructure/         pytest unit tests — optimizer config & reports
├── 2_collision/              pytest unit tests + matplotlib visual tests
├── 3_geometry/               pytest unit tests + matplotlib visual tests
├── 4_petiole_lock/           pytest unit tests + USD generator + Isaac Sim compare
├── 5_lateral_reduce/         pytest unit tests + USD generator
├── 8_leaf_branch_reduce/     pytest unit tests + USD generator + Isaac Sim compare
├── 9_integration/            pytest integration tests (multi-technique pipeline)
├── 10_thin_link_lock/        pytest unit tests + USD generator + Isaac Sim compare
├── visual_validation/        Full-pipeline USD generator + Isaac Sim loader
├── test_integration.py       pytest — real CSV plant end-to-end
├── test_cli_integration.py   pytest — CLI flag wiring
└── demo_integration.py       standalone demo script
```

---

## 1 — Infrastructure

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/1_infrastructure/ -v

# Interactive demo
uv run python src/exporterV2/core/optimizations/tests/1_infrastructure/demo_task1.py
```

**Covers:** config loading, joint counting, lower bound calculation, report formatting.

---

## 2 — Collision Detection

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/2_collision/ -v

# Visual: 4 static scenarios saved as PNG + shown interactively
uv run python src/exporterV2/core/optimizations/tests/2_collision/visual_collision_4_scenarios.py

# Visual: random N-body test
uv run python src/exporterV2/core/optimizations/tests/2_collision/visual_collision_random_test.py

# Visual: 3D interactive (matplotlib)
uv run python src/exporterV2/core/optimizations/tests/2_collision/visual_collision_3d_interactive.py
```

**Covers:** two-stage detection (sphere → AABB), margin handling, pairwise scan.

---

## 3 — Geometry Remapping

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/3_geometry/ -v

# Visual: 3D matplotlib comparison (before/after)
uv run python src/exporterV2/core/optimizations/tests/3_geometry/visual_remapping_3d_new.py

# Generate remapping USD for Isaac Sim
uv run python src/exporterV2/core/optimizations/tests/3_geometry/generate_remapping_usd.py
# → then load:
# ~/isaacsim/python.sh -m isaacsim <path to generated .usda>
```

**Covers:** `attach_frac` proportional remapping, height preservation, batch remapping.

---

## 4 — Petiole Lock

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/4_petiole_lock/ -v

# Generate baseline.usda + petiole_lock.usda
uv run python src/exporterV2/core/optimizations/tests/4_petiole_lock/generate_comparison_usd.py

# Load each USD in Isaac Sim (open viewer)
~/isaacsim/python.sh -m isaacsim \
    src/exporterV2/core/optimizations/tests/4_petiole_lock/usd_output/baseline.usda
~/isaacsim/python.sh -m isaacsim \
    src/exporterV2/core/optimizations/tests/4_petiole_lock/usd_output/petiole_lock.usda

# Side-by-side comparison scene in Isaac Sim
~/isaacsim/python.sh \
    src/exporterV2/core/optimizations/tests/4_petiole_lock/compare_isaac_sim.py
```

**Covers:** petiolule identification, D6→Fixed conversion, geometry unchanged, DOF count.

---

## 5 — Lateral Reduce

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/5_lateral_reduce/ -v

# Generate baseline.usda + lateral_reduce.usda
uv run python src/exporterV2/core/optimizations/tests/5_lateral_reduce/generate_comparison_usd.py

# Load each USD in Isaac Sim (open viewer)
~/isaacsim/python.sh -m isaacsim \
    src/exporterV2/core/optimizations/tests/5_lateral_reduce/usd_output/baseline.usda
~/isaacsim/python.sh -m isaacsim \
    src/exporterV2/core/optimizations/tests/5_lateral_reduce/usd_output/lateral_reduce.usda
```

> ℹ️ No `compare_isaac_sim.py` for this folder yet — load the files individually.

**Covers:** n_links reduction, total length preserved, child `attach_frac` remapping, Fixed-branch exclusion.

---

## 8 — Leaf Branch Reduce

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce/ -v

# Generate baseline.usda + leaf_branch_reduce.usda
uv run python src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce/generate_comparison_usd.py

# Side-by-side comparison scene in Isaac Sim
~/isaacsim/python.sh \
    src/exporterV2/core/optimizations/tests/8_leaf_branch_reduce/compare_isaac_sim.py
```

**Covers:** petiole+rachis merge, merged length preservation, petiolule `attach_frac` remapping.

---

## 9 — Integration (multi-technique pipeline)

```bash
# Full pipeline integration tests
uv run pytest src/exporterV2/core/optimizations/tests/9_integration/ -v

# Also run the top-level integration tests (real CSV + CLI wiring)
uv run pytest src/exporterV2/core/optimizations/tests/test_integration.py \
              src/exporterV2/core/optimizations/tests/test_cli_integration.py -v

# Standalone demo (prints a full run without pytest)
uv run python src/exporterV2/core/optimizations/tests/demo_integration.py
```

**Covers:** 5 budget scenarios (within/over/impossible budget, progressive reduction, real CSV plant),
CLI flag `--optimize` wiring, `BudgetOptimizer` loop correctness.

---

## 10 — Thin Link Lock

```bash
# Unit tests (via pytest)
uv run pytest src/exporterV2/core/optimizations/tests/10_thin_link_lock/ -v

# Generate baseline.usda + thin_link_lock.usda
uv run python src/exporterV2/core/optimizations/tests/10_thin_link_lock/generate_comparison_usd.py

# Side-by-side comparison scene in Isaac Sim
~/isaacsim/python.sh \
    src/exporterV2/core/optimizations/tests/10_thin_link_lock/compare_isaac_sim.py
```

**Covers:** thin branch identification (radius threshold from `tree_config.GLOBAL_SCALE`),
D6→Fixed conversion, no geometry change, physics stability.

---

## Visual Validation (full pipeline)

Generates one USD per optimization stage so you can step through each technique in Isaac Sim.

```bash
# Step 1 — generate all 6 USD stages (pure Python, no Isaac Sim needed)
uv run python src/exporterV2/core/optimizations/tests/visual_validation/run_visual_test.py
# → writes usd_output/  0_baseline.usda … 5_fully_optimized.usda
# → prints a joint-count summary table

# Step 2a — load a single stage in Isaac Sim
~/isaacsim/python.sh -m isaacsim \
    src/exporterV2/core/optimizations/tests/visual_validation/usd_output/0_baseline.usda

# (replace 0_baseline with 1_petiole_lock, 2_lateral_reduce, etc.)

# Step 2b — load all stages in a single Isaac Sim scene
~/isaacsim/python.sh \
    src/exporterV2/core/optimizations/tests/visual_validation/load_final_test.py

# Optional: combination matrix test
uv run python src/exporterV2/core/optimizations/tests/visual_validation/generate_combinations_usd.py
~/isaacsim/python.sh \
    src/exporterV2/core/optimizations/tests/visual_validation/load_combination_isaacsim.py
```

**Stages generated:**

| File | Technique applied | D6 joints saved |
|------|-------------------|-----------------|
| `0_baseline.usda` | None | — |
| `1_petiole_lock.usda` | PetioleLock | petiolules × 1 |
| `2_lateral_reduce.usda` | LateralReduce | lateral links |
| `3_stem_collapse.usda` | StemCollapse | trunk links |
| `4_leaf_branch_reduce.usda` | LeafBranchReduce | rachis links |
| `5_fully_optimized.usda` | BudgetOptimizer (all) | cumulative |

---

## Coverage status

| Folder | pytest | USD generator | Isaac Sim script | Status |
|--------|--------|---------------|------------------|--------|
| `1_infrastructure` | ✅ | — | — | ✅ |
| `2_collision` | ✅ | — | — | ✅ |
| `3_geometry` | ✅ | ✅ | manual load | ✅ |
| `4_petiole_lock` | ✅ | ✅ | ✅ compare | ✅ |
| `5_lateral_reduce` | ✅ | ✅ | manual load | ✅ |
| `6_stem_collapse` | — | — | — | ❌ no tests yet |
| `7_truss_static` | — | — | — | ❌ removed (not in pipeline) |
| `8_leaf_branch_reduce` | ✅ | ✅ | ✅ compare | ✅ |
| `9_integration` | ✅ | — | — | ✅ |
| `10_thin_link_lock` | ✅ | ✅ | ✅ compare | ✅ |
| `visual_validation` | — | ✅ | ✅ loader | ✅ |
| `test_integration.py` | ✅ | — | — | ✅ |
| `test_cli_integration.py` | ✅ | — | — | ✅ |

**Total pytest count: 92 tests, 0 failures.**

---

## Notes

- All commands assume CWD = `~/isaacsim/autotom_digital_twin`
- `uv run` handles the Python environment — no need to activate conda
- Isaac Sim scripts must use `~/isaacsim/python.sh`, not `uv run`
- The `visual_validation/` folder is excluded from the default `pytest` run because its scripts are USD generators, not pytest files
- Task 7 (TrussStatic) was removed from the active pipeline; no tests planned
