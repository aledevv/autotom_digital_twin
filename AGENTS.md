# Repository Guidelines

## Project Structure & Module Organization

This repository generates OpenUSD tomato-plant digital twins for NVIDIA Isaac Sim. The active pipeline is `src/exporterV2/`; `src/exporterV1/` and older root scripts are legacy references.

- `src/exporterV2/core/`: USD stage construction, geometry, joints, physics config, and optimization logic.
- `src/exporterV2/adapters/groimp_csv/`: GroIMP CSV parsing and conversion into branch/truss/terminal-body configs.
- `src/exporterV2/profiles/`: cultivar and generation profiles.
- `src/exporterV2/docs/`: technical notes for the V2 architecture.
- `src/exporterV2/**/tests/`: pytest suites colocated with the modules they exercise.
- `data/`, `model/`, `output/`, `src/data/usd_models/`: input CSV/model data and generated USD/JSON/HTML artifacts.
- `typings/`: USD Python stubs for editor support.

## Build, Test, and Development Commands

Use `uv` for normal Python work:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run pytest
UV_CACHE_DIR=/tmp/uv-cache uv run pytest src/exporterV2/core/optimizations/tests/11_truss_static/test_truss_static.py
./run_mainV2.sh --day 100 --optimize
```

Use Isaac Sim’s Python only for scripts that require Isaac/PhysX runtime APIs:

```bash
~/isaacsim/python.sh path/to/script.py
```

Do not run Isaac-specific scripts with `uv run`; `usd-core` tests and offline USD generation can use `uv`.

## Coding Style & Naming Conventions

Python is the primary language. Use 4-space indentation, descriptive snake_case names for functions and variables, and PascalCase only for classes. Keep exporter changes modular: parsing in `adapters`, physical constants in `core/tree_config.py`, USD authoring in `core/usd/`, and budget logic in `core/optimizations/`. Prefer structured USD/PXR APIs over text editing `.usda` files directly.

## Testing Guidelines

Tests use `pytest`. Name test files `test_*.py` and keep focused tests near the module under test. For exporter changes, add or update targeted tests first, then run the relevant subset. Physics/runtime behavior that depends on Isaac Sim should be validated with `~/isaacsim/python.sh`; pure parser, optimizer, and USD-authoring tests should run with `uv`.

## Commit & Pull Request Guidelines

Recent commits use short, imperative summaries, for example `Truss optimization implemented` or `Solved instability problem + started detachment`. Keep commits scoped to one behavior change when possible. PRs should describe the affected pipeline stage, list test commands run, mention generated artifacts intentionally updated, and include screenshots or videos for visual/Isaac Sim behavior changes.

## Agent-Specific Instructions

Avoid committing generated `__pycache__`, temporary USD outputs, or unrelated simulation artifacts. If tests dirty generated files, restore only those artifacts and preserve user changes in source files.
