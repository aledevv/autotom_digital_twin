# Troubleshooting

## Import Errors

Run ordinary tests from the project root so `src/` imports resolve:

```bash
uv run pytest src/exporterV2 -v
```

If a test needs `PhysxSchema`, run it with Isaac Sim Python:

```bash
~/isaacsim/python.sh -m pytest src/exporterV2/core/usd/tests -v
```

## Interactive FPS vs validation rate

ExporterV2 authors and validates physics at `480 Hz`, but the interactive GUI
defaults to `60 Hz`, matching the effective runtime of the legacy V2 loader.
The rates are intentionally separate:

```bash
# Fluid interactive inspection (default: 60 Hz)
./run_mainV2.sh --day 50 --interactive-physics-hz 60

# High-rate finite validation (always uses --physics-hz)
./run_mainV2.sh --day 50 --headless --duration 5 --physics-hz 480
```

Interactive alternatives are `120`, `240`, and `480 Hz`. Increasing the GUI
rate adds respectively 2, 4, or 8 PhysX substeps to every 60 Hz render update.
Do not diagnose this cost from joint count alone: compare stages at the same
runtime timestep.

The repeatable comparison tool reports USD complexity, render updates/s and
raw PhysX steps/s for both stages:

```bash
uv run python -m exporterV2.performance_benchmark \
  --baseline legacy=/tmp/tree_v2_day_50_legacy.usda \
  --candidate plantstate=/tmp/tree_v2_day_50_leaves.usda \
  --physics-hz 60,120,240,480 \
  --output /tmp/v2_day50_performance.json
```

On Isaac Sim 4.5, constructing `World` after opening an existing stage can
leave `SimulationManager` unaware of its already-authored `PhysicsScene`; its
rate getter then silently returns 60 Hz. The loader synchronizes that runtime
registry explicitly and reapplies the selected cadence after `World.reset()`.
Do not remove this compatibility shim merely because the USD attribute reads
480 Hz: authored and effective runtime rates are separate checks.

Solver iterations remain centralized in `PhysicsRuntimeConfig`; changing them
affects generated USD metadata and must be documented with the output.

## Tomatoes Fall Immediately

Check `TrussPhysicsConfig.TOMATO_DETACHMENT_BREAK_FORCE_N`. If it is below the static and dynamic load from the tomato, the FixedJoint can break as soon as simulation starts. Current default is `6.0 N`.

## Tomatoes Do Not Detach

Verify that detachment is enabled and serialized:

- `TOMATO_DETACHMENT_ENABLED = True`
- tomato body under `/World/TerminalBodies`
- FixedJoint has a finite `breakForce`
- `physics:excludeFromArticulation = True`
- the interacting object actually applies enough impulse to exceed the break force

## Collision Instability Near Tomatoes

Inspect `FilteredPairs` on the tomato body. Detachable tomatoes should filter collisions toward their pedicel and related truss rachis. Missing filters can make contact resolution fight the FixedJoint.

## Demo Paths

Visual and Isaac demo scripts now live under `src/exporterV2/demos/`. Root runner scripts are intentionally limited to `run_mainV1.sh` and `run_mainV2.sh`.
