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

## Truss Lag

The current V2 runtime uses `480 Hz` and GPU dynamics. If Isaac Sim feels slow, first test with fewer generated organs or lower day complexity. Solver iterations are centralized in `PhysicsRuntimeConfig`; changing them affects generated USD metadata and should be documented with the output.

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

Visual and Isaac demo scripts now live under `src/exporterV2/demos/`. Root runner scripts are intentionally limited to `run_main.sh` and `run_mainV2.sh`.
