# Canonical PlantState inputs

ExporterV1 resolves serverless inputs here using:

```text
plant_state_day_N.json
plant_state_day_N_plant_P.json  # plant IDs other than 1
```

Generate or refresh a snapshot explicitly with the GroIMP extractor:

```bash
uv run python -m groimp_bridge.extractor \
  --project model/project_bridge.gsz \
  --steps N --plant-id P \
  --output data/plant_states/plant_state_day_N.json
```

For one day or consecutive days, use the lifecycle-safe wrapper:

```bash
./extract_plant_states.sh --day 50
./extract_plant_states.sh --from-day 1 --to-day 160 --skip-existing
```

The script checks the GroIMP API first, advances one isolated simulation in
order, validates each canonical state, and atomically writes
`plant_state_day_N.json`. Without an explicit policy it refuses the complete
range before starting if a destination already exists. Use `--overwrite` for
a clean regeneration, or `--skip-existing` to resume safely after an
interruption.
The detailed rationale and recovery procedure are in
[`docs/GROIMP_BATCH_EXTRACTION.md`](../../docs/GROIMP_BATCH_EXTRACTION.md).

Important: GroIMP 2.2.1 headless may return `/home/<user>/` from `getWD()`
instead of the opened project directory. The bridge therefore rewrites
`PATH_INPUT` and `PATH_OUTPUT` only inside its disposable GSZ copy before
`openWB`. Do not remove this runtime override: without it,
`parameters_derived.rgg` fails around its `listFiles()[0]` initialization and
GroIMP can leave an empty, unresponsive workbench behind.

Once generated, the JSON contains everything ExporterV1 needs. Neither
`python -m exporterV1` nor `run_mainV1.sh` contacts GroIMP or falls back to CSV.
