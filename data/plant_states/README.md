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

Once generated, the JSON contains everything ExporterV1 needs. Neither
`python -m exporterV1` nor `run_mainV1.sh` contacts GroIMP or falls back to CSV.
