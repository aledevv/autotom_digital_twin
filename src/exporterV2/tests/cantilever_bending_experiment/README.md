# Cantilever Bending Experiment

This experiment validates articulated branch physics in two separate ways:

- `legacy_current` vs `new_physics`: pre/post behavior under identical benchmark inputs.
- simulation vs beam theory: realism sanity check against declared Euler-Bernoulli benchmarks.

The quantitative entrypoint is:

```bash
~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py all
```

The convenience script runs the pure Python formula check first, then the Isaac Sim workflow:

```bash
./src/exporterV2/tests/cantilever_bending_experiment/run_experiment.sh
```

Outputs are written to:

```text
src/exporterV2/tests/cantilever_bending_experiment/results/
data/usd_models/physics_tests/
```

## Benchmarks

`synthetic_solid_40cm`

- Solid cylinder, `L=0.40 m`, `r=0.010 m`, `E=100 MPa`, `rho=1000 kg/m3`.
- Expected self-weight deflection: about `12.56 mm`.

`tomato_gao_20cm`

- Hollow tomato stalk approximation from Gao et al. 2024.
- `L=0.20 m`, `d_o=11.1 mm`, `d_i=3.82 mm`, `E=50.64 MPa`, `rho=769.96 kg/m3`.
- Expected self-weight deflection: about `3.46 mm`.
- Expected `0.05 N` tip-force deflection with gravity off: about `3.58 mm`.

These are sanity checks, not absolute proof of a living tomato plant. The report must keep failures and non-settled runs visible.

## Commands

Pure Python formula check:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py formula-check
```

Generate USD only:

```bash
~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py generate
```

Audit existing USD only:

```bash
uv run python src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py audit
```

Run quantitative simulation on existing USD:

```bash
~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/cantilever_validation.py simulate
```

Visual pre/post comparison:

```bash
~/isaacsim/python.sh src/exporterV2/tests/cantilever_bending_experiment/compare_cantilevers.py --benchmark tomato_gao_20cm --n 10
```

Legacy script names remain as wrappers, but `cantilever_validation.py` is the source of truth for quantitative measurements.
