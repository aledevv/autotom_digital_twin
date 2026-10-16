# Extracting plant states from GroIMP

Always use `extract_plant_states.sh` to create the `plant_state/1.0` JSON
snapshots that the GroIMP adapter reads. The script uses no CSV files and does
not modify the project or its original inputs.

## Commands

One day:

```bash
./extract_plant_states.sh --day 50
```

Every day from 1 to 160, replacing existing files:

```bash
./extract_plant_states.sh --from-day 1 --to-day 160 --overwrite
```

Safe resume after an interruption, keeping the days already done:

```bash
./extract_plant_states.sh --from-day 1 --to-day 160 --skip-existing
```

Files are written to `data/plant_states/plant_state_day_N.json`. Use
`--output-dir PATH` to write elsewhere and `--plant-id N` for another plant.

Extracting day `N` always starts from day 1 and runs every step up to `N` in
order: the intermediate dynamic state cannot be skipped. For a range, the
workbench is compiled once, so one batch is both more correct and faster than
160 separate calls, and native node ids stay consistent within the run.

## Why the script does not open the GSZ directly

In headless GroIMP 2.2.1, `getWD()` does not necessarily return the directory
of the opened `.gsz` file; it can return the user's home. During static
initialisation, before the bridge can compile the project,
`parameters_derived.rgg` then reads the wrong `input/` folder. Observed symptoms
include `listFiles()[0]` errors, `NoClassDefFoundError: parameters_derived` and
the message `Infinity Removed`. This is not something the user must reset, and
it does not mean the original GSZ is corrupt.

The runtime fixes it before `openWB`:

1. copy the GSZ and `model/input/` into a temporary folder;
2. rewrite `PATH_INPUT` and `PATH_OUTPUT` only inside that copy of the GSZ;
3. open, compile and step one isolated workbench;
4. check the observed day after every step;
5. validate the `PlantState` and publish each JSON with an atomic replace;
6. always close the workbench, also on error or interruption.

The fix is `groimp_bridge.runtime.configure_isolated_paths`. Do not remove or
move this call: after `openWB` it would be too late for the static initialiser
of `parameters_derived.rgg`.

## On failure

The command exits with a non-zero code and prints the expected day and the day
GroIMP reported when they differ. JSON files already published stay valid; the
file of the day in progress is never left half-written. Restart the GroIMP
server only if it stops responding, then run the batch again with
`--skip-existing`.

Do not combine `--overwrite` and `--skip-existing`: the first allows files to
be replaced, the second requires them to be kept.
