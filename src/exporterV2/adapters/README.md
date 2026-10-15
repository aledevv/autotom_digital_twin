# Adapters

An adapter turns another module's plant description into
[`plant_skeleton/1.0`](../../plant_skeleton/README.md), the only format the
exporter reads. Everything specific to a source (its ids, its quirks, its
units) belongs in the adapter, so the exporter stays the same for every plant.

```text
your module's output  --adapter-->  input/skeleton.json  --run_main.sh-->  output/plant.usda  -->  Isaac Sim
```

If the other module can write `plant_skeleton/1.0` directly, no adapter is
needed: save the file as `input/skeleton.json`.

## The GroIMP adapter (reference example)

[`groimp.py`](groimp.py) converts the GroIMP `plant_state/1.0` snapshots in
`data/plant_states/` into a skeleton:

```bash
uv run python -m exporterV2.adapters.groimp --day 160      # writes input/skeleton.json
uv run python -m exporterV2.adapters.groimp --day 50 --output input/day50.json
./run_main.sh --input input/day50.json
```

What it does, in order (`plant_state_to_skeleton`):

1. **Stem and side shoots** (`_add_structure`): order-0 internodes become
   `stem`, higher orders become `branch` with an explicit `axis` per shoot.
   Exact duplicate organs that GroIMP sometimes emits are dropped.
2. **Leaves** (`_add_leaves`, `_add_leaflets`): the GroIMP petiole axis
   becomes the `petiole` node, its rachis axes `leaf_rachis` nodes; each
   leaflet gets a side and an inclination, attached to the closest support;
   leaf dry biomass (mg) is split over the leaflets as `mass_kg`.
3. **Trusses** (`_add_trusses`): rachis segments become `truss_rachis`,
   each pedicel one `pedicel` node (drawn 3x longer, `PEDICEL_LENGTH_SCALE`,
   so big tomatoes do not crowd the rachis), and each tomato a fruit with its
   ripeness.
4. Zero-size organs (young leaves, empty trusses) are skipped.
5. The result is checked with `validate_skeleton` before it is saved.

Its tests (`tests/test_groimp_adapter.py`) are a good template: segment counts
per day, counts per kind, leaflet sides, and a check that the committed
`input/skeleton.json` equals the adapter output.

## Writing a new adapter

1. Create `adapters/<source>.py` with one public function that returns a
   `PlantSkeleton`, and a small `main()` that saves it, like `groimp.py`.
2. Map every segment of your plant to one node:
   - pick its `kind` from `stem`, `branch`, `petiole`, `leaf_rachis`,
     `truss_rachis`, `pedicel`;
   - set `parent` to the segment it grows from, and `start` to the parent's
     `end`;
   - use metres, Z up, and put the root at the origin.
3. Add leaflets (on `petiole` or `leaf_rachis` nodes) and fruits (one per
   `pedicel`) if your source has them. Both are optional.
4. Leave `frame` out unless your source has meaningful orientations; the
   exporter derives one from each segment's direction.
5. Use `axis` only where a branch forks into two same-kind children and you
   need to choose which one continues the branch.
6. Call `validate_skeleton(skeleton)` and `skeleton_warnings(skeleton)`;
   fix everything they report.
7. Export and look at it:

```bash
uv run python -m exporterV2.adapters.<source> --output input/skeleton.json
./run_main.sh --generate-only      # writes output/plant.usda and prints the counts
./run_main.sh                      # opens it in Isaac Sim
```

Minimal skeleton built in Python:

```python
from plant_skeleton import PlantSkeleton, SkeletonNode, SkeletonFruit, save_skeleton, validate_skeleton

nodes = (
    SkeletonNode("s1", None, "stem", (0, 0, 0), (0, 0, 0.1), 0.006),
    SkeletonNode("s2", "s1", "stem", (0, 0, 0.1), (0, 0, 0.2), 0.005),
    SkeletonNode("t1", "s2", "truss_rachis", (0, 0, 0.2), (0.03, 0, 0.18), 0.0015),
    SkeletonNode("p1", "t1", "pedicel", (0.03, 0, 0.18), (0.03, 0, 0.165), 0.0008),
)
skeleton = PlantSkeleton(nodes, fruits=(SkeletonFruit("f1", "p1", 0.012, ripeness=1.0),))
validate_skeleton(skeleton)
save_skeleton(skeleton, "input/skeleton.json")
```

## When the plant is very different in size

Physics settings are in `config/physics.yaml`. For a much smaller or larger
plant, start with `global_scale: auto` (keeps the thinnest segment at least
2 mm thick, which PhysX needs) and adjust the `segments.<kind>` blocks.
If the plant has more than about 220 bending joints, set `joints.optimize`
to `lock` or `full`.
