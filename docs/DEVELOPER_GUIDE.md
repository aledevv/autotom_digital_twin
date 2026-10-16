# Developer guide

A short tour of the project for a developer who has never seen it. It takes
about ten minutes to read; the detailed references are linked at the end.
Coding agents should read [`AGENTS_GUIDE.md`](AGENTS_GUIDE.md) instead.

## 1. What the project is

AutoTom Digital Twin builds an interactive tomato plant for NVIDIA Isaac Sim.
It reads a **plant skeleton** (a graph of straight segments with leaflets and
fruits) and writes an OpenUSD scene in which every segment is a rigid body,
joints bend under gravity and touch, and the plant can be pushed and dragged
with the mouse.

The skeleton can come from any growth model. The reference source is a GroIMP
simulation of a dwarf tomato, converted by a small adapter.

![Plant rendered in Isaac Sim](../assets/gallery/Screenshot%20from%202026-08-20%2014-14-53.png)

## 2. Quick start

Requirements: Linux, [uv](https://github.com/astral-sh/uv) (Python 3.14 is
installed by uv), and Isaac Sim at `~/isaacsim` to open the scene (set
`ISAACSIM_DIR` if it is elsewhere). Building the USD file does not need Isaac
Sim.

```bash
git clone --recurse-submodules https://github.com/aledevv/autotom_digital_twin
cd autotom_digital_twin
./run_main.sh                    # input/skeleton.json -> output/plant.usda -> Isaac Sim
./run_main.sh --generate-only    # only write the USD file
./run_main.sh --headless         # 5 s physics check without the GUI
```

`--recurse-submodules` fetches the leaflet shape generator
(`external/real_leaves`). In an existing clone run
`git submodule update --init`. Without it, set `leaves.generator: simple` in
`config/visuals.yaml`.

A successful build prints a summary such as
`[OK] 216 segments -> 216 rigid bodies, 206 D6 joints, 131 leaflets, 72 fruits (scale 2)`.

## 3. How the pipeline works

```text
growth model ──adapter──> input/skeleton.json ──exporter──> output/plant.usda ──> Isaac Sim
 (GroIMP, ...)            plant_skeleton/1.0    + config/physics.yaml
                                                + config/visuals.yaml
```

1. **Adapter.** Converts the growth model's output into `plant_skeleton/1.0`:
   one node per segment, each with a kind (`stem`, `branch`, `petiole`,
   `leaf_rachis`, `truss_rachis`, `pedicel`), plus leaflets and fruits.
2. **Branches.** The exporter groups nodes into chains (trunk, side shoots,
   leaves, trusses). Each node becomes one rigid link with capsule colliders.
3. **Joints.** Links are connected by D6 joints whose stiffness comes from the
   segment's Young's modulus, radius and length (`config/physics.yaml`). The
   main stem is welded with fixed joints so the plant stands.
4. **Joint budget (optional).** Isaac Sim handles about 220 bending joints per
   plant; above that the optimizer can lock or merge the least important ones.
5. **Visuals.** Organic stem meshes, leaflet blades, curved pedicels, tomato
   spheres with a calyx. Visuals never change the physics.
6. **Checks.** Colliders that overlap at rest are filtered, and a manifest JSON
   next to the USD file records counts, poses, masses and settings.

## 4. Repository map

| Path | What it is |
| --- | --- |
| `run_main.sh` | The one launcher: build, then open in Isaac Sim. |
| `input/skeleton.json` | Default plant (GroIMP day 160). `input/examples/small_plant.json` is a 16-segment hand-written example. |
| `config/physics.yaml`, `config/visuals.yaml` | All settings, each option commented with its default. |
| `src/plant_skeleton/` | The input format: data model, JSON I/O, validation. |
| `src/exporterV2/` | The exporter: pipeline, adapters, mesh generators, Isaac Sim launcher. |
| `src/exporterV2/adapters/` | Converters from other programs; `groimp.py` is the example. |
| `src/groimp_bridge/`, `src/plant_state/`, `model/`, `data/plant_states/` | GroIMP side: the growth model, its API client and its daily snapshots. Only needed to produce new GroIMP skeletons. |
| `external/real_leaves/` | Leaflet shape generator (git submodule). |
| `output/` | Generated files, not tracked. |

Things you can ignore at first: `src/exporterV2/legacy_csv/` (old builder kept
as a test fixture), `src/exporterV2/demos/`, `src/SKINNING/` (research notes),
and the diagnostic scripts next to `isaac_app.py`.

## 5. Common tasks

**Use another GroIMP day.**

```bash
uv run python -m exporterV2.adapters.groimp --day 80    # rewrites input/skeleton.json
./run_main.sh
```

Snapshots exist for days 1, 10, 25, 50, 80 and 160. New ones are extracted
from a running GroIMP with `./extract_plant_states.sh --day N`
(see `data/plant_states/README.md`).

**Load a plant from another program.** Write the skeleton directly, or add an
adapter next to `adapters/groimp.py`. The step-by-step guide is
[`src/exporterV2/adapters/README.md`](../src/exporterV2/adapters/README.md);
the format is in [`src/plant_skeleton/README.md`](../src/plant_skeleton/README.md).

**Change how the plant looks.** Edit `config/visuals.yaml` (mesh density,
leaflet generator, decorations, tomato size, calyx). To change a mesh
generator itself, find it in
[`src/exporterV2/visuals/README.md`](../src/exporterV2/visuals/README.md).

**Change the physics.** Edit `config/physics.yaml`: one block per segment kind
(joint type, Young's modulus, damping, density, bend limit), fruit mode,
solver iterations and `global_scale`. For a much smaller or larger plant,
start with `global_scale: auto`.

**Run the tests.** There is no CI; run them by hand before merging:

```bash
uv run pytest -q \
  src/plant_skeleton/tests \
  src/exporterV2/tests \
  src/exporterV2/core/optimizations/tests \
  src/exporterV2/core/skinning/tests \
  src/exporterV2/legacy_csv/tests \
  src/groimp_bridge/tests \
  tests
```

Interaction in Isaac Sim (dragging, bending, fruit detachment) is checked by
hand with `./run_main.sh`.

## 6. Versions and branches

| Where | What |
| --- | --- |
| This version (branch `claude/project-thread-eryl3b`) | The simple, generic exporter described here: skeleton input, two YAML configs, five launcher flags. |
| `main` | The thesis version: GroIMP-only input, exporters V1 and V2, and the main scene with five hand-built trusses (`./run_mainV2.sh`). |
| `thesis-archive` | All thesis experiments, notes and old outputs. Never delete it. |

Old branches are kept on purpose; do not delete them.

## 7. Known limitations

- Physics parameters are tuned by eye in Isaac Sim, not calibrated on a real
  plant.
- `global_scale: 2` is needed for the GroIMP tomato: PhysX is unstable with
  bodies thinner than about 2 mm.
- Physical tomatoes (`fruits.mode: physical`) are stable only on trusses of the
  main stem. The default is `visual`: tomatoes are drawn but have no mass.
- Fruits are spheres only.
- Only one plant per scene has been tested.

## 8. Future work

- **Visual details.** Plant tips and terminal shoots can look more natural.
- **PhysX time-step bug.** PhysX seems to have a bug related to the time step
  (dt), also visible in the public PhysX source. To confirm it, reproduce it in
  a minimal scene or build PhysX with the fix.
- **Tomatoes on side branches.** They make the plant unstable. A PhysX
  TGS-on-GPU bug lowers the effective break threshold to about
  `breakForce / N` (N = solver iterations), which explains why fruits on side
  trusses fall off on their own. It does not explain the 13-24 cm sagging of the
  side branches, which also happens with the PGS solver. Evidence is on branch
  `thesis-archive`, in `src/experiments/detachable_fruit_v2/*.md` and
  `src/experiments/break_threshold_probe/REPORT.md`.
- **Several plants in one scene.** Not tested in Isaac Sim yet. It should be
  possible to turn collisions and physics on or off per plant.
- **Branch generator.** There is no generator that builds a branch from a
  length and a number of joints; GroIMP has always provided complete branches.

## 9. Further reading

- [`src/plant_skeleton/README.md`](../src/plant_skeleton/README.md): input format with examples.
- [`src/exporterV2/README.md`](../src/exporterV2/README.md): pipeline steps, folder map, what to edit.
- [`src/exporterV2/adapters/README.md`](../src/exporterV2/adapters/README.md): writing an adapter.
- [`src/exporterV2/visuals/README.md`](../src/exporterV2/visuals/README.md): every mesh generator.
- [`src/exporterV2/docs/`](../src/exporterV2/docs/): physics and mechanics, collisions, segmented visuals, leaf shapes, troubleshooting.
- [`src/exporterV2/core/optimizations/docs/`](../src/exporterV2/core/optimizations/docs/): joint-budget optimizer.
