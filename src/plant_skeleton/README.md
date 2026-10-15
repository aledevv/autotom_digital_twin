# plant_skeleton/1.0

The input format of the Isaac Sim exporter. A plant is a **graph of straight
segments** (nodes), plus the **leaflets** hanging from leaves and the
**fruits** hanging from pedicels. Nothing in the format is specific to GroIMP.

`run_main.sh` reads `input/skeleton.json` by default. The smallest complete
example is [`input/examples/small_plant.json`](../../input/examples/small_plant.json)
(16 segments, 6 leaflets, 3 tomatoes, written by hand).

Units are metres and kilograms. Z is up. The root segment starts at the ground.

## File layout

```json
{
  "schema_version": "plant_skeleton/1.0",
  "metadata": {"plant_id": 1, "source": "my-module", "description": "", "time": 42.0},
  "nodes":    [ ... ],
  "leaflets": [ ... ],
  "fruits":   [ ... ]
}
```

`metadata` is optional. `leaflets` and `fruits` may be empty or missing.

## Nodes (segments)

One node is one straight piece of the plant, typically one internode.

```json
{"id": "stem2", "parent": "stem1", "kind": "stem",
 "start": [0.0, 0.0, 0.1], "end": [0.0, 0.0, 0.2], "radius": 0.0055}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `id` | yes | Unique string. It is written into the USD (`autotom:skeletonNodeId`). |
| `parent` | yes | Id of the node it grows from, or `null` for the single root. |
| `kind` | yes | One of the kinds below. |
| `start`, `end` | yes | World positions in metres. `start` is normally the parent's `end`. |
| `radius` | yes | Metres. |
| `frame` | no | 3x3 rotation, rows of a matrix whose columns are local X (left), Y (up), Z (along the segment). Derived from the direction when missing. |
| `axis` | no | Groups consecutive nodes into one branch. See "How nodes become branches". |
| `organ` | no | Name of the biological organ (for example a leaf id), kept for bookkeeping. |

### Kinds and allowed parents

| Kind | What it is | Allowed parent kinds |
| --- | --- | --- |
| `stem` | main stem | none (root) or `stem` |
| `branch` | side shoot | `stem`, `branch` |
| `petiole` | leaf stalk, first segment of a leaf | `stem`, `branch` |
| `leaf_rachis` | leaf midrib, carries leaflets | `petiole`, `leaf_rachis` |
| `truss_rachis` | stalk of a tomato truss | `stem`, `branch`, `truss_rachis` |
| `pedicel` | small stalk holding one tomato | `truss_rachis` |

Each kind has its own physics block in `config/physics.yaml` (`segments.<kind>`).

### How nodes become branches

The exporter groups nodes into chains; each chain becomes one articulated
branch of rigid links in Isaac Sim (one link per node).

1. All `stem` nodes form the trunk.
2. Nodes sharing an `axis` value form one chain. That chain must not fork.
3. Without `axis`, a node continues its parent's chain when it has the same
   kind and is the parent's only child of that kind.
4. A child chain attaches to the **end** of its parent node.

Use `axis` when a branch forks into two same-kind children and you want to
choose which one continues the branch (the GroIMP adapter does this for side
shoots).

## Leaflets

A leaflet is a small stalk (petiolule) from `start` to `end`; the exporter
generates the blade mesh at its end.

```json
{"id": "leafA_left1", "node": "leafA_rachis1", "side": "left",
 "start": [0.0, 0.12, 0.34], "end": [-0.015, 0.12, 0.34], "radius": 0.0008,
 "inclination_deg": 60, "mass_kg": 0.002}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `id` | yes | Unique string. |
| `node` | yes | The `petiole` or `leaf_rachis` it grows from. |
| `side` | yes | `left`, `right` or `terminal` (the tip leaflet). |
| `start`, `end`, `radius` | yes | Petiolule geometry; the length sets the blade size. |
| `frame` | no | Used only with `pose: canonical` in `config/visuals.yaml`. |
| `inclination_deg` | no | Angle from the rachis in the default pose (90 when missing). |
| `mass_kg` | no | Added to the mass of the host segment (default 0). |

## Fruits

```json
{"id": "tomato2", "node": "pedicel2", "radius": 0.015, "ripeness": 0.8}
```

| Field | Required | Meaning |
| --- | --- | --- |
| `id` | yes | Unique string. |
| `node` | yes | The `pedicel` it hangs from (one fruit per pedicel). |
| `radius` | yes | Metres; the fruit is a sphere. |
| `ripeness` | no | 0 = green, 1 = red (default 0). |
| `center` | no | Used only with `pose: canonical`; otherwise the sphere hangs at the pedicel tip. |

## Validation

`load_skeleton(path)` checks the file and raises with every problem listed,
for example:

- `exactly one node must have parent null, found [...]`
- `node 'x' has unknown kind 'leaf'; allowed: [...]`
- `node 'x' has missing parent 'y'`
- `leaflet 'l' must hang from a petiole or leaf_rachis, not 'stem'`
- `pedicel 'p' carries two fruits (...)`
- `fruit 'f' ripeness must be between 0 and 1`

`skeleton_warnings(skeleton)` reports suspicious but legal things, such as a
child that does not start at its parent's end.

## Python API

```python
from plant_skeleton import load_skeleton, save_skeleton, PlantSkeleton, SkeletonNode

skeleton = load_skeleton("input/skeleton.json")
print(len(skeleton.nodes), skeleton.root.id)
save_skeleton(skeleton, "copy.json")
```

To produce this format from another source, write an adapter; see
[`src/exporterV2/adapters/README.md`](../exporterV2/adapters/README.md).
