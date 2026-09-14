# Input-size fruit and support density controls

Experimental main-derived one-fruit scene only; neither main nor groPy is edited.

Source: `2026-09-10/native-comparison/main/code/data/simulation_output/dynamic_output/graphs/graph_day_160.csv`, day160/rank6, seventh fruit radius 0.012486447 m. The corresponding `direct/build.json` records mass 0.008154648458 kg. The source builder applies GLOBAL_SCALE=2 to sphere radius while preserving mass. The resulting sphere radius 0.024972894 m and mass 0.008154648356 kg imply density 125 kg/m³.

`prepare_support_matrix.py --coherent-fruit` preserves the loaded fruit mass and derives sphere radius at density1000 kg/m³ (0.012486446948 m in this case, consistent with input). It updates the visual/collision sphere, extent, analytical solid-sphere inertia, fruit center and fruit-side attachment offset. The parent joint frame, world attachment, 2mm visual overlap, breakForce6N and all drives remain unchanged. Support density changes independently rescale support mass/inertia, without changing geometry or drives. Stem properties remain unchanged. This is an opt-in scene correction, not a global scale change to the production builder.

Validation: eight support-control tests pass, including a rotated-body regression checking mass, density, inertia and world attachment preservation. Static scene audit passes and native runtime mass/inertia/DOF checks run for each case.

| Case | Support density | Fruit | 20s headless |
|---|---:|---|---|
| `coherent-fruit-1000-screen` | 1000 | input-size | Failed: spontaneous JOINT_BREAK within0.20s |
| `coherent-fruit-2000-screen` | 2000 | input-size | Passed20s, no breaks/errors |
| `density-1000-only-screen` | 1000 | original enlarged sphere | Failed: spontaneous JOINT_BREAK within0.20s |

The successful reduced case is ready for manual GUI inspection; headless success does not establish interactive stability. Do not combine density reduction with size correction as a validated default.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --coherent-fruit --support-density 2000
```

`--coherent-fruit` alone requests the combined1000 kg/m³ variant, which failed screening. All heavy artifacts remain local and every launch prepares a fresh scene. Native mouse, TGS/GPU60Hz,32/4 and32/1 iterations and6N detachment remain unchanged.

Support density1000 fails both with the original sphere and the corrected sphere. This implicates density reduction under the current runtime; it does not establish a general minimum stable density.
