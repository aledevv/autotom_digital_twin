# Main-scene trusses

The five trusses of the main scene (`./run_mainV2.sh`), read by `src/exporterV2/main_scene.py`.

| File | Content |
| --- | --- |
| `five_trusses.usda` | Fixed stem with the original main-stem trusses of ranks 6-10 (20 bodies each: rachis links, pedicels, tomatoes). Only the truss bodies and their materials are copied into the main scene. |
| `five_trusses_mass.json` | Mass, centre of mass and inertia of those 100 bodies as loaded by PhysX; written explicitly on the grafted bodies. |
| `runtime.json` | Isaac Sim settings of the main scene: PGS on CPU, 60 Hz, 32/4 articulation and 32/1 fruit iterations, 3 N break force, native mouse force 50. |

These come from the thesis experiment campaign (`original-rank-campaign/five-rest`,
see `src/experiments/detachable_fruit_v2/` on branch `thesis-archive`). They
were built by hand from an older exporter and tested in Isaac Sim, so they are
kept as data instead of being regenerated. Fruit sizes are the GroIMP ones;
the truss support density (20000 kg/m3) is artificial and was tuned for
stability, not measured.
