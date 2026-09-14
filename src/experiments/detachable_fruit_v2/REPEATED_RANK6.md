# Five direct trusses with the accepted rank-6 fruit profile

The user requested an immediate replication test of the stable reference. Keep
the five original main-derived direct trusses and full stem, including attachment
locations and orientations. Assign each truss the eight rank-6 fruit masses by
pedicel suffix, and coherent spherical dimensions/inertia at 1000 kg/m3. Preserve
world attachment anchors. Each truss carries 74.708532 g of fruit, total 373.54266 g.
This is a replicated load profile, not the original day-160 mass distribution.

TGS/GPU, 60 Hz, articulation 32/4, fruit 32/1, native joint mouse coefficient 10,
6 N detachment from startup, main support density 20000 kg/m3 and original drives.
No solver changes. Source: multiple-trusses/five-screen; reference profile:
full-truss-return-main/geometry. Tools record reference/source hashes, attribute
changes and fruit geometry/anchor corrections.

Headless five-rank6-profile: 110 bodies, 40 fruits; functional pass for 20 seconds,
zero breaks and zero effective property errors, process exit 0. Strict angular
and velocity advisories remain diagnostic, not evidence of manual acceptance.

GUI opened as gui-freeze-lmkcp0e6 with no automatic deadline. Manual feedback and
full-cycle FPS acceptance are pending. Heavy evidence remains local. Summary:
repeated_rank6_results.json.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --source-case artifacts/detachable_fruit_v2/multiple-trusses/five-rank6-profile \
  --coherent-fruit --support-density 20000 --fruit-break-force 6
```
