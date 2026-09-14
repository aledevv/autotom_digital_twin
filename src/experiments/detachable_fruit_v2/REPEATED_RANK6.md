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

## Manual result

User feedback: "mi pare che sia bella stabile". Window closed explicitly after
13.983334 simulated seconds / 47.920565 loop wall seconds. Monitor errors: none;
status awaiting_user_review. Mean GUI 31.81 FPS, after startup 31.64 FPS, p05
29.38 FPS. Real-time factor 0.292: visual frame cadence is distinct from simulated
time speed. Positive observed stability/interaction reference, but the requested
60 simulated-second full validation has not been completed. Event summary and
feedback are preserved in repeated_rank6_results.json.

## Second manual trial

Fresh GUI gui-freeze-rpl5yne3, same settings. User: "a me sembra stabile".
Explicit window close after 34.600002 simulated seconds / 62.611739 loop wall
seconds. Four real joint breaks, all attributed to the selected fruit drag;
no monitor errors. Mean FPS 33.16, after startup 33.22, p05 30.45. Real-time
factor 0.553. Process exits 0. This second positive manual observation strengthens
the five-truss reference, but does not complete a continuous 60 simulated-second
trial. Preserve the measured cadence and simulation speed as separate quantities;
the first trial's 0.292 real-time factor is not a fixed property of this setup.
