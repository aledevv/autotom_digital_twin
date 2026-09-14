# PGS lateral truss: controlled-force comparison

The native GUI review was stable but did not detach fruits. Its completed report
contains 197.65 simulated seconds, no joint breaks or physical errors, 56.51 steady
FPS and a simulated/real time ratio of 0.943. Multiple fruit picks were verified.
This is not an accepted native-interaction candidate.

The follow-up compares the same fruit (`lat_2_L` on `Truss_r5_o0_g421757`) with
all four lateral joints mobile versus all four fixed. Internal truss joints
remain mobile. Both start from the same authored scene, settle for 30 s, then
apply an upward world-Z force at the fruit COM, ramping 0–12 N in 5 s. The force
stops on the first target JOINT_BREAK; observation continues to 60 simulated s.
These are direct PhysX forces, not native mouse commands or estimates of mouse force.

Both use PGS/GPU, 60 Hz, articulation 32/4, fruits 32/1, collisions enabled and
6 N break force from startup. Expected loaded mass, inertia and COM checks cover
all bodies. No solver, mass, drive or mouse-gain changes were made after the GUI
feedback. The only scene difference between the two force tests is the four
branch joint constraints.

Heavy evidence: `artifacts/detachable_fruit_v2/standard-integration/lateral-force-pair/`.
Compact results: `lateral_force_pair_results.json`.

Reproduce with fresh output directories:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/prepare_lateral_force_pair.py \
  artifacts/detachable_fruit_v2/standard-integration/lateral-diagnosis-v2/pgs-minute \
  /tmp/lateral-force-pair
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_batch.py \
  /tmp/lateral-force-pair/mobile /tmp/lateral-force-pair/fixed
```

## Result

Both tests detach only the intended fruit at the same completed step, 32.550 s,
with 6.120004 N commanded force. Motion continuity checks pass, the force stops
on the next before-step call, and both complete 60 s without further breaks or
physical errors. This shows that controlled-force detachment works with the
mobile support; native drag usability remains unresolved. The next proposed
comparison is a repeatable native gesture on mobile/fixed supports, retaining
the same 6 N threshold and coefficient 10. No such extra replay was run here.
