# Recovered successful main-derived native gestures

The positive reference `gui-freeze-rpl5yne3` retains its complete native event
log and four user-attributed JOINT_BREAK events. Settings match the intended
native reference: TGS/GPU, 60 Hz, joint mouse mode, coefficient 10, break force 6 N.
`successful_native_gestures.json` preserves the four raw gesture clips, source
hashes, event times and diagnostic projections. It is an observation dataset,
not a new physics or mouse preset.

| Fruit | Time from pick to break | Last prebreak projected displacement | Peak projected speed |
| --- | ---: | ---: | ---: |
| rank 8, lat_3_R | 0.350 s | 0.566 m | 3.78 m/s |
| rank 7, lat_3_L | 0.300 s | 0.742 m | 6.13 m/s |
| rank 7, lat_0_R | 0.150 s | 0.561 m | 9.47 m/s |
| rank 8, lat_2_R | 0.217 s | 0.778 m | 7.85 m/s |

The recent synthetic comparison moved its target 0.20 m over 5 s (0.04 m/s).
It therefore did not reproduce the successful gestures' amplitude or timing.
Its failures cannot by themselves establish that native picking or a solver
cannot detach fruit. The recorded gestures suggest amplitude/rate as concrete
factors to test, without establishing which one caused detachment.

Projection means intersecting each recorded ray with a plane through the first
hit, perpendicular to the first ray. It is not hand travel, fruit travel, a
measurement of force, or a verified reconstruction of PhysX's internal target.
Timestamp resolution is limited by physics-step sampling. A later replay must
verify selection, especially when moving a clip to a different fruit or pose.
Earlier interactions changed the reference before these picks, so replaying an
isolated clip from a fresh scene is a diagnostic adaptation, not an identical
reproduction of the original session.

No new simulation or parameter change was made in this recovery/comparison step.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/extract_successful_native_gestures.py \
  artifacts/detachable_fruit_v2/gui-freeze-rpl5yne3 \
  --output /tmp/successful-native-gestures.json
```
