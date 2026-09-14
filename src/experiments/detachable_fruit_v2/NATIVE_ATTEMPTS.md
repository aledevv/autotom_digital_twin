# Five-attempt native lateral-detachment goal

User limit: at most five candidate configurations, then stop. An attempt is a
candidate configuration with its headless and manual validation. Extra input
validation or correcting a launcher error does not create a new configuration.
`native_attempts.json` is the persistent ledger; `prepare_native_attempt.py`
refuses to prepare a sixth candidate. The accepted direct-stem reference remains
unchanged. Success requires native detachment on a mobile lateral branch,
stability/recovery, usable manual gestures, and >=20 GUI FPS with user approval.

## Current evidence

| Attempt | Native mode / coefficient | Recorded successful gesture on lateral branch |
| --- | --- | --- |
| 1 | joint / 10 | No detachment; 60 s otherwise stable |
| 2 | joint / 50 | No detachment; recorded states exactly equal to attempt 1 |
| 3 | force / 50 | Intended fruit detaches at 30.45 s; continuity passes, no other breaks through 60 s |

Attempt 3 also underwent a slower 0.2 m / 5 s gesture. It stayed stable for
60 s but did not detach; manual usability is therefore still a required gate.
Two candidate configurations remain available. Do not mark the goal achieved
from the successful rapid headless replay alone.

Physics remains PGS/GPU, 60 Hz, articulation 32/4, fruit 32/1, 6 N break force
from startup, original mobile branch, unchanged masses/geometry/collisions.
The only candidate controls so far are the built-in PhysX picking mode and
coefficient. This coefficient is not a force measured in newtons.

## Native-only boundary

`InteractionReplay.recorded_native_step` retargets recorded input rays by a
single translation matching the current fruit center to the source fruit center,
checks the selected collider, and sends begin/move/end to PhysX's original
`update_interaction`. It does not apply forces, delete joints, or implement
mouse dynamics. The input clip and source hash are saved in each config.
Different poses/prior interactions mean this is a retargeted diagnostic replay,
not an exact reproduction of the entire original main session.

Manual GUI launch clears `force_target` and removes `native_recording`; only
the user's native input drives the scene. The existing observer forwards each
native call unchanged and records it; its summary now identifies force mode
correctly rather than always labelling native mode as joint.

## GUI launcher correction

The first attempt-3 GUI process exited before simulation: `_configure_mouse_interaction`
rejected coefficient 50 although headless allowed it. This was a launcher
validation mismatch, not a physical crash. The ordinary GUI limit stays 10.
The explicit experiment flag `allow_experimental_native_coefficient` permits
finite positive values up to 100; this is a programmatic experiment outside
the native UI slider range, not a new mouse controller. Attempt 3 is reopened
with exactly the same physics and native coefficient, not counted as attempt 4.
Values such as 1000 remain rejected. Unit checks cover normal and experimental
configuration paths without introducing an unmonitored physics step.

## Reopen current candidate

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/standard-integration/native-attempts/03-force50
```

The GUI remains open until explicit close. Results and heavy traces stay in fresh
local directories. The active GUI path and all headless outcomes are in the ledger.
Validation to this point: 14 interaction tests and 10 targeted GUI-configuration
tests passed; real headless replay/slow-gesture runs and GUI startup were checked.
