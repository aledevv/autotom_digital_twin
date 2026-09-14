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
All five candidate configurations have now been used. No sixth configuration is authorized. Do not mark the goal achieved from a headless replay alone.

Physics remains PGS/GPU, 60 Hz, articulation 32/4, fruit 32/1, 6 N break force
from startup, original mobile branch, unchanged masses/geometry/collisions.
Attempts 4 and 5 additionally scale the four mobile lateral joints: stiffness/damping by 10/30 and 100/1000 respectively. No joints are fixed. This coefficient is not a force measured in newtons.

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
  artifacts/detachable_fruit_v2/standard-integration/native-attempts/05-force50-branch-k100-d1000
```

The GUI remains open until explicit close. Results and heavy traces stay in fresh
local directories. The active GUI path and all headless outcomes are in the ledger.
Validation to this point: 14 interaction tests and 10 targeted GUI-configuration
tests passed; real headless replay/slow-gesture runs and GUI startup were checked.

## Final two candidates and manual rejection of attempt 3

The user rejected attempt 3: the lateral branch felt excessively soft and the
window subsequently stopped. The report stopped at 34.1167 s on an unclassified
fruit break after an observer raycast miss; an engine crash is not established.

Attempt 4: target break around 30.40 s, functional 60 s pass. Truss attachment
vertical sag at 29 s remained 15.13 cm (original attempt 3: 17.81 cm). Insufficient
support improvement; no GUI acceptance claimed.

Attempt 5: target break at 30.3333 s with continuity check passed; no additional
breaks through 60 s. Functional gate passed, strict reported-velocity gate did
not (24.53 mm/s, 0.268 rad/s). Pose-derived tail motion was 0.0513 mm/s and
0.276 mm excursion. Sag worsened to 22.95 cm at 29 s. This is not an established
solution to branch support. Native force mode, coefficient 50, 6 N break force;
manual evaluation pending in gui-e91iqf84. All heavy traces remain local.

Final GUI outcome: user rejected attempt 5 as too soft. GUI stopped at 9.866667181253433 s; errors: ["joint broke outside the selected fruit drag: /World/TerminalBodies/Truss_r5_o0_g421757_rachis_pedicel_lat_2_L_tomato/TerminalBodyFixedJoint", "simulation ended before the requested 60-second validation completed"]. Five configurations exhausted; stop without promoting a candidate.
