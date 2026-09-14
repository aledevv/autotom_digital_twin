# Matched native drag on PGS lateral supports

Both native-drag runs finish 60 simulated seconds without joint breaks. Both
fail the required target-detachment check, with no other reported errors.
This reproduces the manual lack of detachment on the mobile branch and shows
that fixing the entire branch does not make this particular gesture detach.

| Support | Peak fruit-center movement during drag | Peak truss attachment movement | Detachment |
| --- | --- | --- | --- |
| Mobile | 6.10 mm | 5.48 mm | None |
| All four lateral joints fixed | 1.74 mm | <0.00004 mm | None |

Protocol: same `Truss_r5_o0_g421757` fruit `lat_2_L`, PGS/GPU, 60 Hz,
articulation 32/4, fruit 32/1, 6 N break threshold from startup. Rest 30 s,
then raise the ray's target point 0.2 m over 5 s, release, observe until 60 s.
The synthetic native gesture uses PhysX `update_interaction` begin/move/end,
not the custom mouse controller and not direct force injection. Mouse settings
are joint mode (`forceGrab=False`), `mouseGrab=True`, `pickingForce=10`.
It is a standardized replay, not the user's exact recorded mouse trajectory.

The independent raycast selects the requested fruit sphere in both runs from
the first +X viewing direction, 0.5 m from its settled center. View translations
follow the different settled fruit positions, while relative ray directions
agree within 1e-8. This verifies matched commands and the independent selection;
it is not a force measurement or an independent internal PhysX pick report.
Both fruits respond to the gesture, then return toward their earlier poses.
The coefficient and target displacement must not be labelled as applied newtons.

The previous controlled COM-force tests detached in both configurations at the
same 6.12 N command. Together these observations locate the unresolved behavior
in the native gesture / force transmission under this configuration, rather
than demonstrating that a mobile branch inherently cannot support detachment.
No threshold, gain, mass or solver changes were made during this pair.

Next proposed control: the same fixed-support scene and native gesture with
TGS versus PGS, changing only solver. Fixed support has already passed its TGS
rest screen, making this a bounded way to compare native response. This further
comparison has not been run here, and neither higher mouse gain nor lower break
threshold is being promoted as a solution.

## Evidence and reproduction

Heavy files remain local at
`artifacts/detachable_fruit_v2/standard-integration/lateral-native-pair/`.
`lateral_native_pair_results.json` records selections, commands, pose response,
reports, hashes and unknown native force explicitly as null.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/prepare_lateral_force_pair.py \
  artifacts/detachable_fruit_v2/standard-integration/lateral-diagnosis-v2/pgs-minute \
  /tmp/lateral-native-pair --interaction native
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_batch.py \
  /tmp/lateral-native-pair/mobile /tmp/lateral-native-pair/fixed
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/analyze_lateral_native_pair.py \
  /tmp/lateral-native-pair --output /tmp/lateral-native-pair/summary.json
```

The preparer retains COM force as its default mode for previous commands.
Validation consists of both real Isaac runs, matched-ray comparison, loaded
body-property checks, Python compilation and diff whitespace checks.

## Follow-up: fixed support, TGS versus PGS

The same native gesture was executed with TGS, keeping the branch fixed.
Verified config differences contain only `solver`; the authored USD difference
is only `/World/PhysicsScene.physxScene:solverType`. GPU, 60 Hz, iterations,
6 N, coefficient 10 and gesture remain unchanged. Replay ray directions match
exactly between the two runs.

Both complete 60 s with zero JOINT_BREAK events and fail the target-detachment
requirement. Peak fruit-center displacement is 1.74 mm for PGS and 2.08 mm for
TGS; the support remains fixed. Changing solver alone does not make this
20 cm / 5 s native gesture effective. This does not invalidate the prior
successful main GUI trials, whose actual gestures differed from this synthetic
one. The next useful control is a recorded successful reference gesture,
rather than assuming that this synthetic gesture should always detach fruit.

Evidence: `lateral-native-pair/fixed-tgs/`; compact comparison:
`lateral_native_solver_results.json`. No GUI was opened for this failed candidate.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/prepare_lateral_diagnosis.py \
  artifacts/detachable_fruit_v2/standard-integration/lateral-native-pair/fixed \
  /tmp/lateral-native-fixed-tgs --variant tgs --duration 60
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_batch.py /tmp/lateral-native-fixed-tgs
```
