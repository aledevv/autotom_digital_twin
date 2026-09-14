# One-time detachment arming after initial support rest

User requested6N and no detachment until the support stops moving. Optional flag
`--arm-after-settle` implements an initial settling gate, not repeated locking:

- Fruit FixedJoint breakForce is infinity in the prepared USD and runtime profile.
- All rachis/pedicel bodies must stay below5mm/s and0.05rad/s for1 continuous
  simulated second, starting no earlier than2s. Rates are derived from measured
  poses, avoiding known differences in reported PhysX body velocities.
- A native mouse grab observed during settling resets the quiet interval.
- After quiet is confirmed, author the configured6N on the existing joints once.
  Subsequent movement or dragging never disables detachment again.
- No joint deletion, articulation reconstruction, pose editing, custom force,
  gravity ramp or new physics steps. The native mouse remains native. The arming
  condition is custom experimental logic, not a biological fracture model.
- Stop/reset invalidates the diagnostic sequence; use a fresh launch for another
  initial-settling trial. This gate does not hide physical divergence from checks.

Explicit command, preserving intermediate stiffness2, original damping and
support density2000 kg/m³ with corrected-size fruit:

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py \
  --coherent-fruit --support-density 2000 --rachis-stiffness-scale 2 \
  --fruit-break-force 6 --arm-after-settle
```

## Verification

33 targeted tests pass (settling gate, support controls and monitor helpers).
Tests cover continuous rest, rejection of dragging/nonfinite rates, one-time arming
without relocking, and unbreakable startup affecting only fruit breakForce.

`rachis-k2-6n-gated-screen`:20s passed without spontaneous breaks. Armed at
8.716667 simulated seconds; initially loaded breakForce infinity, final6N.
`headless-arming.json` records arming time, thresholds and exact joint paths.
Partial and final reports include gate state; a headless run that never arms fails.

`rachis-k2-6n-gated-force`: controlled COM ramp0–12N from30s produced real
JOINT_BREAK at30.083335s but failed the existing continuity check: position step
12.876mm versus11.979mm bound. External commanded force at that instant was
about0.20N; it is not a measurement of the internal joint reaction.
`rachis-k2-6n-ungated-force`: the same load without the gate produced the same
break time, pose step and continuity failure. Therefore the tested discrepancy is
not isolated to startup arming; neither force test is claimed as a passing full
interaction validation. Native manual quality still requires user feedback.

Manual GUI `gui-freeze-8ng5unec` launched with the command above. No production or
main/groPy default is changed. Heavy evidence remains in local artifacts.
