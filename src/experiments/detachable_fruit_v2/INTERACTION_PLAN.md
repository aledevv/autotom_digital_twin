# Detachment through Shift+click: proposed next experiments

Status: planning, 2026-09-10. The preceding implementation and results are
preserved on the remote experimental branch at `6e72d1f`. This plan does not
claim interactive acceptance and does not change the physics configuration.

## Objective and fixed reference

Full day-160 plant, all 72 fruits, native 6 N breakage enabled from startup,
unchanged masses and geometry. Start from CPU / PGS / 60 Hz, articulation
position/velocity iterations 32/0, fruit iterations 255/0. Keep the existing
collision filters and truss stiffness/damping. Angular residual is diagnostic.

The reference completed one minute at rest and three independent controlled
COM-force detachments. Its GUI measured 33.48 steady FPS at 1280x720, but the
user could not detach fruit using native mouse gain 10. Gain 1000 caused a
cascade and nonfinite state; it is rejected and outside the allowed runner
range. These findings do not yet establish the native mouse force or picked
actor. The first nonfinite body in sorted monitor order is not causal proof.

## 1. Reproduce native mouse interaction without manual repetition

Use the installed PhysX `update_interaction(origin, direction, event)` API,
which the installed GUI calls for drag begin/change/end. Preserve its event
ordering relative to physics steps. Verify the headless API path works before
assuming parity with the GUI. The public API is also described in NVIDIA's
[Python reference](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/extensions/runtime/source/omni.physx/docs/api/python.html).

Record ray origin/direction, physics step, scene-query collider and rigid-body
path, hit point, and chosen fruit/support chain. A scene query is evidence of
the ray hit, not automatically proof of the internal mouse target: check it
against native interaction responses, and report ambiguity explicitly.
Check invisible support colliders, occlusion and release handling. Store a
replayable event sequence; avoid changing the NVIDIA installation.

First replay one accessible fruit on the affected `Truss_r5_o0_g421786`,
preferably fruit 08 if its ray can be unambiguously targeted. Use fresh initial
poses, 30 seconds settling, a slow five-second drag, release, and observation
to 60 simulated seconds. Start with native force mode/gain 10; compare D6 mode
only if it resolves a remaining uncertainty. Keep all full-plant runs sequential.

## 2. Separate force magnitude, attachment-point torque and native grabbing

On that same fruit, compare:

1. The existing downward COM ramp, 0–12 N over five seconds, stopping at break.
2. The same force/time profile applied at a defined fruit surface point.
3. The native drag replay, with the ray and gesture recorded.

If the surface case differs, repeat with a lateral force to isolate direction
and torque before changing plant parameters. Do not label a native gain as N.

If native force cannot be read directly, calibrate it on an isolated free fruit
with the same mass and geometry, gravity and contacts disabled, without joints.
Check a known direct force first, then estimate the native resultant from
`m * delta(COM velocity) / dt`; compare actual pose motion too. This estimate
is valid for that isolated probe, not a measurement of mouse force or joint
reaction inside the constrained plant. Probe gains 1/3/10 only as needed.

At each step retain native break events, pre/post-break continuity, actual
support motion, finite-state checks and the first abnormal sample. No more
unbounded native gain tests. Do not infer stability from the angular offset
or raw solver velocity alone.

## 3. Select the smallest intervention supported by the evidence

- Wrong target or incorrect input timing: fix picking/event handling and rerun
  the same replay; keep native interaction if it meets the requirement.
- Direct force detaches stably, native grabbing does not: proposed fallback is
  a fruit-specific Shift+click controller with a force expressed in newtons,
  capped initially at 12 N and ramp-limited initially to 2.4 N/s. Use the tested
  application point (COM initially if surface loading remains problematic).
  Release the command immediately on mouse-up, lost capture or JOINT_BREAK.
  The joint must break through PhysX at 6 N; do not delete it or toggle its
  enabled state to simulate detachment. Prevent double application by native
  grabbing and the custom controller. Preserve interaction with other organs.
- If a controlled surface/COM load destabilizes the support: diagnose that
  smallest reproducible case first. Only then vary a supported solver or
  damping control individually, validating the full plant and GUI FPS again.

The bounded controller is an explicit proposed interaction change, not an
already implemented replacement or a claim that the native issue is solved.
Discuss the diagnostic comparison before expanding into other architectures.

## 4. Verification against the complete requirement

After a candidate works, run independent headless cases for minimum, median
and maximum fruit/pedicel mass ratios, one minute each. Include rest, a slow
pull, release and post-detachment observation. Add one rapid mouse movement
and early-release replay to check force limiting and cancellation. Require
real target breakage, no unintended cascades, no nonfinite state or persistent
support divergence. Preserve all diagnostics even when advisory.

Then run one minute in GUI with the user's Shift+click on multiple trusses.
Keep the plant, physics rate, renderer and viewport comparable to the 1280x720
reference. Measure whole-frame FPS including physics and monitoring, report
post-warmup average and fifth-percentile frame FPS, and report real-time factor
separately. Final acceptance requires at least 20 steady FPS, fruit attached
at rest, successful manual detachment, acceptable support recovery and the
user's positive review. Headless throughput does not satisfy the FPS gate.

The first deliverable is the controlled comparison in steps 1–2 and a decision
on picking, force application or support response. The full objective remains
open until step 4 passes; completing this plan is not completion of the goal.
