# Shift+click interaction comparison — 2026-09-10

The implementation follows [INTERACTION_PLAN.md](INTERACTION_PLAN.md), with
full day-160 geometry/masses, CPU PGS at 60 Hz, articulation 32/0, fruits 255/0,
and native 6 N breakage. The renderer still uses the GPU. Raw evidence is local
under `artifacts/detachable_fruit_v2/2026-09-10/`.

## Comparison on r5 / g421786 / fruit 08

Fresh poses for each 60-second run; stimulation begins after 30 seconds.
The ray search checks the native scene query's rigid-body path and records
obstructions and the surface hit. It does not claim access to the native
interactor's internal selected-actor state or its force.

| Trial | Detachment | Command at break | Other breaks | Continuity |
|---|---:|---:|---:|---|
| COM force, 0–12 N / 5 s | 32.467 s | 5.920 N | 0 | passed |
| Same force at surface point | 32.467 s | 5.920 N | 0 | passed |
| Native force drag, gain 10, 20 cm / 5 s | none | unknown | 0 | n/a |
| Native force drag, gain 10, 60 cm / 5 s | none | unknown | 0 | n/a |
| Bounded COM spring, 60 cm / 5 s | 34.367 s | 5.933 N | 0 | passed |

The controlled COM case moves the fruit approximately 30 cm before breaking;
the initial 20 cm native replay alone was therefore insufficient to attribute
failure to the native mechanism. The additional 60 cm native replay also did
not detach it. Surface loading itself did not cause a cascade in this comparison.
The isolated free-fruit probe verifies a known direct force, then estimates
native resultants from COM velocity changes at gains 1/3/10. These estimates
are specific to the free body/trajectory and are not the mouse force in the
constrained plant. Initial probe attempts saved complete measurements but
crashed during interpreter cleanup; their logs are retained, and the harness
now uses the same post-Kit exit strategy as the main Isaac loader.
The final rerun (`native-force-probe-final`) completed with process exit 0.
For its free-body trajectory, peak estimated resultants were 0.00130,
0.00210 and 0.00364 N at gains 1, 3 and 10. The known-force reference
recovered 0.01002785 N against a command of 0.01002784 N.

## Bounded controller and GUI integration

`LimitedDrag` uses a spring of 60 N/m between the current COM and the translated
COM target derived from the initial surface hit. The target follows ray/plane
intersection on the plane parallel to the initial camera view. The vector
force is capped at 12 N and its vector change at 2.4 N/s, including reversals.
Input release, invalid rays, Escape, or native joint break cancels force.

`--mouse-grab-mode bounded` enables a process-local adapter to the installed
Isaac 4.5 Python viewport overlay. The native interface factory is restored on
close; NVIDIA installation files are not edited. Attached fruit drags use only
the bounded controller. Other picks retain the native path. Holding the mouse
after break cannot accidentally start native grabbing of the detached fruit.
GUI logs retain each selected body, ray, application command and release.
Unexpected fruit breaks outside the selected gesture fail the GUI review.

The headless replay and GUI share the force controller. A headless pass is not
visual approval: residual angular errors and actual support motion remain in
the numerical advisories. The first bounded case has 6.21 mm tail position
excursion and 0.184 rad/s maximum pose-derived angular speed; these exceed
the original screening tolerances and must be judged during the GUI review.

## Validation status

Core controller, ray selection, routing and existing monitor/loader checks:
38 tests passed. Five independent one-minute bounded runs completed:

| Mass-ratio target / gesture | Break time | Peak command | Other breaks | Result |
|---|---:|---:|---:|---|
| Minimum / slow | 32.667 s | 6.400 N | 0 | passed |
| Median / slow | 32.500 s | 5.963 N | 0 | passed |
| Maximum / slow | 32.467 s | 5.920 N | 0 | passed |
| Maximum / rapid | 32.467 s | 5.920 N | 0 | passed |
| Maximum / release after 0.5 s | none | 1.200 N | 0 | passed |

All four detachments passed the movement-continuity check. No run reported
nonfinite states, spontaneous breaks or persistent gross support divergence.
Recorded command vectors obeyed the 2.4 N/s slew limit and the 12 N cap;
no command followed a break. Early release stopped commands at 30.500 s,
with no break through 60 s. The command at break is an externally applied
force, not a measurement of the joint reaction; the joint threshold stays 6 N.

The corrected 60-second manual review completed at 33.16 FPS (1280×720),
but failed detachment acceptance: three grabs peaked at 2.324 N and no fruit
detached. The user reported no instability, but an abrupt grab and missing
native marker/arrow. See [MAIN_COMPARISON.md](MAIN_COMPARISON.md) for the
requested comparison against the working main-branch truss.

The first manual GUI attempt was rejected: no fruit could be grabbed. It
ended when the timeline stopped at approximately 7.83 simulated seconds,
before the required minute. Its approximately 33 FPS is only a partial run.
An automatic UI-input probe subsequently reproduced the cause: native
`omni.ui.scene.Vector3` rays are iterable but direct `numpy.asarray` conversion
raises `ValueError: setting an array element with a sequence`. The original
handler silently cancelled this input. Materializing the components as tuples
before NumPy conversion fixes this boundary; invalid-ray rejection is now logged.
The routing regression test exercises an iterable that rejects direct NumPy
conversion. Temporary overlay and timeline hypotheses were discarded; the
original native viewport event path is retained. Automated UI probes can run
without a desktop window, avoiding interference with manual input.
The corrected native-overlay route completed a 15-second automatic UI probe
(`bounded-gui-vector-fix`, process exit 0): actual ray hit r5 fruit 07, one native
break at 7.267 s with 6.054 N commanded, continuous movement, no other breaks,
and force cancelled at the break. The camera aimed toward fruit 08 but fruit 07
occluded it; this test establishes event routing, not a repeat of the controlled
fruit-08 comparison. Its invisible-window FPS is not a desktop performance result.

Earlier 33.48 GUI FPS belongs to the native-input candidate. The corrected
controller has its own 33.16 FPS measurement but no final acceptance.

## Bounded-controller visual feedback (2026-09-10)

The user's last manual review could grab fruit, but found the gesture abrupt,
with no contact marker/arrow and no detachment. Its maximum commanded force
was 2.324 N. That review remains rejected despite 33.16 steady desktop FPS.
The main-branch controls are documented in MAIN_COMPARISON.md.

The bounded controller now displays the moving surface grip, cursor tether,
COM force arrow and its actual commanded magnitude in newtons. The 60 N/m,
12 N, 2.4 N/s force law and physical 6 N break threshold are unchanged.
The native viewport toast was absent in an offscreen screenshot, so the force
label is also rendered directly beside the arrow. No native grab is added.

`probe_gui_input.py` buffers real Carb Shift/down/move/up events through the
PhysX viewport overlay in an offscreen app. The local `bounded-gui-feedback-
probe-v3` case completed 15 s (exit 0): one genuine break of r5 g421786 fruit07
at 7.266667 s, commanded force 6.054328 N, continuity passed, no other breaks
or functional-monitor errors. It hit fruit07 in front of the aimed-at fruit08;
this is an input-routing smoke check, not an exact-fruit paired experiment.
The same break time and force occurred without the new display.
Screenshots at steps 300/420 show the point, arrow and force label; the
offscreen scene itself is dark, so these images validate overlay presence,
not desktop scene appearance. The offscreen render throughput is not desktop
FPS evidence. The first graphics probe stopped on a screenshot-helper argv
error; logs are retained, and v2/v3 completed after fixing the helper.

Nine controller regression tests pass; the preceding focused exporter checks
passed 38 tests. Final 60-second desktop interaction and acceptance remain
pending for this display revision.

## Manual feedback review and bounded slew comparison

`bounded-gui-feedback-review` recorded two user-triggered native breaks on
r5 g421786 fruit07 (5.900 s) and r10 fruit06 (16.867 s); both continuity
checks passed. Steady desktop FPS was 31.366, fifth-percentile 26.676,
RTF 0.5231. The timeline stopped at 49.0833 s, so the required 60 s was not
completed. The launcher exited 1 and subsequently logged Killed during Kit
shutdown; the cause is not established. This is not a clean-exit pass.

The user confirmed detachment but rejected the gesture because it needs too
much mouse travel/time. The two grabs lasted 3.0 and 2.6 simulated seconds;
95.6% and 98.1% of their force steps hit the 2.4 N/s vector slew limit. Peak
forces were 6.022 and 5.992 N. Continuing to move the mouse could not make
that ramp faster. This motivates a single-control comparison at 4.8 N/s.
Spring gain remains 60 N/m, force cap 12 N and native break threshold 6 N;
no pointer amplification or physical-scene change is included. The runner
records `--drag-slew-rate` for both replay and manual GUI. Default stays 2.4
so old cases remain reproducible; 4.8 is an explicit experimental candidate.

All five independent 4.8 N/s headless cases completed 60 s and exited 0:

| Case | Break delay after 30 s settling | Peak command | Other breaks |
|---|---:|---:|---:|
| Minimum mass ratio | 1.350 s | 6.478 N | 0 |
| Median | 1.267 s | 6.004 N | 0 |
| Maximum | 1.233 s | 5.919 N | 0 |
| Maximum, rapid | 1.233 s | 5.920 N | 0 |
| Maximum, release at 0.5 s | None | 2.400 N | 0 |

All four detachments passed motion continuity. No nonfinite values or
persistent gross support divergence were reported. Command traces respect
12 N and 4.8 N/s, with no commands after break/release. Angular and residual
motion diagnostics remain advisory, not evidence of manual acceptance.
The 28 focused interaction/diagnostic regression tests passed.

The offscreen UI-event replay `slew48-gui-probe` also completed 15 s, exit 0:
one r5 fruit07 break at 6.4333 s (6.1135 N), continuity passed, no other
functional-monitor errors. Its reported GUI controller slew is 4.8 N/s.
With the identical slow pixel trajectory, grab-to-break time decreased from
5.0667 to 4.2333 s; the factor-of-two reduction in rapid headless pulls does
not imply that every mouse trajectory becomes twice as fast.

## Latest manual rejection: held load and loss of grip after detachment

`slew48-gui-review` completed 60 s, exited 0, at 1280x720. Steady desktop FPS
was 31.947 (p05 27.801), RTF 0.53165. Four user-target breaks on r10 fruits08,
07,06 and r5 g421786 fruit07 passed the pose-continuity check. The user rejects
this candidate: under a held load near 3 N the truss keeps moving/rotating;
after break the fruit appears to be recreated and drops instead of following
the mouse. The user explicitly requests that grip persist until mouse release.

No recreation or velocity reset is performed by the controller. Its immediate
force cancellation on JOINT_BREAK explains loss of mouse following. The
recorded positions remain continuous, but this does not establish acceptable
perceived behavior. Post-break mouse following needs a separate free-body
controller; applying the attached-fruit multi-newton force directly to a free
6-11 g fruit would produce excessive acceleration.

The held-load defect is present in saved poses: during 17.5-20 s at about
3.4 N, r10 pedicel06 changes orientation by about 30.23 degrees and the truss
supports span about 60.91 mm. Gross-divergence and post-release tail checks
miss this defect. Their report status is not human acceptance.

Added hold diagnostics: ramp for 1.25 s, hold until stimulation+10 s, then
release; expect no break. `--force-direction` selects a normalized world
direction; `--drag-plane-normal` can reproduce a recorded drag plane. The
COM hold uses 3 N; the bounded hold keeps the cursor target fixed. All six
initial cases (vertical/lateral reduced truss and the full plant) complete
60 s with no breaks, but retain small pose drift under load. In the full
plant's fresh attached state, the recorded 82.29 mm cursor offset gives
3.40 N and only about 1.07 mm / 4.65 degrees motion during 37-40 s. This does
not reproduce the user's much larger motion after earlier detachments.

`replay_recorded_grip.py` replays the complete recorded ray sequence through
the same GUI bridge, with offscreen rendering and synthetic button state.
It checks selected body IDs and restores recorded drag-plane normals. It
reproduces the earlier detachments as physical breaks, not deleted joints.
`recorded-grip-reproduction` reproduces the held defect: approximately
60.88 mm / 30.34 degrees during 17.5-20 s. This is the paired reference for
the next single-change truss damping4-to7 comparison. These are offscreen
measurements, not new desktop FPS evidence.

The truss damping ratio7 control completed 60 s (exit 0), but held motion
remained approximately 59.82 mm / 28.34 degrees: it is not a useful fix.
A separate mouse damping control at 1 N s/m, with original truss ratio4,
reduced the same held interval to approximately 0.80 mm / 2.60 degrees.
`recorded-grip-mouse-damping1` completed 60 s, exit 0. Damping uses measured
COM displacement between physics steps; the preexisting 12 N vector cap and
4.8 N/s vector slew cap still apply. The default mouse damping remains zero
for reproduction. This improvement has not yet been accepted manually.

Following the user's explicit choice, `--retain-fruit-grip` now keeps the
selected fruit under control after JOINT_BREAK until input release. Native
PhysX still produces the break; no body, joint, pose or velocity is recreated
or overwritten. The detached phase uses mass-scaled, velocity-damped tracking
(position gain 5/s, velocity response 0.05 s), desired-speed cap 2 m/s, net
acceleration cap 20 m/s2 and gravity compensation while held. Its force stays
within 12 N and is much smaller for these fruit masses. The attached-phase
vector slew rule does not apply to the detached phase; this replaces the old
immediate-cancel-on-break requirement at the user's request. Release cancels
commands and leaves momentum to physics. Per-step logs now include phase,
COM target and measured COM position. Isaac validation of the combined change
is in progress; the 33 focused controller/monitor tests passed.

The combined candidate completed five independent 60-second headless tests
(`retained-min`, `retained-median`, `retained-max`, `retained-rapid`,
`retained-release`), all exit 0 and no functional-monitor errors. The four
intended breaks were single and continuous, and early release caused none.
The fruit remained controlled after break until the scheduled release. Local
`grip-check.json` files verify attached force/slew limits, detached command
acceleration and absence of commands after release.

`retained-ui-probe` exercised real Carb mouse/Shift events offscreen for 15 s,
exit 0: one break at 5.6333 s, grip retained until release at 10.0167 s, then
no further force commands. At the end of the stationary hold the COM target
error was 0.25 micrometres; peak free-phase force was 0.136 N. Screenshot
`feedback-500.png` shows the retained-grip label. This is not manual acceptance
or desktop FPS validation. The candidate still awaits a new 60-second user
review including sustained load and post-break dragging.
