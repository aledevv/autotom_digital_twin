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
