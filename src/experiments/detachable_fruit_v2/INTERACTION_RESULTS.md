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
38 tests passed. One-minute bounded runs for minimum and median mass ratios
passed; maximum-ratio, rapid movement and early-release cases are in progress.
The GUI FPS and manual detachment review for this controller remain pending.

Earlier 33.48 GUI FPS belongs to the native-input candidate, not this new
controller. No claim of final acceptance is made until the new GUI review.
