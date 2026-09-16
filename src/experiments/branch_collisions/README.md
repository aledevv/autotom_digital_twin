# Branch collision experiment

Experimental scripts only; reference checkpoint `3c2e6cf`, scene
`realism-leaf-droop`. Keep PGS/CPU, GPU rendering, 60 Hz, physical geometry,
masses, drives and native mouse unchanged. Fruit break force is 3 N.

Cases: A original; B explicit collider filters reproducing A with articulation
self-collision enabled; C admissible vegetation contacts; D also admissible
truss support contacts. Fruit pair eligibility is preserved in every case.
All USD, detailed pair audits, contact logs and traces stay under the ignored
`artifacts/branch_collisions/` directory.

## Current evidence

- A/B/C each passed 20 seconds at rest. A and B had identical recorded states.
- C recorded transient contacts during initial settling (0.15–0.267 seconds)
  between a lateral leaf rachis and a lateral internode, without unwanted breaks.
- A/B/C each passed the matched 60-second native drag. The chosen pair remained
  at least 4.398 mm apart during the gesture in all three: this controlled
  contact comparison is **inconclusive**, not evidence of a visual improvement.
- C initial manual feedback: generally good, but the user observed branches
  passing through one another. The bodies involved have not yet been identified;
  this does not establish whether their pair was enabled or whether contacts failed.
  A fresh C GUI records two brief native grabs followed by a crossing gesture
  to identify the pair, without changing filters at runtime. The user's proposal
  to enable selected contacts after settling remains a hypothesis for a separate
  comparison, after checking separation and the reason for exclusion.
- D and the performance/confirmation campaign
  are not yet complete. Headless timings do not establish GUI FPS or crash risk.

The installed PhysX contact API was checked using isolated overlapping spheres:
allowed pairs reported contact; explicit body and collider filters suppressed it.
An earlier negative fixture using a plain nonphysical container did not filter
as expected. Do not generalize filtering semantics to arbitrary USD scopes.

## Reproduction

From the repository root, use `UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync
python` followed by the script and arguments below. Isaac runs through
`/home/alessandro/isaacsim/python.sh`.

```text
src/experiments/branch_collisions/prepare.py SOURCE NEW_CASE --variant A
src/experiments/branch_collisions/run.py NEW_CASE
src/experiments/branch_collisions/run.py artifacts/branch_collisions/C-screen --gui
src/experiments/branch_collisions/analyze.py CASE... --output SUMMARY.json
```

Preparation of B/C/D uses `--offsets
artifacts/branch_collisions/A-screen/loaded-offsets.json` for actual contact
offsets and reference mass/inertia/COM checks. Every run needs a fresh directory;
GUI launches create one automatically and wait for explicit closure.

Native mouse coefficient 50 is a runtime setting, **not a force in newtons**.
No custom force controller is introduced. If replay does not generate contact,
collect a manual gesture instead of increasing force automatically.

## Measurement limits

## Organic surface pilot (2026-09-16)

The user identified `Branch_s2_o1_g421414_Link_04_Internode_g421675`
and both `Leaf_r5_o0_g421371_rachis` links, and reported that contact works on
the rachis but visual organic surfaces can pass through. This is a manual
observation, not yet a measured proof of the exact crossing geometry.

`prepare_organic_pair.py C-screen NEW_CASE` adds ten convex mesh colliders:
three direct organic link surfaces and seven rigid leaflet-support surfaces.
Leaf blades remain noncolliding. Added shapes interact only across the two
selected supports, never with their own assembly, fruit, or unrelated bodies.
This is a pair-local pilot, not a rollout to all petiolules. The desired eventual
rule is to exclude a petiolule's own supporting assembly but allow other branches.

The existing 262-shape collision matrix is unchanged. All new pair exclusions
were checked against the effective authored matrix. Previously loaded mass,
COM, principal axes and diagonal inertia are explicitly retained on the three
affected bodies; the runtime checks cover all 181 bodies. This matters because
the original inertia was automatically computed from collision shapes.

`C-organic-pair-screen` passed 20 simulated seconds at rest with no validation
errors or spontaneous breaks; process exit 0. Isaac logged an extension-window
error during shutdown, after the completed report. Manual contact effectiveness,
convex approximation quality and GUI performance remain to be checked.

Convex hull is an approximation and fills concavities. The organic stalks remain
rigidly attached to their existing body; this adds no independent bending DOF.

## Fixed-size cost boundaries

### Organic pilot correction and failed expanded screen

Manual run `gui-C-organic-pair-670c2wqm`: user reported no improvement.
The native grab hit `OrganicVisual_03`, proving the added surface was selectable.
New organic-to-organic contacts against the thick branch were recorded, but
the trace also showed interactions with `LatLeaf_r3_o0_g421593`, which the
pair-local pilot had excluded. Thus the pilot did not cover leaf-to-leaf
organic contacts and must not be described as solving the user's observation.

`prepare_organic_pair.py ... --include-lateral-leaf` includes that lateral
leaf's petiole, rachis and organic petiolules: 17 new shapes across five bodies.
New shapes only collide across main-leaf/lateral-assembly groups. The effective
authored pair matrix is asserted and all old pairs remain unchanged.

`C-organic-leaf-pair-screen` **failed at reset**, before the 20-second screen:
main leaf rachis link 02 moved 0.00627942 m, triggering the existing reset
projection gate. Loaded masses, COMs and inertias matched the reference.
Six initial mesh AABB overlap candidates were recorded locally; these are
broadphase candidates, not proof of convex intersections or their causal role.
Do not launch this variant as a passing candidate or weaken the reset gate.
Next diagnosis is exact cooked geometry/initial overlap and settled separation,
before any separately validated delayed activation trial. No runtime filter
activation, solver changes or global petiolule rollout have been implemented.

The user subsequently clarified that a changed resting pose is acceptable:
initial separation due to newly enabled contacts is not itself instability.
The diagnostic monitor now accepts an explicit
`diagnostic_reset_projection_limit_m` (default unchanged at 1 um, bounded at
10 mm), reports both the limit and measured displacement, and keeps the fixed
root reset limit at 1 um. This changes validation only, not the authored pose,
physical model, collision activation, or solver. It does not recapture or
substitute the authored reference geometry.

`C-organic-leaf-pair-settle` uses 10 mm and passed 20 seconds with no reported
errors/breaks; measured reset displacement remains 6.2794 mm. Tail position
excursion was 0.00671 mm. Reported tensor velocities and pose-derived motion
remain distinct metrics; this functional pass is not a strict velocity-gate
pass or manual contact acceptance. The 60-second extension is recorded in a
fresh `C-organic-leaf-pair-settle60` directory. Existing monitor tests: 18 passed.
The extension also passed 60 simulated seconds with no errors or breaks.
Compact reports, including numerical advisories, are in
`organic_settling_results.json`. Contact effectiveness remains a manual gate.

The fixture has 181 rigid bodies, 262 collider shapes (162 capsules, 60 cylinders,
40 spheres), and 181 joints. A/B allow 740 shape pairs, all fruit-related;
C allows 12,796. These are eligibility counts, not observed contact counts or
measured solver complexity. Initial cylinder proximity uses a conservative
capsule envelope. Invisible physical support colliders can be thicker than
their graphics. This campaign measures contact cost at fixed plant size.
