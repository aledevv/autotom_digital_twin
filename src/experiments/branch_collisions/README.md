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

The fixture has 181 rigid bodies, 262 collider shapes (162 capsules, 60 cylinders,
40 spheres), and 181 joints. A/B allow 740 shape pairs, all fruit-related;
C allows 12,796. These are eligibility counts, not observed contact counts or
measured solver complexity. Initial cylinder proximity uses a conservative
capsule envelope. Invisible physical support colliders can be thicker than
their graphics. This campaign measures contact cost at fixed plant size.
