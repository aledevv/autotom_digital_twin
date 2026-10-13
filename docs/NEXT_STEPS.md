# Next steps and known limitations

Status on 2026-10-13. This file lists the open work for whoever picks up the
project next, human or coding agent. Experiment reports cited here live on the
`thesis-archive` branch; links point there because `main` may not carry them.

## Open tasks

1. **Documentation.** Write two English docs under `docs/`: a short one for
   developers (what the pipeline does, how to run it) and one for coding agents
   (layout, conventions, how to verify changes). Fold this file into the agent
   doc once it exists.
2. **Simplify script usage.** `run_mainV2.sh` exposes about 40 options, many
   left over from experiments. Keep only the flags a normal run needs
   (day/input, output, headless/GUI, duration) and move the rest out of the
   default usage text.
3. **Branch generator.** Add a function that builds a branch from a given
   length and number of joints. Today there is none, because GroIMP supplies
   every branch through the PlantState JSON in `data/plant_states/`.

## Future work

### Visual details
Some visuals can improve, for example the plant tips (terminal organs).

### PhysX joint-break threshold bug (TGS on GPU)
With the TGS solver on GPU, a joint attached to an articulation breaks at about
`breakForce / N`, where N is the number of position iterations, instead of at
`breakForce`. A 6 N fruit joint with N = 32 breaks at about 0.19 N. PGS, CPU, or
a support outside the articulation all break at about 6 N. Isolated with a
minimal fixture on Isaac Sim 4.5 / PhysX 5.5.1:
[break_threshold_probe/REPORT.md](https://github.com/aledevv/autotom_digital_twin/blob/thesis-archive/src/experiments/break_threshold_probe/REPORT.md),
[BREAK_SCALING_DIAGNOSIS.md](https://github.com/aledevv/autotom_digital_twin/blob/thesis-archive/src/experiments/detachable_fruit_v2/BREAK_SCALING_DIAGNOSIS.md).

The suspected source line is in the public PhysX repository
(`artiConstraintPrep2.cu`: the break impulse uses `params.dt = stepDt = dt / N`
on the TGS articulation path). This is a hypothesis consistent with every
measurement, not a verified cause: the GPU source of 5.5.1, the version in use,
is not public, and no newer PhysX was run. To confirm it, either reproduce the
diagnosis with the REA skill in Claude, or build PhysX with that line changed
and rerun the probe. The current workaround is PGS.

### Fruit on side branches
Trusses are only supported on the main stem; side-branch trusses are disabled.
On a side branch the dt bug above plausibly explains the spontaneous early
fruit breaks (TGS/GPU breaks at 0.333 s; switching only to PGS removes them),
but not the whole instability: even with PGS the mobile branch sags 13–24 cm
under the truss load and native mouse drag does not detach fruit.
Evidence: [LATERAL_DIAGNOSIS.md](https://github.com/aledevv/autotom_digital_twin/blob/thesis-archive/src/experiments/detachable_fruit_v2/LATERAL_DIAGNOSIS.md),
[DECISION_AND_HISTORY.md](https://github.com/aledevv/autotom_digital_twin/blob/thesis-archive/src/experiments/detachable_fruit_v2/DECISION_AND_HISTORY.md)
(sections 6-7). The side-branch runs predate the bug diagnosis and were never
repeated with a compensated threshold. Next check: rerun the TGS side-branch
case with `breakForce × N`. If spontaneous breaks disappear while the branch
still sags, the two causes are separate.

### Several plants in one scene
Loading several plants together in Isaac Sim has not been tested. It should be
possible to turn collisions and physics off for a given plant and back on at
runtime; this needs a test.
