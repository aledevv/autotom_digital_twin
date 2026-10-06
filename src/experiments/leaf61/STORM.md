# Leaf rain experiment log — 2026-09-17

## Scope

Isaac Sim 6.1; 127 skinned leaves, three dynamic rigid segments per lamina. Stem and branches are frozen visual geometry; petioles are attached to the world. CPU/PGS, 480 Hz physics, 30 Hz rendering target. This is not a Surface Deformable experiment.

Rain is a stochastic momentum load applied through batched PhysX forces and torques. All leaves are independently exposed, without canopy shielding, pooling, runoff or splash. Visual streaks are illustrative, not collision-resolved water. Drop diameter 3 mm and speed 7 m/s are assumptions. Gain scales impulse per drop; rate scales expected impact frequency.

## Measured runs

Results below are read from saved report.json files, not GUI FPS estimates. Offscreen runs are individual measurements, not repeated capacity certification. Geometry is sampled at 10 Hz.

| Run | Rate mm/h | Gain | Load Hz | FPS | Max edge extension | Max recovery error mm | Checks |
|---|---:|---:|---:|---:|---:|---:|---|
| [storm200-offscreen](../../../artifacts/leaf61/storm200-offscreen/report.json) | 200 | 1 | 480 | 18.79 | 0.175% | 0.0104 | Pass sampled checks |
| [storm200-gain20-offscreen](../../../artifacts/leaf61/storm200-gain20-offscreen/report.json) | 200 | 20 | 480 | 25.27 | 6.692% | 4.0573 | FAIL structure and recovery |
| [storm200-load120-offscreen](../../../artifacts/leaf61/storm200-load120-offscreen/report.json) | 200 | 1 | 120 | 29.00 | 0.201% | 0.0096 | Pass sampled checks |
| [storm200-gain10-load120-offscreen](../../../artifacts/leaf61/storm200-gain10-load120-offscreen/report.json) | 200 | 10 | 120 | 28.82 | 1.794% | 0.9811 | Pass sampled checks |
| [storm1000-deluge-offscreen](../../../artifacts/leaf61/storm1000-deluge-offscreen/report.json) | 1000 | 1 | 120 | 27.67 | 0.259% | 0.0307 | Pass sampled checks |

Original rain used 1024 visual streaks and 480 Hz load generation. Optimized nominal rain uses 512 streaks and 120 Hz load generation, holding the force over four 480 Hz physics steps. Deluge uses up to 2560 streaks and 1000 mm/h. These configurations differ; the FPS improvement is not attributed to a single change. Gain 20 failed the unchanged 5% stretch and 2 mm recovery limits. Gain 3 has no dedicated recorded validation.

User feedback: original rain visually approved ("bellissimo"); deluge acknowledged positively. Gain 10 passed the recorded numerical smoke test; no explicit separate visual acceptance recorded.

## Launch and controls

Run `./run_leaf61_storm.sh`. GUI labels are English. Controls include Repeat rain ×1, Stress ×3, Stress ×10, Deluge ×5 frequency (1000 mm/h, load ×1), Stop rain, Finish and save. Existing windows use their saved source snapshots and need restarting for new buttons.

## Next experiment: moving branch with skinned leaves

Requested: combined branch and lamina dynamics. Start with one anchored elastic branch and 3–5 existing skinned leaves, then scale to several branches. Preserve accepted leaf mass, geometry and joint parameters. Attach each rigid petiole to a branch rigid body using a fixed joint, replacing its world constraint. Do not merely unfreeze the canopy: existing leaf articulations are rooted at the world-fixed petiole, and their articulation topology and attachment frames require explicit adaptation.

Check gravity settling, branch motion transferring to leaf bases, equal-and-opposite load transfer, attachment-frame error in the moving branch frame, leaf deformation and post-load recovery. Compare fixed-branch and dynamic-branch runs with matched counts, timestep, rendering and load. Record physics, pose reads, skinning and render costs separately. Begin with nominal rain or one tomato, then gain 10/deluge. No combined-dynamics result or FPS is established yet.

The current canopy builder explicitly strips physics in `canopy_fixture.freeze`; the leaf builder roots each articulation at `FixedPetiole`. The moving-branch trial must be a separate fixture and must not change the accepted frozen-canopy tests.
