# Full native plant performance challenge — 2026-09-18

Objective: increase fluidity from 4–5 FPS while retaining compliant native branches and seven-link compliant blades under contact and Shift+drag. Preserve native geometry, mass, leaf stiffness, contact filters and prior artifacts. Two-hour investigation. Runs are sequential on the same machine; GUI acceptance is separate.

## Reference

`opt-baseline-soft`: all131 blades, 1098 bodies, CPU/PGS480Hz,32 position iterations,30Hz rendering, plant contacts with82 explicit initial-overlap exclusions.20 simulated seconds. 4.345 uncapped workFPS; median frame220.43ms,p95 268.37ms. Median physics95.05ms,reads/reports43.11ms,skinning/diagnostics29.69ms,render/sync45.75ms.

The existing illustrative profile fails previous scientific acceptance:78.56% maximum edge extension,11.48mm world recovery,1.788mm residual and2.93mm settled penetration. These are baseline limitations, not relaxed thresholds or evidence of calibrated leaf mechanics.

## Verified results / experiments

- `opt-array480`: direct Vt quaternion arrays, cached attributes, offline geometric diagnostics at the same10Hz pose sampling. Physics settings retained.4.836FPS. Pose arrays exactly equal to baseline across all recorded samples. Root/edge measurement difference only floating-point reconstruction roundoff.
- `opt-batch120-t0`:16-leaf visual clusters, cached contact actor names, synchronous CPU worker configuration,120Hz. **Rejected:** invalid transforms then native PhysX segmentation fault before1 simulated second. No performance claim. Separate480Hz and solver tests follow.

## Implementation under investigation

- `--optimized`: offline geometry diagnostics, direct animation arrays. Always retains finite-state checks and pose traces for bounded recorded duration.
- `--skin-batch N`: heterogeneous native geometry and arbitrary3D poses combined into visual clusters, with unchanged individual colliders/joints. Runtime USD skin validation compares to original CPU skin within1micrometre.
- `--fabric`: use PhysX Fabric renderer transforms instead of per-frame USD synchronization.
- Experimental physics controls: `--physics-hz`, `--solver`, `--iterations`, `--physics-threads`, `--sleep-threshold`. Defaults remain the previous480Hz/PGS/32/8/no sleep.

References: [Isaac6.1 performance handbook](https://docs.isaacsim.omniverse.nvidia.com/6.1.0/reference_material/sim_performance_optimization_handbook.html), [PhysX articulations](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Articulations.html). An articulation sleeps as a unit; individual dormant blades in the same tree do not independently stop solving. Native `SimulationManager.enable_fabric()` and PhysX Fabric `.update()` verified in the installed6.1 source.

### Further measured evidence

- `opt-batch480-t0`:6.416FPS. Native physics pose arrays exactly equal to reference. Median physics77.64ms,contact/read29.70ms,skin1.77ms,render41.89ms. USD combined skin vs CPU surface maximum88nanometres. This combines batched skin, cached actor decoding and zero worker threads; do not attribute its entire gain to one factor.
- `opt-fabric480`:20seconds completed (90.6wallseconds) but **post-run validator failed** because it read legacy Fabric position/orientation attributes; installed6.1 uses Fabric hierarchy matrices. No final report/pose output from this run; retained for diagnosis. Validator corrected using the installed official test example, and subsequent runs save raw evidence before validation.
- `opt-partition480`:10 articulated groups rooted at the ten already-world-fixed trunk links. No rigid bodies added, no compliant joints removed. Fabric + light diagnostics:8.468FPS,physics103.65ms,skin1.89ms,render9.95ms. Render/native position error0;quaternion error1.4e-7. **Not equivalent yet:** leaf108 differs by65.36mm,maximum strain122.1% vs reference78.56%; other selected leaves differ by micrometres. Contact coupling/solver ordering changes require further validation. This is experimental, not the promoted interactive configuration.
- `opt-partition-tgs120`:metadata wildcard matched no articulation (root paths became deeper); aborted before simulation. Corrected to explicit joint-root paths.
- `opt-partition-tgs120-v2`:actual10articulations,max349links/696DOFs. TGS120Hz also unstable at step4; safety check stopped before a native crash. **Rejected.**

- `opt-pgs480-i8`:12.937FPS,median physics63.21ms andrender9.78ms; finite and attached, but maximum edge extension178.6% andworld recovery18.97mm. Not promoted as fidelity-preserving.
- `opt-branches-i8-sleep`:factor every elastic joint leaving the already-world-fixed trunk into a separate fixed-base tree.20virtual fixed frame copies, mass split only among the fixed copies andoriginal fixed parent; total mass retained.30articulations,max153links/304DOFs.16/30groups sleeping at20s (ten roots are the now-isolated fixed trunk).13.103FPS,maximum strain182.6%,world recovery60.37mm. Not promoted. Elastic joints retain their drives/frames; articulation/contact solver grouping changes behavior.

The conservative path retains the original full articulation,480Hz,32iterations,masses,shapes,drives andcollision filtering. Adaptive display scheduling will explicitly report physical simulated-time/wall-time ratio alongside FPS; it never changes physics dt, drops simulation steps, or claims real-time physics from smooth display alone.

- `opt-live480-reference`:8.606FPS,0.287x physical real-time ratio. Original full articulation,480Hz,32iterations,zero sleep. **All recorded poses bitwise identical to baseline**, despite removal of new-leaf contact report generation anduse ofFabric. Maxstrain unchanged to reconstruction roundoff. Collision solving remains enabled; no per-contact impulse claims from this live mode.
- `opt-gpu-preflight`:GPU/TGS480Hz initially exceeded found/lost aggregate pair capacity andproduced nonfinite state. Rejected. Increasing the explicit GPU buffers will be a separate run; no GPU speedup claimed.
