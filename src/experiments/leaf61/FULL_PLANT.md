# Native full plant with skinned blades (2026-09-18)

Separate integration fixture; the production exporter and `run_mainV2.sh` are unchanged.

```bash
./run_leaf61_full_plant.sh
./run_leaf61_full_plant.sh --headless --render --seconds 20
./run_leaf61_full_plant.sh --headless --leaves 1 --seconds 20
```

The default source is the SHA-pinned `artifacts/branch_collisions/C-organic-leaf-pair-settle60/scene.usda`: 131 native blade meshes, 181 native rigid bodies. The full fixture adds 393 lamina bodies. Native branch joints, fruits and their existing contact filters are retained. The native stem is not replaced by the synthetic flexible-stem prototype.

Each blade retains its original outline, curvature and initial world pose. Three lamina segments use the candidate stiffness (0.024 Nm/rad), damping ratio 1, CPU/PGS at 480 Hz. They join the native articulation. New leaf-to-leaf and leaf-to-native-support collisions are filtered in this first integration; mouse picking and external contact remain available. This fixture does not establish leaf/stem contact behavior or rain behavior on the full plant.

Native dry biomass already aggregated into the host is removed, including its original mass moment, and distributed equally among the three lamina segments. Total scene mass is conserved (about 1.439248812 kg). The distributed lamina changes the original point-lumped mass distribution; it does not preserve the entire plant inertia tensor or reproduce the heavier isolated candidate. Source geometry and mass are not biologically recalibrated.

Native meshes are curved. Their basal weights blend gradually between the fixed band and the first segment center; the original rectangular-bench weight rule has a discontinuity for vertices offset from the hinge plane. This adaptation changes only the new integration fixture.

GUI controls are in English. Native **Shift + left-drag** pulls branches or leaf colliders. The slider selects the blade for **Apply 3 mN tip load**; **Release** removes that force. **Finish and save** saves the report. The headless protocol settles for 8 s, applies a downward 3 mN force to the first leaf's last segment during 8–10 s, then recovers until 20 s. This is a direct load test, not a contact test.

Physics executes exactly once per loop iteration. Poses are read together; animation and rendering update at 30 Hz, geometric checks and saved poses at 10 Hz. Native USD transforms synchronize at render frames. Runtime costs exclude startup and the first 8 simulated seconds; `uncapped_work_fps` measures completed frame work, not the capped GUI display rate. Full-plant results are single runs, not a repeated performance certification.

Every launch creates a fresh folder under `artifacts/leaf61/full-plant-*`, with source snapshot/hashes, immutable input copy, scene, layout, mesh bind data, logs, runtime configuration, pose trace and reports. GUI pose recording is bounded to `--seconds` (default 20); manual interaction after that remains visible but is not a complete gesture trace. Manual GUI runs are not treated as the automatic load/recovery protocol.

## Development failures retained

- `full-plant-preflight-1`: preliminary single-leaf rest run, before the canonical lateral-axis correction; not acceptance evidence.
- `full-plant-131-headless`: first 131-leaf attempt stopped at 8 s because the tensor force call required explicit indices.
- `full-plant-load-preflight`: stopped by a geometric contract check. The source is a settled scene: its current world-up differs from the original authoring orientation. Canonical local frames must be used.
- `full-plant-load-preflight-v2`: external leaf-root joint gave 0.359 mm root error and 114.7% maximum edge stretch under load. Rejected.
- `full-plant-native-articulation-1`: including the blade in the native articulation reduced root error to 0.000035 mm and recovered, but hard basal skin weights gave 6.43% stretch. Rejected; motivated the native curved-mesh weight adaptation.

No thresholds are relaxed automatically.

## Verified automatic load runs

Same 1280x720 offscreen renderer, 480 Hz physics and 30 Hz render cadence; one run per count, performance window after 8 s. RTX 4080 Laptop GPU, driver 575.57.08, Isaac 6.1.0-rc.26 / PhysX extension 110.3.2. These are **not** GUI FPS or a successful 20 FPS promotion.

| Flexible blades (native plant stays complete) | Bodies | Work FPS | Frame work p95 | Physics median per 16 steps | Skin + checks median | Render + native sync median |
|---|---:|---:|---:|---:|---:|---:|
| 20 | 241 | 15.80 | 78.96 ms | 35.75 ms | 7.47 ms | 16.29 ms |
| 131 | 574 | 6.86 | 190.35 ms | 66.03 ms | 33.79 ms | 30.70 ms |

For all 131 blades (`full-plant-131-render-v2`): finite states, maximum root constraint error **0.000089 mm**, maximum edge extension **0.525%**, worst recovery error **0.0248 mm**, residual motion **0.00311 mm**. The loaded blade moved **1.364 mm relative to its petiole** under 3 mN. Maximum initial mesh reconstruction error was 1.48e-16 m (CPU bind math). Total mass differed by 1.27e-10 kg from the native source due to USD float authoring.

The 20-blade run (`full-plant-20-render`) passed the same checks: maximum extension **0.287%**, recovery **0.00609 mm**. Unconverted blades remain their original rigid visuals, attached to moving native branches.

The full case is limited by physics as well as skinning/measurement and rendering: physics alone took a median **66.0 ms per displayed frame**. Batching visuals may improve throughput but does not by itself establish a 20 FPS budget. Frequencies and acceptance thresholds have not been reduced.

The log contains a GPU-compatible convex-cooking warning on one thin collider; this fixture uses CPU collision and no particles/deformables. No PhysX errors were logged in the two completed automatic runs. Convex flat segment colliders remain an approximation to the curved visible lamina; detailed contact conformity is not validated by a direct-force test.

The first GUI run (`artifacts/leaf61/full-plant-131-gui`) completed 34.5 simulated seconds, with 5.86 measured work FPS and 2.54% peak extension. It also exposed an inspector startup error (`no running event loop`); the shared mouse helper now enables PhysX UI extensions inside the Kit asyncio loop. The corrected GUI was launched separately in `artifacts/leaf61/full-plant-131-gui-v2`. Shift+drag uses the existing native mouse helper. Human visual/gesture acceptance remains pending. 32 leaf61 tests pass, including invariance of reconstructed relative leaf motion under host rotation and translation; targeted lint and `git diff --check` pass.

## Contact stress modes

```bash
# Blades against native plant bodies (branches, petioles, stem and fruits):
./run_leaf61_full_plant.sh --leaf-contacts plant
# Maximum interaction mode: also blade against blade:
./run_leaf61_full_plant.sh --leaf-contacts all
```

The default remains `--leaf-contacts off`. Native-native filters are unchanged in all modes. Each lamina excludes its own host, native bodies directly joined to that host, and its own three segments. Other contacts are enabled by the requested mode. `contact_policy.json` records every exclusion.

`contacts.json` records actual PhysX contact pairs, impulses and collider penetration. First-step penetration is reported separately from penetration after the 8 s settling window; it must not be interpreted as an externally triggered impact. First-step overlap above 0.1 mm flags invalid initial contact geometry; settled penetration above 1 mm fails the contact check. These thresholds do not repair initial intersections automatically. The contact callback cost is charged to `reads_and_contact_reports`, separately from physical stepping, and its accumulated wall time is also recorded.

A direct 3 mN load test remains the headless protocol. Contact-enabled trials add evidence of collisions but do not prove that visible mesh and convex flat collider surfaces coincide. Manual Shift+drag is still needed to assess interactions at the intended locations.

### Maximum-contact stress trial — initial results

The unfiltered `all` trial (`full-plant-contact-all-preflight`, 3 s) is **rejected**: 49 pairs penetrated by more than 0.1 mm in the first step, worst depth 11.46 mm; native-body displacement reached 45.69 m and skin extension 24.15%. This is invalid initial contact geometry, not a successful leaf/branch interaction test.

The explicit `--exclude-initial-from artifacts/leaf61/full-plant-contact-all-preflight` option omits only those 49 measured pairs, preserving other collisions. The source SHA and leaf count must match; copied evidence and the exact pair list are saved with the run. These are permanent pair exclusions, so those specific pairs will also pass through each other later. This is an intentionally restricted stress demo, not unrestricted self-collision.

The 3 s filtered preflight (`full-plant-contact-filtered-preflight`) stayed finite with native-body displacement 53.2 mm and root error below 0.000080 mm. It still recorded 22 leaf/plant and 11 leaf/leaf impulsive pairs. **It failed the 5% stretch criterion (10.01%)**, and residual first-step penetration was 0.565 mm. The older preflight snapshot's zero post-8s penetration is not a valid settled check: it ran for only 3 s. The current report omits that check until an actual post-8s window exists.

The demo launcher is:

```bash
./run_leaf61_full_plant_contacts.sh
```

It requests all 131 flexible blades, `all` contacts, and the explicit 49-pair exclusion evidence. The original launcher remains contact-disabled by default. `--leaf-contacts plant` enables blade/native contacts without blade/blade contacts, but this mode has not been separately benchmarked. Existing native collider coverage is retained: visual-only petiolules do not acquire new colliders. Fine contact between visible curved meshes and flat convex segment proxies remains unvalidated.

### Completed contact stress run

`artifacts/leaf61/full-plant-contact-filtered-render`: 20 simulated seconds, 118.62 s loop wall time, 480 Hz physics / 30 Hz rendering cadence, **5.10 instrumented offscreen work FPS**, frame-work p95 **248.51 ms**. Median frame components: physical stepping **74.87 ms**, reads/contact reporting **32.16 ms**, skinning/checks **35.88 ms**, rendering/native sync **34.76 ms**. Contact callback total: 19.25 s, accounted separately from physical stepping.

The 49 exclusions prevented the original runaway. All states remained finite, maximum native-body displacement **53.2 mm**, root-attachment error **0.000083 mm**. Recovery **1.546 mm** and residual movement **0.0992 mm** passed their thresholds. **Overall status remains FAILED**: peak mesh extension **10.01%** exceeds 5%; settled collider penetration **1.439 mm** exceeds 1 mm. Initial remaining penetration **0.565 mm** also makes the initial contact geometry invalid. This is runnable as an exploratory stress demo, not a validated physical configuration or a real-time result.

PhysX recorded **16 blade/branch pairs**, **6 blade/fruit pairs**, and **11 blade/blade pairs** with nonzero impulses. Relative to the earlier collision-disabled run at 20 s, **5 blades** differ by more than **1 mm** in their host frame, with maximum shape difference **5.459 mm**. See `contact_comparison.json`. This establishes that enabled contacts affect lamina shape; it does not validate the collider/visual fit or eliminate startup-contact artifacts.

GUI: `artifacts/leaf61/full-plant-contact-filtered-gui`, native Shift+left-drag enabled, human visual acceptance pending. Reopen with `./run_leaf61_full_plant_contacts.sh`. All original contact-disabled results remain unchanged.
