# Pre-integration original-rank checks

2026-09-15: user authorizes tests before implementing anything in v2.3.
Use main-derived original direct trusses6–10. PGS/CPU60 Hz, articulation32/4,
fruit32/1, native force50,3 N active from startup. GUI uses real-time pacing.
The rapid pull-to-pointer behavior is documented, not fixed with a custom grip.

Source: `multiple-trusses/five-screen`, with source/build/input provenance kept
in config. Fruit masses differ by rank (total grams):6=74.7085,7=69.3652,
8=62.6156,9=56.2561,10=57.2251. Fruit sphere sizes/inertias are coherent at
1000 kg/m3. Support density20000 kg/m3 remains artificial and unchanged.
This is not validation of original canonical v2.3 construction or biological
support masses. No rank6 profile is copied onto the other ranks.

`prepare_rank_campaign.py` selects whole trusses by rank from the preserved
five-truss fixture, retaining full fixed stem and original attachments. Prunes
removed body/visual roots and their filtered collision references. Attribute
diff checks prohibit retained-body geometry, mass or drive changes. Expected
loaded mass, COM and inertia are checked against the source runtime evidence.

Progression:60 s rest for each rank,60 s recorded native gesture per rank
(30 s settling, translated same slow clip, hold through physical break until
recorded release). Then five together at rest and with selected-rank gestures,
followed by user GUI review. Every run starts from the initial scene. Do not
start v2.3 integration or claim generic success before combined/manual results.

Heavy artifacts: `artifacts/detachable_fruit_v2/original-rank-campaign/`.
Compact reports: `original_rank_results.json`, generated with
`summarize_rank_campaign.py`; pending is never a pass.

Initial rest: all five individual ranks pass60 s, no breaks/errors. Native
translation-only clips pass ranks7,8,10. Ranks6/9 initially abort before applying
mouse force because ray hits another fruit (respectively1_R and2_L instead of
3_L). These are recorded selection failures, not physical instability.
`rotate_native_recording.py` rotates recorded rays rigidly about the source
fruit center using source/target first-rachis frames from recorded poses. Exact
rotation and pose-file hashes are saved. Scene physics untouched, raycast target
check remains strict. Rank6 reoriented replay passes; rank9 repeat pending.

Both reoriented single-rank replays pass. Combined five-truss rest passes60 s
with40 attached fruits. Start five independent combined-scene native runs,
one target per rank; rotate every clip into its target frame for a consistent
selection protocol. Unrotated five-native configs are preparation intermediates,
marked preparation_only; not executed and excluded from the result table.

## Headless result

16 completed60 s checks pass: five individual rests, five individual native
detachments, combined rest, five independent combined-scene detachments. Each
native report has exactly one JOINT_BREAK on the selected fruit and continuity
passes. All rest reports have zero breaks. No reported physical/validation
errors in these completed runs. The two aborted selection attempts remain in
the result table as runtime_error and are not counted as passes.

Current runtime hashes: `original_rank_manifest.json`. Inherited config hashes
from the source fixture are historical provenance, not the runtime manifest.
No changes to the v2.3 adapter/builder in this campaign.

GUI `gui-2ri67q54` opened with full five-truss scene, real-time pacing and3 N.
Initial cadence approximately60 FPS. Manual review/full-run FPS pending.

```bash
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_prepared_gui.py \
  artifacts/detachable_fruit_v2/original-rank-campaign/five-rest --real-time
```

Scope remains day160, ranks6–10 on the main-derived fixed stem, artificial
support density20000 kg/m3, native force50. Does not validate lateral support,
full vegetation, canonical v2.3 construction, other days or arbitrary fruit counts.
