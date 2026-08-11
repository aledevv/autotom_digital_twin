# Visual Truss Lab

This isolated lab compares proposed truss states without changing the official
V2 parser, truss builder, optimizer, joint configuration, or budget config.

## Generate

From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 UV_CACHE_DIR=/tmp/uv-cache uv run python \
  src/exporterV2/adapters/groimp_csv/tests/truss_visual_lab/generate_truss_visual_lab.py
```

The files are written to `generated/` beside this README.

## Isaac Sim Checklist

| File | Expected D6 joints | Inspect |
| --- | ---: | --- |
| `00_current_simplified.usda` | 4 | Reference geometry. Rachis is articulated; pedicels are fixed. |
| `01_dynamic_pedicels.usda` | 11 | Seven softer pedicel D6 attachments, each limited to +/-25 degrees. Press Play and check the more evident gravity-driven droop. |
| `02_opt_fixed_pedicels.usda` | 4 | First optimization step. Geometry should match stage 01 at rest, but pedicels no longer move. |
| `03_opt_static_prebent_truss.usda` | 1 | Five fixed visual pieces form a constant drooping curve. The whole rigid truss is attached to the stem by one softer D6, and pedicels are tilted 10 degrees farther toward the ground. |

For each stage:

- Confirm all seven tomatoes remain attached to their pedicels.
- Compare the at-rest silhouette and fruit spacing.
- In the Physics Inspector, distinguish `PhysicsJoint` (D6) from `PhysicsFixedJoint`.
- Press Play for all stages and watch for instability, pedicel travel, fruit collisions, and movement of the final truss as one block.
- Treat stage 03 as a visual prototype: its piecewise curve is intentionally local to this lab.

The trunk is fixed in every file so the comparison isolates truss behaviour.
