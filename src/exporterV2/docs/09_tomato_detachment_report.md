# Tomato Detachment Runtime - WIP Report

## Status

This branch isolates the experimental tomato-detachment implementation. It is
not ready to merge into the stable exporter yet.

The first backend represented an attached tomato as an articulation link, then
deleted that link and called `World.reset()` to rebuild PhysX. Minimal scenes
passed pose, velocity, force-sensing, multi-fruit, and reset tests. On a full
day-80 plant, however, rebuilding the large articulation caused a segmentation
fault in Isaac Sim's physics warm start. Reading all wrench tensors every
physics step also contributed substantial interactive slowdown.

## Current Strategy

The WIP replacement keeps articulation topology constant:

- attached tomato geometry is a collider in the pedicel's compound rigid body;
- tomato mass, center of mass, and inertia are aggregated into that pedicel;
- a hidden, collision-disabled kinematic proxy exists under
  `/World/DetachedTomatoes`;
- detachment removes the tomato mass properties from the pedicel, hides its
  attached collider, transfers world pose and point velocity, and makes the
  proxy dynamic.

This avoids `World.reset()`, articulation-view reconstruction, and terminal
tomato links. The runtime samples articulation wrenches at 60 Hz while physics
runs at 480 Hz. Its event/state API remains independent from RL reward or
termination logic.

## Configuration

`TrussPhysicsConfig` exposes the model, sensor rate, force and torque
thresholds, exponents, persistence, diagnostics, and detached-body path. The
current provisional thresholds are `0.60 N` and `0.05 N*m`, with `0.020 s`
continuous overload persistence. They require physical calibration.

## Verification

Pure runtime and USD tests pass with `uv`; the focused detachment suite reports
`7 passed`. The compound-body backend still requires the Isaac gate:

```bash
~/isaacsim/python.sh src/exporterV2/core/detachment/tests/isaac_detachment_smoke.py
```

Acceptance requires nonzero detached-proxy motion, continuous pose/velocity,
constant articulation body count, repeatable reset, and no topology rebuild.
After that, profile day 50 and optimized day 80 for both real-time performance
and interactive Shift-click detachment before considering integration.
