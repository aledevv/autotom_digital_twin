# Native GUI freeze diagnosis

Run on the experimental branch only. The custom controller is not used.

```bash
cd /home/alessandro/isaacsim/autotom_digital_twin
UV_CACHE_DIR=/tmp/autotom-uv-cache uv run --no-sync python \
  src/experiments/detachable_fruit_v2/run_gui_freeze.py
```

Each invocation prepares a fresh local `artifacts/detachable_fruit_v2/gui-freeze-*`
folder from main-rachis: one fruit, density 2000 kg/m³, original mobile joints,
6 N, native joint mouse coefficient 10, TGS/GPU 60 Hz, 32/4 and 32/1 iterations.
No pacing or model changes. The duration argument is not a termination condition
in this mode. Close the window or use Ctrl+C in the terminal. Stop/pausing leaves
the application open; a reset invalidates continuous physical measurements and
requires a fresh run for another physical trial.

For a manual trial use real time: 30 seconds rest, 30 seconds slow dragging with
cursor holds, 30 seconds progressively faster oscillation, 30 seconds recovery.
If the fruit detaches early, observe recovery and use a fresh launch for the
oscillation phase. No fruit is recreated. Wait at least 6 seconds after a freeze
before closing to allow stack capture. Shutdown is monitored separately.

Evidence:
- `launch.json`: exact command, git HEAD and source hashes; config/controls contain
  scene/input hashes and actual changes. `isaac.log` retains startup and shutdown.
- `gui-effective.json`: loaded masses, inertias, articulation and solver settings.
- `gui-phases.jsonl`: phase entry/exit with monotonic times and simulated time.
- `watchdog-state.bin`: latest phase, readable by an independent process.
- `watchdog.jsonl`: one-second snapshots, process status and stalls after 5 seconds.
- `python-stacks.log`: SIGUSR1/faulthandler stacks requested on stalls; no kill.
- `gui-partial.json`, per-step trace chunks and frame chunks: one-second persistence
  during running physics; native events are flushed on each callback.
- Final report includes explicit termination reason, actual JOINT_BREAK events,
  FPS/frame times and simulation/wall-clock ratio. No break is not a GUI freeze.

The watchdog labels stalls, not confirmed deadlocks: slow cleanup may recover.
Python stacks locate the Python call boundary, not necessarily the native root
cause. Journaling adds overhead; performance is measured, not assumed unchanged.
All frame summaries stay in memory until close, while full physical traces are
chunked to disk. This diagnostic mode is intended for the bounded manual session,
not unattended multi-day runs.

Only after reproduction, one comparison may use `--disable-native-observer`.
This removes the raycast/event-recording proxy while retaining native interaction,
physics and the watchdog. Breaks without mouse attribution remain unclassified.

## Verification and observations, 2026-09-14

- Direct `cuInit(0)` succeeded.
- Synthetic independent-process test: running stall and cleanup stall both captured
  Python stacks; monitored process survived and completed. No GPU hang induced.
- Final targeted suite: 41 tests passed (diagnostics, interaction, support matrix,
  native forwarding and independent watchdog).
- First launcher attempt exposed an early import path error; corrected before GUI.
- `gui-freeze-tcj_k9s_`: user interacted during the intended short smoke test.
  User reported good movement/detachment, then freezing after Stop. True break at
  22.5333 simulated seconds; timeline reset detected at 27.0167. Initial monitor
  incorrectly routed the reset into app.close. Stack after >5 seconds showed
  extension shutdown/RTX sensor garbage collection, followed by a segmentation
  fault. The user report and the tool-driven Ctrl+C overlap: not a controlled
  proof of mouse-induced failure. Stop now leaves the diagnostic GUI open.
- `gui-freeze-7gwjdebt`: the short test sent Ctrl+C at 5.56 real / 7.67 simulated
  seconds while the user was testing. User reported freezing after touching fruit.
  Logs show mouse release at 5.8667 simulated seconds and continued physics before
  Ctrl+C. Cleanup lasted about 28.4 real seconds, then completed normally. Stack
  located extension module discovery in app.close. This is a confounded trial,
  not evidence that grabbing caused the stall. No subsequent manual run uses an
  automated close or signal.
- `gui-freeze-ivyig73s`: fresh manual-only trial launched; feedback pending.

No physical model or remake changes were made. A slow/failed shutdown is observed;
mouse-induced freezing during active simulation is not yet established.

### First manual-only result

`gui-freeze-ivyig73s`: user says the movement appears stable, and separately flags
an oversized-looking tomato relative to its pedicel. Geometry was unchanged in
this diagnostic phase; visual proportions require a separate check.
The initial sequence recorded 17.05 simulated seconds, with a selected-fruit
JOINT_BREAK at 10.2667 seconds, mean active-render FPS 85.05 (steady 85.84).
The timeline was reset/restarted during this session; a second break at reset time
4.2 seconds must not be counted in the initial sequence. The observer now stores
post-reset events separately as unvalidated, rather than mixing time origins.
Window close was explicit, exit code 0. Cleanup took 6.31 real seconds, exceeded the
5-second alert threshold, and recovered. The stack sampled plugin unloading in
SimulationApp.close. This session is shorter than the planned two real minutes;
it does not establish sustained stability or resolution of all freezing symptoms.
