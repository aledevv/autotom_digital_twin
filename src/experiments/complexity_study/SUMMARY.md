# How heavy is the tomato digital twin? (complexity study, day 160)

## 1. Goal

We want to say, with numbers, how "heavy" the tomato plant model is to simulate, so that someone with different hardware (for example an industrial PC without a big GPU) can estimate whether it will run.

We do not use a single made-up "complexity score". Instead we look at three levels and link them with measurements:

| Level | Question | Depends on hardware? | Examples |
|---|---|---|---|
| A. Plant | How complex is the plant itself? | No | organs, internodes, branching order |
| B. Model | How heavy is the simulation model we built? | No | joints, degrees of freedom (DOF), rigid bodies, colliders, triangles |
| C. Cost | How long does it take on a given PC? | Yes | ms per physics step, ms per frame, FPS |

The link between B and C is a simple cost formula fitted on measurements, for example *"each DOF costs X µs, each body costs Y µs"*.

**Key terms**
- **D6 joint**: a flexible joint that can bend in 2 directions, so it has **2 DOF**.
- **Fixed joint**: a rigid connection with 0 DOF. The two parts still exist as separate bodies.
- **Rigid body (link)**: one piece of the plant that PhysX moves as a solid block.

Test machine: Intel i9-13900HK laptop, NVIDIA RTX 4080 Laptop GPU (12 GB), Isaac Sim 4.5 / PhysX 5. Plant: GroIMP tomato at day 160.

---

## 2. What we built to run the experiments

| Tool | What it does |
|---|---|
| `complexity_profile.py` | Reads a USD file and counts everything for level A and level B: organs, DOF, joints by type, bodies, colliders, triangles, solver settings. |
| Joint-budget optimizer (`--joint-budget N`) | Makes simpler versions of the **same** plant with fewer D6 joints. It can *lock* joints (D6 → fixed) or *merge* links (two bodies → one body). The exact GroIMP geometry is kept: nothing moves or changes shape. |
| `fidelity_bench.py` | **Static test**: lets each plant settle under gravity, then compares the final position of every one of the 558 visual pieces with the full reference plant. |
| `push_bench.py` | **Dynamic test**: gives the same sideways push to the same three organs of every plant and records how they swing back. A GUI mode shows the push with arrows and live numbers. |
| `interactive_bench.py` | **Cost test**: opens the plant in the Isaac Sim GUI (RTX rendering, 60 Hz physics) and measures the time per frame, split into physics and rendering. |
| `analyze.py`, `make_figures.py` | Fit the cost formula and draw the figures. |

---

## 3. Experiments: what we did and how

### 3.1 Hard limit of PhysX on plant size

**What**: The old notes spoke of a limit of "about 250 joints", without a clear test. We looked for the exact limit.

**How**:
1. We took one plant and merged only *fixed* pieces, which does not change the physics.
2. This gave versions with exactly 255, 256 and 257 bodies in one articulation, plus a 347-body version.
3. We opened each one in Isaac Sim, with both GPU and CPU physics.

**Result**:

| Bodies in one articulation | GPU physics | CPU physics |
|---|---|---|
| 216 (production model) | works | works |
| 255 | works | works |
| 256 or more | crash (CUDA error 700) | crash (segmentation fault) |

**Observations**
- The real limit is **255 bodies (links) per articulation**, not "250 joints". In the old pipeline joints and bodies were almost the same number, which is why it looked like a joint limit.
- It is a hard crash, not a memory problem.
- The exporter now refuses models above 255 bodies with a clear error message, instead of crashing Isaac Sim.

### 3.2 Making simpler versions of the same plant

**What**: To study cost against accuracy we need the same plant at several complexity levels.

**How**: The optimizer applies, in order of priority:
1. lock petiolules and pedicels (D6 → fixed);
2. merge the segments of side branches, of the stem and of the truss rachis;
3. merge the petiole and rachis of each leaf.

When two bodies are merged, the removed joint's flexibility has to go somewhere. A **compliance policy** decides how much of it is moved to the joint above:

| Policy | Idea |
|---|---|
| `keep` | merged part becomes rigid (behaviour of the old optimizer) |
| `series` | move all the flexibility up |
| `load` | weight it by the gravity load each joint carries |
| `tip` | weight it for a force at the tip of the branch |
| `blend` | average of `load` and `tip` (**current default**) |

**Variants used** (starting from the production model):

| Variant | D6 joints | Bodies |
|---|---|---|
| Reference (production model) | 206 | 216 |
| Budget 120 | 116 | 191 |
| Budget 60 | 60 | 135 |
| Budget 40 (minimum: 1 joint per branch, leaf and truss) | 40 | 115 |
| Same joints, more bodies (physical petiolules) | 206 | 255 |

### 3.3 Static accuracy (gravity)

**What**: Does the simpler plant hang in the same shape as the full plant?

**How**:
1. Each plant settled for 5 s under gravity (480 Hz, headless).
2. We measured the final position of all 558 visual pieces (stem, leaves, trusses).
3. We computed the distance of each piece from the same piece in the reference plant.

Re-running the reference gives exactly 0 mm difference, so the simulation is repeatable and the differences are real. For scale: in the reference, pieces sag by about **15 mm** on average under their own weight.

| Version (40 D6) | Mean position error |
|---|---|
| `load` | **3.0 mm** |
| `blend` | 5.1 mm |
| `keep` (old optimizer) | 7.7 mm |
| `tip` | 9.4 mm |
| `series` | 37.6 mm |

### 3.4 Dynamic accuracy (push)

**What**: Does the simpler plant move like the full plant when touched?

**How**:
1. All plants were placed side by side in one scene and settled for 6 s at 60 Hz.
2. Three organs received the same sideways push for 0.2 s: the tip of the longest side branch, the tip of a truss and a leaf blade.
3. The push force is equal to the weight of the plant part beyond the pushed point (0.36 / 0.03 / 0.10 N).
4. We recorded the swing for 4 s and compared the **peak movement** and the **oscillation frequency** with the reference.
5. The push was checked visually in the GUI before running it headless.

| Version (40 D6) | Error on peak movement | Error on frequency |
|---|---|---|
| `tip` | **18%** | 11% |
| `blend` | 29% | **1.3%** |
| `series` | 33% | 38% |
| `load` | 43% | 14.5% |
| `keep` (old optimizer) | 70% | 58% |

**Observations**
- **No policy is best for both statics and dynamics.** `load` is best at rest, `tip` best for pushes, and `blend` is a good compromise: it matches the swing frequency almost exactly. That is why it is now the default.
- The old optimizer behaviour (`keep`) is the worst: the plant becomes much too stiff.
- At budget 120 the errors are already small (`tip`: 7% on the peak).

### 3.5 Cost in the interactive (GUI) regime

**What**: How many ms per frame does the plant cost when you use it interactively, which is the case that matters for us?

**How**:
1. Each variant was opened in a fresh Isaac Sim GUI: RTX viewport at 1600×900, physics at 60 Hz, no frame-rate cap (to measure capacity).
2. We measured 600 frames each of: plant at rest, plant being pushed, paused (rendering only), and physics only.
3. Everything was run with **GPU physics** and with **CPU physics**, twice, in alternating order.

| Physics on | D6 / bodies | Physics per step | Total frame | FPS |
|---|---|---|---|---|
| GPU | 206 / 216 (reference) | 67.6 ms | 81.2 ms | 12 |
| GPU | 40 / 115 | 25.6 ms | 42.7 ms | 25 |
| **CPU** | 206 / 216 (reference) | **10.2 ms** | **18.8 ms** | **53** |
| CPU | 116 / 191 | 8.9 ms | 17.2 ms | 58 |
| CPU | 60 / 135 | 5.9 ms | 13.8 ms | 73 |
| CPU | 40 / 115 | 5.0 ms | 12.8 ms | 78 |
| CPU | 206 / 255 | 11.5 ms | 20.3 ms | 49 |

**Fitted cost formula** (physics time per step):

| Physics on | Formula | Fit quality |
|---|---|---|
| CPU | 0.1 ms + **2.8 µs per DOF** + **41 µs per body** | R² 0.995 |
| GPU | **16.6 ms fixed** + 122 µs per DOF | R² 0.998 (DOF only, see below) |

**Observations**
- **For one plant, CPU physics is 5–7× faster than GPU physics.** The GPU has a fixed cost of about 17 ms per step, which is the overhead of starting GPU work for a small model. The GPU only pays off with many plants in parallel. **So you do not need a big GPU to run this model interactively.**
- On CPU, the production model already runs at **53 FPS**; the 40-D6 version at **78 FPS**.
- On CPU, **bodies cost about 15× more than DOF**. Removing bodies, for example by merging fixed pieces (which costs no accuracy), is the most useful optimization.
- Rough budget on this laptop with CPU physics: about **128 D6 joints fit in 60 FPS**.
- Timing noise: CPU results are stable between runs (under 8%), GPU results vary a lot (25–43%), probably because the laptop GPU changes its clock speed. We always report the median of repeated runs.
- On the GPU, DOF and bodies could not be separated (the body term came out negative), so the GPU formula uses DOF only.
- The model is currently saved with **GPU physics**, which is the slow option for interactive use.

---

## 4. Main take-aways

1. The real size limit is **255 bodies per articulation**. Above that Isaac Sim crashes, on GPU and CPU.
2. The optimizer can cut the plant from 206 to 40 joints while keeping the exact shape. With the `blend` policy it stays within about 5 mm at rest and matches the swing frequency within about 1%.
3. Accuracy depends on the task: statics, pushes and swing frequency prefer different policies.
4. **CPU physics is the right choice for a single interactive plant**: 53 FPS for the full model on a laptop CPU, against 12 FPS with GPU physics.
5. Simple cost model on CPU: about 3 µs per DOF and 41 µs per body per physics step, plus about 8 ms of rendering per frame.

Figures, in `src/experiments/complexity_study/figures/`:
- `complexity_skeletons.png`: the plant at 206 / 116 / 60 / 40 joints, with joints shown as blue dots;
- `cost_vs_fidelity.png`: speed-up against static and dynamic error, one line per policy;
- `push_trajectories.png`: swing of the three pushed organs, reference against variants;
- `interactive_cost.png`: physics cost per step and frame breakdown, GPU against CPU.

---

## 5. Still to do

- Repeat the accuracy tests (gravity and push) with **CPU physics**: all accuracy results so far used GPU physics.
- If CPU physics behaves the same, make it the default for the interactive model.
- Measure more points for the cost formula:
  - other plant ages (day 50, day 80);
  - solver iterations (8 / 16 / 32 / 64);
  - PGS against TGS solver;
  - number of CPU threads.
- Separate the cost of DOF and bodies on the GPU (needs more, and less noisy, data points).
- Decide whether the optimizer should merge links by default (`--optimizer-techniques full`) or only lock joints (`lock`, the current default).
- Correct the old documentation that still mentions "~250 joints" and "64 links per articulation".
- Optional: a test with a real interaction (scripted mouse or gripper drag) and with fruits attached.
