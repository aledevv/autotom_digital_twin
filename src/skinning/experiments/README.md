# Skinning Experiments

Spike isolato per validare la pipeline `UsdSkel + PhysX` prima di integrarla in `exporterV2`.

## Struttura

```
experiments/
├── README.md           ← questo file
├── common/             ← codice condiviso tra più esperimenti (se necessario)
├── test_0a_static/     ← Test 0A: mesh UsdSkel statica
│   ├── generate.py
│   ├── run.py
│   ├── run.sh
│   └── output/
└── test_0b_runtime/    ← Test 0B: animazione sinusoidale runtime (CURRENT)
    ├── generate.py
    ├── run.py
    ├── run.sh
    └── output/
```

## Sequenza Spike 0

| Test | Stato      | Obiettivo                                         |
|------|------------|---------------------------------------------------|
| 0A   | ✅ Done     | UsdSkel statico — mesh continua con 3 bones       |
| 0B   | ✅ CURRENT  | Runtime — bones oscillano a sinusoide via Python  |
| 0C   | TODO        | Bridge PhysX → UsdSkel: pose D6 → skin runtime   |

## Test 0A — UsdSkel statico

```bash
bash src/skinning/experiments/test_0a_static/run.sh
```

**Success criteria:** tubo curvo 0°→25°→45°, nessun gap, binding valido.

## Test 0B — Runtime animation

```bash
bash src/skinning/experiments/test_0b_runtime/run.sh
```

Bone1 oscilla a `±30° · sin(1.5t)`, Bone2 con sfasamento di 60°.

**Success criteria:**
- Mesh oscilla continuamente senza freeze
- Hydra aggiorna la mesh ogni frame (fps stabile)
- Nessun conflitto con l'evaluator USD
- Nessun twist inatteso o perdita di volume evidente

## Test 0C — PhysX → Skeleton bridge (TODO)

Pose PhysX (rigid body D6) → aggiornamento SkelAnimation → skin runtime.
È il **GO / NO-GO** dell'intera architettura.

## Riferimento

Piano completo: [`src/skinning/Centerline-Based Visual Geometry & Physics-Driven Skinning.md`](../Centerline-Based%20Visual%20Geometry%20%26%20Physics-Driven%20Skinning.md)
