# Visual Validation — Checklist per Isaac Sim

Questa directory contiene DUE test suite:

1. **Sequential stages** (`run_visual_validation.py`) — 6 USD che mostrano l'effetto cumulativo delle tecniche applicate in sequenza
2. **Technique combinations** (`generate_combinations_usd.py`) — 8 USD che confrontano ogni tecnica/combinazione contro la baseline comune

## Suite 1: Sequential Stages (Cumulative)

Questa suite genera 6 file USD per verificare visivamente ogni tecnica di ottimizzazione applicata sequenzialmente.  
Carica i file in Isaac Sim uno alla volta e segui il checklist per ogni stage.

## Come eseguire

```bash
cd autotom_digital_twin
uv run python src/exporterV2/demos/optimization_visual_validation/run_visual_validation.py
```

Output: `usd_output/` con 6 file `.usda`

## Caricare in Isaac Sim

```bash
~/isaacsim/python.sh -m isaacsim 'src/exporterV2/demos/optimization_visual_validation/usd_output/0_baseline.usda'
```

Sostituisci il filename per ogni stage.

---

## Stage 0 — Baseline (`0_baseline.usda`)

Pianta originale, nessuna ottimizzazione applicata.

**Struttura attesa:**
- Trunk: 10 segmenti
- 5 rami laterali: 5 segmenti ciascuno
- 8 foglie composte: petiole (2 link) + rachis (3 link) + 3 petioluli (1 link)

**Checklist:**
- [ ] La pianta appare eretta con trunk visibile
- [ ] 5 rami laterali distribuiti lungo il trunk
- [ ] Ogni foglia ha una struttura articolata (petiole → rachis → petioluli)
- [ ] Avviando la simulazione fisica, tutti i segmenti oscillano liberamente
- [ ] I petioluli si muovono autonomamente rispetto al rachis

**D6 joints attesi: ~99**

---

## Stage 1 — Petiole Lock (`1_petiole_lock.usda`)

I petioluli sono convertiti da D6 (articolati) a Fixed (statici).

**Differenze rispetto al baseline:**
- Geometria identica (stessa posizione, stessa forma)
- I petioluli hanno `joint_type: fixed` nel metadata USD

**Checklist:**
- [ ] La pianta ha geometria identica al baseline (visivamente uguale a riposo)
- [ ] Avviando la simulazione, i **petioluli NON oscillano** — rimangono rigidi
- [ ] Il resto della pianta (trunk, laterali, petiole, rachis) oscilla normalmente
- [ ] Non ci sono collisioni o artefatti visivi introdotti

**D6 joints attesi: ~75** (24 petioluli → Fixed, non contano)

---

## Stage 2 — Lateral Reduce (`2_lateral_reduce.usda`)

I rami laterali sono ridotti a 1 segmento ciascuno (da 5).

**Differenze rispetto allo stage 1:**
- I rami laterali passano da 5 link a 1 link
- La lunghezza totale di ogni ramo è preservata (height per link aumenta)
- Le foglie attaccate ai rami vengono rimappate

**Checklist:**
- [ ] I rami laterali appaiono più "rigidi" (meno segmenti = meno articolazione)
- [ ] La lunghezza dei rami laterali è visivamente simile al baseline
- [ ] Le foglie sui rami laterali sono ancora attaccate correttamente
- [ ] Non ci sono rami laterali "spariti" o staccati dal trunk
- [ ] In simulazione i rami oscillano ma con meno gradi di libertà

**D6 joints attesi: ~55** (25 link laterali → 5)

---

## Stage 3 — Stem Collapse (`3_stem_collapse.usda`)

Il trunk principale è ridotto da 10 a 3 segmenti. Tutti i figli rimappati con `attach_frac`.

**Differenze rispetto allo stage 2:**
- Il trunk ha 3 segmenti invece di 10
- I rami laterali e le foglie sono rimappati alle stesse altezze assolute
- `attach_frac` specifica la posizione precisa dentro il segmento target

**Checklist:**
- [ ] Il trunk appare più corto in termini di articolazioni (meno segmenti visibili)
- [ ] L'altezza totale della pianta è invariata (solo i segmenti sono più lunghi)
- [ ] I 5 rami laterali sono ancora distribuiti lungo il trunk (non tutti in cima)
- [ ] Le foglie direttamente sul trunk sono nella posizione corretta
- [ ] ⚠️ Verifica che nessun ramo sia "saltato" in cima al trunk

**D6 joints attesi: ~48** (10 trunk link → 3, -7)

---

## Stage 4 — Leaf Branch Reduce (`4_leaf_branch_reduce.usda`)

Petiole e rachis di ogni foglia sono fusi in un singolo segmento.

**Differenze rispetto allo stage 3:**
- Il petiole di ogni foglia diventa un segmento merged (petiole + rachis)
- Il rachis viene rimosso
- I petioluli sono rimappati con `attach_frac` per preservare la posizione assoluta

**Checklist:**
- [ ] Ogni foglia ha ora un solo segmento (invece di petiole + rachis separati)
- [ ] I petioluli sono ancora presenti e distribuiti lungo la foglia (non tutti in cima)
- [ ] La lunghezza totale della foglia (petiole + rachis) è preservata
- [ ] In simulazione la foglia si muove come un'unica asta rigida
- [ ] Nessun petiolulo è "orfano" o in posizione errata

**D6 joints attesi: ~24** (8 rachis × 3 link rimossi = -24)

---

## Stage 5 — Fully Optimized (`5_fully_optimized.usda`)

Risultato del pipeline `BudgetOptimizer` completo (tutte le tecniche in sequenza).

Questo file dovrebbe essere identico o molto simile allo stage 4 se il budget è rispettato dopo le prime tecniche.

**Checklist:**
- [ ] Il report a console indica `success: True`
- [ ] Il numero di joints finali è ≤ budget (default: 250)
- [ ] La pianta è strutturalmente integra (nessun branch orfano)
- [ ] Confrontando con il baseline: la pianta è visivamente simile ma più "rigida"

---

## Confronto rapido

| Stage | File USD | D6 Joints | Cosa cambia |
|-------|----------|-----------|-------------|
| 0 Baseline | `0_baseline.usda` | ~99 | Pianta originale |
| 1 Petiole Lock | `1_petiole_lock.usda` | ~75 | Petioluli statici |
| 2 Lateral Reduce | `2_lateral_reduce.usda` | ~55 | Rami laterali meno segmentati |
| 3 Stem Collapse | `3_stem_collapse.usda` | ~48 | Trunk compresso |
| 4 Leaf Reduce | `4_leaf_branch_reduce.usda` | ~24 | Foglie fuse |
| 5 Fully Optimized | `5_fully_optimized.usda` | ≤250 | Pipeline completo |

> I valori D6 joints sono stime basate sulla pianta sintetica generata dallo script.
> I valori esatti vengono stampati a console durante `run_visual_validation.py`.

---

## Cosa segnalare come problemi

Se noti uno di questi comportamenti, c'è un bug:

- **Rami che spariscono** dopo una tecnica → problema nell'applicazione o validation
- **Pianta che collassa** in simulazione → attachment non rimappato correttamente
- **Geometria che si distorce** (pianta troppo lunga/corta) → height non ricalcolata
- **Petioluli che "saltano"** in posizione sbagliata → `attach_frac` non propagato al builder USD
- **Stage 5 diverso da stage 4** in modo inatteso → ordine tecniche nel `budget_config.yaml` sbagliato

---

## Suite 2: Technique Combinations (Baseline-Relative)

Per confrontare ogni tecnica o combinazione contro la **stessa baseline**, usa la seconda suite:

**Genera USD:**
```bash
uv run python src/exporterV2/demos/optimization_visual_validation/generate_combinations_usd.py
```

**Carica in Isaac Sim (side-by-side):**
```bash
~/isaacsim/python.sh src/exporterV2/demos/optimization_visual_validation/load_combination_isaacsim.py --combo 1
```

**Test non-visuali:**
```bash
uv run pytest src/exporterV2/demos/optimization_visual_validation/validate_combinations.py -v
```

**Documentazione completa:** Vedi [`COMBINATIONS_README.md`](./COMBINATIONS_README.md) per:
- Lista completa delle 8 combinazioni (ID 0-7)
- Joint count attesi per ogni combo
- Checklist verifica per ogni tecnica
- Comandi Isaac Sim per ogni combo
