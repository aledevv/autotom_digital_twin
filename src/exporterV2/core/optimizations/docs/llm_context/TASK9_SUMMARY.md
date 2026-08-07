# Task 9 - Integration Tests: Composizione Tecniche

## Obiettivo

Verificare che le tecniche di ottimizzazione funzionino correttamente in sequenza: dal conteggio iniziale dei joints fino al rispetto del budget finale, su piante sintetiche e reali da CSV.

## Scenari Testati

File: `tests/9_integration/test_technique_composition.py` — **6/6 test passati**

### Scenario 1 — Pianta over budget, ottimizzazione riuscita

```
Plant: 195 joints (sintetica: trunk+laterali+foglie+petioluli)
Budget: 150 joints
Result: ✓ Riduzione a 135 joints (petiole_lock applicato)
```

Verifica:
- Almeno una tecnica applicata
- `joints_before` di ogni tecnica = `joints_after` della precedente
- Budget rispettato alla fine

### Scenario 2 — Pianta dentro budget, nessuna ottimizzazione

```
Plant: 17 joints
Budget: 250 joints (default)
Result: ✓ Nessuna tecnica applicata, pianta invariata
```

Verifica:
- `technique_reports` vuoto
- `branches` output == input

### Scenario 3 — Budget impossibile (sotto lower bound)

```
Plant: 195 joints
Lower bound: 6 joints (trunk + 5 laterali min 1 link ciascuno)
Budget: 5 joints (sotto lower bound)
Result: ✓ ValueError con messaggio "impossible" + "lower bound" + valore
```

Verifica:
- `ValueError` sollevato
- Messaggio include lower bound numerico
- Messaggio dice che è impossibile

### Scenario 4 — Riduzione progressiva verificata

```
Plant: 195 joints (default budget)
Result: ✓ Ogni tecnica riduce o mantiene, mai aumenta
```

Verifica:
- Ordine tecniche rispetta priority
- `joints_after <= joints_before` per ogni tecnica
- `joints_saved >= 0` sempre

### Scenario 5 — Pianta reale da CSV

```
CSV: data/simulation_output/dynamic_output/graphs/graph_day_*.csv (day 30-50)
Result: ✓ Parser integrato, ottimizzazione completata
```

Verifica:
- `parse_csv_to_branches()` funziona
- Joints non aumentano dopo ottimizzazione
- Output non vuoto

### Scenario 6 — Formattazione report

```
Result: ✓ Report leggibile con sezioni chiare
```

Verifica:
- "Joint-Budget Optimization Report" nel testo
- "Original joints:", "Final joints:", "Budget:" presenti
- Indicatore ✓/✗ corretto

## Pianta Sintetica Usata

```
Trunk:        1 × 10 links = 10
Lateral:      5 × 5 links  = 25
Petioli:     20 × 2 links  = 40
Rachis:      20 × 3 links  = 60
Petioluli:   60 × 1 link   = 60
─────────────────────────────────
Totale:                      195 D6 joints iniziali
Dopo petiole_lock:           135 D6 joints (60 petioluli → Fixed)
```

## Bug Fix Risolto in questa Task

**Problema**: i joints Fixed (petioluli locked) venivano contati nel budget insieme ai D6, causando mismatch nei report delle tecniche.

**Soluzione**: `count_d6_joints()` in `base.py` esclude branch con `joint_type='fixed'`:

```python
def count_d6_joints(branches):
    return sum(
        b.get("n_links", 1) for b in branches
        if b.get("joint_type", "d6").lower() != "fixed"
    )
```

Aggiornati tutti i file:
- `techniques/petiole_lock.py`
- `techniques/lateral_reduce.py`
- `techniques/stem_collapse.py`
- `techniques/leaf_branch_reduce.py`
- `optimizer.py` (`calculate_total_joints`)

## Note Task 10 (Visual Validation)

Per verificare visivamente le differenze tra le tecniche, la Task 10 dovrà generare USD separati per ogni stadio di ottimizzazione. Struttura suggerita:

```
tests/visual_validation/
├── README.md          ← checklist manuale per IsaacSim
├── run_visual_test.py ← genera tutti gli USD + stampa diff
├── usd_output/
│   ├── 0_baseline.usda
│   ├── 1_petiole_lock.usda
│   ├── 2_lateral_reduce.usda
│   ├── 3_stem_collapse.usda
│   ├── 4_leaf_branch_reduce.usda
│   └── 5_fully_optimized.usda
```

Ogni USD deve avere accanto a sé un report testuale con:
- Joint count prima/dopo
- Tecniche applicate
- Diff strutturale (branches aggiunti/rimossi/modificati)
