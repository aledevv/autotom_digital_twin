# Task 5 - Lateral Branch Reduction

## Obiettivo

Ridurre incrementalmente il numero di segmenti (`n_links`) nei rami laterali, con ricalcolo dell'altezza e remapping degli attachment points dei figli.

## Rationale

I rami laterali hanno spesso 3-5 segmenti ma possono essere ridotti a 1 senza perdita strutturale grave. Riduzione incrementale (un link per volta, dal ramo meno importante) permette di controllare il tradeoff accuratezza/risparmio.

## Implementazione

File: `techniques/lateral_reduce.py`

### Identificazione rami laterali

```python
# Pattern: "Branch_r*_o*" e "LateralLeaf_r*_o*"
def _is_lateral_branch(self, branch):
    return ("Branch_r" in branch["id"] or "LateralLeaf_r" in branch["id"])
```

### Strategia di priorità (quale ramo ridurre prima)

Per ogni iterazione, il ramo da ridurre viene scelto in ordine:
1. **Raggio più piccolo** (rami sottili prima)
2. **Attach link più basso** (rami in basso prima)
3. **Alfabetico** (determinismo)

Questo minimizza l'impatto visivo: i rami sottili e bassi sono meno prominenti.

### Applicazione

Per ogni ramo selezionato:

```python
# 1. Riduci n_links di 1
branch["n_links"] -= 1

# 2. Ricalcola height per mantenere lunghezza totale invariata
branch["height"] = branch["height"] * old_n_links / branch["n_links"]

# 3. Remap children attachment
for child in children_of(branch):
    new_link, new_frac = remap_link_attachment(
        child["attach_link"], old_n_links, branch["n_links"]
    )
    child["attach_link"] = new_link
    child["attach_frac"] = new_frac
```

Il remapping usa la funzione di Task 3 che preserva l'altezza assoluta del punto di attacco.

### Rispetto del minimo strutturale

`min_segments` (default: 1) garantisce che ogni ramo mantenga almeno 1 link.

## Test

File: `tests/5_lateral_reduce/test_lateral_reduce.py` — **13/13 test passati**

| Test | Cosa verifica |
|------|---------------|
| `test_identify_lateral_branches` | Pattern naming |
| `test_can_reduce` | Presenza rami riducibili |
| `test_reduction_priority` | Ordine priorità (raggio/altezza/alfa) |
| `test_can_apply` | `can_apply()` corretto |
| `test_estimate_reduction` | Stima riduzione |
| `test_apply_single_branch` | Riduzione singolo ramo |
| `test_apply_with_child_remapping` | Figli rimappati correttamente |
| `test_apply_multiple_branches` | Priorità con più rami |
| `test_apply_respects_minimum` | Rispetto `min_segments` |
| `test_validate_success` | Validation OK |
| `test_validate_detects_errors` | Errori rilevati |
| `test_no_reducible_branches` | `can_apply()` → False |

## Parametri configurazione (budget_config.yaml)

```yaml
techniques:
  - id: lateral_reduce
    priority: 2
    enabled: true
    params:
      min_segments: 1
```

Priority 2 = seconda tecnica (dopo petiole_lock). Modifica geometria ma mantiene struttura.

## Esempio output

Pianta con 5 rami laterali da 5 link ciascuno:

```
Branch_r1_o0: 5 links → 1 link (-4)
Branch_r2_o0: 5 links → 1 link (-4)
...
Total lateral links saved: 20
Children remapped: 8 (foglie attaccate ai rami)
```

## Note

- **Height invariant**: la lunghezza totale del ramo rimane uguale (`height` per link × n_link = costante)
- **Child remapping**: usa `remap_link_attachment()` da Task 3 (sub-millimeter precision)
- **Iterativo**: ogni call a `apply()` riduce di una iterazione (per run nel budget loop)
