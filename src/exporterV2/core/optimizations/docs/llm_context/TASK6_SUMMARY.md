# Task 6 - Stem Collapse con Remapping

## Obiettivo

Ridurre il numero di segmenti del trunk principale (`n_links`) fino a un target configurabile, rimappando tutti gli attachment points dei figli (rami laterali, foglie, trusses) usando il geometry remapping di Task 3.

## Rationale

Il trunk principale può avere 10-20 segmenti. Ridurlo a 3 segmenti:
- Risparmia fino a 17 joints
- Mantiene la struttura ad albero intatta
- I rami laterali vengono rimappati alle stesse altezze assolute

## Implementazione

File: `techniques/stem_collapse.py`

### Identificazione trunk

```python
def _find_trunk(self, branches):
    # Trunk = branch senza parent
    return next((b for b in branches if b.get("parent") is None), None)
```

### Applicazione

```python
# 1. Riduci trunk a target_segments
trunk["n_links"] = self._target_segments

# 2. Per ogni figlio diretto del trunk:
for child in trunk_children:
    new_link, new_frac = remap_link_attachment(
        child["attach_link"],
        original_links,       # es. 10
        self._target_segments # es. 3
    )
    child["attach_link"] = new_link
    child["attach_frac"]  = new_frac   # frazione precisa nel segmento
```

### Remapping con attach_frac

La chiave del remapping preciso è `attach_frac`: invece di attaccare al top di un segmento, si specifica la frazione esatta all'interno del segmento target. Questo preserva l'altezza assoluta del punto di attacco.

```
Trunk originale: 10 segmenti da 10cm = 100cm totali
Branch attaccato a link 7 → altezza assoluta 70cm

Trunk collassato: 3 segmenti da 33.3cm
Altezza 70cm → link 3 (60-90cm), frac = (70-60)/30 = 0.33
```

Funzione usata: `remap_link_attachment(attach_link, n_old, n_new)` da `geometry/remapping.py`.

**Fallback**: se il modulo geometry non è disponibile, usa remapping proporzionale semplificato (`attach_frac = 1.0`).

## Test

La tecnica è testata sia direttamente che nell'integration test Task 9. I test di Task 3 (geometry remapping) coprono la funzione `remap_link_attachment` usata internamente.

## Parametri configurazione (budget_config.yaml)

```yaml
techniques:
  - id: stem_collapse
    priority: 3
    enabled: true
    params:
      target_segments: 3
```

Priority 3 = terza tecnica (alto impatto: il trunk è il componente più visibile). `target_segments` configurabile.

## Esempio output

Trunk da 10 segmenti, 5 rami laterali attaccati:

```
Trunk: 10 links → 3 links (-7 joints)
Children remapped: 5
  Branch_r1: attach_link 2 → 1 (frac 0.67)
  Branch_r2: attach_link 4 → 2 (frac 0.33)
  Branch_r3: attach_link 6 → 2 (frac 1.0)
  Branch_r4: attach_link 8 → 3 (frac 0.67)
  Branch_r5: attach_link 10 → 3 (frac 1.0)
```

## Note

- Solo i **figli diretti del trunk** vengono rimappati (i nipoti sono già relativi ai figli)
- Il remapping preserva l'**altezza assoluta** di ogni punto di attacco (non la posizione relativa)
- `target_segments=3` è il default; impostare a 1 per massima riduzione
- Validation: verifica che nessun branch sia orfano dopo il remapping
