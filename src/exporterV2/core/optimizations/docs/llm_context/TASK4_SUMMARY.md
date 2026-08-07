# Task 4 - Petiole Lock (D6 → Fixed Joint)

## Obiettivo

Convertire i joints dei petioluli da **D6** (articolati, 6 DOF) a **Fixed** (statici, 0 DOF), riducendo il carico computazionale senza alterare geometria o aspetto visivo.

## Rationale

I petioluli (gambi delle foglioline) hanno movimento trascurabile in pratica. Convertirli a fixed joints:
- Elimina 6 DOF per petiolulo dal budget PhysX
- Non cambia la posizione/geometria nella scena
- Impatto visivo nullo (il petiolulo rimane dov'è, smette solo di oscillare)

## Implementazione

File: `techniques/petiole_lock.py`

### Identificazione petioluli

```python
def _is_petiolule(self, branch):
    # Pattern naming: "Petiolule_*"
    if branch["id"].startswith("Petiolule_"):
        return True
    # Fallback: branch con parent rachis e n_links <= 2
    if branch.get("parent", "").startswith("Rachis_") and branch.get("n_links", 1) <= 2:
        return True
    return False
```

### Applicazione

Il metodo `apply()` aggiunge metadata `joint_type: "fixed"` a ogni petiolulo trovato:

```python
branch_copy["joint_type"] = "fixed"
```

Nessuna modifica geometrica. Il builder USD (`stage.py`) legge questo metadata e crea `UsdPhysics.FixedJoint` invece di `UsdPhysics.D6Joint`.

### Conteggio joints (bug fix)

**Importante**: dopo l'applicazione di questa tecnica, i petioluli Fixed NON contano più nel budget. La funzione `count_d6_joints()` esclude i branch con `joint_type='fixed'`.

```python
# In base.py
def count_d6_joints(branches):
    return sum(b.get("n_links", 1) for b in branches
               if b.get("joint_type", "d6").lower() != "fixed")
```

## Test

File: `tests/4_petiole_lock/test_petiole_lock.py` — **9/9 test passati**

| Test | Cosa verifica |
|------|---------------|
| `test_identify_petiolules` | Pattern naming corretto |
| `test_can_apply` | Presenza petioluli D6 |
| `test_estimate_reduction` | Stima DOF riduzione (×6 per petiolulo) |
| `test_apply_locks_petiolules` | Metadata `joint_type: fixed` aggiunto |
| `test_apply_preserves_geometry` | `n_links`, `height`, `radius` invariati |
| `test_validate_topology` | Parent-child invariati |
| `test_already_fixed` | Idempotente (non blocca due volte) |
| `test_no_petiolules` | `can_apply()` → False su piante senza petioluli |
| `test_validate` | Validation geometrica OK |

## Parametri configurazione (budget_config.yaml)

```yaml
techniques:
  - id: petiole_lock
    priority: 1
    enabled: true
    params: {}
```

Priority 1 = prima tecnica applicata (impatto visivo minimo).

## Esempio output

Pianta sintetica con 60 petioluli:

```
Joints before: 195 (includes 60 petiolule D6 joints)
After petiole_lock: 135 D6 joints (60 petiolule → Fixed, non contano)
DOF reduced: 360 (60 × 6 DOF)
Geometry: unchanged
```

## Note

- **Reversibile**: rimuovere il metadata `joint_type: fixed` ripristina i D6
- **Backward compatible**: branch senza `joint_type` trattati come D6 (default)
- `joints_saved` nel report è 0 perché i joints fisici non vengono rimossi, solo convertiti — il risparmio emerge nel `count_d6_joints()`
