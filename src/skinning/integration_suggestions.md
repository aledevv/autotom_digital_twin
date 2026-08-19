Io farei lo **Step 4 in modo molto conservativo**: niente riscrittura di ExporterV2 e niente copia-incolla dell’intera cartella `experiments`. Inseriamo il nuovo sistema come **backend per i soli rami**, lasciando invariati parser CSV, optimizer e truss.

Oggi `main.py` fa sostanzialmente:

```text
CSV
 ↓
parse_csv_to_branches()
 ↓
branches + terminal_bodies
 ↓
limit / optimization
 ↓
build_stage()
 ↓
PhysX settings
 ↓
simulation
```

Questa pipeline è già una buona separazione e non la toccherei nella parte iniziale.  Inoltre il parser distingue già logicamente lateral branches, leaf structures e trusses, quindi abbiamo un buon punto da cui separare i due sistemi.

## Piano Step 4

### 4A — Estrarre il codice validato dagli experiments

Prima sposterei il codice stabile in moduli veri di ExporterV2, per esempio:

```text
exporterV2/core/
└── skinning/
    ├── centerline.py
    ├── frames.py
    ├── radius_profile.py
    ├── branch_spec.py
    ├── branch_mesh.py
    ├── branch_physics.py
    ├── collision_proxy.py
    ├── junction.py
    ├── skeleton.py
    ├── plant_graph.py
    └── runtime.py
```

Non devono contenere codice specifico dei test.

Per esempio:

```python
generate_branch(...)
build_collision_proxy(...)
build_skinned_visual(...)
create_branch_junction(...)
sync_branch_skin(...)
```

Questo è principalmente refactoring: **non cambiamo algoritmi**.

---

### 4B — Creare un adapter `ExporterV2 branch → BranchSpec`

Questa secondo me è la parte più importante.

Non vogliamo che il nuovo sistema conosca GroIMP.

ExporterV2 continua a produrre:

```python
{
    "id": ...,
    "parent": ...,
    "n_links": ...,
    "radius": ...,
    "height": ...,
    ...
}
```

e introduciamo qualcosa tipo:

```python
branch_spec = branch_to_skinning_spec(branch_definition)
```

che converte i dati V2 nel formato usato dal nostro generatore:

```text
ExporterV2 branch
      ↓
BranchSpec
├ centerline/control points
├ radii
├ physics links
├ parent
├ attachment position
└ material parameters
```

In questo modo **non tocchiamo il parser CSV**.

---

### 4C — Separare chiaramente structural branches e truss

Non userei euristiche sparse nel codice.

Aggiungerei una classificazione esplicita, ad esempio:

```python
representation = "skinned_branch"
```

oppure:

```python
system = "vegetative"
```

contro:

```python
system = "truss"
```

Poi:

```text
branches
   │
   ├── structural vegetative
   │       ↓
   │   new PlantGraph/skinning backend
   │
   └── truss / pedicel / tomato
           ↓
       existing V2 backend
```

Il parser ha già configurazioni separate per lateral branches e trusses, quindi questo confine è coerente con l'architettura esistente.

**All'inizio limiterei il nuovo backend a main stem + lateral/recursive structural branches.**

Non cercherei subito di migrare ogni petiole, rachis fogliare ecc.

---

### 4D — Nuovo builder ibrido

Qui modifichiamo davvero `build_stage()`.

Oggi `build_stage()` costruisce tutte le chain con lo stesso sistema di rigid segment.

Lo trasformerei concettualmente in:

```python
def build_stage(...):

    structural, legacy = partition_components(branches)

    # NEW
    plant_graph = build_plant_graph(structural)
    build_skinned_branch_system(stage, plant_graph)

    # EXISTING
    build_existing_components(stage, legacy)

    # existing terminal/truss handling
    ...
```

Quindi nello stesso USD avremo:

```text
/World

├── PlantPhysics
│   └── new branch articulation
│
├── PlantVisual
│   └── SkelRoots / skinned branch meshes
│
├── Truss...
│   └── existing representation
│
└── TerminalBodies
    └── tomatoes...
```

La truss non deve sapere nulla dello skinning.

---

## 4E — Qui c'è il cambiamento runtime fondamentale

Questo è il punto che **non dobbiamo dimenticare**.

Il `main.py` attuale esegue:

```python
while simulation_app.is_running():
    my_world.step(render=True)
```

Questo non basta per il nuovo sistema, perché noi dobbiamo fare il bridge:

```text
PhysX step
   ↓
read rigid link transforms
   ↓
PhysX world → bone local
   ↓
write SkelAnimation
   ↓
render
```

Quindi estrarrei dagli experiment un oggetto tipo:

```python
skinning_runtime = SkinningRuntime(stage, plant_graph)
```

e il main diventerebbe più o meno:

```python
while simulation_app.is_running():

    my_world.step(render=False)

    skinning_runtime.sync()

    simulation_app.update()
```

Esattamente la pipeline che abbiamo già validato.

**Non metterei questa logica dentro `main.py`**. Il main dovrebbe soltanto chiamare:

```python
runtime.sync()
```

---

# 4F — Non rompere l'optimizer

Questo merita attenzione.

Attualmente `main.py` fa ancora prima:

```text
limit_branch_resolution()
BudgetOptimizer
count_d6_joints()
```

La cosa bella è che possiamo conservarlo **se facciamo coincidere**:

```text
V2 n_links
       =
new physics_links
```

almeno nella prima integrazione.

Quindi:

```python
BranchSpec.physics_links = branch["n_links"]
```

Così l'optimizer continua ad avere significato.

La risoluzione visuale invece resta indipendente:

```text
n_links = 4

physics:
●────●────●────●

visual:
|||||||||||||||||||||||||||||
```

Questa è proprio una delle caratteristiche migliori che abbiamo validato.

Più avanti potremo separare anche:

```text
branch["physics_links"]
branch["visual_samples"]
```

ma **non nello Step 4 iniziale**.

---

# 4G — Primo integration test: NON usare subito una pianta enorme

Farei:

### `4A integration smoke test`

Input manuale V2:

```text
trunk
├── lateral_01
└── lateral_02
```

Passa attraverso il **vero `build_stage()` di ExporterV2**, non attraverso gli experiments.

Verificare:

```text
✓ USD generato
✓ PlantGraph corretto
✓ mesh skinnate
✓ capsule invisibili
✓ D6
✓ junction
✓ runtime sync
✓ Shift+click
```

Se passa:

### `4B CSV integration`

```bash
./run_mainV2.sh --day <giorno semplice>
```

Con CSV vero.

Verifichiamo che:

```text
GroIMP
  ↓
existing parser
  ↓
existing branch dictionaries
  ↓
NEW adapter
  ↓
PlantGraph
  ↓
new skinned branches
```

---

# 4H — Poi test ibrido con truss

Solo dopo:

```text
structural branches → NEW
truss              → OLD
tomatoes           → OLD
```

La domanda del test non è se la truss è bella.

È soltanto:

> l'introduzione del nuovo backend per i rami ha lasciato completamente intatto il sottosistema truss?

Il V2 ha già fisica e configurazioni particolari per truss e detachable tomatoes, quindi questa separazione è importante.

---

## La sequenza che userei

```text
4A — Extract validated modules
     experiments → exporterV2/core/skinning
                              ↓
                         no behavior change

4B — BranchSpec Adapter
     V2 branch dict → BranchSpec

4C — Hybrid builder
     structural branches → new backend
     truss              → existing backend

4D — Skinning runtime
     PhysX → SkelAnimation in real main loop

4E — Static V2 smoke test
     trunk + 2 laterals

4F — Real CSV test
     --day N

4G — Hybrid plant
     skinned branches + existing truss

4H — Optimization compatibility
     --optimize

4I — Regression
     compare old V2 vs new V2
```

### Una cosa che eviterei

Non farei:

```text
experiments/
   ↓ COPY EVERYTHING
stage.py
```

perché `stage.py` è già molto grosso e gestisce attachment, terminal bodies e diversi tipi di chain.

Piuttosto gli farei delegare:

```python
build_skinned_vegetative_structure(...)
```

Così ExporterV2 diventa orchestration, mentre il nuovo sistema rimane modulare.

### Obiettivo finale dello Step 4

Idealmente `main.py` cambia pochissimo:

```text
CSV parsing            SAME
optimization           SAME
truss generation       SAME
tomato system          SAME

branch generation      NEW
runtime loop           + skin sync
```

Questa per me è la strategia meno rischiosa: **integriamo ciò che abbiamo validato senza rifare ExporterV2 attorno allo skinning**.
