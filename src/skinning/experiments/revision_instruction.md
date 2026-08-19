Certo. Ti preparo un testo già pensato da incollare a un LLM/code agent per fare **review + semplificazione**, spiegando contesto, comportamento validato e cosa non deve rompere.

---

## Contesto della revisione

Nel branch `feature/realistic-skinning` di `autotom_digital_twin`, dentro `src/exporterV2/core/skinning/`, abbiamo recentemente aggiunto una modalità visuale alternativa allo skinning per mantenere l'aspetto organico dei rami ma recuperare le performance.

La motivazione è stata verificata sperimentalmente su una pianta day 40:

```text
legacy / cilindri            ~18–20 FPS
smooth mesh statica          ~19–20 FPS
full UsdSkel skinning         ~8 FPS
global/shared Skeleton       ~10 FPS
segmented organic mesh       ~20 FPS
```

Quindi il costo principale era UsdSkel, non la geometria organica.

La soluzione realtime attuale è `segmented`: ogni rigid link PhysX mantiene la stessa fisica/collisione precedente, ma invece di mostrare un cilindro viene visualizzato tramite un pezzo della smooth organic mesh. Ogni pezzo è parentato direttamente al proprio rigid body, quindi non esistono Skeleton, SkelAnimation o runtime sync.

**Importante: la modalità `skinned` originale deve assolutamente rimanere disponibile.** È una feature valida e serve come modalità high-quality/demo. La review può ridurre complessità, duplicazione e codice diagnostico, ma non deve eliminare il backend skinned né cambiare il comportamento fisico validato.

### File principali coinvolti

* `src/exporterV2/core/skinning/visual_modes.py`: contiene le modalità visuali non-UsdSkel. Qui è stata implementata la modalità `segmented`, cioè una organic mesh rigida per ogni PhysX link. I segmenti interni hanno un piccolo overlap/tongue per nascondere meglio le separazioni durante la flessione. Successivamente sono state aggiunte alcune logiche specifiche per `segmented-fork`: overlap visuale della root del petiole centrato, chiusura/dome del terminale del lateral branch e taper terminale opzionale. Questa è probabilmente una delle aree che può essere semplificata, perché si sono accumulate diverse funzioni helper durante le iterazioni estetiche.

* `src/exporterV2/core/skinning/terminal_fork.py`: contiene il visual dressing delle terminazioni dei lateral branches. La logica finale deve essere **leaf-only**: la truss/tomato non deve essere modificata. Quando un lateral branch termina con un vero `petiole`, il petiole reale è considerato la continuazione principale del ramo; viene aggiunto soltanto un piccolo fake young twig statico con una piccola foglia. Il fake twig non ha fisica, rigid body, collider, joint o UsdSkel: è semplicemente una mesh figlia dell'ultimo rigid link del lateral branch. Il twig è corto e sottile apposta, così il comportamento rigido è visivamente plausibile. Recentemente è stata aggiunta una variazione pseudo-random dell'azimuth attorno all'asse del lateral branch usando un hash deterministico di `parent_id + child_id`; quindi i fake twig non sono tutti nello stesso piano, ma una stessa pianta rigenerata mantiene sempre la stessa disposizione. Attualmente la variazione è circa ±75°. Sarebbe utile verificare se questo file può essere ridotto o se alcune utility geometriche duplicate possono essere condivise con `visual_modes.py`.

* `src/exporterV2/core/skinning/builder.py`: orchestra le modalità `skinned`, `segmented`, `segmented-fork`, `static`, `global`, `rigid-single`. Per `segmented-fork` identifica i petiole terminali dei lateral branches e marca il petiole come `_terminal_fork_centered` e il parent come `_terminal_fork_centered_host`. Inoltre calcola il taper terminale del lateral branch in base al **raggio visuale effettivo** del petiole, non semplicemente al radius raw del CSV. Questo è stato necessario perché il petiole ha un visual root flare e quindi `_visual_radius(child_axis, 0.0)` può essere diverso da `child["radius"]`. C'è una funzione `_compute_centered_fork_tip_scales(...)` che usa quindi:

  ```python
  parent_radius = _visual_radius(parent_axis, parent_axis.total_length)
  child_contact_radius = _visual_radius(child_axis, 0.0)
  scale = child_contact_radius / parent_radius
  ```

  con clamp difensivo. Questa parte è ancora un compromesso estetico e non è perfetta, quindi la review può proporre una forma più semplice purché non rompa il risultato attuale.

* `src/exporterV2/core/skinning/adapter.py`: per i petiole terminali marcati `_terminal_fork_centered`, il punto di attachment viene spostato sulla centerline del parent invece del vecchio offset radiale. Questo è importante perché il leaf branch reale deve sembrare la continuazione del lateral branch, non un tubo attaccato lateralmente con una specie di bridge. La modifica deve restare limitata alla modalità/flag specifica e non cambiare il comportamento degli altri branch.

* `src/exporterV2/core/skinning/global_visual.py`: è stato aggiunto come esperimento diagnostico per verificare se il problema di performance fosse il numero di Skeleton oppure UsdSkel stesso. Crea un unico shared Skeleton/SkelAnimation per tutti gli assi. Il test ha prodotto circa 10 FPS contro 8 FPS del full skinning, quindi non ha risolto il problema. Questo file può essere considerato diagnostico e potenzialmente separato/pulito, ma non va confuso con la modalità skinned principale.

* `src/exporterV2/main.py` e `run_mainV2.sh`: sono state aggiunte le opzioni per selezionare `--skinning-visual-mode`, incluse `skinned`, `segmented`, `segmented-fork` e alcune modalità diagnostiche. La modalità realtime che interessa ora è:

  ```bash
  ./run_mainV2.sh \
    --day 40 \
    --branch-backend skinned \
    --skinning-visual-mode segmented-fork
  ```

  Nonostante il nome `branch-backend skinned`, `segmented-fork` non usa UsdSkel per le organic meshes.

* `src/skinning/experiments/test_4a_terminal_visual_fork/`: test isolato usato per sviluppare la geometria visuale della biforcazione prima dell'integrazione nell'exporter. Contiene una versione statica con parent + real-organ mock + fake young shoot. È materiale sperimentale e non deve necessariamente influenzare l'architettura finale.

### Comportamento finale che deve essere preservato

La modalità `segmented-fork` deve restare intorno ai ~20 FPS sulla pianta day 40 e non deve introdurre UsdSkel/runtime sync.

La fisica deve rimanere identica: capsule collider, rigid links, D6/fixed joints ecc. non devono essere sostituiti dalla visual mesh. Le organic meshes sono solo rendering.

Il lateral branch terminale deve avere il vero leaf branch/petiole centrato sulla propria centerline e visivamente deve sembrare una continuazione del lateral branch. Un piccolo fake young twig può uscire dal nodo come ramo secondario. Questo twig deve essere corto, sottile, rigido e avere una piccola foglia.

La posizione angolare del fake twig deve variare tra lateral branches. Attualmente è pseudo-random ma deterministica: stessa struttura → stesso risultato a ogni export.

La truss/pomodoro deve rimanere completamente fuori dalla terminal-fork logic. In precedenza era stato provato un mock visuale anche sulla truss, ma il risultato non era convincente ed è stato rimosso.

La modalità:

```text
skinned
```

deve continuare a funzionare esattamente come modalità high-quality con deformazione continua.

La modalità:

```text
segmented
```

deve continuare a funzionare senza terminal fork dressing.

`segmented-fork` deve essere sostanzialmente:

```text
segmented
+
centered terminal leaf branch
+
small visual young twig
+
visual junction dressing
```

### Obiettivo della review

Vorrei una review focalizzata soprattutto sulla **riduzione della complessità accidentale introdotta dalle iterazioni**. In particolare, verifica se:

```text
visual_modes.py
terminal_fork.py
builder.py
adapter.py
```

hanno responsabilità troppo intrecciate, helper geometrici duplicati, marker temporanei nei branch dict (`_terminal_fork_centered`, `_terminal_fork_centered_host`, ecc.) che possono essere modellati meglio, o logiche diagnostiche che possono essere separate dal percorso production.

È accettabile proporre refactoring o piccoli dataclass/config dedicati, ma evita over-engineering. L'obiettivo è avere una pipeline facile da spiegare:

```text
PlantGraph / branch definitions
        ↓
resolve physics
        ↓
build visual axes
        ↓
segmented organic mesh per rigid link
        ↓
optional terminal leaf-fork visual dressing
```

Non modificare formule o parametri fisici già validati a meno che non siano strettamente necessari al refactoring.

Prima di modificare il codice, indicami:

1. quali parti consideri accidentalmente complesse;
2. quali parti sono effettivamente necessarie;
3. quali file/funzioni vorresti semplificare;
4. quali invarianti/test useresti per essere sicuro di non rompere né la modalità `skinned` né i ~20 FPS della modalità `segmented`.

Poi proponi una refactor incrementale, non una riscrittura completa.
