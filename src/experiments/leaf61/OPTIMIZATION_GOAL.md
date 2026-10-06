# Ottimizzazione del banco, 17 settembre 2026

Risultati del goal esteso a due ore. Lamine skinnate con tre segmenti e giunti elastici,
non Surface Deformable. Gli esperimenti precedenti e i loro launcher restano disponibili.

## Protocollo e limiti

Isaac Sim 6.1, CPU/PGS, 480 Hz fisici, rendering offscreen 1280×720 a 30 Hz
simulati, senza pacing. Finestra prestazionale t=9–17 s, dopo assestamento.
Pomodoro 20 g, raggio 20 mm, caduta libera da 60 mm sulla prima foglia.
Le altre foglie non ricevono contatti. Tutte hanno corpi e giunti dinamici;
"awake" indica sleeping disabilitato, non movimento simultaneo imposto.
Soglia: media >20 FPS e p95 lavoro/frame <50 ms, oltre ai controlli fisici.
RTF <1 indica rallentamento del tempo simulato anche quando gli FPS passano.

La pianta di origine ha 131 lamine visive: 120 posizioni non sovrapposte ospitano
il candidato standard seed42 di 90 mm, orizzontale e con yaw nativo. Non sono
le 131 forme originali. Fusto e rami sono fissi e visivi; i contatti tra foglie
sono esclusi nel test di capacità. Oltre 120 si replicano le piante (2–4 copie),
con camera che include tutte le copie: non è un confronto a camera identica
con la singola pianta, né una prova di vegetazione tutta in movimento.

## Strategie

- Letture delle pose in batch e conversione vettoriale: tracce fisiche a 30
  foglie identiche bit per bit al banco precedente.
- Aggiornamento dei soli scheletri visivi e di una copia visiva del pomodoro:
  niente sincronizzazione dei corpi nascosti in USD; la palla fisica rimane
  libera. Verifica indipendente della posizione visibile durante l'urto.
- Scrittura Vt da NumPy e salto degli aggiornamenti identici.
- 32 iterazioni del solver: cascata di cinque foglie verificata. 16 iterazioni
  scartate per penetrazione di 1,023 mm. Frequenza di 240 Hz scartata perché
  fallisce sia la penetrazione PhysX sia quella geometrica.
- Tolleranza visiva opzionale di 1 micrometro, con limite conservativo calcolato
  rispetto all'ultima posa realmente disegnata, senza deriva cumulativa.
- Sleeping opzionale 0,00005: verificato addormentamento e risveglio all'urto;
  anche la cascata di cinque foglie passa tutti i controlli.
- Collision groups per evitare relazioni quadratiche tra foglie.
- Mesh/scheletri accorpati: geometria verificata contro UsdSkel, ma rendering
  peggiore; strategia non promossa e disabilitata per default.

## Misure raccolte

Cartelle sotto `artifacts/leaf61/`; ogni cartella conserva sorgenti, configurazione,
scena, versioni, log, trace.npz, report.json e tempi. Le misure a 120 e più foglie
sono singole esecuzioni, non una conferma in tre ripetizioni.

| Cartella | Foglie | FPS | p95 ms | RTF | Esito prestazionale |
|---|---:|---:|---:|---:|---|
| goal-batched-30 | 30 | 32,48 | 35,93 | 1,083 | passa |
| goal-visual-30 | 30 | 35,55 | 31,44 | 1,185 | passa |
| goal-visual-120-iter32 | 120 | 24,75 | 45,82 | 0,825 | passa, sleeping disabilitato |
| goal-visual-240-iter32 | 240 | 15,60 | 71,81 | 0,520 | fallisce |
| goal-visual-240-tol | 240 | 21,48 | 51,34 | 0,716 | fallisce p95 |
| goal-360-sleep | 360 | 25,55 | 47,44 | 0,852 | passa FPS, sleeping abilitato |
| goal-480-sleep | 480 | 21,47 | 55,08 | 0,716 | fallisce p95 |

Tutte queste prove superano i controlli numerici. I casi `*-scout` usano
campionamento geometrico ridotto e non costituiscono la validazione finale.
Massimo provato che supera il criterio FPS: **360 foglie** in una singola
esecuzione. Nel primo secondo dopo il rilascio p95 45,44 ms e massimo 49,10 ms.
Tutte dormono prima del rilascio; la foglia colpita si risveglia.

Attenzione ai campi storici del report: `target_20fps_passed` richiede anche
RTF >=1, quindi risulta false a 120 e 360. Il criterio FPS della tabella è
esplicitamente media >20 e p95 <50 ms, separato dal tempo reale. Nessuno dei
risultati con RTF <1 è promosso come simulazione in tempo reale. Il campo
`performance_passed` mantiene anch'esso la richiesta originale di tempo reale.
La massima numerosità di questa ricerca con RTF >=1 effettivamente provata
è 30; non è stato cercato il limite intermedio fra 30 e 120.

## Riproduzione

```bash
./run_leaf61_capacity_optimized.sh --run-dir artifacts/leaf61/repeat-120
# GUI finita: termina automaticamente dopo i 17 secondi simulati della prova.
./run_leaf61_capacity_optimized.sh --gui --gui-benchmark
# Variante sleeping e copie, da misurare con le stesse soglie:
./run_leaf61_capacity_optimized.sh --leaves 360 --plant-copies 3 \
  --collision-filter groups --sleep-threshold 0.00005 --visual-tolerance 0.000001
```

Il default riproduce il candidato 120 senza sleeping. La modalità GUI libera
(`--gui` senza `--gui-benchmark`) conserva tutte le pose fino alla chiusura:
non lasciarla aperta indefinitamente con centinaia di foglie. La revisione GUI
resta distinta dalle misure offscreen e non è ancora confermata per i nuovi
parametri. `--stress-all` aggiunge un impulso gravitazionale globale a t=10–11 s
con risveglio delle articolazioni e otto secondi di recupero: opzione sperimentale,
non una prova già superata finché manca il relativo report.

## Verifica geometrica a frequenza piena

`--geometry-method bounded` conserva ogni posa fisica a 480 Hz. Per foglie
non colpite confronta tutte le trasformazioni con una foglia di riferimento
ricostruita esattamente. Il limite per vertice è il massimo sui giunti di
`norm(delta_t) + norm(delta_R, Frobenius) * r_max`; pesi LBS non negativi e
normalizzati consentono di usarlo come limite dell'errore superficiale.
Il limite propaga conservativamente a deriva, allungamento, area, sag,
recupero e oscillazione. Se non certifica le soglie, viene ricostruita la
superficie completa. I potenziali contatti visivi sono controllati con
ricostruzioni delle superfici vicine; distanze dai collider a ogni passo.
Non si saltano passi per dichiarare la validazione full. Test confrontano
questi limiti con ricostruzioni esplicite perturbate.

Il riuso `--analysis-reference` richiede uguaglianza esatta delle tracce,
contatti, geometria e configurazione rilevante: riusa solo analisi geometrica,
mai i tempi. 20 test unitari passano; Ruff e git diff --check superati.


## Consegna finale dopo l'estensione a due ore

**Candidato pratico per la pianta: 127 lamine**, CPU/PGS 32 iterazioni, 480 Hz,
sleeping 0,00005, errore visivo limitato a 1 micrometro e scritture Sdf raggruppate.
`goal-127-primary-sdf`: 37,17 FPS offscreen senza pacing, p95 34,20 ms, RTF
1,239, controlli numerici superati. Tutte le 127 rispondono all'impulso globale
2g fra t=10 e t=11 s e recuperano dopo otto secondi. Non è una prova di urti
simultanei su tutte le lamine. Il p95 è sopra 33,33 ms: supera la soglia 20 FPS,
ma non la più severa cadenza costante di 30 Hz. Una sola esecuzione completa.

La selezione `primary` dà precedenza ai rami principali e usa un controllo SAT
su inviluppi convessi, sempre con 1 mm di separazione, al posto delle AABB.
Include tutte le 51 posizioni dei rami principali. Ammette anche le lamine
basse, verificando un margine dal suolo superiore alla massima flessione
ammessa sotto gravità. Quattro posizioni laterali restano visive e statiche.
Forme seed42 orizzontali e yaw nativo: **non sono ancora le forme curve e le
inclinazioni originali**. L'audit di copertura è in
`artifacts/leaf61/goal-native-coverage-primary.json`.

```bash
# GUI persistente; pomodoro in caduta libera, Ripeti caduta / Termina e salva.
./run_leaf61_day160.sh
# Stessa GUI con prova di risposta simultanea alla gravità 2g, una volta a t=10–11.
./run_leaf61_day160.sh --stress-all
# Riproduzione offscreen della prova numerica e prestazionale da 127:
./run_leaf61_capacity_optimized.sh --leaves 127 --canopy-selection primary \
  --collision-filter groups --sleep-threshold 0.00005 --visual-tolerance 0.000001 \
  --skin-writes sdf --stress-all --run-dir artifacts/leaf61/repeat-primary127
```

La GUI registra solo la prima finestra; continua poi a simulare e consente
altre cadute senza accumulare pose indefinitamente. `goal-480-r60` verifica
questa modalità: 8161 campioni fino a t=17 s, simulazione continuata fino a
20 s senza nuovi campioni, clock e metriche corretti. Se si interrompe o si
ripete la prima prova prima del suo completamento, il report può fallire;
non si sostituisce automaticamente con una prova successiva.

**Massimo sopra 20 FPS effettivamente provato: 480**, quattro copie della
pianta con sleeping, rendering 60 Hz simulati: 34,45 FPS, p95 34,24 ms,
RTF 0,574 (`goal-480-r60`). Tutti i controlli numerici passano; una foglia
colpita, altre a riposo. Questo risultato comporta rallentamento fisico e
non dimostra 480 foglie tutte in movimento. I risultati a rendering 30 e
60 Hz sono configurazioni distinte; non confondere FPS e velocità simulata.

Ulteriori prove:

- `goal-120-stress`: tutte sollecitate, sleeping disabilitato, 23,84 FPS,
  p95 49,09 ms, RTF 0,795; numerica superata.
- `goal-120-sleep-stress`: tutte sollecitate con sleeping, 38,05 FPS,
  p95 32,84 ms, RTF 1,268; numerica superata.
- `goal-240-threads4`: quattro thread PhysX peggiorano il risultato; non adottati.
- `goal-cascade5-tgs8`: TGS 8 iterazioni supera la cascata. Il successivo
  `goal-240-tgs8-stress` misura 27,65 FPS / p95 59,05 ms, ma scade il timeout
  durante l'analisi geometrica offline: **verifica incompleta**, non promosso.
  Le tracce restano salvate. Il candidato consegnato mantiene PGS.
- Mesh accorpate, 240 Hz fisici e PGS 16 iterazioni restano scartati.

La geometria offline ora usa anche un limite dell'errore del vettore spigolo
basato sulle differenze dei coefficienti LBS, evitando il limite troppo largo
`2 * errore_vertice / spigolo_minimo`. Si propagano conservativamente questi
limiti alle aree. I test li confrontano con superfici ricostruite esplicitamente.
Le scritture Sdf avvengono in un ChangeBlock senza interrogazioni USD al suo
interno; la verifica indipendente UsdSkel di foglie campione a quattro istanti
confronta la superficie realmente autorializzata con quella ricostruita dalle
pose fisiche. Nessuna soglia fisica è stata allentata.

**Limiti di conferma:** mancano le tre ripetizioni sui nuovi candidati e la
revisione visiva GUI. `repeat_goal_run.py` prepara la conferma riproducendo
configurazione e input: riutilizza le metriche solo dopo uguaglianza bit per
bit di tutte le tracce. Non è stato eseguito nella campagna conclusa.
`goal_report.py artifacts/leaf61` rigenera `goal-summary.json` distinguendo
criterio FPS, tempo reale e cadenza stretta. Le massime riportate sono singole
prove, senza estrapolazione.

Riferimenti consultati: [thread CPU e prestazioni Isaac](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/reference_material/sim_performance_optimization_handbook.html),
[output della simulazione e solver PhysX](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.3/dev_guide/simulation_control/simulation_control.html).
La configurazione è verificata contro il codice locale 6.1; il manuale online
ricercato per i thread rimanda alla pagina 6.0.
