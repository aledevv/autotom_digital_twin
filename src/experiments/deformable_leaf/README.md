# Foglia deformabile isolata — D0

Prima tappa di `docs/plan.md`, Isaac Sim **4.5 / omni.physx 106.5.7**.
Il banco non importa né modifica la pianta o gli esperimenti sui frutti.
Esito delle prime prove: [D0 non ancora superato](RESULTS.md).
**D1 e D2–D2.4 non sono ancora implementati:** si procede dopo il superamento
dei controlli D0 e la revisione GUI dell'utente, come concordato.

## Esecuzione

Dalla radice del repository, con Isaac installato in `~/isaacsim`:

```bash
/usr/bin/python3 src/experiments/deformable_leaf/run.py --hz 120 --gui
```

Il launcher usa il Python di sistema solo per supervisionare il processo.
La fisica usa **`~/isaacsim/python.sh`**, CUDA e TGS. `ISAAC_PYTHON` permette
di indicare un'altra installazione compatibile. Ogni esecuzione crea una nuova
directory locale in `artifacts/deformable_leaf/`; un percorso esistente viene
rifiutato. Il log completo è `isaac.log` dentro la directory stampata come
`RUN_DIR`. Nessuna installazione di dipendenze viene effettuata dal launcher.

La GUI esegue 8 secondi con gravità e 8 senza gravità, con pacing reale se
sostenibile, poi resta aperta in pausa. Osservare la flessione della lamina,
il collegamento al morsetto e il ritorno alla forma iniziale. Chiudere la
finestra per terminare. Per ripetere, rilanciare il comando: Stop/reset durante
la misura invalida la prova. Non usare il mouse per applicare forze in D0.
La revisione visiva resta umana; il rapporto automatico non la promuove.
L'utente ha respinto la prima GUI come molto instabile (vedi RESULTS.md).

Il runner avanza un solo passo fisico con `step(render=False)` e chiama poi
`render()` senza avanzare la fisica. Controlla il contatore esatto dei passi e
registra il tempo letto da Isaac, ammettendo solo la deriva del dt float32.
`--offscreen-render` esercita la stessa cadenza di rendering senza finestra
per verificare l'equivalenza con l'headless; non sostituisce una revisione GUI.

Per il confronto headless, eseguire **in sequenza**, una GPU alla volta:

```bash
/usr/bin/python3 src/experiments/deformable_leaf/run.py --hz 60 --run-dir artifacts/deformable_leaf/d0-60
/usr/bin/python3 src/experiments/deformable_leaf/run.py --hz 120 --run-dir artifacts/deformable_leaf/d0-120
/usr/bin/python3 src/experiments/deformable_leaf/run.py --hz 240 --run-dir artifacts/deformable_leaf/d0-240
UV_CACHE_DIR=/tmp/uv-cache uv run --offline --no-sync python src/experiments/deformable_leaf/compare.py artifacts/deformable_leaf/d0-60 artifacts/deformable_leaf/d0-120 artifacts/deformable_leaf/d0-240 --output artifacts/deformable_leaf/comparison
```

Il launcher ritorna 1 se il controllo numerico fallisce, conservando tutti i
risultati. `process.json` distingue un esperimento numericamente negativo ma
completato (worker: codice 0) da eccezioni, errori PhysX e timeout.
Il confronto ammette risultati negativi completi, verifica che cambino solo
gli Hz e che il codice sia identico, e produce `comparison.json` e
`comparison.png`. La soglia esplorativa sulla differenza di sag rispetto a
240 Hz è 10%; superarla non equivale a convergenza rispetto alla mesh.

## Fixture e misure

- Metri, kg, secondi; striscia 60 × 20 × 2 mm, morsetto sui primi 6 mm.
- Griglia 30 × 10 × 2 celle; 1.023 nodi e 3.600 tetraedri conformi, con
  superficie esterna triangolata e normale uscente. Mesh di simulazione e
  collisione uguali, due celle nello spessore, self-collision disattivata.
- Attachment PhysX a un morsetto statico. Collisioni con il morsetto escluse
  per non contrastare il vincolo. D0 non dimostra attacco a supporti dinamici.
- Materiale sperimentale: E = 5 MPa, Poisson = 0,3, densità = 1.000 kg/m³,
  massa da volume × densità = 2,4 g. Questo **non** calibra una foglia biologica.
- TGS/GPU, 128 iterazioni, damping nodale 2, elasticità 0,005, damping scale 0;
  sleeping e settling automatici disattivati per osservare il recupero.
- Configurazione completa salvata in `config.json`; `--config file.json`
  consente override dei campi di `model.Config`, con `--hz` prevalente.
- `runtime.json` registra interprete, versione, configurazione della scena,
  E/Poisson letti via tensor API, topologia e hash USD. La massa riportata è
  calcolata dal volume e dalla densità, non misurata indipendentemente dal solver.
- `scene.usda` conserva la scena iniziale sotto gravità, `rest_mesh.npz` la mesh
  caricata e `trace.npz` i nodi a ogni passo, punta/centro/regione di attacco nel
  mondo e nel frame del morsetto, posa del morsetto e tempi di passo/lettura.
  Aprire soltanto l'USD non riproduce la fase senza gravità: usare il launcher.

Il reset avviene senza gravità. Il worker verifica ordine dei nodi e volumi
iniziali prima di misurare e non forza mai la posa dei nodi durante la prova.
La deformazione caricata si misura rispetto allo stato iniziale; il recupero
D0 rispetto alla forma iniziale senza gravità. In D1 il riferimento sarà invece
l'equilibrio sotto gravità precedente al contatto.

## Criteri numerici e confini

Tutte le condizioni devono passare: sequenza completa e dati finiti; nessun
tetraedro invertito; spostamenti < 2 lunghezze; deriva massima della regione
fissata < 0,2 mm; sag misurabile > 0,01 mm; escursione della punta nell'ultimo
secondo sotto gravità < 0,5 mm; errore massimo su tutti i nodi nell'ultimo secondo
di recupero < 1 mm. Le ampiezze per finestre di un secondo restano nel rapporto
per identificare oscillazioni persistenti. Le soglie sono diagnostiche, non
requisiti biomeccanici, e non vengono rilassate automaticamente dopo un fallimento.

I tempi distinguono passo fisico, copia/lettura dei nodi, rendering, rapporto fra tempo
simulato e reale, e FPS solo in GUI. Le sincronizzazioni GPU e la strumentazione
influenzano il costo: non sono tempi puri del kernel né FPS della pianta.

Il worker chiude Isaac prima di saltare il garbage collector finale di Python,
seguendo il pattern già usato nei runner sperimentali del repository: Isaac 4.5
può altrimenti andare in segmentation fault dopo aver scaricato i plugin.
Timeout e codice di uscita restano controllati dal supervisore.

## Verifica del codice

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run --offline --no-sync pytest -q src/experiments/deformable_leaf/tests
git diff --check
```

I test controllano volume, chiusura e orientamento della superficie, conformità
dei tetraedri, configurazioni invalide, sag/recupero e controlli negativi per
distacco, inversioni, NaN, assenza di deformazione e prove interrotte.
Non sostituiscono l'esecuzione PhysX o la revisione GUI.
