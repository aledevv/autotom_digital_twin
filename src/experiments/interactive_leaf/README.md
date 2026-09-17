# Foglia interattiva isolata — Isaac Sim 4.5

Prototipo alternativo al D0 deformabile, realizzato dopo la sua bocciatura visiva.
Una lamina continua segue **cinque corpi rigidi con giunti elastici D6**, collegati
a un picciolo dinamico. Il movimento nasce da gravità e contatti PhysX; solo
lo strumento arancione è cinematico. La base dell'articolazione è fissata al mondo.
La fisica usa PGS/CPU; la GPU serve al rendering. Nessuna integrazione nella pianta.

## Avvio interattivo

Dalla radice del repository:

```bash
./run_interactive_leaf.sh
```

Attendere l'assestamento iniziale e usare la finestra **Interactive leaf**:

- **Press leaf / Release**: abbassa lentamente la sfera arancione e la ritira.
- **Drop ball**: lascia cadere una pallina libera da 1 g, 5 cm sopra la foglia.
- **Remove ball**: riporta la pallina sul tavolo.
- **Run automatic contact demo**: esegue pressione, rilascio, caduta e recupero.
- **End session**: salva le misure e chiude Isaac.

La simulazione parte già in Play. La demo dura 20 secondi simulati. Gli oggetti
sono riutilizzati; premere Drop ball riposiziona la stessa pallina. Il pulsante
Stop della timeline conclude la sessione. Non è necessario trascinare i link.

## Modello e regolazione

Lamina di 80 × 36 mm, massa totale 0,6 g, cinque collider sottili da 0,8 mm;
picciolo di 22 mm e 1,5 g. Ogni giunto ammette flessione e torsione, blocca
traslazioni e rotazione laterale e ha limiti angolari. La mesh ha 441 vertici e
768 triangoli, con pesi UsdSkel continui fra segmenti. Le lunghezze dello scheletro
restano vincolate: la lamina si piega senza allungarsi come un elastico.

Configurazione iniziale: 120 Hz, rigidezza di flessione 0,024 N m/rad, rigidezza
del supporto 0,08 N m/rad, smorzamento relativo 0,9. La conversione radiante/grado
avviene solo durante l'authoring USD dei drive.

Per una variante più rigida o per pressione fuori centro:

```bash
./run_interactive_leaf.sh --config src/experiments/interactive_leaf/config_stiffer.json
./run_interactive_leaf.sh --config src/experiments/interactive_leaf/config_offcenter.json
```

Un JSON può sovrascrivere i campi di `geometry.Config`. Le dimensioni sono in metri,
le masse in kg; la rigidezza è in N m/rad. I parametri non derivano da calibrazione
biomeccanica. La forma è sintetica. I collider a segmenti approssimano la superficie
visiva e possono produrre piccoli disallineamenti durante il contatto. Non è un
modello FEM e non dimostra il superamento di D0–D2.4 della traccia deformabile.

## Verifica riproducibile

```bash
/usr/bin/python3 src/experiments/interactive_leaf/run.py --duration 60
/usr/bin/python3 src/experiments/interactive_leaf/run.py --render
/usr/bin/python3 src/experiments/interactive_leaf/run.py --gui-test --render
UV_CACHE_DIR=/tmp/uv-cache uv run --offline --no-sync pytest -q src/experiments/interactive_leaf/tests
```

`--gui-test` apre una sessione limitata e invoca gli stessi callback dei pulsanti;
non simula click del mouse. `--scenario rest|press|drop|cycle` seleziona il caso
automatico. Ogni processo riparte da una scena nuova. `--hz 60|120|240` cambia il
passo, ma il riferimento operativo è 120 Hz. `ISAAC_PYTHON` permette di specificare
un'installazione diversa da `~/isaacsim/python.sh`.

Gli output rimangono in `artifacts/interactive_leaf/<run>/`: configurazione completa,
hash dei sorgenti, scena iniziale, log, report, runtime e `trace.npz`. Quest'ultimo
contiene pose di tutti i corpi, punta nel mondo e nel riferimento del picciolo,
errore dei giunti, posa reale della sfera e forze di contatto sui corpi **rigidi**.
Le forze non sono report dei deformabili. La mesh si ricostruisce dalle pose con
`geometry.skin`; l'animazione della scena richiede il launcher Python.

L'accettazione controlla contatto effettivo, flessione locale, recupero entro
1,5 mm, moto residuo sotto 0,2 mm, vincoli entro 0,3 mm, valori finiti e freccia
sotto gravità inferiore al 10% della lunghezza. Nei test ripetuti ogni ciclo deve
passare. Il tempo per passo, la velocità della simulazione e gli FPS GUI sono
registrati separatamente. Il rendering non avanza il clock fisico.

Risultati misurati e limiti: [RESULTS.md](RESULTS.md).
