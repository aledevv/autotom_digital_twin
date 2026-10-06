# Soglia di rottura dei giunti con TGS su GPU: un'anomalia dipendente dal numero di iterazioni

Data: 6 ottobre 2026. Software: Isaac Sim 4.5 (PhysX 5.5.1, binario). Questo documento riassume il problema, l'ipotesi, gli esperimenti e i limiti. La ricostruzione dal sorgente è un'ipotesi coerente con le misure, non una verifica sul codice del binario usato.

## 1. Il problema

Nel gemello digitale della pianta i frutti sono corpi rigidi collegati al rachide da un `FixedJoint` con `breakForce`: se la forza del vincolo supera la soglia, il giunto si rompe e il frutto si stacca. Il rachide fa parte di un'articolazione (articulation) PhysX, mentre il giunto del frutto è esterno all'articolazione (`excludeFromArticulation = 1`).

Con il solver TGS su GPU i frutti si staccavano da soli a riposo, con carichi molto inferiori alla soglia. Un frutto da 20 g (peso circa 0,196 N) rompeva un giunto con soglia nominale 6 N. Aumentando le iterazioni di posizione da 32 a 64 si rompeva anche un frutto da 10 g. Con PGS su CPU lo stesso modello era stabile. Il solver PGS/CPU è quindi diventato il riferimento della pianta, ma non era chiaro il perché.

## 2. Ipotesi

La lettura del sorgente PhysX (repository pubblico NVIDIA-Omniverse/PhysX) mostra come si decide la rottura: l'impulso accumulato dal solver sulle righe del vincolo viene confrontato con `breakForce × dt`.

- CPU (PGS e TGS) e GPU PGS usano il passo intero della simulazione: `DyConstraintSetup.cpp`, `DyTGSContactPrep.cpp`, `artiConstraintPrep2.cu` nella versione PGS.
- GPU TGS tra corpi rigidi usa il passo intero (`totalDt`): `jointConstraintBlockPrepTGS.cuh`.
- **GPU TGS per giunti collegati a un'articolazione usa `params.dt`, che in TGS vale `stepDt = dt / N`**, con N il numero di iterazioni di posizione: `artiConstraintPrep2.cu`, riga 1764 e 1766 nel tag 5.9.0, con `params.dt = sharedDesc->stepDt`.

In TGS la forza accumulata del vincolo viene azzerata solo nella fase di preparazione e si accumula sui N sotto-passi. Un carico costante F produce un impulso totale di circa F·dt, che viene confrontato con `breakForce · dt/N`. La soglia effettiva diventa `breakForce / N`.

Previsione: solo nella combinazione TGS + GPU + giunto su articolazione la soglia scala come `breakForce/N` e non dipende da `dt`. Tutti gli altri casi rompono a circa `breakForce`.

## 3. Esperimento

Script: `probe_break_threshold.py`. Fixture minimo: due link fissi (articolazione a base fissa, zero gradi di libertà), un frutto sferico appeso con un `FixedJoint` esterno a soglia 6 N, senza collisioni né input, gravità 9,81 m/s². Si cerca per bisezione geometrica la massa minima che rompe il giunto entro 2 s simulati. Precisione circa 1%. Il peso soglia è `m·g`.

Controlli: solver (TGS/PGS), backend (GPU/CPU), supporto (articolazione o corpo cinematico fuori dall'articolazione), N, soglia e frequenza fisica.

Nota sul fixture: senza `excludeFromArticulation = 1` il parser incorpora il frutto nell'articolazione e il giunto non si rompe mai (nemmeno a 100 kg, nemmeno su CPU). Il primo tentativo ha fallito per questo motivo.

## 4. Risultati (Isaac Sim 4.5)

| Configurazione | Peso soglia | Previsione |
|---|---|---|
| TGS/GPU, N = 8 | 0,755 N | 0,750 N |
| TGS/GPU, N = 16 | 0,375 N | 0,375 N |
| TGS/GPU, N = 32 | 0,189 N | 0,1875 N |
| TGS/GPU, N = 64 | 0,094 N | 0,094 N |
| TGS/GPU, N = 32, breakForce 3 N | 0,094 N | 0,094 N |
| TGS/GPU, N = 32, breakForce 12 N | 0,375 N | 0,375 N |
| TGS/GPU, N = 32, 120 Hz | 0,189 N | 0,1875 N |
| PGS/GPU, N = 32 | 5,99 N | circa 6 N |
| TGS/CPU, N = 32 e 64 | 5,99 N | circa 6 N |
| PGS/CPU, N = 32 | 5,99 N | circa 6 N |
| TGS/GPU, N = 32, supporto cinematico | 5,99 N | circa 6 N |

Dati grezzi in `results_isaac45.jsonl`.

## 5. Interpretazione

- Il comportamento è riprodotto in un fixture isolato, senza collisioni, input o altra dinamica della pianta. L'anomalia non dipende dalla geometria della pianta.
- La soglia misurata coincide con `breakForce/N` entro l'errore di bisezione, per N, soglia e frequenza diverse.
- Il difetto sparisce cambiando una sola cosa: solver, backend o tipo di supporto. Questo spiega perché la pianta usa PGS/CPU.
- Il legame con la riga di codice `params.dt = stepDt` è coerente con tutte le misure ma resta un'ipotesi. Il sorgente GPU del tag 5.5.1 (il binario usato) non è pubblico, quindi non l'ho letto.

## 6. È già noto o corretto?

- Il sorgente GPU è pubblico solo da PhysX 5.6.0 in poi. In tutti i tag controllati (5.6.0, 5.6.1, 5.7.0, 5.8.0, 5.9.0) e nel `main` consultato il 6 ottobre 2026 la riga è invariata: `linBreakImpulse = breakForce * params.dt` con `params.dt = stepDt` nel percorso TGS delle articolazioni.
- Nel tracker del repository PhysX (ricerche "breakForce", "TGS joint break", "stepDt break impulse") non ho trovato nessuna segnalazione di questo problema. Ci sono molte issue su articolazioni GPU, ma nessuna su questa soglia.
- Una ricerca web non ha trovato discussioni specifiche. Compare solo una domanda generica su un forum non legato a TGS sulla dipendenza della soglia dal numero di iterazioni.
- Conclusione prudente: non risulta noto né corretto. La mancanza di risultati non prova che non esista una segnalazione altrove. Non ho testato PhysX 5.6 o successivi in esecuzione, quindi non so se il comportamento sia identico anche lì.

## 7. Limiti e possibili azioni

- Non è stato verificato in esecuzione su Isaac Sim 6.1 o su altre versioni di PhysX.
- Non è stata verificata la forza reale riportata dal giunto: la soglia è dedotta dal punto di rottura.
- Non è stata compensata la soglia nella pianta (per esempio `breakForce × N`): resta una scelta da validare caso per caso.
- Azioni possibili: segnalare il problema al repository PhysX con questo fixture come riproduttore minimo, oppure restare su PGS/CPU, che già regge la pianta.

## Come replicare

Vedi `README.md` in questa cartella.
