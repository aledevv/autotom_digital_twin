# D0 — risultato del 16 settembre 2026

**Banco implementato; criterio D0 non superato. Prima GUI respinta dall'utente
come "decisamente molto instabile".**
Nessun test D1/D2 e nessuna modifica alla fisica della pianta.

## Confronto controllato

Tre partenze indipendenti, stessa implementazione e configurazione salvo Hz:
striscia 60 × 20 × 2 mm, 3.600 tetraedri, E = 5 MPa, densità 1.000 kg/m³,
TGS/GPU, 128 iterazioni. Per ogni caso: 8 s di gravità e 8 s di recupero senza
gravità. Le misure sono numeriche, non biomeccaniche.

| Hz | Sag punta [mm] | Deriva massima attacco [mm] | Escursione finale punta sotto gravità [mm] | Errore recupero [mm] | Volume minimo / iniziale |
|---:|---:|---:|---:|---:|---:|
| 60 | 59,792 | 0,508 | 1,560 | 1,715 | 0,074 |
| 120 | 56,653 | 0,316 | 0,687 | 0,637 | 0,319 |
| 240 | 54,720 | 0,158 | 0,219 | 4,731 | 0,594 |

L'attacco richiede deriva < 0,2 mm, l'assestamento escursione < 0,5 mm e il
recupero errore < 1 mm. Nessuna frequenza supera tutte le soglie.
Il recupero misura il massimo scostamento di qualunque nodo nell'ultimo secondo
della fase senza gravità, non il solo scostamento finale della punta.

I dati sono finiti e i tetraedri restano positivi in queste tre prove. Questo
non rende piccole le distorsioni: il volume minimo relativo a 60 Hz è circa 0,074.
La sag varia di circa 9,27% fra 60 e 240 Hz e passa lo screening esplorativo del
10%, ma il recupero e la qualità della deformazione impediscono di promuovere
il modello. Il tempo strumentato per 16 s simulati è circa 9,76 / 19,84 / 39,16 s
rispettivamente; non sono FPS GUI né prestazioni della pianta.

E e Poisson letti dal solver coincidono con il materiale richiesto. Il morsetto
seleziona 132 punti di attachment nella mesh di riferimento. Non è stata
identificata una causa unica del comportamento: non attribuiamo il risultato
a un bug PhysX né consideriamo validata la risposta del materiale.

## Diagnosi preliminare conservata

- Prima prova: corretto un attributo di trasformazione mancante al reset.
- A 120 Hz e 32 iterazioni si osservavano inversioni (volume relativo minimo
  circa −0,217). Aumentare a 128 le elimina nel caso provato, senza risolvere
  tutti gli altri controlli.
- Due controlli con griglia 12 × 4 × 2, a 120/240 Hz, non hanno risolto tutti i
  criteri. Questa griglia cambia anche l'ultimo piano di nodi nel morsetto
  (5 mm invece di 6 mm): è una diagnosi esplorativa, non uno studio pulito di
  convergenza della mesh.
- Corretto il crash del garbage collector dopo lo scaricamento dei plugin;
  processo e risultato fisico sono registrati separatamente.

## Evidenza locale e prossimo punto di controllo

Directory: `artifacts/deformable_leaf/d0-baseline-{60,120,240}`.
Configurazioni, sorgenti hashati, USD, log e tracce restano locali e ignorati da Git.
Il [grafico comparativo](../../../artifacts/deformable_leaf/comparison/comparison.png)
e il [rapporto JSON](../../../artifacts/deformable_leaf/comparison/comparison.json)
sono riproducibili con i comandi nel [README](README.md).

16 test automatici del codice passati; `git diff --check` passato.
D1 resta sospeso fino a una configurazione D0 accettabile: aumentare soltanto
la frequenza non ha risolto il recupero.

## Revisione GUI e correzione del clock

La prova `20260916T092420.508101Z-d0-120hz-gui` è stata interrotta prima della
fase di recupero. Il feedback umano è conservato separatamente in
`manual_review.json`, senza alterare il rapporto originale.

L'analisi ha identificato un errore del runner: `step(render=True)` con dt di
rendering 1/60 e dt fisico 1/120 avanzava due passi, alternati a un passo nelle
chiamate senza rendering. Il vecchio runner contava invece un solo passo per
chiamata. Prima della transizione di gravità dell'headless, riallineare le due
tracce con la sequenza 2/1 dà differenza nodale **esattamente zero**. La prima
GUI non dimostra quindi una nuova divergenza fisica causata dal rendering.
Il suo tempo registrato e le metriche derivate dal tempo non sono validi.

Correzione: un solo `step(render=False)` a ogni iterazione, seguito da
`render()` separato, controllo del contatore dei passi e lettura del clock
effettivo. Rendering e simulazione hanno tempi di esecuzione separati.
Questa correzione riguarda il runner; non risolve la risposta fisica insoddisfacente
già osservata nei tre test headless e non promuove D0.

Verifica completata: `d0-clockfix-render-v2-120` esegue 16 secondi con rendering
offscreen alla cadenza GUI. Tutti i **1.921 campioni nodali coincidono esattamente**
con `d0-baseline-120` (differenza massima 0 m); il clock differisce di soli
0,834 µs per l'accumulo float32. Nessun errore PhysX, worker terminato con
codice 0. Evidenza in `render_equivalence.json`. I criteri fisici rimangono
falliti; il test offscreen non costituisce una nuova accettazione visiva.
I 16 test del codice e `git diff --check` passano dopo la correzione.
