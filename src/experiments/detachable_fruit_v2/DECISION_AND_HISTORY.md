# Detachment: resoconto delle prove e decisione finale

Decisione dell'utente, 14 settembre 2026: **per ora i truss vengono ammessi solo
sullo stem principale; niente truss sui rami laterali**. La ricerca sul ramo
laterale si ferma dopo i cinque tentativi finali. Non è stato dimostrato che il
distacco nativo su un ramo mobile sia impossibile: non abbiamo ottenuto una
configurazione utilizzabile e accettata nelle prove eseguite.

Questo documento registra la decisione e riunisce gli esperimenti. Non introduce
un filtro nuovo nel codice: i fixture `lateral` e `full` restano disponibili per
riprodurre la diagnosi, **non sono configurazioni approvate per l'uso corrente**.
Il fixture da usare come riferimento è `direct`.

## Soluzione scelta, in breve

Truss standard derivato dalla costruzione main, integrato come preset opzionale
`main-rank6-standard` in v2.3, limitato al caso day 160 con truss diretti sullo stem.

| Proprietà | Riferimento scelto |
|---|---|
| Struttura interna | 4 segmenti di rachide, 8 pedicelli, 8 pomodori per truss |
| Frutti | Profilo del rango 6 replicato: 74,708532 g complessivi per truss |
| Masse individuali | Circa 10,632; 10,464; 10,126; 9,689; 9,347; 8,776; 8,155; 7,520 g |
| Dimensioni e inerzie dei frutti | Sfere coerenti con 1.000 kg/m³ e con le masse indicate |
| Supporti del truss | Densità **artificiale** 20.000 kg/m³, drive/limiti del riferimento main |
| Runtime | TGS, fisica GPU, 60 Hz; articolazione 32/4, frutti 32/1 |
| Distacco | FixedJoint esterno all'articolazione, soglia 6 N attiva dall'avvio |
| Mouse | Shift+click PhysX **nativo**, modalità joint, coefficiente 10 |
| Esclusi dalla soluzione | Controller custom, ricostruzione del frutto/articolazione, protezione iniziale, truss laterali |

L'integrazione conserva parentela e punto d'attacco PlantState e usa un template
versionato per l'interno del truss. Non è una copia indistinta di tutta la pianta
main. Il profilo standard sostituisce dichiaratamente quantità e proprietà degli
organi originali; non rappresenta le masse biologiche originali di tutti i ranghi.

Evidenza v2.3 diretta: cinque truss, 40 frutti, 110 corpi; dopo la correzione
dell'orientamento, screening 20 s e conferma headless separata 60 s senza rotture.
GUI: giudizio positivo, quattro distacchi selezionati su quattro truss, nessun
errore riportato, 31,02 FPS dopo avvio, rapporto tempo simulato/reale 0,519.
Quella sessione dura **22,95 secondi simulati**: non va descritta come un minuto
completo di interazione validato. La pianta completa con vegetazione aggiuntiva
non è validata da questo risultato. Dettagli: [preset](../../exporterV2/presets/README.md).

## 1. Riproduzione iniziale su v2.3 e controlli del codice

- Corretta la sovrascrittura nell'adattatore della soglia 6 N/esclusione
  dall'articolazione: ora rispetta la configurazione. Corretto anche il caricamento
  in cui Isaac World ripristinava TGS su un esperimento dichiarato PGS.
- Controllati frame, collegamenti, masse e inerzie caricate; nessuna duplicazione
  strutturale o disallineamento iniziale dimostrato negli audit eseguiti.
- A 480 Hz, pianta senza frutti: nessun distacco ma avvisi sulle velocità. Pianta
  con frutti: prime rotture a 0,077 s, identiche con collisioni on/off. Rendere i
  frutti infrangibili non elimina il forte movimento della pianta completa.
- Un solo frutto con pedicello ancorato sopravvive; un truss intero riproduce il
  guasto. Togliere vegetazione o collisioni non basta. Frutto interno
  all'articolazione migliora alcuni errori, ma non soddisfa il distacco richiesto.
- Provati iterazioni, TGS CPU/GPU, velocità zero, rigidità e smorzamento. PGS
  migliora l'assestamento, ma restano errori angolari e la verifica manuale.
- I rapporti di massa effettivamente trovati inizialmente sono 6,24–10,445,
  non la prima stima geometrica 50–84. Velocità del solver e movimento ricavato
  dalle pose sono tenuti separati; alcuni vecchi calcoli del monitor sono stati
  corretti e i report storici indicano quali risultati non usare.

Dettagli e matrice completa: [RESULTS](RESULTS.md),
[ITERATIONS_60HZ](ITERATIONS_60HZ.md).

## 2. Mouse nativo, workaround custom e blocchi della GUI

- Sulla prima pianta PGS, una forza controllata crescente al baricentro o sulla
  superficie produce il distacco; i trascinamenti nativi provati non sempre lo
  producono. Il coefficiente nativo 1000 porta a comportamento violento: scartato.
- Sperimentato un controller custom a forza limitata. Corretti problemi di lettura
  dei raggi mouse e aggiunti punto di presa, freccia e forza visualizzata. Il gesto
  inizialmente è brusco o richiede troppo spostamento/tempo; successive versioni
  riescono a staccare e a mantenere il frutto afferrato fino al rilascio.
- Feedback importante sul custom: una trazione mantenuta intorno a 2–3 N su un
  frutto laterale trascina il supporto; ramo e truss iniziano a ruotare. È
  un'osservazione, non una misura della distribuzione delle reazioni nei joint.
- Il custom resta archiviato (`e1908c6`), ma **non è la soluzione finale scelta**.
  UI in inglese e soglia sperimentale 2,5 N appartengono a quella fase.
- Distinti i veri problemi fisici dagli arresti del monitor e dal cleanup.
  Introdotti log per fase, eventi timeline e supervisore indipendente con stack.
  Alcuni congelamenti osservati avvengono dopo Stop/reset o durante app.close;
  altri tentativi sono confusi da chiusure automatiche o controlli interrotti.
  Non tutti i “crash” riportati dimostrano un blocco del solver/GPU.

Fonti: [INTERACTION_RESULTS](INTERACTION_RESULTS.md),
[MAIN_COMPARISON](MAIN_COMPARISON.md), [GUI_FREEZE](GUI_FREEZE.md).

## 3. Confronto costruzione main/v2.3 e combinazioni

Il confronto ridotto mantiene stem e truss diretto rango 6. Main ha 4 segmenti di
rachide, v2.3 7; masse dei supporti, segmentazione e drive differiscono. I frutti
hanno lo stesso carico totale. La differenza di rigidità interna del rachide
(circa 56×) e dei pedicelli (circa 41×) segue le formule e le geometrie: non è
stata dimostrata una semplice conversione di unità errata.

| Prova | Comportamento |
|---|---|
| Main ridotto, 6 N | Riferimento manuale positivo per il gesto nativo |
| v2.3 ridotto | Distacchi spontanei e successiva forte oscillazione/accartocciamento osservati |
| Rigidità main, damping main, separati e insieme | Nessuna combinazione supera lo screening v2.3 |
| Stessi fattori × velocità solver zero (8 casi) | Tutti falliscono; prima rottura fra 0,05 e 0,117 s |
| Supporto minimo fisso o pedicello articolato, un frutto | Main e v2.3 superano 20 s |
| Rachide completo, un solo frutto | Main supera 20 s; v2.3 rompe a 4,9 s |
| v2.3: solo attacco del rachide bloccato | Guasto ritardato a 7,87 s, non risolto |
| v2.3: joint interni del rachide bloccati | Supera screening, ma mobilità sacrificata |
| v2.3: supporti a 20.000 kg/m³ | Supera screening del caso a un frutto; assestamento non perfetto |
| Main: supporti a 2.000 kg/m³, un frutto | Supera screening anche mantenendo mobilità originale |

I test combinati evitano di scartare un fattore solo perché la sua variazione
isolata non risolve tutto. Non provano però una causa microscopica unica.
Fonti: [NATIVE_COMPARISON](NATIVE_COMPARISON.md),
[STABILITY_FACTORIAL](STABILITY_FACTORIAL.md), [SUPPORT_ABLATION](SUPPORT_ABLATION.md),
[SUPPORT_MATRIX](SUPPORT_MATRIX.md).

## 4. Proporzioni, densità, rachide e soglia di rottura

- Dimostrato nel riferimento main un raggio del frutto raddoppiato da
  GLOBAL_SCALE=2 con massa invariata: densità implicita 125 kg/m³. Corrette sfere,
  inerzie e posizione rispetto all'attacco a 1.000 kg/m³, conservando la massa.
- Supporti a 1.000 kg/m³ falliscono nella scena a un frutto; 2.000 passano quel
  caso, ma non costituiscono una garanzia per un truss completo.
- Aumentare solo il damping del rachide (2×/4×) provoca rotture in quei controlli.
  Rigidità 4× riduce caduta/rimbalzo, ma l'utente la rifiuta: “come un bastone”.
  Provata anche rigidità intermedia 2× e combinazioni con damping.
- Soglie 2,5/3 N non sono promosse: possono rompere all'avvio. Il caso intermedio
  a 3 N rompe a 0,133 s. La protezione iniziale a 6 N dopo assestamento funziona
  nel caso ridotto, ma non stabilizza automaticamente un truss completo.
- Nella transizione a otto frutti, ridurre la densità 20.000→2.000 oppure
  raddoppiare la rigidità può già causare rotture. La combinazione alleggerita
  con protezione iniziale diverge senza neppure abilitare il distacco. Per il
  riferimento finale si mantengono supporti main e 6 N dall'avvio.

Matrici e feedback: [FRUIT_SCALE_CORRECTION](FRUIT_SCALE_CORRECTION.md),
[RACHIS_MOTION](RACHIS_MOTION.md), [SETTLING_GATE](SETTLING_GATE.md),
[MAIN_TRANSITION](MAIN_TRANSITION.md).

## 5. Più truss sullo stem e standardizzazione del profilo

- Due truss originali superano 20 s; cinque falliscono a 0,2 s sul rango 10.
  Il rango 10 da solo riproduce lo stesso guasto: il numero di truss non è
  necessario a questa prima rottura.
- Collisioni disabilitate: stesso guasto. Gravità zero verificata: passa.
  Trasferire solo le masse dei frutti rango 6→10 fa passare; il trasferimento
  inverso fa fallire il rango 6. Cambiare solo le inerzie non risolve.
  Il rango 10 pesa complessivamente meno: non basta spiegare il guasto come
  “troppo peso”; conta la distribuzione e la risposta dinamica accoppiata.
- 120 Hz: cinque truss superano riposo, ma la GUI fallisce durante interazione,
  circa 17,55 FPS e arresto per un altro frutto rotto. Il cleanup lento spiega
  il blocco successivo documentato; non è una soluzione accettata.
- A 60 Hz, aumentare iterazioni posizione a 64/64 e 128/128 peggiora i tempi
  della prima rottura del rango 10; nessuna promozione.
- Replicare il profilo rango 6 su cinque truss a 60 Hz dà il riferimento positivo:
  più GUI accettate, distacchi reali, circa 32–33 FPS; riposo headless 60 s passato.
  I report distinguono le sessioni brevi con distacchi dal minuto senza distacchi.
- Prima integrazione v2.3 capovolta: corretta la rotazione trasversale del template
  rispetto alla gravità, conservando origine e asse longitudinale canonici.
  Dopo la correzione i cinque truss diretti superano le prove sopra riportate.

Fonti: [MULTIPLE_TRUSSES](MULTIPLE_TRUSSES.md), [RANK_DIAGNOSIS](RANK_DIAGNOSIS.md),
[LAST_SOLVER_TEST](LAST_SOLVER_TEST.md), [REPEATED_RANK6](REPEATED_RANK6.md),
[preset e orientamento](../../exporterV2/presets/README.md).

## 6. Diagnosi specifica del ramo laterale

Caso: `Branch_s2_o0_g421408`, truss `Truss_r5_o0_g421757`, otto frutti standard.
Il ramo aggiunge quattro joint sferici mobili, due DOF angolari ciascuno ±30°,
rispetto allo stem a collegamenti fissi. Ramo 48,88 g; truss completo 175,24 g.

| Intervento | Risultato osservato |
|---|---|
| TGS, ramo originale | Prima rottura spontanea a 0,333 s |
| Solo stem→ramo fixed | Non basta: rottura a circa 0,300 s |
| Tutti quattro joint laterali fixed | 20 s superati; solo controllo diagnostico, mobilità persa |
| Collisioni off, ramo mobile | Stessa rottura a 0,333 s, stati registrati identici al caso con contatti |
| Collisioni off, ramo interamente fixed | 20 s superati |
| Origini dei corpi sui COM, geometria globale conservata | Stessa rottura a 0,333 s |
| TGS, iterazioni velocità zero | Stessa rottura a 0,333 s |
| PGS, solo solver cambiato | 20 e 60 s superati a riposo, ma marcata flessione iniziale |
| PGS GUI, native joint 10 | Movimento stabile, circa 56,51 FPS su 197,65 s; nessun distacco, rifiutato |
| Forza diretta COM 0–12 N/5 s, mobile/fixed | Entrambi staccano il bersaglio a comando 6,12 N; 60 s senza ulteriori rotture |
| Native joint 10, salita 20 cm/5 s, mobile/fixed | Nessuno stacca; movimento frutto 6,10 / 1,74 mm |
| Stesso gesto su fixed, TGS anziché PGS | Ancora nessun distacco; circa 2,08 mm di movimento |

Prima della rottura TGS il supporto scende già fortemente; joint entro i limiti,
frame iniziali coerenti, inerzie positive. Collisioni non necessarie al difetto
precoce. Le limitazioni NVIDIA su COM/joint sferici e TGS sono state considerate,
ma i controlli dedicati **non identificano né risolvono uno specifico bug NVIDIA**.
Una forza diretta che funziona non dimostra che funzioni il gesto nativo.
Il coefficiente mouse non è una misura in newton.

Fonti: [LATERAL_DIAGNOSIS](LATERAL_DIAGNOSIS.md),
[LATERAL_FORCE_PAIR](LATERAL_FORCE_PAIR.md), [LATERAL_NATIVE_PAIR](LATERAL_NATIVE_PAIR.md),
[SUCCESSFUL_NATIVE_GESTURES](SUCCESSFUL_NATIVE_GESTURES.md).

## 7. Ultimi cinque tentativi: esito definitivo della ricerca corrente

Replay del primo gesto nativo registrato con successo sul main, traslato sul
frutto laterale e con selezione verificata. PGS/GPU, 60 Hz, 32/4 e 32/1, soglia
6 N, masse/geometrie invariate. Nessun controller mouse custom.

| # | Configurazione | Headless e riscontro |
|---|---|---|
| 1 | Native joint 10, ramo originale | 60 s senza instabilità grossolana, ma nessun distacco |
| 2 | Native joint 50, ramo originale | Nessun distacco; stati uguali al tentativo 1 |
| 3 | Native force 50, ramo originale | Replay rapido stacca a 30,45 s, 60 s passano; gesto lento non stacca. GUI: ramo “spaghetto cotto”, rifiutato; arresto dopo rottura non attribuita alla selezione |
| 4 | Native force 50, K ramo ×10, D ×30 | Distacco e 60 s passano; attacco scende ancora 15,13 cm contro 17,81 cm del caso 3. Miglioramento insufficiente; non promosso in GUI |
| 5 | Native force 50, K ×100, D ×1000 | Distacco a 30,333 s, 60 s funzionali; attacco scende 22,95 cm. GUI rifiutata: ancora troppo molle; arresto a 9,87 s per rottura fuori dalla presa identificata |

Nel caso 5 le escursioni finali delle pose sono piccole (0,276 mm), ma le
velocità riportate dal solver superano il filtro numerico stretto. Né quel dato
né il pass funzionale annullano il giudizio negativo sul sostegno del ramo.
Le moltiplicazioni K/D non hanno fissato i joint e non sono calibrazioni biologiche.
Il primo avvio GUI del caso 3 falliva sul controllo del coefficiente >10 prima
della fisica: corretto il launcher sperimentale e ripetuta la stessa configurazione,
non conteggiata come nuovo candidato. Nessun sesto candidato è stato eseguito.

Registro completo di proprietà e risultati: [native_attempts.json](native_attempts.json).
Resoconto: [NATIVE_ATTEMPTS](NATIVE_ATTEMPTS.md). Checkpoint delle prove: `efb2da9`.

## Limiti e conservazione

La scelta di escludere i truss laterali è un limite operativo concordato, non
una conclusione botanica o la dimostrazione che il loro numero sia la causa.
Non è stata isolata una causa unica dell'instabilità. Restano possibili interazioni
fra catena mobile, carico, drive e solver. La modalità native force non è stata
accettata come sostituto della modalità joint del riferimento diretto.

Codice e report sintetici sono sul branch `experiment/v2-detachable-fruit-stability`;
main e groPy restano intatti. USD, log, frame, stack e serie temporali rimangono
locali sotto `artifacts/detachable_fruit_v2/`. I documenti collegati conservano
matrici complete, comandi, hash e limiti delle singole fasi; le loro proposte di
“prossimo passo” sono storiche e non autorizzano altre prove dopo questa decisione.
