# v2.3 — Percorso di miglioramento: stabilità, realismo e collisioni

Aggiornamento: 16 settembre 2026. Sintesi dell'intero percorso discusso nella chat, verificata contro i resoconti sperimentali del repository. Non è una nuova campagna di test né una validazione generale dell'exporter.

Pubblicato anche come [nota nella pagina Notion v2.3](https://app.notion.com/p/3dd6c3a1ba6f810b95cbda62bd85543b).

## Risultato raggiunto e confini

Siamo passati dalla pianta v2.3 senza frutti fisici utilizzabili a una **scena sperimentale day 160 con vegetazione completa e cinque truss diretti sullo stem, 40 pomodori con profili originali dei ranghi 6–10, distacco fisico e mouse nativo**. Il riferimento estetico ha ricevuto feedback positivo. L'estensione delle collisioni fra supporti e petioluli resta un esperimento locale, non una soluzione generalizzata e definitivamente accettata.

La configurazione mantenuta prima della campagna collisioni usa PGS/CPU, rendering GPU, 60 Hz, iterazioni articolazione 32/4 e frutti 32/1, distacco a 3 N dall'avvio, mouse PhysX nativo in modalità force con coefficiente 50 e pacing GUI a velocità reale quando sostenibile. **50 è un coefficiente del mouse, non una forza misurata in newton.**

Restano due compromessi importanti: costruzione interna dei truss derivata da main e densità artificiale dei loro supporti pari a 20.000 kg/m³. I truss laterali sono esclusi per decisione operativa; questo non dimostra che siano fisicamente impossibili. L'integrazione più recente usa script sperimentali post-export, non un nuovo preset pubblico generico che sostituisce automaticamente tutto l'exporter.

## 1. Metodo: rendere il problema osservabile

Il requisito pratico è diventato: frutti attaccati a riposo, staccabili, recupero utilizzabile e almeno 20 FPS in GUI. Il minuto simulato ha sostituito la richiesta iniziale di tre minuti; le sessioni manuali più brevi sono rimaste documentate come tali.

- Checkpoint e branch sperimentale separato da main/groPy; USD e log pesanti locali.
- Controllo di parametri effettivamente caricati, frame dei joint, masse, inerzie, collider e scala.
- Campionamento per passo fisico, eventi JOINT_BREAK, errori degli attacchi e primo evento anomalo.
- Distinzione tra velocità restituite dal solver e movimento ricavato dalle pose; un frutto che si muove non è necessariamente staccato.
- Confronti a scena e input invariati, ripartenza dalla posa iniziale, prove ridotte e verifica manuale precoce.
- Ablazioni e combinazioni A, B, A+B: un singolo cambiamento che non risolve tutto non esclude che quel fattore contribuisca al problema.

Sono stati corretti errori concreti: sovrascritture della configurazione del distacco nell'adattatore, selezione del solver ripristinata dal caricamento, alcune misure del monitor e problemi di selezione nei replay. Queste correzioni vanno distinte dalle successive scelte di parametri.

## 2. Frutti, struttura e confronto con main

Togliere collisioni o vegetazione non eliminava alcune rotture precoci. Il caso minimo con un frutto poteva funzionare mentre un rachide completo falliva: non bastava attribuire il problema ai frutti o al numero totale di organi.

Il confronto main/v2.3 ha mostrato differenze di segmentazione, masse dei supporti e drive. Portare rigidità e smorzamento verso main, separatamente e insieme, non ha risolto il caso v2.3. Bloccare i joint interni migliorava la stabilità ma sacrificava la mobilità: utile come diagnosi, non come risultato accettabile.

Il riferimento main non era automaticamente realistico: il raggio dei frutti subiva una scala 2 con massa invariata, dando una densità implicita di 125 kg/m³. Dimensioni, inerzie e attacco sono stati corretti verso sfere coerenti con 1.000 kg/m³. Ridurre le masse dei supporti o aumentare il damping non ha prodotto miglioramenti universali; il risultato dipendeva anche dalla scena e dal carico.

## 3. Interazione: workaround custom e ritorno al nativo

Una forza controllata riusciva a staccare dove alcuni gesti nativi non riuscivano. È stato quindi provato un controller custom con forza limitata, indicatori e mantenimento della presa dopo il distacco. Il checkpoint custom è stato conservato, ma non è il riferimento finale.

Il feedback manuale ha individuato problemi che il semplice test a riposo non mostrava: gesto troppo lungo, distacco brusco, supporti trascinati sotto trazione mantenuta e frutto apparentemente ricreato/cadente. Il confronto con main ha motivato il ritorno al mouse PhysX nativo. Non sono stati modificati i sorgenti o i binari PhysX.

I congelamenti sono stati analizzati separando input, fisica, lettura, rendering e cleanup. Supervisore esterno e stack hanno mostrato che alcuni blocchi avvenivano dopo Stop/reset o durante la chiusura: non ogni segnalazione di “crash” era una divergenza fisica.

## 4. Perché il rango 6 sembrava speciale

Replicare il profilo del rango 6 su cinque truss aveva dato un riferimento stabile, ma sostituiva la distribuzione originale dei frutti. Era una soluzione standardizzata, non generale.

Il rango 10 riproduceva il guasto anche da solo. Scambi di masse mostravano sensibilità alla distribuzione, non semplicemente al peso totale: il rango 10 pesava complessivamente meno del 6. Un'indagine successiva ha isolato un comportamento anomalo del percorso TGS/GPU con FixedJoint esterno sostenuto da un'articolazione.

Nel controllo con massa sospesa, 20 g producevano una rottura con soglia nominale 6 N e 32 iterazioni, benché il peso fosse circa 0,196 N. A 64 iterazioni rompevano anche 10 g. I controlli CPU/TGS, GPU/PGS e supporto cinematico esterno all'articolazione non riproducevano quel comportamento nei casi provati.

La soglia osservata si comportava approssimativamente come 6/N nei fixture analizzati. È **evidenza empirica circoscritta**, non identificazione della riga difettosa nel codice NVIDIA né spiegazione di tutte le instabilità. Non abbiamo compensato moltiplicando arbitrariamente la soglia né patchato PhysX. Il percorso utilizzabile è stato cercato fra configurazioni già disponibili.

## 5. Recupero, soglia a 3 N e dati originali

PGS/CPU è diventato il riferimento utile per recupero e interazione. Il frutto che “cadeva pesantissimo” in una prova correva in realtà in una simulazione oltre 2,5 volte più veloce del tempo reale: la successiva analisi del moto mostrava accelerazione gravitazionale normale. È stato aggiunto pacing opzionale della GUI, separato da frequenza fisica e rendering.

Sul rango 10, 4 N e 3 N hanno superato prove a riposo e di distacco da 60 secondi. Con lo stesso gesto registrato il picco di velocità del frutto è sceso da circa 12,41 m/s a 6 N a 6,96 m/s a 3 N. Questo riduceva la bruschezza, senza dimostrare un gesto perfettamente naturale. Il rapido richiamo verso il puntatore è rimasto una caratteristica osservata del gesto nativo, non risolta con un nuovo controller custom.

La successiva campagna sui ranghi originali 6–10 ha completato 16 controlli headless da 60 secondi: riposo e distacco sui ranghi singoli, riposo combinato e un distacco per rango nella scena combinata. Le masse totali dei frutti per truss erano circa 74,71 / 69,37 / 62,62 / 56,26 / 57,23 g. Non veniva più copiato il rango 6 sugli altri.

L'utente ha accettato la GUI con cinque truss e circa 60 FPS. La sessione registrata durava 47,73 secondi simulati, con 12 distacchi selezionati: feedback positivo, ma non un minuto manuale completo.

## 6. Vegetazione completa e limite dei rami laterali

L'integrazione conserva vegetazione e parentela v2.3, innestando i truss main-derived nelle posizioni dirette corrette. Sono state verificate scala, frame e proprietà caricate. Una prima integrazione capovolta è stata corretta nell'orientamento del template; non era la prova di una nuova instabilità del solver.

Otto controlli headless da 60 secondi sulla progressione diretta/completa hanno passato riposo e distacco sui cinque ranghi. La scena completa di riferimento conta 181 corpi: 81 vegetativi, 60 supporti truss e 40 frutti, con 131 foglioline generate. L'utente ha successivamente riportato buona stabilità in GUI.

Sui rami laterali, invece, il supporto poteva flettersi molto, assorbire il gesto o diventare incontrollabile. Fissare soltanto l'attacco allo stem non bastava; fissare tutta la catena eliminava la mobilità richiesta. I confronti con collisioni off, frame COM, solver e trazione controllata non hanno prodotto un candidato manualmente accettato. Decisione: **per ora niente truss laterali**.

## 7. Realismo visivo senza rifare la fisica

La foto fornita è stata usata come riferimento qualitativo di proporzioni, non come misura. È stato conservato il raggio GroIMP dei frutti, senza ingrandimento estetico.

- Rachidi visivamente più sottili e rastremati: diametro 3→1,95 mm, lieve curvatura.
- Pedicelli 2,4→1,92 mm e calici decorativi a cinque sepali, solidali al frutto.
- Supporti delle foglie leggermente più cedevoli: rigidità rotX/rotY ×0,8, con damping e masse invariati. Una prova abbinata a 0,05 N mostrava circa il 26% di cedevolezza in più e recupero vicino alla posa precedente.
- Petioluli statici inclinati di 10° verso il basso, con la lamina che segue; è una posa estetica, non flessione fisica sotto gravità.

Il risultato estetico ha ricevuto riscontro positivo. Le variazioni restano script sperimentali riproducibili. La grafica più sottile non rende automaticamente più sottili i collider: questa separazione è diventata importante nella fase seguente.

## 8. Collisioni: separare filtri, geometria e costo

Dal riferimento con self-collision dell'articolazione disabilitata sono stati preparati A (originale), B (stesse esclusioni esplicite), C (contatti vegetativi ammissibili) e previsto D (anche supporti truss). A/B/C hanno superato gli screening. A e B mostravano stati registrati identici a riposo; C registrava alcuni contatti durante l'assestamento.

La scena iniziale ha 262 collider: 162 capsule, 60 cilindri e 40 sfere. Le coppie ammissibili sono 740 in A/B e 12.796 in C. Sono conteggi di eleggibilità, **non contatti effettivi né complessità misurata del solver**.

Il primo replay A/B/C non generava il contatto cercato: i supporti restavano separati. È stato classificato inconcludente, senza aumentare automaticamente la forza. La prova manuale ha poi evidenziato attraversamenti e permesso di identificare i prim corretti.

## 9. OrganicVisual e petioluli: cosa è stato corretto davvero

Le superfici visibili OrganicVisual non erano tutte collider. I petioluli statici e le lamine avevano grafica ma non collisione. I contatti misurati sul rachide non dimostravano quindi copertura dell'intera superficie visibile.

Il primo test aggiungeva dieci collider convessi soltanto contro il ramo grosso; l'utente non vedeva miglioramento. I log mostravano che il gesto coinvolgeva anche la foglia laterale `LatLeaf_r3_o0_g421593`, esclusa dalla prova. È stata corretta la copertura: 17 nuove forme su cinque corpi, includendo entrambi i segmenti di `Leaf_r5_o0_g421371_rachis`, il ramo `Branch_s2_o1_g421414_Link_04_Internode_g421675` e la foglia laterale.

Le nuove forme collidono fra i due gruppi selezionati ma non all'interno del proprio gruppo. Nessuna estensione globale, nessun nuovo corpo o joint, nessun collider sulle lamine. Le masse e inerzie effettive sono state rese esplicite e confrontate: aggiungere forme non deve modificarle involontariamente. I convex hull possono riempire concavità: sono approssimazioni da valutare visivamente.

## 10. Posa diversa non significa instabilità

La variante estesa si fermava al reset perché un supporto si spostava di 6,279 mm. Il vecchio controllo ammetteva solo 1 µm rispetto alla posa iniziale. L'utente ha chiarito correttamente che i nuovi contatti possono separare rami inizialmente sovrapposti e produrre una posa di equilibrio diversa.

È stata aggiunta una tolleranza **diagnostica esplicita**, impostata a 10 mm solo per questi casi. Il default resta 1 µm, come il controllo della radice fissa; la posa sorgente e lo spostamento misurato restano registrati. Nessuna attivazione ritardata delle collisioni è stata implementata.

Con questo criterio, 20 e 60 secondi a riposo passano senza errori riportati o distacchi. L'ultima GUI ha registrato 293,8 secondi simulati, senza errori rilevati né eventi di rottura; chiusura richiesta via SIGINT, processo terminato con codice 0. FPS riportati: circa 38,8 medi; l'utente osservava circa 33 FPS nella parte finale. Non è un benchmark A/B controllato e non attribuisce tutto il costo ai collider. Il rapporto grezzo tempo simulato/reale non è ancora una misura corretta escludendo tutte le pause.

Il beneficio finale contro l'attraversamento non ha ancora ricevuto un'esplicita conferma dell'utente. Superare riposo e soglia FPS non basta a dichiarare risolto il contatto.

## 11. Strategie da riutilizzare

- Separare costruzione, parametri, interazione e monitoraggio; non chiamare ogni arresto “crash fisico”.
- Usare main come controllo positivo, verificandone però scala e proprietà artificiali.
- Ridurre la scena e provare anche combinazioni; un miglioramento parziale resta informativo.
- Verificare la selezione mouse: un replay che colpisce il corpo sbagliato non è un test valido.
- Misurare proprietà caricate e contatti reali; flag USD e silhouette non bastano.
- Consentire un nuovo equilibrio quando cambia la fisica dei contatti, preservando i controlli di divergenza e la provenienza della posa.
- Tenere separati FPS, frequenza fisica e velocità del tempo simulato; il limite di 60 FPS può nascondere costi aggiuntivi.
- Affiancare sempre al test automatico il giudizio umano su gesto, recupero e realismo.

## 12. Stato salvato e prossimi confini

Checkpoint del codice: [545d068](https://github.com/aledevv/autotom_digital_twin/commit/545d068), branch `experiment/v2-detachable-fruit-stability`. Il salvataggio include anche `docs/plan.md`, piano separato per futuri esperimenti sulle foglie, non eseguito in questa fase.

Solo commentata, non attivata, l'estensione a `LeafVisual_g421563_petiolule_left_02` e analoghi: collisioni con terzi, esclusione del proprio gruppo, controllo di sovrapposizioni e costo prima di ampliare.

Restano aperti: accettazione finale dei contatti, variante D, benchmark abbinati con/senza pacing e registrazione, conferme indipendenti e generalizzazione ad altri organi/giorni. La campagna a pianta invariata non misura la complessità al crescere del numero di organi; dai pochi test non stimiamo una probabilità di crash.

## Fonti tecniche nel checkpoint

- [Storia iniziale e decisione sui laterali](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/detachable_fruit_v2/DECISION_AND_HISTORY.md): documento storico, non configurazione finale del 16 settembre.
- [Diagnosi della soglia TGS/GPU](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/detachable_fruit_v2/BREAK_SCALING_DIAGNOSIS.md).
- [Recupero, gesto nativo e soglie](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/detachable_fruit_v2/GENERIC_TRUSS_SEARCH.md).
- [Campagna sui ranghi originali](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/detachable_fruit_v2/ORIGINAL_RANK_CAMPAIGN.md).
- [Pianta completa](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/detachable_fruit_v2/FULL_PLANT_PROGRESSION.md) e [varianti estetiche](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/detachable_fruit_v2/REALISM_VARIANTS.md).
- [Collisioni e stato da riprendere](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/branch_collisions/HANDOFF.md), [risultati assestamento](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/branch_collisions/organic_settling_results.json), [ultima GUI](https://github.com/aledevv/autotom_digital_twin/blob/545d068/src/experiments/branch_collisions/organic_gui_results.json).

USD, log, stack e serie temporali dettagliate restano locali nelle cartelle `artifacts/detachable_fruit_v2/` e `artifacts/branch_collisions/`.
