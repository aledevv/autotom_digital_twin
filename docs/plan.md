# Foglia stabile — Isaac Sim 6.1

## Obiettivo

Una lamina che si piega sotto gravità, reagisce a pressione lenta centrale e
fuori centro e recupera senza instabilità. Il picciolo è rigido e fissato al
supporto con un fixed joint. Non è richiesta la dinamica del picciolo.

Il banco isolato è in `src/experiments/leaf61`. Non modifica la pianta né i
precedenti esperimenti 4.5. Risultati locali in `artifacts/leaf61`.

## Sequenza

1. Verifica runtime 6.1, cooking surface, attachment e lettura nodale.
2. Rettangolo 90 × 40 mm, 30 × 14 celle, fascia fissata di 6 mm; riferimento 120 Hz.
3. Assestamento sotto gravità, poi sfera di raggio 10 mm al 65% della parte libera.
   Discesa di 2 s, mantenimento 1 s, ritiro 2 s, recupero 8 s. Profondità 10 mm
   dal primo contatto geometrico; tre cicli centrali e tre a y=10 mm.
4. Regolazione limitata a 12 configurazioni surface, separando Young, bending e
   damping, senza variare densità e spessore. Non usare stretch/shear override
   come proprietà supportate senza evidenza nella build installata.
5. Confronto con tre segmenti rigidi e UsdSkel, radice elastica e due giunti
   interni, picciolo fissato, stessa massa e stesso scenario.
6. Sul candidato: 60/120/240 Hz, raddoppio della risoluzione surface,
   contorno deterministico `real_leaves` da 90 mm, nuova verifica.

## Accettazione

Dati finiti; deriva della fascia <0,2 mm; estensione degli spigoli <5%; sag
visibile e <30% della lunghezza libera; nessun collasso. Ogni ciclo richiede
contatto, recupero <2 mm dall'equilibrio sotto gravità, oscillazione residua
<0,5 mm nell'ultimo secondo dopo 8 s dal rilascio. Controllo della distanza
sfera-triangoli per escludere penetrazione >1 mm. La revisione GUI dell'utente
resta necessaria: un report numerico non certifica plausibilità visiva.

Preferire surface se supera i controlli e mantiene il tempo reale. Altrimenti
valutare skinning e documentare i limiti dei collider rispetto alla mesh.
Nessuna promozione automatica se entrambi falliscono; nessun rilassamento
silenzioso delle soglie. Urti, picciolo oscillante, calibrazione biologica e
integrazione nella pianta sono esclusi da questa fase.

## Evidenza

Ogni run conserva configurazione, hash dei sorgenti, runtime, scena, traccia,
log e report. Tempi fisici, letture e rendering sono distinti. I risultati
4.5 rimangono evidenza storica, non validazione di questa implementazione.

## Smoke successivo: piantina fissa con più lamine skinnate

- Fixture sintetica separata da PlantState: tre rami statici, piccioli fissati al
  mondo, stesso contorno reale seed 42 e stessi parametri del candidato.
- Un mondo e un passo per iterazione; pose lette insieme a 120 Hz, UsdSkel a
  60 Hz. Casi annidati da 1/5/10/20 lamine; contatti reciproci filtrati.
- Regressione singola <0,1 mm prima della campagna. Una prova full per N,
  tre light headless e tre light offscreen; misure dopo t=9 s.
- Promozione: verifiche fisiche originali, lamine non premute entro 0,5 mm
  dall'equilibrio, RTF >=1 e p95 per due passi più rendering <=16,67 ms in tutte
  le ripetizioni. Sei cicli sul candidato, benchmark GUI separato e revisione umana.
- Coppia distinta con separazione 3 mm, sfera solo sulla superiore, confronto
  collisioni on/off, impulsi e separazioni PhysX, recupero di entrambe; penetrazione
  dei collider <1 mm. Gli attraversamenti visivi sono segnalati separatamente.
- Consegna: `run_leaf61_plant.sh`, `plant_campaign.py`, report per lamina e
  confronto locale; nessuna estrapolazione oltre la numerosità verificata.

Esito implementato e verificato: [rapporto piantina](../src/experiments/leaf61/PLANT_RESULTS.md).
Campagna di 32 esecuzioni concluse senza errori runtime o fallimenti numerici;
5 lamine superano anche tutte le soglie prestazionali offscreen e il benchmark GUI.
10 e 20 falliscono i criteri prestazionali indicati nel rapporto. Coppia in contatto
superata; revisione visiva umana della nuova fixture ancora da completare.
