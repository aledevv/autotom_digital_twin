# Estrazione robusta dei PlantState da GroIMP

Usare sempre `extract_plant_states.sh` per creare i JSON canonici destinati
agli exporter. Lo script non usa CSV e non modifica il progetto o gli input
originali.

## Comandi

Un solo giorno:

```bash
./extract_plant_states.sh --day 50
```

Tutti i giorni da 1 a 160, rigenerando eventuali file esistenti:

```bash
./extract_plant_states.sh --from-day 1 --to-day 160 --overwrite
```

Ripresa sicura dopo un'interruzione, mantenendo i giorni già completati:

```bash
./extract_plant_states.sh --from-day 1 --to-day 160 --skip-existing
```

I file vengono salvati in `data/plant_states/plant_state_day_N.json`. Per
scrivere altrove usare `--output-dir PATH`; per una pianta diversa usare
`--plant-id N`.

L'estrazione di un giorno `N` parte comunque dal giorno 1 e applica in ordine
tutti gli step fino a `N`: non è possibile saltare direttamente lo stato
dinamico intermedio. Per un intervallo lo stesso workbench viene compilato una
sola volta, quindi il batch è più corretto ed efficiente di 160 invocazioni
separate. I node ID nativi rimangono inoltre coerenti all'interno dello stesso
run.

## Perché non aprire direttamente il GSZ dallo script

In GroIMP 2.2.1 headless `getWD()` non indica necessariamente la directory del
file `.gsz` aperto. In questa installazione può restituire la home utente.
`parameters_derived.rgg` prova allora a leggere una directory `input/`
sbagliata durante l'inizializzazione statica, prima che il bridge possa
compilare il progetto. Le conseguenze osservate includono `listFiles()[0]`,
`NoClassDefFoundError: parameters_derived` e il messaggio `Infinity Removed`.
Non è un reset che l'utente deve eseguire e non implica che il GSZ originale
sia corrotto.

Il runtime risolve il problema prima di `openWB`:

1. copia GSZ e `model/input/` in una directory temporanea;
2. riscrive `PATH_INPUT` e `PATH_OUTPUT` esclusivamente nella copia del GSZ;
3. apre, compila e avanza un solo workbench isolato;
4. verifica il giorno osservato dopo ogni step;
5. valida il `PlantState` e pubblica ciascun JSON con una sostituzione atomica;
6. chiude sempre il workbench, anche in caso di errore o interruzione.

La correzione è implementata in `groimp_bridge.runtime.configure_isolated_paths`.
Non rimuovere o posticipare questa chiamata: applicarla dopo `openWB` sarebbe
troppo tardi per l'inizializzatore statico di `parameters_derived.rgg`.

## Comportamento in caso di errore

Il comando termina con codice diverso da zero e stampa il giorno atteso e
quello letto da GroIMP quando non coincidono. I JSON già pubblicati rimangono
validi; il file del giorno in corso non può rimanere parzialmente scritto.
Riavviare il server GroIMP solo se non risponde più, quindi rilanciare il batch
con `--skip-existing`.

Non combinare `--overwrite` e `--skip-existing`: il primo dichiara che i file
possono essere sostituiti, il secondo che devono essere preservati.
