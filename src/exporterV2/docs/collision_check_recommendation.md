# Collision check dopo remapping degli attachment

Per il tuo caso la scelta migliore è **d. Other: broad-phase a due stadi**.

## Raccomandazione

Usa:
1. **Sphere overlap** come pre-check molto veloce.
2. **AABB overlap** o, meglio ancora, un **OBB / capsule-style check** solo se il pre-check passa.
3. Se serve precisione finale, valida con una query PhysX/USD più costosa solo sui casi ambigui.

## Perché non sceglierei un singolo metodo

### AABB overlap
È il più semplice e veloce, ma nel tuo caso può generare molti falsi positivi quando i rami sono inclinati o ruotati. Dopo il remapping degli attachment, i rami cambiano orientamento: l’AABB tende a “gonfiare” il volume e quindi può bloccare soluzioni che in realtà sono sicure.

### Sphere overlap
È ancora più rapido e robusto per una prima esclusione, ma è troppo conservativo per geometrie allungate come stem, petioli e rami. Va benissimo per dire “questo attachment è sicuramente lontano da siblings e parent”, ma non basta come check unico.

### Cilindri orientati
È la soluzione più vicina alla geometria reale dei tuoi link. Però è più complessa da implementare, soprattutto se devi gestire attachment remappati, rotazioni locali e collisioni tra siblings su più livelli della gerarchia.

## Strategia consigliata

La combinazione più sensata è questa:

- **Stage 1: sphere overlap** per scartare i casi chiaramente validi o chiaramente invalidi.
- **Stage 2: AABB / OBB** per i casi dubbi.
- **Stage 3: collision validation in engine** solo se il caso resta ambiguo.

Questa architettura è coerente con il tuo obiettivo: ottimizzare in modo progressivo senza spendere troppo costo computazionale nella fase di progettazione/export.

## Nel tuo caso specifico

Dato che lavori con:
- links cilindrici,
- attacchi remappati in altezza,
- siblings da filtrare,
- e un budget rigido di joint,

la soluzione più bilanciata è **sphere + AABB**, non AABB da solo e non cilindri orientati come primo step.

In pratica:
- **sfera** per controllare rapidamente se il nuovo attachment è troppo vicino a un sibling o al parent,
- **AABB** per verificare l’effettivo ingombro locale,
- **collision filtering** in USD/PhysX per evitare contatti spurii quando il remapping è comunque valido.

## Suggerimento operativo

Per la feature che stai pianificando, inserirei questa logica nel file dedicato alla tecnica di remapping del main stem:

- calcolo della nuova posizione di attachment,
- generazione di un bounding proxy per il link nuovo,
- test sphere-based contro i vicini,
- test AABB contro i candidati rimasti,
- applicazione dei filtered pairs se il check passa.

Se vuoi una scelta secca: **non userei solo AABB**. Per il tuo exporter sceglierei **approccio ibrido broad-phase sphere + AABB**, perché è il miglior compromesso tra semplicità, robustezza e costo.
