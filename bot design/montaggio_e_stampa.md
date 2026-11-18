# Scocca, bozza 2.3: pezzi, montaggio, stampa e prove (11 novembre 2026)

**Larghezza esterna 150 mm** (prima 160), altezza 176 mm. Piastre delle staffe a ±76 mm: tra le estremità dei due motori restano ~30 mm. Con le ruote il robot è largo ~214 mm.

File in `bot design\modelli`; in Fusion lo script **Scocca** li importa.

## Pezzi

| Pezzo | Cosa fa | Stampa | Supporti stimati |
|---|---|---|---|
| **SCHIENA** | guscio posteriore: finestre delle staffe (aperte sotto e davanti), mensole per pavimento e vassoio, dente posteriore del vassoio, sporgenza IMU, nicchie per interruttore e ricarica (metà fissa della culla), fascia di centraggio della cover sui fianchi, **guide per i moduli** sui fianchi interni | **capovolta**, tetto sul piatto (il tetto è spianato apposta). Alta 174 mm, **senza brim** (colla sul piatto), parete esterna lenta, altezza layer variabile | ~14 g |
| **COVER** | guscio frontale con l'apertura per la visiera | **capovolta**, tetto sul piatto | ~9 g |
| **VISIERA** | pezzo nero P1 con 4 colonnine (2 nel collo, 2 ai lati della bocca), viti autofilettanti | **faccia in alto**, appoggiata sulle colonnine: i supporti stanno solo sotto il retro, la faccia viene pulita | ~3 g, solo sul retro |
| **PARATIA** | regge display e HC-SR04; con la visiera forma il **modulo viso** | dorso sul piatto, **senza supporti** | 0 |
| **FERMO** | barretta che ferma la batteria davanti; si avvita dietro la paratia (pezzo a parte, così la paratia si stampa senza supporti) | piatto | 0 |
| **PAVIMENTO** | regge staffe e motori (4 fori per staffa: due lembi arrivano fino alle piastre passando nelle finestre dei fianchi) e le due millefori nelle **guide a cassetto** | piatto, guide in su, **pieno al 100%** (sotto le teste tonde delle viti solo plastica piena) | 0 |
| **VASSOIO** | cassetto della batteria: fermi laterali bassi e pieni (6 × 3 mm), niente labbri; sopra la tiene il tetto, davanti i fermi della paratia | piatto, fermi in su | 0 |
| **TAPPO** | metà superiore della culla del connettore di ricarica; si chiude con 2 viti verticali | capovolto, sedi dei dadi sul piatto | 0 |
| **SLITTA** | modello di partenza per agganciare un modulo (speaker, microfono) alle guide | faccia del modulo sul piatto | 0 |

Pezzi ~230 g di PLA in tutto, supporti ~34 g (stima per eccesso, con supporti ad albero).

## Viteria (tutto M3 con dado; fori passanti 3,4 mm, sedi esagonali per dadi da 5,5)

Unica eccezione: le 4 viti della visiera entrano direttamente nelle colonnine di plastica (foro da 2,8 mm, la vite M3 normale si crea il filetto). Vanno bene perché la visiera si smonta di rado.

| Dove | Viti | Dado |
|---|---|---|
| Staffe → pavimento | 8 × M3×12, testa sopra il pavimento (le 4 interne si avvitano dall'alto attraverso i fori nelle guide delle millefori) | nelle staffe |
| Pavimento → mensole della schiena | 4 × M3×12 dal basso, testa sotto il pavimento (a x −30/y ±63,5 e x −34,8/y ±56,5). Cacciavite dritto dal basso attraverso i 4 fori nel fondo della schiena | infilato dalla faccia interna della mensola |
| Vassoio → mensole della schiena | 2 × M3×10 **dal basso** (x 15, y ±64), testa su una spianatura sotto la mensola. Si avvitano **prima** di infilare il modulo pavimento, dal fondo aperto | nella torretta esagonale sopra il vassoio |
| Paratia → pavimento | 2 × M3×10 **dal basso**, testa sotto il pavimento | infilato dal davanti nel piede della paratia (piede da 6 mm) |
| Paratia → vassoio | 2 × M3×10 dal davanti (y ±57) | nel blocco del vassoio, sede aperta in alto |
| Fermo batteria → paratia | 2 × M3×12 dal davanti (ai lati del display, y +32,5 e −36) | infilato nella sede esagonale sul retro del fermo (lato batteria) |
| Visiera → paratia | 4 × M3×10 da dietro la paratia | nessuno: la vite fa il filetto nella colonnina |
| Cover, sotto | 2 × **M3×12** dal basso, testa incassata | nella sede esagonale sopra il pavimento (metterli prima di infilare la millefori grande) |
| Cover, fianchi | 2 × **M3×8 a brugola** (testa cilindrica 5,5 × 3) dall'esterno, testa **incassata a filo** del fianco della schiena (sede Ø6,4). Non più lunghe: la punta si ferma 0,4 mm prima dell'orecchio del vassoio | sede esagonale nel **tassello** sul fianco interno della cover, infilato dall'interno prima di montare la cover (goccia di colla se non resta) |
| IMU | 2 × **M3×10** dal davanti, attraverso la scheda e i 2 mezzi anelli da 3 mm | infilato dall'alto nella sede dentro il blocco (sede su misura, fondo a V: non gira), **prima di mettere il vassoio** |
| Tappo della ricarica | 2 × **M3×16** a brugola **dal basso** (testa incassata 4,5 mm sotto la metà fissa), chiave a L da 2,5: lato corto in su, ripiano tolto | nelle sedi esagonali sopra il tappo |
| Morsetti delle staffe | 4 × M3×16 | dado |
| Motori → staffe (faccia del riduttore) | 4 × M3×6 (o ×8) testa cilindrica | nessuno: filetto del riduttore |
| Slitta → guida dei moduli | 1–2 × M3×20, verticali — **DA RIFARE**: a robot montato le viti non si raggiungono né dall'alto (vassoio) né dal basso (pavimento, staffe) | infilato dalla faccia della guida |
| Modulo → slitta | 2 × M3 (lunghezza secondo il modulo) | nella slitta |

Il display è tenuto da due perni nei fori della linguetta, ribattuti scaldandone la punta con il saldatore; davanti la visiera lo ferma a 0,8 mm, senza toccare il vetro. L'HC-SR04 sta su 4 colonnine con perni.

Davanti il corpo non è avvitato direttamente al pavimento: sotto il pavimento, per tutta la fascia davanti, ci sono le basi delle staffe (larghe 48 mm). Il collegamento passa da un telaio chiuso: pavimento → paratia (2 viti) → vassoio (2 viti) → mensole della schiena (2 viti), più le 4 viti dietro e la cover (2 sotto, 2 sui fianchi).

## Millefori: guide a cassetto

- Millefori grande 50 × 70 (fori di fissaggio Ø2 a 46,5 × 66 tra i centri: non usati), millefori di potenza 20 × 80 (fori a 17 × 76).
- Le schede scorrono da davanti in scanalature alte 2,1 mm (schede da 1,6 mm), entrano 1,5 mm per lato sui bordi lunghi; faccia inferiore a 10 mm sopra il pavimento (spazio per saldature e fili). Fermo dietro; davanti la scheda grande la ferma la paratia, quella di potenza la cover (~2 mm di gioco).
- Lungo i bordi lunghi, per 1,5 mm, niente componenti o saldature sporgenti.

## Retro della schiena (17 novembre)

Provini prima della schiena: `stampa\provini_schiena\provini_schiena.3mf` (~18 g): IMU, interruttore, ricarica, tratto di guida + spalla del ripiano, ritagliati dalla schiena definitiva e orientati come nella stampa vera.


- **Interruttore KCD1** (corpo 18,5 × 11,5, profondo 11): dietro a **destra**, foro 12 × 19 in una parete da 2 mm, a scatto. Dopo i provini il blocco delle nicchie è più alto: la striscia sopra le nicchie è spessa 3,5 mm (prima era fragile).
- **Ricarica**: dietro a **sinistra** (scambiata con l'interruttore: i cavi della batteria escono davanti a sinistra). Si usa il connettore femmina della batteria (Ø11 per 21 mm, poi Ø9; lungo 38). Sta in una **culla sagomata** sul connettore (Ø11 per 21 mm, cono di 3 mm, poi Ø9), divisa **sopra-sotto** sull'asse del connettore: la metà di sotto fa parte della schiena, quella di sopra è il **TAPPO** (`stampa\tappo`, 0,5 mm di luce per stringere). 2 viti M3×16 verticali ai lati del connettore, dal basso con la brugola (sopra il tappo restano solo ~18 mm fino al vassoio); dadi nelle sedi sopra il tappo. Verso fuori lo ferma la parete (1 mm, foro Ø8 con svasatura Ø12 da fuori per la spina), verso dentro il cono. Niente più V e fascetta (provini del 17/11). Seconda prova: il connettore stava 1 mm più in alto del foro di uscita. La culla tonda va bene così (provata anche a goccia: peggio); il **foro di uscita (Ø8 e svasatura Ø12) è alzato di 1 mm** rispetto all'asse della culla.
- **ESP32** 52 × 29 mm, USB-C al centro del lato corto, verso il retro; montato su **header femmina** sulla millefori. Nessuna apertura USB sul retro: per riprogrammarlo si toglie la cover, si sfila l'ESP32 dagli header passando di lato alla paratia (anche con il ripiano montato) e lo si collega al PC fuori dal robot. Segnare il verso prima di sfilarlo. In alternativa: aggiornamento via WiFi (ArduinoOTA).
- **IMU**: la sede dei dadi sfondava il retro di 0,5 mm; ora è chiusa dietro (2,3 mm) e la vite è M3×10. Dopo i provini: sedi dei dadi su misura (5,6 mm, fondo a V) perché il dado girava; distanziali ridotti a **mezzi anelli** sopra i fori perché quello in alto a destra premeva su un componente.
- **Guide**: gioco ridotto da 0,25 a 0,15 mm per lato (sul provino il ripiano ballava).

## IMU (13 novembre)

- GY-521: 20,5 × 16 mm, 2 fori Ø3 su un lato lungo (15,5 mm tra i centri), pin sull'altro lato lungo.
- Montaggio: lato lungo orizzontale, fori in alto, pin in basso rivolti verso l'interno del robot (scheda a ~9,5–11 cm sopra l'asse). La faccia verso la parete appoggia su 2 distanziali da 3 mm (mezzi anelli, dal 17/11), così chip e saldature non toccano il blocco. Gli assi si correggono nel software.

## Viso, misure del 12 novembre

- Display GC9A01: Ø40 (non 38), spessore 3,2, modulo lungo 46 → linguetta da 6 mm, alta 23. Fori Ø2 a 19 mm tra i centri, a 5,5 mm dal bordo della linguetta (20,5 mm dal centro del display); fila dei pin a ~1 mm dal bordo (25 mm dal centro); pin saldati, sporgono 8 mm dietro. I connettori dupont passano nella finestra della paratia e stanno tra paratia e batteria.
- HC-SR04: scheda 45 × 20 × 1,6, cilindri Ø16 alti 12 a 26 mm. Prima stampa della paratia (12/11 sera): perni fuori posto e troppo grossi. Fori del sensore misurati: Ø1,7, a 43,5 × 18,5 mm bordo-bordo → centri 41,8 × 16,8. Perni ora Ø1,4 (la stampa li ingrossa di ~0,1 mm). Tra colonnine del sensore e viti della visiera ai lati restano ~4 mm. Finestra del cavo del sensore nella paratia alzata di 3 mm (13/11): i pin ci sbattevano contro. Pin in basso, paralleli alla scheda, 7 mm: il cavo scende davanti alla paratia e passa dietro da una finestra sotto la scheda.
- Visiera: le 4 colonnine ora stanno tutte dentro la sagoma (2 nel collo a ±13,6, 2 ai lati della bocca a ±25,8). Bocca 3 mm più larga per lato e collo da 36 mm; il foro della cover segue la visiera.

## Paratia, visiera e cover: stampa (12 novembre, sera)

- Paratia: i fermi della batteria sono diventati un pezzo a parte (FERMO, avvitato con 2 viti); la paratia si stampa sul dorso senza supporti. Il fermo sta sopra i connettori del display e lascia 4,5 mm liberi verso i cavi della batteria. Gioco della batteria avanti-indietro: 0,6 mm.
- Visiera: la faccia è curva, quindi va stampata a faccia in alto (sulle colonnine), non a faccia in giù: altrimenti i supporti finirebbero sulla parte che si vede.
- Viti della visiera: autofilettanti nelle colonnine (foro 2,8): vanno bene perché la visiera si smonta di rado.
- Cover: resta capovolta (tetto sul piatto). Circa 18 cm² di supporti toccano la curva esterna in alto davanti: da carteggiare. Possibile miglioramento futuro: stamparla appoggiata sul bordo posteriore (esterno pulito, supporti solo dentro), rifacendo le 4 linguette di fissaggio che oggi sporgono 8 mm oltre quel bordo.

## Vassoio, prova del 12 novembre

- Prima stampa: batteria troppo libera (fermi a 96 mm, batteria 94/95), labbri 1 mm troppo alti e stampati nel vuoto, guide alte 15 mm fragili lungo i layer.
- Batteria misurata: 94 mm dietro, 95 mm davanti (lato dei cavi), 60 mm avanti-indietro, spessore max 20 con bordi arrotondati.
- Nuovo vassoio: fermi corti bassi e pieni (6 mm × 3 mm) a 94,4 mm dietro e 95,4 mm davanti; fermo posteriore lungo alto 12 mm; niente labbri (sopra la batteria ci sono ~3 mm fino al tetto).
- Paratia: fermi anteriori della batteria alzati a 16 mm, così toccano la parte piatta della faccia e non i bordi arrotondati. Gioco avanti-indietro della batteria: 0,6 mm in tutto.

## Revisione dell'11 novembre (sera): cosa è cambiato

- **Pavimento**: niente più sedi esagonali sotto (le viti della paratia ora arrivano da sotto); restano solo le 2 sedi esagonali sopra, per i dadi delle viti della cover. Le 4 viti verso la schiena sono state spostate (le teste sotto il pavimento toccavano la parete curva del retro).
- **Schiena**: mensole del pavimento allargate/allungate per le nuove posizioni delle viti; sedi dei dadi delle mensole laterali corrette (prima il dado non arrivava in asse con la vite); 4 fori nel fondo per il cacciavite; mensole del vassoio con spianatura e foro per le viti dal basso.
- **Vassoio**: viti verso la schiena dal basso con torretta esagonale per il dado; orecchie della paratia spostate a ±57; asola per i cavi della batteria davanti a sinistra.
- **Paratia**: piede da 6 mm con sede del dado infilata dal davanti; orecchie allargate a ±62,5.
- **Cover, visiera, slitta**: nessuna modifica (file rigenerati).
- Controllato per ogni vite: raggiungibilità con cacciavite o chiave, posizione del dado in asse, teste libere. Unico problema aperto: le viti della slitta dei moduli.

## Passaggi dei cavi

| Cavi | Percorso |
|---|---|
| Motori ed encoder (6 fili per motore) | escono dagli encoder verso il retro, corrono **sotto il pavimento** fino alla schiena e salgono da un passaggio nel pavimento **dietro a destra** (8 × 21 mm), lungo la schiena, fino alla millefori. Il cavo del motore sinistro attraversa sotto il pavimento. |
| Batteria (2 cavi Ø3 × 22 cm, connettori DC 5,5×2,1 da 10–12 mm) | la batteria va montata **capovolta** (etichetta in basso): i cavi escono dalla faccia lunga davanti, sullo spigolo sinistro, e scendono da un'asola di 14,6 × 14 mm nel vassoio (davanti a sinistra, aperta sul bordo) fino alla millefori di potenza |
| Display | dalla linguetta, attraverso la finestra della paratia, poi dietro la paratia e giù da una tacca nel vassoio |
| HC-SR04 | dalla finestra in basso nella paratia, giù fino alla millefori |
| IMU, interruttore, ricarica | lungo la parete posteriore |

I fermi anteriori della batteria (sulla paratia) sono vicini al centro, per lasciare libero il passaggio del cavo del display.

## Espansioni: guide, ripiano e moduli (16 novembre)

- Su ogni fianco interno della schiena c'è una **guida** orizzontale a 72 mm sopra l'asse, da x = −36 a +22 (58 mm), con un fermo in fondo. Profilo a **cuneo**: sotto piatto, punta alta 8 mm, sopra inclinato a 45° verso la parete, sporge 10 mm. Si stampa senza supporti.
- **RIPIANO** (pezzo a parte, `stampa\ripiano`): piastra da 3 mm con due spalle che abbracciano le guide; scorre da davanti come un cassetto. Dietro lo ferma il fermo delle guide, davanti la paratia (con una linguetta, 0,3 mm di gioco): **nessuna vite**. Non può alzarsi (il cuneo lo spingerebbe verso il centro, ma lo tiene la guida dell'altro lato).
  - Piastra a 61–64 mm sopra l'asse: sotto restano ~14 mm sopra l'ESP32, sopra ~50 mm al centro (attenzione dietro: connettori dell'IMU a 94–98 mm).
  - Griglia di fori Ø3,4 ogni 10 mm per montare schede e sensori con distanziali M3 o fascette.
  - Aperture già previste per i cavi di display, HC-SR04 e batteria.
  - Si monta dopo IMU e batteria, prima del modulo viso.
- **SLITTA** (modello di partenza, per moduli singoli sulla guida): stesso profilo. Problema ancora aperto: la vite verticale di blocco non si raggiunge a robot montato. Con il ripiano, i moduli si possono invece fissare al ripiano.
- Le viti del vassoio (da sotto, x 15) passano attraverso un foro nelle guide.
- Speaker: servirà una griglia di fori nel fianco della schiena davanti allo speaker.

## Ordine di montaggio

1. **Vassoio** nella schiena da davanti, sulle mensole, bordo posteriore sotto il dente. 2 viti dal basso (dal fondo aperto, prima del pavimento).
2. **Modulo pavimento** (al banco): staffe e motori sotto il pavimento (8 viti), dadi della cover nelle sedi sopra il pavimento, millefori infilate nelle guide da davanti, cablaggio. Poi infilalo nella schiena **da davanti** (le piastre delle staffe e i lembi del pavimento scorrono nelle finestre dei fianchi) e avvita le 4 viti dal basso.
3. **Batteria**, capovolta: prima infila i due connettori nell'asola davanti a sinistra del vassoio, poi falla scorrere nel vassoio da davanti, sotto i labbri delle guide. Dietro la blocca il tetto.
4. IMU sulla parete posteriore (2 viti, dal davanti) prima del modulo viso. Connettore di ricarica nella culla, tappo sopra (entra dal davanti, sotto il vassoio), dadi nelle sedi del tappo, 2 viti M3×16 dal basso con la brugola (prima del ripiano).
5. **Modulo viso** (al banco): display e HC-SR04 sulla paratia, poi visiera avvitata da dietro (4 viti).
6. Modulo viso nel robot: piede sul pavimento (2 viti dal basso, dadi infilati dal davanti nel piede), orecchie sul vassoio (2 viti dal davanti). La paratia fa anche da fermo anteriore della batteria e della millefori grande.
7. **Cover**: scorre da davanti, centrata dalla fascia sui fianchi. 2 viti sotto e 2 sui fianchi.

Togliendo solo la cover il robot resta aperto davanti e funzionante: nessun cavo da staccare.

## Controlli fatti

- Tutti i pezzi sono solidi validi e in un pezzo unico. Due schegge della schiena, negli angoli in basso davanti, sono state tolte apposta: bloccherebbero l'uscita del modulo pavimento.
- Nessuna interferenza tra pezzi. Unici contatti con i componenti: i perni di display e HC-SR04 (voluti).
- Montaggio simulato a passi di 3 mm: cover, modulo viso, vassoio e modulo pavimento entrano ed escono senza urti.
- Versione da 150 mm (11 novembre): rifatti tutti i controlli. Pezzi validi, nessuna interferenza (a parte 0,1 mm³ trascurabili tra schiena e cover), montaggio libero per cover, slitta, modulo viso, vassoio, modulo pavimento e le due millefori nelle guide; passaggi dei cavi liberi. Le guide coprono 1,5 mm dei bordi lunghi delle millefori (previsto).

## File di stampa per pezzo (`bot design\stampa`)

Ogni pezzo ha una cartella con il modello **già orientato per la stampa** (`.3mf`) e il **suo profilo di processo** (`processo_<pezzo>.json`). Il profilo del filamento è uno solo per tutti: `stampa\Robot_PLA_filamento.json` (ventola sporgenze 100%, tempo minimo per layer 10 s, ugello 210 °C, primo layer 215 °C).
In tutti i profili: compensazione XY dei fori 0, rallenta su sporgenze e su perimetri arricciati, riempimento gyroid.

| Pezzo | Orientamento | Layer | Pareti | Riempimento | Supporti | Altro |
|---|---|---|---|---|---|---|
| ruota | faccia esterna sul piatto | 0,16 | 4 | 30% | no | 5 strati sopra/sotto |
| staffa | piastra sul piatto | 0,20 | 4 | 40% | no | |
| schiena | capovolta, tetto sul piatto | 0,20 (variabile) | 3 | 15% | ad albero (~14 g), soglia 25°, interfaccia | **senza brim**, colla; parete esterna 80 mm/s, accelerazione 2000; scarf seam; parete precisa; alta 174 mm |
| cover | capovolta, tetto sul piatto | 0,20 | 3 | 15% | ad albero (~9 g) | brim 6 mm |
| visiera | faccia in alto, sulle colonnine | 0,16 | 3 | 40% | ad albero, solo sotto il retro | filamento nero |
| paratia | dorso sul piatto | 0,20 | 3 | 25% | no | |
| fermo | piatto | 0,20 | 4 | 40% | no | si avvita dietro la paratia |
| tappo | capovolto, dadi sul piatto | 0,20 | 4 | 40% | no | chiude il connettore di ricarica |
| pavimento | piatto, guide in su | 0,20 | 4 | **100%** | no | porta motori e staffe; alto 17 mm con le guide |
| vassoio | piatto, guide in su | 0,20 | 3 | 25% | no | |
| slitta | faccia del modulo sul piatto | 0,16 | 4 | 40% | no | |

Uso: importare una volta tutti i `processo_*.json` e il filamento (*File > Importa > Importa configurazioni*); per ogni pezzo aprire il suo `.3mf`, scegliere "Robot - <pezzo>" e salvare il progetto nella stessa cartella. Da lì in poi basta riaprire il progetto.

## Prove sui pezzi stampati (da completare)

**Staffa motore** (provata l'11 novembre con i motori e le viti arrivati): il morsetto con le M3×16 blocca il motore e il gioco nella culla è sparito. Nessuna modifica a culla e morsetto.
- Viti frontali del motore: le viti comprate sono a testa cilindrica (Ø5,5 × 3 mm), non svasate. Provati tre provini sulla piastra da 2,5 mm (file eliminato dopo la prova): sede tonda Ø6,4 × 1,5; foro passante Ø5,8; sede Ø5,8 con fondo da 0,4 mm. Scelto il secondo (2 tacche, foro passante Ø5,8): la testa si stringe direttamente contro la faccia metallica del riduttore (metallo su metallo, non si allenta come su un fondo di PLA) e fa da perno contro la rotazione; lungo l'asse tiene il morsetto. La testa sporge 0,5 mm verso la ruota (gioco residuo ~1 mm). Il terzo (fondo da 0,4 mm) era scartato perché, con la piastra sul piatto, quel fondo andrebbe stampato nel vuoto.
- Viti: M3×6 o M3×8 (i fori del riduttore sono profondi almeno 8 mm). Per evitare che si allentino: una goccia di frenafiletti medio.
- Nuova staffa (11 novembre): script `Motor mount` aggiornato (`FACE_HOLE_DIA` 5,8, niente svasature) e `stampa\staffa\staffa.3mf` rigenerato. Se il primo layer (piede d'elefante) stringe i fori, ripassarli con un cutter.

**Ruota**: la versione giusta è quella da Ø89,5 × **29,3 mm** con gole a soffitto inclinato (file `stampa\ruota\ruota.3mf`, modello `modelli\ruota_89.step`, script Wheel in Fusion).
- Prima stampa (9 novembre, versione da 23,9 mm): gole fallite. Il "soffitto" orizzontale di ogni gola (3,5 mm di sporgenza) veniva stampato nel vuoto, quindi colava e si arricciava. Corretto con un profilo stampabile: pavimento piatto, 3,6 mm di parete verticale, soffitto a 45°. Apertura 7,1 mm, ruota da 29,3 mm. Il robot con le ruote passa da ~213 a ~224 mm di larghezza.
- Provino delle gole a 45° (9 novembre): profilo corretto, soffitti puliti su metà del pezzo e leggermente cedevoli sull'altra metà, quella dalla parte opposta alla ventola dell'A1 mini, che soffia da un solo lato. Il provino è il caso peggiore: pezzo piccolo, layer rapidi, poco raffreddamento. Per la ruota intera: ventola per sporgenze 100%, tempo minimo per layer ~8–10 s, ugello 5–10 °C più freddo, "rallenta per perimetri arricciati" attivo. Se cede ancora: soffitto a 55° (ruota ~2 mm più larga per lato).
- Impostazioni di stampa: vedi la sezione **"File di stampa per pezzo"** qui sotto.
- Impostazioni di stampa (9 novembre): PLA, faccia esterna sul piatto, layer 0,16 mm, 4 pareti, riempimento 30% gyroid, 5 strati sopra e sotto, niente supporti né brim, compensazione XY dei fori 0, piede d'elefante di default.
- Da annotare dopo la prova: come entra l'albero nel foro a D (gioco attuale nel modello: 0,15 mm), come si siede l'O-ring nella gola, se la vite M3×10 blocca bene l'albero.

**Misure da prendere sui motori**: fori filettati del riduttore profondi almeno 8 mm (11 novembre). Albero: sporge ~10 mm dalla faccia del riduttore (11 novembre, più corto dei 12 del catalogo). Con piastra 2,5 e ~1,5 mm tra piastra e ruota, nella ruota entrano ~6 mm, di cui ~4,5 sul piatto della D (i primi 1,5 mm del foro sono lo scarico): sufficiente, provato a mano. La vite radiale (a 4,5 mm dalla faccia lato motore) cade sull'albero. Montare la ruota a ~1–1,5 mm dalla piastra, senza toccare le teste delle viti (sporgono 0,5 mm). Ancora da misurare: piatto della D e diametro, lunghezze di riduttore, motore ed encoder, uscita e ingombro del cavo dell'encoder.

## Da sapere

- La riga verticale al centro del frontale è il bordo di chiusura della superficie: nella stampa non si vede. In Fusion si nasconde con Visualizza > Stile di visualizzazione > Ombreggiato.
- I fianchi della schiena negli angoli in basso davanti sono aperti (dietro la ruota): servono a far passare le staffe.
- I file `.3mf` di prima dell'11 novembre avevano la maglia aperta (Bambu: "open edges"): nello slicing un foro della staffa spariva. Rigenerati tutti chiusi (tranne il provino delle gole, già stampato). Se Bambu segnala di nuovo "open edges", non stampare: avvisare.

## Visiera e display (17 novembre)

- Controllo aggiunto: montaggio della cover con la visiera e del modulo viso completo (paratia + visiera): liberi.
- Il bordo alto del display toccava di 0,3 mm la faccia interna inclinata della visiera: aggiunto un incavo Ø40,8 profondo 0,4 mm (la visiera lì è spessa 3,3 mm). Ristampa facoltativa se il display monta senza forzare.

## Schiena: stampa "bella" (18 novembre)

- **Tetto spianato 2 mm in più** (piano a 146,9 sopra l'asse, robot 2 mm più basso; tetto spesso 2 mm, batteria 1,9 mm sotto). Prima il bordo del tetto partiva dal piatto quasi orizzontale (70–76°): supporti sulla parte in vista e brim saldato sotto il bordo. Ora parte a ~58°: niente supporti né brim lì.
- **Niente brim**: appoggio 8000 mm² sul piatto, basta la colla.
- Profilo `Robot - schiena`: parete esterna 80 mm/s, accelerazione 2000 (pezzo alto su piatto mobile), cucitura allineata con scarf seam, parete precisa, supporti ad albero solo sotto 25° dall'orizzontale, con interfaccia e 0,2 mm di distacco.
- **Altezza layer variabile** (non si salva nel profilo, va fatta nel progetto): tasto destro sul pezzo → *Altezza layer variabile* → *Adattivo* (qualità ~0,7) → *Leviga*. Layer fini nei primi 20 mm (bordo del tetto) e negli ultimi 20 (curva in basso dietro), 0,20 sulle pareti dritte; non oltre 0,20.
- Cucitura: se finisce sulla faccia in vista, dipingerla (*Pittura cucitura*) sui bordi davanti, dove si unisce la cover.
- Cover (18/11): la linguetta sottile con la fessura per il dado è sostituita da un tassello pieno (5,2 × 9 mm) con sede esagonale aperta verso l'interno; la vite laterale tira il dado contro il tassello, senza gioco.
