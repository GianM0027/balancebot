# Requisiti strutturali del robot self-balancing

Versione dell'8 novembre 2026. Il documento elenca **cosa** la struttura deve garantire per ogni componente, non **come** realizzarlo: le soluzioni costruttive sono libere.

## Come leggere il documento

Ogni dato porta un'etichetta di affidabilità:

- **[V]** = dato del venditore (pagina di acquisto).
- **[M]** = misurato da Gian Mario.
- **[S]** = misura standard della famiglia di componenti, da scheda tecnica di un prodotto equivalente. Da verificare sul pezzo vero.
- **[S]** con stima = ricavata da un'immagine. Incertezza indicata accanto.
- **[?]** = non noto. Va misurato all'arrivo: lasciare margine o una soluzione regolabile.

Convenzione degli assi del robot:
- **x** = avanti, nella direzione di marcia;
- **y** = asse delle ruote, verso sinistra;
- **z** = verso l'alto, con il robot in equilibrio.

---

## 1. Vincoli globali

### 1.1 Massa e baricentro
- La simulazione usa: massa del corpo 0,69 kg, baricentro **9,5 cm sopra l'asse delle ruote**, massa delle due ruote 80 g. Il progetto può discostarsi, ma i valori reali vanno poi misurati e aggiornati in `balancebot/params.py`.
- Baricentro **alto**, con la batteria in cima: il robot cade più lentamente ed è più facile da controllare. Decisione già presa.
- Baricentro **sul piano verticale che contiene l'asse delle ruote**, lungo x. Se è spostato in avanti o indietro, il robot sta in equilibrio inclinato. Il controllo lo compensa, ma conviene evitarlo.
- Masse **simmetriche** tra sinistra e destra, lungo y.
- Componenti pesanti, da considerare per il baricentro:
  - batteria 250 g [V];
  - motori circa 90–100 g ciascuno [S];
  - display circa 10 g [V];
  - driver circa 20 g [V];
  - step-down circa 4 g (20 g per la confezione da 5) [V];
  - il resto è trascurabile.

### 1.2 Rigidità
- Telaio rigido: flessioni e vibrazioni entrano direttamente nella misura dell'IMU.
- Niente parti che possano ballare, come cavi liberi o schede fissate da un solo lato.

### 1.3 Ruote e terreno
- Raggio ruota circa **46 mm** (vedi §2.11), quindi l'asse sta a circa 46 mm da terra.
- Ogni parte sotto l'asse deve restare **dentro il raggio della ruota**, con un margine, in tutte le inclinazioni di lavoro.
- **Inclinazione da cui il robot deve poter recuperare**: fino a 34–40° (limite dei motori, da simulazione). In questo intervallo nessuna parte del telaio deve toccare terra.
- **Caduta**: oltre l'inclinazione di recupero il robot cade. Conviene che a toccare terra per prime siano parti robuste e sacrificabili, e non display, sensore a ultrasuoni o interruttore.

### 1.4 Accessibilità a robot montato
Devono restare raggiungibili senza smontare:
1. **Porta USB-C dell'ESP32**: programmazione e monitor seriale (§2.1).
2. **Interruttore** (§2.7).
3. **Ricarica della batteria**: ingresso jack femmina raggiungibile, oppure batteria estraibile (§2.6).
4. **Vite di regolazione dello step-down**: almeno al primo montaggio (§2.4).

Inoltre, i moduli innestati sulla millefori devono poter essere sostituiti.

### 1.5 Elementi di fissaggio disponibili
- Kit viti M3 a brugola [V]: 50 × M3×6, 40 × M3×8, 50 × M3×10, 40 × M3×12, 50 × M3×16, 50 × M3×20, 220 dadi M3.
  - Dado M3 standard: 5,5 mm tra i lati piatti, circa 2,4 mm di spessore [S].
  - **Non ci sono grani** (viti senza testa).
- Kit millefori [V]: 12 colonnine esagonali in nylon **M2** × 10 mm + 6 mm di filetto maschio.
  - Attenzione: sono M2, non M3.
  - Il kit non include viti né dadi M2.
- **Non disponibili**: fascette e colla a caldo. Batteria, schede e cavi vanno trattenuti da incastri, sedi o viti.

---

## 2. Componenti

### 2.1 ESP32 (scheda di sviluppo)
- **Dimensioni**: circa **53 × 30 mm** [M], misura approssimativa. Altezza tipica della scheda con modulo circa 5 mm, pin esclusi [S].
- **Distanza tra le file di pin**: circa **25 mm** [M], cioè 10 passi da 2,54 mm (25,4 mm).
  - Sulla millefori i due header femmina stanno a 10 fori di distanza. La scheda copre i 9 fori in mezzo, che restano usabili solo dal lato saldature.
- **Da verificare** [?]: modello esatto e presenza di fori di fissaggio.
- **Fissaggio previsto**: innestata su pin header femmina saldati sulla millefori, non saldata.
  - Altezza totale sopra la millefori con gli header: circa 13–15 mm [S].
- **Requisiti**:
  - Porta **USB-C raggiungibile** a robot montato (§1.4). Serve lo spazio per inserire il connettore e il cavo: circa 25–30 mm liberi davanti alla porta [S].
  - Lontano da parti metalliche che schermino l'antenna Wi-Fi, cioè l'estremità del modulo opposta all'USB. Il Wi-Fi è previsto solo come opzione per la telemetria.
- **Dipendenze**: cavi dupont da 20 cm massimo verso IMU, display, sensore a ultrasuoni ed encoder. Ogni componente collegato deve stare a **meno di circa 15–18 cm di percorso cavo** dall'ESP32.

### 2.2 IMU GY-521 (MPU-6050)
- **Dimensioni**: circa **21 × 16 mm** [S] (20,5 × 15,5 secondo altre schede tecniche), spessore circa 3 mm più i pin.
  - Pin header da 8 pin, passo 2,54 mm, lungo un lato lungo.
- **Fori**: 2 fori da circa **3 mm** (per M3) [S], interasse circa 15,5 mm [S]. Verificabili subito sul pezzo che hai.
- **Requisiti, tutti importanti**:
  - **Fissaggio rigido** al telaio, senza gommini, biadesivo spugnoso o giochi. Il gioco aggiunge oscillazioni alla misura.
  - **Assi allineati** con quelli del robot: un asse della scheda parallelo all'asse delle ruote (y), un altro verticale (z). Gli assi sono stampati sulla scheda.
  - **Sul piano di simmetria** sinistra/destra del robot, o vicino.
  - **Altezza**: la simulazione la pone **10 cm sopra l'asse delle ruote**. Altezze diverse vanno bene, ma il valore reale va riportato nella simulazione.
  - Lontano dai motori, per limitare le vibrazioni.
- **Dipendenze**: cavo I2C verso l'ESP32 corto, possibilmente sotto i 15 cm.

### 2.3 Driver motori TB6612FNG (DollaTek)
- **Dimensioni**: circa **20,3 × 20,3 mm** [S], dalla scheda SparkFun equivalente con lo stesso schema di pin. Il venditore non le indica.
  - Due file da 8 pin, passo 2,54 mm. Ingressi da un lato, uscite e alimentazione dall'altro [V].
  - **Nessun foro di fissaggio** [V, da foto].
- **Fissaggio previsto**: innestato su header femmina sulla millefori.
- **Requisiti**:
  - Lato uscite (AO1/AO2, BO1/BO2, VM) orientato verso i motori e la linea di potenza.
  - Lato ingressi orientato verso l'ESP32.
  - Un po' d'aria sopra il chip. Alle correnti previste scalda poco, ma non va chiuso a contatto con la plastica.
- **Dipendenze**: condensatore da 100 µF il più vicino possibile ai pin VM/GND (§2.5).

### 2.4 Step-down MP1584 (ARCELI)
- **Dimensioni**: **22 × 17 × 4 mm** [V]. Nessun foro di fissaggio [S].
  - Piazzole di ingresso e uscita sui due lati corti [S].
- **Requisiti**:
  - **Vite del trimmer raggiungibile** con un cacciavite, almeno al primo montaggio: va regolato a 5 V prima di collegare l'ESP32.
  - Può stare sulla millefori, saldato con spezzoni di filo o pin, oppure in una sede propria.
- **Dipendenze**: vicino all'ESP32, con un cavo corto da 5 V.

### 2.5 Condensatore elettrolitico 100 µF, 50 V
- **Dimensioni**: **Ø 6 mm × 12 mm** di corpo [M], più due reofori.
- **Requisiti**: entro circa 1–2 cm dai pin VM e GND del TB6612. Può essere montato in piedi o sdraiato.
- **Polarizzato**: la striscia indica il negativo. Nessun vincolo strutturale, ma va lasciato spazio per montarlo nel verso giusto.

### 2.6 Batteria AFTERTECH 12 V, 4200 mAh
- **Dimensioni**: **95 × 60 × 20 mm**, **250 g** [V].
- **Cavi** [V, da foto]: due cavi volanti escono dallo **stesso lato**.
  - uscita: jack **maschio** 5,5 × 2,1 mm;
  - ricarica: jack **femmina** 5,5 × 2,1 mm.
  - **Lato esatto di uscita e lunghezza dei cavi** [?]: non indicati. Dalle foto sembrano circa 15–20 cm, ma è una stima grezza. Lasciare margine per l'uscita dei cavi e per una sede adattabile.
- **Requisiti**:
  - È il componente più pesante. Va **in alto**, decisione già presa, e deve essere **bloccato in tutte le direzioni**: in caso di caduta subisce il colpo maggiore.
  - **Nessuna vite passante** né spigoli vivi contro il pacco, per evitare di forarlo. Niente parti metalliche che possano cortocircuitarlo.
  - **Ricarica possibile**: ingresso femmina raggiungibile, oppure batteria estraibile senza attrezzi (§1.4).
  - La batteria va sul piano di simmetria; il suo baricentro sopra l'asse delle ruote lungo x (§1.1).
- **Dipendenze**: il jack di uscita si collega al connettore jack femmina a morsetti (§2.8), che deve stare a portata del cavo.

### 2.7 Interruttore a bilanciere KCD1
- **Foro a scatto**: circa **12 × 19 mm** [V]. Il venditore indica una tolleranza da 12 × 18,5 a 13,5 × 20 mm.
- **Cornice frontale**: circa **21 × 15 mm** [V].
- **Profondità dietro il pannello**, contatti inclusi: circa **18 mm** [V]. Contatti a lamella da 4,8 mm, da saldare o con faston.
- **Spessore della parete** per l'aggancio a scatto [?]: non indicato. Le alette di questo tipo di interruttore sono pensate per pannelli sottili, circa 1–2 mm. Conviene prevedere una parete sottile intorno al foro.
- **Requisiti**:
  - Raggiungibile dall'esterno in ogni momento, anche col robot a terra, per spegnere subito in caso di problemi.
  - Azionabile senza far cadere il robot, e lontano dalle ruote.
  - In una posizione dove non si urti in caso di caduta, per non spegnerlo involontariamente.

### 2.8 Connettori jack DC 5,5 × 2,1 mm con morsetti a vite (YIXISI)
- **Dimensioni del corpo** [?]: il venditore indica solo lo standard del connettore (5,5 mm esterno, 2,1 mm interno). Lunghezza e diametro del corpo sono da misurare all'arrivo.
- **Uso**: un connettore **femmina** riceve il jack maschio della batteria e porta i fili al coassiale.
- **Requisiti**:
  - Il collegamento batteria–connettore va **staccato** per ricaricare o estrarre la batteria, quindi deve essere raggiungibile.
  - Il connettore non deve restare libero di sbattere: serve una sede, o un vincolo che lo trattenga e scarichi lo strappo sui morsetti.

### 2.9 Display rotondo GC9A01 1,28" (ARCELI)
- **Dimensioni del modulo**: **38 × 45,5 × 3,2 mm** [V]. È formato da un cerchio Ø 38 mm più una linguetta con i pin, larga 23 mm [V, da immagine].
- **Area visibile**: **Ø 32,4 mm** [V].
- **Pin**: 7 pin in fila sulla linguetta (RST, CS, DC, SDA, SCL, GND, VCC) [V, da immagine]. Uscita verso il retro.
  - Non c'è un pin di retroilluminazione, quindi il display resta sempre acceso.
  - **Dietro la linguetta** servono circa **20–25 mm** liberi, per i pin e i connettori dupont femmina [S]. In alternativa i cavi si possono saldare direttamente, oppure piegare i pin.
- **Fori di fissaggio**: 2 fori sulla linguetta, uno sopra e uno sotto la fila di pin [V, da immagine].
  - Stima dall'immagine: **interasse circa 20 mm**, con un'incertezza di ±2 mm; centri a circa 2–3 mm dal bordo esterno della linguetta; diametro circa 2–3 mm.
  - **Da verificare all'arrivo** prima di stampare la sede definitiva.
  - Il cerchio non ha fori: il lato opposto alla linguetta va sostenuto o trattenuto in altro modo.
- **Requisiti**:
  - Sul **frontale** (lato +x): è la "faccia" del robot.
  - Superficie del vetro **protetta dagli urti** in caduta, ad esempio incassata o protetta da un bordo. Va comunque lasciata libera l'intera area visibile.
  - Il vetro non deve essere premuto: lo schermo è fragile.
- **Dipendenze**: 5 segnali più l'alimentazione verso l'ESP32, entro la lunghezza dei dupont (§2.1).

### 2.10 Motori JGA25-370 con riduttore ed encoder, 280 rpm
Tutte le misure seguenti sono **[S]**, dalla famiglia JGA25-370. La pagina del venditore non era leggibile. Da verificare all'arrivo.
- **Corpo**: Ø circa 25 mm (motore 24,4 mm).
  - Lunghezza: motore circa 30,8 mm + riduttore circa 19 mm [V, dal venditore] + encoder posteriore circa 8,7 mm, cioè **circa 58–59 mm**.
  - Dietro l'encoder servono altri circa 10–15 mm per il connettore e la piega del cavo.
- **Frontale**:
  - risalto di centraggio **Ø 7 mm × 2,5 mm**, quindi la piastra di montaggio deve avere un foro di almeno 7 mm, oppure una sede per il risalto;
  - **due fori filettati M3 a 17 mm** di interasse, **profondi 4,5 mm**. La vite non deve entrare più di circa 4 mm, altrimenti tocca gli ingranaggi.
  - Esempio con le viti del kit: piastra da 4 mm più vite M3×8.
- **Albero**: **Ø 4 mm a D**, con il lato piatto a 3,5 mm e lungo 8 mm. Sporge circa 12 mm dal frontale (risalto compreso), circa 9,6 mm oltre il risalto.
- **Staffe di fissaggio** [?]: non è noto se siano incluse.
- **Massa**: circa 90–100 g ciascuno [S].
- **Requisiti**:
  - I due alberi devono essere **coassiali**, cioè sullo stesso asse y, e paralleli al terreno.
  - **Fissaggio rigido**: un motore che si muove anche di poco rovina il controllo.
  - **Carreggiata** (distanza tra le ruote): libera. Più larga dà più stabilità in rotazione sul posto e laterale, ma rende il robot più ingombrante.
  - Il cavo encoder a 6 fili esce dal retro: va previsto il percorso fino all'ESP32 e al driver.

### 2.11 Ruote: cerchio stampato con O-ring Othmro 90 × 80 × 5 mm
- **O-ring**: diametro esterno 90 mm, interno 80 mm, sezione 5 mm, NBR durezza 70A [V].
- **Gola del cerchio**: circa **Ø 82,5 mm** sul fondo, cioè 3–5% più grande del diametro interno così l'O-ring non slitta. Decisione già presa.
- **Diametro della ruota montata**: circa **92 mm**, raggio circa **46 mm**. Coincide con la simulazione.
- **Mozzo**: foro a D per l'albero da 4 mm (piatto a 3,5 mm). Più bloccaggio contro il piatto: il kit non ha grani, quindi serve una soluzione con le viti M3 a brugola e i dadi disponibili.
  - Profondità utile dell'albero circa 9,6 mm oltre il risalto.
- **Massa delle due ruote**: la simulazione usa 80 g totali. Se la differenza è grande, il valore reale va aggiornato.
- **Requisiti**: luce di qualche millimetro tra la ruota e qualsiasi parte del telaio o dei cavi, anche considerando la deformazione del telaio e la gomma.

### 2.12 Millefori e accessori (kit VooGenzek)
- **Formati disponibili** [V], 3 pezzi per formato: **20 × 80**, **30 × 70**, **40 × 60** e **50 × 70 mm**.
  - Passo 2,54 mm, fori da 1 mm, doppia faccia, FR4.
  - Spessore della scheda circa 1,6 mm [S].
- **Fori di fissaggio** [?]: non dichiarati. Le colonnine del kit sono M2, quindi probabilmente i fori agli angoli sono per M2. Da verificare.
- **Morsetti a vite**: 2 e 3 poli, passo 5,08 mm, 16 A [V]. Sono quelli per i cavi di potenza.
- **Header femmina e maschio**: 40 pin, passo 2,54 mm [V].
- **Requisiti comuni a tutte le configurazioni**:
  - Millefori fissata rigidamente, senza flessioni.
  - Sotto la scheda serve un piccolo spazio libero per le saldature, circa 3 mm [S].
  - **Estraibile** per le riparazioni.
  - I morsetti di potenza devono essere raggiungibili con un cacciavite, e i cavi devono potervi entrare.

**Configurazioni possibili** (spazio indicativo, da verificare con il layout reale):

| Opzione | Schede | Contenuto | Pro | Contro |
|---|---|---|---|---|
| A | 1 × 50×70 | ESP32 + TB6612 + MP1584 + condensatore + morsetti | un solo pezzo da fissare, meno cavi | molto stretta: l'ESP32 (53×30, file di pin a 25,4 mm) occupa quasi tutta la lunghezza, e per il resto restano circa 20 mm (7–8 colonne di fori) |
| B | 50×70 + 20×80 (o 30×70) | prima scheda: ESP32 + TB6612; seconda: potenza (MP1584, condensatore, morsetti batteria) | potenza e segnali separati, layout comodo | due schede da fissare e cavi tra le due |
| C | 50×70 + una piccola per ogni sottosistema | come B, con IMU e sensore su schede proprie | massima libertà di posizione | più cavi e più fissaggi |

Nota per tutte le opzioni: l'**IMU può stare sulla millefori solo se la millefori stessa è montata rigida e nella posizione giusta per l'IMU** (§2.2). Altrimenti l'IMU va su un supporto proprio.

### 2.13 Cavi
- **Potenza**: cavo coassiale recuperato da un alimentatore di portatile.
  - Diametro esterno circa 4–5 mm [?], da misurare.
  - Relativamente rigido: servono raggi di curvatura ampi e percorsi semplici.
  - Percorso: batteria → interruttore → step-down e VM del driver.
- **Segnali**: dupont fino a **20 cm** [M]. Con le vibrazioni si sfilano: dove possibile conviene un vincolo che li trattenga, ad esempio una sede o un passacavo.
- **Motori**: cavo a 6 fili in dotazione, lunghezza [?].
- **Requisiti**:
  - Nessun cavo vicino alle ruote.
  - Passacavi o canaline stampate, perché non ci sono fascette.

---

## 3. Componente previsto ma non ancora acquistato

### 3.1 Sensore a ultrasuoni HC-SR04P (o equivalente a 3,3 V)
- **Dimensioni**: **45 × 20 × 15 mm** [S]. Due trasduttori **Ø 16 mm** sporgono di circa 12 mm dalla scheda [S].
  - 4 pin sul bordo lungo, verso il basso o il retro.
  - 4 fori agli angoli, diametro circa 2 mm [S].
- **Requisiti**:
  - Sul **frontale**, rivolto in avanti (+x) e orizzontale col robot in equilibrio.
  - **Cono di misura libero**, apertura circa 15–30° [S]: nessuna parte del robot, ruote comprese, davanti ai trasduttori.
  - Il robot oscilla di qualche grado mentre sta in equilibrio. Più il sensore è basso, prima il cono colpisce il pavimento e misura il pavimento invece dell'ostacolo. Va tenuto il più in alto possibile, o con una leggera inclinazione verso l'alto.
  - Protetto in caduta: i trasduttori sono la parte più esposta.
- **Dipendenze**: cavo verso l'ESP32 entro la lunghezza dei dupont. Display e sensore condividono il frontale: va previsto lo spazio per entrambi.

---

## 4. Riepilogo

| Componente | Dimensioni (mm) | Massa | Fissaggio | Vincolo principale |
|---|---|---|---|---|
| ESP32 | ~53 × 30, file di pin a 25,4 [M] | ~10 g | header femmina su millefori | USB-C raggiungibile |
| IMU GY-521 | ~21 × 16 [S] | ~2 g | 2 × M3, rigido | rigida, assi allineati, ~10 cm sopra l'asse |
| TB6612 | ~20,3 × 20,3 [S] | ~20 g [V] | header femmina | condensatore vicino; uscite verso i motori |
| MP1584 | 22 × 17 × 4 [V] | ~4 g | millefori o sede | trimmer raggiungibile |
| Condensatore | Ø6 × 12 [M] | — | millefori | entro 1–2 cm dal VM del TB6612 |
| Batteria | 95 × 60 × 20 [V] | 250 g | sede bloccata, senza viti | in alto, simmetrica, ricaricabile |
| Interruttore | foro 12 × 19 [V] | — | a scatto | raggiungibile, protetto dagli urti |
| Jack DC | [?] | — | sede o vincolo | staccabile per la ricarica |
| Display | 38 × 45,5 × 3,2 [V] | 10 g | 2 fori (~20 mm, stima) + appoggio | frontale, protetto; 20–25 mm dietro la linguetta |
| Motori (×2) | Ø25 × ~59 + albero 12 [S] | ~90–100 g | 2 × M3 a 17 mm, frontale | coassiali e rigidi |
| Ruote (×2) | Ø ~92 | ~40 g ciascuna | mozzo a D | luce dal telaio |
| HC-SR04P (previsto) | 45 × 20 × 15 [S] | ~9 g | 4 fori Ø2 | frontale, cono libero |

---

## 5. Da misurare all'arrivo dei componenti

1. **Batteria**: lato di uscita e lunghezza dei due cavi.
2. **Motori**: lunghezza totale, interasse dei fori e profondità; staffe incluse o no; lunghezza del cavo encoder.
3. **Display**: interasse e diametro dei fori, sporgenza dei pin dietro la scheda.
4. **Jack DC**: lunghezza e diametro del corpo.
5. **Interruttore**: profondità effettiva e spessore di pannello accettato dalle alette.
6. **ESP32**: fori di fissaggio e modello esatto.
7. **Millefori**: posizione e diametro dei fori agli angoli.
8. **Coassiale**: diametro esterno e rigidità.

Prima di stampare le sedi definitive, conviene stampare **piastrine di prova** con i soli fori e incastri, e provarle sui componenti.
