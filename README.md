# Yamaha FZ8 Dynamics Telemetry Computer 🏍️📊
### ESP32-C3 Mini | GY-89 (10-DOF IMU) | 1.6" Transflective LCD SPI (130x130 SSD1283A)

Un computer di bordo dedicato all'analisi della dinamica motociclistica per **Yamaha FZ8** (779cc). Il sistema misura in tempo reale:
- **Angolo di piega effettivo (Lean Angle)** con filtro cinematico selettivo anti-decadimento in curva.
- **Forze G Longitudinali**: staccata/frenata violenta (G negativi) e accelerazione in uscita di curva (G positivi).
- **Diagramma G-G (Cerchio di Attrito)**: combinazione tra accelerazione laterale di percorrenza e accelerazione longitudinale.
- **Altimetro Barometrico & Termometro**: quota altimetrica e dislivello da BMP180.
- **Memoria Record di Sessione**: record piega massima SX/DX, G max di frenata e accelerazione.
- **Web Dashboard Wi-Fi On-Demand**: attivabile a moto ferma per visualizzare la telemetria da qualsiasi smartphone su `http://192.168.4.1`.

---

## 1. Schema di Cablaggio (Pinout ESP32-C3)

| Dispositivo | Pin Modulo | Pin ESP32-C3 | Note e Funzione |
|---|---|---|---|
| **GY-89** | VCC | **3.3V** (o 5V) | Alimentazione (il GY-89 integra regolatore LDO) |
| **GY-89** | GND | **GND** | Massa comune |
| **GY-89** | SCL | **GPIO 5** | Bus I2C Clock (400 kHz Fast-Mode) |
| **GY-89** | SDA | **GPIO 4** | Bus I2C Data |
| **LCD 1.6"** | VCC | **3.3V** | Alimentazione logica |
| **LCD 1.6"** | GND | **GND** | Massa comune |
| **LCD 1.6"** | CS | **GPIO 10** | SPI Chip Select |
| **LCD 1.6"** | RST | **GPIO 3** | Display Reset |
| **LCD 1.6"** | A0 / DC | **GPIO 2** | Data / Command |
| **LCD 1.6"** | SDA / MOSI | **GPIO 7** | SPI Master Out Slave In |
| **LCD 1.6"** | SCK / SCLK | **GPIO 6** | SPI Clock |
| **LCD 1.6"** | LED / BL | **GPIO 1** (o 3.3V) | Retroilluminazione display |
| **Pulsante** | Switch | **GPIO 9** | Pulsante BOOT integrato a bordo (o esterno a massa) |
| **LED Stato** | LED onboard | **GPIO 8** | Indicatore di stato e modalità Wi-Fi |

> [!WARNING]
> **Attenzione a GPIO 0**: GPIO 0 è uno *strapping pin* sull'ESP32-C3 e deve essere a livello logico ALTO all'avvio per caricare il firmware da Flash (se tenuto a massa all'accensione entra in bootloader UART). Per questo motivo il firmware usa di default **GPIO 9** (che ha già pull-up interno ed è il tasto BOOT della scheda).

---

## 2. Alimentazione Sicura su Yamaha FZ8 (Protezione Elettrica 12V)

L'impianto elettrico a 12V di una moto come l'FZ8 è esposto a violenti transitori di commutazione (load dump dell'alternatore, spike induttivi delle 4 bobine di accensione).

### Punti di Prelievo Consigliati (12V Sottochiave)
1. **Presa Ausiliaria Ausiliaria Yamaha 2-pin** (sotto la cover del serbatoio/fianchetto).
2. **Luce di posizione anteriore** o **luce targa posteriore**.

### Circuito di Alimentazione Consigliato
```
12V Sottochiave FZ8 ---> [Diodo Schottky 1N5819 / SS14] ---> [Modulo Step-Down Buck 12V->5V] ---> ESP32-C3 (5V Pin)
                                    |                                     |
                         [TVS SMAJ24A a GND]                    [Capacità 100uF + 100nF]
```
- **Diodo Schottky**: protegge da inversioni di polarità accidentali.
- **Diodo TVS SMAJ24A (o SMBJ18A)**: sopprime istantaneamente qualsiasi picco di tensione superiore a 24V.
- **Modulo Step-Down (es. MP1584EN)**: converte i 12-14.4V della moto a 5.0V stabili con alta efficienza e ridotto calore.

---

## 3. Dinamica di Piega in Moto: Il Filtro Cinematico Selettivo

### Il Problema del Filtro Madgwick Tradizionale in Curva
In una curva percorsa correttamente senza derapata, la moto è in equilibrio dinamico: la risultante tra gravità ($g$) e accelerazione centripeta ($v^2/R$) punta quasi esattamente **lungo l'asse Z della moto (verso le pedane)**.
Se si usasse un filtro IMU convenzionale, l'accelerometro scambierebbe la risultante centripeta per la gravità statica, **trascinando erroneamente l'angolo di piega stimato verso 0° durante curvoni lunghi o rotonde**.

### La Soluzione del Firmware FZ8
Il nostro filtro implementa un **Gate Cinematico**:
1. **In Curva** ($|\omega_z| > 3^\circ/\text{s}$ oppure $|a_{\text{tot}}| > 1.08g$ oppure $|\omega_x| > 3.5^\circ/\text{s}$):
   - La correzione gravitazionale dell'accelerometro sull'asse di rollio viene **immediatamente disabilitata (peso 0%)**.
   - L'angolo di piega viene sostenuto dall'integrazione ad alta frequenza (100 Hz) del giroscopio a 3 assi (L3GD20), mantenendo fedelmente i gradi di inclinazione per tutta la durata della curva.
2. **Nei Tratti a Moto Dritta** ($|a_{\text{tot}}| \approx 1g$, $|\omega_z| < 3^\circ/\text{s}$, $|a_y| < 0.08g$ per $> 250\text{ms}$):
   - Viene riabilitata una correzione complementare dolce per azzerare la deriva e tarare il bias del giroscopio.

---

## 4. Modalità di Funzionamento & Comandi Pulsante

Il firmware gestisce un unico pulsante (GPIO 9, tasto BOOT della board) con logica temporale:

- **Pressione Breve (< 800 ms)**: Cambio schermata display:
  1. `DYNAMICS`: Indicatore ad arco, gradi digitali istantanei, record L/R e G-force.
  2. `G-G DIAGRAM`: Cerchio di attrito 2D (accelerazione laterale vs frenata/accelerazione).
  3. `SESSION STATS`: Riepilogo staccata massima, accelerazione massima, dislivello metri, tempo sessione.
- **Pressione Media (0.8s - 2.5s)**: **Zero Tare**:
  - Calibra l'inclinazione attuale come punto zero neutro (utile con moto sul cavalletto o prima di partire).
- **Pressione Prolungata (> 2.5s)**: **Toggle Wi-Fi Access Point**:
  - Il Wi-Fi resta **SPENTO durante la guida** per risparmiare energia, abbassare le temperature della CPU e annullare qualsiasi disturbo RF.
  - Tenendo premuto il tasto a moto ferma, l'ESP32-C3 avvia l'Access Point `FZ8-Telemetry` (password: `yamaha-fz8`) e il LED di stato si illumina.
  - Collegandosi con lo smartphone all'indirizzo **`http://192.168.4.1`**, si apre la dashboard completa con grafici live, record e tasti di reset.

---

## 5. Compilazione & Upload con PlatformIO

1. Apri la cartella del progetto in VSCode / PlatformIO.
2. Collega l'ESP32-C3 Mini via USB.
3. Compila e carica:
```bash
pio run -t upload
```
4. Apri il monitor seriale a 115200 baud per verificare i log diagnostici:
```bash
pio device monitor -b 115200
```
