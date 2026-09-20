# Walkthrough: Computer di Telemetria & Dinamica per Yamaha FZ8

Il progetto è stato creato con successo, testato e salvato nel nuovo repository Git locale:
📁 **Percorso Progetto Git**: `C:\Users\Edoardo\Documents\GitHub\FZ8_Dynamics_Telemetry`

---

## 1. Risultati della Compilazione (PlatformIO)

Il progetto è stato compilato con successo su architettura **ESP32-C3 RISC-V**:
- **Toolchain**: `toolchain-riscv32-esp@14.2.0`
- **Framework**: Arduino ESP32 (`esp32-c3-devkitm-1`)
- **RAM Utilizzata**: `37,396 byte / 327,680 byte` (**11.4%** - ampio margine libero per FreeRTOS e buffer grafici)
- **Flash Utilizzata**: `1,058,922 byte / 1,310,720 byte` (**80.8%**)
- **Esito Build**: `[SUCCESS] Took 121.04 seconds` -> File `firmware.bin` generato e pronto per il flash.

---

## 2. Struttura del Repository

```
C:\Users\Edoardo\Documents\GitHub\FZ8_Dynamics_Telemetry\
├── .gitignore
├── platformio.ini              # Configurazione PlatformIO, USB CDC e flag compilatore
├── README.md                   # Documentazione completa, schema elettrico e protezioni 12V
├── include/
│   ├── Config.h                # Pinout ESP32-C3, frequenze di campionamento e soglie
│   ├── SensorsGY89.h           # Driver I2C 400kHz per LSM303D, L3GD20 e BMP180
│   ├── MotorcycleFilter.h      # Filtro cinematico selettivo (anti-deriva rollio in piega)
│   ├── DisplayUI.h             # Rendering grafico parziale ad alto contrasto per SSD1283A
│   └── WebDashboard.h          # Server Web SoftAP autonomo per telemetria su smartphone
├── lib/
│   └── SSD1283A/               # Libreria driver display ottimizzata per ESP32 SPI
└── src/
    └── main.cpp                # Loop deterministico multi-frequenza (IMU 100Hz, UI 25Hz)
```

---

## 3. Schema dei Collegamenti Elettrici Aggiornato (Pin 0-10 Consecutivi)

### Modulo Sensori GY-89 (10-DOF)
| Pin Modulo GY-89 | Pin ESP32-C3 | Funzione / Descrizione |
|---|---|---|
| **VCC** | **3.3V** | Alimentazione modulo IMU |
| **GND** | **GND** | Massa comune |
| **CS2** | **GPIO 0** | Chip Select 2 (pilotato ad ALTO per abilitare L3GD20 in I2C) |
| **CS1** | **GPIO 1** | Chip Select 1 (pilotato ad ALTO per abilitare LSM303D in I2C) |
| **SDO / SA0** | **GPIO 2** | Indirizzo I2C (pilotato ad ALTO: LSM303D=0x1E, L3GD20=0x6B) |
| **SDA** | **GPIO 3** | Dati bus I2C |
| **SCL** | **GPIO 4** | Clock bus I2C (400 kHz Fast-Mode) |

### Display LCD 1.6" SPI 130x130 (SSD1283A)
| Pin Display | Pin ESP32-C3 | Funzione / Descrizione |
|---|---|---|
| **VCC** | **3.3V** | Alimentazione logica |
| **GND** | **GND** | Massa comune |
| **CS** | **GPIO 5** | SPI Chip Select |
| **RESET / RST** | **GPIO 6** | Reset display |
| **A0 / DC** | **GPIO 7** | Data / Command |
| **SDA / MOSI** | **GPIO 8** | SPI Hardware Data Out |
| **SCK / SCLK** | **GPIO 9** | SPI Hardware Clock |
| **LED / BL** | **GPIO 10** | Retroilluminazione display (pilotata ad ALTO / PWM) |

### Controlli & Stato
| Componente | Pin ESP32-C3 | Funzione |
|---|---|---|
| **Pulsante Utente** | **GPIO 20** | Tasto multifunzione a manubrio (a massa, pull-up interno) |
| **LED Stato** | **GPIO 21** | Indicatore stato Wi-Fi e calibrazione |

> [!TIP]
> **Scelta del tasto su GPIO 9 anziché GPIO 0:** Come discusso, GPIO 0 è uno *strapping pin* che se tenuto a massa all'avvio manda l'ESP32 in modalità bootloader UART. Abbiamo configurato **GPIO 9** (il tasto BOOT già saldato su tutte le schede ESP32-C3 Super Mini), che dispone di pull-up integrato e può essere premuto senza alcun rischio di blocco al boot.

---

## 4. Architettura Fisica del Filtro in Curva

Il filtro implementato in [MotorcycleFilter.h](file:///C:/Users/Edoardo/Documents/GitHub/FZ8_Dynamics_Telemetry/include/MotorcycleFilter.h) risolve la criticità fisica della moto in curva:
1. **In Curva** ($|a_{\text{tot}}| > 1.08g$ oppure $|\omega_z| > 3^\circ/\text{s}$ oppure $|\omega_x| > 3.5^\circ/\text{s}$):
   - La correzione gravitazionale dell'accelerometro sul rollio viene **completamente disattivata** ($0\%$).
   - L'angolo di piega viene mantenuto fedele per tutta la percorrenza dall'integrazione a 100 Hz del giroscopio L3GD20, eliminando l'effetto "richiamo a 0°" nelle curve lunghe o rotonde.
2. **Nei Tratti a Moto Dritta** ($|a_{\text{tot}}| \approx 1g$, $|\omega_z| < 3^\circ/\text{s}$ per $> 250\text{ms}$):
   - Viene riattivata la correzione complementare lenta per azzerare la deriva e correggere il bias termico del giroscopio.

---

## 5. Stato di Programmazione del Dispositivo

🎉 **Firmware caricato con successo sull'ESP32-C3!**
- **Porta rilevata**: `COM9` (`USB\VID_303A&PID_1001` - Espressif USB-Serial/JTAG)
- **Flash eseguito**:
  - `bootloader.bin` scritto a `0x00000000`
  - `partitions.bin` scritto a `0x00008000`
  - `firmware.bin` scritto a `0x00010000` (1,091,344 byte scritti e verificati tramite checksum SHA)
- **Chip rilevato**: ESP32-C3 (QFN32 rev 0.4), 4MB Flash embedded, MAC `ac:a7:04:d0:a1:b4`.

---

## 6. Come Caricare Futuri Aggiornamenti

Dalla cartella del progetto:
```powershell
cd C:\Users\Edoardo\Documents\GitHub\FZ8_Dynamics_Telemetry
$env:PYTHONIOENCODING="utf-8"
& "$HOME\.platformio\penv\Scripts\esptool.exe" --chip esp32c3 --port COM9 --baud 460800 write-flash 0x0 .pio\build\esp32-c3-devkitm-1\bootloader.bin 0x8000 .pio\build\esp32-c3-devkitm-1\partitions.bin 0x10000 .pio\build\esp32-c3-devkitm-1\firmware.bin
```

### Comandi del Tasto Fisico (GPIO 20):
- **Pressione Breve (< 0.8s)**: Cambia schermata sul display LCD 130x130:
  1. *DYNAMICS*: Grafica ad arco, gradi numerici grandi con codice colore (Verde/Arancio/Rosso), record L/R e G-force.
  2. *G-G DIAGRAM*: Cerchio delle forze / attrito (staccata vs percorrenza laterale).
  3. *SESSION STATS*: Riepilogo staccata massima, accelerazione massima, dislivello metri, tempo sessione.
- **Pressione Media (0.8s - 2.5s)**: **Tara Zero**: Azzera l'inclinazione corrente (es. moto sul cavalletto o prima di partire).
- **Pressione Lunga (> 2.5s)**: **Accende / Spegne il Wi-Fi**:
  - Wi-Fi spento durante la guida per risparmio energetico e zero rumore RF.
  - A moto ferma, crea la rete `FZ8-Telemetry` (password: `yamaha-fz8`). Connettendoti col telefono su **`http://192.168.4.1`** visualizzi la dashboard live e i record di sessione.
