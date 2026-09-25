#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==============================================================================
// HARDWARE PIN CONFIGURATION (ESP32-C3 Mini / Super Mini)
// ==============================================================================

// GY-89 10-DOF Module Pins
#define GY89_CS2_PIN        0       // CS2 -> GPIO 0 (L3GD20 I2C enable)
#define GY89_CS1_PIN        1       // CS1 -> GPIO 1 (LSM303D I2C enable)
#define GY89_SA0_PIN        2       // SDO / SA0 -> GPIO 2 (Address select)
#define I2C_SDA_PIN         3       // SDA -> GPIO 3 (I2C Data)
#define I2C_SCL_PIN         4       // SCL -> GPIO 4 (I2C Clock)

#define I2C_FREQ_HZ         400000  // 400 kHz Fast-Mode for low-latency (<1.5ms)

// Analog Telemetry Inputs (Free ADC Pins on ESP32-C3)
#define TPS_ADC_PIN         7       // Throttle Position Sensor (TPS) Signal -> GPIO 7 (ADC1_CH2)
#define TPS_MIN_ADC_DEFAULT 450     // Default ADC a farfalla 0% (gas chiuso)
#define TPS_MAX_ADC_DEFAULT 2800    // Default ADC a farfalla 100% (WOT)

// SPI Bus & Display Pins (SSD1283A 1.6" Transflective LCD 130x130)
#define LCD_CS_PIN          5       // CS -> GPIO 5
#define LCD_RST_PIN         6       // Reset -> GPIO 6
#define LCD_DC_PIN          7       // A0 / DC -> GPIO 7
#define LCD_MOSI_PIN        8       // SDA / MOSI -> GPIO 8
#define LCD_SCK_PIN         9       // SCK -> GPIO 9
#define LCD_LED_PIN         10      // LED / Backlight -> GPIO 10

// User Interface / Status Pins (GPIO 20 and 21)
#define BUTTON_PIN          20      // User button (active LOW, internal pull-up) -> GPIO 20
#define STATUS_LED_PIN      21      // Status LED (active LOW / optional) -> GPIO 21

// ==============================================================================
// IMU SENSOR AXIS ORIENTATION
// ==============================================================================
#define IMU_SWAP_XY             true    // true = Roll e Pitch orientati correttamente con USB-C verso anteriore
#define IMU_INVERT_ROLL         true    // true = Inverte il verso della piega (inclinando a SX mostra SX)
#define IMU_INVERT_ACCEL        false   // Inverte il verso dell'accelerazione (ACC/BRK) se necessario

// Allineamento Rotazionale Orizzontale (Yaw / Skew Correction)
// Con SwapXY=true il cross-talk e' azzerato a 0.3 deg
#define IMU_YAW_ALIGN_DEG       0.0f    // 0.0 deg (disaccoppiamento perfetto con SwapXY=true)

// Offset manuali di montaggio (se si desidera impostare un valore fisso da codice anziche salvare via Tare)
#define MANUAL_ROLL_OFFSET_DEG   0.0f   // Inclinazione laterale a riposo (gradi)
#define MANUAL_PITCH_OFFSET_DEG  0.0f   // Inclinazione beccheggio/frontale a riposo (gradi)

// ==============================================================================
// MOTORCYCLE DYNAMICS & FILTER PARAMETERS
// ==============================================================================
#define IMU_SAMPLE_FREQ_HZ      100     // 100 Hz loop for IMU (10ms dt)
#define IMU_SAMPLE_PERIOD_MS    (1000 / IMU_SAMPLE_FREQ_HZ)
constexpr float FILTER_SAMPLE_RATE_HZ = 100.0f;
constexpr float DT                    = 1.0f / FILTER_SAMPLE_RATE_HZ;

// Soglie Gating Accelerometro (anti-trappola curva coordinata e rigetto vibrazioni)
constexpr float GATE_TOTAL_G_MIN       = 0.95f;   // [g] Range gravità statica min
constexpr float GATE_TOTAL_G_MAX       = 1.05f;   // [g] Range gravità statica max
constexpr float GATE_AX_MAX            = 0.08f;   // [g] Soglia accelerazione longitudinale (gas/freno -> DYNAMIC istantaneo)
constexpr float GATE_AY_MAX            = 0.06f;   // [g] Soglia accelerazione laterale residua
constexpr float GATE_GYRO_MAX_DPS      = 3.5f;    // [deg/s] Movimento angolare totale massimo (|omega|)
constexpr float GATE_YAW_RATE_MAX      = 3.0f;    // [deg/s] Max yaw rate (|wz|)
constexpr float GATE_ROLL_RATE_MAX     = 3.5f;    // [deg/s] Max roll rate (|wx|)
constexpr uint32_t STRAIGHT_HOLD_MS    = 350;     // [ms] Tempo continuo di rettilineo richiesto prima di abilitare correzioni
#define STRAIGHT_TIME_CONFIRM_MS        350
#define GATE_LATERAL_G_MAX              0.06f

// Pesi Filtro Complementare Dinamico a 3 Stati
constexpr float ALPHA_DYNAMIC          = 1.000f;  // Giroscopio puro (100%), accel escluso totalmente (piega, gas, frenata)
constexpr float ALPHA_STRAIGHT         = 0.997f;  // Correzione lenta e impercettibile in rettilineo confermato
constexpr float ALPHA_STATIONARY       = 0.850f;  // Allineamento rapido alla gravità a moto ferma (zero drift all'avvio)

// Bias Tracker Adattivo Giroscopio
constexpr float BIAS_LEARN_RATE        = 0.0002f; // Tasso di apprendimento drift giroscopico conservativo
constexpr float BIAS_MAX_DPS_LIMIT     = 3.0f;    // Taglio di sicurezza sul bias massimo accumulabile (deg/s)
#define BIAS_MAX_DPS                    3.0f

// Display Refresh Rate
#define DISPLAY_REFRESH_HZ      25      // 25 Hz UI refresh (40ms)
#define DISPLAY_REFRESH_MS      (1000 / DISPLAY_REFRESH_HZ)

// Barometer (BMP180) Poll & Aggressive Low-Pass Filtering (immunità turbolenze aerodinamiche)
#define BARO_POLL_INTERVAL_MS   500     // 2 Hz altitude/temp query (non-blocking)
#define BARO_ALT_ALPHA          0.08f   // Filtro IIR passa-basso su quota (tau ~ 2 sec)
#define ELEVATION_GAIN_DEADBAND_M 2.0f  // Deadband minima accumulo D+ per rigettare turbolenze cupolino

// Datalogger Compatto Binario (LittleFS Flash)
#define DATALOGGER_INTERVAL_MS   100    // 10 Hz sampling rate for Flash datalogger
#define LOG_RAM_BUFFER_SAMPLES   50     // 50 campioni = 5 secondi di registrazione in RAM
#define LOG_STRUCT_SIZE          20     // 20 byte per campione binario
#define LOG_RAM_BUFFER_BYTES     (LOG_RAM_BUFFER_SAMPLES * LOG_STRUCT_SIZE) // 1000 byte buffer RAM

// Radio Modes (Mutua Esclusione per ESP32-C3 Single-Core)
enum RadioMode {
    RADIO_MODE_DASHBOARD = 0,   // Solo Hotspot Wi-Fi AP + Captive Portal (zero STA, zero scansioni)
    RADIO_MODE_RACECHRONO = 1   // NimBLE attivo a 20Hz, Wi-Fi OFF (zero contesa RF, zero latenza)
};

#define BLE_UPDATE_INTERVAL_MS  50      // 20 Hz update rate for RaceChrono BLE
#define BLE_DEVICE_NAME         "FZ8-Telemetry"

// Corner Analyzer (Isteresi Chicane)
#define CORNER_EXIT_HYSTERESIS_MS 250   // 250ms per confermare uscita curva (evita micro-reset su chicane)

// Lean Warning & Crash Detection (Roll-Rate Gated)
#define DEFAULT_MAX_LEAN_WARN_DEG 48.0f // Allarme visivo pedana a terra
#define CRASH_DETECT_ROLL_DEG    65.0f // Soglia rilevamento caduta/moto sdraiata
#define CRASH_DETECT_TIME_MS     3500  // Tempo minimo con moto ferma a terra per allarme
#define CRASH_DETECT_MAX_RATE_DPS 12.0f // Roll rate deve essere quasi nullo (moto ferma a terra)
#define CRASH_DETECT_ACCEL_TOL_G  0.35f // Accelerazione statica 1.0G +- 0.35G

// Magnetometer & Heading Gating
#define ENABLE_MAGNETOMETER          true
#define MAG_DECLINATION_DEG          3.5f   // Declinazione magnetica media Italia (+3.5 deg Est)
#define COMPASS_MAX_ROLL_VALID_DEG   6.0f   // Entro 6° di rollio la bussola è VALIDA
#define COMPASS_MAX_GLAT_VALID       0.15f  // Max 0.15G laterale
#define COMPASS_MAX_YAWRATE_VALID    4.0f   // Max 4 deg/s yaw rate

// ==============================================================================
// WI-FI CONFIGURATION: HOTSPOT AP AD ALTA STABILITA (12 dBm TX POWER)
// Rete Aperta, Canale 1, Captive Portal DNS, Zero cadute di tensione
// ==============================================================================
#define AP_SSID                 "FZ8-Telemetry"
#define AP_PASSWORD             ""               // Rete APERTA per massima compatibilita immediata
#define AP_IP_ADDR              "192.168.4.1"
#define AP_CHANNEL              1                // Canale 1 universale
#define AP_MAX_TX_POWER         48               // 12.0 dBm (evita cali di tensione USB/LDO)
#define MDNS_HOSTNAME           "fz8"            // Accessibile come http://fz8.local o http://192.168.4.1

#endif // CONFIG_H
