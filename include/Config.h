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
// Imposta a true per ruotare di 90 gradi l'orientamento tra accelerazione e piega
#define IMU_SWAP_XY             true    // Scambia assi X e Y (rende accelerazione e piega perpendicolari)
#define IMU_INVERT_ROLL         false   // Inverte il verso della piega (Destra/Sinistra) se necessario
#define IMU_INVERT_ACCEL        false   // Inverte il verso dell'accelerazione (ACC/BRK) se necessario

// Offset manuali di montaggio (se si desidera impostare un valore fisso da codice anziche salvare via Tare)
#define MANUAL_ROLL_OFFSET_DEG   0.0f   // Inclinazione laterale a riposo (gradi)
#define MANUAL_PITCH_OFFSET_DEG  0.0f   // Inclinazione beccheggio/frontale a riposo (gradi)

// ==============================================================================
// MOTORCYCLE DYNAMICS & FILTER PARAMETERS
// ==============================================================================
#define IMU_SAMPLE_FREQ_HZ      100     // 100 Hz loop for IMU (10ms dt)
#define IMU_SAMPLE_PERIOD_MS    (1000 / IMU_SAMPLE_FREQ_HZ)

// Straight & Upright Detection Gates (Disables Accel roll correction during cornering)
#define GATE_TOTAL_G_MIN        0.92f   // Total G lower bound for straight motion (g)
#define GATE_TOTAL_G_MAX        1.08f   // Total G upper bound for straight motion (g)
#define GATE_YAW_RATE_MAX       3.0f    // Max yaw rate (|wz|) for straight motion (deg/s)
#define GATE_ROLL_RATE_MAX      3.5f    // Max roll rate (|wx|) for straight motion (deg/s)
#define GATE_LATERAL_G_MAX      0.08f   // Max lateral accel (|ay|) for straight motion (g)
#define STRAIGHT_TIME_CONFIRM_MS 250    // Time bike must remain straight to allow gyro bias update

// Display Refresh Rate
#define DISPLAY_REFRESH_HZ      25      // 25 Hz UI refresh (40ms)
#define DISPLAY_REFRESH_MS      (1000 / DISPLAY_REFRESH_HZ)

// Barometer (BMP180) Poll Interval
#define BARO_POLL_INTERVAL_MS   500     // 2 Hz altitude/temp query (non-blocking)

// Wi-Fi Access Point Configuration (On-Demand)
#define AP_SSID                 "FZ8-Telemetry"
#define AP_PASSWORD             "yamaha-fz8"
#define AP_IP_ADDR              "192.168.4.1"

#endif // CONFIG_H
