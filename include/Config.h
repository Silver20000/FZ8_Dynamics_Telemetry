#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==============================================================================
// HARDWARE PIN CONFIGURATION (ESP32-C3 Mini / Super Mini)
// ==============================================================================

// I2C Bus Pins (GY-89: LSM303D + L3GD20 + BMP180)
#define I2C_SDA_PIN         4
#define I2C_SCL_PIN         5
#define I2C_FREQ_HZ         400000  // 400 kHz Fast-Mode for low-latency (<1.5ms)

// SPI Bus & Display Pins (SSD1283A 1.6" Transflective LCD 130x130)
#define LCD_SCK_PIN         6       // SPI Clock
#define LCD_MOSI_PIN        7       // SPI MOSI (SDA on display board)
#define LCD_CS_PIN          10      // Chip Select
#define LCD_DC_PIN          2       // Data / Command (A0)
#define LCD_RST_PIN         3       // Reset
#define LCD_LED_PIN         1       // Backlight PWM / Control (-1 if tied to 3.3V)

// User Interface / Button Pins
// Primary button: GPIO 9 (Built-in BOOT button on ESP32-C3 Super Mini, active LOW)
// Note: GPIO 0 is a strapping pin (requires 10k pullup, HIGH at boot)
#define BUTTON_PIN          9       // Onboard button (active LOW, internal pull-up)
#define STATUS_LED_PIN      8       // Onboard status LED (active LOW on most C3 mini boards)

// ==============================================================================
// MOTORCYCLE DYNAMICS & FILTER PARAMETERS
// ==============================================================================
#define IMU_SAMPLE_FREQ_HZ      100     // 100 Hz loop for IMU (10ms dt)
#define IMU_SAMPLE_PERIOD_MS    (1000 / IMU_SAMPLE_FREQ_HZ)

// Straight & Upright Detection Gates (Disables Accel roll correction during cornering)
// A motorcycle in a coordinated turn has apparent gravity vector along Z axis (pedane),
// so accelerometer roll measurement is deceptive during cornering.
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
