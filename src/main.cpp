#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Config.h"
#include "SensorsGY89.h"
#include "MotorcycleFilter.h"
#include "DisplayUI.h"
#include "WebDashboard.h"

// Global instances
SensorsGY89 imuSensors;
MotorcycleFilter motoFilter;
DisplayUI uiDisplay;
WebDashboard webServer;

// Shared dynamics state
MotorcycleDynamics currentDynamics;
IMURawData rawImu;

// Timing intervals
uint32_t lastImuMicros = 0;
uint32_t lastDisplayMillis = 0;
uint32_t lastBaroMillis = 0;

// Button state machine
bool lastButtonState = HIGH;
uint32_t buttonPressStart = 0;
bool buttonHandled = false;

void handleButton() {
    bool currentBtn = digitalRead(BUTTON_PIN);
    uint32_t now = millis();

    // Button pressed (active LOW)
    if (currentBtn == LOW && lastButtonState == HIGH) {
        buttonPressStart = now;
        buttonHandled = false;
    }
    // Button held down
    else if (currentBtn == LOW && !buttonHandled) {
        uint32_t duration = now - buttonPressStart;
        if (duration >= 2500) {
            // Long Press (> 2.5s): Toggle Wi-Fi AP
            if (webServer.isRunning()) {
                webServer.stop();
                uiDisplay.setWifiStatus(false);
                digitalWrite(STATUS_LED_PIN, HIGH); // LED off
            } else {
                webServer.start(&motoFilter, &currentDynamics);
                uiDisplay.setWifiStatus(true);
                digitalWrite(STATUS_LED_PIN, LOW);  // LED on
            }
            buttonHandled = true;
        }
    }
    // Button released
    else if (currentBtn == HIGH && lastButtonState == LOW) {
        if (!buttonHandled) {
            uint32_t duration = now - buttonPressStart;
            if (duration >= 800 && duration < 2500) {
                // Medium Press: Zero Tare
                motoFilter.tareZero();
                uiDisplay.showTareNotice(motoFilter.getTareRoll(), motoFilter.getTarePitch());
            } else if (duration >= 50 && duration < 800) {
                // Short Click: Next Screen
                uiDisplay.nextScreen();
            }
        }
        buttonHandled = true;
    }

    lastButtonState = currentBtn;
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n==========================================");
    Serial.println(" Yamaha FZ8 Dynamics Telemetry Computer");
    Serial.println(" ESP32-C3 + GY-89 (10-DOF) + LCD SSD1283A");
    Serial.println("==========================================");

    // Initialize Button & Status LED
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, HIGH); // Turn off onboard LED

    // 1. Initialize Display UI (SPI)
    Serial.println("[SYSTEM] Initializing 1.6\" LCD (SSD1283A)...");
    uiDisplay.begin();

    // 2. Initialize GY-89 I2C Sensors (LSM303D + L3GD20 + BMP180)
    Serial.println("[SYSTEM] Initializing GY-89 10-DOF Sensors at 400kHz...");
    if (!imuSensors.begin()) {
        Serial.println("[SYSTEM] CRITICAL ERROR: Failed to detect IMU!");
    } else {
        Serial.println("[SYSTEM] IMU Sensors ready and calibrated.");
    }

    // 3. Initialize Dynamics Filter & Load Stored Zero Tare from Flash NVS
    motoFilter.begin();

    lastImuMicros = micros();
    lastDisplayMillis = millis();
    lastBaroMillis = millis();
}

void loop() {
    uint32_t currentMicros = micros();
    uint32_t currentMillis = millis();

    // ==========================================================================
    // 1. HIGH PRIORITY: IMU SAMPLING & MOTORCYCLE FILTER (100 Hz / 10ms)
    // ==========================================================================
    if (currentMicros - lastImuMicros >= (IMU_SAMPLE_PERIOD_MS * 1000)) {
        lastImuMicros = currentMicros;

        // Fast burst read from LSM303D (accel) & L3GD20 (gyro)
        imuSensors.readIMU(rawImu);

        // Run motorcycle-specific kinematic gating filter
        motoFilter.update(rawImu, currentDynamics);
    }

    // ==========================================================================
    // 2. MEDIUM PRIORITY: DISPLAY REFRESH (25 Hz / 40ms)
    // ==========================================================================
    if (currentMillis - lastDisplayMillis >= DISPLAY_REFRESH_MS) {
        lastDisplayMillis = currentMillis;
        uiDisplay.update(currentDynamics);
    }

    // ==========================================================================
    // 3. LOW PRIORITY: BMP180 BAROMETER STATE MACHINE (every 500ms)
    // ==========================================================================
    imuSensors.updateBarometer();

    // ==========================================================================
    // 4. USER INTERACTION: BUTTON DEBOUNCER & WI-FI SERVER
    // ==========================================================================
    handleButton();

    if (webServer.isRunning()) {
        webServer.handleClient();
    }
}
