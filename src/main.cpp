#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Config.h"
#include "SensorsGY89.h"
#include "MotorcycleFilter.h"
#include "DisplayUI.h"
#include "WebDashboard.h"
#include "SessionLogger.h"
#include "BluetoothTelemetry.h"

// Global instances
SensorsGY89 imuSensors;
MotorcycleFilter motoFilter;
DisplayUI uiDisplay;
SessionLogger sessionLogger;
BluetoothTelemetry bleTelemetry;
WebDashboard webServer;

// Shared dynamics state
MotorcycleDynamics currentDynamics;
IMURawData rawImu;

// Timing intervals
uint32_t lastImuMicros = 0;
uint32_t lastDisplayMillis = 0;
uint32_t lastBaroMillis = 0;
uint32_t lastLogMillis = 0;

// Radio Mutual Exclusion State (Default: Bluetooth BLE 20Hz for Smartphone App)
RadioMode currentRadioMode = RADIO_MODE_RACECHRONO;

// Button state machine
bool lastButtonState = HIGH;
uint32_t buttonPressStart = 0;
bool buttonHandled = false;

void switchRadioMode(RadioMode newMode) {
    if (newMode == currentRadioMode) return;

    if (newMode == RADIO_MODE_RACECHRONO) {
        Serial.println("[RADIO] Switch -> MODALITA PISTA (RaceChrono NimBLE 20Hz)...");
        // 1. Spegni Wi-Fi AP + STA per liberare il frontend RF a 2.4 GHz e CPU
        webServer.stop();
        uiDisplay.setWifiStatus(false);
        uiDisplay.setRadioMode(RADIO_MODE_RACECHRONO);
        webServer.setRadioMode(RADIO_MODE_RACECHRONO);
        digitalWrite(STATUS_LED_PIN, HIGH); // LED off
        delay(100);

        // 2. Avvia NimBLE Server ultraleggero (~25 KB RAM)
        bleTelemetry.begin();
        currentRadioMode = RADIO_MODE_RACECHRONO;
        Serial.println("[RADIO] *** MODALITA PISTA ATTIVA: Wi-Fi OFF, NimBLE 20Hz ON ***");
    } else {
        Serial.println("[RADIO] Switch -> MODALITA DASHBOARD (Wi-Fi AP + STA)...");
        // 1. Spegni e de-inizializza NimBLE
        bleTelemetry.stop();
        delay(100);

        // 2. Avvia Wi-Fi Server
        webServer.start(&motoFilter, &currentDynamics, &uiDisplay, &sessionLogger, &bleTelemetry);
        uiDisplay.setWifiStatus(true);
        uiDisplay.setRadioMode(RADIO_MODE_DASHBOARD);
        webServer.setRadioMode(RADIO_MODE_DASHBOARD);
        digitalWrite(STATUS_LED_PIN, LOW); // LED on
        currentRadioMode = RADIO_MODE_DASHBOARD;
        Serial.println("[RADIO] *** MODALITA DASHBOARD ATTIVA: Wi-Fi ON, NimBLE OFF ***");
    }
}

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
        if (duration >= 1500) {
            // Long Press (>= 1.5s): Commutazione Modalità Radio (Wi-Fi vs NimBLE Pista)
            RadioMode nextMode = (currentRadioMode == RADIO_MODE_DASHBOARD) ? RADIO_MODE_RACECHRONO : RADIO_MODE_DASHBOARD;
            switchRadioMode(nextMode);
            buttonHandled = true;
        }
    }
    // Button released
    else if (currentBtn == HIGH && lastButtonState == LOW) {
        if (!buttonHandled) {
            uint32_t duration = now - buttonPressStart;
            if (duration >= 500 && duration < 1500) {
                // Medium Press (500ms - 1.5s): Zero Tare
                motoFilter.tareZero();
                uiDisplay.showTareNotice(motoFilter.getTareRoll(), motoFilter.getTarePitch());
            } else if (duration >= 40 && duration < 500) {
                // Short Click (< 500ms): Next Screen
                uiDisplay.nextScreen();
            }
        }
        buttonHandled = true;
    }

    lastButtonState = currentBtn;
}

void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == 's' || c == 'S') {
            Serial.printf("[STATUS] RadioMode: %s, AP: %s (Clients: %d), Roll: %.1f, Pitch: %.1f, swapXY: %d, invRoll: %d\n",
                currentRadioMode == RADIO_MODE_DASHBOARD ? "DASHBOARD (HOTSPOT)" : "PISTA (BLE)",
                WiFi.softAPIP().toString().c_str(),
                WiFi.softAPgetStationNum(),
                currentDynamics.rollDeg, currentDynamics.pitchDeg,
                motoFilter.getSwapXY(), motoFilter.getInvertRoll());
        } else if (c == 'm' || c == 'M') {
            RadioMode nextMode = (currentRadioMode == RADIO_MODE_DASHBOARD) ? RADIO_MODE_RACECHRONO : RADIO_MODE_DASHBOARD;
            switchRadioMode(nextMode);
        } else if (c == 'r' || c == 'R') {
            motoFilter.toggleInvertRoll();
            Serial.printf("[SERIAL] InvertRoll toggled to: %d\n", motoFilter.getInvertRoll());
        } else if (c == 'x' || c == 'X') {
            motoFilter.toggleSwapXY();
            Serial.printf("[SERIAL] SwapXY toggled to: %d\n", motoFilter.getSwapXY());
        } else if (c == 't' || c == 'T') {
            motoFilter.tareZero();
            Serial.println("[SERIAL] Tare Zero executed!");
        } else if (c == 'l' || c == 'L') {
            if (sessionLogger.isLogging()) sessionLogger.stopSession();
            else sessionLogger.startSession();
            Serial.printf("[SERIAL] Datalogger isLogging: %d\n", sessionLogger.isLogging());
        } else if (c == 'b' || c == 'B') {
            RadioMode nextMode = (currentRadioMode == RADIO_MODE_DASHBOARD) ? RADIO_MODE_RACECHRONO : RADIO_MODE_DASHBOARD;
            switchRadioMode(nextMode);
        } else if (c == 'd' || c == 'D') {
            uiDisplay.nextScreen();
            Serial.printf("[SERIAL] Display Screen: %d (%s)\n", (int)uiDisplay.getScreen(), uiDisplay.getScreenName(uiDisplay.getScreen()));
        } else if (c >= '1' && c <= '9') {
            int sc = c - '1';
            if (sc < SCREEN_COUNT) {
                uiDisplay.setScreen((UIScreenMode)sc);
                Serial.printf("[SERIAL] Display Screen: %d (%s)\n", sc, uiDisplay.getScreenName((UIScreenMode)sc));
            }
        } else if (c == 'w' || c == 'W') {
            uint8_t primaryChan = 0;
            wifi_second_chan_t secondChan;
            esp_wifi_get_channel(&primaryChan, &secondChan);
            wifi_config_t conf;
            esp_wifi_get_config(WIFI_IF_AP, &conf);
            int8_t maxPwr = 0;
            esp_wifi_get_max_tx_power(&maxPwr);
            Serial.printf("[WIFI-DIAG] Mode:%d SSID:'%s' len:%d Pass:'%s' Ch:%d confCh:%d hidden:%d auth:%d maxConn:%d beacon:%d TxPwr:%d maxPwr:%d Clients:%d\n",
                (int)WiFi.getMode(),
                (char*)conf.ap.ssid,
                conf.ap.ssid_len,
                (char*)conf.ap.password,
                (int)primaryChan,
                (int)conf.ap.channel,
                conf.ap.ssid_hidden,
                conf.ap.authmode,
                conf.ap.max_connection,
                conf.ap.beacon_interval,
                (int)WiFi.getTxPower(),
                (int)maxPwr,
                WiFi.softAPgetStationNum());
        } else if (c == 'p' || c == 'P') {
            Serial.println("[WIFI-SCAN] Avvio scansione reti 2.4GHz da ESP32...");
            int n = WiFi.scanNetworks(false, true); // non-blocking or sync
            Serial.printf("[WIFI-SCAN] Reti trovate da ESP32: %d\n", n);
            for (int i = 0; i < n; i++) {
                Serial.printf("  %d: SSID='%s', Ch=%d, RSSI=%d dBm\n", i+1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
            }
        } else if (c == 'a' || c == 'A') {
            bool nextAc = !uiDisplay.isAutoCycle();
            uiDisplay.setAutoCycle(nextAc);
            Serial.printf("[SERIAL] AutoCycle: %s\n", nextAc ? "ATTIVO" : "DISATTIVO");
        }
    }
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

    // 4. Initialize LittleFS Flash Datalogger (Compact 20B struct + RAM buffer)
    sessionLogger.begin();

    // 5. Start in BLE Motorcycle Telemetry Mode by default (Zero RF power spikes, 20Hz stream)
    Serial.println("[SYSTEM] Avvio Modalita Telemetria Bluetooth (BLE 20Hz)...");
    bleTelemetry.begin();
    uiDisplay.setWifiStatus(false);
    uiDisplay.setRadioMode(RADIO_MODE_RACECHRONO);
    webServer.setRadioMode(RADIO_MODE_RACECHRONO);
    digitalWrite(STATUS_LED_PIN, HIGH); // LED OFF: BLE low power mode

    lastImuMicros = micros();
    lastDisplayMillis = millis();
    lastBaroMillis = millis();
    lastLogMillis = millis();
}

void loop() {
    uint32_t currentMicros = micros();
    uint32_t currentMillis = millis();

    // ==========================================================================
    // 1. HIGH PRIORITY: IMU SAMPLING & MOTORCYCLE FILTER (100 Hz / 10ms)
    // ==========================================================================
    if (currentMicros - lastImuMicros >= (IMU_SAMPLE_PERIOD_MS * 1000)) {
        lastImuMicros = currentMicros;

        // Fast burst read from LSM303D (accel/mag) & L3GD20 (gyro)
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
    // 4. FLASH DATALOGGER (10 Hz / 100ms) & RACECHRONO BLE (20 Hz)
    // ==========================================================================
    if (currentMillis - lastLogMillis >= DATALOGGER_INTERVAL_MS) {
        lastLogMillis = currentMillis;
        sessionLogger.logSample(currentDynamics, currentMillis);
    }

    // Update BLE Telemetry only when in RaceChrono mode
    if (currentRadioMode == RADIO_MODE_RACECHRONO) {
        bleTelemetry.update(currentDynamics);
    }

    // ==========================================================================
    // 5. USER INTERACTION: BUTTON, SERIAL & RADIO/SERVER HANDLING
    // ==========================================================================
    handleSerial();
    handleButton();

    if (currentRadioMode == RADIO_MODE_DASHBOARD && webServer.isRunning()) {
        webServer.handleClient();

        // Check if Web Dashboard requested a radio switch to Pista (BLE)
        RadioMode reqMode;
        if (webServer.checkRadioSwitch(reqMode)) {
            switchRadioMode(reqMode);
        }
    }
}
