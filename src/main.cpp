#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "Config.h"
#include "SensorsGY89.h"
#include "MotorcycleFilter.h"
#include "SessionLogger.h"
#include "BluetoothTelemetry.h"
#include "EspNowProtocol.h"

// ==============================================================================
// YAMAHA FZ8 MOTORCYCLE DYNAMICS MASTER UNIT (UNDER-SEAT / TAIL NODE)
// High-Performance 100Hz IMU Kinematic Gating + TPS Sensor + 20Hz BLE + 50Hz ESP-NOW
// ==============================================================================

// Global Instances
SensorsGY89 imuSensors;
MotorcycleFilter motoFilter;
SessionLogger sessionLogger;
BluetoothTelemetry bleTelemetry;
Preferences prefs;

// Dynamics State
MotorcycleDynamics currentDynamics;
IMURawData rawImu;

// TPS (Throttle Position Sensor) State & Calibration
uint16_t tpsMinAdc = TPS_MIN_ADC_DEFAULT;
uint16_t tpsMaxAdc = TPS_MAX_ADC_DEFAULT;
float filteredTpsAdc = TPS_MIN_ADC_DEFAULT;

// ESP-NOW Broadcast Setup (for Wireless Cockpit Display)
uint8_t espNowBroadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
EspNowTelemetryPacket espNowPacket;
bool espNowActive = false;
uint8_t selectedScreenIndex = 0;

// Timing intervals
uint32_t lastImuMicros = 0;
uint32_t lastBaroMillis = 0;
uint32_t lastLogMillis = 0;
uint32_t lastEspNowMillis = 0;
uint32_t lastHeartbeatMillis = 0;

// Button state machine (GPIO 20)
bool lastButtonState = HIGH;
uint32_t buttonPressStart = 0;
bool buttonHandled = false;

void loadTpsCalibration() {
    prefs.begin("fz8_tps", false);
    tpsMinAdc = prefs.getUShort("min", TPS_MIN_ADC_DEFAULT);
    tpsMaxAdc = prefs.getUShort("max", TPS_MAX_ADC_DEFAULT);
    prefs.end();
    filteredTpsAdc = tpsMinAdc;
    Serial.printf("[TPS] Calibrazione caricata: Min(0%%)=%d ADC, Max(100%%)=%d ADC\n", tpsMinAdc, tpsMaxAdc);
}

void saveTpsZero() {
    uint16_t currentAdc = analogRead(TPS_ADC_PIN);
    tpsMinAdc = currentAdc;
    prefs.begin("fz8_tps", false);
    prefs.putUShort("min", tpsMinAdc);
    prefs.end();
    Serial.printf("[TPS] *** ZERO GAS CALIBRATO: %d ADC ***\n", tpsMinAdc);
}

void saveTpsWot() {
    uint16_t currentAdc = analogRead(TPS_ADC_PIN);
    if (currentAdc > tpsMinAdc + 200) {
        tpsMaxAdc = currentAdc;
        prefs.begin("fz8_tps", false);
        prefs.putUShort("max", tpsMaxAdc);
        prefs.end();
        Serial.printf("[TPS] *** 100%% GAS (WOT) CALIBRATO: %d ADC ***\n", tpsMaxAdc);
    } else {
        Serial.println("[TPS] ERRORE: Valore WOT troppo basso rispetto al minimo!");
    }
}

void updateTps() {
    uint16_t rawAdc = analogRead(TPS_ADC_PIN);
    // Filtro IIR passa-basso rapido (immunità a disturbi bobine/candele)
    filteredTpsAdc = 0.85f * filteredTpsAdc + 0.15f * (float)rawAdc;

    float pct = 0.0f;
    if (tpsMaxAdc > tpsMinAdc) {
        pct = (filteredTpsAdc - tpsMinAdc) * 100.0f / (float)(tpsMaxAdc - tpsMinAdc);
    }
    currentDynamics.tpsPercent = constrain(pct, 0.0f, 100.0f);
}

void initEspNowBroadcaster() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    if (esp_now_init() == ESP_OK) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, espNowBroadcastAddress, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            espNowActive = true;
            Serial.println("[ESPNOW] Broadcaster attivo a 50Hz (Canale 1) per Cruscotto Wireless!");
        }
    } else {
        Serial.println("[ESPNOW] Inizializzazione ESP-NOW fallita!");
    }
}

void sendEspNowTelemetry(const MotorcycleDynamics& dyn) {
    if (!espNowActive) return;

    espNowPacket.timestampMs = millis();
    espNowPacket.rollDegX10 = (int16_t)(dyn.rollDeg * 10.0f);
    espNowPacket.pitchDegX10 = (int16_t)(dyn.pitchDeg * 10.0f);
    espNowPacket.gLateralX100 = (int16_t)(dyn.gLateral * 100.0f);
    espNowPacket.gLongitudinalX100 = (int16_t)(dyn.gLongitudinal * 100.0f);
    espNowPacket.rollRateDpsX10 = (int16_t)(dyn.rollRateDps * 10.0f);
    espNowPacket.yawRateDpsX10 = (int16_t)(dyn.yawRateDps * 10.0f);
    espNowPacket.totalGX100 = (int16_t)(dyn.totalG * 100.0f);
    espNowPacket.altitudeM = (int16_t)dyn.altitudeM;
    espNowPacket.maxLeanLeftX10 = (int16_t)(dyn.maxLeanLeft * 10.0f);
    espNowPacket.maxLeanRightX10 = (int16_t)(dyn.maxLeanRight * 10.0f);
    espNowPacket.maxBrakingGX100 = (int16_t)(dyn.maxBrakingG * 100.0f);
    espNowPacket.maxAccelGX100 = (int16_t)(dyn.maxAccelG * 100.0f);
    espNowPacket.tpsPercent = (uint8_t)dyn.tpsPercent;
    espNowPacket.activeScreen = selectedScreenIndex;
    
    uint8_t flags = 0;
    if (dyn.isLeanWarning) flags |= (1 << 0);
    if (dyn.isBraking)     flags |= (1 << 1);
    if (dyn.isAccelerating)flags |= (1 << 2);
    if (sessionLogger.isLogging()) flags |= (1 << 3);
    espNowPacket.flags = flags;

    esp_now_send(espNowBroadcastAddress, (uint8_t*)&espNowPacket, sizeof(espNowPacket));
}

void handleButton() {
    bool currentBtn = digitalRead(BUTTON_PIN);
    uint32_t now = millis();

    if (currentBtn == LOW && lastButtonState == HIGH) {
        buttonPressStart = now;
        buttonHandled = false;
    } else if (currentBtn == LOW && !buttonHandled) {
        uint32_t duration = now - buttonPressStart;
        if (duration >= 2000) {
            // Long Press (>= 2s): Start / Stop Flash Datalogger
            if (sessionLogger.isLogging()) {
                sessionLogger.stopSession();
                Serial.println("[BUTTON] Datalogger STOP");
            } else {
                sessionLogger.startSession();
                Serial.println("[BUTTON] Datalogger START");
            }
            buttonHandled = true;
        }
    } else if (currentBtn == HIGH && lastButtonState == LOW) {
        if (!buttonHandled) {
            uint32_t duration = now - buttonPressStart;
            if (duration >= 50 && duration < 2000) {
                // Short Click (< 2s): Zero Tare
                motoFilter.tareZero();
                Serial.println("[BUTTON] Zero Tare Salvato in Flash!");
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
            Serial.printf("[STATUS] Roll: %.1f deg, Pitch: %.1f deg, GLat: %.2f G, GLong: %.2f G, TPS: %.1f%% (ADC:%d)\n",
                currentDynamics.rollDeg, currentDynamics.pitchDeg,
                currentDynamics.gLateral, currentDynamics.gLongitudinal,
                currentDynamics.tpsPercent, (int)filteredTpsAdc);
        } else if (c == 't' || c == 'T') {
            motoFilter.tareZero();
            Serial.println("[SERIAL] Zero Tare Eseguito!");
        } else if (c == 'r' || c == 'R') {
            motoFilter.toggleInvertRoll();
            Serial.printf("[SERIAL] InvertRoll: %d\n", motoFilter.getInvertRoll());
        } else if (c == 'x' || c == 'X') {
            motoFilter.toggleSwapXY();
            Serial.printf("[SERIAL] SwapXY: %d\n", motoFilter.getSwapXY());
        } else if (c == 'c') {
            saveTpsZero();
        } else if (c == 'C') {
            saveTpsWot();
        } else if (c == 'l' || c == 'L') {
            if (sessionLogger.isLogging()) sessionLogger.stopSession();
            else sessionLogger.startSession();
            Serial.printf("[SERIAL] Datalogger: %s\n", sessionLogger.isLogging() ? "REC" : "STOP");
        } else if (c >= '1' && c <= '9') {
            selectedScreenIndex = c - '1';
            Serial.printf("[SERIAL] Selected Screen: %d\n", selectedScreenIndex);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("\n================================================================");
    Serial.println(" Yamaha FZ8 Dynamics Telemetry Master Unit (Tail Node + TPS)");
    Serial.println(" ESP32-C3 + GY-89 (10-DOF) | Dual Radio: BLE + ESP-NOW");
    Serial.println("================================================================");

    // Hardware Pins
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(STATUS_LED_PIN, OUTPUT);
    digitalWrite(STATUS_LED_PIN, LOW); // LED ON at boot

    // Analog Inputs Configuration (TPS on GPIO 7)
    analogReadResolution(12); // 0 - 4095
    analogSetPinAttenuation(TPS_ADC_PIN, ADC_11db); // 0 - 3.1V Full Scale
    loadTpsCalibration();

    // 1. Initialize GY-89 10-DOF I2C Sensors (400 kHz Fast-Mode)
    Serial.println("[SYSTEM] Inizializzazione GY-89 (LSM303D + L3GD20 + BMP180)...");
    if (!imuSensors.begin()) {
        Serial.println("[SYSTEM] CRITICAL ERROR: IMU non rilevata!");
    } else {
        Serial.println("[SYSTEM] GY-89 pronto e calibrato.");
    }

    // 2. Initialize Motorcycle Dynamics Filter & Load Tare from NVS
    motoFilter.begin();

    // 3. Initialize LittleFS Flash Datalogger
    sessionLogger.begin();

    // 4. Initialize ESP-NOW Broadcaster (50 Hz Wireless link for Cockpit Display)
    initEspNowBroadcaster();

    // 5. Initialize Bluetooth BLE Telemetry Server
    bleTelemetry.setCommandCallback([](const String& cmd) {
        if (cmd.startsWith("SCREEN:")) {
            selectedScreenIndex = cmd.substring(7).toInt();
            Serial.printf("[BLE EXEC] Screen remota: %d\n", selectedScreenIndex);
        } else if (cmd == "SCREEN_NEXT") {
            selectedScreenIndex = (selectedScreenIndex + 1) % 9;
            Serial.printf("[BLE EXEC] Next Screen: %d\n", selectedScreenIndex);
        } else if (cmd == "TARE") {
            motoFilter.tareZero();
            Serial.println("[BLE EXEC] Zero Tare Salvato!");
        } else if (cmd == "RESET_TARE") {
            motoFilter.resetTare();
            Serial.println("[BLE EXEC] Zero Tare resettato!");
        } else if (cmd == "CALIB_TPS_ZERO") {
            saveTpsZero();
        } else if (cmd == "CALIB_TPS_WOT") {
            saveTpsWot();
        } else if (cmd == "INV_ROLL") {
            motoFilter.toggleInvertRoll();
            Serial.printf("[BLE EXEC] InvertRoll: %d\n", motoFilter.getInvertRoll());
        } else if (cmd == "SWAP_XY") {
            motoFilter.toggleSwapXY();
            Serial.printf("[BLE EXEC] SwapXY: %d\n", motoFilter.getSwapXY());
        } else if (cmd == "INV_ACCEL") {
            motoFilter.toggleInvertAccel();
            Serial.printf("[BLE EXEC] InvertAccel: %d\n", motoFilter.getInvertAccel());
        } else if (cmd == "RESET_RECORDS") {
            motoFilter.resetRecords();
            Serial.println("[BLE EXEC] Record sessione azzerati!");
        } else if (cmd == "REC_START") {
            sessionLogger.startSession();
            Serial.println("[BLE EXEC] Datalogger avviato!");
        } else if (cmd == "REC_STOP") {
            sessionLogger.stopSession();
            Serial.println("[BLE EXEC] Datalogger fermato!");
        }
    });

    bleTelemetry.begin();
    digitalWrite(STATUS_LED_PIN, HIGH); // LED OFF: Pronta

    lastImuMicros = micros();
    lastBaroMillis = millis();
    lastLogMillis = millis();
    lastEspNowMillis = millis();
    lastHeartbeatMillis = millis();
    
    Serial.println("[SYSTEM] *** Master Node operativo a 100Hz + TPS attivo! ***\n");
}

void loop() {
    uint32_t currentMicros = micros();
    uint32_t currentMillis = millis();

    // ==========================================================================
    // 1. HIGH PRIORITY: IMU BURST SAMPLING & KINEMATIC GATING FILTER (100 Hz / 10ms)
    // ==========================================================================
    if (currentMicros - lastImuMicros >= (IMU_SAMPLE_PERIOD_MS * 1000)) {
        lastImuMicros = currentMicros;

        imuSensors.readIMU(rawImu);
        motoFilter.update(rawImu, currentDynamics);
        updateTps();
    }

    // ==========================================================================
    // 2. ESP-NOW WIRELESS BROADCAST TO COCKPIT DISPLAY (50 Hz / 20ms)
    // ==========================================================================
    if (currentMillis - lastEspNowMillis >= 20) {
        lastEspNowMillis = currentMillis;
        sendEspNowTelemetry(currentDynamics);
    }

    // ==========================================================================
    // 3. BLE 20Hz TELEMETRY STREAM TO SMARTPHONE WEB APP / RACECHRONO
    // ==========================================================================
    bleTelemetry.update(currentDynamics);

    // ==========================================================================
    // 4. LOW PRIORITY: BMP180 BAROMETER STATE MACHINE (every 500ms)
    // ==========================================================================
    imuSensors.updateBarometer();

    // ==========================================================================
    // 5. FLASH DATALOGGER (10 Hz / 100ms)
    // ==========================================================================
    if (currentMillis - lastLogMillis >= DATALOGGER_INTERVAL_MS) {
        lastLogMillis = currentMillis;
        sessionLogger.logSample(currentDynamics, currentMillis);
    }

    // ==========================================================================
    // 6. HEARTBEAT LED & USER INTERACTION (BUTTON & SERIAL)
    // ==========================================================================
    handleSerial();
    handleButton();

    if (currentMillis - lastHeartbeatMillis >= 1000) {
        lastHeartbeatMillis = currentMillis;
        digitalWrite(STATUS_LED_PIN, LOW);
        delayMicroseconds(20000);
        digitalWrite(STATUS_LED_PIN, HIGH);
    }
}
