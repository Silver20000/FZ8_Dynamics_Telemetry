#ifndef BLUETOOTH_TELEMETRY_H
#define BLUETOOTH_TELEMETRY_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <functional>
#include "Config.h"
#include "MotorcycleFilter.h"
#include "DynamicAutoCalibrator.h"

// RaceChrono DIY Service and Characteristic UUIDs
#define RACECHRONO_SERVICE_UUID        "00001ff8-0000-1000-8000-00805f9b34fb"
#define RACECHRONO_CHARACTERISTIC_UUID "00000002-0000-1000-8000-00805f9b34fb"
#define FZ8_CMD_CHARACTERISTIC_UUID    "00000001-0000-1000-8000-00805f9b34fb"

class BluetoothTelemetry;

class BleCmdCallbacks : public NimBLECharacteristicCallbacks {
public:
    BleCmdCallbacks(BluetoothTelemetry* parent) : _parent(parent) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override;
private:
    BluetoothTelemetry* _parent;
};

class BluetoothTelemetry : public NimBLEServerCallbacks {
public:
    typedef std::function<void(const String&)> CommandCallback;

    BluetoothTelemetry() : 
        _enabled(false), 
        _initialized(false),
        _deviceConnected(false), 
        _pServer(nullptr), 
        _pCharacteristic(nullptr),
        _pCmdCharacteristic(nullptr),
        _lastUpdateMillis(0),
        _packetCounter(0),
        _cmdCallback(nullptr) {}

    bool isEnabled() const { return _enabled; }
    bool isConnected() const { return _deviceConnected; }

    void setCommandCallback(CommandCallback cb) {
        _cmdCallback = cb;
    }

    void handleIncomingCommand(const String& cmd) {
        if (_cmdCallback) {
            _cmdCallback(cmd);
        }
    }

    void begin() {
        if (_enabled) return;

        if (!_initialized) {
            Serial.println("[BLE] Inizializzazione NimBLE...");
            NimBLEDevice::init(BLE_DEVICE_NAME);
            NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Massima potenza RF

            _pServer = NimBLEDevice::createServer();
            _pServer->setCallbacks(this);

            NimBLEService* pService = _pServer->createService(RACECHRONO_SERVICE_UUID);
            
            // 1. Data Characteristic (Notify 20Hz Telemetry)
            _pCharacteristic = pService->createCharacteristic(
                RACECHRONO_CHARACTERISTIC_UUID,
                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
            );

            // 2. Command Characteristic (Write from Web App / Smartphone)
            _pCmdCharacteristic = pService->createCharacteristic(
                FZ8_CMD_CHARACTERISTIC_UUID,
                NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
            );
            _pCmdCharacteristic->setCallbacks(new BleCmdCallbacks(this));

            pService->start();

            NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
            pAdvertising->addServiceUUID(RACECHRONO_SERVICE_UUID);
            pAdvertising->setScanResponse(true);
            _initialized = true;
        }

        NimBLEDevice::startAdvertising();
        _enabled = true;
        Serial.println("[BLE] *** NimBLE Server Attivo: FZ8-Telemetry (visibile per Web App & RaceChrono) ***");
    }

    void stop() {
        if (!_enabled) return;
        NimBLEDevice::getAdvertising()->stop();
        _enabled = false;
        _deviceConnected = false;
        Serial.println("[BLE] NimBLE Advertising fermato.");
    }

    void toggle() {
        if (_enabled) stop();
        else begin();
    }

    void onConnect(NimBLEServer* pServer) override {
        _deviceConnected = true;
        Serial.println("[BLE] Smartphone connesso via NimBLE!");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        _deviceConnected = false;
        Serial.println("[BLE] Smartphone disconnesso.");
        if (_enabled) {
            NimBLEDevice::startAdvertising();
        }
    }

    void update(const MotorcycleDynamics& dyn, const DynamicAutoCalibrator& autoCal) {
        if (!_enabled || !_deviceConnected || !_pCharacteristic) return;

        uint32_t now = millis();
        if (now - _lastUpdateMillis < BLE_UPDATE_INTERVAL_MS) return;
        _lastUpdateMillis = now;

        // RaceChrono DIY NMEA Format v2 (20Hz):
        // $RC2,[time_ms],[count],[g_lat],[g_long],[roll_deg],[pitch_deg],[roll_rate],[yaw_rate],[altitude],[tps_pct],[autocal_pct],[autocal_samples],[heading],[cardinal],[head_valid],[d_plus],[temp]*[checksum]\r\n
        char sentence[200];
        int len = snprintf(sentence, sizeof(sentence),
            "$RC2,%lu,%lu,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%u,%lu,%.1f,%s,%d,%.1f,%.1f",
            (unsigned long)now,
            (unsigned long)_packetCounter++,
            dyn.gLateral,
            dyn.gLongitudinal,
            dyn.rollDeg,
            dyn.pitchDeg,
            dyn.rollRateDps,
            dyn.yawRateDps,
            dyn.altitudeM,
            dyn.tpsPercent,
            autoCal.getProgressPercent(),
            (unsigned long)autoCal.getStraightSamples(),
            dyn.headingDeg,
            dyn.cardinal[0] ? dyn.cardinal : "-",
            dyn.isHeadingValid ? 1 : 0,
            dyn.totalElevationGainM,
            dyn.tempC
        );

        // Calcola Checksum XOR standard NMEA
        uint8_t checksum = 0;
        for (int i = 1; i < len; i++) { // Salta '$'
            checksum ^= (uint8_t)sentence[i];
        }

        char fullPacket[210];
        int totalLen = snprintf(fullPacket, sizeof(fullPacket), "%s*%02X\r\n", sentence, checksum);

        _pCharacteristic->setValue((uint8_t*)fullPacket, totalLen);
        _pCharacteristic->notify();
    }

private:
    bool _enabled;
    bool _initialized;
    bool _deviceConnected;
    NimBLEServer* _pServer;
    NimBLECharacteristic* _pCharacteristic;
    NimBLECharacteristic* _pCmdCharacteristic;
    uint32_t _lastUpdateMillis;
    uint32_t _packetCounter;
    CommandCallback _cmdCallback;
};

inline void BleCmdCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0 && _parent) {
        String cmdStr = String(rxValue.c_str());
        cmdStr.trim();
        Serial.printf("[BLE CMD] Ricevuto comando: %s\n", cmdStr.c_str());
        _parent->handleIncomingCommand(cmdStr);
    }
}

#endif // BLUETOOTH_TELEMETRY_H
