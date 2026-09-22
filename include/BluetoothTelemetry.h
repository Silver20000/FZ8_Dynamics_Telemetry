#ifndef BLUETOOTH_TELEMETRY_H
#define BLUETOOTH_TELEMETRY_H

#include <Arduino.h>
#include <NimBLEDevice.h>
#include "Config.h"
#include "MotorcycleFilter.h"

// RaceChrono DIY Service and Characteristic UUIDs
#define RACECHRONO_SERVICE_UUID        "00001ff8-0000-1000-8000-00805f9b34fb"
#define RACECHRONO_CHARACTERISTIC_UUID "00000002-0000-1000-8000-00805f9b34fb"

class BluetoothTelemetry : public NimBLEServerCallbacks {
public:
    BluetoothTelemetry() : 
        _enabled(false), 
        _initialized(false),
        _deviceConnected(false), 
        _pServer(nullptr), 
        _pCharacteristic(nullptr),
        _lastUpdateMillis(0),
        _packetCounter(0) {}

    bool isEnabled() const { return _enabled; }
    bool isConnected() const { return _deviceConnected; }

    void begin() {
        if (_enabled) return;

        if (!_initialized) {
            Serial.println("[BLE] Inizializzazione NimBLE (Lightweight Stack)...");
            NimBLEDevice::init(BLE_DEVICE_NAME);
            NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Massima potenza RF

            _pServer = NimBLEDevice::createServer();
            _pServer->setCallbacks(this);

            NimBLEService* pService = _pServer->createService(RACECHRONO_SERVICE_UUID);
            _pCharacteristic = pService->createCharacteristic(
                RACECHRONO_CHARACTERISTIC_UUID,
                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
            );
            pService->start();

            NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
            pAdvertising->addServiceUUID(RACECHRONO_SERVICE_UUID);
            pAdvertising->setScanResponse(true);
            _initialized = true;
        }

        NimBLEDevice::startAdvertising();
        _enabled = true;
        Serial.println("[BLE] *** NimBLE Server Attivo: FZ8-Telemetry (visibile per RaceChrono) ***");
    }

    void stop() {
        if (!_enabled) return;
        NimBLEDevice::getAdvertising()->stop();
        _enabled = false;
        _deviceConnected = false;
        Serial.println("[BLE] NimBLE Advertising fermato (Radio BLE Silente).");
    }

    void toggle() {
        if (_enabled) stop();
        else begin();
    }

    void onConnect(NimBLEServer* pServer) override {
        _deviceConnected = true;
        Serial.println("[BLE] Smartphone connesso via NimBLE (RaceChrono attivo)!");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        _deviceConnected = false;
        Serial.println("[BLE] Smartphone disconnesso.");
        if (_enabled) {
            NimBLEDevice::startAdvertising();
        }
    }

    void update(const MotorcycleDynamics& dyn) {
        if (!_enabled || !_deviceConnected || !_pCharacteristic) return;

        uint32_t now = millis();
        if (now - _lastUpdateMillis < BLE_UPDATE_INTERVAL_MS) return;
        _lastUpdateMillis = now;

        // RaceChrono DIY format:
        // $RC2,[time],[count],[x_acc],[y_acc],[z_acc],[roll],[pitch],[yaw_rate]*[checksum]\r\n
        char sentence[100];
        int len = snprintf(sentence, sizeof(sentence),
            "$RC2,,%lu,%lu,,%.2f,%.2f,%.1f,%.1f",
            (unsigned long)now,
            (unsigned long)_packetCounter++,
            dyn.gLateral,        // Lateral G (centripetal)
            dyn.gLongitudinal,   // Longitudinal G (+accel, -brake)
            dyn.rollDeg,         // Lean Angle
            dyn.pitchDeg         // Dive / Squat
        );

        // Compute XOR checksum
        uint8_t checksum = 0;
        for (int i = 1; i < len; i++) { // Skip '$'
            checksum ^= (uint8_t)sentence[i];
        }

        char fullPacket[120];
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
    uint32_t _lastUpdateMillis;
    uint32_t _packetCounter;
};

#endif // BLUETOOTH_TELEMETRY_H
