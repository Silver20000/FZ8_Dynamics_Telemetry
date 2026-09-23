#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <Arduino.h>

// ==============================================================================
// ESP-NOW ULTRA-FAST BINARY TELEMETRY PACKET (32 Bytes)
// Broadcasted at 50Hz with <2.5ms latency to Cockpit Display (1.69" ST7789 / 1.28" GC9A01)
// ==============================================================================
struct __attribute__((packed)) EspNowTelemetryPacket {
    uint32_t timestampMs;
    int16_t  rollDegX10;        // e.g. -45.2 deg -> -452
    int16_t  pitchDegX10;       // e.g. +3.5 deg -> +35
    int16_t  gLateralX100;      // e.g. 1.25 G -> 125
    int16_t  gLongitudinalX100; // e.g. -0.85 G -> -85
    int16_t  rollRateDpsX10;    // e.g. 85.0 deg/s -> 850
    int16_t  yawRateDpsX10;     // e.g. 25.0 deg/s -> 250
    int16_t  totalGX100;        // e.g. 1.35 G -> 135
    int16_t  altitudeM;         // e.g. 450 m
    int16_t  maxLeanLeftX10;    // Peak Left lean
    int16_t  maxLeanRightX10;   // Peak Right lean
    int16_t  maxBrakingGX100;   // Peak braking G
    int16_t  maxAccelGX100;     // Peak accel G
    uint8_t  tpsPercent;        // 0 - 100% Throttle Position
    uint8_t  activeScreen;      // Remote screen index
    uint8_t  flags;             // bit0: isLeanWarn, bit1: isBraking, bit2: isAccel, bit3: isLogging
};

#endif // ESPNOW_PROTOCOL_H
