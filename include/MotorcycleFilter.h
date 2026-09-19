#ifndef MOTORCYCLE_FILTER_H
#define MOTORCYCLE_FILTER_H

#include <Arduino.h>
#include "Config.h"
#include "SensorsGY89.h"

// ==============================================================================
// MOTORCYCLE SELECTIVE GATED LEAN ANGLE & DYNAMICS FILTER
// ==============================================================================
// Solves the fundamental motorcycle physics paradox:
// In a coordinated turn, centripetal acceleration tilts the apparent gravity
// vector along the motorcycle's Z-axis (towards footpegs). Standard AHRS/Madgwick
// filters mistakenly pull roll back to 0 deg during prolonged corners.
//
// This filter implements an intelligent kinematic gate:
// - In turns (|wz| > threshold OR |Total G| > 1.08g OR |wx| > threshold):
//   Accelerometer roll correction is strictly 0%. The lean angle is held by
//   pure, high-frequency gyro integration.
// - On straights (|Total G| ~ 1g AND |wz| < threshold AND |ay| < threshold):
//   Gentle drift compensation aligns roll with true gravity and tracks bias.
// ==============================================================================

struct MotorcycleDynamics {
    float rollDeg;              // Instantaneous lean angle (negative = Left, positive = Right)
    float pitchDeg;             // Pitch angle (dive under braking / squat on accel)
    float yawRateDps;           // Angular yaw rate (deg/s)
    float rollRateDps;          // Roll transition speed (deg/s)
    
    float gLongitudinal;        // Longitudinal G (+ accel, - braking)
    float gLateral;             // Lateral G (centripetal / cornering force)
    float gVertical;            // Downward G into suspension
    float totalG;               // Magnitude of total acceleration vector

    bool isCornering;           // True when currently in a turn
    bool isBraking;             // True when braking (> 0.25G deceleration)
    bool isAccelerating;        // True when accelerating (> 0.20G)

    // Session Records
    float maxLeanLeft;          // Peak lean left (always >= 0 in degrees)
    float maxLeanRight;         // Peak lean right (always >= 0 in degrees)
    float maxBrakingG;          // Peak braking G (positive value, e.g. 0.95G)
    float maxAccelG;            // Peak acceleration G (positive value, e.g. 0.68G)
    
    float altitudeM;            // Current barometric altitude
    float minAltitudeM;         // Min elevation in session
    float maxAltitudeM;         // Max elevation in session
    float tempC;                // Ambient temperature
    uint32_t sessionStartTime;  // Timestamp session began
};

class MotorcycleFilter {
public:
    MotorcycleFilter() :
        _roll(0.0f), _pitch(0.0f),
        _tareOffset(0.0f),
        _straightCycles(0),
        _lastUpdateMicros(0),
        _firstRun(true) {
        resetRecords();
    }

    void resetRecords() {
        _dynamics.maxLeanLeft = 0.0f;
        _dynamics.maxLeanRight = 0.0f;
        _dynamics.maxBrakingG = 0.0f;
        _dynamics.maxAccelG = 0.0f;
        _dynamics.minAltitudeM = 9999.0f;
        _dynamics.maxAltitudeM = -9999.0f;
        _dynamics.sessionStartTime = millis();
    }

    void tareZero() {
        // Set current roll as mechanical zero reference (e.g. on paddock stand or rider level)
        _tareOffset = _roll;
        _roll = 0.0f;
    }

    void update(const IMURawData& raw, MotorcycleDynamics& outDynamics) {
        uint32_t nowMicros = micros();
        if (_firstRun) {
            _lastUpdateMicros = nowMicros;
            _firstRun = false;
            if (raw.altitude_m != 0.0f) {
                _dynamics.minAltitudeM = raw.altitude_m;
                _dynamics.maxAltitudeM = raw.altitude_m;
            }
            return;
        }

        float dt = (nowMicros - _lastUpdateMicros) * 1e-6f;
        _lastUpdateMicros = nowMicros;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // Clamp anomalous dt

        // 1. Calculate Acceleration Vector Magnitude
        float ax = raw.ax;
        float ay = raw.ay;
        float az = raw.az;
        float totalG = sqrtf(ax * ax + ay * ay + az * az);

        // 2. Gyro Rates
        float gx = raw.gx; // Roll rate (deg/s)
        float gy = raw.gy; // Pitch rate (deg/s)
        float gz = raw.gz; // Yaw rate (deg/s)

        // 3. Continuous Gyro Integration for Roll & Pitch
        _roll += gx * dt;
        _pitch += gy * dt;

        // 4. Motorcycle Kinematic Gate Check
        // A motorcycle in a turn has elevated total G (centripetal + gravity),
        // significant yaw rate (|gz| > 3 deg/s), or transient roll rate.
        bool isStraightG = (totalG >= GATE_TOTAL_G_MIN && totalG <= GATE_TOTAL_G_MAX);
        bool isLowYaw    = (fabsf(gz) < GATE_YAW_RATE_MAX);
        bool isLowRoll   = (fabsf(gx) < GATE_ROLL_RATE_MAX);
        bool isLowLat    = (fabsf(ay) < GATE_LATERAL_G_MAX);

        bool isSteadyStraight = isStraightG && isLowYaw && isLowRoll && isLowLat;

        if (isSteadyStraight) {
            _straightCycles++;
            // Require sustained straight movement before allowing accelerometer drift correction
            const uint16_t REQUIRED_CYCLES = (STRAIGHT_TIME_CONFIRM_MS / IMU_SAMPLE_PERIOD_MS);
            if (_straightCycles >= REQUIRED_CYCLES) {
                // Safe to apply gentle accelerometer gravity correction
                float accelRoll = atan2f(ay, az) * (180.0f / (float)M_PI);
                float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);

                // Gentle complementary alpha (99.8% gyro, 0.2% accel)
                _roll = 0.998f * _roll + 0.002f * accelRoll;
                _pitch = 0.995f * _pitch + 0.005f * accelPitch;

                _dynamics.isCornering = false;
            }
        } else {
            _straightCycles = 0;
            // CRITICAL MOTORCYCLE LOGIC:
            // Disconnect accelerometer roll correction completely during cornering!
            // Gyro integration maintains pure, uncorrupted roll angle through the bend.
            if (fabsf(gz) > GATE_YAW_RATE_MAX || totalG > GATE_TOTAL_G_MAX || fabsf(_roll - _tareOffset) > 5.0f) {
                _dynamics.isCornering = true;
            }
        }

        // Apply Tare Offset
        float effectiveRoll = _roll - _tareOffset;

        // 5. Dynamics State & G-Forces
        _dynamics.rollDeg = effectiveRoll;
        _dynamics.pitchDeg = _pitch;
        _dynamics.rollRateDps = gx;
        _dynamics.yawRateDps = gz;

        _dynamics.gLongitudinal = ax; // Forward accel / braking
        _dynamics.gLateral = ay;
        _dynamics.gVertical = az;
        _dynamics.totalG = totalG;

        _dynamics.isBraking = (ax < -0.25f);
        _dynamics.isAccelerating = (ax > 0.20f);

        // 6. Record Peak Lean Angles (Left vs Right)
        if (effectiveRoll < -0.5f) {
            float leftAngle = fabsf(effectiveRoll);
            if (leftAngle > _dynamics.maxLeanLeft) {
                _dynamics.maxLeanLeft = leftAngle;
            }
        } else if (effectiveRoll > 0.5f) {
            if (effectiveRoll > _dynamics.maxLeanRight) {
                _dynamics.maxLeanRight = effectiveRoll;
            }
        }

        // 7. Record Peak Longitudinal Gs
        if (ax < -_dynamics.maxBrakingG) {
            _dynamics.maxBrakingG = fabsf(ax);
        }
        if (ax > _dynamics.maxAccelG) {
            _dynamics.maxAccelG = ax;
        }

        // 8. Environmental / Barometer updates
        _dynamics.tempC = raw.temp_c;
        _dynamics.altitudeM = raw.altitude_m;
        if (raw.altitude_m > 0.0f) {
            if (raw.altitude_m < _dynamics.minAltitudeM) _dynamics.minAltitudeM = raw.altitude_m;
            if (raw.altitude_m > _dynamics.maxAltitudeM) _dynamics.maxAltitudeM = raw.altitude_m;
        }

        outDynamics = _dynamics;
    }

private:
    float _roll;
    float _pitch;
    float _tareOffset;
    uint16_t _straightCycles;
    uint32_t _lastUpdateMicros;
    bool _firstRun;
    MotorcycleDynamics _dynamics;
};

#endif // MOTORCYCLE_FILTER_H
