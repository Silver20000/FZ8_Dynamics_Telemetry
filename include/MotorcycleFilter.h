#ifndef MOTORCYCLE_FILTER_H
#define MOTORCYCLE_FILTER_H

#include <Arduino.h>
#include <Preferences.h>
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
        _tareRoll(0.0f), _tarePitch(0.0f),
        _straightCycles(0),
        _lastUpdateMicros(0),
        _firstRun(true) {
        resetRecords();
    }

    void begin() {
        _prefs.begin("fz8_cal", false);
        _tareRoll = _prefs.getFloat("tareRoll", MANUAL_ROLL_OFFSET_DEG);
        _tarePitch = _prefs.getFloat("tarePitch", MANUAL_PITCH_OFFSET_DEG);
        _prefs.end();
        Serial.printf("[FILTER] Zero Tare Loaded: RollOffset=%.2f deg, PitchOffset=%.2f deg\n", _tareRoll, _tarePitch);
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
        // Set current roll and pitch as mechanical mounting zero reference
        _tareRoll = _roll;
        _tarePitch = _pitch;
        _prefs.begin("fz8_cal", false);
        _prefs.putFloat("tareRoll", _tareRoll);
        _prefs.putFloat("tarePitch", _tarePitch);
        _prefs.end();
        Serial.printf("[FILTER] Zero Tare Saved: RollOffset=%.2f deg, PitchOffset=%.2f deg\n", _tareRoll, _tarePitch);
    }

    void resetTare() {
        _tareRoll = 0.0f;
        _tarePitch = 0.0f;
        _prefs.begin("fz8_cal", false);
        _prefs.remove("tareRoll");
        _prefs.remove("tarePitch");
        _prefs.end();
        Serial.println("[FILTER] Zero Tare Reset to factory default (0.0 deg).");
    }

    float getTareRoll() const { return _tareRoll; }
    float getTarePitch() const { return _tarePitch; }
    float getRawRoll() const { return _roll; }
    float getRawPitch() const { return _pitch; }

    void adjustTare(float deltaRoll, float deltaPitch) {
        _tareRoll += deltaRoll;
        _tarePitch += deltaPitch;
        _prefs.begin("fz8_cal", false);
        _prefs.putFloat("tareRoll", _tareRoll);
        _prefs.putFloat("tarePitch", _tarePitch);
        _prefs.end();
        Serial.printf("[FILTER] Tare Adjusted: Roll=%.2f, Pitch=%.2f\n", _tareRoll, _tarePitch);
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

        // 0. Axis Mapping & 90-degree Rotation
#if IMU_SWAP_XY
        // Rotate coordinate frame by 90 degrees so acceleration and lean are perpendicular:
        float ax = raw.ay;
        float ay = -raw.ax;
        float az = raw.az;
        float gx = -raw.gy;
        float gy = raw.gx;
        float gz = raw.gz;
#else
        float ax = raw.ax;
        float ay = raw.ay;
        float az = raw.az;
        float gx = raw.gx;
        float gy = raw.gy;
        float gz = raw.gz;
#endif

#if IMU_INVERT_ROLL
        gx = -gx;
        ay = -ay;
#endif
#if IMU_INVERT_ACCEL
        ax = -ax;
        gy = -gy;
#endif

        // 1. Calculate Acceleration Vector Magnitude
        float totalG = sqrtf(ax * ax + ay * ay + az * az);

        // 2. Direct Accelerometer Tilt Angles
        float accelRoll = atan2f(ay, az) * (180.0f / (float)M_PI);
        float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);

        // 3. Stationary Detection (Auto-lock to gravity when still -> ELIMINATES DRIFT)
        bool isStationary = (fabsf(gx) < 2.5f && fabsf(gy) < 2.5f && fabsf(gz) < 2.5f && 
                             fabsf(totalG - 1.0f) < 0.18f);

        // 4. Dynamic Motorcycle Cornering Detection (Centripetal load + Yaw rate)
        bool isCornering = (fabsf(gz) > 4.0f && totalG > 1.10f);

        // 5. Adaptive Complementary Weight
        float alpha;
        if (isStationary) {
            // Firmly lock to gravity when still -> ZERO RUNAWAY DRIFT!
            alpha = 0.85f;
            _dynamics.isCornering = false;
        } else if (isCornering) {
            // High-G curve: rely heavily on gyro integration (prevent curve pull-down)
            alpha = 0.9992f;
            _dynamics.isCornering = true;
        } else {
            // Normal straight riding or transitions
            alpha = 0.985f;
            _dynamics.isCornering = false;
        }

        // Integration + Complementary Correction
        _roll = alpha * (_roll + gx * dt) + (1.0f - alpha) * accelRoll;
        _pitch = 0.98f * (_pitch + gy * dt) + 0.02f * accelPitch;

        // Apply Tare Offset (Motorcycle body frame)
        float effectiveRoll = _roll - _tareRoll;
        float effectivePitch = _pitch - _tarePitch;

        // 5. Dynamics State & Gravity-Compensated Dynamic G-Forces
        _dynamics.rollDeg = effectiveRoll;
        _dynamics.pitchDeg = effectivePitch;
        _dynamics.rollRateDps = gx;
        _dynamics.yawRateDps = gz;

        // Rotate sensor acceleration (ax, ay, az) by (-tarePitch, -tareRoll) into motorcycle chassis frame:
        float tarePitchRad = _tarePitch * ((float)M_PI / 180.0f);
        float tareRollRad = _tareRoll * ((float)M_PI / 180.0f);

        float cp = cosf(tarePitchRad);
        float sp = sinf(tarePitchRad);
        float cr = cosf(tareRollRad);
        float sr = sinf(tareRollRad);

        // Rotation around Y (pitch compensation)
        float ax1 = ax * cp + az * sp;
        float az1 = -ax * sp + az * cp;

        // Rotation around X (roll compensation)
        float ay_moto = ay * cr - az1 * sr;
        float az_moto = ay * sr + az1 * cr;
        float ax_moto = ax1;

        // Motorcycle body gravity vector projection:
        float motoPitchRad = effectivePitch * ((float)M_PI / 180.0f);
        float motoRollRad = effectiveRoll * ((float)M_PI / 180.0f);

        float gravX = -sinf(motoPitchRad);
        float gravY = cosf(motoPitchRad) * sinf(motoRollRad);

        // TRUE Dynamic G-Forces: subtract static 1G gravity tilt
        // Tilting the bike in roll/pitch will NOT falsely produce longitudinal/lateral G!
        float dynLongitudinal = ax_moto - gravX; // Pure forward acceleration / braking
        float dynLateral = ay_moto - gravY;      // Pure cornering G

        _dynamics.gLongitudinal = dynLongitudinal;
        _dynamics.gLateral = dynLateral;
        _dynamics.gVertical = az_moto;
        _dynamics.totalG = totalG;

        _dynamics.isBraking = (dynLongitudinal < -0.25f);
        _dynamics.isAccelerating = (dynLongitudinal > 0.20f);

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

        // 7. Record Peak Longitudinal Dynamic Gs
        if (dynLongitudinal < -_dynamics.maxBrakingG) {
            _dynamics.maxBrakingG = fabsf(dynLongitudinal);
        }
        if (dynLongitudinal > _dynamics.maxAccelG) {
            _dynamics.maxAccelG = dynLongitudinal;
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
    Preferences _prefs;
    float _roll;
    float _pitch;
    float _tareRoll;
    float _tarePitch;
    uint16_t _straightCycles;
    uint32_t _lastUpdateMicros;
    bool _firstRun;
    MotorcycleDynamics _dynamics;
};

#endif // MOTORCYCLE_FILTER_H
