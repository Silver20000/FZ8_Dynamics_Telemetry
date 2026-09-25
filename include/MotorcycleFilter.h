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

enum class BikeState : uint8_t {
    STATIONARY = 0,
    STRAIGHT   = 1,
    DYNAMIC    = 2
};

struct CornerRecord {
    bool isLeft;
    float maxRollDeg;
    float maxRollRateDps;
    float durationSec;
    uint32_t timestampMs;
};

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

    // TPS Throttle
    float tpsPercent;           // Throttle position (0.0 to 100.0%)
    uint16_t tpsRawAdc;         // Raw ADC value from TPS pin

    // Heading & Compass
    float headingDeg;           // 0 - 360 degrees
    char cardinal[4];           // N, NE, E, SE, S, SW, W, NW
    bool isHeadingValid;        // True when bike is upright and riding straight (reliable)

    // Mountain & Elevation
    float totalElevationGainM;  // D+ accumulated elevation gain (m)
    float verticalSpeedMps;     // Vertical speed (m/s)
    float roadGradientPct;      // Road slope gradient (%)

    // Corner Analyzer
    CornerRecord lastCorners[5];// Historical records of last 5 corners
    uint8_t cornerHistoryCount; // Number of corners in history (0-5)
    bool isInsideCorner;        // True if currently carving a corner
    float currentCornerMaxRoll; // Peak lean in active corner
    float currentCornerDuration;// Duration in seconds of active corner

    // Safety Alarms
    bool isLeanWarning;         // True if exceeding lean threshold
    bool isCrashDetected;       // True if bike on ground stationary
    float maxLeanThreshold;     // Configurable threshold (e.g. 48.0 deg)

    // Session Records
    float maxLeanLeft;          // Peak lean left (always >= 0 in degrees)
    float maxLeanRight;         // Peak lean right (always >= 0 in degrees)
    float maxBrakingG;          // Peak braking G (positive value, e.g. 0.95G)
    float maxAccelG;            // Peak acceleration G (positive value, e.g. 0.68G)
    float maxPitchUp;           // Peak wheelie / squat pitch angle (degrees >= 0)
    float maxPitchDown;         // Peak dive / stoppie pitch angle (degrees <= 0)
    bool isWheelie;             // True if pitch > +6.5 deg
    bool isStoppie;             // True if pitch < -5.5 deg and braking
    
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
        _firstRun(true),
        _swapXY(IMU_SWAP_XY),
        _invertRoll(IMU_INVERT_ROLL),
        _invertAccel(IMU_INVERT_ACCEL),
        _maxLeanThreshold(DEFAULT_MAX_LEAN_WARN_DEG),
        _totalElevationGainM(0.0f),
        _filteredAltM(0.0f),
        _lastAltM(0.0f),
        _lastAltTimeMs(0),
        _verticalSpeedMps(0.0f),
        _inCorner(false),
        _cornerExitPending(false),
        _cornerExitPendingStartTime(0),
        _cornerStartTime(0),
        _cornerMaxRoll(0.0f),
        _cornerMaxRate(0.0f),
        _cornerIsLeft(false),
        _crashStartTime(0),
        _gyroBiasRollDps(0.0f),
        _gyroBiasPitchDps(0.0f),
        _straightConfirmMs(0),
        _lastStraightStartMs(0),
        _filteredTotalG(1.0f),
        _yawAlignDeg(IMU_YAW_ALIGN_DEG),
        _lastState(BikeState::DYNAMIC),
        _filterState(2),
        _magCalActive(false),
        _magCalStartMs(0),
        _magMinX(999.0f), _magMaxX(-999.0f),
        _magMinY(999.0f), _magMaxY(-999.0f),
        _magMinZ(999.0f), _magMaxZ(-999.0f),
        _magOffsetX(0.0f), _magOffsetY(0.0f), _magOffsetZ(0.0f),
        _magScaleX(1.0f), _magScaleY(1.0f), _magScaleZ(1.0f) {
        resetRecords();
    }

    void begin() {
        _prefs.begin("fz8_cal", false);
        _tareRoll = _prefs.getFloat("tareRoll", MANUAL_ROLL_OFFSET_DEG);
        _tarePitch = _prefs.getFloat("tarePitch", MANUAL_PITCH_OFFSET_DEG);
        _yawAlignDeg = _prefs.getFloat("yawAlign", IMU_YAW_ALIGN_DEG);
        _swapXY = _prefs.getBool("swapXY", IMU_SWAP_XY);
        _invertRoll = _prefs.getBool("invRoll", IMU_INVERT_ROLL);
        _invertAccel = _prefs.getBool("invAcc", IMU_INVERT_ACCEL);
        _maxLeanThreshold = _prefs.getFloat("maxLeanWarn", DEFAULT_MAX_LEAN_WARN_DEG);
        _magOffsetX = _prefs.getFloat("magOffX", 0.0f);
        _magOffsetY = _prefs.getFloat("magOffY", 0.0f);
        _magOffsetZ = _prefs.getFloat("magOffZ", 0.0f);
        _magScaleX  = _prefs.getFloat("magScaleX", 1.0f);
        _magScaleY  = _prefs.getFloat("magScaleY", 1.0f);
        _magScaleZ  = _prefs.getFloat("magScaleZ", 1.0f);

        // Sanity check on tare: reset if anomalous/corrupted from previous drift
        if (fabsf(_tareRoll) > 45.0f) {
            Serial.printf("[FILTER] TareRoll (%.1f) anomalo rilevato! Resettato a 0.0\n", _tareRoll);
            _tareRoll = 0.0f;
            _prefs.putFloat("tareRoll", 0.0f);
        }
        if (fabsf(_tarePitch) > 30.0f) {
            _tarePitch = 0.0f;
            _prefs.putFloat("tarePitch", 0.0f);
        }
        _prefs.end();

        _dynamics.maxLeanThreshold = _maxLeanThreshold;
        Serial.printf("[FILTER] Loaded: RollOffset=%.2f deg, PitchOffset=%.2f deg | swapXY=%d, invRoll=%d, leanWarn=%.1f\n", 
                      _tareRoll, _tarePitch, _swapXY, _invertRoll, _maxLeanThreshold);
        Serial.printf("[FILTER] Mag Calibration: Offset=(%.3f, %.3f, %.3f) Scale=(%.2f, %.2f, %.2f)\n",
                      _magOffsetX, _magOffsetY, _magOffsetZ, _magScaleX, _magScaleY, _magScaleZ);
    }

    void resetRecords() {
        _dynamics.maxLeanLeft = 0.0f;
        _dynamics.maxLeanRight = 0.0f;
        _dynamics.maxBrakingG = 0.0f;
        _dynamics.maxAccelG = 0.0f;
        _dynamics.maxPitchUp = 0.0f;
        _dynamics.maxPitchDown = 0.0f;
        _dynamics.isWheelie = false;
        _dynamics.isStoppie = false;
        _dynamics.minAltitudeM = 9999.0f;
        _dynamics.maxAltitudeM = -9999.0f;
        _dynamics.sessionStartTime = millis();

        _totalElevationGainM = 0.0f;
        _dynamics.totalElevationGainM = 0.0f;
        _dynamics.verticalSpeedMps = 0.0f;
        _dynamics.roadGradientPct = 0.0f;
        _dynamics.cornerHistoryCount = 0;
        _dynamics.isInsideCorner = false;
        _dynamics.isLeanWarning = false;
        _dynamics.isCrashDetected = false;
        memset(_dynamics.lastCorners, 0, sizeof(_dynamics.lastCorners));
        _inCorner = false;
    }

    void setMaxLeanThreshold(float deg) {
        if (deg < 30.0f) deg = 30.0f;
        if (deg > 60.0f) deg = 60.0f;
        _maxLeanThreshold = deg;
        _dynamics.maxLeanThreshold = deg;
        _prefs.begin("fz8_cal", false);
        _prefs.putFloat("maxLeanWarn", _maxLeanThreshold);
        _prefs.end();
        Serial.printf("[FILTER] Max lean warning threshold set to: %.1f deg\n", _maxLeanThreshold);
    }

    float getMaxLeanThreshold() const { return _maxLeanThreshold; }

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

    // Magnetometer Calibration Controls
    void startMagCal() {
        _magCalActive = true;
        _magCalStartMs = millis();
        _magMinX = 999.0f; _magMaxX = -999.0f;
        _magMinY = 999.0f; _magMaxY = -999.0f;
        _magMinZ = 999.0f; _magMaxZ = -999.0f;
        Serial.println("[MAG CAL] *** CALIBRAZIONE BUSSOLA AVVIATA (15s) ***");
        Serial.println("[MAG CAL] Ruota lentamente la schedina a 360 gradi su tutti gli assi...");
    }

    void finishMagCal() {
        if (!_magCalActive) return;
        _magCalActive = false;
        float spanX = _magMaxX - _magMinX;
        float spanY = _magMaxY - _magMinY;
        float spanZ = _magMaxZ - _magMinZ;
        if (spanX > 0.08f && spanY > 0.08f) {
            _magOffsetX = (_magMaxX + _magMinX) * 0.5f;
            _magOffsetY = (_magMaxY + _magMinY) * 0.5f;
            _magOffsetZ = (spanZ > 0.08f) ? ((_magMaxZ + _magMinZ) * 0.5f) : 0.0f;
            float avgSpan = (spanX + spanY) * 0.5f;
            _magScaleX = avgSpan / spanX;
            _magScaleY = avgSpan / spanY;
            _magScaleZ = (spanZ > 0.08f) ? (avgSpan / spanZ) : 1.0f;

            _prefs.begin("fz8_cal", false);
            _prefs.putFloat("magOffX", _magOffsetX);
            _prefs.putFloat("magOffY", _magOffsetY);
            _prefs.putFloat("magOffZ", _magOffsetZ);
            _prefs.putFloat("magScaleX", _magScaleX);
            _prefs.putFloat("magScaleY", _magScaleY);
            _prefs.putFloat("magScaleZ", _magScaleZ);
            _prefs.end();
            Serial.printf("[MAG CAL] SALVATA! Offset=(%.3f, %.3f, %.3f) Scale=(%.2f, %.2f, %.2f)\n",
                          _magOffsetX, _magOffsetY, _magOffsetZ, _magScaleX, _magScaleY, _magScaleZ);
        } else {
            Serial.printf("[MAG CAL] Annullata o escursione insufficiente (spanX=%.3f, spanY=%.3f).\n", spanX, spanY);
        }
    }

    bool isMagCalibrating() const { return _magCalActive; }
    uint32_t getMagCalRemainingSec() const {
        if (!_magCalActive) return 0;
        uint32_t elapsed = (millis() - _magCalStartMs) / 1000;
        return (elapsed >= 15) ? 0 : (15 - elapsed);
    }
    void getMagCalibration(float& offX, float& offY, float& offZ, float& scX, float& scY, float& scZ) const {
        offX = _magOffsetX; offY = _magOffsetY; offZ = _magOffsetZ;
        scX = _magScaleX; scY = _magScaleY; scZ = _magScaleZ;
    }
    float getRawPitch() const { return _pitch; }
    float getAccelRoll() const { return _lastAccelRoll; }
    float getAccelPitch() const { return _lastAccelPitch; }
    float getFilteredTotalG() const { return _filteredTotalG; }
    float getGyroBiasRoll() const { return _gyroBiasRollDps; }
    float getGyroBiasPitch() const { return _gyroBiasPitchDps; }
    BikeState getState() const { return _lastState; }
    uint8_t getFilterState() const { return _filterState; }
    float getYawAlign() const { return _yawAlignDeg; }

    void setYawAlign(float deg) {
        _yawAlignDeg = deg;
        _prefs.begin("fz8_cal", false);
        _prefs.putFloat("yawAlign", _yawAlignDeg);
        _prefs.end();
        Serial.printf("[FILTER] Yaw Align Salvato: %+.2f deg\n", _yawAlignDeg);
    }

    void adjustYawAlign(float deltaDeg) {
        setYawAlign(_yawAlignDeg + deltaDeg);
    }

    void resetYawAlign() {
        setYawAlign(IMU_YAW_ALIGN_DEG);
    }

    bool getSwapXY() const { return _swapXY; }
    bool getInvertRoll() const { return _invertRoll; }
    bool getInvertAccel() const { return _invertAccel; }

    void toggleSwapXY() {
        _swapXY = !_swapXY;
        _prefs.begin("fz8_cal", false);
        _prefs.putBool("swapXY", _swapXY);
        _prefs.end();
        Serial.printf("[FILTER] swapXY set to: %d\n", _swapXY);
    }

    void toggleInvertRoll() {
        _invertRoll = !_invertRoll;
        _prefs.begin("fz8_cal", false);
        _prefs.putBool("invRoll", _invertRoll);
        _prefs.end();
        Serial.printf("[FILTER] invertRoll set to: %d\n", _invertRoll);
    }

    void toggleInvertAccel() {
        _invertAccel = !_invertAccel;
        _prefs.begin("fz8_cal", false);
        _prefs.putBool("invAcc", _invertAccel);
        _prefs.end();
        Serial.printf("[FILTER] invertAccel set to: %d\n", _invertAccel);
    }

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
        float dt = (nowMicros - _lastUpdateMicros) * 1e-6f;
        _lastUpdateMicros = nowMicros;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.01f; // Clamp anomalous dt
        uint32_t nowMs = nowMicros / 1000;

        // 0. Axis Mapping & 90-degree Rotation
        float ax, ay, az, gx, gy, gz;
        if (_swapXY) {
            ax = -raw.ax;
            ay = -raw.ay;
            az = raw.az;
            gx = raw.gy;
            gy = raw.gx;
            gz = raw.gz;
        } else {
            ax = raw.ax;
            ay = raw.ay;
            az = raw.az;
            gx = raw.gx;
            gy = raw.gy;
            gz = raw.gz;
        }

        if (_invertRoll) {
            gx = -gx;
            ay = -ay;
        }
        if (_invertAccel) {
            ax = -ax;
            gy = -gy;
        }

        // Pure accelerometer tilt angles
        float initAccelRoll = atan2f(ay, az) * (180.0f / (float)M_PI);
        float initAccelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);

        if (_firstRun) {
            _lastUpdateMicros = nowMicros;
            _firstRun = false;
            _lastStraightStartMs = millis();
            _roll = initAccelRoll;
            _pitch = initAccelPitch;
            if (raw.altitude_m != 0.0f) {
                _dynamics.minAltitudeM = raw.altitude_m;
                _dynamics.maxAltitudeM = raw.altitude_m;
            }
            return;
        }

        // 0b. Disaccoppiamento Rotazionale Orizzontale (Yaw Trim / Allineamento Telaio)
        // Elimina matematicamente il cross-coupling tra Rollio e Beccheggio
        if (fabsf(_yawAlignDeg) > 0.01f) {
            float rad = _yawAlignDeg * ((float)M_PI / 180.0f);
            float cosY = cosf(rad);
            float sinY = sinf(rad);

            float ax_rot = ax * cosY - ay * sinY;
            float ay_rot = ax * sinY + ay * cosY;
            ax = ax_rot;
            ay = ay_rot;

            float gx_rot = gx * cosY - gy * sinY;
            float gy_rot = gx * sinY + gy * cosY;
            gx = gx_rot;
            gy = gy_rot;
        }

        // 1. Modulo accelerazione istantaneo e filtrato passa-basso IIR a 1 polo
        // Taglia rumore picco-picco ad alta frequenza da vibrazioni motore e rugosità asfalto
        float rawTotalG = sqrtf(ax * ax + ay * ay + az * az);
        _filteredTotalG = 0.90f * _filteredTotalG + 0.10f * rawTotalG;

        // 2. Compensazione Bias Giroscopio su asse Roll (X) e Pitch (Y)
        float gxCorrected = gx - _gyroBiasRollDps;
        float gyCorrected = gy - _gyroBiasPitchDps;

        // 3. Risoluzione Angolo Accelerometro puro
        float accelRoll = atan2f(ay, az) * (180.0f / (float)M_PI);
        _lastAccelRoll = accelRoll;
        float accelPitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);
        _lastAccelPitch = accelPitch;

        // 4. Modulo rotazione angolare totale (|omega|)
        float totalOmega = sqrtf(gx * gx + gy * gy + gz * gz);

        // =====================================================================
        // 5. RICONOSCIMENTO STATO (Gating a 3 Stati Anti-Trappola Curva Coordinata)
        // =====================================================================
        // Quando la moto o schedina è ferma (totalOmega < 2.5 deg/s e gravità pura ~1g),
        // NON c'è accelerazione centripeta o di marcia: siamo SEMPRE in STATIONARY!
        bool isStationary = (totalOmega < 2.5f && fabsf(_filteredTotalG - 1.0f) < 0.10f);

        bool lowRotation = (totalOmega < GATE_GYRO_MAX_DPS);
        bool validGNorm  = (_filteredTotalG >= GATE_TOTAL_G_MIN && _filteredTotalG <= GATE_TOTAL_G_MAX);
        bool lowLongAcc  = (fabsf(ax) < GATE_AX_MAX);
        bool lowLatAcc   = (fabsf(ay) < GATE_AY_MAX);
        float currentEffectiveRoll = _roll - _tareRoll;
        bool isNearUpright = (fabsf(currentEffectiveRoll) < 6.0f);

        BikeState currentState = BikeState::DYNAMIC;

        if (isStationary) {
            currentState = BikeState::STATIONARY;
            _straightConfirmMs = 0;
        } else if (lowRotation && validGNorm && lowLongAcc && lowLatAcc && isNearUpright) {
            _straightConfirmMs += static_cast<uint32_t>(dt * 1000.0f);
            if (_straightConfirmMs >= STRAIGHT_HOLD_MS) {
                currentState = BikeState::STRAIGHT;
            } else {
                currentState = BikeState::DYNAMIC;
            }
        } else {
            currentState = BikeState::DYNAMIC;
            _straightConfirmMs = 0;
        }

        // =====================================================================
        // 6. ESECUZIONE FILTRO COMPLEMENTARE & BIAS TRACKER PROTETTO
        // =====================================================================
        float alpha = ALPHA_DYNAMIC;

        switch (currentState) {
            case BikeState::STATIONARY:
                alpha = ALPHA_STATIONARY; // 0.85f: rapido riallineamento all'accelerometro a moto ferma
                _dynamics.isCornering = false;
                // Da fermo aggiorniamo il bias per azzerare completamente il drift giroscopico
                _gyroBiasRollDps += (gx - _gyroBiasRollDps) * 0.02f;
                _gyroBiasPitchDps += (gy - _gyroBiasPitchDps) * 0.02f;
                break;

            case BikeState::STRAIGHT:
                alpha = ALPHA_STRAIGHT;
                _dynamics.isCornering = false;
                // Tracking lento del bias: correggiamo solo l'errore sistematico residuo
                {
                    float rollError = accelRoll - _roll;
                    // Se l'errore è contenuto (< 5 gradi), è drift; altrimenti è pendenza/schiena d'asino
                    if (fabsf(rollError) < 5.0f) {
                        _gyroBiasRollDps -= (rollError * dt) * BIAS_LEARN_RATE;
                        // Limitazione clamp anti-windup
                        if (_gyroBiasRollDps > BIAS_MAX_DPS_LIMIT) _gyroBiasRollDps = BIAS_MAX_DPS_LIMIT;
                        if (_gyroBiasRollDps < -BIAS_MAX_DPS_LIMIT) _gyroBiasRollDps = -BIAS_MAX_DPS_LIMIT;
                    }
                    float pitchError = accelPitch - _pitch;
                    if (fabsf(pitchError) < 5.0f) {
                        _gyroBiasPitchDps -= (pitchError * dt) * BIAS_LEARN_RATE;
                        if (_gyroBiasPitchDps > BIAS_MAX_DPS_LIMIT) _gyroBiasPitchDps = BIAS_MAX_DPS_LIMIT;
                        if (_gyroBiasPitchDps < -BIAS_MAX_DPS_LIMIT) _gyroBiasPitchDps = -BIAS_MAX_DPS_LIMIT;
                    }
                }
                break;

            case BikeState::DYNAMIC:
            default:
                alpha = ALPHA_DYNAMIC; // 1.0f: esclude interamente l'accelerometro
                _dynamics.isCornering = (fabsf(gz) > 4.0f || _filteredTotalG > 1.08f || fabsf(currentEffectiveRoll) > 8.0f);
                break;
        }

        _filterState = static_cast<uint8_t>(currentState);
        _lastState = currentState;

        // 7. Integrazione stato Roll e Pitch con Disaccoppiamento Cinematico 3D di Eulero
        // dTheta = gy * cos(roll) - gz * sin(roll) annulla la proiezione di imbardata sul beccheggio
        float rollRad = _roll * ((float)M_PI / 180.0f);
        float pitchRate = gyCorrected * cosf(rollRad) - gz * sinf(rollRad);

        _roll = alpha * (_roll + gxCorrected * dt) + (1.0f - alpha) * accelRoll;
        _pitch = alpha * (_pitch + pitchRate * dt) + (1.0f - alpha) * accelPitch;

        // Limitazione di sicurezza fisica per prevenire runaway dell'integratore
        if (_roll > 75.0f) _roll = 75.0f;
        if (_roll < -75.0f) _roll = -75.0f;
        if (_pitch > 55.0f) _pitch = 55.0f;
        if (_pitch < -55.0f) _pitch = -55.0f;

        // Apply Tare Offset (Motorcycle body frame)
        float effectiveRoll = _roll - _tareRoll;
        float effectivePitch = _pitch - _tarePitch;

        // 5. Dynamics State & Gravity-Compensated Dynamic G-Forces
        _dynamics.rollDeg = effectiveRoll;
        _dynamics.pitchDeg = effectivePitch;
        _dynamics.rollRateDps = gxCorrected;
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
        float dynLongitudinal = ax_moto - gravX;
        float dynLateral = ay_moto - gravY;

        _dynamics.gLongitudinal = dynLongitudinal;
        _dynamics.gLateral = dynLateral;
        _dynamics.gVertical = az_moto;
        _dynamics.totalG = _filteredTotalG;

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

        // 7b. Record Peak Pitch Attitudes (Wheelie & Dive)
        if (effectivePitch > _dynamics.maxPitchUp) {
            _dynamics.maxPitchUp = effectivePitch;
        }
        if (effectivePitch < _dynamics.maxPitchDown) {
            _dynamics.maxPitchDown = effectivePitch;
        }
        _dynamics.isWheelie = (effectivePitch >= 6.5f);
        _dynamics.isStoppie = (effectivePitch <= -5.5f && _dynamics.isBraking);

        // 8. Environmental / Barometer updates
        _dynamics.tempC = raw.temp_c;
        _dynamics.altitudeM = raw.altitude_m;
        if (raw.altitude_m > 0.0f) {
            if (raw.altitude_m < _dynamics.minAltitudeM) _dynamics.minAltitudeM = raw.altitude_m;
            if (raw.altitude_m > _dynamics.maxAltitudeM) _dynamics.maxAltitudeM = raw.altitude_m;
        }

        // 9. Heading & Compass with Kinematic Gating & Hard-Iron Calibration
        // Adaptive min/max tracker for auto-calibration and active calibration
        if (_magCalActive || (_magOffsetX == 0.0f && _magOffsetY == 0.0f)) {
            if (raw.mx < _magMinX) _magMinX = raw.mx;
            if (raw.mx > _magMaxX) _magMaxX = raw.mx;
            if (raw.my < _magMinY) _magMinY = raw.my;
            if (raw.my > _magMaxY) _magMaxY = raw.my;
            if (raw.mz < _magMinZ) _magMinZ = raw.mz;
            if (raw.mz > _magMaxZ) _magMaxZ = raw.mz;

            float spanX = _magMaxX - _magMinX;
            float spanY = _magMaxY - _magMinY;
            if (!_magCalActive && spanX > 0.15f && spanY > 0.15f) {
                // Initial auto-seed so compass works right out of the box even before manual calibration
                _magOffsetX = (_magMaxX + _magMinX) * 0.5f;
                _magOffsetY = (_magMaxY + _magMinY) * 0.5f;
                if (_magMaxZ - _magMinZ > 0.15f) _magOffsetZ = (_magMaxZ + _magMinZ) * 0.5f;
            }
        }

        if (_magCalActive && (millis() - _magCalStartMs >= 15000)) {
            finishMagCal();
        }

        // Apply Hard-Iron Offset & Soft-Iron Scaling
        float calMx = (raw.mx - _magOffsetX) * _magScaleX;
        float calMy = (raw.my - _magOffsetY) * _magScaleY;
        float calMz = (raw.mz - _magOffsetZ) * _magScaleZ;

        // Transform Magnetometer to Motorcycle Body Coordinates
        // With USB-C pointing towards motorcycle front (_swapXY == true):
        //   Longitudinal Forward = +Y axis of sensor (calMy)
        //   Lateral Right        = +X axis of sensor (calMx)
        //   Vertical Up          = +Z axis of sensor (calMz)
        float mx_body, my_body, mz_body;
        if (_swapXY) {
            mx_body = calMy;
            my_body = calMx;
            mz_body = calMz;
        } else {
            mx_body = calMx;
            my_body = calMy;
            mz_body = calMz;
        }

        SensorsGY89::computeTiltCompensatedHeading(effectiveRoll, effectivePitch, 
                                                  _dynamics.gLateral, _dynamics.yawRateDps,
                                                  mx_body, my_body, mz_body, 
                                                  _dynamics.headingDeg, _dynamics.cardinal,
                                                  _dynamics.isHeadingValid);

        // 10. Advanced Altimetry: Dynamic Low-Pass IIR Filter & Deadband D+ Accumulator
        if (raw.altitude_m > 0.0f) {
            if (_filteredAltM == 0.0f) {
                _filteredAltM = raw.altitude_m;
                _lastAltM = raw.altitude_m;
                _lastAltTimeMs = nowMs;
            } else {
                // Reject cockpit aerodynamic dynamic pressure fluctuation with low-pass IIR
                _filteredAltM = (1.0f - BARO_ALT_ALPHA) * _filteredAltM + BARO_ALT_ALPHA * raw.altitude_m;
            }
            _dynamics.altitudeM = _filteredAltM;

            if (nowMs - _lastAltTimeMs >= 500) {
                float dtSec = (nowMs - _lastAltTimeMs) * 0.001f;
                float dAlt = _filteredAltM - _lastAltM;
                // Deadband of 2.0 meters rejects air stream buffeting / Pitot-like pressure variations
                if (dAlt >= ELEVATION_GAIN_DEADBAND_M) {
                    _totalElevationGainM += dAlt;
                    _lastAltM = _filteredAltM;
                } else if (dAlt <= -ELEVATION_GAIN_DEADBAND_M) {
                    _lastAltM = _filteredAltM;
                }
                float vz = (dtSec > 0.0f) ? (dAlt / dtSec) : 0.0f;
                _verticalSpeedMps = 0.90f * _verticalSpeedMps + 0.10f * vz;
                _lastAltTimeMs = nowMs;

                // Suspension dive/squat attitude angle (replaces fake road slope)
                _dynamics.roadGradientPct = effectivePitch;
            }
        }
        _dynamics.totalElevationGainM = _totalElevationGainM;
        _dynamics.verticalSpeedMps = _verticalSpeedMps;

        // 11. Corner Analyzer State Machine with 250ms Chicane & Exit Hysteresis
        float absRoll = fabsf(effectiveRoll);
        if (!_inCorner) {
            if (absRoll >= 12.0f) {
                _inCorner = true;
                _cornerExitPending = false;
                _cornerStartTime = nowMs;
                _cornerMaxRoll = absRoll;
                _cornerMaxRate = fabsf(gx);
                _cornerIsLeft = (effectiveRoll < 0.0f);
            }
            _dynamics.isInsideCorner = false;
        } else {
            // Actively in corner
            if (absRoll > _cornerMaxRoll) _cornerMaxRoll = absRoll;
            if (fabsf(gx) > _cornerMaxRate) _cornerMaxRate = fabsf(gx);

            // Chicane / rapid direction flip without stopping
            bool curLeft = (effectiveRoll < 0.0f);
            if (absRoll >= 12.0f && curLeft != _cornerIsLeft) {
                float duration = (nowMs - _cornerStartTime) * 0.001f;
                if (duration >= 0.4f) {
                    for (int i = 4; i > 0; i--) {
                        _dynamics.lastCorners[i] = _dynamics.lastCorners[i - 1];
                    }
                    _dynamics.lastCorners[0].isLeft = _cornerIsLeft;
                    _dynamics.lastCorners[0].maxRollDeg = _cornerMaxRoll;
                    _dynamics.lastCorners[0].maxRollRateDps = _cornerMaxRate;
                    _dynamics.lastCorners[0].durationSec = duration;
                    _dynamics.lastCorners[0].timestampMs = nowMs;
                    if (_dynamics.cornerHistoryCount < 5) _dynamics.cornerHistoryCount++;
                }
                // Transition to opposite lean immediately
                _cornerStartTime = nowMs;
                _cornerMaxRoll = absRoll;
                _cornerMaxRate = fabsf(gx);
                _cornerIsLeft = curLeft;
                _cornerExitPending = false;
            }

            _dynamics.isInsideCorner = true;
            _dynamics.currentCornerMaxRoll = _cornerMaxRoll;
            _dynamics.currentCornerDuration = (nowMs - _cornerStartTime) * 0.001f;

            // Exit detected when bike returns upright (<8 deg) with 250ms confirmation timer
            if (absRoll < 8.0f) {
                if (!_cornerExitPending) {
                    _cornerExitPending = true;
                    _cornerExitPendingStartTime = nowMs;
                } else if (nowMs - _cornerExitPendingStartTime >= CORNER_EXIT_HYSTERESIS_MS) {
                    // Confirmed exit after 250ms of upright riding
                    float duration = (_cornerExitPendingStartTime - _cornerStartTime) * 0.001f;
                    if (duration >= 0.4f) {
                        for (int i = 4; i > 0; i--) {
                            _dynamics.lastCorners[i] = _dynamics.lastCorners[i - 1];
                        }
                        _dynamics.lastCorners[0].isLeft = _cornerIsLeft;
                        _dynamics.lastCorners[0].maxRollDeg = _cornerMaxRoll;
                        _dynamics.lastCorners[0].maxRollRateDps = _cornerMaxRate;
                        _dynamics.lastCorners[0].durationSec = duration;
                        _dynamics.lastCorners[0].timestampMs = _cornerExitPendingStartTime;
                        if (_dynamics.cornerHistoryCount < 5) _dynamics.cornerHistoryCount++;
                    }
                    _inCorner = false;
                    _cornerExitPending = false;
                    _dynamics.isInsideCorner = false;
                }
            } else {
                _cornerExitPending = false;
            }
        }

        // 12. Lean Angle Warning & Safety Alarms
        _dynamics.maxLeanThreshold = _maxLeanThreshold;
        _dynamics.isLeanWarning = (absRoll >= _maxLeanThreshold);

        // 13. Crash Detection (Bike on ground > 65 deg, stationary roll-rate < 12 deg/s, static gravity 1G for > 3.5s)
        bool rollStationary = (fabsf(gx) < CRASH_DETECT_MAX_RATE_DPS);
        bool accelGravityStatic = (fabsf(_filteredTotalG - 1.0f) < CRASH_DETECT_ACCEL_TOL_G);

        if (absRoll >= CRASH_DETECT_ROLL_DEG && rollStationary && accelGravityStatic) {
            if (_crashStartTime == 0) _crashStartTime = nowMs;
            else if (nowMs - _crashStartTime >= CRASH_DETECT_TIME_MS) {
                _dynamics.isCrashDetected = true;
            }
        } else {
            _crashStartTime = 0;
            _dynamics.isCrashDetected = false;
        }

        outDynamics = _dynamics;
    }

private:
    Preferences _prefs;
    float _roll;
    float _pitch;
    float _lastAccelRoll = 0.0f;
    float _lastAccelPitch = 0.0f;
    float _tareRoll;
    float _tarePitch;
    uint16_t _straightCycles;
    uint32_t _lastUpdateMicros;
    bool _firstRun;
    bool _swapXY;
    bool _invertRoll;
    bool _invertAccel;

    float _maxLeanThreshold;
    float _totalElevationGainM;
    float _filteredAltM;
    float _lastAltM;
    uint32_t _lastAltTimeMs;
    float _verticalSpeedMps;

    bool _inCorner;
    bool _cornerExitPending;
    uint32_t _cornerExitPendingStartTime;
    uint32_t _cornerStartTime;
    float _cornerMaxRoll;
    float _cornerMaxRate;
    bool _cornerIsLeft;

    uint32_t _crashStartTime;
    MotorcycleDynamics _dynamics;

    // Gyro Bias Tracker state
    float _gyroBiasRollDps;       // Estimated gyro roll bias (deg/s)
    float _gyroBiasPitchDps;      // Estimated gyro pitch bias (deg/s)
    uint32_t _straightConfirmMs;  // Temporal confirmation counter
    uint32_t _lastStraightStartMs;// Timestamp when straight conditions started
    float _filteredTotalG;        // Low-pass filtered Total G magnitude (IIR 1-pole)
    float _yawAlignDeg;           // Horizontal yaw alignment trim (degrees)
    BikeState _lastState;         // Current filter state
    uint8_t _filterState;         // 0=STATIONARY, 1=STRAIGHT, 2=DYNAMIC (for debug)

    // Magnetometer Calibration State
    bool _magCalActive;
    uint32_t _magCalStartMs;
    float _magMinX, _magMaxX;
    float _magMinY, _magMaxY;
    float _magMinZ, _magMaxZ;
    float _magOffsetX, _magOffsetY, _magOffsetZ;
    float _magScaleX, _magScaleY, _magScaleZ;
};

#endif // MOTORCYCLE_FILTER_H
