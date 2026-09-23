#ifndef DYNAMIC_AUTO_CALIBRATOR_H
#define DYNAMIC_AUTO_CALIBRATOR_H

#include <Arduino.h>
#include <Preferences.h>
#include "Config.h"
#include "SensorsGY89.h"
#include "MotorcycleFilter.h"

// ==============================================================================
// 3-MINUTE DYNAMIC RIDING AUTO-CALIBRATION & SELF-LEARNING ENGINE
// Analyzes dynamic straight-line riding, steady cruising, and throttle ranges
// Automatically calculates and saves optimal Tare Roll, Pitch & TPS Min/Max to Flash
// ==============================================================================

enum AutoCalState {
    AUTOCAL_STATE_IDLE = 0,
    AUTOCAL_STATE_RUNNING = 1,
    AUTOCAL_STATE_COMPLETED = 2
};

class DynamicAutoCalibrator {
public:
    DynamicAutoCalibrator() :
        _state(AUTOCAL_STATE_IDLE),
        _startTimeMs(0),
        _durationMs(180000), // 3 minutes = 180 seconds
        _straightSampleCount(0),
        _cruisePitchSampleCount(0),
        _sumRoll(0.0),
        _sumPitch(0.0),
        _minTpsAdc(4095),
        _maxTpsAdc(0),
        _calculatedTareRoll(0.0f),
        _calculatedTarePitch(0.0f),
        _calculatedTpsMin(TPS_MIN_ADC_DEFAULT),
        _calculatedTpsMax(TPS_MAX_ADC_DEFAULT),
        _justFinished(false) {}

    void start() {
        _state = AUTOCAL_STATE_RUNNING;
        _startTimeMs = millis();
        _straightSampleCount = 0;
        _cruisePitchSampleCount = 0;
        _sumRoll = 0.0;
        _sumPitch = 0.0;
        _minTpsAdc = 4095;
        _maxTpsAdc = 0;
        _justFinished = false;
        Serial.println("\n========================================================");
        Serial.println("[AUTOCAL] *** AVVIO AUTO-CALIBRAZIONE IN MARCIA (3 MINUTI) ***");
        Serial.println("[AUTOCAL] Guida normalmente: curva, accelera e mantieni rettilinei!");
        Serial.println("========================================================\n");
    }

    void cancel() {
        _state = AUTOCAL_STATE_IDLE;
        _justFinished = false;
        Serial.println("[AUTOCAL] Auto-Calibrazione Annullata dall'utente.");
    }

    bool isRunning() const { return _state == AUTOCAL_STATE_RUNNING; }
    bool isCompleted() const { return _state == AUTOCAL_STATE_COMPLETED; }
    bool hasJustFinished() { 
        bool f = _justFinished; 
        _justFinished = false; 
        return f; 
    }

    uint8_t getProgressPercent() const {
        if (_state != AUTOCAL_STATE_RUNNING) return (_state == AUTOCAL_STATE_COMPLETED) ? 100 : 0;
        uint32_t elapsed = millis() - _startTimeMs;
        if (elapsed >= _durationMs) return 100;
        return (uint8_t)((elapsed * 100) / _durationMs);
    }

    uint16_t getRemainingSeconds() const {
        if (_state != AUTOCAL_STATE_RUNNING) return 0;
        uint32_t elapsed = millis() - _startTimeMs;
        if (elapsed >= _durationMs) return 0;
        return (uint16_t)((_durationMs - elapsed) / 1000);
    }

    uint32_t getStraightSamples() const { return _straightSampleCount; }
    float getEstimatedRollOffset() const { return (_straightSampleCount > 0) ? (float)(_sumRoll / _straightSampleCount) : 0.0f; }
    float getEstimatedPitchOffset() const { return (_cruisePitchSampleCount > 0) ? (float)(_sumPitch / _cruisePitchSampleCount) : 0.0f; }
    uint16_t getObservedTpsMin() const { return (_minTpsAdc < 4000) ? _minTpsAdc : TPS_MIN_ADC_DEFAULT; }
    uint16_t getObservedTpsMax() const { return (_maxTpsAdc > 500) ? _maxTpsAdc : TPS_MAX_ADC_DEFAULT; }

    void update(const MotorcycleDynamics& dyn, uint16_t rawTpsAdc, MotorcycleFilter& filter, Preferences& prefs) {
        if (_state != AUTOCAL_STATE_RUNNING) return;

        uint32_t now = millis();
        uint32_t elapsed = now - _startTimeMs;

        // 1. Monitor TPS Min / Max over riding session
        if (rawTpsAdc >= 150 && rawTpsAdc < 3900) {
            if (rawTpsAdc < _minTpsAdc) _minTpsAdc = rawTpsAdc;
            if (rawTpsAdc > _maxTpsAdc) _maxTpsAdc = rawTpsAdc;
        }

        // 2. Dynamic Straight-line Detection Gate for Roll Auto-Zero
        // Criteria: Bike moving straight, zero turn rate, 1G downward gravity
        bool isStraight = (fabsf(dyn.yawRateDps) < 1.8f) &&
                          (fabsf(dyn.rollRateDps) < 2.0f) &&
                          (fabsf(dyn.gLateral) < 0.06f) &&
                          (dyn.totalG >= 0.94f && dyn.totalG <= 1.06f);

        if (isStraight) {
            // Raw un-tared roll angle from gyro/accelerometer integration
            float rawRoll = dyn.rollDeg + filter.getTareRoll();
            _sumRoll += rawRoll;
            _straightSampleCount++;
        }

        // 3. Steady Cruise Pitch Gate (Constant Speed, no heavy braking or wheelies)
        bool isSteadyCruise = isStraight && (fabsf(dyn.gLongitudinal) < 0.08f);
        if (isSteadyCruise) {
            float rawPitch = dyn.pitchDeg + filter.getTarePitch();
            _sumPitch += rawPitch;
            _cruisePitchSampleCount++;
        }

        // 4. Check if 3 Minutes Completed
        if (elapsed >= _durationMs) {
            finalizeCalibration(filter, prefs);
        }
    }

private:
    void finalizeCalibration(MotorcycleFilter& filter, Preferences& prefs) {
        _state = AUTOCAL_STATE_COMPLETED;
        _justFinished = true;

        if (_straightSampleCount > 100) {
            _calculatedTareRoll = (float)(_sumRoll / _straightSampleCount);
        } else {
            _calculatedTareRoll = filter.getTareRoll(); // fallback if too few samples
        }

        if (_cruisePitchSampleCount > 80) {
            _calculatedTarePitch = (float)(_sumPitch / _cruisePitchSampleCount);
        } else {
            _calculatedTarePitch = filter.getTarePitch();
        }

        if (_minTpsAdc < 2000 && _maxTpsAdc > 1500 && (_maxTpsAdc - _minTpsAdc > 500)) {
            _calculatedTpsMin = _minTpsAdc;
            _calculatedTpsMax = _maxTpsAdc;
        } else {
            _calculatedTpsMin = TPS_MIN_ADC_DEFAULT;
            _calculatedTpsMax = TPS_MAX_ADC_DEFAULT;
        }

        // Save Tare Roll & Pitch to Flash
        prefs.begin("fz8_cal", false);
        prefs.putFloat("tareRoll", _calculatedTareRoll);
        prefs.putFloat("tarePitch", _calculatedTarePitch);
        prefs.end();

        // Save TPS calibration to Flash
        prefs.begin("fz8_tps", false);
        prefs.putUShort("min", _calculatedTpsMin);
        prefs.putUShort("max", _calculatedTpsMax);
        prefs.end();

        // Reload into active filter
        filter.begin();

        Serial.println("\n========================================================");
        Serial.println(" [AUTOCAL] *** AUTO-CALIBRAZIONE 3 MINUTI COMPLETATA CON SUCCESSO! ***");
        Serial.printf("  - Campioni Rettilineo Analizzati : %lu\n", (unsigned long)_straightSampleCount);
        Serial.printf("  - Offset Rollio (Zero Piega)     : %+.2f deg -> SALVATO IN FLASH\n", _calculatedTareRoll);
        Serial.printf("  - Offset Beccheggio (Assetto)    : %+.2f deg -> SALVATO IN FLASH\n", _calculatedTarePitch);
        Serial.printf("  - TPS Minimo (Gas Chiuso)        : %u ADC -> SALVATO IN FLASH\n", _calculatedTpsMin);
        Serial.printf("  - TPS Massimo (WOT 100%%)         : %u ADC -> SALVATO IN FLASH\n", _calculatedTpsMax);
        Serial.println("========================================================\n");
    }

    AutoCalState _state;
    uint32_t _startTimeMs;
    uint32_t _durationMs;
    uint32_t _straightSampleCount;
    uint32_t _cruisePitchSampleCount;
    double _sumRoll;
    double _sumPitch;
    uint16_t _minTpsAdc;
    uint16_t _maxTpsAdc;
    
    float _calculatedTareRoll;
    float _calculatedTarePitch;
    uint16_t _calculatedTpsMin;
    uint16_t _calculatedTpsMax;
    bool _justFinished;
};

#endif // DYNAMIC_AUTO_CALIBRATOR_H
