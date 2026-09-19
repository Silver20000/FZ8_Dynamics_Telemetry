#ifndef SENSORS_GY89_H
#define SENSORS_GY89_H

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"

// ==============================================================================
// GY-89 10-DOF SENSOR INTERFACE (LSM303D + L3GD20 + BMP180)
// ==============================================================================

// LSM303D Registers & Constants
#define LSM303D_WHO_AM_I_REG    0x0F
#define LSM303D_EXPECTED_ID     0x49
#define LSM303D_CTRL1           0x20
#define LSM303D_CTRL2           0x21
#define LSM303D_OUT_X_L_A       0x28

// L3GD20 Registers & Constants
#define L3GD20_WHO_AM_I_REG     0x0F
#define L3GD20_EXPECTED_ID1     0xD4
#define L3GD20_EXPECTED_ID2     0xD7  // L3GD20H
#define L3GD20_CTRL1            0x20
#define L3GD20_CTRL4            0x23
#define L3GD20_OUT_X_L          0x28

// BMP180 Registers
#define BMP180_I2C_ADDR         0x77
#define BMP180_REG_CONTROL      0xF4
#define BMP180_REG_RESULT       0xF6
#define BMP180_CMD_TEMP         0x2E
#define BMP180_CMD_PRESSURE_3   0xF4  // oss = 3 (ultra high resolution)

struct IMURawData {
    float ax, ay, az;       // Linear acceleration in G (longitudinal, lateral, vertical)
    float gx, gy, gz;       // Angular velocity in deg/s (roll rate, pitch rate, yaw rate)
    float temp_c;           // Temperature in Celsius
    float pressure_hpa;     // Barometric pressure in hPa
    float altitude_m;       // Calculated altitude in meters
    bool data_ready;
};

class SensorsGY89 {
public:
    SensorsGY89() : 
        _lsmAddr(0x1D), _l3gAddr(0x6B), 
        _gyroBiasX(0.0f), _gyroBiasY(0.0f), _gyroBiasZ(0.0f),
        _accelBiasX(0.0f), _accelBiasY(0.0f), _accelBiasZ(0.0f),
        _baroState(0), _baroTimer(0), _hasBMP(false) {}

    bool begin() {
        Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
        Wire.setClock(I2C_FREQ_HZ);

        delay(50);

        // 1. Detect & Initialize LSM303D (Accelerometer)
        if (!initLSM303D()) {
            Serial.println("[IMU] LSM303D not found at 0x1D, trying 0x1E...");
            _lsmAddr = 0x1E;
            if (!initLSM303D()) {
                Serial.println("[IMU] ERROR: LSM303D failed to initialize!");
                return false;
            }
        }
        Serial.printf("[IMU] LSM303D initialized at 0x%02X (+/-4G range)\n", _lsmAddr);

        // 2. Detect & Initialize L3GD20 (Gyroscope)
        if (!initL3GD20()) {
            Serial.println("[IMU] L3GD20 not found at 0x6B, trying 0x6A...");
            _l3gAddr = 0x6A;
            if (!initL3GD20()) {
                Serial.println("[IMU] ERROR: L3GD20 failed to initialize!");
                return false;
            }
        }
        Serial.printf("[IMU] L3GD20 initialized at 0x%02X (+/-500 dps)\n", _l3gAddr);

        // 3. Initialize BMP180
        _hasBMP = initBMP180();
        if (_hasBMP) {
            Serial.println("[IMU] BMP180 barometric altimeter initialized");
        } else {
            Serial.println("[IMU] BMP180 not detected (altitude disabled)");
        }

        // 4. Initial Gyro Calibration (Zero-rate bias)
        calibrateGyro(150);

        return true;
    }

    void readIMU(IMURawData& outData) {
        // Read Accel (6 bytes, auto-increment: bit 7 = 1 -> 0x80)
        uint8_t aBuf[6];
        if (readI2C(_lsmAddr, LSM303D_OUT_X_L_A | 0x80, aBuf, 6)) {
            int16_t rawAx = (int16_t)((aBuf[1] << 8) | aBuf[0]);
            int16_t rawAy = (int16_t)((aBuf[3] << 8) | aBuf[2]);
            int16_t rawAz = (int16_t)((aBuf[5] << 8) | aBuf[4]);

            // +/- 4G range: sensitivity = 0.122 mg/LSB = 0.000122 g/LSB
            const float ACCEL_SCALE = 0.000122f;
            outData.ax = (rawAx * ACCEL_SCALE) - _accelBiasX;
            outData.ay = (rawAy * ACCEL_SCALE) - _accelBiasY;
            outData.az = (rawAz * ACCEL_SCALE);
        }

        // Read Gyro (6 bytes, auto-increment: bit 7 = 1 -> 0x80)
        uint8_t gBuf[6];
        if (readI2C(_l3gAddr, L3GD20_OUT_X_L | 0x80, gBuf, 6)) {
            int16_t rawGx = (int16_t)((gBuf[1] << 8) | gBuf[0]);
            int16_t rawGy = (int16_t)((gBuf[3] << 8) | gBuf[2]);
            int16_t rawGz = (int16_t)((gBuf[5] << 8) | gBuf[4]);

            // +/- 500 dps range: sensitivity = 17.5 mdps/LSB = 0.0175 deg/s/LSB
            const float GYRO_SCALE = 0.0175f;
            outData.gx = (rawGx * GYRO_SCALE) - _gyroBiasX;
            outData.gy = (rawGy * GYRO_SCALE) - _gyroBiasY;
            outData.gz = (rawGz * GYRO_SCALE) - _gyroBiasZ;
        }

        outData.temp_c = _cachedTemp;
        outData.pressure_hpa = _cachedPressure;
        outData.altitude_m = _cachedAltitude;
        outData.data_ready = true;
    }

    // Non-blocking state machine for BMP180 baro/altimeter
    void updateBarometer() {
        if (!_hasBMP) return;

        uint32_t now = millis();
        switch (_baroState) {
            case 0: // Start Temperature measurement
                writeI2CByte(BMP180_I2C_ADDR, BMP180_REG_CONTROL, BMP180_CMD_TEMP);
                _baroTimer = now;
                _baroState = 1;
                break;

            case 1: // Wait 5ms for Temperature conversion
                if (now - _baroTimer >= 5) {
                    uint8_t tBuf[2];
                    if (readI2C(BMP180_I2C_ADDR, BMP180_REG_RESULT, tBuf, 2)) {
                        int32_t ut = (tBuf[0] << 8) | tBuf[1];
                        computeTrueTemperature(ut);
                    }
                    // Start Pressure measurement (oss = 3, requires 26ms)
                    writeI2CByte(BMP180_I2C_ADDR, BMP180_REG_CONTROL, BMP180_CMD_PRESSURE_3);
                    _baroTimer = now;
                    _baroState = 2;
                }
                break;

            case 2: // Wait 26ms for Pressure conversion
                if (now - _baroTimer >= 26) {
                    uint8_t pBuf[3];
                    if (readI2C(BMP180_I2C_ADDR, BMP180_REG_RESULT, pBuf, 3)) {
                        int32_t up = (((int32_t)pBuf[0] << 16) | ((int32_t)pBuf[1] << 8) | (int32_t)pBuf[2]) >> (8 - 3);
                        computeTruePressure(up);
                    }
                    _baroTimer = now;
                    _baroState = 3;
                }
                break;

            case 3: // Pause until next poll period
                if (now - _baroTimer >= BARO_POLL_INTERVAL_MS) {
                    _baroState = 0;
                }
                break;
        }
    }

    void calibrateGyro(int samples = 150) {
        Serial.println("[IMU] Calibrating gyro bias (keep motorcycle still)...");
        float sumX = 0, sumY = 0, sumZ = 0;
        int valid = 0;

        for (int i = 0; i < samples; i++) {
            uint8_t gBuf[6];
            if (readI2C(_l3gAddr, L3GD20_OUT_X_L | 0x80, gBuf, 6)) {
                int16_t rx = (int16_t)((gBuf[1] << 8) | gBuf[0]);
                int16_t ry = (int16_t)((gBuf[3] << 8) | gBuf[2]);
                int16_t rz = (int16_t)((gBuf[5] << 8) | gBuf[4]);
                sumX += rx * 0.0175f;
                sumY += ry * 0.0175f;
                sumZ += rz * 0.0175f;
                valid++;
            }
            delay(8);
        }

        if (valid > 0) {
            _gyroBiasX = sumX / valid;
            _gyroBiasY = sumY / valid;
            _gyroBiasZ = sumZ / valid;
            Serial.printf("[IMU] Gyro bias calculated: X=%.3f, Y=%.3f, Z=%.3f deg/s\n", 
                          _gyroBiasX, _gyroBiasY, _gyroBiasZ);
        }
    }

    void setTareOffsets(float rollBiasDeg) {
        // Tare zero position
    }

private:
    uint8_t _lsmAddr;
    uint8_t _l3gAddr;
    float _gyroBiasX, _gyroBiasY, _gyroBiasZ;
    float _accelBiasX, _accelBiasY, _accelBiasZ;

    // BMP180 calibration coefficients
    int16_t ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t b1, b2;
    int16_t mb, mc, md;
    int32_t b5;

    uint8_t _baroState;
    uint32_t _baroTimer;
    bool _hasBMP;
    float _cachedTemp = 25.0f;
    float _cachedPressure = 1013.25f;
    float _cachedAltitude = 0.0f;

    bool initLSM303D() {
        uint8_t id = readI2CByte(_lsmAddr, LSM303D_WHO_AM_I_REG);
        if (id != LSM303D_EXPECTED_ID) return false;

        // CTRL1: 100Hz ODR, BDU=1, All axes enabled (0x67)
        writeI2CByte(_lsmAddr, LSM303D_CTRL1, 0x67);
        // CTRL2: Anti-alias 773Hz, +/-4G range (0x08)
        writeI2CByte(_lsmAddr, LSM303D_CTRL2, 0x08);
        return true;
    }

    bool initL3GD20() {
        uint8_t id = readI2CByte(_l3gAddr, L3GD20_WHO_AM_I_REG);
        if (id != L3GD20_EXPECTED_ID1 && id != L3GD20_EXPECTED_ID2) return false;

        // CTRL1: 190Hz ODR, 50Hz Cutoff, Normal mode, XYZ enabled (0x6F)
        writeI2CByte(_l3gAddr, L3GD20_CTRL1, 0x6F);
        // CTRL4: BDU enabled, +/-500 dps full scale (0x90)
        writeI2CByte(_l3gAddr, L3GD20_CTRL4, 0x90);
        return true;
    }

    bool initBMP180() {
        Wire.beginTransmission(BMP180_I2C_ADDR);
        if (Wire.endTransmission() != 0) return false;

        // Read 22 bytes of calibration data (0xAA to 0xBF)
        uint8_t cal[22];
        if (!readI2C(BMP180_I2C_ADDR, 0xAA, cal, 22)) return false;

        ac1 = (int16_t)((cal[0] << 8) | cal[1]);
        ac2 = (int16_t)((cal[2] << 8) | cal[3]);
        ac3 = (int16_t)((cal[4] << 8) | cal[5]);
        ac4 = (uint16_t)((cal[6] << 8) | cal[7]);
        ac5 = (uint16_t)((cal[8] << 8) | cal[9]);
        ac6 = (uint16_t)((cal[10] << 8) | cal[11]);
        b1  = (int16_t)((cal[12] << 8) | cal[13]);
        b2  = (int16_t)((cal[14] << 8) | cal[15]);
        mb  = (int16_t)((cal[16] << 8) | cal[17]);
        mc  = (int16_t)((cal[18] << 8) | cal[19]);
        md  = (int16_t)((cal[20] << 8) | cal[21]);

        return true;
    }

    void computeTrueTemperature(int32_t ut) {
        int32_t x1 = ((ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
        int32_t x2 = ((int32_t)mc << 11) / (x1 + md);
        b5 = x1 + x2;
        _cachedTemp = ((b5 + 8) >> 4) / 10.0f;
    }

    void computeTruePressure(int32_t up) {
        int32_t b6 = b5 - 4000;
        int32_t x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
        int32_t x2 = (ac2 * b6) >> 11;
        int32_t x3 = x1 + x2;
        int32_t b3 = ((((int32_t)ac1 * 4 + x3) << 3) + 2) >> 2;

        x1 = (ac3 * b6) >> 13;
        x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
        x3 = ((x1 + x2) + 2) >> 2;
        uint32_t b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
        uint32_t b7 = ((uint32_t)up - b3) * (50000 >> 3);

        int32_t p = 0;
        if (b7 < 0x80000000) {
            p = (b7 * 2) / b4;
        } else {
            p = (b7 / b4) * 2;
        }

        x1 = (p >> 8) * (p >> 8);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * p) >> 16;
        p = p + ((x1 + x2 + 3791) >> 4);

        _cachedPressure = p / 100.0f; // hPa
        // Hypsometric altitude formula
        _cachedAltitude = 44330.0f * (1.0f - powf(_cachedPressure / 1013.25f, 0.1903f));
    }

    uint8_t readI2CByte(uint8_t devAddr, uint8_t reg) {
        Wire.beginTransmission(devAddr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return 0xFF;
        Wire.requestFrom(devAddr, (uint8_t)1);
        return Wire.available() ? Wire.read() : 0xFF;
    }

    bool writeI2CByte(uint8_t devAddr, uint8_t reg, uint8_t value) {
        Wire.beginTransmission(devAddr);
        Wire.write(reg);
        Wire.write(value);
        return (Wire.endTransmission() == 0);
    }

    bool readI2C(uint8_t devAddr, uint8_t reg, uint8_t* buffer, uint8_t count) {
        Wire.beginTransmission(devAddr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return false;
        uint8_t readBytes = Wire.requestFrom(devAddr, count);
        if (readBytes != count) return false;
        for (uint8_t i = 0; i < count; i++) {
            buffer[i] = Wire.read();
        }
        return true;
    }
};

#endif // SENSORS_GY89_H
