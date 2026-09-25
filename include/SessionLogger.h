#ifndef SESSION_LOGGER_H
#define SESSION_LOGGER_H

#include <Arduino.h>
#include <LittleFS.h>
#include <WebServer.h>
#include "Config.h"
#include "MotorcycleFilter.h"

// ==============================================================================
// COMPACT BINARY LOG SAMPLE STRUCT (Exactly 20 Bytes)
// Prevents Flash wear: 1000-byte block writes every 5s instead of 10Hz text I/O
// ==============================================================================
#pragma pack(push, 1)
struct LogSampleBinary {
    uint32_t timestampMs;   // 4B: relative timestamp
    int16_t  roll_cdeg;      // 2B: roll * 100 (-18000..+18000)
    int16_t  pitch_cdeg;     // 2B: pitch * 100 (-9000..+9000)
    int16_t  heading_cdeg;   // 2B: heading * 100 (0..36000)
    int16_t  gLon_mG;        // 2B: G-Longitudinal * 1000 (-2500..+2500)
    int16_t  gLat_mG;        // 2B: G-Lateral * 1000 (-2500..+2500)
    int16_t  alt_dm;         // 2B: altitude in decimeters (-500..32000)
    int16_t  dPlus_dm;       // 2B: elevation gain D+ in decimeters (0..32000)
    int8_t   temp_c;         // 1B: temperature in Celsius (-40..125)
    uint8_t  flags;          // 1B: bit0:corner, bit1:brake, bit2:accel, bit3:warn, bit4:crash, bit5:heading_valid
};
#pragma pack(pop)

static const char SESSION_BIN_PATH[] = "/session.bin";

class SessionLogger {
public:
    SessionLogger() : 
        _isLogging(false), 
        _sampleCount(0), 
        _bufferCount(0), 
        _fsMounted(false) {}

    bool begin() {
        if (!LittleFS.begin(true)) {
            Serial.println("[LOGGER] ERRORE: Mount LittleFS fallito!");
            _fsMounted = false;
            return false;
        }
        _fsMounted = true;
        size_t total = LittleFS.totalBytes();
        size_t used = LittleFS.usedBytes();
        Serial.printf("[LOGGER] LittleFS montato: %u KB totali, %u KB usati (Struct=%u B)\n", 
                      (unsigned)(total / 1024), (unsigned)(used / 1024), (unsigned)sizeof(LogSampleBinary));
        return true;
    }

    bool isMounted() const { return _fsMounted; }
    bool isLogging() const { return _isLogging; }
    uint32_t getSampleCount() const { return _sampleCount; }

    size_t getFileSize() {
        if (!_fsMounted) return 0;
        if (LittleFS.exists(SESSION_BIN_PATH)) {
            File f = LittleFS.open(SESSION_BIN_PATH, "r");
            if (f) {
                size_t sz = f.size();
                f.close();
                return sz;
            }
        }
        return 0;
    }

    bool startSession() {
        if (!_fsMounted) return false;

        _bufferCount = 0;
        _file = LittleFS.open(SESSION_BIN_PATH, "a");
        if (!_file) {
            Serial.println("[LOGGER] ERRORE: Impossibile aprire /session.bin in scrittura!");
            return false;
        }

        _isLogging = true;
        _sampleCount = 0;
        Serial.println("[LOGGER] *** REGISTRAZIONE SESSIONE BINARIA AVVIATA (Buffer RAM 1000 B) ***");
        return true;
    }

    void flushBufferToFlash() {
        if (!_file || _bufferCount == 0) return;
        size_t bytesToWrite = _bufferCount * sizeof(LogSampleBinary);
        size_t written = _file.write((const uint8_t*)_ramBuffer, bytesToWrite);
        _file.flush();
        _bufferCount = 0;
        if (written != bytesToWrite) {
            Serial.printf("[LOGGER] WARN: Scrittura flash parziale (%u/%u bytes)\n", (unsigned)written, (unsigned)bytesToWrite);
        }
    }

    void stopSession() {
        if (_isLogging) {
            flushBufferToFlash();
            if (_file) {
                _file.close();
            }
            _isLogging = false;
            Serial.printf("[LOGGER] *** SESSIONE TERMINATA: %u campioni salvati (%u bytes su Flash) ***\n", 
                          _sampleCount, (unsigned)getFileSize());
        }
    }

    void clearSession() {
        stopSession();
        if (_fsMounted && LittleFS.exists(SESSION_BIN_PATH)) {
            LittleFS.remove(SESSION_BIN_PATH);
            Serial.println("[LOGGER] File /session.bin cancellato.");
        }
        _sampleCount = 0;
        _bufferCount = 0;
    }

    void logSample(const MotorcycleDynamics& dyn, uint32_t nowMs) {
        if (!_isLogging) return;

        // 1. Pack sample directly into RAM buffer (duration < 1 microsecond, ZERO flash I/O)
        LogSampleBinary& s = _ramBuffer[_bufferCount];
        s.timestampMs   = nowMs;
        s.roll_cdeg     = (int16_t)constrain(dyn.rollDeg * 100.0f, -18000.0f, 18000.0f);
        s.pitch_cdeg    = (int16_t)constrain(dyn.pitchDeg * 100.0f, -9000.0f, 9000.0f);
        s.heading_cdeg  = (int16_t)constrain(dyn.headingDeg * 100.0f, 0.0f, 36000.0f);
        s.gLon_mG       = (int16_t)constrain(dyn.gLongitudinal * 1000.0f, -2500.0f, 2500.0f);
        s.gLat_mG       = (int16_t)constrain(dyn.gLateral * 1000.0f, -2500.0f, 2500.0f);
        s.alt_dm        = (int16_t)constrain(dyn.altitudeM * 10.0f, -500.0f, 32000.0f);
        s.dPlus_dm      = (int16_t)constrain(dyn.totalElevationGainM * 10.0f, 0.0f, 32000.0f);
        s.temp_c        = (int8_t)constrain((int)dyn.tempC, -40, 125);

        uint8_t flags = 0;
        if (dyn.isCornering)     flags |= 0x01;
        if (dyn.isBraking)       flags |= 0x02;
        if (dyn.isAccelerating)  flags |= 0x04;
        if (dyn.isLeanWarning)   flags |= 0x08;
        if (dyn.isCrashDetected) flags |= 0x10;
        if (dyn.isHeadingValid)  flags |= 0x20;
        s.flags = flags;

        _bufferCount++;
        _sampleCount++;

        // 2. Flush block of 50 samples (1000 bytes) to Flash LittleFS only when buffer is full
        if (_bufferCount >= LOG_RAM_BUFFER_SAMPLES) {
            flushBufferToFlash();
        }
    }

    // Streams binary session file dynamically converted into standard CSV text on-the-fly
    bool streamFileTo(WebServer& server) {
        if (!_fsMounted || !LittleFS.exists(SESSION_BIN_PATH)) {
            server.send(404, "text/plain", "Nessun file di sessione trovato.");
            return false;
        }

        // Flush any lingering samples in RAM buffer before reading
        if (_isLogging) {
            flushBufferToFlash();
        }

        File binFile = LittleFS.open(SESSION_BIN_PATH, "r");
        if (!binFile || binFile.size() == 0) {
            if (binFile) binFile.close();
            server.send(404, "text/plain", "Sessione vuota o non valida.");
            return false;
        }

        WiFiClient client = server.client();
        if (!client) {
            binFile.close();
            return false;
        }

        // Send HTTP headers for chunked streaming with standard CRLF CSV format
        server.sendHeader("Content-Type", "text/csv; charset=utf-8");
        server.sendHeader("Content-Disposition", "attachment; filename=\"fz8_session.csv\"");
        server.sendHeader("Connection", "close");
        server.setContentLength(CONTENT_LENGTH_UNKNOWN);
        server.send(200, "text/csv", "");

        // CSV Header row with RFC 4180 standard CRLF (\r\n)
        static const char CSV_HEADER[] = "timestamp_ms,roll_deg,pitch_deg,heading_deg,heading_valid,g_lon,g_lat,alt_m,d_plus_m,temp_c,is_cornering,is_braking,is_accel,is_warn,is_crash\r\n";
        
        // 1024-byte TX aggregation buffer: eliminates TCP packet fragmentation & browser corrupt download errors
        char txBuffer[1024];
        size_t txLen = 0;

        auto flushTx = [&]() {
            if (txLen > 0) {
                server.sendContent(txBuffer, txLen);
                txLen = 0;
            }
        };

        auto appendTx = [&](const char* str, size_t len) {
            if (txLen + len >= sizeof(txBuffer)) {
                flushTx();
            }
            memcpy(txBuffer + txLen, str, len);
            txLen += len;
        };

        appendTx(CSV_HEADER, strlen(CSV_HEADER));

        // Read in chunks of 25 binary structs (500 bytes)
        const size_t CHUNK_SAMPLES = 25;
        LogSampleBinary readChunk[CHUNK_SAMPLES];
        char lineBuf[160];

        while (binFile.available()) {
            size_t bytesRead = binFile.read((uint8_t*)readChunk, sizeof(readChunk));
            size_t samplesRead = bytesRead / sizeof(LogSampleBinary);

            for (size_t i = 0; i < samplesRead; i++) {
                const LogSampleBinary& s = readChunk[i];
                int lineLen = snprintf(lineBuf, sizeof(lineBuf),
                    "%lu,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,%d,%d\r\n",
                    (unsigned long)s.timestampMs,
                    s.roll_cdeg * 0.01f,
                    s.pitch_cdeg * 0.01f,
                    s.heading_cdeg * 0.01f,
                    (s.flags & 0x20) ? 1 : 0,
                    s.gLon_mG * 0.001f,
                    s.gLat_mG * 0.001f,
                    s.alt_dm * 0.1f,
                    s.dPlus_dm * 0.1f,
                    (int)s.temp_c,
                    (s.flags & 0x01) ? 1 : 0,
                    (s.flags & 0x02) ? 1 : 0,
                    (s.flags & 0x04) ? 1 : 0,
                    (s.flags & 0x08) ? 1 : 0,
                    (s.flags & 0x10) ? 1 : 0
                );
                if (lineLen > 0) {
                    appendTx(lineBuf, (size_t)lineLen);
                }
            }
        }

        flushTx();
        server.sendContent(""); // Terminate chunked HTTP transfer
        binFile.close();
        return true;
    }

    // Direct Serial Export for debugging and offline CSV download
    bool dumpCsvToSerial(Stream& out) {
        if (!_fsMounted || !LittleFS.exists(SESSION_BIN_PATH)) {
            out.println("[LOGGER] Nessuna sessione memorizzata su Flash LittleFS.");
            return false;
        }
        if (_isLogging) {
            flushBufferToFlash();
        }
        File binFile = LittleFS.open(SESSION_BIN_PATH, "r");
        if (!binFile || binFile.size() == 0) {
            if (binFile) binFile.close();
            out.println("[LOGGER] File di sessione vuoto.");
            return false;
        }

        out.printf("[LOGGER] Inizio Dump CSV (%u bytes totali)...\n", (unsigned)binFile.size());
        out.println("timestamp_ms,roll_deg,pitch_deg,heading_deg,heading_valid,g_lon,g_lat,alt_m,d_plus_m,temp_c,is_cornering,is_braking,is_accel,is_warn,is_crash");

        LogSampleBinary s;
        char lineBuf[160];
        uint32_t count = 0;
        while (binFile.read((uint8_t*)&s, sizeof(LogSampleBinary)) == sizeof(LogSampleBinary)) {
            snprintf(lineBuf, sizeof(lineBuf),
                "%lu,%.1f,%.1f,%.1f,%d,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d,%d,%d",
                (unsigned long)s.timestampMs,
                s.roll_cdeg * 0.01f,
                s.pitch_cdeg * 0.01f,
                s.heading_cdeg * 0.01f,
                (s.flags & 0x20) ? 1 : 0,
                s.gLon_mG * 0.001f,
                s.gLat_mG * 0.001f,
                s.alt_dm * 0.1f,
                s.dPlus_dm * 0.1f,
                (int)s.temp_c,
                (s.flags & 0x01) ? 1 : 0,
                (s.flags & 0x02) ? 1 : 0,
                (s.flags & 0x04) ? 1 : 0,
                (s.flags & 0x08) ? 1 : 0,
                (s.flags & 0x10) ? 1 : 0
            );
            out.println(lineBuf);
            count++;
        }
        binFile.close();
        out.printf("[LOGGER] Dump completato: %u righe CSV esportate con successo.\n", count);
        return true;
    }

private:
    bool _isLogging;
    uint32_t _sampleCount;
    uint16_t _bufferCount;
    bool _fsMounted;
    File _file;
    LogSampleBinary _ramBuffer[LOG_RAM_BUFFER_SAMPLES]; // 1000 byte buffer in RAM
};

#endif // SESSION_LOGGER_H
