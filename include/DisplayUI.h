#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <SSD1283A.h>
#include "Config.h"
#include "MotorcycleFilter.h"

// 16-bit 565 Colors
#define COLOR_BLACK         0x0000
#define COLOR_DARK_GRAY     0x2104
#define COLOR_MID_GRAY      0x4208
#define COLOR_WHITE         0xFFFF
#define COLOR_CYAN          0x07FF
#define COLOR_GREEN         0x07E0
#define COLOR_AMBER         0xFD20
#define COLOR_ORANGE        0xFD20
#define COLOR_RED           0xF800
#define COLOR_YELLOW        0xFFE0
#define COLOR_BLUE          0x001F
#define COLOR_MAGENTA       0xF81F

enum UIScreenMode {
    SCREEN_DYNAMICS = 0,      // 0: Classic Analog Arc Gauge + Peaks L/R
    SCREEN_RACE_MOTOGP,       // 1: MotoGP Clean Display: Giant digital lean, Left/Right LED segment bars
    SCREEN_GG_DIAGRAM,        // 2: G-G Traction / Friction Circle
    SCREEN_CORNER_ANALYZE,    // 3: Corner Analyzer Live: Active corner, apex angle, entry roll rate, last 3 corners
    SCREEN_ALTIMETRY_TOUR,    // 4: Mountain Touring: Big compass rose, Altitude, D+ gain, pitch angle & temp
    SCREEN_PITCH_SUSPENSION,  // 5: Suspension & Wheelie/Stoppie Inclinometer (Assetto Dinamico)
    SCREEN_HORIZON_HUD,       // 6: Artificial Horizon / Cockpit Gyro Horizon HUD (Aviation style)
    SCREEN_MINIMAL_RACE,      // 7: Minimal Ultra-High Contrast Race HUD (Giant lean + warning border)
    SCREEN_SESSION_STATS,     // 8: Summary: Peaks, max braking, max accel, run duration
    SCREEN_COUNT
};

class DisplayUI {
public:
    DisplayUI() : 
        _lcd(LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN, LCD_LED_PIN),
        _currentScreen(SCREEN_DYNAMICS),
        _needsFullRedraw(true),
        _prevRoll(0.0f),
        _prevNeedleX(65), _prevNeedleY(30),
        _prevDotX(65), _prevDotY(65),
        _prevPitchBarY(58),
        _prevHorizonX1(20), _prevHorizonY1(63),
        _prevHorizonX2(110), _prevHorizonY2(63),
        _wifiActive(false),
        _radioMode(RADIO_MODE_DASHBOARD),
        _autoCycle(false),
        _autoCycleIntervalMs(6000),
        _lastCycleMillis(0) {}

    bool begin() {
        SPI.begin(LCD_SCK_PIN, -1, LCD_MOSI_PIN, LCD_CS_PIN);
        
        _lcd.init();
        _lcd.setRotation(0);
        _lcd.fillScreen(COLOR_BLACK);

        if (LCD_LED_PIN >= 0) {
            pinMode(LCD_LED_PIN, OUTPUT);
            digitalWrite(LCD_LED_PIN, HIGH);
        }

        drawBootLogo();
        delay(1100);

        _lcd.fillScreen(COLOR_BLACK);
        _needsFullRedraw = true;
        return true;
    }

    void nextScreen() {
        _currentScreen = (UIScreenMode)((_currentScreen + 1) % SCREEN_COUNT);
        _needsFullRedraw = true;
    }

    void setScreen(UIScreenMode mode) {
        _currentScreen = (UIScreenMode)(mode % SCREEN_COUNT);
        _needsFullRedraw = true;
    }

    UIScreenMode getScreen() const { return _currentScreen; }

    const char* getScreenName(UIScreenMode mode) const {
        switch (mode) {
            case SCREEN_DYNAMICS:         return "Arc Gauge";
            case SCREEN_RACE_MOTOGP:      return "MotoGP Race";
            case SCREEN_GG_DIAGRAM:       return "G-G Circle";
            case SCREEN_CORNER_ANALYZE:   return "Corner Apex";
            case SCREEN_ALTIMETRY_TOUR:   return "Tour / Passi";
            case SCREEN_PITCH_SUSPENSION: return "Pitch & Wheelie";
            case SCREEN_HORIZON_HUD:      return "Horizon HUD";
            case SCREEN_MINIMAL_RACE:     return "Minimal Race";
            case SCREEN_SESSION_STATS:    return "Session Stats";
            default:                      return "Screen";
        }
    }

    void setAutoCycle(bool enable, uint32_t intervalMs = 6000) {
        _autoCycle = enable;
        _autoCycleIntervalMs = intervalMs;
        _lastCycleMillis = millis();
    }

    bool isAutoCycle() const { return _autoCycle; }
    uint32_t getAutoCycleInterval() const { return _autoCycleIntervalMs; }

    void setWifiStatus(bool active) {
        if (_wifiActive != active) {
            _wifiActive = active;
            _needsFullRedraw = true;
        }
    }

    void setRadioMode(RadioMode mode) {
        if (_radioMode != mode) {
            _radioMode = mode;
            _needsFullRedraw = true;
        }
    }

    RadioMode getRadioMode() const { return _radioMode; }

    void update(const MotorcycleDynamics& dyn) {
        if (_autoCycle && (millis() - _lastCycleMillis >= _autoCycleIntervalMs)) {
            _lastCycleMillis = millis();
            nextScreen();
        }

        if (_needsFullRedraw) {
            _lcd.fillScreen(COLOR_BLACK);
            switch (_currentScreen) {
                case SCREEN_DYNAMICS:         drawDynamicsStaticLayout(); break;
                case SCREEN_RACE_MOTOGP:      drawRaceMotoGPStaticLayout(); break;
                case SCREEN_GG_DIAGRAM:       drawGGDiagramStaticLayout(); break;
                case SCREEN_CORNER_ANALYZE:   drawCornerAnalyzeStaticLayout(); break;
                case SCREEN_ALTIMETRY_TOUR:   drawAltimetryTourStaticLayout(); break;
                case SCREEN_PITCH_SUSPENSION: drawPitchSuspensionStaticLayout(); break;
                case SCREEN_HORIZON_HUD:      drawHorizonHUDStaticLayout(); break;
                case SCREEN_MINIMAL_RACE:     drawMinimalRaceStaticLayout(); break;
                case SCREEN_SESSION_STATS:    drawStatsStaticLayout(); break;
            }
            _needsFullRedraw = false;
        }

        switch (_currentScreen) {
            case SCREEN_DYNAMICS:         renderDynamicsDynamic(dyn); break;
            case SCREEN_RACE_MOTOGP:      renderRaceMotoGPDynamic(dyn); break;
            case SCREEN_GG_DIAGRAM:       renderGGDiagramDynamic(dyn); break;
            case SCREEN_CORNER_ANALYZE:   renderCornerAnalyzeDynamic(dyn); break;
            case SCREEN_ALTIMETRY_TOUR:   renderAltimetryTourDynamic(dyn); break;
            case SCREEN_PITCH_SUSPENSION: renderPitchSuspensionDynamic(dyn); break;
            case SCREEN_HORIZON_HUD:      renderHorizonHUDDynamic(dyn); break;
            case SCREEN_MINIMAL_RACE:     renderMinimalRaceDynamic(dyn); break;
            case SCREEN_SESSION_STATS:    renderStatsDynamic(dyn); break;
        }
    }

    void showTareNotice(float rollOffset = 0.0f, float pitchOffset = 0.0f) {
        _lcd.fillRect(10, 36, 110, 58, COLOR_AMBER);
        _lcd.drawRect(9, 35, 112, 60, COLOR_WHITE);
        _lcd.setTextColor(COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setCursor(24, 42);
        _lcd.print("ZERO TARE");
        _lcd.setCursor(16, 56);
        _lcd.print("CALIBRATO OK!");
        _lcd.setCursor(14, 72);
        _lcd.printf("R:%+.1f P:%+.1f", rollOffset, pitchOffset);
        delay(1200);
        _needsFullRedraw = true;
    }

    void showTareResetNotice() {
        _lcd.fillRect(10, 40, 110, 50, COLOR_DARK_GRAY);
        _lcd.drawRect(9, 39, 112, 52, COLOR_WHITE);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setTextSize(1);
        _lcd.setCursor(24, 48);
        _lcd.print("ZERO RESET");
        _lcd.setCursor(16, 64);
        _lcd.print("DEFAULT FABBRICA");
        delay(1000);
        _needsFullRedraw = true;
    }

private:
    SSD1283A _lcd;
    UIScreenMode _currentScreen;
    bool _needsFullRedraw;
    float _prevRoll;
    int16_t _prevNeedleX, _prevNeedleY;
    int16_t _prevDotX, _prevDotY;
    int16_t _prevPitchBarY;
    int16_t _prevHorizonX1, _prevHorizonY1, _prevHorizonX2, _prevHorizonY2;
    bool _wifiActive;
    RadioMode _radioMode;
    bool _autoCycle;
    uint32_t _autoCycleIntervalMs;
    uint32_t _lastCycleMillis;

    void drawBootLogo() {
        _lcd.fillScreen(COLOR_BLACK);
        _lcd.drawRoundRect(10, 20, 110, 90, 8, COLOR_CYAN);
        _lcd.drawRoundRect(12, 22, 106, 86, 6, COLOR_DARK_GRAY);

        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setTextSize(2);
        _lcd.setCursor(20, 36);
        _lcd.print("YAMAHA");

        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setTextSize(3);
        _lcd.setCursor(35, 58);
        _lcd.print("FZ8");

        _lcd.setTextColor(COLOR_GREEN);
        _lcd.setTextSize(1);
        _lcd.setCursor(25, 88);
        _lcd.print("TELEMETRY 10-DOF");
    }

    void drawHeaderRadioBadge() {
        _lcd.setTextSize(1);
        if (_radioMode == RADIO_MODE_DASHBOARD) {
            _lcd.setTextColor(COLOR_GREEN);
            _lcd.setCursor(96, 4);
            _lcd.print("WIFI");
        } else {
            _lcd.setTextColor(COLOR_CYAN);
            _lcd.setCursor(102, 4);
            _lcd.print("BLE");
        }
    }

    bool checkCrashOverlay(const MotorcycleDynamics& dyn) {
        if (dyn.isCrashDetected) {
            _lcd.fillRect(8, 25, 114, 80, COLOR_RED);
            _lcd.drawRect(7, 24, 116, 82, COLOR_WHITE);
            _lcd.setTextColor(COLOR_WHITE);
            _lcd.setTextSize(2);
            _lcd.setCursor(20, 35);
            _lcd.print("CRASH!");
            _lcd.setTextSize(1);
            _lcd.setCursor(16, 62);
            _lcd.print("MOTO A TERRA");
            _lcd.setCursor(14, 80);
            _lcd.print("RADDRIZZA MOTO");
            return true;
        }
        return false;
    }

    // ==========================================================================
    // SCREEN 0: CLASSIC DYNAMICS VIEW (Analog Arc Gauge + Peaks)
    // ==========================================================================
    void drawDynamicsStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(2, 4);
        _lcd.print("FZ8");
        drawHeaderRadioBadge();

        const int cx = 65, cy = 76, r = 50;
        drawTick(cx, cy, r, 0, COLOR_WHITE, 6);
        drawTick(cx, cy, r, -15, COLOR_MID_GRAY, 4);
        drawTick(cx, cy, r, 15, COLOR_MID_GRAY, 4);
        drawTick(cx, cy, r, -30, COLOR_GREEN, 5);
        drawTick(cx, cy, r, 30, COLOR_GREEN, 5);
        drawTick(cx, cy, r, -45, COLOR_AMBER, 5);
        drawTick(cx, cy, r, 45, COLOR_AMBER, 5);
        drawTick(cx, cy, r, -60, COLOR_RED, 6);
        drawTick(cx, cy, r, 60, COLOR_RED, 6);

        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(6, 100); _lcd.print("L MAX");
        _lcd.setCursor(74, 100); _lcd.print("R MAX");
    }

    void renderDynamicsDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        // Top Bar: Altitude & Compass Heading
        _lcd.fillRect(26, 3, 68, 9, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(28, 4);
        _lcd.printf("%dm", (int)dyn.altitudeM);

        _lcd.setTextColor(dyn.isHeadingValid ? COLOR_CYAN : COLOR_MID_GRAY);
        _lcd.setCursor(64, 4);
        _lcd.printf("%s %d%c", dyn.cardinal[0] ? dyn.cardinal : "-", (int)dyn.headingDeg, dyn.isHeadingValid ? ' ' : '~');

        // Center Gauge Arc & Needle
        const int cx = 65, cy = 76, r = 48;
        _lcd.drawLine(cx, cy, _prevNeedleX, _prevNeedleY, COLOR_BLACK);

        float clampedRoll = constrain(dyn.rollDeg, -60.0f, 60.0f);
        float rad = (clampedRoll - 90.0f) * ((float)M_PI / 180.0f);
        int16_t nx = cx + (int16_t)(cosf(rad) * r);
        int16_t ny = cy + (int16_t)(sinf(rad) * r);

        float absRoll = fabsf(dyn.rollDeg);
        uint16_t needleColor = COLOR_GREEN;
        if (dyn.isLeanWarning) needleColor = COLOR_MAGENTA;
        else if (absRoll >= 45.0f) needleColor = COLOR_RED;
        else if (absRoll >= 32.0f) needleColor = COLOR_AMBER;

        if (dyn.isLeanWarning) {
            _lcd.fillRect(22, 18, 86, 10, COLOR_RED);
            _lcd.setTextColor(COLOR_WHITE);
            _lcd.setTextSize(1);
            _lcd.setCursor(26, 19);
            _lcd.print("! LEAN WARN !");
        } else {
            _lcd.fillRect(22, 18, 86, 10, COLOR_BLACK);
        }

        _lcd.drawLine(cx, cy, nx, ny, needleColor);
        _lcd.fillCircle(cx, cy, 3, COLOR_WHITE);
        _prevNeedleX = nx; _prevNeedleY = ny;

        // Digital Angle Value
        _lcd.fillRect(28, 54, 74, 20, COLOR_BLACK);
        _lcd.setTextSize(2);
        _lcd.setTextColor(needleColor);
        char angleBuf[10];
        if (clampedRoll < -1.0f) snprintf(angleBuf, sizeof(angleBuf), "L%2d*", (int)absRoll);
        else if (clampedRoll > 1.0f) snprintf(angleBuf, sizeof(angleBuf), "R%2d*", (int)absRoll);
        else snprintf(angleBuf, sizeof(angleBuf), " 0*");
        _lcd.setCursor(38, 57);
        _lcd.print(angleBuf);

        // Peaks
        _lcd.fillRect(4, 108, 58, 18, COLOR_BLACK);
        _lcd.setTextSize(2);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(6, 110);
        _lcd.printf("%2d*", (int)dyn.maxLeanLeft);

        _lcd.fillRect(72, 108, 56, 18, COLOR_BLACK);
        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(74, 110);
        _lcd.printf("%2d*", (int)dyn.maxLeanRight);

        // G-Force Indicator
        _lcd.fillRect(25, 82, 80, 10, COLOR_BLACK);
        _lcd.setTextSize(1);
        if (dyn.isBraking) {
            _lcd.setTextColor(COLOR_RED);
            _lcd.setCursor(35, 83);
            _lcd.printf("BRK: %.2fG", dyn.gLongitudinal);
        } else if (dyn.isAccelerating) {
            _lcd.setTextColor(COLOR_GREEN);
            _lcd.setCursor(35, 83);
            _lcd.printf("ACC: +%.2fG", dyn.gLongitudinal);
        } else {
            _lcd.setTextColor(COLOR_MID_GRAY);
            _lcd.setCursor(40, 83);
            _lcd.printf("G: %.2f", dyn.gLongitudinal);
        }
    }

    // ==========================================================================
    // SCREEN 1: MOTOGP RACE VIEW (Giant digits, Side LED segment bars, G-Braking)
    // ==========================================================================
    void drawRaceMotoGPStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("MOTO GP");
        drawHeaderRadioBadge();

        // Lateral segment bar outlines (Left & Right columns)
        _lcd.drawRect(2, 20, 8, 86, COLOR_DARK_GRAY);
        _lcd.drawRect(120, 20, 8, 86, COLOR_DARK_GRAY);

        // Horizontal tick dividers
        for (int y = 20; y <= 106; y += 14) {
            _lcd.drawFastHLine(3, y, 6, COLOR_MID_GRAY);
            _lcd.drawFastHLine(121, y, 6, COLOR_MID_GRAY);
        }

        _lcd.drawFastHLine(0, 112, 130, COLOR_DARK_GRAY);
    }

    void renderRaceMotoGPDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        float absRoll = fabsf(dyn.rollDeg);
        float clampedRoll = constrain(dyn.rollDeg, -60.0f, 60.0f);

        // Color theme based on lean severity
        uint16_t themeCol = COLOR_GREEN;
        if (dyn.isLeanWarning) themeCol = COLOR_MAGENTA;
        else if (absRoll >= 45.0f) themeCol = COLOR_RED;
        else if (absRoll >= 32.0f) themeCol = COLOR_AMBER;

        // Giant Lean Digits (Size 4 font in center)
        _lcd.fillRect(16, 32, 98, 38, COLOR_BLACK);
        _lcd.setTextSize(4);
        _lcd.setTextColor(themeCol);
        if (absRoll < 10.0f) {
            _lcd.setCursor(44, 34);
        } else {
            _lcd.setCursor(30, 34);
        }
        _lcd.printf("%d*", (int)absRoll);

        // Direction / Status Subtitle
        _lcd.fillRect(20, 74, 90, 12, COLOR_BLACK);
        _lcd.setTextSize(1);
        if (dyn.isLeanWarning) {
            _lcd.setTextColor(COLOR_MAGENTA);
            _lcd.setCursor(24, 76);
            _lcd.print("! LEAN LIMIT !");
        } else if (clampedRoll < -2.0f) {
            _lcd.setTextColor(COLOR_CYAN);
            _lcd.setCursor(32, 76);
            _lcd.printf("<-- SX (%d*)", (int)absRoll);
        } else if (clampedRoll > 2.0f) {
            _lcd.setTextColor(COLOR_AMBER);
            _lcd.setCursor(34, 76);
            _lcd.printf("DX (%d*) -->", (int)absRoll);
        } else {
            _lcd.setTextColor(COLOR_MID_GRAY);
            _lcd.setCursor(40, 76);
            _lcd.print("VERTICALE");
        }

        // Peak Lean Indicator underneath
        _lcd.fillRect(20, 92, 90, 12, COLOR_BLACK);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(22, 94);
        _lcd.printf("L:%d*  R:%d*", (int)dyn.maxLeanLeft, (int)dyn.maxLeanRight);

        // Lateral LED-Style Segment Bars (0 to 55 deg = 6 blocks)
        int numSegments = (int)(absRoll / 8.5f);
        if (numSegments > 6) numSegments = 6;

        // Clear both bars
        _lcd.fillRect(3, 21, 6, 84, COLOR_BLACK);
        _lcd.fillRect(121, 21, 6, 84, COLOR_BLACK);

        if (clampedRoll < -2.0f) {
            // Fill left bar from bottom upwards
            for (int s = 0; s < numSegments; s++) {
                int by = 92 - (s * 14);
                uint16_t bCol = (s >= 5) ? COLOR_RED : ((s >= 3) ? COLOR_AMBER : COLOR_CYAN);
                _lcd.fillRect(3, by, 6, 12, bCol);
            }
        } else if (clampedRoll > 2.0f) {
            // Fill right bar from bottom upwards
            for (int s = 0; s < numSegments; s++) {
                int by = 92 - (s * 14);
                uint16_t bCol = (s >= 5) ? COLOR_RED : ((s >= 3) ? COLOR_AMBER : COLOR_ORANGE);
                _lcd.fillRect(121, by, 6, 12, bCol);
            }
        }

        // Bottom G-Force Bar
        _lcd.fillRect(0, 116, 130, 14, COLOR_BLACK);
        _lcd.setTextSize(1);
        if (dyn.isBraking) {
            _lcd.setTextColor(COLOR_RED);
            _lcd.setCursor(14, 118);
            _lcd.printf("STACCATA: -%.2f G", dyn.maxBrakingG);
        } else if (dyn.isAccelerating) {
            _lcd.setTextColor(COLOR_GREEN);
            _lcd.setCursor(14, 118);
            _lcd.printf("ACCEL   : +%.2f G", dyn.gLongitudinal);
        } else {
            _lcd.setTextColor(COLOR_WHITE);
            _lcd.setCursor(14, 118);
            _lcd.printf("G-LON: %+.2f G", dyn.gLongitudinal);
        }
    }

    // ==========================================================================
    // SCREEN 2: G-G ACCELERATION DIAGRAM (Traction Circle)
    // ==========================================================================
    void drawGGDiagramStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("G-G CIRCLE");
        drawHeaderRadioBadge();

        const int cx = 65, cy = 66;
        _lcd.drawCircle(cx, cy, 18, COLOR_DARK_GRAY); // 0.5 G
        _lcd.drawCircle(cx, cy, 36, COLOR_DARK_GRAY); // 1.0 G
        _lcd.drawCircle(cx, cy, 48, COLOR_MID_GRAY);  // 1.3 G limit

        _lcd.drawFastHLine(cx - 50, cy, 100, COLOR_DARK_GRAY);
        _lcd.drawFastVLine(cx, cy - 50, 100, COLOR_DARK_GRAY);

        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(cx - 10, cy - 48); _lcd.print("BRK");
        _lcd.setCursor(cx - 10, cy + 40); _lcd.print("ACC");
        _lcd.setCursor(cx - 48, cy - 8);  _lcd.print("L");
        _lcd.setCursor(cx + 42, cy - 8);  _lcd.print("R");
    }

    void renderGGDiagramDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        const int cx = 65, cy = 66;
        const float pxPerG = 36.0f;

        _lcd.fillCircle(_prevDotX, _prevDotY, 3, COLOR_BLACK);

        // Re-draw circle lines underneath dot
        _lcd.drawCircle(cx, cy, 18, COLOR_DARK_GRAY);
        _lcd.drawCircle(cx, cy, 36, COLOR_DARK_GRAY);
        _lcd.drawFastHLine(cx - 50, cy, 100, COLOR_DARK_GRAY);
        _lcd.drawFastVLine(cx, cy - 50, 100, COLOR_DARK_GRAY);

        int16_t dotX = cx + (int16_t)(dyn.gLateral * pxPerG);
        int16_t dotY = cy - (int16_t)(dyn.gLongitudinal * pxPerG);

        dotX = constrain(dotX, cx - 48, cx + 48);
        dotY = constrain(dotY, cy - 48, cy + 48);

        _lcd.fillCircle(dotX, dotY, 3, COLOR_GREEN);
        _prevDotX = dotX; _prevDotY = dotY;

        _lcd.fillRect(0, 116, 130, 14, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(4, 120);
        _lcd.printf("LON:%+.2fG", dyn.gLongitudinal);
        _lcd.setCursor(72, 120);
        _lcd.printf("LAT:%+.2fG", dyn.gLateral);
    }

    // ==========================================================================
    // SCREEN 3: CORNER ANALYZER & APEX MONITOR (Live telemetric curve inspection)
    // ==========================================================================
    void drawCornerAnalyzeStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("CORNER APEX");
        drawHeaderRadioBadge();

        // Box active corner
        _lcd.drawRoundRect(2, 18, 126, 44, 4, COLOR_MID_GRAY);

        // Separator
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(4, 68);
        _lcd.print("ULTIME CURVE:");
        _lcd.drawFastHLine(0, 78, 130, COLOR_DARK_GRAY);
    }

    void renderCornerAnalyzeDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        // Active Corner Box
        _lcd.fillRect(4, 20, 122, 40, COLOR_BLACK);
        _lcd.setTextSize(1);

        if (dyn.isInsideCorner) {
            uint16_t cCol = (dyn.rollDeg < 0.0f) ? COLOR_CYAN : COLOR_AMBER;
            _lcd.setTextColor(cCol);
            _lcd.setTextSize(2);
            _lcd.setCursor(8, 24);
            _lcd.printf("%s %d*", (dyn.rollDeg < 0.0f) ? "SX" : "DX", (int)dyn.currentCornerMaxRoll);

            _lcd.setTextSize(1);
            _lcd.setTextColor(COLOR_WHITE);
            _lcd.setCursor(80, 24);
            _lcd.printf("%.1fs", dyn.currentCornerDuration);

            _lcd.setTextColor(COLOR_MID_GRAY);
            _lcd.setCursor(8, 44);
            _lcd.printf("Vel. Rollio: %d*/s", (int)fabsf(dyn.rollRateDps));
        } else {
            _lcd.setTextColor(COLOR_GREEN);
            _lcd.setTextSize(1);
            _lcd.setCursor(12, 28);
            _lcd.print("MOTO SUL DRITTO");
            _lcd.setTextColor(COLOR_MID_GRAY);
            _lcd.setCursor(12, 44);
            _lcd.print("Pronto per la curva");
        }

        // Last 3 Corners from History
        for (int i = 0; i < 3; i++) {
            int yPos = 82 + (i * 16);
            _lcd.fillRect(2, yPos, 126, 14, COLOR_BLACK);
            _lcd.setTextSize(1);

            if (i < dyn.cornerHistoryCount) {
                const CornerRecord& c = dyn.lastCorners[i];
                uint16_t col = c.isLeft ? COLOR_CYAN : COLOR_AMBER;
                _lcd.setTextColor(col);
                _lcd.setCursor(4, yPos + 2);
                _lcd.printf("#%d %s", i + 1, c.isLeft ? "SX" : "DX");

                _lcd.setTextColor(COLOR_WHITE);
                _lcd.setCursor(42, yPos + 2);
                _lcd.printf("%d*", (int)c.maxRollDeg);

                _lcd.setTextColor(COLOR_MID_GRAY);
                _lcd.setCursor(72, yPos + 2);
                _lcd.printf("%d*/s %.1fs", (int)c.maxRollRateDps, c.durationSec);
            } else {
                _lcd.setTextColor(COLOR_DARK_GRAY);
                _lcd.setCursor(4, yPos + 2);
                _lcd.printf("#%d ---", i + 1);
            }
        }
    }

    // ==========================================================================
    // SCREEN 4: MOUNTAIN TOUR & ALTIMETRY (Bussola grande, Passi, Quota, D+)
    // ==========================================================================
    void drawAltimetryTourStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("PASSI / TOUR");
        drawHeaderRadioBadge();

        // Compass Rose Box
        _lcd.drawRoundRect(2, 18, 126, 44, 4, COLOR_MID_GRAY);

        _lcd.drawFastHLine(0, 66, 130, COLOR_DARK_GRAY);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setTextSize(1);
        _lcd.setCursor(4, 72);  _lcd.print("QUOTA  :");
        _lcd.setCursor(4, 86);  _lcd.print("D+ GAIN:");
        _lcd.setCursor(4, 100); _lcd.print("ASSETTO:");
        _lcd.setCursor(4, 114); _lcd.print("TEMP   :");
    }

    void renderAltimetryTourDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        // Big Compass Display
        _lcd.fillRect(4, 20, 122, 40, COLOR_BLACK);

        // Cardinal Letter (Size 3, e.g. "NW")
        _lcd.setTextSize(3);
        _lcd.setTextColor(dyn.isHeadingValid ? COLOR_CYAN : COLOR_MID_GRAY);
        _lcd.setCursor(14, 28);
        _lcd.printf("%-2s", dyn.cardinal[0] ? dyn.cardinal : "-");

        // Degree Number (Size 2)
        _lcd.setTextSize(2);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(68, 26);
        _lcd.printf("%3d*", (int)dyn.headingDeg);

        // Valid vs Estimated
        _lcd.setTextSize(1);
        if (dyn.isHeadingValid) {
            _lcd.setTextColor(COLOR_GREEN);
            _lcd.setCursor(68, 46);
            _lcd.print("[VALIDA]");
        } else {
            _lcd.setTextColor(COLOR_AMBER);
            _lcd.setCursor(68, 46);
            _lcd.print("[IN PIEGA]");
        }

        // Altimetry Metrics
        _lcd.setTextSize(1);

        // Altitude
        _lcd.fillRect(58, 72, 70, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(58, 72);
        _lcd.printf("%d m", (int)dyn.altitudeM);

        // D+ Elevation Gain
        _lcd.fillRect(58, 86, 70, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_GREEN);
        _lcd.setCursor(58, 86);
        _lcd.printf("+%d m", (int)dyn.totalElevationGainM);

        // Suspension Pitch (Dive/Squat)
        _lcd.fillRect(58, 100, 70, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(58, 100);
        _lcd.printf("%+.1f deg", dyn.pitchDeg);

        // Temperature
        _lcd.fillRect(58, 114, 70, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(58, 114);
        _lcd.printf("%.1f *C", dyn.tempC);
    }

    // ==========================================================================
    // SCREEN 5: PITCH & WHEELIE / STOPPIE INCLINOMETER (Assetto Dinamico)
    // ==========================================================================
    void drawPitchSuspensionStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("ASSETTO / PITCH");
        drawHeaderRadioBadge();

        // Vertical pitch ladder frame (center around X=65, Y=26..90)
        _lcd.drawRect(56, 26, 18, 64, COLOR_DARK_GRAY);
        _lcd.drawFastHLine(52, 26, 6, COLOR_RED);    // +20 deg (Wheelie)
        _lcd.drawFastHLine(54, 42, 4, COLOR_AMBER);  // +10 deg
        _lcd.drawFastHLine(50, 58, 8, COLOR_WHITE);  //   0 deg (Level)
        _lcd.drawFastHLine(54, 74, 4, COLOR_CYAN);   // -10 deg
        _lcd.drawFastHLine(52, 90, 6, COLOR_BLUE);   // -20 deg (Stoppie)

        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(4, 38); _lcd.print("PITCH");
        _lcd.setCursor(82, 38); _lcd.print("ROLL");
    }

    void renderPitchSuspensionDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        // Banner Alert at Top (Y=16..24)
        _lcd.fillRect(4, 16, 122, 9, COLOR_BLACK);
        _lcd.setTextSize(1);
        if (dyn.isWheelie) {
            _lcd.fillRect(8, 16, 114, 9, COLOR_YELLOW);
            _lcd.setTextColor(COLOR_BLACK);
            _lcd.setCursor(22, 17);
            _lcd.print("! WHEELIE / SQUAT !");
        } else if (dyn.isStoppie) {
            _lcd.fillRect(8, 16, 114, 9, COLOR_RED);
            _lcd.setTextColor(COLOR_WHITE);
            _lcd.setCursor(16, 17);
            _lcd.print("! STOPPIE / DIVE !");
        } else {
            float fLoad = constrain(50.0f - dyn.pitchDeg * 2.5f - dyn.gLongitudinal * 20.0f, 10.0f, 90.0f);
            _lcd.setTextColor(COLOR_MID_GRAY);
            _lcd.setCursor(14, 17);
            _lcd.printf("CARICO F:%d%% R:%d%%", (int)fLoad, (int)(100.0f - fLoad));
        }

        // Clear previous bar inside ladder
        _lcd.fillRect(57, 27, 16, 62, COLOR_BLACK);
        _lcd.drawFastHLine(57, 58, 16, COLOR_DARK_GRAY); // Level marker

        // Draw vertical pitch bar from center (58) to target Y
        float clampedP = constrain(dyn.pitchDeg, -20.0f, 20.0f);
        int targetY = 58 - (int)(clampedP * 1.55f);
        targetY = constrain(targetY, 27, 89);

        if (targetY < 58) {
            // Pitch UP (Wheelie/Squat)
            uint16_t barCol = (dyn.pitchDeg >= 6.5f) ? COLOR_RED : COLOR_AMBER;
            _lcd.fillRect(58, targetY, 14, 58 - targetY, barCol);
        } else if (targetY > 58) {
            // Pitch DOWN (Dive/Stoppie)
            uint16_t barCol = (dyn.pitchDeg <= -5.5f) ? COLOR_RED : COLOR_CYAN;
            _lcd.fillRect(58, 58, 14, targetY - 58, barCol);
        }
        _lcd.drawFastHLine(54, targetY, 22, COLOR_WHITE);

        // Digital pitch value (Left)
        _lcd.fillRect(2, 48, 50, 18, COLOR_BLACK);
        _lcd.setTextSize(2);
        _lcd.setTextColor((fabsf(dyn.pitchDeg) > 6.0f) ? COLOR_RED : COLOR_WHITE);
        _lcd.setCursor(4, 50);
        _lcd.printf("%+.0f*", dyn.pitchDeg);

        // Digital roll value (Right)
        _lcd.fillRect(80, 48, 48, 18, COLOR_BLACK);
        _lcd.setTextSize(2);
        _lcd.setTextColor((dyn.rollDeg < 0.0f) ? COLOR_CYAN : (dyn.rollDeg > 0.0f ? COLOR_AMBER : COLOR_WHITE));
        _lcd.setCursor(80, 50);
        _lcd.printf("%s%d*", (dyn.rollDeg < -1.0f) ? "L" : (dyn.rollDeg > 1.0f ? "R" : " "), (int)fabsf(dyn.rollDeg));

        // Peak Records at bottom (Y=98..128)
        _lcd.fillRect(0, 96, 130, 32, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(4, 100);
        _lcd.printf("WHEELIE MAX : %+.1f*", dyn.maxPitchUp);

        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 114);
        _lcd.printf("STACCATA MAX: %+.1f*", dyn.maxPitchDown);
    }

    // ==========================================================================
    // SCREEN 6: ARTIFICIAL HORIZON HUD (Aviation Style Cockpit Horizon)
    // ==========================================================================
    void drawHorizonHUDStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("HORIZON HUD");
        drawHeaderRadioBadge();

        // Round HUD viewport boundary
        _lcd.drawRoundRect(4, 18, 122, 92, 8, COLOR_DARK_GRAY);

        // Fixed Aircraft Crosshair reticle (Motorcycle chassis reference)
        _lcd.drawFastHLine(44, 64, 14, COLOR_YELLOW);
        _lcd.drawFastVLine(44, 64, 4, COLOR_YELLOW);
        _lcd.drawFastHLine(72, 64, 14, COLOR_YELLOW);
        _lcd.drawFastVLine(85, 64, 4, COLOR_YELLOW);
        _lcd.fillCircle(65, 64, 2, COLOR_YELLOW);
    }

    void renderHorizonHUDDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        // Erase previous horizon line
        _lcd.drawLine(_prevHorizonX1, _prevHorizonY1, _prevHorizonX2, _prevHorizonY2, COLOR_BLACK);
        _lcd.drawLine(_prevHorizonX1, _prevHorizonY1 - 1, _prevHorizonX2, _prevHorizonY2 - 1, COLOR_BLACK);

        // Calculate horizon line with roll angle & pitch translation
        float clampedPitch = constrain(dyn.pitchDeg, -16.0f, 16.0f);
        int cy = 64 + (int)(clampedPitch * 1.6f);
        cy = constrain(cy, 28, 100);

        float rad = -dyn.rollDeg * ((float)M_PI / 180.0f);
        const float halfLen = 46.0f;
        int16_t x1 = 65 - (int16_t)(cosf(rad) * halfLen);
        int16_t y1 = cy - (int16_t)(sinf(rad) * halfLen);
        int16_t x2 = 65 + (int16_t)(cosf(rad) * halfLen);
        int16_t y2 = cy + (int16_t)(sinf(rad) * halfLen);

        // Clamp coordinates within HUD frame
        x1 = constrain(x1, 6, 124);
        x2 = constrain(x2, 6, 124);
        y1 = constrain(y1, 20, 108);
        y2 = constrain(y2, 20, 108);

        // Draw new horizon line (Cyan)
        _lcd.drawLine(x1, y1, x2, y2, COLOR_CYAN);
        _lcd.drawLine(x1, y1 - 1, x2, y2 - 1, COLOR_CYAN);
        _prevHorizonX1 = x1; _prevHorizonY1 = y1;
        _prevHorizonX2 = x2; _prevHorizonY2 = y2;

        // Redraw fixed aircraft center reticle
        _lcd.drawFastHLine(44, 64, 14, COLOR_YELLOW);
        _lcd.drawFastHLine(72, 64, 14, COLOR_YELLOW);
        _lcd.fillCircle(65, 64, 2, COLOR_YELLOW);

        // Numerical HUD Telemetry
        _lcd.fillRect(8, 22, 38, 10, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(8, 23);
        _lcd.printf("R:%2d*", (int)fabsf(dyn.rollDeg));

        _lcd.fillRect(84, 22, 38, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_GREEN);
        _lcd.setCursor(84, 23);
        _lcd.printf("P:%+2d*", (int)dyn.pitchDeg);

        // Bottom HUD Bar (Y=114..128)
        _lcd.fillRect(0, 114, 130, 14, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(6, 116);
        _lcd.printf("G: %.2f", dyn.totalG);

        _lcd.setTextColor(dyn.isHeadingValid ? COLOR_CYAN : COLOR_MID_GRAY);
        _lcd.setCursor(68, 116);
        _lcd.printf("%s %3d*", dyn.cardinal[0] ? dyn.cardinal : "-", (int)dyn.headingDeg);
    }

    // ==========================================================================
    // SCREEN 7: MINIMAL ULTRA-HIGH CONTRAST RACE HUD (Anti-glare Night / Track)
    // ==========================================================================
    void drawMinimalRaceStaticLayout() {
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("MINIMAL RACE");
        drawHeaderRadioBadge();
    }

    void renderMinimalRaceDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        float absRoll = fabsf(dyn.rollDeg);

        // Perimeter Warning Border
        if (dyn.isLeanWarning) {
            _lcd.drawRect(0, 0, 130, 130, COLOR_RED);
            _lcd.drawRect(1, 1, 128, 128, COLOR_RED);
        } else if (absRoll >= 42.0f) {
            _lcd.drawRect(0, 0, 130, 130, COLOR_AMBER);
            _lcd.drawRect(1, 1, 128, 128, COLOR_AMBER);
        } else {
            _lcd.drawRect(0, 0, 130, 130, COLOR_BLACK);
            _lcd.drawRect(1, 1, 128, 128, COLOR_BLACK);
        }

        // Giant Lean Number (Font Size 4/5)
        _lcd.fillRect(24, 30, 82, 42, COLOR_BLACK);
        _lcd.setTextSize(5);
        uint16_t numColor = (dyn.isLeanWarning) ? COLOR_MAGENTA : ((absRoll >= 42.0f) ? COLOR_RED : ((absRoll >= 30.0f) ? COLOR_AMBER : COLOR_GREEN));
        _lcd.setTextColor(numColor);
        if (absRoll < 10.0f) {
            _lcd.setCursor(48, 34);
        } else {
            _lcd.setCursor(28, 34);
        }
        _lcd.printf("%d*", (int)absRoll);

        // Directional Chevrons
        _lcd.fillRect(4, 36, 18, 28, COLOR_BLACK);
        _lcd.fillRect(108, 36, 18, 28, COLOR_BLACK);
        _lcd.setTextSize(3);
        if (dyn.rollDeg < -2.0f) {
            _lcd.setTextColor(COLOR_CYAN);
            _lcd.setCursor(4, 40);
            _lcd.print("<<");
        } else if (dyn.rollDeg > 2.0f) {
            _lcd.setTextColor(COLOR_AMBER);
            _lcd.setCursor(108, 40);
            _lcd.print(">>");
        }

        // Bottom Peak Records
        _lcd.fillRect(6, 88, 118, 36, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(24, 90);
        _lcd.print("RECORD SESSIONE");

        _lcd.setTextSize(2);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(16, 104);
        _lcd.printf("L%d*", (int)dyn.maxLeanLeft);

        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(76, 104);
        _lcd.printf("R%d*", (int)dyn.maxLeanRight);
    }

    // ==========================================================================
    // SCREEN 8: SESSION STATS & PEAKS
    // ==========================================================================
    void drawStatsStaticLayout() {
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(4, 4);
        _lcd.print("SESSION STATS");
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);
        drawHeaderRadioBadge();

        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(4, 20);  _lcd.print("PIEGA SX  :");
        _lcd.setCursor(4, 34);  _lcd.print("PIEGA DX  :");
        _lcd.setCursor(4, 48);  _lcd.print("STACCATA  :");
        _lcd.setCursor(4, 62);  _lcd.print("ACCEL MAX :");
        _lcd.setCursor(4, 76);  _lcd.print("WHEELIE   :");
        _lcd.setCursor(4, 90);  _lcd.print("LAST CORN :");
        _lcd.setCursor(4, 104); _lcd.print("D+ GAIN   :");
        _lcd.setCursor(4, 118); _lcd.print("TEMPO     :");
    }

    void renderStatsDynamic(const MotorcycleDynamics& dyn) {
        if (checkCrashOverlay(dyn)) return;

        _lcd.setTextSize(1);

        // Max Lean L
        _lcd.fillRect(78, 20, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(78, 20);
        _lcd.printf("%.1f deg", dyn.maxLeanLeft);

        // Max Lean R
        _lcd.fillRect(78, 34, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(78, 34);
        _lcd.printf("%.1f deg", dyn.maxLeanRight);

        // Max Braking G
        _lcd.fillRect(78, 48, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_RED);
        _lcd.setCursor(78, 48);
        _lcd.printf("-%.2f G", dyn.maxBrakingG);

        // Max Accel G
        _lcd.fillRect(78, 62, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_GREEN);
        _lcd.setCursor(78, 62);
        _lcd.printf("+%.2f G", dyn.maxAccelG);

        // Max Wheelie Angle
        _lcd.fillRect(78, 76, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_YELLOW);
        _lcd.setCursor(78, 76);
        _lcd.printf("%+.1f deg", dyn.maxPitchUp);

        // Last Corner from Analyzer
        _lcd.fillRect(78, 90, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(78, 90);
        if (dyn.cornerHistoryCount > 0) {
            _lcd.printf("%s %d* %.1fs", 
                dyn.lastCorners[0].isLeft ? "L" : "R",
                (int)dyn.lastCorners[0].maxRollDeg,
                dyn.lastCorners[0].durationSec);
        } else {
            _lcd.print("---");
        }

        // D+ Elevation Gain
        _lcd.fillRect(78, 104, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_GREEN);
        _lcd.setCursor(78, 104);
        _lcd.printf("+%d m", (int)dyn.totalElevationGainM);

        // Run Time
        uint32_t sec = (millis() - dyn.sessionStartTime) / 1000;
        _lcd.fillRect(78, 118, 50, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(78, 118);
        _lcd.printf("%02d:%02d", (int)(sec / 60), (int)(sec % 60));
    }

    void drawTick(int cx, int cy, int r, float deg, uint16_t color, int length) {
        float rad = (deg - 90.0f) * ((float)M_PI / 180.0f);
        int x1 = cx + (int)(cosf(rad) * r);
        int y1 = cy + (int)(sinf(rad) * r);
        int x2 = cx + (int)(cosf(rad) * (r - length));
        int y2 = cy + (int)(sinf(rad) * (r - length));
        _lcd.drawLine(x1, y1, x2, y2, color);
    }
};

#endif // DISPLAY_UI_H
