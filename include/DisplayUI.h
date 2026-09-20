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
#define COLOR_RED           0xF800
#define COLOR_YELLOW        0xFFE0
#define COLOR_BLUE          0x001F

enum UIScreenMode {
    SCREEN_DYNAMICS = 0,    // Main View: Arc Gauge, Lean Degrees, Peak L/R, G-Force
    SCREEN_GG_DIAGRAM,      // Friction Circle (G-G Circle)
    SCREEN_SESSION_STATS,   // Peaks summary, altitude, duration
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
        _wifiActive(false) {}

    bool begin() {
        // Initialize SPI with custom pins on ESP32-C3
        SPI.begin(LCD_SCK_PIN, -1, LCD_MOSI_PIN, LCD_CS_PIN);
        
        // Initialize SSD1283A controller
        _lcd.init();
        _lcd.setRotation(0); // 0 or 2 depending on physical mounting orientation
        _lcd.fillScreen(COLOR_BLACK);

        // Turn on backlight (if controlled by pin)
        if (LCD_LED_PIN >= 0) {
            pinMode(LCD_LED_PIN, OUTPUT);
            digitalWrite(LCD_LED_PIN, HIGH);
        }

        drawBootLogo();
        delay(1200);

        _lcd.fillScreen(COLOR_BLACK);
        _needsFullRedraw = true;
        return true;
    }

    void nextScreen() {
        _currentScreen = (UIScreenMode)((_currentScreen + 1) % SCREEN_COUNT);
        _needsFullRedraw = true;
    }

    void setWifiStatus(bool active) {
        if (_wifiActive != active) {
            _wifiActive = active;
            _needsFullRedraw = true;
        }
    }

    void update(const MotorcycleDynamics& dyn) {
        if (_needsFullRedraw) {
            _lcd.fillScreen(COLOR_BLACK);
            switch (_currentScreen) {
                case SCREEN_DYNAMICS:
                    drawDynamicsStaticLayout();
                    break;
                case SCREEN_GG_DIAGRAM:
                    drawGGDiagramStaticLayout();
                    break;
                case SCREEN_SESSION_STATS:
                    drawStatsStaticLayout();
                    break;
            }
            _needsFullRedraw = false;
        }

        switch (_currentScreen) {
            case SCREEN_DYNAMICS:
                renderDynamicsDynamic(dyn);
                break;
            case SCREEN_GG_DIAGRAM:
                renderGGDiagramDynamic(dyn);
                break;
            case SCREEN_SESSION_STATS:
                renderStatsDynamic(dyn);
                break;
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
    bool _wifiActive;

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

    // --------------------------------------------------------------------------
    // SCREEN 1: DYNAMICS VIEW
    // --------------------------------------------------------------------------
    void drawDynamicsStaticLayout() {
        // Top status bar
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);

        // Header text
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(2, 4);
        _lcd.print("FZ8");

        if (_wifiActive) {
            _lcd.setTextColor(COLOR_GREEN);
            _lcd.setCursor(95, 4);
            _lcd.print("WIFI");
        }

        // Center Gauge Arc reference markings (-60 to +60 deg)
        const int cx = 65;
        const int cy = 76;
        const int r = 50;

        // Draw tick marks for 0, 15, 30, 45, 60
        drawTick(cx, cy, r, 0, COLOR_WHITE, 6);
        drawTick(cx, cy, r, -15, COLOR_MID_GRAY, 4);
        drawTick(cx, cy, r, 15, COLOR_MID_GRAY, 4);
        drawTick(cx, cy, r, -30, COLOR_GREEN, 5);
        drawTick(cx, cy, r, 30, COLOR_GREEN, 5);
        drawTick(cx, cy, r, -45, COLOR_AMBER, 6);
        drawTick(cx, cy, r, 45, COLOR_AMBER, 6);
        drawTick(cx, cy, r, -55, COLOR_RED, 5);
        drawTick(cx, cy, r, 55, COLOR_RED, 5);

        // Bottom separator
        _lcd.drawFastHLine(0, 94, 130, COLOR_DARK_GRAY);
        _lcd.drawFastVLine(65, 94, 36, COLOR_DARK_GRAY);

        // Labels for peak cards
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(4, 97);
        _lcd.print("L MAX");
        _lcd.setCursor(72, 97);
        _lcd.print("R MAX");
    }

    void renderDynamicsDynamic(const MotorcycleDynamics& dyn) {
        // Top Bar: Altitude & Temp
        _lcd.fillRect(30, 3, 62, 9, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(32, 4);
        _lcd.printf("%dm", (int)dyn.altitudeM);

        _lcd.setCursor(70, 4);
        _lcd.printf("%dC", (int)dyn.tempC);

        // Center Gauge Arc & Needle
        const int cx = 65;
        const int cy = 76;
        const int r = 48;

        // Erase old needle
        _lcd.drawLine(cx, cy, _prevNeedleX, _prevNeedleY, COLOR_BLACK);

        // Calculate new needle position (-60 to +60 deg mapped)
        float clampedRoll = constrain(dyn.rollDeg, -60.0f, 60.0f);
        // 0 deg points up (angle = -90 deg in standard polar coordinates)
        float rad = (clampedRoll - 90.0f) * ((float)M_PI / 180.0f);
        int16_t nx = cx + (int16_t)(cosf(rad) * r);
        int16_t ny = cy + (int16_t)(sinf(rad) * r);

        // Choose color based on severity
        float absRoll = fabsf(dyn.rollDeg);
        uint16_t needleColor = COLOR_GREEN;
        if (absRoll >= 45.0f) needleColor = COLOR_RED;
        else if (absRoll >= 32.0f) needleColor = COLOR_AMBER;

        // Draw new needle
        _lcd.drawLine(cx, cy, nx, ny, needleColor);
        _lcd.fillCircle(cx, cy, 3, COLOR_WHITE);
        _prevNeedleX = nx;
        _prevNeedleY = ny;

        // Digital Angle Value (Large Font)
        _lcd.fillRect(28, 54, 74, 20, COLOR_BLACK);
        _lcd.setTextSize(2);
        _lcd.setTextColor(needleColor);
        
        char angleBuf[10];
        if (clampedRoll < -1.0f) {
            snprintf(angleBuf, sizeof(angleBuf), "L%2d*", (int)absRoll);
        } else if (clampedRoll > 1.0f) {
            snprintf(angleBuf, sizeof(angleBuf), "R%2d*", (int)absRoll);
        } else {
            snprintf(angleBuf, sizeof(angleBuf), " 0*");
        }
        _lcd.setCursor(38, 57);
        _lcd.print(angleBuf);

        // Bottom Peak Cards (L MAX & R MAX)
        _lcd.fillRect(4, 108, 58, 18, COLOR_BLACK);
        _lcd.setTextSize(2);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(6, 110);
        _lcd.printf("%2d*", (int)dyn.maxLeanLeft);

        _lcd.fillRect(72, 108, 56, 18, COLOR_BLACK);
        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(76, 110);
        _lcd.printf("%2d*", (int)dyn.maxLeanRight);

        // Longitudinal G-Force Indicator (Staccata / Accelerazione)
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

    // --------------------------------------------------------------------------
    // SCREEN 2: G-G FRICTION CIRCLE
    // --------------------------------------------------------------------------
    void drawGGDiagramStaticLayout() {
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(24, 4);
        _lcd.print("G-G DIAGRAM");
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);

        const int cx = 65;
        const int cy = 72;

        // Draw concentric G-rings: 0.5G (radius 18), 1.0G (radius 36), 1.4G (radius 50)
        _lcd.drawCircle(cx, cy, 18, COLOR_DARK_GRAY);
        _lcd.drawCircle(cx, cy, 36, COLOR_MID_GRAY);
        _lcd.drawCircle(cx, cy, 50, COLOR_DARK_GRAY);

        // Crosshairs
        _lcd.drawFastHLine(15, cy, 100, COLOR_DARK_GRAY);
        _lcd.drawFastVLine(cx, 22, 100, COLOR_DARK_GRAY);

        // Ring labels
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(68, cy - 34);
        _lcd.print("1.0G");
    }

    void renderGGDiagramDynamic(const MotorcycleDynamics& dyn) {
        const int cx = 65;
        const int cy = 72;
        const float pixelsPerG = 36.0f; // 1.0G = 36 pixels

        // Erase old dot
        _lcd.fillCircle(_prevDotX, _prevDotY, 3, COLOR_BLACK);
        // Redraw grid pixel if needed
        if (_prevDotX == cx || _prevDotY == cy) {
            _lcd.drawPixel(_prevDotX, _prevDotY, COLOR_DARK_GRAY);
        }

        // New dot coordinates
        // X-axis: Lateral G (ay)
        // Y-axis: Longitudinal G (ax: up = accel, down = braking)
        int16_t dx = cx + (int16_t)(dyn.gLateral * pixelsPerG);
        int16_t dy = cy - (int16_t)(dyn.gLongitudinal * pixelsPerG);

        dx = constrain(dx, 16, 114);
        dy = constrain(dy, 24, 120);

        uint16_t dotColor = (dyn.totalG > 1.0f) ? COLOR_RED : COLOR_GREEN;
        _lcd.fillCircle(dx, dy, 3, dotColor);
        _prevDotX = dx;
        _prevDotY = dy;

        // Numeric G summary at bottom
        _lcd.fillRect(0, 118, 130, 12, COLOR_BLACK);
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(4, 120);
        _lcd.printf("LON:%+.2fG", dyn.gLongitudinal);
        _lcd.setCursor(72, 120);
        _lcd.printf("LAT:%+.2fG", dyn.gLateral);
    }

    // --------------------------------------------------------------------------
    // SCREEN 3: SESSION STATS & PEAKS
    // --------------------------------------------------------------------------
    void drawStatsStaticLayout() {
        _lcd.setTextSize(1);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(20, 4);
        _lcd.print("SESSION STATS");
        _lcd.drawFastHLine(0, 14, 130, COLOR_DARK_GRAY);

        _lcd.setTextColor(COLOR_MID_GRAY);
        _lcd.setCursor(6, 22);
        _lcd.print("MAX LEAN L:");
        _lcd.setCursor(6, 40);
        _lcd.print("MAX LEAN R:");
        _lcd.setCursor(6, 58);
        _lcd.print("MAX BRAKE :");
        _lcd.setCursor(6, 76);
        _lcd.print("MAX ACCEL :");
        _lcd.setCursor(6, 94);
        _lcd.print("ELEVATION :");
        _lcd.setCursor(6, 112);
        _lcd.print("RUN TIME  :");
    }

    void renderStatsDynamic(const MotorcycleDynamics& dyn) {
        _lcd.setTextSize(1);

        // Max Lean L
        _lcd.fillRect(80, 22, 48, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_CYAN);
        _lcd.setCursor(80, 22);
        _lcd.printf("%.1f deg", dyn.maxLeanLeft);

        // Max Lean R
        _lcd.fillRect(80, 40, 48, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_AMBER);
        _lcd.setCursor(80, 40);
        _lcd.printf("%.1f deg", dyn.maxLeanRight);

        // Max Braking G
        _lcd.fillRect(80, 58, 48, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_RED);
        _lcd.setCursor(80, 58);
        _lcd.printf("-%.2f G", dyn.maxBrakingG);

        // Max Accel G
        _lcd.fillRect(80, 76, 48, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_GREEN);
        _lcd.setCursor(80, 76);
        _lcd.printf("+%.2f G", dyn.maxAccelG);

        // Altitude Delta
        _lcd.fillRect(80, 94, 48, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(80, 94);
        int deltaElev = (dyn.maxAltitudeM > dyn.minAltitudeM) ? (int)(dyn.maxAltitudeM - dyn.minAltitudeM) : 0;
        _lcd.printf("+%dm", deltaElev);

        // Run Time
        uint32_t sec = (millis() - dyn.sessionStartTime) / 1000;
        _lcd.fillRect(80, 112, 48, 10, COLOR_BLACK);
        _lcd.setTextColor(COLOR_WHITE);
        _lcd.setCursor(80, 112);
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
