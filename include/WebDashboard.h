#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Config.h"
#include "MotorcycleFilter.h"

// ==============================================================================
// ON-DEMAND WI-FI ACCESS POINT & TELEMETRY DASHBOARD
// ==============================================================================
// Turned OFF during riding to eliminate RF noise, save ~90mA, and avoid CPU interrupts.
// Turned ON when parked to inspect telemetry on any smartphone browser at 192.168.4.1.
// ==============================================================================

static const char PROGMEM DASHBOARD_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Yamaha FZ8 Telemetria</title>
<style>
  :root {
    --bg: #0d1117;
    --card: #161b22;
    --border: #30363d;
    --text: #c9d1d9;
    --accent: #58a6ff;
    --green: #3fb950;
    --orange: #d29922;
    --red: #f85149;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 16px; }
  .header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border); padding-bottom: 12px; margin-bottom: 16px; }
  h1 { font-size: 20px; color: #fff; display: flex; align-items: center; gap: 8px; }
  .badge { background: #238636; color: #fff; font-size: 11px; padding: 2px 8px; border-radius: 12px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; margin-bottom: 16px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 14px; text-align: center; }
  .card.wide { grid-column: span 2; }
  .label { font-size: 12px; color: #8b949e; text-transform: uppercase; margin-bottom: 4px; }
  .val { font-size: 28px; font-weight: bold; color: #fff; }
  .val.left { color: var(--accent); }
  .val.right { color: var(--orange); }
  .val.brake { color: var(--red); }
  .val.accel { color: var(--green); }
  .actions { display: flex; gap: 10px; }
  button { flex: 1; padding: 12px; background: #21262d; color: #fff; border: 1px solid var(--border); border-radius: 6px; font-weight: bold; cursor: pointer; }
  button:hover { background: #30363d; }
  button.danger { border-color: var(--red); color: var(--red); }
</style>
</head>
<body>
<div class="header">
  <h1>Yamaha FZ8 <span class="badge">LIVE</span></h1>
  <span id="clock">00:00</span>
</div>

<div class="grid">
  <div class="card wide">
    <div class="label">Angolo di Piega Istantaneo</div>
    <div id="liveRoll" class="val" style="font-size: 42px;">0.0&deg;</div>
    <div id="leanDir" style="color: #8b949e; font-size: 14px; margin-top: 4px;">DRITTA</div>
    <div id="tareInfo" style="color: #8b949e; font-size: 11px; margin-top: 6px;">Tara Montaggio: R: +0.0&deg; | P: +0.0&deg;</div>
  </div>

  <div class="card">
    <div class="label">Piega Max Sinistra</div>
    <div id="maxLeft" class="val left">0&deg;</div>
  </div>

  <div class="card">
    <div class="label">Piega Max Destra</div>
    <div id="maxRight" class="val right">0&deg;</div>
  </div>

  <div class="card">
    <div class="label">Staccata Max (Frenata)</div>
    <div id="maxBrake" class="val brake">0.00 G</div>
  </div>

  <div class="card">
    <div class="label">Accelerazione Max</div>
    <div id="maxAccel" class="val accel">0.00 G</div>
  </div>

  <div class="card">
    <div class="label">Altitudine</div>
    <div id="alt" class="val">0 m</div>
  </div>

  <div class="card">
    <div class="label">Temperatura</div>
    <div id="temp" class="val">0 &deg;C</div>
  </div>
</div>

<div class="actions" style="margin-bottom: 8px;">
  <button onclick="sendAction('/api/tare')" style="background: #238636; border-color: #2ea043;">🎯 Salva Posizione come Zero (Tara)</button>
</div>
<div class="actions">
  <button onclick="sendAction('/api/resettare')">↺ Reset Zero Fabbrica</button>
  <button class="danger" onclick="sendAction('/api/reset')">Azzera Record</button>
</div>

<script>
function fetchTelemetry() {
  fetch('/api/telemetry')
    .then(r => r.json())
    .then(d => {
      let roll = d.roll.toFixed(1);
      document.getElementById('liveRoll').innerHTML = Math.abs(roll) + '&deg;';
      if (d.roll < -1.5) {
        document.getElementById('leanDir').innerHTML = 'PIEGA A SINISTRA';
        document.getElementById('liveRoll').style.color = 'var(--accent)';
      } else if (d.roll > 1.5) {
        document.getElementById('leanDir').innerHTML = 'PIEGA A DESTRA';
        document.getElementById('liveRoll').style.color = 'var(--orange)';
      } else {
        document.getElementById('leanDir').innerHTML = 'DRITTA';
        document.getElementById('liveRoll').style.color = '#fff';
      }

      if (d.tR !== undefined) {
        document.getElementById('tareInfo').innerHTML = 'Tara Montaggio: R: ' + (d.tR >= 0 ? '+' : '') + d.tR.toFixed(1) + '&deg; | P: ' + (d.tP >= 0 ? '+' : '') + d.tP.toFixed(1) + '&deg;';
      }

      document.getElementById('maxLeft').innerHTML = d.maxL.toFixed(1) + '&deg;';
      document.getElementById('maxRight').innerHTML = d.maxR.toFixed(1) + '&deg;';
      document.getElementById('maxBrake').innerHTML = '-' + d.maxBrake.toFixed(2) + ' G';
      document.getElementById('maxAccel').innerHTML = '+' + d.maxAccel.toFixed(2) + ' G';
      document.getElementById('alt').innerHTML = Math.round(d.alt) + ' m';
      document.getElementById('temp').innerHTML = Math.round(d.temp) + ' &deg;C';
    })
    .catch(e => console.log(e));
}

function sendAction(endpoint) {
  fetch(endpoint, { method: 'POST' }).then(() => fetchTelemetry());
}

setInterval(fetchTelemetry, 250);
fetchTelemetry();
</script>
</body>
</html>
)rawliteral";

class WebDashboard {
public:
    WebDashboard() : _server(80), _active(false) {}

    bool isRunning() const { return _active; }

    void start(MotorcycleFilter* filter, const MotorcycleDynamics* dynPtr) {
        _filter = filter;
        _dynPtr = dynPtr;

        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        delay(100);

        _server.on("/", HTTP_GET, [this]() {
            _server.send_P(200, "text/html", DASHBOARD_HTML);
        });

        _server.on("/api/telemetry", HTTP_GET, [this]() {
            if (!_dynPtr) {
                _server.send(500, "application/json", "{}");
                return;
            }
            char json[280];
            snprintf(json, sizeof(json), 
                "{\"roll\":%.1f,\"pitch\":%.1f,\"maxL\":%.1f,\"maxR\":%.1f,\"maxBrake\":%.2f,\"maxAccel\":%.2f,\"gLon\":%.2f,\"gLat\":%.2f,\"alt\":%.1f,\"temp\":%.1f,\"tR\":%.1f,\"tP\":%.1f}",
                _dynPtr->rollDeg, _dynPtr->pitchDeg,
                _dynPtr->maxLeanLeft, _dynPtr->maxLeanRight,
                _dynPtr->maxBrakingG, _dynPtr->maxAccelG,
                _dynPtr->gLongitudinal, _dynPtr->gLateral,
                _dynPtr->altitudeM, _dynPtr->tempC,
                _filter ? _filter->getTareRoll() : 0.0f,
                _filter ? _filter->getTarePitch() : 0.0f);
            _server.send(200, "application/json", json);
        });

        _server.on("/api/tare", HTTP_POST, [this]() {
            if (_filter) _filter->tareZero();
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/resettare", HTTP_POST, [this]() {
            if (_filter) _filter->resetTare();
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/reset", HTTP_POST, [this]() {
            if (_filter) _filter->resetRecords();
            _server.send(200, "text/plain", "OK");
        });

        _server.begin();
        _active = true;
        Serial.println("[WIFI] AP started: FZ8-Telemetry (192.168.4.1)");
    }

    void stop() {
        if (!_active) return;
        _server.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        _active = false;
        Serial.println("[WIFI] AP stopped (Power saving / Riding mode active)");
    }

    void handleClient() {
        if (_active) {
            _server.handleClient();
        }
    }

private:
    WebServer _server;
    bool _active;
    MotorcycleFilter* _filter;
    const MotorcycleDynamics* _dynPtr;
};

#endif // WEB_DASHBOARD_H
