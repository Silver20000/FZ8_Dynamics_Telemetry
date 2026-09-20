#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "Config.h"
#include "MotorcycleFilter.h"
#include "DisplayUI.h"

// ==============================================================================
// DUAL-MODE WI-FI (HOTSPOT + RETE DI CASA) & TELEMETRY CONTROLLER
// ==============================================================================
// 1. Hotspot AP sempre attivo: SSID "FZ8-Telemetry" (192.168.4.1)
// 2. Client STA connesso al Wi-Fi di casa: raggiungibile anche da "http://fz8.local"
// 3. Gestione totale da smartphone: Tara Zero, Micro-regolazione, Cambio schermate LCD
// ==============================================================================

static const char PROGMEM DASHBOARD_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
<title>Yamaha FZ8 Telemetria</title>
<style>
  :root {
    --bg: #0b0e14;
    --card: #151922;
    --border: #262d3d;
    --text: #c9d1d9;
    --accent: #58a6ff;
    --green: #2ea043;
    --orange: #d29922;
    --red: #f85149;
    --btn-bg: #21262d;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
  body { background: var(--bg); color: var(--text); padding: 14px; max-width: 480px; margin: 0 auto; }
  .header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid var(--border); padding-bottom: 10px; margin-bottom: 14px; }
  h1 { font-size: 19px; color: #fff; display: flex; align-items: center; gap: 6px; }
  .badge { background: var(--green); color: #fff; font-size: 11px; padding: 3px 8px; border-radius: 12px; font-weight: bold; }
  .net-status { font-size: 11px; color: #8b949e; margin-top: 2px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 14px; }
  .card { background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 12px; text-align: center; }
  .card.wide { grid-column: span 2; }
  .label { font-size: 11px; color: #8b949e; text-transform: uppercase; margin-bottom: 4px; letter-spacing: 0.5px; }
  .val { font-size: 26px; font-weight: bold; color: #fff; }
  .val.left { color: var(--accent); }
  .val.right { color: var(--orange); }
  .val.brake { color: var(--red); }
  .val.accel { color: var(--green); }
  
  .section-title { font-size: 13px; font-weight: bold; color: #fff; text-transform: uppercase; margin: 16px 0 8px 2px; display: flex; align-items: center; gap: 6px; }
  .actions { display: flex; gap: 8px; margin-bottom: 8px; }
  button { flex: 1; padding: 11px 8px; background: var(--btn-bg); color: #fff; border: 1px solid var(--border); border-radius: 6px; font-weight: 600; font-size: 13px; cursor: pointer; transition: background 0.15s; }
  button:active { background: #30363d; transform: scale(0.98); }
  button.primary { background: #238636; border-color: #2ea043; color: #fff; font-size: 14px; padding: 13px; }
  button.danger { border-color: var(--red); color: var(--red); }
  button.active-screen { background: #1f6feb; border-color: #388bfd; }
  
  .tune-row { display: flex; align-items: center; justify-content: space-between; background: var(--card); border: 1px solid var(--border); border-radius: 6px; padding: 8px 12px; margin-bottom: 6px; }
  .tune-label { font-size: 12px; font-weight: bold; color: #8b949e; }
  .tune-val { font-size: 14px; font-weight: bold; color: #fff; }
  .tune-btn-group { display: flex; gap: 6px; }
  .tune-btn-group button { padding: 6px 12px; font-size: 12px; flex: initial; }

  .input-group { display: flex; flex-direction: column; gap: 6px; margin-bottom: 8px; }
  input[type="text"], input[type="password"] { width: 100%; padding: 10px; background: #0d1117; border: 1px solid var(--border); border-radius: 6px; color: #fff; font-size: 13px; }
  
  #toast { display: none; background: #238636; color: #fff; padding: 10px; border-radius: 6px; font-size: 13px; font-weight: bold; text-align: center; margin-bottom: 12px; }
</style>
</head>
<body>
<div class="header">
  <div>
    <h1>Yamaha FZ8 <span class="badge">ONLINE</span></h1>
    <div id="netInfo" class="net-status">Hotspot: 192.168.4.1</div>
  </div>
  <div id="clock" style="font-weight: bold; color: #8b949e; font-size: 13px;">00:00</div>
</div>

<div id="toast"></div>

<!-- 1. LIVE TELEMETRY -->
<div class="grid">
  <div class="card wide">
    <div class="label">Angolo di Piega Istantaneo</div>
    <div id="liveRoll" class="val" style="font-size: 44px; margin: 4px 0;">0.0&deg;</div>
    <div id="leanDir" style="color: #8b949e; font-size: 13px; font-weight: bold;">DRITTA</div>
    <div style="display: flex; justify-content: space-around; margin-top: 8px; padding-top: 8px; border-top: 1px solid var(--border); font-size: 11px; color: #8b949e;">
      <span>G-Long: <b id="liveGLon" style="color:#fff;">0.00 G</b></span>
      <span>Beccheggio: <b id="livePitch" style="color:#fff;">0.0&deg;</b></span>
    </div>
  </div>

  <div class="card">
    <div class="label">Piega Max SX</div>
    <div id="maxLeft" class="val left">0&deg;</div>
  </div>

  <div class="card">
    <div class="label">Piega Max DX</div>
    <div id="maxRight" class="val right">0&deg;</div>
  </div>

  <div class="card">
    <div class="label">Staccata Max</div>
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

<!-- 2. PUNTO ZERO & CALIBRAZIONE TARA -->
<div class="section-title">&#127919; Punto Zero (Calibrazione Montaggio)</div>
<div style="background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 12px; margin-bottom: 14px;">
  <p style="font-size: 12px; color: #8b949e; margin-bottom: 10px;">
    Metti la moto dritta nella posizione desiderata e premi il pulsante per salvare il punto 0 in memoria Flash:
  </p>
  <button class="primary" onclick="tareZero()" style="width: 100%; margin-bottom: 10px;">
    &#127919; IMPOSTA POSIZIONE ATTUALE COME ZERO
  </button>
  
  <div class="tune-row">
    <div>
      <div class="tune-label">OFFSET ROLL (PIEGA)</div>
      <div id="tareRollVal" class="tune-val">+0.0&deg;</div>
    </div>
    <div class="tune-btn-group">
      <button onclick="adjustTare(-0.5, 0)">-0.5&deg;</button>
      <button onclick="adjustTare(+0.5, 0)">+0.5&deg;</button>
    </div>
  </div>

  <div class="tune-row">
    <div>
      <div class="tune-label">OFFSET PITCH (BECCHEGGIO)</div>
      <div id="tarePitchVal" class="tune-val">+0.0&deg;</div>
    </div>
    <div class="tune-btn-group">
      <button onclick="adjustTare(0, -0.5)">-0.5&deg;</button>
      <button onclick="adjustTare(0, +0.5)">+0.5&deg;</button>
    </div>
  </div>

  <div style="display: flex; gap: 8px; margin-top: 10px;">
    <button onclick="resetTare()">&#8634; Reset Zero Fabbrica (0.0&deg;)</button>
    <button class="danger" onclick="resetStats()">&#128465; Azzera Record</button>
  </div>
</div>

<!-- 3. CONTROLLO DISPLAY LCD (SENZA TASTO FISICO) -->
<div class="section-title">&#128241; Controllo Schermata Display LCD</div>
<div style="background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 12px; margin-bottom: 14px;">
  <p style="font-size: 12px; color: #8b949e; margin-bottom: 10px;">
    Seleziona la schermata attiva sul display da 1.6" o attiva l'auto-rotazione:
  </p>
  <div class="actions">
    <button id="btnScr0" onclick="setScreen(0)">&#128202; Inclinometro</button>
    <button id="btnScr1" onclick="setScreen(1)">&#9898; Cerchio G-G</button>
    <button id="btnScr2" onclick="setScreen(2)">&#9201; Statistiche</button>
  </div>
  <button id="btnAutoCycle" onclick="toggleAutoCycle()" style="width: 100%; margin-top: 4px;">
    &#128257; Auto-Rotazione Schermate (ogni 6s): OFF
  </button>
</div>

<!-- 4. WI-FI DI CASA (MODALITA DOPPIA) -->
<div class="section-title">&#128246; Connessione Wi-Fi di Casa</div>
<div style="background: var(--card); border: 1px solid var(--border); border-radius: 8px; padding: 12px; margin-bottom: 14px;">
  <p style="font-size: 12px; color: #8b949e; margin-bottom: 10px;">
    Collega l'ESP32 al router di casa per raggiungerlo comodamente su <b>http://fz8.local</b>:
  </p>
  <div class="input-group">
    <input type="text" id="wifiSsid" placeholder="Nome Rete Wi-Fi di casa (SSID)">
    <input type="password" id="wifiPass" placeholder="Password Wi-Fi">
  </div>
  <button onclick="saveHomeWifi()" style="width: 100%;">
    &#128190; Salva e Connetti al Wi-Fi di Casa
  </button>
</div>

<script>
let autoCycleActive = false;

function showToast(msg) {
  let t = document.getElementById('toast');
  t.innerText = msg;
  t.style.display = 'block';
  setTimeout(() => { t.style.display = 'none'; }, 3000);
}

function fetchTelemetry() {
  fetch('/api/telemetry')
    .then(r => r.json())
    .then(d => {
      let roll = d.roll.toFixed(1);
      document.getElementById('liveRoll').innerHTML = Math.abs(roll) + '&deg;';
      let rollEl = document.getElementById('liveRoll');
      let dirEl = document.getElementById('leanDir');
      
      if (d.roll < -1.5) {
        dirEl.innerHTML = 'PIEGA A SINISTRA (' + Math.abs(roll) + '&deg;)';
        rollEl.style.color = 'var(--accent)';
      } else if (d.roll > 1.5) {
        dirEl.innerHTML = 'PIEGA A DESTRA (' + Math.abs(roll) + '&deg;)';
        rollEl.style.color = 'var(--orange)';
      } else {
        dirEl.innerHTML = 'DRITTA (0.0&deg;)';
        rollEl.style.color = '#fff';
      }

      document.getElementById('liveGLon').innerHTML = (d.gLon >= 0 ? '+' : '') + d.gLon.toFixed(2) + ' G';
      document.getElementById('livePitch').innerHTML = (d.pitch >= 0 ? '+' : '') + d.pitch.toFixed(1) + '&deg;';

      document.getElementById('maxLeft').innerHTML = d.maxL.toFixed(1) + '&deg;';
      document.getElementById('maxRight').innerHTML = d.maxR.toFixed(1) + '&deg;';
      document.getElementById('maxBrake').innerHTML = '-' + d.maxBrake.toFixed(2) + ' G';
      document.getElementById('maxAccel').innerHTML = '+' + d.maxAccel.toFixed(2) + ' G';
      document.getElementById('alt').innerHTML = Math.round(d.alt) + ' m';
      document.getElementById('temp').innerHTML = Math.round(d.temp) + ' &deg;C';

      if (d.tR !== undefined) {
        document.getElementById('tareRollVal').innerText = (d.tR >= 0 ? '+' : '') + d.tR.toFixed(1) + '°';
        document.getElementById('tarePitchVal').innerText = (d.tP >= 0 ? '+' : '') + d.tP.toFixed(1) + '°';
      }

      // Net Info
      let netStr = 'Hotspot: ' + d.apIp;
      if (d.staConnected) {
        netStr += ' | Casa: ' + d.staIp + ' (fz8.local)';
      } else if (d.staSsid && d.staSsid.length > 0) {
        netStr += ' | Connessione a ' + d.staSsid + '...';
      }
      document.getElementById('netInfo').innerText = netStr;

      // Active screen button highlight
      [0, 1, 2].forEach(i => {
        let b = document.getElementById('btnScr' + i);
        if (b) {
          if (d.screen === i) b.classList.add('active-screen');
          else b.classList.remove('active-screen');
        }
      });

      autoCycleActive = d.autoCycle;
      document.getElementById('btnAutoCycle').innerText = 
        '🔄 Auto-Rotazione Schermate (ogni 6s): ' + (autoCycleActive ? 'ATTIVA (ON)' : 'DISATTIVA (OFF)');
    })
    .catch(e => console.log(e));
}

function tareZero() {
  fetch('/api/tare', { method: 'POST' })
    .then(r => r.text())
    .then(() => {
      showToast('Punto Zero calibrato e salvato in memoria Flash!');
      fetchTelemetry();
    });
}

function resetTare() {
  fetch('/api/resettare', { method: 'POST' })
    .then(() => {
      showToast('Tara reimpostata al valore di fabbrica (0.0°)');
      fetchTelemetry();
    });
}

function adjustTare(dr, dp) {
  fetch('/api/adjusttare?dr=' + dr + '&dp=' + dp, { method: 'POST' })
    .then(() => {
      showToast('Offset aggiornato!');
      fetchTelemetry();
    });
}

function setScreen(mode) {
  fetch('/api/screen?mode=' + mode, { method: 'POST' })
    .then(() => fetchTelemetry());
}

function toggleAutoCycle() {
  let newState = autoCycleActive ? 0 : 1;
  fetch('/api/autocycle?enable=' + newState, { method: 'POST' })
    .then(() => {
      showToast('Auto-rotazione: ' + (newState ? 'ATTIVATA' : 'DISATTIVATA'));
      fetchTelemetry();
    });
}

function resetStats() {
  if (confirm('Azzerare tutti i record di sessione?')) {
    fetch('/api/reset', { method: 'POST' }).then(() => fetchTelemetry());
  }
}

function saveHomeWifi() {
  let ssid = document.getElementById('wifiSsid').value.trim();
  let pass = document.getElementById('wifiPass').value;
  if (!ssid) {
    alert('Inserisci il nome della rete Wi-Fi!');
    return;
  }
  fetch('/api/wifi?ssid=' + encodeURIComponent(ssid) + '&pass=' + encodeURIComponent(pass), { method: 'POST' })
    .then(() => {
      showToast('Credenziali Wi-Fi salvate! Connessione in corso...');
      fetchTelemetry();
    });
}

setInterval(fetchTelemetry, 250);
fetchTelemetry();
</script>
</body>
</html>
)rawliteral";

class WebDashboard {
public:
    WebDashboard() : 
        _server(80), 
        _active(false), 
        _filter(nullptr), 
        _dynPtr(nullptr), 
        _uiPtr(nullptr) {}

    bool isRunning() const { return _active; }

    void start(MotorcycleFilter* filter, const MotorcycleDynamics* dynPtr, DisplayUI* uiPtr = nullptr) {
        _filter = filter;
        _dynPtr = dynPtr;
        _uiPtr = uiPtr;

        // 1. Leggi credenziali Wi-Fi di casa da Flash NVS
        _prefs.begin("fz8_net", false);
        _homeSsid = _prefs.getString("homeSsid", DEFAULT_HOME_SSID);
        _homePass = _prefs.getString("homePass", DEFAULT_HOME_PASS);
        _prefs.end();

        // 2. Avvia Wi-Fi in Modalita Doppia (AP Aperto + STA)
        WiFi.mode(WIFI_AP_STA);
        if (strlen(AP_PASSWORD) == 0) {
            WiFi.softAP(AP_SSID); // Rete APERTA senza alcuna password!
            Serial.printf("[WIFI] SoftAP APERTO Avviato: SSID='%s' (Nessuna Password), IP=%s\n", 
                          AP_SSID, WiFi.softAPIP().toString().c_str());
        } else {
            WiFi.softAP(AP_SSID, AP_PASSWORD);
            Serial.printf("[WIFI] SoftAP Avviato: SSID='%s', Pass='%s', IP=%s\n", 
                          AP_SSID, AP_PASSWORD, WiFi.softAPIP().toString().c_str());
        }
        delay(100);

        // Aggiungi le reti note a WiFiMulti (Silvestrini 2.4g e Silver)
        _wifiMulti.addAP("Silvestrini 2.4g", "11042025");
        _wifiMulti.addAP("Silvestrini 2.4g", "silver11");
        _wifiMulti.addAP("Silvestrini 2.4g", "Silver11");
        _wifiMulti.addAP("Silver", "silver11");
        _wifiMulti.addAP("Silver", "Silver11");
        if (_homeSsid.length() > 0 && _homeSsid != "Silvestrini 2.4g" && _homeSsid != "Silver") {
            _wifiMulti.addAP(_homeSsid.c_str(), _homePass.c_str());
        }
        _wifiMulti.run();

        // 3. Avvia mDNS responder (http://fz8.local)
        if (MDNS.begin(MDNS_HOSTNAME)) {
            MDNS.addService("http", "tcp", 80);
            Serial.printf("[WIFI] mDNS responder attivo: http://%s.local\n", MDNS_HOSTNAME);
        }

        // 4. Registra endpoint HTTP
        _server.on("/", HTTP_GET, [this]() {
            _server.send_P(200, "text/html", DASHBOARD_HTML);
        });

        _server.on("/api/telemetry", HTTP_GET, [this]() {
            if (!_dynPtr) {
                _server.send(500, "application/json", "{}");
                return;
            }

            bool staConnected = (WiFi.status() == WL_CONNECTED);
            String staIpStr = staConnected ? WiFi.localIP().toString() : "";
            String apIpStr = WiFi.softAPIP().toString();
            int curScreen = _uiPtr ? (int)_uiPtr->getScreen() : 0;
            bool autoCycle = _uiPtr ? _uiPtr->isAutoCycle() : false;

            char json[512];
            snprintf(json, sizeof(json), 
                "{"
                "\"roll\":%.1f,\"pitch\":%.1f,"
                "\"rawR\":%.1f,\"rawP\":%.1f,"
                "\"maxL\":%.1f,\"maxR\":%.1f,"
                "\"maxBrake\":%.2f,\"maxAccel\":%.2f,"
                "\"gLon\":%.2f,\"gLat\":%.2f,"
                "\"alt\":%.1f,\"temp\":%.1f,"
                "\"tR\":%.1f,\"tP\":%.1f,"
                "\"screen\":%d,\"autoCycle\":%s,"
                "\"staConnected\":%s,\"staIp\":\"%s\",\"staSsid\":\"%s\","
                "\"apIp\":\"%s\""
                "}",
                _dynPtr->rollDeg, _dynPtr->pitchDeg,
                _filter ? _filter->getRawRoll() : 0.0f,
                _filter ? _filter->getRawPitch() : 0.0f,
                _dynPtr->maxLeanLeft, _dynPtr->maxLeanRight,
                _dynPtr->maxBrakingG, _dynPtr->maxAccelG,
                _dynPtr->gLongitudinal, _dynPtr->gLateral,
                _dynPtr->altitudeM, _dynPtr->tempC,
                _filter ? _filter->getTareRoll() : 0.0f,
                _filter ? _filter->getTarePitch() : 0.0f,
                curScreen, autoCycle ? "true" : "false",
                staConnected ? "true" : "false",
                staIpStr.c_str(), _homeSsid.c_str(),
                apIpStr.c_str());
            _server.send(200, "application/json", json);
        });

        _server.on("/api/tare", HTTP_POST, [this]() {
            if (_filter) {
                _filter->tareZero();
                if (_uiPtr) {
                    _uiPtr->showTareNotice(_filter->getTareRoll(), _filter->getTarePitch());
                }
            }
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/resettare", HTTP_POST, [this]() {
            if (_filter) {
                _filter->resetTare();
                if (_uiPtr) {
                    _uiPtr->showTareResetNotice();
                }
            }
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/adjusttare", HTTP_POST, [this]() {
            float dr = 0.0f, dp = 0.0f;
            if (_server.hasArg("dr")) dr = _server.arg("dr").toFloat();
            if (_server.hasArg("dp")) dp = _server.arg("dp").toFloat();
            if (_filter) {
                _filter->adjustTare(dr, dp);
                if (_uiPtr) {
                    _uiPtr->showTareNotice(_filter->getTareRoll(), _filter->getTarePitch());
                }
            }
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/screen", HTTP_POST, [this]() {
            if (_server.hasArg("mode") && _uiPtr) {
                int mode = _server.arg("mode").toInt();
                _uiPtr->setScreen((UIScreenMode)mode);
            }
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/autocycle", HTTP_POST, [this]() {
            if (_server.hasArg("enable") && _uiPtr) {
                int en = _server.arg("enable").toInt();
                _uiPtr->setAutoCycle(en == 1);
            }
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/wifi", HTTP_POST, [this]() {
            if (_server.hasArg("ssid")) {
                _homeSsid = _server.arg("ssid");
                _homePass = _server.hasArg("pass") ? _server.arg("pass") : "";
                
                // Salva in Flash
                _prefs.begin("fz8_net", false);
                _prefs.putString("homeSsid", _homeSsid);
                _prefs.putString("homePass", _homePass);
                _prefs.end();

                Serial.printf("[WIFI] Nuove credenziali salvate: '%s'. Connessione...\n", _homeSsid.c_str());
                WiFi.begin(_homeSsid.c_str(), _homePass.c_str());
            }
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/reset", HTTP_POST, [this]() {
            if (_filter) _filter->resetRecords();
            _server.send(200, "text/plain", "OK");
        });

        _server.begin();
        _active = true;
        Serial.println("[WIFI] Server Web avviato.");
    }

    void stop() {
        if (!_active) return;
        _server.stop();
        WiFi.softAPdisconnect(true);
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        _active = false;
        Serial.println("[WIFI] Wi-Fi disattivato.");
    }

    void handleClient() {
        if (_active) {
            _wifiMulti.run();
            _server.handleClient();
        }
    }

private:
    WebServer _server;
    WiFiMulti _wifiMulti;
    Preferences _prefs;
    bool _active;
    String _homeSsid;
    String _homePass;
    MotorcycleFilter* _filter;
    const MotorcycleDynamics* _dynPtr;
    DisplayUI* _uiPtr;
};

#endif // WEB_DASHBOARD_H
