#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include "Config.h"
#include "MotorcycleFilter.h"
#include "DisplayUI.h"
#include "SessionLogger.h"
#include "BluetoothTelemetry.h"

// ==============================================================================
// SMARTPHONE-NATIVE DEDICATED MOTORCYCLE TELEMETRY APP
// Pure SoftAP (192.168.4.1), Captive Portal DNS, Zero-Latency 10Hz Polling
// ==============================================================================

static const char PROGMEM DASHBOARD_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="it">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no, viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-status-bar-style" content="black-translucent">
<meta name="theme-color" content="#07090e">
<title>Yamaha FZ8 Telemetry</title>
<style>
  :root {
    --bg: #07090e;
    --card: #10141f;
    --card-elevated: #161c2c;
    --border: #222b3e;
    --text: #e6edf3;
    --muted: #8b949e;
    --cyan: #00f2fe;
    --green: #00e676;
    --orange: #ffab00;
    --red: #ff1744;
    --magenta: #d500f9;
    --nav-h: 64px;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    padding-bottom: calc(var(--nav-h) + 20px);
    user-select: none;
    -webkit-user-select: none;
    overflow-x: hidden;
  }
  
  /* Top App Bar */
  .app-header {
    position: sticky;
    top: 0;
    z-index: 100;
    background: rgba(7, 9, 14, 0.92);
    backdrop-filter: blur(12px);
    -webkit-backdrop-filter: blur(12px);
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 12px 16px;
    border-bottom: 1px solid var(--border);
  }
  .app-brand { display: flex; align-items: center; gap: 8px; }
  .app-title { font-size: 16px; font-weight: 800; letter-spacing: 0.5px; color: #fff; }
  .app-model { font-size: 11px; font-weight: 700; color: var(--cyan); background: rgba(0, 242, 254, 0.12); padding: 2px 6px; border-radius: 4px; }
  
  .status-pill {
    display: flex;
    align-items: center;
    gap: 6px;
    font-size: 11px;
    font-weight: 700;
    padding: 4px 10px;
    border-radius: 20px;
    background: rgba(0, 230, 118, 0.15);
    color: var(--green);
    border: 1px solid rgba(0, 230, 118, 0.3);
  }
  .status-pill.offline {
    background: rgba(255, 23, 68, 0.15);
    color: var(--red);
    border-color: rgba(255, 23, 68, 0.3);
  }
  .pulse-dot {
    width: 7px;
    height: 7px;
    border-radius: 50%;
    background: currentColor;
    animation: blink 1.2s infinite;
  }
  @keyframes blink { 0%, 100% { opacity: 1; } 50% { opacity: 0.3; } }

  /* App Views */
  .view-container { display: none; padding: 12px 14px; max-width: 520px; margin: 0 auto; }
  .view-container.active { display: block; }

  /* Toast & Crash Banner */
  #toast {
    position: fixed;
    top: 60px;
    left: 50%;
    transform: translateX(-50%);
    z-index: 1000;
    background: var(--card-elevated);
    color: #fff;
    border: 1px solid var(--border);
    box-shadow: 0 8px 24px rgba(0,0,0,0.6);
    padding: 10px 18px;
    border-radius: 24px;
    font-size: 13px;
    font-weight: 700;
    display: none;
    text-align: center;
    white-space: nowrap;
  }
  #crashBanner {
    display: none;
    background: var(--red);
    color: #fff;
    padding: 14px;
    border-radius: 12px;
    font-weight: 900;
    font-size: 14px;
    text-align: center;
    margin-bottom: 12px;
    animation: alertPulse 0.8s infinite alternate;
  }
  @keyframes alertPulse { from { opacity: 1; } to { opacity: 0.5; } }

  /* MotoGP Lean Angle Gauge */
  .gauge-card {
    background: linear-gradient(180deg, #131826 0%, #0c101a 100%);
    border: 1px solid var(--border);
    border-radius: 18px;
    padding: 16px 12px;
    text-align: center;
    position: relative;
    overflow: hidden;
    margin-bottom: 12px;
  }
  .gauge-title { font-size: 11px; font-weight: 700; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; }
  .lean-display-wrap { position: relative; margin: 10px auto; width: 220px; height: 125px; }
  .lean-value {
    position: absolute;
    bottom: 4px;
    left: 0;
    right: 0;
    font-size: 52px;
    font-weight: 900;
    line-height: 1;
    color: #fff;
    font-feature-settings: "tnum";
  }
  .lean-dir-tag {
    font-size: 13px;
    font-weight: 800;
    letter-spacing: 0.5px;
    color: var(--muted);
    margin-top: 4px;
  }
  .g-force-bar {
    display: flex;
    justify-content: space-around;
    padding-top: 12px;
    margin-top: 12px;
    border-top: 1px solid rgba(255,255,255,0.06);
    font-size: 12px;
  }
  .g-sub { font-size: 10px; color: var(--muted); text-transform: uppercase; }
  .g-num { font-size: 16px; font-weight: 800; color: #fff; margin-top: 2px; }

  /* 4-Tile Grid */
  .grid-2x2 { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 12px; }
  .tile {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 14px;
    padding: 12px;
  }
  .tile-label { font-size: 10px; font-weight: 700; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px; display: flex; align-items: center; gap: 4px; }
  .tile-val { font-size: 20px; font-weight: 800; color: #fff; margin-top: 4px; }
  .tile-sub { font-size: 10px; font-weight: 700; color: var(--muted); margin-top: 2px; }

  /* Peak Records Ribbon */
  .records-ribbon {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 14px;
    padding: 10px 14px;
    display: flex;
    justify-content: space-between;
    margin-bottom: 12px;
    font-size: 11px;
    text-align: center;
  }
  .rec-item b { display: block; font-size: 15px; font-weight: 800; margin-top: 2px; }

  /* Action Buttons & Cards */
  .section-label { font-size: 11px; font-weight: 800; color: var(--muted); text-transform: uppercase; letter-spacing: 1px; margin: 16px 0 8px 4px; }
  .btn-big {
    width: 100%;
    padding: 16px;
    border-radius: 14px;
    border: none;
    font-size: 15px;
    font-weight: 800;
    cursor: pointer;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 8px;
    color: #fff;
    background: #1f6feb;
    box-shadow: 0 4px 14px rgba(31, 111, 235, 0.3);
    transition: transform 0.1s, opacity 0.1s;
  }
  .btn-big:active { transform: scale(0.97); opacity: 0.9; }
  .btn-big.primary { background: #238636; box-shadow: 0 4px 14px rgba(46, 160, 67, 0.3); }
  .btn-big.danger { background: #da3633; box-shadow: 0 4px 14px rgba(248, 81, 73, 0.3); }
  .btn-big.secondary { background: var(--card-elevated); border: 1px solid var(--border); color: #fff; box-shadow: none; }
  
  .btn-row { display: flex; gap: 8px; margin-top: 8px; }
  .btn-row button { flex: 1; padding: 13px 8px; border-radius: 12px; font-size: 13px; font-weight: 700; border: 1px solid var(--border); background: var(--card-elevated); color: #fff; cursor: pointer; }
  .btn-row button:active { transform: scale(0.97); }

  /* Setting Row */
  .setting-row {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 14px;
    padding: 14px;
    margin-bottom: 8px;
    display: flex;
    justify-content: space-between;
    align-items: center;
  }
  .setting-info { font-size: 13px; font-weight: 700; color: #fff; }
  .setting-sub { font-size: 10px; color: var(--muted); margin-top: 2px; }

  /* Screen Switcher Grid */
  .screen-btn {
    padding: 10px 4px;
    border-radius: 10px;
    border: 1px solid var(--border);
    background: var(--card-elevated);
    color: #fff;
    font-size: 11px;
    font-weight: 700;
    cursor: pointer;
    transition: background 0.15s, border-color 0.15s;
    text-align: center;
  }
  .screen-btn:active { transform: scale(0.96); }
  .screen-btn.active {
    background: #1f6feb !important;
    border-color: #58a6ff !important;
    box-shadow: 0 0 10px rgba(31, 111, 235, 0.5);
  }

  /* Corner Analyzer Item */
  .corner-card {
    background: var(--card);
    border: 1px solid var(--border);
    border-radius: 12px;
    padding: 12px 14px;
    margin-bottom: 6px;
    display: flex;
    justify-content: space-between;
    align-items: center;
    font-size: 13px;
  }
  .corner-dir-badge {
    padding: 3px 8px;
    border-radius: 6px;
    font-weight: 800;
    font-size: 12px;
  }
  .corner-dir-badge.left { background: rgba(0, 242, 254, 0.15); color: var(--cyan); }
  .corner-dir-badge.right { background: rgba(255, 171, 0, 0.15); color: var(--orange); }

  /* Bottom Navigation Bar */
  .nav-bar {
    position: fixed;
    bottom: 0;
    left: 0;
    right: 0;
    height: var(--nav-h);
    background: rgba(13, 17, 26, 0.94);
    backdrop-filter: blur(14px);
    -webkit-backdrop-filter: blur(14px);
    border-top: 1px solid var(--border);
    display: flex;
    justify-content: space-around;
    align-items: center;
    z-index: 200;
    padding-bottom: env(safe-area-inset-bottom, 0px);
  }
  .nav-item {
    flex: 1;
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: center;
    color: var(--muted);
    font-size: 10px;
    font-weight: 700;
    height: 100%;
    cursor: pointer;
    transition: color 0.15s;
  }
  .nav-item.active { color: var(--cyan); }
  .nav-icon { font-size: 20px; margin-bottom: 3px; }
</style>
</head>
<body>

<!-- Header -->
<div class="app-header">
  <div class="app-brand">
    <span style="font-size: 18px;">&#127933;</span>
    <div>
      <div class="app-title">Yamaha FZ8</div>
      <div style="font-size: 9px; color: var(--muted); margin-top: -2px;">DYNAMICS TELEMETRY</div>
    </div>
    <span class="app-model">10-DOF</span>
  </div>
  <div id="livePill" class="status-pill">
    <div class="pulse-dot"></div>
    <span id="liveText">LIVE 10Hz</span>
  </div>
</div>

<div id="toast"></div>

<!-- TAB 1: COCKPIT -->
<div id="viewCockpit" class="view-container active">
  <div id="crashBanner">&#9888; CRASH DETECTED / MOTO A TERRA!</div>

  <!-- Big Lean Gauge -->
  <div class="gauge-card">
    <div class="gauge-title">Inclinometro Istantaneo</div>
    <div class="lean-display-wrap">
      <svg viewBox="0 0 220 120" style="width: 100%; height: 100%; overflow: visible;">
        <!-- Gauge Track -->
        <path d="M 20 110 A 90 90 0 0 1 200 110" fill="none" stroke="#1d2436" stroke-width="12" stroke-linecap="round"/>
        <!-- Safe 0-32 deg zone -->
        <path d="M 68 38 A 90 90 0 0 1 152 38" fill="none" stroke="rgba(0, 230, 118, 0.3)" stroke-width="4"/>
        <!-- Dynamic Needle / Arc Indicator -->
        <line id="gaugeNeedle" x1="110" y1="110" x2="110" y2="24" stroke="var(--cyan)" stroke-width="4" stroke-linecap="round"/>
        <circle cx="110" cy="110" r="7" fill="#fff"/>
      </svg>
      <div id="leanVal" class="lean-value">0.0&deg;</div>
    </div>
    <div id="leanDirText" class="lean-dir-tag">VERTICALE</div>

    <div class="g-force-bar">
      <div>
        <div class="g-sub">Frenata / Accel</div>
        <div id="gLonText" class="g-num">0.00 G</div>
      </div>
      <div>
        <div class="g-sub">Centrifuga Lat</div>
        <div id="gLatText" class="g-num">0.00 G</div>
      </div>
      <div>
        <div class="g-sub">Roll Rate</div>
        <div id="rollRateText" class="g-num">0 &deg;/s</div>
      </div>
    </div>
  </div>

  <!-- 4-Tile Quick Grid -->
  <div class="grid-2x2">
    <div class="tile">
      <div class="tile-label">&#129517; Bussola GY-89</div>
      <div id="compassText" class="tile-val">0&deg; -</div>
      <div id="compassBadge" class="tile-sub" style="color: var(--green);">[VALIDA]</div>
    </div>
    <div class="tile">
      <div class="tile-label">&#9968; Dislivello D+</div>
      <div id="dPlusText" class="tile-val" style="color: var(--green);">+0 m</div>
      <div id="altText" class="tile-sub">Quota: 0m</div>
    </div>
    <div class="tile">
      <div class="tile-label">&#128392; Assetto Beccheggio</div>
      <div id="pitchText" class="tile-val">0.0&deg;</div>
      <div id="pitchSub" class="tile-sub">Forcella / Mono</div>
    </div>
    <div class="tile">
      <div class="tile-label">&#127777; Sensori &amp; Temp</div>
      <div id="tempText" class="tile-val">0&deg;C</div>
      <div id="sampleHzText" class="tile-sub">Loop: 100 Hz</div>
    </div>
  </div>

  <!-- Peak Records Ribbon -->
  <div class="records-ribbon">
    <div class="rec-item">
      <span style="color: var(--cyan); font-weight: 700;">MAX SX</span>
      <b id="recMaxL" style="color: var(--cyan);">0.0&deg;</b>
    </div>
    <div class="rec-item">
      <span style="color: var(--orange); font-weight: 700;">MAX DX</span>
      <b id="recMaxR" style="color: var(--orange);">0.0&deg;</b>
    </div>
    <div class="rec-item">
      <span style="color: var(--red); font-weight: 700;">STACCATA</span>
      <b id="recBrake" style="color: var(--red);">0.00 G</b>
    </div>
    <div class="rec-item">
      <span style="color: var(--green); font-weight: 700;">ACCEL</span>
      <b id="recAccel" style="color: var(--green);">0.00 G</b>
    </div>
  </div>

  <!-- LCD Display Quick Switcher Bar -->
  <div class="setting-row" style="padding: 10px 14px; margin-bottom: 12px; align-items: center;">
    <div style="font-size: 11px; font-weight: 800; color: var(--muted); text-transform: uppercase; letter-spacing: 0.5px;">&#128250; LCD MOTO</div>
    <div style="display: flex; align-items: center; gap: 8px;">
      <button onclick="prevLcdScreen()" style="width:34px; height:34px; border-radius:8px; border:1px solid var(--border); background:var(--card-elevated); color:#fff; font-weight:800; font-size:14px; cursor:pointer;">&#9664;</button>
      <span id="cockpitLcdName" style="font-size: 13px; font-weight: 800; color: var(--cyan); min-width: 100px; text-align: center;">Arc Gauge</span>
      <button onclick="nextLcdScreen()" style="width:34px; height:34px; border-radius:8px; border:1px solid var(--border); background:var(--card-elevated); color:#fff; font-weight:800; font-size:14px; cursor:pointer;">&#9654;</button>
    </div>
  </div>
</div>

<!-- TAB 2: GRAFICI -->
<div id="viewCharts" class="view-container">
  <div class="section-label">&#128200; Timeline Piega in Tempo Reale (30s)</div>
  <div style="background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 10px; margin-bottom: 14px;">
    <canvas id="rollTimelineCanvas" width="480" height="130" style="width:100%; height:130px; display:block; border-radius: 8px; background:#0a0d14;"></canvas>
    <div style="display:flex; justify-content:space-between; font-size:10px; color:var(--muted); margin-top:6px;">
      <span>30 sec fa</span>
      <span id="canvasCurWarn" style="color:var(--magenta); font-weight:700;">Soglia Limite: 48&deg;</span>
      <span>Adesso</span>
    </div>
  </div>

  <div class="section-label">&#9898; Cerchio di Trazione G-G (Scatter Dinamico)</div>
  <div style="background: var(--card); border: 1px solid var(--border); border-radius: 14px; padding: 16px; text-align:center;">
    <canvas id="ggCircleCanvas" width="220" height="220" style="background:#0a0d14; border-radius:50%; border:1px solid var(--border); display:inline-block;"></canvas>
    <div style="display:flex; justify-content:space-around; font-size:11px; color:var(--muted); margin-top:10px;">
      <span>Frenata &uarr;</span>
      <span>Centrifuga &harr;</span>
      <span>Accelerazione &darr;</span>
    </div>
  </div>
</div>

<!-- TAB 3: CURVE -->
<div id="viewCorners" class="view-container">
  <div class="section-label">&#9201; Curva Attiva</div>
  <div id="curCornerBox" class="setting-row" style="background: var(--card-elevated); margin-bottom: 14px;">
    <div>
      <div id="curCornerState" style="font-size: 15px; font-weight: 800; color: var(--cyan);">MOTO SUL DRITTO</div>
      <div id="curCornerSub" class="setting-sub">Pronto per la prossima staccata</div>
    </div>
    <span id="curCornerBadge" class="corner-dir-badge" style="background:#21262d; color:#8b949e;">ATTESA</span>
  </div>

  <div class="section-label">&#128337; Storico Ultime 5 Curve</div>
  <div id="cornerHistoryList">
    <div style="color:var(--muted); font-size:12px; text-align:center; padding:20px;">Nessuna curva recente registrata.</div>
  </div>
</div>

<!-- TAB 4: DATALOGGER -->
<div id="viewLogger" class="view-container">
  <div class="gauge-card" style="padding: 24px 16px;">
    <div style="font-size: 12px; font-weight: 700; color: var(--muted); margin-bottom: 12px;">REGISTRATORE FLASH LITTLEFS (20B/SAMPLE)</div>
    
    <button id="btnRecBig" onclick="toggleRecord()" style="width: 110px; height: 110px; border-radius: 50%; border: 4px solid var(--border); background: #161c2c; color: #fff; font-size: 15px; font-weight: 900; margin: 0 auto 16px auto; cursor: pointer; display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 4px; box-shadow: 0 6px 20px rgba(0,0,0,0.5);">
      <div id="recDot" style="width: 20px; height: 20px; border-radius: 50%; background: var(--red);"></div>
      <span id="recBtnLabel">REC</span>
    </button>

    <div id="recStatusText" style="font-size: 14px; font-weight: 800; color: var(--muted); margin-bottom: 6px;">PRONTO PER REGISTRARE</div>
    <div style="font-size: 12px; color: var(--muted);">
      Spazio occupato: <b id="logKbText" style="color: #fff;">0 KB</b> (<span id="logSamplesText">0 campioni</span>)
    </div>
  </div>

  <div class="btn-row">
    <button class="btn-big" onclick="downloadCsv()" style="background: #238636; border-color: #2ea043;">
      &#128229; Scarica File CSV
    </button>
    <button class="btn-big secondary" onclick="clearLogData()" style="flex: 0.5;">
      &#128465; Azzera
    </button>
  </div>

  <div style="margin-top: 14px; padding: 12px; background: rgba(0, 242, 254, 0.05); border: 1px solid rgba(0, 242, 254, 0.15); border-radius: 12px; font-size: 11px; color: var(--muted); line-height: 1.4;">
    &#128161; <b>Buffer RAM a blocchi attivo:</b> i dati vengono salvati a blocchi di 50 campioni ogni 5 secondi, proteggendo la memoria Flash SPI e consentendo oltre 75 minuti continui di sessione.
  </div>
</div>

<!-- TAB 5: SETUP -->
<div id="viewSetup" class="view-container">
  <div class="section-label">&#127919; Punto Zero (Calibrazione Montaggio)</div>
  <button class="btn-big primary" onclick="tareZero()" style="margin-bottom: 12px; height: 58px; font-size: 16px;">
    &#127919; IMPOSTA ZERO TARA
  </button>

  <div class="setting-row">
    <div>
      <div class="setting-info">Offset Piega Attuale</div>
      <div class="setting-sub">Correzione montaggio telaio</div>
    </div>
    <div id="tareRollText" style="font-size: 16px; font-weight: 800; color: var(--cyan);">+0.0&deg;</div>
  </div>

  <div class="btn-row" style="margin-bottom: 14px;">
    <button onclick="adjustTare(-0.5, 0)">-0.5&deg; SX</button>
    <button onclick="adjustTare(+0.5, 0)">+0.5&deg; DX</button>
    <button onclick="resetTareDefault()">Ripristina 0.0&deg;</button>
  </div>

  <div class="section-label">&#9888; Allarme Angolo di Piega</div>
  <div class="setting-row">
    <div>
      <div class="setting-info">Soglia Allarme Pedana</div>
      <div class="setting-sub">Lampeggio display oltre la soglia</div>
    </div>
    <div style="display: flex; align-items: center; gap: 8px;">
      <button onclick="adjustLeanLimit(-2)" style="width:36px; height:36px; border-radius:8px; border:1px solid var(--border); background:var(--card-elevated); color:#fff; font-size:16px; font-weight:800;">-</button>
      <span id="warnLimitText" style="font-size: 17px; font-weight: 800; min-width: 48px; text-align: center; color: var(--magenta);">48.0&deg;</span>
      <button onclick="adjustLeanLimit(+2)" style="width:36px; height:36px; border-radius:8px; border:1px solid var(--border); background:var(--card-elevated); color:#fff; font-size:16px; font-weight:800;">+</button>
    </div>
  </div>

  <div class="section-label">&#128250; Display LCD Moto (9 Schermate)</div>
  <div class="setting-row" style="flex-direction: column; align-items: stretch; gap: 10px;">
    <div style="display: flex; justify-content: space-between; align-items: center;">
      <div>
        <div class="setting-info">Schermata Attiva LCD</div>
        <div class="setting-sub" id="setupLcdSub">Tocca un tasto per cambiare display</div>
      </div>
      <button onclick="toggleAutoCycle()" id="btnAutoCycle" style="padding: 8px 12px; border-radius: 10px; border: 1px solid var(--border); background: var(--card-elevated); color: #fff; font-size: 11px; font-weight: 800; cursor: pointer;">&#128260; Auto-Cycle: OFF</button>
    </div>
    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px;">
      <button class="screen-btn" id="scBtn0" onclick="setLcdScreen(0)">&#127919; 1. Arc</button>
      <button class="screen-btn" id="scBtn1" onclick="setLcdScreen(1)">&#127937; 2. MotoGP</button>
      <button class="screen-btn" id="scBtn2" onclick="setLcdScreen(2)">&#9898; 3. G-G</button>
      <button class="screen-btn" id="scBtn3" onclick="setLcdScreen(3)">&#9889; 4. Curve</button>
      <button class="screen-btn" id="scBtn4" onclick="setLcdScreen(4)">&#129517; 5. Tour</button>
      <button class="screen-btn" id="scBtn5" onclick="setLcdScreen(5)">&#128640; 6. Pitch</button>
      <button class="screen-btn" id="scBtn6" onclick="setLcdScreen(6)">&#9992; 7. Horizon</button>
      <button class="screen-btn" id="scBtn7" onclick="setLcdScreen(7)">&#127769; 8. Minimal</button>
      <button class="screen-btn" id="scBtn8" onclick="setLcdScreen(8)">&#128202; 9. Stats</button>
    </div>
  </div>

  <div class="section-label">&#127937; Modalit&agrave; Pista (RaceChrono NimBLE)</div>
  <div class="setting-row" style="flex-direction: column; align-items: stretch; gap: 10px;">
    <div>
      <div class="setting-info">Passa a Modalit&agrave; Pista BLE</div>
      <div class="setting-sub">Spegne il Wi-Fi e trasmette a 20Hz verso RaceChrono. Per riaccendere il Wi-Fi premi a lungo (1.5s) il tasto fisico.</div>
    </div>
    <button class="btn-big" onclick="switchToBle()" style="background: #1f6feb; border-color: #388bfd;">
      &#127937; Attiva RaceChrono BLE (20Hz)
    </button>
  </div>

  <div class="btn-row" style="margin-top: 14px;">
    <button onclick="resetSessionRecords()" class="danger" style="background:rgba(255,23,68,0.1); border-color:var(--red); color:var(--red);">
      &#128472; Azzera Record Sessione
    </button>
  </div>
</div>

<!-- Bottom Navigation Bar -->
<div class="nav-bar">
  <div class="nav-item active" onclick="switchTab('viewCockpit', this)">
    <div class="nav-icon">&#127933;</div>
    <div>Cockpit</div>
  </div>
  <div class="nav-item" onclick="switchTab('viewCharts', this)">
    <div class="nav-icon">&#128200;</div>
    <div>Grafici</div>
  </div>
  <div class="nav-item" onclick="switchTab('viewCorners', this)">
    <div class="nav-icon">&#9201;</div>
    <div>Curve</div>
  </div>
  <div class="nav-item" onclick="switchTab('viewLogger', this)">
    <div class="nav-icon">&#128190;</div>
    <div>Rec</div>
  </div>
  <div class="nav-item" onclick="switchTab('viewSetup', this)">
    <div class="nav-icon">&#9881;</div>
    <div>Setup</div>
  </div>
</div>

<script>
let currentTab = 'viewCockpit';
let curWarnLimit = 48.0;
let isLoggingActive = false;
let rollHistory = [];
let ggTrail = [];
let isFetching = false;
let lastSuccessTime = Date.now();
let currentLcdScreen = 0;
let autoCycleActive = false;
const SCREEN_NAMES = [
  "Arc Gauge", "MotoGP Race", "G-G Circle", "Corner Apex",
  "Tour / Passi", "Pitch & Wheelie", "Horizon HUD", "Minimal Race", "Session Stats"
];

function setLcdScreen(mode) {
  fetch('/api/set_screen?mode=' + mode).catch(e=>{});
  highlightScreenBtn(mode);
}
function nextLcdScreen() {
  let next = (currentLcdScreen + 1) % 9;
  setLcdScreen(next);
}
function prevLcdScreen() {
  let prev = (currentLcdScreen + 8) % 9;
  setLcdScreen(prev);
}
function toggleAutoCycle() {
  autoCycleActive = !autoCycleActive;
  fetch('/api/set_autocycle?enable=' + (autoCycleActive ? '1' : '0')).catch(e=>{});
  let b = document.getElementById('btnAutoCycle');
  if (b) {
    b.innerText = '\uD83D\uDD04 Auto-Cycle: ' + (autoCycleActive ? 'ON' : 'OFF');
    b.style.borderColor = autoCycleActive ? 'var(--green)' : 'var(--border)';
    b.style.color = autoCycleActive ? 'var(--green)' : '#fff';
  }
  showToast('Auto-Cycle: ' + (autoCycleActive ? 'ATTIVO (6s)' : 'DISATTIVATO'));
}
function highlightScreenBtn(mode) {
  currentLcdScreen = mode;
  for (let i = 0; i < 9; i++) {
    let b = document.getElementById('scBtn' + i);
    if (b) {
      if (i === mode) b.classList.add('active');
      else b.classList.remove('active');
    }
  }
  let cName = SCREEN_NAMES[mode] || ("Schermata " + mode);
  let bCockpit = document.getElementById('cockpitLcdName');
  if (bCockpit) bCockpit.innerText = cName;
  let sSub = document.getElementById('setupLcdSub');
  if (sSub) sSub.innerText = 'Attiva: ' + cName;
}

function switchTab(tabId, el) {
  document.querySelectorAll('.view-container').forEach(v => v.classList.remove('active'));
  document.querySelectorAll('.nav-item').forEach(n => n.classList.remove('active'));
  document.getElementById(tabId).classList.add('active');
  el.classList.add('active');
  currentTab = tabId;
  if (navigator.vibrate) navigator.vibrate(15);
}

function showToast(msg) {
  let t = document.getElementById('toast');
  t.innerText = msg;
  t.style.display = 'block';
  setTimeout(() => { t.style.display = 'none'; }, 2200);
}

function updateGaugeNeedle(rollDeg) {
  let clamped = Math.max(-60, Math.min(60, rollDeg));
  let rad = (clamped - 90) * (Math.PI / 180);
  let cx = 110, cy = 110, r = 86;
  let nx = cx + Math.cos(rad) * r;
  let ny = cy + Math.sin(rad) * r;

  let needle = document.getElementById('gaugeNeedle');
  needle.setAttribute('x2', nx);
  needle.setAttribute('y2', ny);

  let absRoll = Math.abs(rollDeg);
  let col = 'var(--cyan)';
  if (absRoll >= curWarnLimit) col = 'var(--magenta)';
  else if (absRoll >= 45) col = 'var(--red)';
  else if (absRoll >= 32) col = 'var(--orange)';
  needle.setAttribute('stroke', col);
  document.getElementById('leanVal').style.color = col;
}

function drawRollTimeline(rollVal, warnLim) {
  let canvas = document.getElementById('rollTimelineCanvas');
  if (!canvas) return;
  let ctx = canvas.getContext('2d');
  let w = canvas.width;
  let h = canvas.height;
  let midY = h / 2;

  rollHistory.push(rollVal);
  if (rollHistory.length > 150) rollHistory.shift();

  ctx.clearRect(0, 0, w, h);

  // Reference lines: 0 deg (center), +-30 deg, +-48 deg
  ctx.strokeStyle = '#1d2436';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(0, midY); ctx.lineTo(w, midY);
  ctx.stroke();

  let pxPerDeg = (midY - 10) / 60;

  // Warning thresholds
  ctx.strokeStyle = 'rgba(213, 0, 249, 0.35)';
  ctx.setLineDash([4, 4]);
  let warnY1 = midY - warnLim * pxPerDeg;
  let warnY2 = midY + warnLim * pxPerDeg;
  ctx.beginPath();
  ctx.moveTo(0, warnY1); ctx.lineTo(w, warnY1);
  ctx.moveTo(0, warnY2); ctx.lineTo(w, warnY2);
  ctx.stroke();
  ctx.setLineDash([]);

  // Plot history
  ctx.strokeStyle = '#00f2fe';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  let step = w / 150;
  for (let i = 0; i < rollHistory.length; i++) {
    let x = i * step;
    let y = midY - (rollHistory[i] * pxPerDeg);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function drawGGCircle(gLat, gLon) {
  let canvas = document.getElementById('ggCircleCanvas');
  if (!canvas) return;
  let ctx = canvas.getContext('2d');
  let w = canvas.width;
  let h = canvas.height;
  let cx = w / 2;
  let cy = h / 2;
  let scale = (cx - 14) / 1.3;

  ggTrail.push({ x: gLat, y: gLon });
  if (ggTrail.length > 20) ggTrail.shift();

  ctx.clearRect(0, 0, w, h);

  // Concentric rings: 0.5G, 1.0G, 1.3G
  ctx.lineWidth = 1;
  [0.5, 1.0, 1.3].forEach(g => {
    ctx.strokeStyle = g === 1.0 ? '#8b949e' : '#1d2436';
    ctx.beginPath();
    ctx.arc(cx, cy, g * scale, 0, 2 * Math.PI);
    ctx.stroke();
  });

  // Crosshairs
  ctx.strokeStyle = '#1d2436';
  ctx.beginPath();
  ctx.moveTo(cx, 10); ctx.lineTo(cx, h - 10);
  ctx.moveTo(10, cy); ctx.lineTo(w - 10, cy);
  ctx.stroke();

  // Trail
  for (let i = 0; i < ggTrail.length - 1; i++) {
    let alpha = (i + 1) / ggTrail.length * 0.5;
    ctx.fillStyle = 'rgba(0, 230, 118, ' + alpha + ')';
    let px = cx + ggTrail[i].x * scale;
    let py = cy - ggTrail[i].y * scale;
    ctx.beginPath();
    ctx.arc(px, py, 3, 0, 2 * Math.PI);
    ctx.fill();
  }

  // Active Dot
  let px = cx + gLat * scale;
  let py = cy - gLon * scale;
  let totG = Math.sqrt(gLat * gLat + gLon * gLon);
  ctx.fillStyle = totG > 1.0 ? '#ff1744' : '#00e676';
  ctx.beginPath();
  ctx.arc(px, py, 6, 0, 2 * Math.PI);
  ctx.fill();
}

function updateUI(d) {
  let roll = d.roll || 0;
  let absRoll = Math.abs(roll);

  // Lean Value & Direction
  document.getElementById('leanVal').innerText = absRoll.toFixed(1) + '°';
  let dirEl = document.getElementById('leanDirText');
  if (roll < -1.5) {
    dirEl.innerHTML = '&#9664; PIEGA A SINISTRA';
    dirEl.style.color = 'var(--cyan)';
  } else if (roll > 1.5) {
    dirEl.innerHTML = 'PIEGA A DESTRA &#9654;';
    dirEl.style.color = 'var(--orange)';
  } else {
    dirEl.innerHTML = 'VERTICALE (0.0&deg;)';
    dirEl.style.color = 'var(--muted)';
  }
  updateGaugeNeedle(roll);

  // G-forces
  document.getElementById('gLonText').innerText = (d.gLon >= 0 ? '+' : '') + d.gLon.toFixed(2) + ' G';
  document.getElementById('gLatText').innerText = d.gLat.toFixed(2) + ' G';
  document.getElementById('rollRateText').innerText = Math.round(d.rawR || 0) + ' °/s';

  // 4-Tile Grid
  document.getElementById('compassText').innerText = Math.round(d.heading || 0) + '° ' + (d.cardinal || '-');
  let compB = document.getElementById('compassBadge');
  if (compB) {
    if (d.headingValid) {
      compB.innerText = '[VALIDA]'; compB.style.color = 'var(--green)';
    } else {
      compB.innerText = '[IN PIEGA / STIMATA]'; compB.style.color = 'var(--orange)';
    }
  }

  document.getElementById('dPlusText').innerText = '+' + Math.round(d.dPlus || 0) + ' m';
  document.getElementById('altText').innerText = 'Quota: ' + Math.round(d.alt || 0) + 'm';
  document.getElementById('pitchText').innerText = (d.pitch >= 0 ? '+' : '') + d.pitch.toFixed(1) + '°';
  document.getElementById('tempText').innerText = Math.round(d.temp || 0) + '°C';

  // Peak records
  document.getElementById('recMaxL').innerText = (d.maxL || 0).toFixed(1) + '°';
  document.getElementById('recMaxR').innerText = (d.maxR || 0).toFixed(1) + '°';
  document.getElementById('recBrake').innerText = '-' + (d.maxBrake || 0).toFixed(2) + ' G';
  document.getElementById('recAccel').innerText = '+' + (d.maxAccel || 0).toFixed(2) + ' G';

  // Crash banner
  let crashB = document.getElementById('crashBanner');
  if (crashB) crashB.style.display = d.isCrash ? 'block' : 'none';

  // Corner Analyzer Tab
  let cState = document.getElementById('curCornerState');
  let cBadge = document.getElementById('curCornerBadge');
  let cSub = document.getElementById('curCornerSub');
  if (d.inCorner) {
    cState.innerText = 'IN PIEGA: ' + d.curCornMax.toFixed(1) + '°';
    cState.style.color = roll < 0 ? 'var(--cyan)' : 'var(--orange)';
    cBadge.innerText = (roll < 0 ? 'SX' : 'DX') + ' ATTIVA';
    cBadge.className = 'corner-dir-badge ' + (roll < 0 ? 'left' : 'right');
    cSub.innerText = 'Durata: ' + d.curCornDur.toFixed(1) + 's';
  } else {
    cState.innerText = 'MOTO SUL DRITTO';
    cState.style.color = 'var(--green)';
    cBadge.innerText = 'RETTILINEO';
    cBadge.className = 'corner-dir-badge';
    cSub.innerText = 'Pronto per la prossima curva';
  }

  let cList = document.getElementById('cornerHistoryList');
  if (cList && d.corners && d.corners.length > 0) {
    let html = '';
    d.corners.forEach((c, idx) => {
      let isL = c.dir === 'L';
      html += '<div class="corner-card">' +
              '<div>' +
                '<span class="corner-dir-badge ' + (isL ? 'left' : 'right') + '">Curva #' + (idx + 1) + ' ' + (isL ? 'SX' : 'DX') + '</span>' +
                '<b style="font-size:15px; margin-left:8px;">' + c.max.toFixed(1) + '&deg;</b>' +
              '</div>' +
              '<div style="font-size:11px; color:var(--muted); text-align:right;">' +
                'Rate: <b>' + c.rate.toFixed(0) + '&deg;/s</b> | Durata: <b>' + c.dur.toFixed(1) + 's</b>' +
              '</div>' +
              '</div>';
    });
    cList.innerHTML = html;
  }

  // Datalogger Tab
  isLoggingActive = d.logging;
  let recBtn = document.getElementById('btnRecBig');
  let recDot = document.getElementById('recDot');
  let recLabel = document.getElementById('recBtnLabel');
  let recText = document.getElementById('recStatusText');
  if (d.logging) {
    recBtn.style.borderColor = 'var(--red)';
    recDot.style.background = 'var(--red)';
    recDot.style.animation = 'blink 0.8s infinite';
    recLabel.innerText = 'STOP';
    recText.innerText = 'REGISTRAZIONE IN CORSO...';
    recText.style.color = 'var(--red)';
  } else {
    recBtn.style.borderColor = 'var(--border)';
    recDot.style.background = '#484f58';
    recDot.style.animation = 'none';
    recLabel.innerText = 'REC';
    recText.innerText = 'PRONTO PER REGISTRARE';
    recText.style.color = 'var(--muted)';
  }
  document.getElementById('logKbText').innerText = (d.logSz / 1024).toFixed(1) + ' KB';
  document.getElementById('logSamplesText').innerText = d.logSamples + ' campioni';

  // Setup Tab
  curWarnLimit = d.warnLim || 48.0;
  document.getElementById('warnLimitText').innerText = curWarnLimit.toFixed(1) + '°';
  document.getElementById('canvasCurWarn').innerText = 'Soglia Limite: ' + curWarnLimit.toFixed(0) + '°';
  if (d.tR !== undefined) {
    document.getElementById('tareRollText').innerText = (d.tR >= 0 ? '+' : '') + d.tR.toFixed(1) + '°';
  }

  // LCD Screen status
  if (d.screen !== undefined && d.screen !== currentLcdScreen) {
    highlightScreenBtn(d.screen);
  }
  if (d.autoCycle !== undefined && d.autoCycle !== autoCycleActive) {
    autoCycleActive = d.autoCycle;
    let b = document.getElementById('btnAutoCycle');
    if (b) {
      b.innerText = '\uD83D\uDD04 Auto-Cycle: ' + (autoCycleActive ? 'ON' : 'OFF');
      b.style.borderColor = autoCycleActive ? 'var(--green)' : 'var(--border)';
      b.style.color = autoCycleActive ? 'var(--green)' : '#fff';
    }
  }

  // Draw Charts
  if (currentTab === 'viewCharts') {
    drawRollTimeline(roll, curWarnLimit);
    drawGGCircle(d.gLat, d.gLon);
  }
}

// Resilient, Non-Blocking 10Hz Polling Loop
function loopTelemetry() {
  if (isFetching) {
    if (Date.now() - lastSuccessTime > 2500) isFetching = false;
    setTimeout(loopTelemetry, 150);
    return;
  }
  isFetching = true;

  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 1200);

  fetch('/api/telemetry?t=' + Date.now(), { signal: controller.signal, cache: 'no-store' })
    .then(r => r.json())
    .then(d => {
      clearTimeout(timeoutId);
      isFetching = false;
      lastSuccessTime = Date.now();
      updateUI(d);
      document.getElementById('livePill').className = 'status-pill';
      document.getElementById('liveText').innerText = 'LIVE 10Hz';
      setTimeout(loopTelemetry, 100);
    })
    .catch(() => {
      clearTimeout(timeoutId);
      isFetching = false;
      document.getElementById('livePill').className = 'status-pill offline';
      document.getElementById('liveText').innerText = 'RICONNESSIONE...';
      setTimeout(loopTelemetry, 350);
    });
}

function toggleRecord() {
  if (isLoggingActive) {
    fetch('/api/log_stop', { method: 'POST' }).then(() => showToast('Registrazione fermata!'));
  } else {
    fetch('/api/log_start', { method: 'POST' }).then(() => showToast('Registrazione avviata su Flash!'));
  }
}

function downloadCsv() {
  window.location.href = '/api/download_csv';
}

function clearLogData() {
  if (confirm('Cancellare i dati di sessione memorizzati nella Flash?')) {
    fetch('/api/clear_log', { method: 'POST' }).then(() => showToast('Memoria azzerata.'));
  }
}

function tareZero() {
  fetch('/api/tare', { method: 'POST' }).then(() => showToast('Zero Tara Calibrato!'));
}

function resetTareDefault() {
  fetch('/api/resettare', { method: 'POST' }).then(() => showToast('Tara ripristinata a 0.0°'));
}

function adjustTare(dr, dp) {
  fetch('/api/adjusttare?dr=' + dr + '&dp=' + dp, { method: 'POST' }).then(() => showToast('Offset aggiornato!'));
}

function adjustLeanLimit(delta) {
  let newLim = curWarnLimit + delta;
  fetch('/api/set_lean_limit?limit=' + newLim, { method: 'POST' })
    .then(() => showToast('Soglia allarme: ' + newLim + '°'));
}

function resetSessionRecords() {
  if (confirm('Azzerare tutti i record di staccata e piega massima?')) {
    fetch('/api/reset', { method: 'POST' }).then(() => showToast('Record azzerati!'));
  }
}

function switchToBle() {
  if (confirm("Attivare Modalità Pista (RaceChrono NimBLE 20Hz)?\n\nIl Wi-Fi verrà disattivato per garantire zero latenza. Per riaccendere il Wi-Fi premi a lungo (1.5s) il pulsante fisico della moto.")) {
    fetch('/api/set_radio_mode?mode=ble', { method: 'POST' })
      .then(() => showToast('Passaggio a RaceChrono BLE avviato...'));
  }
}

// Start live polling loop immediately
loopTelemetry();
</script>
</body>
</html>
)rawliteral";

class WebDashboard {
public:
    WebDashboard() : 
        _server(80), 
        _active(false), 
        _currentRadioMode(RADIO_MODE_DASHBOARD),
        _pendingRadioSwitch(RADIO_MODE_DASHBOARD),
        _pendingRadioSwitchTime(0),
        _filter(nullptr), 
        _dynPtr(nullptr), 
        _uiPtr(nullptr),
        _logger(nullptr),
        _ble(nullptr) {}

    bool isRunning() const { return _active; }

    void setRadioMode(RadioMode mode) {
        _currentRadioMode = mode;
    }

    bool checkRadioSwitch(RadioMode& newMode) {
        if (_pendingRadioSwitchTime > 0 && millis() >= _pendingRadioSwitchTime) {
            newMode = _pendingRadioSwitch;
            _pendingRadioSwitchTime = 0;
            return true;
        }
        return false;
    }

    void start(MotorcycleFilter* filter, const MotorcycleDynamics* dynPtr, DisplayUI* uiPtr = nullptr,
               SessionLogger* logger = nullptr, BluetoothTelemetry* ble = nullptr) {
        _filter = filter;
        _dynPtr = dynPtr;
        _uiPtr = uiPtr;
        _logger = logger;
        _ble = ble;
        _currentRadioMode = RADIO_MODE_DASHBOARD;
        _pendingRadioSwitchTime = 0;

        // Registra listener eventi Wi-Fi
        static bool eventHandlerRegistered = false;
        if (!eventHandlerRegistered) {
            WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
                switch (event) {
                    case ARDUINO_EVENT_WIFI_AP_START:
                        Serial.println("[WIFI-EVENT] >>> SoftAP Avviato e Beaconing ATTIVO! <<<");
                        break;
                    case ARDUINO_EVENT_WIFI_AP_STOP:
                        Serial.println("[WIFI-EVENT] SoftAP Arrestato!");
                        break;
                    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
                        Serial.println("[WIFI-EVENT] >>> Dispositivo connesso all'Hotspot FZ8! <<<");
                        break;
                    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
                        Serial.println("[WIFI-EVENT] Dispositivo disconnesso dall'Hotspot.");
                        break;
                    default:
                        break;
                }
            });
            eventHandlerRegistered = true;
        }

        // 1. Avvio SoftAP diretto e affidabile
        WiFi.persistent(false);
        WiFi.mode(WIFI_AP_STA);
        const char* pass = (strlen(AP_PASSWORD) >= 8) ? AP_PASSWORD : nullptr;
        bool apOk = WiFi.softAP(AP_SSID, pass);
        delay(100);

        IPAddress apIp = WiFi.softAPIP();
        Serial.printf("[WIFI] SoftAP Avviato (%s): SSID='%s', Sicurezza='%s', IP=%s\n", 
                      apOk ? "OK" : "FALLITO", AP_SSID, pass ? "WPA2" : "APERTA (NESSUNA PASSWORD)", apIp.toString().c_str());

        // 2. Avvia Captive Portal DNS Server (porta 53, risolve qualsiasi dominio su IP AP)
        _dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        _dnsServer.start(53, "*", apIp);
        Serial.println("[WIFI] Captive Portal DNS Server attivo su porta 53 (* -> 192.168.4.1)");

        // 3. Avvia mDNS responder (http://fz8.local)
        if (MDNS.begin(MDNS_HOSTNAME)) {
            MDNS.addService("http", "tcp", 80);
            Serial.printf("[WIFI] mDNS responder attivo: http://%s.local\n", MDNS_HOSTNAME);
        }

        // 4. Captive Portal Probes & Redirects
        auto captiveRedirect = [this]() {
            _server.sendHeader("Location", "http://192.168.4.1/");
            _server.sendHeader("Cache-Control", "no-cache");
            _server.send(302, "text/plain", "");
        };

        _server.on("/generate_204", captiveRedirect); // Android
        _server.on("/gen_204", captiveRedirect);
        _server.on("/hotspot-detect.html", captiveRedirect); // Apple iOS
        _server.on("/ncsi.txt", [this]() {
            _server.send(200, "text/plain", "Microsoft NCSI");
        });

        // 5. Main Dashboard Endpoint
        _server.on("/", HTTP_GET, [this]() {
            _server.sendHeader("Connection", "close");
            _server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
            _server.send_P(200, "text/html", DASHBOARD_HTML);
        });

        // 6. Live Telemetry API (Ultra-low latency, CORS enabled, no-cache)
        _server.on("/api/telemetry", HTTP_GET, [this]() {
            if (!_dynPtr) {
                _server.send(500, "application/json", "{}");
                return;
            }

            char cornersJson[320] = "[";
            if (_dynPtr->cornerHistoryCount > 0) {
                for (int i = 0; i < _dynPtr->cornerHistoryCount && i < 5; i++) {
                    char cBuf[64];
                    snprintf(cBuf, sizeof(cBuf), "%s{\"dir\":\"%s\",\"max\":%.1f,\"rate\":%.1f,\"dur\":%.1f}",
                        (i > 0) ? "," : "",
                        _dynPtr->lastCorners[i].isLeft ? "L" : "R",
                        _dynPtr->lastCorners[i].maxRollDeg,
                        _dynPtr->lastCorners[i].maxRollRateDps,
                        _dynPtr->lastCorners[i].durationSec);
                    strcat(cornersJson, cBuf);
                }
            }
            strcat(cornersJson, "]");

            char json[1500];
            snprintf(json, sizeof(json), 
                "{"
                "\"roll\":%.1f,\"pitch\":%.1f,"
                "\"rawR\":%.1f,\"rawP\":%.1f,"
                "\"maxL\":%.1f,\"maxR\":%.1f,"
                "\"maxBrake\":%.2f,\"maxAccel\":%.2f,"
                "\"maxPitchUp\":%.1f,\"maxPitchDown\":%.1f,"
                "\"isWheelie\":%s,\"isStoppie\":%s,"
                "\"gLon\":%.2f,\"gLat\":%.2f,"
                "\"alt\":%.1f,\"temp\":%.1f,"
                "\"heading\":%.1f,\"cardinal\":\"%s\",\"headingValid\":%s,"
                "\"dPlus\":%.1f,\"vSpeed\":%.1f,\"slope\":%.1f,"
                "\"isWarn\":%s,\"isCrash\":%s,\"warnLim\":%.1f,"
                "\"inCorner\":%s,\"curCornMax\":%.1f,\"curCornDur\":%.1f,"
                "\"corners\":%s,"
                "\"logging\":%s,\"logSz\":%u,\"logSamples\":%u,"
                "\"radioMode\":%d,"
                "\"screen\":%d,\"screenName\":\"%s\",\"autoCycle\":%s,"
                "\"tR\":%.1f,\"tP\":%.1f,"
                "\"apIp\":\"192.168.4.1\""
                "}",
                _dynPtr->rollDeg, _dynPtr->pitchDeg,
                _filter ? _filter->getRawRoll() : 0.0f,
                _filter ? _filter->getRawPitch() : 0.0f,
                _dynPtr->maxLeanLeft, _dynPtr->maxLeanRight,
                _dynPtr->maxBrakingG, _dynPtr->maxAccelG,
                _dynPtr->maxPitchUp, _dynPtr->maxPitchDown,
                _dynPtr->isWheelie ? "true" : "false",
                _dynPtr->isStoppie ? "true" : "false",
                _dynPtr->gLongitudinal, _dynPtr->gLateral,
                _dynPtr->altitudeM, _dynPtr->tempC,
                _dynPtr->headingDeg, _dynPtr->cardinal,
                _dynPtr->isHeadingValid ? "true" : "false",
                _dynPtr->totalElevationGainM, _dynPtr->verticalSpeedMps, _dynPtr->roadGradientPct,
                _dynPtr->isLeanWarning ? "true" : "false",
                _dynPtr->isCrashDetected ? "true" : "false",
                _dynPtr->maxLeanThreshold,
                _dynPtr->isInsideCorner ? "true" : "false",
                _dynPtr->currentCornerMaxRoll,
                _dynPtr->currentCornerDuration,
                cornersJson,
                (_logger && _logger->isLogging()) ? "true" : "false",
                (unsigned)(_logger ? _logger->getFileSize() : 0),
                (unsigned)(_logger ? _logger->getSampleCount() : 0),
                (int)_currentRadioMode,
                _uiPtr ? (int)_uiPtr->getScreen() : 0,
                _uiPtr ? _uiPtr->getScreenName(_uiPtr->getScreen()) : "Unknown",
                (_uiPtr && _uiPtr->isAutoCycle()) ? "true" : "false",
                _filter ? _filter->getTareRoll() : 0.0f,
                _filter ? _filter->getTarePitch() : 0.0f
            );

            _server.sendHeader("Access-Control-Allow-Origin", "*");
            _server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
            _server.sendHeader("Pragma", "no-cache");
            _server.sendHeader("Expires", "0");
            _server.sendHeader("Connection", "close");
            _server.send(200, "application/json", json);
        });

        // 7. Datalogger Endpoints
        _server.on("/api/log_start", HTTP_ANY, [this]() {
            if (_logger) _logger->startSession();
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/log_stop", HTTP_ANY, [this]() {
            if (_logger) _logger->stopSession();
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/download_csv", HTTP_GET, [this]() {
            if (_logger) {
                _logger->streamFileTo(_server);
            } else {
                _server.sendHeader("Connection", "close");
                _server.send(404, "text/plain", "Logger non inizializzato");
            }
        });

        _server.on("/api/clear_log", HTTP_ANY, [this]() {
            if (_logger) _logger->clearSession();
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        // 8. Radio Mode Switch Endpoint
        _server.on("/api/set_radio_mode", HTTP_ANY, [this]() {
            String m = _server.arg("mode");
            if (m == "ble" || m == "1") {
                _server.sendHeader("Connection", "close");
                _server.send(200, "text/plain", "OK");
                _pendingRadioSwitch = RADIO_MODE_RACECHRONO;
                _pendingRadioSwitchTime = millis() + 400;
            } else {
                _server.sendHeader("Connection", "close");
                _server.send(200, "text/plain", "OK");
                _pendingRadioSwitch = RADIO_MODE_DASHBOARD;
                _pendingRadioSwitchTime = millis() + 400;
            }
        });

        // 9. Safety & Settings Endpoints
        _server.on("/api/set_lean_limit", HTTP_ANY, [this]() {
            if (_server.hasArg("limit") && _filter) {
                float lim = _server.arg("limit").toFloat();
                _filter->setMaxLeanThreshold(lim);
            }
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/tare", HTTP_ANY, [this]() {
            if (_filter) {
                _filter->tareZero();
                if (_uiPtr) {
                    _uiPtr->showTareNotice(_filter->getTareRoll(), _filter->getTarePitch());
                }
            }
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/resettare", HTTP_ANY, [this]() {
            if (_filter) {
                _filter->resetTare();
                if (_uiPtr) {
                    _uiPtr->showTareResetNotice();
                }
            }
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/adjusttare", HTTP_ANY, [this]() {
            if (_filter && _server.hasArg("dr") && _server.hasArg("dp")) {
                float dr = _server.arg("dr").toFloat();
                float dp = _server.arg("dp").toFloat();
                _filter->adjustTare(dr, dp);
            }
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/reset", HTTP_ANY, [this]() {
            if (_filter) _filter->resetRecords();
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        // 10. LCD Screen Remote Control
        _server.on("/api/set_screen", HTTP_ANY, [this]() {
            if (_server.hasArg("mode") && _uiPtr) {
                int m = _server.arg("mode").toInt();
                if (m >= 0 && m < SCREEN_COUNT) {
                    _uiPtr->setScreen((UIScreenMode)m);
                }
            }
            _server.sendHeader("Access-Control-Allow-Origin", "*");
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        _server.on("/api/set_autocycle", HTTP_ANY, [this]() {
            if (_uiPtr) {
                bool en = (_server.arg("enable") == "1" || _server.arg("enable") == "true");
                uint32_t interval = _server.hasArg("interval") ? _server.arg("interval").toInt() : 6000;
                _uiPtr->setAutoCycle(en, interval);
            }
            _server.sendHeader("Access-Control-Allow-Origin", "*");
            _server.sendHeader("Connection", "close");
            _server.send(200, "text/plain", "OK");
        });

        // 10. Fallback Catch-All
        _server.onNotFound([this]() {
            String uri = _server.uri();
            if (!uri.startsWith("/api/")) {
                _server.sendHeader("Location", "http://192.168.4.1/");
                _server.send(302, "text/plain", "");
            } else {
                _server.send(404, "text/plain", "Not Found");
            }
        });

        _server.begin();
        _active = true;
        Serial.println("[WIFI] Server Web Mobile avviato con successo.");
    }

    void stop() {
        if (!_active) return;
        _dnsServer.stop();
        _server.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_OFF);
        _active = false;
        Serial.println("[WIFI] Server Web e Hotspot fermati.");
    }

    void handleClient() {
        if (_active) {
            _dnsServer.processNextRequest();
            _server.handleClient();
        }
    }

private:
    WebServer _server;
    DNSServer _dnsServer;
    Preferences _prefs;
    bool _active;
    RadioMode _currentRadioMode;
    RadioMode _pendingRadioSwitch;
    uint32_t _pendingRadioSwitchTime;
    MotorcycleFilter* _filter;
    const MotorcycleDynamics* _dynPtr;
    DisplayUI* _uiPtr;
    SessionLogger* _logger;
    BluetoothTelemetry* _ble;
};

#endif // WEB_DASHBOARD_H
