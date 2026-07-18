/*
  ESP32 + PCA9685  —  15x 360° Continuous-Rotation Servo WebSocket Dashboard
  ============================================================================
  TIMING-BASED (OPEN-LOOP) ANGLE CONTROL
  ---------------------------------------
  Every channel is a 360° continuous-rotation servo. These servos have NO
  position feedback — the pulse width controls SPEED/DIRECTION, not angle
  (e.g. ~1500us = stop, <1500us = spin one way, >1500us = spin the other way,
  further from 1500us = faster). There is no way to command "go to 90°"
  directly like a normal positional servo.

  So angle control is done by TIMING: we know (from your measurement) that a
  full-speed 180° sweep takes ~4000ms. From that:
        ms per degree = 4000 / 180 = 22.222 ms/deg
  To move by N degrees: spin at full speed for (N * 22.222) ms, then stop.
  Examples:  90° -> 2000ms   45° -> 1000ms   180° -> 4000ms   360° -> 8000ms

  IMPORTANT — THIS IS ESTIMATION, NOT MEASUREMENT:
  There is no feedback, so the firmware's idea of "current angle" is only as
  good as the calibration constant and a known starting point. Voltage sag,
  load, and servo-to-servo variation will all skew the real angle over time.
  -> Physically center/zero every horn by hand after flashing, then hit
     "Sync All to 0°" so the firmware's estimate matches reality.
  -> If you ever notice an arm has drifted (visual check), use that channel's
     per-servo "Sync" button to re-anchor just that one, without moving it.
  -> Recalibrate SERVO_MS_PER_180[] per channel if a servo consistently
     over/undershoots — real servos vary even within the same batch.

  UI: each slider runs 0° to 360° in steps of 45° (0,45,90,...,360). Moving
  the slider sends a target angle; firmware computes the spin duration from
  the calibration constant, spins that channel only, then stops it — fully
  non-blocking (millis()-based), so all 15 channels can move independently
  and simultaneously without blocking the WebSocket/UI.

  YOUR HARDWARE
  -------------
  - PCA9685 16-channel PWM driver over I2C (SDA=21, SCL=22 on ESP32 by default)
  - Channels 0-14 -> 15x 360° continuous-rotation servos
    (Adjust SERVO_MIN_US/SERVO_MAX_US/STOP_US below per channel if a servo
     buzzes at "stop", creeps when it should be still, or spins too
     slow/fast — every servo, and even every unit in a batch, differs.)

  REQUIRED LIBRARIES (Arduino IDE -> Tools -> Manage Libraries):
    1. Adafruit PWM Servo Driver Library   (by Adafruit)
    2. ESPAsyncWebServer                    (by lacamera / me-no-dev)
    3. AsyncTCP                             (by me-no-dev)
    4. ArduinoJson  (v6.x)                  (by Benoit Blanchon)
  "Preferences.h" is built into the ESP32 core — no install needed.

  WIRING
  ------
  PCA9685 VCC  -> ESP32 3V3 (logic only)
  PCA9685 GND  -> ESP32 GND (common ground with servo supply too!)
  PCA9685 SDA  -> ESP32 GPIO 21
  PCA9685 SCL  -> ESP32 GPIO 22
  PCA9685 V+   -> external 5-6V servo power supply (NOT from ESP32/USB — 15
                  servos draw far more current than USB can give)
  Servos       -> PCA9685 channels 0-14, signal pin per channel
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Preferences.h>

// ============================== CONFIG ==============================

const char* AP_SSID = "ESP32-ServoControl";
const char* AP_PASS = "servo1234";      // 8+ chars, or "" for an open network

const int NUM_SERVOS  = 15;
const int PCA_ADDR    = 0x40;

// Per-channel FULL-SPEED pulses in microseconds (index = PCA9685 channel 0-14).
// For a 360° continuous-rotation servo these are NOT position limits — they
// are "spin at max speed in direction A" (MIN) and "spin at max speed in
// direction B" (MAX). STOP_US is the neutral point where the servo should
// sit still (often ~1500us, but real units drift — trim per channel if a
// servo creeps at "stop").
int SERVO_MIN_US[NUM_SERVOS] = {   // full-speed CCW
  900,900,900,900,900,900,900,900,900,900,900,900,900,900,900
};
int SERVO_MAX_US[NUM_SERVOS] = {   // full-speed CW
  2100,2100,2100,2100,2100,2100,2100,2100,2100,2100,2100,2100,2100,2100,2100
};
int STOP_US[NUM_SERVOS] = {        // neutral / stopped
  1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500,1500
};

// CALIBRATION: measured time (ms) for a full-speed 180° sweep, per channel.
// You measured ~4000ms for 180° -> that's the default for every channel.
// Recalibrate an individual channel here if it consistently over/undershoots.
float SERVO_MS_PER_180[NUM_SERVOS] = {
  4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000
};

// Default "home" reference angle (degrees, 0-360) for each channel.
uint16_t HOME_ANGLES[NUM_SERVOS] = {
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

const int NUM_POSE_SLOTS   = 10;
const uint32_t TICK_MS      = 15;   // spin-completion check / live-estimate interval
const uint32_t BROADCAST_MS = 66;   // ~15 Hz state broadcast while anything is spinning

// ======================================================================

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences prefs;

float    currentAngle[NUM_SERVOS];    // best-effort ESTIMATE of position, 0-360
float    targetAngle[NUM_SERVOS];     // where the current/last spin is headed
bool     servoActive[NUM_SERVOS];     // true once a channel has been commanded/synced
bool     spinning[NUM_SERVOS];        // true while a timed spin is in progress
int8_t   spinDir[NUM_SERVOS];         // +1 = toward MAX_US (CW), -1 = toward MIN_US (CCW), 0 = idle
float    spinStartAngle[NUM_SERVOS];  // currentAngle at the moment this spin began
uint32_t spinStartMillis[NUM_SERVOS];
uint32_t spinTotalMs[NUM_SERVOS];     // total planned duration of this spin
uint32_t spinEndMillis[NUM_SERVOS];

bool     poseSlotUsed[NUM_POSE_SLOTS];
bool     eStop = false;
uint32_t lastTick = 0;
uint32_t lastBroadcast = 0;
bool     dirty = false; // true while any servo is still spinning (needs broadcasting)

struct PoseData {
  bool used;
  uint16_t angles[NUM_SERVOS];  // 0-360, so uint16_t (not uint8_t like a 0-180 pose)
};

// ============================== WEB PAGE ==============================
const char index_html[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Servo Dashboard</title>
<style>
  :root{
    --bg:#0e1016; --card:#1b1f2b; --card2:#20263a; --border:#2a3040;
    --accent:#4f8cff; --accent2:#8b5cf6; --ok:#33d17a; --warn:#ffb020;
    --danger:#ff4d5e; --text:#e9edf5; --muted:#8a92a6;
  }
  *{box-sizing:border-box;}
  body{
    margin:0; font-family:'Segoe UI',Roboto,Arial,sans-serif;
    background:radial-gradient(circle at top,#171b26,var(--bg));
    color:var(--text); min-height:100vh; padding:22px 16px 60px;
  }
  header{text-align:center; margin-bottom:6px;}
  header h1{
    margin:0; font-size:1.6rem; font-weight:700;
    background:linear-gradient(90deg,var(--accent),var(--accent2));
    -webkit-background-clip:text; background-clip:text; color:transparent;
  }
  #statusBar{
    display:flex; align-items:center; justify-content:center; gap:8px;
    font-size:0.85rem; color:var(--muted); margin:8px 0 18px;
  }
  .dot{
    width:9px; height:9px; border-radius:50%; background:var(--danger);
    box-shadow:0 0 0 0 rgba(255,77,94,.6); animation:pulseDot 1.6s infinite;
  }
  .dot.ok{ background:var(--ok); box-shadow:0 0 0 0 rgba(51,209,122,.6); }
  @keyframes pulseDot{
    0%{box-shadow:0 0 0 0 rgba(255,77,94,.5);}
    70%{box-shadow:0 0 0 8px rgba(255,77,94,0);}
    100%{box-shadow:0 0 0 0 rgba(255,77,94,0);}
  }
  .dot.ok{animation:pulseDotOk 1.6s infinite;}
  @keyframes pulseDotOk{
    0%{box-shadow:0 0 0 0 rgba(51,209,122,.5);}
    70%{box-shadow:0 0 0 8px rgba(51,209,122,0);}
    100%{box-shadow:0 0 0 0 rgba(51,209,122,0);}
  }

  #estopBanner{
    display:none; max-width:680px; margin:0 auto 20px; text-align:center;
    background:linear-gradient(90deg,#4a1218,#5c1620); border:1px solid var(--danger);
    color:#ffd7db; padding:10px 16px; border-radius:12px; font-weight:600;
    letter-spacing:.4px; animation:flash 1s infinite;
  }
  @keyframes flash{ 50%{opacity:.55;} }

  .toolbar{
    display:flex; justify-content:center; gap:10px; margin-bottom:26px; flex-wrap:wrap;
  }
  .btn{
    background:var(--card2); color:var(--text); border:1px solid var(--border);
    padding:10px 18px; border-radius:10px; font-size:0.85rem; cursor:pointer;
    transition:.15s; font-weight:600; letter-spacing:.3px;
  }
  .btn:hover{ border-color:var(--accent); color:var(--accent); transform:translateY(-1px); }
  .btn:active{ transform:translateY(0); }
  .btn.danger{ border-color:var(--danger); color:var(--danger); }
  .btn.danger:hover{ background:var(--danger); color:#fff; }
  .btn.warn{ border-color:var(--warn); color:var(--warn); }
  .btn.warn:hover{ background:var(--warn); color:#1a1500; }

  section{ max-width:1200px; margin:0 auto 30px; }
  .section-title{
    font-size:0.8rem; text-transform:uppercase; letter-spacing:1.2px;
    color:var(--muted); margin:0 4px 12px;
  }

  .grid{
    display:grid; gap:14px;
    grid-template-columns:repeat(auto-fill,minmax(240px,1fr));
  }
  .card{
    background:var(--card); border:1px solid var(--border); border-radius:14px;
    padding:15px 18px 13px; box-shadow:0 4px 14px rgba(0,0,0,.25);
    transition:opacity .2s, transform .2s;
  }
  .card.locked{ opacity:.4; pointer-events:none; }
  .card-head{ display:flex; justify-content:space-between; align-items:center; margin-bottom:9px; }
  .card-head .name{ font-weight:600; font-size:0.92rem; display:flex; align-items:center; gap:6px; }
  .card-head .tag{
    font-size:0.6rem; padding:2px 6px; border-radius:6px; font-weight:700;
    background:#2a3145; color:var(--muted); letter-spacing:.4px;
  }
  .card-head .power{
    font-size:0.58rem; padding:2px 6px; border-radius:6px; font-weight:700;
    background:#2a3145; color:var(--muted); letter-spacing:.4px;
  }
  .card-head .power.on{ background:#123321; color:var(--ok); }
  .card-head .power.spin{ background:#3a2a12; color:var(--warn); }
  .card-head .value{
    font-variant-numeric:tabular-nums; background:#12141c; border:1px solid var(--border);
    border-radius:8px; padding:2px 9px; font-size:0.85rem; color:var(--accent); min-width:44px; text-align:center;
  }
  .etatext{ font-size:0.65rem; color:var(--muted); margin-top:6px; min-height:12px; }
  input[type=range]{
    -webkit-appearance:none; width:100%; height:6px; border-radius:4px;
    background:linear-gradient(90deg,var(--accent),var(--accent2)); outline:none; cursor:pointer;
  }
  input[type=range]::-webkit-slider-thumb{
    -webkit-appearance:none; width:20px; height:20px; border-radius:50%;
    background:#fff; border:3px solid var(--accent); box-shadow:0 2px 6px rgba(0,0,0,.4);
    cursor:pointer; margin-top:-7px; transition:transform .1s;
  }
  input[type=range]:active::-webkit-slider-thumb{ transform:scale(1.15); }
  input[type=range]::-moz-range-thumb{
    width:20px; height:20px; border-radius:50%; background:#fff;
    border:3px solid var(--accent); cursor:pointer;
  }
  .scale{ display:flex; justify-content:space-between; font-size:0.6rem; color:var(--muted); margin-top:4px; }
  .syncrow{ display:flex; justify-content:flex-end; margin-top:8px; }
  .syncbtn{
    font-size:0.65rem; padding:4px 8px; border-radius:7px; border:1px solid var(--border);
    background:var(--card2); color:var(--muted); cursor:pointer;
  }
  .syncbtn:hover{ border-color:var(--accent); color:var(--accent); }

  .pose-grid{
    display:grid; gap:10px; grid-template-columns:repeat(auto-fill,minmax(150px,1fr));
  }
  .pose-card{
    background:var(--card); border:1px solid var(--border); border-radius:12px;
    padding:12px; text-align:center;
  }
  .pose-card .plabel{ font-size:0.8rem; font-weight:600; margin-bottom:8px; }
  .pose-card .pstate{
    font-size:0.62rem; color:var(--muted); margin-bottom:8px; letter-spacing:.3px;
  }
  .pose-card .pstate.saved{ color:var(--ok); }
  .pose-actions{ display:flex; gap:6px; justify-content:center; }
  .pose-actions button{
    flex:1; padding:6px 4px; font-size:0.72rem; border-radius:7px;
    border:1px solid var(--border); background:var(--card2); color:var(--text); cursor:pointer;
  }
  .pose-actions .load:hover{ border-color:var(--accent); color:var(--accent); }
  .pose-actions .save:hover{ border-color:var(--ok); color:var(--ok); }
  .pose-actions .clear:hover{ border-color:var(--danger); color:var(--danger); }

  footer{ text-align:center; color:var(--muted); font-size:0.75rem; margin-top:36px; }
</style>
</head>
<body>

<header><h1>ESP32 · PCA9685 · 360° Servo Dashboard</h1></header>
<div id="statusBar"><span class="dot" id="statusDot"></span><span id="statusText">Connecting…</span></div>
<div id="estopBanner">⛔ EMERGENCY STOP ACTIVE — servos disabled</div>

<div class="toolbar">
  <button class="btn" onclick="cmd({type:'centerAll'})">Center All (180°)</button>
  <button class="btn" onclick="cmd({type:'home'})">🏠 Home Position</button>
  <button class="btn" onclick="syncAll()">📍 Sync All to 0°</button>
  <button class="btn warn" id="estopBtn" onclick="toggleEstop()">⛔ Emergency Stop</button>
</div>

<section>
  <div class="section-title">Servos · 0°–360° in 45° steps · timed spin (≈22.2ms/°, 4s per 180°)</div>
  <div class="grid" id="grid"></div>
</section>

<section>
  <div class="section-title">Poses (10 slots)</div>
  <div class="pose-grid" id="poseGrid"></div>
</section>

<footer>Live over WebSocket · Timing-based open-loop control · Re-sync after any manual/physical repositioning</footer>

<script>
  const NUM_SERVOS = 15;
  const NUM_POSES  = 10;
  const grid = document.getElementById('grid');
  const poseGrid = document.getElementById('poseGrid');
  const statusDot = document.getElementById('statusDot');
  const statusText = document.getElementById('statusText');
  const estopBanner = document.getElementById('estopBanner');
  const estopBtn = document.getElementById('estopBtn');

  let estopActive = false;

  // ---- Build 15 servo cards (0-360, step 45) ----
  for (let i = 1; i <= NUM_SERVOS; i++) {
    const card = document.createElement('div');
    card.className = 'card';
    card.id = `card${i}`;
    card.innerHTML = `
      <div class="card-head">
        <span class="name">Servo ${i} <span class="tag">360°</span> <span class="power" id="pwr${i}">OFF</span></span>
        <span class="value" id="val${i}">0°</span>
      </div>
      <input type="range" min="0" max="360" step="45" value="0" id="slider${i}">
      <div class="scale"><span>0°</span><span>90°</span><span>180°</span><span>270°</span><span>360°</span></div>
      <div class="etatext" id="eta${i}"></div>
      <div class="syncrow"><button class="syncbtn" onclick="syncOne(${i})">📍 Sync this to slider value</button></div>
    `;
    grid.appendChild(card);

    const slider = card.querySelector(`#slider${i}`);
    const valLabel = card.querySelector(`#val${i}`);

    // Snap to nearest 45° as the user drags, live-update the label…
    slider.addEventListener('input', () => {
      valLabel.textContent = slider.value + '°';
    });
    // …but only actually command a spin once they release (avoids firing a
    // new timed spin on every intermediate tick while dragging).
    slider.addEventListener('change', () => {
      cmd({ type: 'servo', id: i, angle: Number(slider.value) });
    });
  }

  // ---- Build 10 pose slot cards ----
  for (let s = 1; s <= NUM_POSES; s++) {
    const card = document.createElement('div');
    card.className = 'pose-card';
    card.id = `pose${s}`;
    card.innerHTML = `
      <div class="plabel">Pose ${s}</div>
      <div class="pstate" id="pstate${s}">empty</div>
      <div class="pose-actions">
        <button class="load" onclick="cmd({type:'loadPose', slot:${s}})">▶ Load</button>
        <button class="save" onclick="cmd({type:'savePose', slot:${s}})">💾 Save</button>
        <button class="clear" onclick="cmd({type:'clearPose', slot:${s}})">✕</button>
      </div>
    `;
    poseGrid.appendChild(card);
  }

  // ---- WebSocket ----
  let ws, reconnectTimer;
  function connect() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => {
      statusDot.classList.add('ok');
      statusText.textContent = 'Connected';
    };
    ws.onclose = () => {
      statusDot.classList.remove('ok');
      statusText.textContent = 'Disconnected — retrying…';
      clearTimeout(reconnectTimer);
      reconnectTimer = setTimeout(connect, 1500);
    };
    ws.onerror = () => ws.close();
    ws.onmessage = (evt) => {
      let msg;
      try { msg = JSON.parse(evt.data); } catch (e) { return; }
      handleMessage(msg);
    };
  }

  function handleMessage(msg) {
    if (msg.type === 'state') {
      msg.angles.forEach((a, idx) => {
        const i = idx + 1;
        // Don't yank the slider out from under a user who's mid-drag on it.
        const slider = document.getElementById(`slider${i}`);
        if (document.activeElement !== slider) slider.value = a;
        document.getElementById(`val${i}`).textContent = Math.round(a) + '°';
      });
      if (msg.active) {
        msg.active.forEach((on, idx) => {
          const i = idx + 1;
          const spin = msg.spinning ? msg.spinning[idx] : false;
          const el = document.getElementById(`pwr${i}`);
          el.textContent = spin ? 'SPINNING' : (on ? 'LIVE' : 'OFF');
          el.classList.toggle('on', on && !spin);
          el.classList.toggle('spin', spin);
          const eta = document.getElementById(`eta${i}`);
          eta.textContent = spin ? 'moving…' : '';
        });
      }
    } else if (msg.type === 'status') {
      setEstop(msg.estop);
    } else if (msg.type === 'poseList') {
      msg.slots.forEach((used, idx) => {
        const s = idx + 1;
        const el = document.getElementById(`pstate${s}`);
        el.textContent = used ? 'saved' : 'empty';
        el.classList.toggle('saved', used);
      });
    }
  }

  function setEstop(active) {
    estopActive = active;
    estopBanner.style.display = active ? 'block' : 'none';
    estopBtn.textContent = active ? '✅ Resume' : '⛔ Emergency Stop';
    for (let i = 1; i <= NUM_SERVOS; i++) {
      document.getElementById(`card${i}`).classList.toggle('locked', active);
    }
  }

  function toggleEstop() {
    cmd({ type: estopActive ? 'resume' : 'estop' });
  }

  function syncOne(i) {
    const slider = document.getElementById(`slider${i}`);
    cmd({ type: 'sync', id: i, angle: Number(slider.value) });
  }

  function syncAll() {
    cmd({ type: 'syncAll', angle: 0 });
  }

  function cmd(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
  }

  connect();
</script>
</body>
</html>
)HTML";

// ============================== SERIAL DIAGNOSTICS ==============================
// Bypasses WiFi/WebSocket entirely so you can test/calibrate a single PCA9685
// channel in isolation. Open Serial Monitor at 115200 baud, line ending
// "Newline", and type:
//
//   raw <ch> <us>        -> send a RAW pulse width (us) directly to a channel.
//                            Useful to find true STOP_US (where it sits still
//                            with no creep) and true full-speed MIN/MAX_US.
//   spin <ch> <ms> <dir>  -> timed spin test: dir=1 (CW/MAX_US) or -1 (CCW/MIN_US)
//                            for <ms> milliseconds, then auto-stop. e.g.
//                            "spin 3 2000 1" spins channel 3 CW for 2 seconds —
//                            compare the resulting angle to what you'd expect
//                            from SERVO_MS_PER_180 to check calibration.
//   goto <ch> <deg>       -> same as a slider move: timed spin to an absolute
//                            angle (0-360) using the calibrated ms/degree.
//   sync <ch> <deg>       -> tell firmware "this channel is currently at
//                            <deg>°" without moving it (calibration anchor).
//   off <ch>              -> cut PWM to a channel (de-energize it).
//   help                  -> show this again.
//
// CALIBRATION WORKFLOW FOR A CHANNEL (say channel 3):
//   1. off 3, then physically hold/rest the horn at a known start point.
//   2. sync 3 0            (tell firmware "you are at 0 right now")
//   3. spin 3 4000 1       (spin CW for exactly 4000ms, i.e. the "180° time")
//   4. Measure the ACTUAL angle it landed at with a protractor/eye.
//        - Landed at ~180°? SERVO_MS_PER_180[3] = 4000 is already correct.
//        - Landed at less than 180°? That servo is slower -> increase
//          SERVO_MS_PER_180[3] (e.g. landed at 160° -> try 4000*180/160 ≈ 4500).
//        - Landed at more than 180°? That servo is faster -> decrease it
//          the same way (e.g. landed at 200° -> try 4000*180/200 = 3600).
//   5. Re-flash with the corrected value for that channel.

void printDiagHelp() {
  Serial.println(F("\n--- Serial diagnostics (360° timed-spin model) ---"));
  Serial.println(F("raw <ch> <us>        e.g. raw 3 1500"));
  Serial.println(F("spin <ch> <ms> <dir> e.g. spin 3 2000 1   (dir: 1=CW, -1=CCW)"));
  Serial.println(F("goto <ch> <deg>      e.g. goto 3 180      (0-360, timed move)"));
  Serial.println(F("sync <ch> <deg>      e.g. sync 3 0        (set position estimate, no move)"));
  Serial.println(F("off <ch>             e.g. off 3"));
  Serial.println(F("help"));
  Serial.println(F("---------------------------------------------------\n"));
}

void handleSerialDiag() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  int sp1 = line.indexOf(' ');
  String cmd = sp1 == -1 ? line : line.substring(0, sp1);

  if (cmd == "help") {
    printDiagHelp();

  } else if (cmd == "raw") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    int ch = line.substring(sp1 + 1, sp2).toInt();
    int us = line.substring(sp2 + 1).toInt();
    Serial.printf("raw: channel %d -> %d us\n", ch, us);
    pwm.writeMicroseconds(ch, us);

  } else if (cmd == "spin") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    int ch  = line.substring(sp1 + 1, sp2).toInt();
    uint32_t ms = (uint32_t) line.substring(sp2 + 1, sp3).toInt();
    int dir = line.substring(sp3 + 1).toInt();
    if (ch < 0 || ch >= NUM_SERVOS) { Serial.println(F("channel must be 0-14")); return; }
    dir = dir >= 0 ? 1 : -1;
    Serial.printf("spin: channel %d for %lums dir=%d\n", ch, (unsigned long)ms, dir);
    spinDir[ch] = dir;
    spinStartAngle[ch] = currentAngle[ch];
    spinStartMillis[ch] = millis();
    spinTotalMs[ch] = ms;
    spinEndMillis[ch] = spinStartMillis[ch] + ms;
    targetAngle[ch] = currentAngle[ch] + dir * (ms / SERVO_MS_PER_180[ch] * 180.0);
    spinning[ch] = true;
    servoActive[ch] = true;
    writeServoUS(ch, dir > 0 ? SERVO_MAX_US[ch] : SERVO_MIN_US[ch]);

  } else if (cmd == "goto") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    int ch = line.substring(sp1 + 1, sp2).toInt();
    float deg = line.substring(sp2 + 1).toFloat();
    if (ch < 0 || ch >= NUM_SERVOS) { Serial.println(F("channel must be 0-14")); return; }
    Serial.printf("goto: channel %d -> %.1f deg\n", ch, deg);
    startSpin(ch, deg);

  } else if (cmd == "sync") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    int ch = line.substring(sp1 + 1, sp2).toInt();
    float deg = line.substring(sp2 + 1).toFloat();
    if (ch < 0 || ch >= NUM_SERVOS) { Serial.println(F("channel must be 0-14")); return; }
    Serial.printf("sync: channel %d set to %.1f deg (no move)\n", ch, deg);
    syncServo(ch, deg);

  } else if (cmd == "off") {
    int ch = line.substring(sp1 + 1).toInt();
    if (ch < 0 || ch >= NUM_SERVOS) { Serial.println(F("channel must be 0-14")); return; }
    Serial.printf("off: channel %d\n", ch);
    pwm.setPWM(ch, 0, 0);
    spinning[ch] = false;
    spinDir[ch] = 0;

  } else {
    Serial.println(F("unknown command, type 'help'"));
  }
}

// ============================== HELPERS ==============================

void writeServoUS(int ch, int us) {
  if (ch < 0 || ch >= NUM_SERVOS) return;
  pwm.writeMicroseconds(ch, us);
}

// Start (or retarget) a timed, non-blocking spin toward an absolute angle
// (0-360, measured from wherever this channel was last synced/zeroed).
// Uses SERVO_MS_PER_180[idx] to convert the degree delta into a spin
// duration:  ms = |target - current| * (SERVO_MS_PER_180 / 180)
void startSpin(int idx, float targetDeg) {
  if (idx < 0 || idx >= NUM_SERVOS || eStop) return;
  targetDeg = constrain(targetDeg, 0, 360);

  float diff = targetDeg - currentAngle[idx];
  if (fabs(diff) < 0.5) {
    // Already there (within rounding) -> just make sure it's stopped.
    stopSpinChannel(idx);
    currentAngle[idx] = targetDeg;
    targetAngle[idx]  = targetDeg;
    return;
  }

  float msPerDeg = SERVO_MS_PER_180[idx] / 180.0;
  uint32_t moveMs = (uint32_t) round(fabs(diff) * msPerDeg);

  spinDir[idx]         = (diff > 0) ? 1 : -1;
  spinStartAngle[idx]  = currentAngle[idx];
  spinStartMillis[idx] = millis();
  spinTotalMs[idx]     = moveMs;
  spinEndMillis[idx]   = spinStartMillis[idx] + moveMs;
  targetAngle[idx]     = targetDeg;
  spinning[idx]        = true;
  servoActive[idx]     = true;

  writeServoUS(idx, spinDir[idx] > 0 ? SERVO_MAX_US[idx] : SERVO_MIN_US[idx]);
  dirty = true;
}

// Stop a channel's PWM immediately (de-energizes it — no feedback/holding
// torque is meaningful for a continuous-rotation servo once it's stopped).
void stopSpinChannel(int idx) {
  if (idx < 0 || idx >= NUM_SERVOS) return;
  pwm.setPWM(idx, 0, 0);
  spinning[idx] = false;
  spinDir[idx]  = 0;
}

// Tell the firmware "this channel is physically at <angle> right now"
// without commanding any movement. Use after manually repositioning a horn,
// or right after flashing (once you've physically centered everything).
void syncServo(int idx, float angle) {
  if (idx < 0 || idx >= NUM_SERVOS) return;
  angle = constrain(angle, 0, 360);
  stopSpinChannel(idx);
  currentAngle[idx] = angle;
  targetAngle[idx]  = angle;
  servoActive[idx]  = true;
  dirty = true;
}

void syncAllServos(float angle) {
  for (int i = 0; i < NUM_SERVOS; i++) syncServo(i, angle);
}

void broadcastPoseList() {
  StaticJsonDocument<256> doc;
  doc["type"] = "poseList";
  JsonArray arr = doc.createNestedArray("slots");
  for (int i = 0; i < NUM_POSE_SLOTS; i++) arr.add(poseSlotUsed[i]);
  char buf[256];
  size_t n = serializeJson(doc, buf);
  ws.textAll(buf, n);
}

void refreshPoseOccupancy() {
  prefs.begin("poses", true); // read-only
  for (int i = 0; i < NUM_POSE_SLOTS; i++) {
    String key = "pose" + String(i);
    poseSlotUsed[i] = prefs.isKey(key.c_str());
  }
  prefs.end();
}

void savePose(int slot) {
  if (slot < 0 || slot >= NUM_POSE_SLOTS) return;
  PoseData d;
  d.used = true;
  for (int i = 0; i < NUM_SERVOS; i++) d.angles[i] = (uint16_t)round(currentAngle[i]);

  prefs.begin("poses", false);
  String key = "pose" + String(slot);
  prefs.putBytes(key.c_str(), &d, sizeof(d));
  prefs.end();

  poseSlotUsed[slot] = true;
  broadcastPoseList();
}

void loadPose(int slot) {
  if (slot < 0 || slot >= NUM_POSE_SLOTS || !poseSlotUsed[slot]) return;
  PoseData d;
  prefs.begin("poses", true);
  String key = "pose" + String(slot);
  size_t got = prefs.getBytes(key.c_str(), &d, sizeof(d));
  prefs.end();

  if (got != sizeof(d) || !d.used) return;
  for (int i = 0; i < NUM_SERVOS; i++) startSpin(i, d.angles[i]);
}

void clearPose(int slot) {
  if (slot < 0 || slot >= NUM_POSE_SLOTS) return;
  prefs.begin("poses", false);
  String key = "pose" + String(slot);
  prefs.remove(key.c_str());
  prefs.end();
  poseSlotUsed[slot] = false;
  broadcastPoseList();
}

void engageEstop() {
  eStop = true;
  uint32_t now = millis();
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (spinning[i]) {
      // Best-effort: freeze the position estimate at wherever it got to.
      float frac = spinTotalMs[i] > 0
        ? constrain((float)(now - spinStartMillis[i]) / (float)spinTotalMs[i], 0.0, 1.0)
        : 1.0;
      currentAngle[i] = spinStartAngle[i] + spinDir[i] * frac * fabs(targetAngle[i] - spinStartAngle[i]);
    }
    stopSpinChannel(i); // pwm.setPWM(i,0,0)
  }
  broadcastStatus();
  broadcastState();
}

void resumeFromEstop() {
  eStop = false;
  // Nothing to re-assert: a continuous-rotation servo has no static "hold
  // this position" pulse, so channels simply stay stopped until the next
  // slider move / goto / pose load.
  broadcastStatus();
}

// ============================== WEBSOCKET HANDLER ==============================

void handleWsMessage(uint8_t* data, size_t len) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) return;

  const char* type = doc["type"] | "";

  if (strcmp(type, "servo") == 0) {
    int id = doc["id"] | 0;          // 1-15 from UI
    float angle = doc["angle"] | 0;
    if (id >= 1 && id <= NUM_SERVOS && !eStop) {
      startSpin(id - 1, angle);
    }
  } else if (strcmp(type, "sync") == 0) {
    int id = doc["id"] | 0;
    float angle = doc["angle"] | 0;
    if (id >= 1 && id <= NUM_SERVOS) syncServo(id - 1, angle);
    broadcastState();
  } else if (strcmp(type, "syncAll") == 0) {
    float angle = doc["angle"] | 0;
    syncAllServos(angle);
    broadcastState();
  } else if (strcmp(type, "centerAll") == 0) {
    if (!eStop) for (int i = 0; i < NUM_SERVOS; i++) startSpin(i, 180);
  } else if (strcmp(type, "home") == 0) {
    if (!eStop) for (int i = 0; i < NUM_SERVOS; i++) startSpin(i, HOME_ANGLES[i]);
  } else if (strcmp(type, "estop") == 0) {
    engageEstop();
  } else if (strcmp(type, "resume") == 0) {
    resumeFromEstop();
  } else if (strcmp(type, "savePose") == 0) {
    int slot = (int)(doc["slot"] | 1) - 1;
    savePose(slot);
  } else if (strcmp(type, "loadPose") == 0) {
    int slot = (int)(doc["slot"] | 1) - 1;
    if (!eStop) loadPose(slot);
  } else if (strcmp(type, "clearPose") == 0) {
    int slot = (int)(doc["slot"] | 1) - 1;
    clearPose(slot);
  }
}

void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WS client #%u connected\n", client->id());
      broadcastState();
      broadcastStatus();
      broadcastPoseList();
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WS client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len &&
          info->opcode == WS_TEXT) {
        handleWsMessage(data, len);
      }
      break;
    }
    default:
      break;
  }
}

void broadcastState() {
  StaticJsonDocument<896> doc;
  doc["type"] = "state";
  JsonArray arr = doc.createNestedArray("angles");
  for (int i = 0; i < NUM_SERVOS; i++) arr.add(currentAngle[i]);
  JsonArray act = doc.createNestedArray("active");
  for (int i = 0; i < NUM_SERVOS; i++) act.add(servoActive[i]);
  JsonArray spin = doc.createNestedArray("spinning");
  for (int i = 0; i < NUM_SERVOS; i++) spin.add(spinning[i]);
  char buf[896];
  size_t n = serializeJson(doc, buf);
  ws.textAll(buf, n);
}

void broadcastStatus() {
  StaticJsonDocument<128> doc;
  doc["type"] = "status";
  doc["estop"] = eStop;
  char buf[128];
  size_t n = serializeJson(doc, buf);
  ws.textAll(buf, n);
}

// ============================== SETUP / LOOP ==============================

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(21, 22);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // standard analog/continuous-rotation servo frequency

  for (int i = 0; i < NUM_SERVOS; i++) {
    currentAngle[i]   = 0;   // assumed reference position at boot -- SYNC after
                              // physically centering, don't trust this blindly
    targetAngle[i]    = 0;
    servoActive[i]    = false;
    spinning[i]       = false;
    spinDir[i]        = 0;
    spinStartAngle[i] = 0;
    spinStartMillis[i]= 0;
    spinTotalMs[i]    = 0;
    spinEndMillis[i]  = 0;
  }
  // NOTE: we deliberately do NOT call writeServoUS() here. The PCA9685 leaves
  // every channel with no PWM output until pwm.writeMicroseconds()/setPWM()
  // is called for that specific channel, so nothing spins at boot.

  refreshPoseOccupancy();

  // ---------------- WIFI: ACCESS POINT MODE (default) ----------------
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP started. Connect to WiFi \"");
  Serial.print(AP_SSID);
  Serial.println("\" then open:");
  Serial.print("http://");
  Serial.println(WiFi.softAPIP());

  // ---------------- WIFI: STATION MODE (alternative) ----------------
  // const char* STA_SSID = "your-wifi-name";
  // const char* STA_PASS = "your-wifi-password";
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(STA_SSID, STA_PASS);
  // while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  // Serial.println();
  // Serial.print("Connected! Open: http://");
  // Serial.println(WiFi.localIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
  Serial.println("HTTP + WebSocket server started.");
  printDiagHelp();
}

void loop() {
  ws.cleanupClients();
  handleSerialDiag();

  uint32_t now = millis();
  if (now - lastTick >= TICK_MS) {
    bool anySpinning = false;

    if (!eStop) {
      for (int i = 0; i < NUM_SERVOS; i++) {
        if (!spinning[i]) continue;

        if (now >= spinEndMillis[i]) {
          // Spin complete -> land exactly on target and stop.
          currentAngle[i] = targetAngle[i];
          stopSpinChannel(i);
        } else {
          // Still spinning -> update the live position ESTIMATE for the UI
          // (this does not change the actual PWM output, which is already
          // pinned at full-speed MIN/MAX_US for the whole duration).
          float frac = (float)(now - spinStartMillis[i]) / (float)spinTotalMs[i];
          currentAngle[i] = spinStartAngle[i] + spinDir[i] * frac * fabs(targetAngle[i] - spinStartAngle[i]);
          anySpinning = true;
        }
      }
    }
    dirty = dirty || anySpinning;
    lastTick = now;
  }

  if (dirty && now - lastBroadcast >= BROADCAST_MS) {
    broadcastState();
    lastBroadcast = now;
    // Keep broadcasting only while something is actually still moving.
    bool stillSpinning = false;
    for (int i = 0; i < NUM_SERVOS; i++) if (spinning[i]) { stillSpinning = true; break; }
    dirty = stillSpinning;
  }
}
