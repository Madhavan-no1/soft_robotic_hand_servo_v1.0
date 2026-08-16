/*
  ESP32 + PCA9685 - 15 x MG90S 360-degree continuous-rotation servos
  -------------------------------------------------------------------
  Each servo is controlled like a PS2-style spring-centered joystick.

    Slider 0..89   : anticlockwise at one CONSTANT slow speed
    Slider 90      : STOP
    Slider 91..180 : clockwise at one CONSTANT slow speed

  The slider is NOT a speed control. Moving farther from the center does not
  increase speed. Releasing the pointer/touch returns it to 90 and stops.

  Required libraries:
    - Adafruit PWM Servo Driver Library
    - ESP Async WebServer 3.12.0 (author: ESP32Async)
    - Async TCP 3.5.0 (author: ESP32Async)
    - ArduinoJson 6.x

  Verified compatible with:
    - ESP32 by Espressif Systems board package 3.3.11
    - ESP32-S3 (QFN56, 8 MB PSRAM)

  Do not use the old lacamera ESPAsyncWebServer 3.1.0 or dvarrel AsyncTCP
  1.1.4 with ESP32 Core 3.x.
*/

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

const char* AP_SSID = "ESP32-ServoControl";
const char* AP_PASS = "servo1234";

const uint8_t NUM_SERVOS = 15;
const uint8_t PCA_ADDR = 0x40;

// The ONLY two motion pulses. These are the last known working values for
// this servo setup. The slider distance never changes these values.
int SERVO_CCW_US[NUM_SERVOS] = {
  1300,1300,1300,1300,1300,
  1300,1300,1300,1300,1300,
  1300,1300,1300,1300,1300
};
int SERVO_CW_US[NUM_SERVOS] = {
  1460,1600,1600,1600,1600,
  1600,1600,1600,1600,1600,
  1600,1600,1600,1600,1600
};

const int JOY_MIN = 0;
const int JOY_CENTER = 90;
const int JOY_MAX = 180;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA_ADDR);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool servoMoving[NUM_SERVOS] = {false};
int joystickValue[NUM_SERVOS] = {
  90,90,90,90,90,90,90,90,90,90,90,90,90,90,90
};
bool eStop = false;
uint32_t lastBroadcast = 0;
// The controls are sent immediately.  A slower dashboard mirror avoids
// needlessly saturating the Wi-Fi/WebSocket connection on page reloads.
const uint32_t STATE_BROADCAST_MS = 200;
const uint32_t AP_HEALTH_CHECK_MS = 5000;
uint32_t lastApHealthCheck = 0;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>ESP32 15 Servo Joystick</title>
<style>
:root{--bg:#0e1016;--card:#1b1f2b;--card2:#20263a;--line:#2a3040;--text:#e9edf5;--muted:#8a92a6;--blue:#4f8cff;--purple:#8b5cf6;--green:#33d17a;--red:#ff4d5e;--rev:#ff7a59;--fwd:#4fd1ff;}
*{box-sizing:border-box}
body{margin:0;padding:22px 16px 60px;background:radial-gradient(circle at top,#171b26,var(--bg));color:var(--text);font-family:Segoe UI,Arial,sans-serif;user-select:none}
header{text-align:center;margin-bottom:6px}h1{margin:0;font-size:1.6rem;background:linear-gradient(90deg,var(--blue),var(--purple));-webkit-background-clip:text;color:transparent}.sub{text-align:center;color:var(--muted);font-size:.78rem;margin:7px 0 16px}
.status{display:flex;justify-content:center;align-items:center;gap:8px;color:var(--muted);font-size:.85rem;margin:8px 0 18px}.dot{width:9px;height:9px;border-radius:50%;background:var(--red);animation:pulse 1.6s infinite}.dot.ok{background:var(--green)}@keyframes pulse{70%{box-shadow:0 0 0 8px transparent}}
.toolbar{display:flex;justify-content:center;gap:10px;flex-wrap:wrap;margin-bottom:26px}.btn{border:1px solid var(--line);background:var(--card2);color:var(--text);padding:10px 18px;border-radius:10px;font-weight:700;cursor:pointer}.btn:hover{border-color:var(--blue);color:var(--blue)}.btn.stop{border-color:var(--red);color:var(--red)}
#estop{display:none;max-width:680px;margin:0 auto 20px;padding:10px;text-align:center;border:1px solid var(--red);border-radius:12px;color:#ffd8dd;background:#4a1218;font-weight:700}
.grid{max-width:1200px;margin:auto;display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:14px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:15px 18px 14px;box-shadow:0 4px 14px #0004;transition:.15s}.card.locked{opacity:.45;pointer-events:none}.card.rev{border-color:var(--rev)}.card.fwd{border-color:var(--fwd)}
.head{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}.name{font-weight:700}.tag{font-size:.58rem;color:var(--muted);background:#2a3145;border-radius:6px;padding:2px 6px;margin-left:5px}.tag.rev{background:#3a1f18;color:var(--rev)}.tag.fwd{background:#123246;color:var(--fwd)}.tag.stop{background:#123321;color:var(--green)}.value{font-size:.78rem;color:var(--blue);background:#12141c;border:1px solid var(--line);padding:4px 8px;border-radius:7px;min-width:88px;text-align:center}
.joy{display:block}.side{display:none}.track{position:relative;padding-top:4px}.track:after{content:"";position:absolute;left:50%;top:0;width:2px;height:8px;transform:translateX(-1px);background:var(--muted);pointer-events:none}
input[type=range]{width:100%;height:8px;-webkit-appearance:none;appearance:none;border-radius:5px;background:linear-gradient(90deg,var(--rev) 0 48%,#33384a 48% 52%,var(--fwd) 52% 100%);outline:0;touch-action:none;cursor:grab}input[type=range]:active{cursor:grabbing}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:22px;height:22px;border-radius:50%;background:#fff;border:3px solid var(--blue);box-shadow:0 2px 8px #0008}input[type=range]::-moz-range-thumb{width:22px;height:22px;border-radius:50%;background:#fff;border:3px solid var(--blue)}
.scale{display:flex;justify-content:space-between;color:var(--muted);font-size:.62rem;margin-top:5px}.hint{text-align:center;color:var(--muted);font-size:.65rem;margin-top:9px}
footer{text-align:center;color:var(--muted);font-size:.75rem;margin-top:36px}
</style></head>
<body>
<header><h1>ESP32 · PCA9685 · 15 Servo Controller</h1><div class="sub">PS2-style spring joystick · constant slow speed · release to stop</div></header>
<div class="status"><span id="dot" class="dot"></span><span id="status">Connecting…</span></div>
<div id="estop">⛔ EMERGENCY STOP ACTIVE — all servos stopped</div>
<div class="toolbar"><button class="btn stop" onclick="stopAll()">⏹ STOP ALL</button><button class="btn" id="estopBtn" onclick="toggleEstop()">⛔ EMERGENCY STOP</button></div>
<div id="grid" class="grid"></div>
<footer>Left = anticlockwise · Right = clockwise · 90 = STOP · no variable speed control</footer>
<script>
const N=15, CENTER=90; let socket=null, reconnect=null, estop=false;
const grid=document.getElementById('grid'),dot=document.getElementById('dot'),statusEl=document.getElementById('status'),banner=document.getElementById('estop'),estopBtn=document.getElementById('estopBtn');
function label(i,v){const e=document.getElementById('v'+i); if(!e)return; e.textContent=v<CENTER?v+' · ANTICLOCKWISE':v>CENTER?v+' · CLOCKWISE':'90 · STOP';}
function paint(i,v){const c=document.getElementById('c'+i),p=document.getElementById('p'+i);if(!c||!p)return;c.classList.toggle('rev',v<CENTER);c.classList.toggle('fwd',v>CENTER);if(v<CENTER){p.textContent='REVERSE';p.className='tag rev'}else if(v>CENTER){p.textContent='FORWARD';p.className='tag fwd'}else{p.textContent='STOPPED';p.className='tag stop'}}
function send(o){if(socket&&socket.readyState===WebSocket.OPEN)socket.send(JSON.stringify(o));}
// Hot control path: compact plain text "servo,value", avoiding JSON parsing
// for every slider movement. Example: "7,100".
function sendJoy(i,v){label(i,v);paint(i,v);if(socket&&socket.readyState===WebSocket.OPEN)socket.send(i+','+v);}
function release(i){const s=document.getElementById('s'+i);s.value=CENTER;sendJoy(i,CENTER);}
for(let i=1;i<=N;i++){
 const c=document.createElement('div');c.className='card';c.id='c'+i;c.innerHTML=`<div class="head"><span class="name">Servo ${i}<span class="tag" id="p${i}">STOPPED</span></span><span class="value" id="v${i}">90 · STOP</span></div><div class="joy"><div class="track"><input id="s${i}" type="range" min="0" max="180" step="1" value="90"></div></div><div class="scale"><span>◀ REVERSE</span><span>90 STOP</span><span>FORWARD ▶</span></div><div class="hint">Hold left/right · release to stop and recenter</div>`;grid.appendChild(c);
 const s=c.querySelector('#s'+i);
 s.addEventListener('pointerdown',e=>{s.setPointerCapture(e.pointerId);sendJoy(i,+s.value)});
 s.addEventListener('input',()=>sendJoy(i,+s.value));
 s.addEventListener('pointerup',()=>release(i));
 s.addEventListener('pointercancel',()=>release(i));
 s.addEventListener('lostpointercapture',()=>{if(+s.value!==CENTER)release(i)});
 s.addEventListener('keydown',e=>{if(e.key==='ArrowLeft'||e.key==='ArrowRight'){e.preventDefault();s.value=e.key==='ArrowLeft'?80:100;sendJoy(i,+s.value)}});
 s.addEventListener('keyup',e=>{if(e.key==='ArrowLeft'||e.key==='ArrowRight'){e.preventDefault();release(i)}});
}
function setEstop(x){estop=x;banner.style.display=x?'block':'none';estopBtn.textContent=x?'✅ RESUME':'⛔ EMERGENCY STOP';for(let i=1;i<=N;i++)document.getElementById('c'+i).classList.toggle('locked',x)}
function toggleEstop(){send({type:estop?'resume':'estop'})}
function stopAll(){for(let i=1;i<=N;i++){document.getElementById('s'+i).value=CENTER;label(i,CENTER);paint(i,CENTER)}send({type:'stopall'})}
// If the browser loses focus while a slider is held, send the same STOP
// command as a pointer release.  This prevents a held command from surviving
// an app switch, browser minimisation, or an accidental touch interruption.
window.addEventListener('blur',stopAll);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll()});
function connect(){clearTimeout(reconnect);socket=new WebSocket('ws://'+location.host+'/ws');socket.onopen=()=>{dot.classList.add('ok');statusEl.textContent='Connected'};socket.onclose=()=>{dot.classList.remove('ok');statusEl.textContent='Disconnected — retrying…';reconnect=setTimeout(connect,1200)};socket.onerror=()=>socket.close();socket.onmessage=e=>{let m;try{m=JSON.parse(e.data)}catch(_){return}if(m.type==='status'){setEstop(!!m.estop)}else if(m.type==='state'&&m.joystick){m.joystick.forEach((v,k)=>{const i=k+1,s=document.getElementById('s'+i);if(document.activeElement!==s)s.value=v;label(i,v);paint(i,v)})}}}
connect();
</script></body></html>
)rawliteral";

void stopServo(uint8_t ch) {
  if (ch >= NUM_SERVOS) return;
  // A continuous-rotation servo has no guaranteed universal neutral pulse.
  // Disabling this PCA9685 output removes the drive command and is the same
  // proven stop method used by the supplied timed-spin reference sketch.
  pwm.setPWM(ch, 0, 0);
  servoMoving[ch] = false;
  joystickValue[ch] = JOY_CENTER;
}

void setJoystick(uint8_t ch, int value) {
  if (ch >= NUM_SERVOS) return;
  value = constrain(value, JOY_MIN, JOY_MAX);
  joystickValue[ch] = value;

  // Centre is the only STOP position.  The distance from centre is ignored:
  // this is a direction-only joystick, not a speed controller.
  if (eStop || value == JOY_CENTER) {
    stopServo(ch);
    return;
  }

  // map() is used only to select the direction.  It does NOT select speed.
  // 0..89 -> negative (CCW), 90 -> 0 (STOP), 91..180 -> positive (CW).
  int direction = map(value, JOY_MIN, JOY_MAX, -100, 100);

  // There are only two fixed pulses per channel: CCW and CW.
  pwm.writeMicroseconds(ch, direction < 0 ? SERVO_CCW_US[ch] : SERVO_CW_US[ch]);
  servoMoving[ch] = true;
}

void broadcastState() {
  StaticJsonDocument<512> doc;
  doc["type"] = "state";
  JsonArray joy = doc.createNestedArray("joystick");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) joy.add(joystickValue[i]);
  JsonArray moving = doc.createNestedArray("moving");
  for (uint8_t i = 0; i < NUM_SERVOS; i++) moving.add(servoMoving[i]);

  char buffer[512];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  ws.textAll(buffer, len);
}

void broadcastStatus() {
  StaticJsonDocument<64> doc;
  doc["type"] = "status";
  doc["estop"] = eStop;
  char buffer[64];
  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  ws.textAll(buffer, len);
}

void engageEstop() {
  eStop = true;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) stopServo(i);
  broadcastStatus();
  broadcastState();
}

void resumeFromEstop() {
  eStop = false;
  for (uint8_t i = 0; i < NUM_SERVOS; i++) stopServo(i);
  broadcastStatus();
  broadcastState();
}

void stopAllServos() {
  for (uint8_t i = 0; i < NUM_SERVOS; i++) stopServo(i);
}

// Low-latency joystick protocol: "id,value" (for example "7,100").
// It deliberately bypasses JSON parsing because this is the hot control path.
bool handleCompactJoystick(uint8_t* data, size_t len) {
  if (len == 0 || len >= 16 || data[0] < '0' || data[0] > '9') return false;

  char command[16];
  memcpy(command, data, len);
  command[len] = '\0';
  char* comma = strchr(command, ',');
  if (!comma) return false;
  *comma = '\0';

  char* idEnd = nullptr;
  char* valueEnd = nullptr;
  long id = strtol(command, &idEnd, 10);
  long value = strtol(comma + 1, &valueEnd, 10);
  if (*idEnd != '\0' || *valueEnd != '\0' || id < 1 || id > NUM_SERVOS) return true;

  setJoystick((uint8_t)(id - 1), (int)value); // PWM write occurs synchronously here.
  return true;
}

void handleWsMessage(uint8_t* data, size_t len) {
  if (handleCompactJoystick(data, len)) return;

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, data, len);
  if (error) return;

  const char* type = doc["type"] | "";

  if (strcmp(type, "stopall") == 0) {
    stopAllServos();
  } else if (strcmp(type, "estop") == 0) {
    engageEstop();
  } else if (strcmp(type, "resume") == 0) {
    resumeFromEstop();
  }
}

void onWsEvent(AsyncWebSocket* serverPtr, AsyncWebSocketClient* client,
               AwsEventType type, void* arg, uint8_t* data, size_t len) {
  (void)serverPtr;
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected\n", client->id());
      if (client->client()) client->client()->setNoDelay(true);
      broadcastState();
      broadcastStatus();
      break;

    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;

    case WS_EVT_DATA: {
      AwsFrameInfo* info = (AwsFrameInfo*)arg;
      if (info && info->final && info->index == 0 &&
          info->len == len && info->opcode == WS_TEXT) {
        handleWsMessage(data, len);
      }
      break;
    }

    default:
      break;
  }
}

// Non-blocking Serial calibration commands (115200 baud, newline ending):
//   jog <channel> <dir>  e.g. jog 3 -1   (channels are 0..14)
//   raw <channel> <us>   e.g. raw 3 1500
void processSerialLine(char* line) {
  int channel, value;
  if (sscanf(line, "jog %d %d", &channel, &value) == 2) {
    if (channel >= 0 && channel < NUM_SERVOS) {
      setJoystick(channel, value < 0 ? 80 : (value > 0 ? 100 : JOY_CENTER));
      Serial.printf("jog channel %d, direction %d\n", channel, value < 0 ? -1 : (value > 0 ? 1 : 0));
    }
  } else if (sscanf(line, "raw %d %d", &channel, &value) == 2) {
    if (channel >= 0 && channel < NUM_SERVOS) {
      pwm.writeMicroseconds(channel, value);
      Serial.printf("raw channel %d -> %d us\n", channel, value);
    }
  } else if (strcmp(line, "help") == 0) {
    Serial.println("jog <channel 0-14> <-1|0|1>  |  raw <channel 0-14> <us>");
  }
}

void handleSerialCommands() {
  static char line[40];
  static uint8_t used = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[used] = '\0';
      if (used) processSerialLine(line);
      used = 0;
    } else if (used < sizeof(line) - 1) {
      line[used++] = c;
    } else {
      used = 0; // discard an overlong command without blocking motor control
    }
  }
}

// Returns true only when the ESP32 is actively advertising its access point.
// It is safe to call again after a Wi-Fi-driver drop.
bool startAccessPoint(bool resetDriver) {
  if (resetDriver) {
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
  }

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  return WiFi.softAP(AP_SSID, AP_PASS, 1, false, 4);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // GPIO9 and GPIO10 are adjacent pins on this ESP32-S3 DevKit header.
  // I2C pin order: SDA, SCL.
  Wire.begin(9, 10);
  Wire.setClock(400000);  // Faster PCA9685 I2C updates.
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  // Start every continuous servo at neutral STOP.
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    stopServo(i);
  }

  bool apStarted = startAccessPoint(true);
  if (!apStarted) {
    Serial.println("ERROR: Wi-Fi AP start failed; retrying...");
    delay(250);
    apStarted = startAccessPoint(true);
  }

  Serial.println();
  Serial.println("========================================");
  Serial.println("ESP32 15-Servo Joystick Controller");
  Serial.print("WiFi SSID : ");
  Serial.println(AP_SSID);
  Serial.print("Password  : ");
  Serial.println(AP_PASS);
  if (apStarted) {
    Serial.print("Open      : http://");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("AP status : FAILED");
  }
  Serial.println("========================================");

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  ws.cleanupClients();
  handleSerialCommands();

  // A page reload closes only the WebSocket and is handled by the browser.
  // If the AP itself drops, restart its Wi-Fi driver and HTTP listener.
  uint32_t now = millis();
  if (now - lastApHealthCheck >= AP_HEALTH_CHECK_MS) {
    lastApHealthCheck = now;
    wifi_mode_t mode = WiFi.getMode();
    if ((mode != WIFI_AP && mode != WIFI_AP_STA) ||
        WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) {
      Serial.println("Wi-Fi AP lost; restarting access point...");
      server.end();
      if (startAccessPoint(true)) {
        server.begin();
        Serial.print("Wi-Fi AP restored at http://");
        Serial.println(WiFi.softAPIP());
      } else {
        Serial.println("ERROR: Wi-Fi AP recovery failed");
      }
    }
  }

  // Dashboard mirror only. Actuation is never delayed by this broadcast.
  if (now - lastBroadcast >= STATE_BROADCAST_MS) {
    lastBroadcast = now;
    broadcastState();
  }
}
