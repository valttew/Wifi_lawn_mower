// ESP32-C6-DevKitM-1
// Throttle: GPIO9 -> RC filter -> KT throttle signal (8-bit analogWrite @ 25kHz)
// Stepper:  A4988 STEP=GPIO2, DIR=GPIO3, EN=GPIO5 (ACTIVE-LOW)
// Wi-Fi SoftAP + simple WS UI (forward-only throttle + hold-to-steer)

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include "esp_timer.h"

// ---------- Wi-Fi ----------
static const char* AP_SSID = "MowerRC";
static const char* AP_PASS = "mower1234";

// ---------- Pins ----------
constexpr int THROTTLE_PWM_PIN = 9;  // to RC node -> KT throttle signal
constexpr int STEPPER_STEP_PIN = 2;
constexpr int STEPPER_DIR_PIN  = 3;
constexpr int STEPPER_EN_PIN   = 5;  // A4988 EN (ACTIVE-LOW)
constexpr int RGB_PIN          = 8;  // onboard NeoPixel

// ---------- RGB ----------
Adafruit_NeoPixel rgb(1, RGB_PIN, NEO_GRB + NEO_KHZ800);
inline void rgbSet(uint8_t r, uint8_t g, uint8_t b){ rgb.setPixelColor(0, rgb.Color(r,g,b)); rgb.show(); }
inline void rgbClear(){ rgb.clear(); rgb.show(); }

// ---------- Throttle ----------
constexpr uint32_t FREQ_THROTTLE = 25000; // 25 kHz
constexpr float THR_MIN_V = 1.00f;
constexpr float THR_MAX_V = 2.40f;

// ---------- Steering profile ---------
constexpr float    START_SPS       = 6.0f;
constexpr float    MAX_SPS         = 100.0f;

constexpr float    ACCEL_STAGE1    = 100.0f; // 0 -> 25 sps
constexpr float    ACCEL_STAGE2    = 120.0f; // 25 -> 70 sps
constexpr float    ACCEL_STAGE3    =  80.0f; // 70 -> 100 sps

constexpr float    STAGE1_END_SPS  = 25.0f;
constexpr float    STAGE2_END_SPS  = 70.0f;

constexpr float    ACCEL_DOWN      = 6000.0f; // fast stop
constexpr uint32_t PRESS_TAIL_MS   = 100;     // merge quick taps of buttons

// Breakaway / EN hold
constexpr uint32_t BREAKAWAY_MS    = 300;     // nudge duration
constexpr float    BREAKAWAY_SPS   = 30.0f;   // slow initial steps
constexpr uint32_t HOLD_ENABLE_MS  = 400;     // keep EN low after release

// ---------- ISR state ---------
static esp_timer_handle_t stepTimer = nullptr;
volatile int   isr_dir = 0;                // -1,0,+1
volatile float isr_current_sps = 0.0f;
volatile float isr_target_sps  = 0.0f;
volatile bool  isr_step_level  = false;
volatile int64_t isr_last_us   = 0;
volatile bool  isr_running     = false;
volatile uint32_t isr_enable_guard_ms = 0;

volatile uint32_t breakaway_until_ms = 0;

// ---------- Server / WS --------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------- Commands / state ---------
struct Cmd { float throttle=0.0f; int steer=0; uint32_t last_ms=0; } cmd;
float    throttle_cached = 0.0f;
bool     stepper_enabled = false;
uint32_t last_press_ms   = 0;
int      last_press_dir  = 0;

// ---------- Fwds ----------
void startSteppingIfNeeded();
void IRAM_ATTR stepTimerISR(void*);

// ---------- Throttle helpers ----------
inline void analogWrite8(int pin, uint8_t duty8){ analogWrite(pin, duty8); }

void setThrottleVoltage(float outV){
  if (outV < 0) outV = 0; if (outV > 3.3f) outV = 3.3f;
  float duty = outV / 3.3f;
  uint8_t duty8 = (uint8_t)roundf(duty * 255.0f); // 0..255
  analogWrite8(THROTTLE_PWM_PIN, duty8);
}

void setThrottle(float t){
  t = constrain(t, 0.0f, 1.0f);
  float outV = THR_MIN_V + t * (THR_MAX_V - THR_MIN_V);
  setThrottleVoltage(outV);
}

// ---------- Stepper helpers ----------
inline void stepperEnable(bool en){
  digitalWrite(STEPPER_EN_PIN, en ? LOW : HIGH); // ACTIVE-LOW
  stepper_enabled = en;
  if (en) isr_enable_guard_ms = millis() + 15;   // small settle before 1st step
  else { isr_step_level=false; digitalWrite(STEPPER_STEP_PIN, LOW); }
}
inline void stepperSetDir(int dir){
  digitalWrite(STEPPER_DIR_PIN, (dir>=0)?HIGH:LOW);
  delayMicroseconds(5);
  isr_dir = dir;
}
void IRAM_ATTR stopStepTimer(){ esp_timer_stop(stepTimer); isr_running=false; }
void IRAM_ATTR scheduleNextToggle(int64_t half_us){
  if (half_us<2) half_us=2;
  esp_timer_stop(stepTimer);
  esp_timer_start_once(stepTimer, half_us);
  isr_running = true;
}
inline float pickAccelUp(float sps){
  if (sps < STAGE1_END_SPS) return ACCEL_STAGE1;
  if (sps < STAGE2_END_SPS) return ACCEL_STAGE2;
  return ACCEL_STAGE3;
}
void IRAM_ATTR stepTimerISR(void*){
  const int dir = isr_dir;
  const int64_t now = esp_timer_get_time();
  float current = isr_current_sps;
  const float target = isr_target_sps;
  const float dt = (isr_last_us==0)?0.0f:(now - isr_last_us)*1e-6f;
  isr_last_us = now;

  if (dt>0.0f){
    if (current < target){
      current = fminf(current + pickAccelUp(current)*dt, target);
    }else if (current > target){
      current = fmaxf(current - ACCEL_DOWN*dt, target);
    }
  }

  if (dir==0 || target<=0.0f){
    isr_current_sps=0.0f;
    isr_step_level=false; digitalWrite(STEPPER_STEP_PIN, LOW);
    stopStepTimer();
    return;
  }

  if (current < START_SPS) current = START_SPS;
  if (current > MAX_SPS)   current = MAX_SPS;
  isr_current_sps = current;

  isr_step_level = !isr_step_level;
  digitalWrite(STEPPER_STEP_PIN, isr_step_level?HIGH:LOW);

  int64_t half_us = (int64_t)(500000.0f/current);
  if (half_us<2) half_us=2;
  scheduleNextToggle(half_us);
}

void ensureStepTimerCreated(){
  if (stepTimer) return;
  esp_timer_create_args_t args = {};
  args.callback = &stepTimerISR;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "stepISR";
  esp_timer_create(&args, &stepTimer);
}

// ---------- WebSocket ----------
void onWs(AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType t, void*, uint8_t* data, size_t len){
  if (t!=WS_EVT_DATA) return;
  data[len]=0;
  JsonDocument j;
  if (!deserializeJson(j, (char*)data)){
    float thr = j["throttle"] | 0.0f;
    float left  = j["left"]   | 0.0f;
    float right = j["right"]  | 0.0f;
    if (thr<0) thr=0; if (thr>1) thr=1;
    throttle_cached = thr;
    cmd.steer = (right>0.5f)?+1:((left>0.5f)?-1:0);
    cmd.last_ms = millis();
  }
}

// ---------- UI ----------
static const char INDEX_HTML[] =
"<!doctype html><meta name=viewport content=\"width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no\">"
"<style>html,body{margin:20px;font-family:sans-serif;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;touch-action:none;overscroll-behavior:contain}"
".btn{font-size:22px;padding:14px 20px;margin:8px;border:1px solid #999;border-radius:10px;display:inline-block;background:#eee}.btn:active{background:#ddd}#th{width:260px}</style>"
"<h2>Leikkuri ohjain</h2><label>Nopeus</label><br>"
"<input id=th type=range min=0 max=100 value=0><span id=tv> 0%</span><br><br>"
"<div id=left class=btn role=button tabindex=0>Vasempaan</div>"
"<div id=right class=btn role=button tabindex=0>Oikeaan</div>"
"<script>"
"const ws=new WebSocket('ws://'+location.host+'/ws');let left=0,right=0,thr=0;"
"const th=document.getElementById('th'),tv=document.getElementById('tv');"
"th.addEventListener('input',()=>{thr=th.value/100;tv.textContent=' '+Math.round(thr*100)+'%';if(ws.readyState===1)ws.send(JSON.stringify({throttle:thr,left:left,right:right}));});"
"function send(){if(ws.readyState===1){ws.send(JSON.stringify({throttle:thr,left:left,right:right}));}}"
"setInterval(send,100);document.addEventListener('selectstart',e=>e.preventDefault(),{passive:false});document.addEventListener('contextmenu',e=>e.preventDefault());"
"function makeHold(id,setter){const el=document.getElementById(id);const down=e=>{e.preventDefault();if(e.setPointerCapture)el.setPointerCapture(e.pointerId);setter(1);if(ws.readyState===1)ws.send(JSON.stringify({throttle:thr,left:left,right:right}));};const up=e=>{e.preventDefault();setter(0);if(ws.readyState===1)ws.send(JSON.stringify({throttle:thr,left:left,right:right}));};"
"el.addEventListener('pointerdown',down,{passive:false});el.addEventListener('pointerup',up,{passive:false});el.addEventListener('pointercancel',up,{passive:false});el.addEventListener('pointerleave',up,{passive:false});"
"el.addEventListener('touchstart',down,{passive:false});el.addEventListener('touchend',up,{passive:false});el.addEventListener('touchcancel',up,{passive:false});}"
"makeHold('left',v=>left=v);makeHold('right',v=>right=v);"
"let wl=null;if('wakeLock' in navigator){const req=async()=>{try{wl=await navigator.wakeLock.request('screen');wl.addEventListener('release',()=>{wl=null;});}catch{}};document.addEventListener('visibilitychange',()=>{if(!document.hidden&&!wl)req();});req();}"
"</script>";

// ---------- Setup / Loop ----------
void startSteppingIfNeeded();

void setup(){
  // Throttle pin low ASAP
  pinMode(THROTTLE_PWM_PIN, OUTPUT);
  digitalWrite(THROTTLE_PWM_PIN, LOW);

  pinMode(STEPPER_DIR_PIN, OUTPUT);  digitalWrite(STEPPER_DIR_PIN, LOW);
  pinMode(STEPPER_STEP_PIN, OUTPUT); digitalWrite(STEPPER_STEP_PIN, LOW);
  pinMode(STEPPER_EN_PIN, OUTPUT);   digitalWrite(STEPPER_EN_PIN, HIGH); // disabled at boot

  rgb.begin(); rgb.setBrightness(60);
  rgbSet(255, 0, 255); delay(300); rgbClear();

  analogWriteFrequency(THROTTLE_PWM_PIN, FREQ_THROTTLE);
  setThrottle(throttle_cached); // idle ~1.0 V

  WiFi.persistent(false); WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASS, 6, false, 1);

  ws.onEvent(onWs);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r){ r->send(200, "text/html", INDEX_HTML); });
  server.begin();

  esp_timer_create_args_t args = {};
  args.callback = &stepTimerISR;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "stepISR";
  esp_timer_create(&args, &stepTimer);

  isr_dir=0; isr_current_sps=0.0f; isr_target_sps=0.0f; isr_last_us=0; isr_step_level=false; isr_running=false;
}

void startSteppingIfNeeded(){
  if (!isr_running){
    if ((int32_t)(millis()-isr_enable_guard_ms) < 0) return;
    isr_last_us=0; isr_step_level=false; digitalWrite(STEPPER_STEP_PIN, LOW);
    isr_current_sps = START_SPS;
    int64_t half_us = (int64_t)(500000.0f/isr_current_sps);
    if (half_us<2) half_us=2;
    scheduleNextToggle(half_us);
  }
}

void loop(){
  const uint32_t now_ms = millis();

  // Always drive last-known throttle
  setThrottle(throttle_cached);

  // Merge quick taps
  const bool packet_fresh = ((now_ms - cmd.last_ms) <= 300);
  const bool pressing_now = packet_fresh && (cmd.steer != 0);
  if (pressing_now){ last_press_ms = now_ms; last_press_dir = cmd.steer; }
  const bool within_tail = (now_ms - last_press_ms) <= PRESS_TAIL_MS;
  const int  desired_dir = pressing_now ? cmd.steer : (within_tail ? last_press_dir : 0);

  static uint32_t en_off_at = 0;

  if (desired_dir != 0){
    if (!stepper_enabled) stepperEnable(true);

    if (!isr_running){
      if (isr_dir != desired_dir) stepperSetDir(desired_dir);
      breakaway_until_ms = now_ms + BREAKAWAY_MS;
      isr_target_sps = BREAKAWAY_SPS;
      startSteppingIfNeeded();
      en_off_at = 0; // cancel pending disable
    }else{
      // During breakaway window, hold target low; afterwards, ramp to MAX
      if ((int32_t)(now_ms - breakaway_until_ms) <= 0){
        isr_target_sps = BREAKAWAY_SPS;
      }else{
        isr_target_sps = MAX_SPS;
      }
    }
  }else{
    // No command: ramp down and disable after a short hold
    isr_target_sps = 0.0f;

    if (!within_tail){
      if (en_off_at == 0) en_off_at = now_ms + HOLD_ENABLE_MS;
      if ((int32_t)(now_ms - en_off_at) >= 0){
        en_off_at = 0;
        isr_dir = 0;
        stopStepTimer();
        stepperEnable(false);
      }
    }
  }

  // RGB: steer left/blue right/yellow throttle for debug.
  if      (desired_dir == -1) rgbSet(0,255,0);
  else if (desired_dir ==  1) rgbSet(0,0,255);
  else if (throttle_cached > 0.02f) rgbSet(255,160,0);
  else rgbClear();

  ws.cleanupClients();
}
