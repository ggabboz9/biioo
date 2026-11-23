#include <Arduino.h>
#include <TFT_eSPI.h>
#include <NimBLEDevice.h>
#include <Wire.h>
#include <Preferences.h>
#include <Update.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <esp_sleep.h>

// ---- T-Embed CC1101 board pins ----
#define BOARD_PWR_EN   15
#define BOARD_SD_CS    13
#define BOARD_LORA_CS  12
#define DISPLAY_BL     21
#define ENCODER_INA    4
#define ENCODER_INB    5
#define ENCODER_KEY    0
#define BOARD_USER_KEY 6          // the side button: hold it to turn the device off

// ---- Real analog fuel input ----
// Use an ADC pin that is exposed on the header (GPIO 1-10 preferred). NOT 16 (display DC).
// Max ~3.1 V. Never connect 12 V directly.
#define FUEL_ADC_PIN   2          // <-- CHANGE to the pin you actually wire
const int EMPTY_MV = 0;           // mV measured at empty
const int FULL_MV  = 3100;        // mV measured at full

// ---- Range (autonomy) estimate shown when no phone is connected ----
// PLACEHOLDERS: set these to your real tank size and average consumption.
#define TANK_LITERS    8.0f       // fuel tank capacity in liters
#define KM_PER_LITER   30.0f      // average km per liter
#define RESERVE_LITERS 0.0f       // liters you never count on (reserve)

// ---- Battery (BQ27220 fuel gauge on the I2C bus) ----
#define BOARD_I2C_SDA  8
#define BOARD_I2C_SCL  18
#define BQ27220_ADDR   0x55
#define BQ25896_ADDR   0x6B      // charger chip (tells us if USB power is connected)

// ---- Heat / power savings ----
#define CPU_MHZ          160     // 240 is the default; 160 is plenty and runs cooler
#define BACKLIGHT_DUTY   170     // 0-255. Lower = cooler and less battery use (255 = full)
#define LIMIT_CHARGE_CURRENT 1   // 1 = cap the battery charge current (set 0 to leave the chip's default)
#define CHARGE_CURRENT_MA    500 // for a ~1300 mAh cell keep this at or below ~600 mA

// ---- Needle look: 0 = plain orange wedge, 1 = slim cyan pointer, 2 = classic Vespa (shadow + chrome hub)
#define NEEDLE_STYLE 2

// ---- Bluetooth link to the Android app (must match the app) ----
#define SERVICE_UUID "7a1e0001-5b1d-4f6a-9c53-6d2f0a9b1e01"
#define RX_UUID      "7a1e0002-5b1d-4f6a-9c53-6d2f0a9b1e01"   // phone -> board (text lines)
#define TX_UUID      "7a1e0003-5b1d-4f6a-9c53-6d2f0a9b1e01"   // board -> phone (commands)
#define OTA_UUID     "7a1e0004-5b1d-4f6a-9c53-6d2f0a9b1e01"   // phone -> board (firmware bytes)
#define FW_VERSION   "1.2.0"

// ---- Layout: emulates a ROUND screen on the rectangular test display ----
// Round screen = circle of diameter 170 centred at (160,85).
// Top half = fuel semicircle, bottom half = 3 text lines controlled by the phone.
#define GCX        160
#define GCY        85
#define R_SCREEN   85
#define R_OUT      78
#define R_IN       68
#define NEEDLE_LEN 60

// ---- Animation settings ----
const unsigned long SWEEP_MS = 750;      // startup: E->F, then F->E (each leg)
const unsigned long SLIDE_MS = 380;      // bottom-half slide transition
const int           SLIDE_W  = 170;      // slide distance in pixels
const unsigned long MARQUEE_PAUSE = 1200; // ms pause at each end of a scrolling title
const int           MARQUEE_SPEED = 30;   // pixels per second

// ---- Colors (RGB565; sprites are 8-bit internally to save RAM) ----
const uint16_t C_BG     = 0x0000;
const uint16_t C_DIM    = 0x2965;
const uint16_t C_GREY   = 0xBDF7;
uint16_t C_ACCENT = 0x065F;     // changed by the app's theme
uint16_t C_NEEDLE = 0xFD20;
int needleStyle = NEEDLE_STYLE;  // 0 wedge, 1 slim, 2 classic

TFT_eSPI tft;
TFT_eSprite spr(&tft);    // frame buffer
TFT_eSprite face(&tft);   // static background
bool spritesOk = false;
unsigned long bootT0 = 0;

// ---------- Fuel (moving average) ----------
const int NUM_READINGS = 15;
int  readings[NUM_READINGS];
int  readIndex = 0;
long total = 0;

// ---------- Rotary encoder = volume ----------
DRAM_ATTR static const int8_t ENC_TABLE[16] = {0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0};
volatile uint8_t encPrev = 0;
volatile int8_t  encAcc = 0;
volatile int     encSteps = 0;

void IRAM_ATTR encISR() {
  uint8_t s = (digitalRead(ENCODER_INA) << 1) | digitalRead(ENCODER_INB);
  encAcc += ENC_TABLE[(encPrev << 2) | s];
  encPrev = s;
  if (encAcc >= 4)       { encSteps++; encAcc = 0; }
  else if (encAcc <= -4) { encSteps--; encAcc = 0; }
}

// ---------- Text lines shown in the bottom half ----------
volatile bool phoneConnected = false;
int latestPercent = 0;

// Phone call state sent by the app: 0 = none, 1 = ringing, 2 = on a call
int callState = 0;
String callName;
unsigned long callStart = 0;

// The charger/gauge chips cannot tell whether a battery is connected while USB is plugged in
// and the battery is full. So: USB unplugged or charging proves one is fitted (remembered),
// otherwise we use this saved setting, which the phone app can change.
Preferences prefs;
bool battFitted = false;
bool forceBatRead = false;
bool showBattIcon = true;             // setting: show the battery icon on the dash

// Range learned by the phone app (km per 1% of fuel) and the backlight level it sets
float kmPerPercent = TANK_LITERS * KM_PER_LITER / 100.0f;   // placeholder estimate until the app has learned one
int backlightDuty = BACKLIGHT_DUTY;
int savedBrt = BACKLIGHT_DUTY;

// Turn-by-turn from Google Maps, sent by the phone app
bool navActive = false;
unsigned long navLastMsg = 0;
String navDist, navStreet;
char navDir = 'S';                    // 'S' straight, 'L' left, 'R' right (used when there is no icon)
uint8_t navIcon[128];                 // 32x32 maneuver icon, 1 bit per pixel
uint8_t navIconNew[128];
uint8_t navIconMask = 0;
bool navHasIcon = false;

// Firmware update over Bluetooth
bool otaActive = false;
size_t otaTotal = 0, otaWritten = 0, otaSinceAck = 0;
bool otaEndRequested = false;
unsigned long otaLastData = 0;
static uint8_t otaBuf[16384];
volatile size_t otaHead = 0, otaTail = 0;

// First-run setup, driven by the phone app. While not done, the dash shows its setup screens.
bool setupDone = false;               // saved on the dash
int  setupStep = 0;                   // from the app: 1 = linking, 2 = confirm on the dash, 3 = all set
unsigned long setupConfirmedAt = 0;   // when the button was pressed during step 2
bool appReady = false;                // the phone app has talked to us since it connected

// mode 0 = three text lines from the phone, mode 1 = range + "no phone" icon
struct Content { String l[3]; int mode = 0; int icon = 0; };   // mode 4 = icon screen (icon + two short lines)

String lines[3];
String actionText = "";
unsigned long actionUntil = 0;

void resetLines() {
  lines[0] = "VESPA ET4";
  lines[1] = "Waiting for phone";
  lines[2] = "";
}
void showAction(const String &t) { actionText = t; actionUntil = millis() + 1500; }

Content currentTarget() {
  Content c;
  if (otaActive) {                                     // firmware update in progress
    c.mode = 6;
    c.l[1] = String(otaTotal ? (int)(otaWritten * 100 / otaTotal) : 0) + "%";
    return c;
  }
  if (!setupDone) {                                    // first-run setup: the fuel gauge stays, the bottom half guides you
    c.mode = 4;
    if (!phoneConnected)       { c.icon = 2; c.l[0] = "Not set up";    c.l[1] = "Open the app"; }
    else if (setupStep >= 3)   { c.icon = 3; c.l[0] = "All set";       c.l[1] = "Tap Finish"; }
    else if (setupStep == 2) {
      if (setupConfirmedAt)    { c.icon = 3; c.l[0] = "Confirmed";     c.l[1] = "Use the app"; }
      else                     { c.icon = 1; c.l[0] = "Press button";  c.l[1] = "to continue"; }
    }
    else                       { c.icon = 1; c.l[0] = "Setting up";    c.l[1] = "Use the app"; }
    return c;
  }
  if (!phoneConnected) {
    c.mode = 1;
    c.l[2] = (millis() < actionUntil) ? actionText : String();
    return c;
  }
  if (!appReady) {                                     // connected, but the app has not sent anything yet
    c.mode = 4;
    c.icon = 1;
    c.l[0] = "Connecting";
    return c;
  }
  if (callState != 0) {
    c.l[0] = callName.length() ? callName : String("Unknown");
    if (callState == 1) {
      c.l[1] = "Click=answer";
      c.l[2] = "Hold=reject";
    } else {
      c.l[1] = "Hold=end call";
      unsigned long sec = (millis() - callStart) / 1000;
      char tb[12];
      snprintf(tb, sizeof(tb), "%02lu:%02lu", sec / 60, sec % 60);
      c.l[2] = tb;
    }
    return c;
  }
  if (navActive && millis() - navLastMsg < 60000) {      // turn-by-turn from Google Maps
    c.mode = 5;
    c.l[0] = navDist;
    c.l[1] = navStreet;
    return c;
  }
  c.l[0] = lines[0];
  c.l[1] = lines[1];
  c.l[2] = (millis() < actionUntil) ? actionText : lines[2];
  return c;
}

// ---------- BLE (NimBLE) ----------
NimBLECharacteristic *txChar = nullptr;

// Messages arrive on the BLE task; hand them to loop() through a small ring buffer
const int QN = 8;
String msgQ[QN];
volatile int qHead = 0, qTail = 0;
void pushMsg(const String &s) {
  int n = (qHead + 1) % QN;
  if (n == qTail) return;           // full: drop
  msgQ[qHead] = s;
  qHead = n;
}
bool popMsg(String &s) {
  if (qTail == qHead) return false;
  s = msgQ[qTail];
  qTail = (qTail + 1) % QN;
  return true;
}

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *s) { phoneConnected = true; }
  void onDisconnect(NimBLEServer *s) {
    phoneConnected = false;
    NimBLEDevice::startAdvertising();
  }
};

class RxCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) {
    std::string v = c->getValue();
    pushMsg(String(v.c_str()));
  }
};

class OtaCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *c) {
    if (!otaActive) return;
    std::string v = c->getValue();
    for (size_t i = 0; i < v.size(); i++) {
      size_t n = (otaHead + 1) % sizeof(otaBuf);
      if (n == otaTail) break;                       // full (the app's window prevents this)
      otaBuf[otaHead] = (uint8_t)v[i];
      otaHead = n;
    }
    otaLastData = millis();
  }
};

void startBle() {
  NimBLEDevice::init("Vespa Dash");
  NimBLEDevice::setMTU(185);
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  NimBLEService *svc = server->createService(SERVICE_UUID);
  NimBLECharacteristic *rx = svc->createCharacteristic(
      RX_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  rx->setCallbacks(new RxCB());
  txChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic *ota = svc->createCharacteristic(OTA_UUID, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  ota->setCallbacks(new OtaCB());
  svc->start();
  NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->start();
  Serial.println("BLE advertising as 'Vespa Dash'");
}

void sendCmd(const char *cmd, const char *label) {
  Serial.printf("CMD %s (phone %s)\n", cmd, phoneConnected ? "connected" : "NOT connected");
  if (!phoneConnected || !txChar) { showAction("NO PHONE"); return; }
  txChar->setValue((const uint8_t *)cmd, strlen(cmd));
  txChar->notify();
  showAction(label);
}

// The phone sends the three lines as separate messages. Stage them and apply
// all three together after a short quiet period, so the slide animation runs once.
String incoming[3];
bool incomingDirty = false;
unsigned long lastRxAt = 0;
const unsigned long SETTLE_MS = 250;

void notifyRaw(const char *s) {
  if (!phoneConnected || !txChar) return;
  txChar->setValue((const uint8_t *)s, strlen(s));
  txChar->notify();
}

void otaFail(const char *why) {
  Update.abort();
  otaActive = false;
  char b[32];
  snprintf(b, sizeof(b), "OTA:ERR:%s", why);
  notifyRaw(b);
}

// Writes the received bytes to flash (the other app slot, so a failed update never breaks the dash)
void otaService() {
  if (!otaActive) return;
  static uint8_t tmp[512];
  int guard = 0;
  while (otaTail != otaHead && guard++ < 16) {
    size_t n = 0;
    while (otaTail != otaHead && n < sizeof(tmp)) { tmp[n++] = otaBuf[otaTail]; otaTail = (otaTail + 1) % sizeof(otaBuf); }
    if (Update.write(tmp, n) != n) { otaFail("write"); return; }
    otaWritten += n;
    otaSinceAck += n;
  }
  if (otaSinceAck >= 4096) {
    otaSinceAck = 0;
    char b[32];
    snprintf(b, sizeof(b), "OTA:ACK:%u", (unsigned)otaWritten);
    notifyRaw(b);
  }
  if (otaWritten >= otaTotal) {
    if (Update.end(true)) {                          // checks the MD5 too
      notifyRaw("OTA:OK");
      delay(800);
      ESP.restart();
    } else {
      otaFail("verify");
    }
    return;
  }
  if (otaEndRequested && otaTail == otaHead) { otaFail("size"); return; }
  if (millis() - otaLastData > 15000) otaFail("timeout");
}

static uint8_t hexVal(char c) { return c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10; }

void handleMessage(const String &m) {
  Serial.println("RX: " + m);
  appReady = true;
  if (m.startsWith("SETUP:")) {
    if (m == "SETUP:START") {
      setupDone = false; setupStep = 0; setupConfirmedAt = 0;
      prefs.putBool("setup", false);
    } else if (m == "SETUP:DONE") {
      setupDone = true; setupStep = 0;
      prefs.putBool("setup", true);
    } else if (m.startsWith("SETUP:STEP:")) {
      setupStep = m.substring(11).toInt();
      if (setupStep != 2) setupConfirmedAt = 0;
    }
    return;
  }
  if (m.startsWith("OTA:")) {
    if (m.startsWith("OTA:BEGIN:")) {                // OTA:BEGIN:<size>:<md5>
      if (otaActive) Update.abort();
      int p = m.indexOf(':', 10);
      size_t size = (p > 0) ? m.substring(10, p).toInt() : 0;
      String md5 = (p > 0) ? m.substring(p + 1) : String("");
      if (size > 0 && Update.begin(size, U_FLASH)) {
        if (md5.length() == 32) Update.setMD5(md5.c_str());
        otaActive = true; otaTotal = size; otaWritten = 0; otaSinceAck = 0;
        otaHead = otaTail = 0; otaEndRequested = false; otaLastData = millis();
        notifyRaw("OTA:READY");
      } else {
        notifyRaw("OTA:ERR:begin");
      }
    } else if (m == "OTA:END") {
      otaEndRequested = true;
    } else if (m == "OTA:ABORT") {
      if (otaActive) Update.abort();
      otaActive = false;
    }
    return;
  }
  if (m.startsWith("CFG:THEME:")) {                // CFG:THEME:<style>:<accent565>:<needle565>
    int p1 = m.indexOf(':', 10);
    int p2 = (p1 > 0) ? m.indexOf(':', p1 + 1) : -1;
    if (p1 > 0 && p2 > 0) {
      needleStyle = constrain((int)m.substring(10, p1).toInt(), 0, 2);
      C_ACCENT = (uint16_t)strtol(m.substring(p1 + 1, p2).c_str(), nullptr, 16);
      C_NEEDLE = (uint16_t)strtol(m.substring(p2 + 1).c_str(), nullptr, 16);
      prefs.putInt("nstyle", needleStyle);
      prefs.putUInt("cacc", C_ACCENT);
      prefs.putUInt("cneed", C_NEEDLE);
    }
    return;
  }
  if (m.startsWith("CFG:KMPP:")) {
    int v = m.substring(9).toInt();
    if (v == 0) {                                   // back to the estimate
      kmPerPercent = TANK_LITERS * KM_PER_LITER / 100.0f;
      prefs.remove("kmpp");
    } else if (v >= 20 && v <= 1000) {
      kmPerPercent = v / 100.0f;
      prefs.putFloat("kmpp", kmPerPercent);
    }
    return;
  }
  if (m.startsWith("CFG:BRT:")) {
    int v = constrain((int)m.substring(8).toInt(), 10, 255);
    backlightDuty = v;
    ledcWrite(0, v);
    if (abs(v - savedBrt) >= 20) { savedBrt = v; prefs.putInt("brt", v); }
    return;
  }
  if (m.startsWith("NAV:")) {
    navLastMsg = millis();
    if (m == "NAV:END") {
      navActive = false; navHasIcon = false; navIconMask = 0;
    } else if (m.startsWith("NAV:D:")) {
      navDist = m.substring(6); navActive = true;
    } else if (m.startsWith("NAV:S:")) {
      navStreet = m.substring(6); navActive = true;
    } else if (m.startsWith("NAV:DIR:") && m.length() > 8) {
      navDir = m[8]; navHasIcon = false; navIconMask = 0; navActive = true;
    } else if ((m.startsWith("NAV:I0:") || m.startsWith("NAV:I1:")) && m.length() >= 7 + 128) {
      int part = m[5] - '0';
      for (int i = 0; i < 64; i++) navIconNew[part * 64 + i] = (hexVal(m[7 + 2 * i]) << 4) | hexVal(m[8 + 2 * i]);
      navIconMask = (part == 0) ? 1 : (navIconMask | 2);
      if (navIconMask == 3) { memcpy(navIcon, navIconNew, 128); navHasIcon = true; }
      navActive = true;
    }
    return;
  }
  if (m.startsWith("CFG:BATTICON:")) {
    showBattIcon = m.endsWith("1");
    prefs.putBool("baticon", showBattIcon);
    return;
  }
  if (m.startsWith("CFG:BATT:")) {
    battFitted = m.endsWith("1");
    prefs.putBool("batt", battFitted);
    forceBatRead = true;
    return;
  }
  if (m.startsWith("CALL:")) {
    if (m.startsWith("CALL:RING:")) {
      callState = 1;
      callName = m.substring(10);
    } else if (m.startsWith("CALL:ACTIVE:")) {
      if (callState != 2) callStart = millis();
      callState = 2;
      callName = m.substring(12);
    } else {
      callState = 0;
      callName = "";
    }
    return;
  }
  if (m.length() >= 3 && m[0] == 'L' && m[2] == ':' && m[1] >= '0' && m[1] <= '2') {
    if (!incomingDirty) {
      for (int i = 0; i < 3; i++) incoming[i] = lines[i];
    }
    incoming[m[1] - '0'] = m.substring(3);
    incomingDirty = true;
    lastRxAt = millis();
  }
}

void applyIncomingIfSettled() {
  if (incomingDirty && millis() - lastRxAt > SETTLE_MS) {
    for (int i = 0; i < 3; i++) lines[i] = incoming[i];
    incomingDirty = false;
  }
}

// ---------- Button: click = play/pause, double = next, hold = previous ----------
bool btnDown = false, longFired = false;
unsigned long btnDownAt = 0, lastClickAt = 0;
int clickCount = 0;

bool setupWaitingConfirm() {
  return !setupDone && phoneConnected && setupStep == 2 && setupConfirmedAt == 0;
}

void setupConfirm() {
  setupConfirmedAt = millis();
  notifyRaw("SETUP:CONFIRMED");
  Serial.println("Setup: confirmed on the dash");
}

void handleButton() {
  bool pressed = digitalRead(ENCODER_KEY) == LOW;
  unsigned long now = millis();
  if (pressed && !btnDown) {
    btnDown = true; btnDownAt = now; longFired = false;
  } else if (pressed && btnDown && !longFired && now - btnDownAt > 600) {
    longFired = true; clickCount = 0;
    if (!setupDone)          { /* no media controls during setup */ }
    else if (callState == 1) sendCmd("DECLINE", "REJECT");
    else if (callState == 2) sendCmd("HANGUP", "END CALL");
    else                     sendCmd("PREV", "PREVIOUS");
  } else if (!pressed && btnDown) {
    btnDown = false;
    if (!longFired && now - btnDownAt > 30) { clickCount++; lastClickAt = now; }
  }
  if (!btnDown && clickCount > 0 && now - lastClickAt > 350) {
    if (!setupDone) {
      if (setupWaitingConfirm()) setupConfirm();   // during setup any click confirms
    } else if (callState == 1) {
      if (clickCount == 1) sendCmd("ANSWER", "ANSWER");   // other click patterns are ignored while ringing
    } else if (callState == 2) {
      // clicks do nothing during a call, hold to hang up
    } else if (clickCount == 1) sendCmd("PLAYPAUSE", "PLAY/PAUSE");
    else                        sendCmd("NEXT", "NEXT");
    clickCount = 0;
  }
}

// ---------- Dial drawing ----------
// percent 0 = E (left, 180 deg), 100 = F (right, 0 deg)
float pctToRad(float p) { return (180.0f - p * 1.8f) * DEG_TO_RAD; }

float easeInOut(float t) {
  return t < 0.5f ? 2.0f * t * t : 1.0f - powf(-2.0f * t + 2.0f, 2.0f) / 2.0f;
}

void drawBand(TFT_eSprite &s, float p0, float p1, uint16_t color) {
  for (float p = p0; p < p1; p += 0.25f) {
    float p2 = p + 0.35f; if (p2 > p1) p2 = p1;
    float a1 = pctToRad(p), a2 = pctToRad(p2);
    int x1o = GCX + R_OUT * cosf(a1), y1o = GCY - R_OUT * sinf(a1);
    int x2o = GCX + R_OUT * cosf(a2), y2o = GCY - R_OUT * sinf(a2);
    int x1i = GCX + R_IN  * cosf(a1), y1i = GCY - R_IN  * sinf(a1);
    int x2i = GCX + R_IN  * cosf(a2), y2i = GCY - R_IN  * sinf(a2);
    s.fillTriangle(x1o, y1o, x2o, y2o, x1i, y1i, color);
    s.fillTriangle(x2o, y2o, x2i, y2i, x1i, y1i, color);
  }
}

void drawTick(TFT_eSprite &s, float p, int r1, int r2, bool major, uint16_t color) {
  float a = pctToRad(p), ca = cosf(a), sa = sinf(a);
  int x1 = GCX + r1 * ca, y1 = GCY - r1 * sa;
  int x2 = GCX + r2 * ca, y2 = GCY - r2 * sa;
  s.drawLine(x1, y1, x2, y2, color);
  if (major) {
    s.drawLine(x1 + 1, y1, x2 + 1, y2, color);
    s.drawLine(x1, y1 + 1, x2, y2 + 1, color);
  }
}

void buildFace() {
  face.fillSprite(C_BG);

  // Outline of the (future) round screen, so you can judge the layout
  face.drawCircle(GCX, GCY, R_SCREEN, C_DIM);

  // Fuel semicircle: red low zone, white the rest
  drawBand(face, 0, 15, TFT_RED);
  drawBand(face, 15, 100, TFT_WHITE);
  for (int i = 0; i <= 4; i++) drawTick(face, i * 25.0f, 66, 56, true, TFT_WHITE);
  for (int i = 0; i < 4; i++)  drawTick(face, 12.5f + i * 25.0f, 66, 60, false, C_GREY);

  face.setTextDatum(TC_DATUM);
  face.setTextColor(TFT_RED, C_BG);
  face.drawString("E", GCX - 73, GCY + 6, 2);
  face.setTextColor(TFT_WHITE, C_BG);
  face.drawString("F", GCX + 73, GCY + 6, 2);
}

// ---------- Needle ----------
void drawWedge(int cx, int cy, float ca, float sa, int len, int halfW, int tail, uint16_t col) {
  int tx = cx + len * ca, ty = cy - len * sa;
  int b1x = cx + halfW * sa, b1y = cy + halfW * ca;
  int b2x = cx - halfW * sa, b2y = cy - halfW * ca;
  int tailx = cx - tail * ca, taily = cy + tail * sa;
  spr.fillTriangle(tx, ty, b1x, b1y, b2x, b2y, col);
  spr.fillTriangle(tailx, taily, b1x, b1y, b2x, b2y, col);
}

void drawNeedle(float pct) {
  float a = pctToRad(pct), ca = cosf(a), sa = sinf(a);
  int tx = GCX + NEEDLE_LEN * ca, ty = GCY - NEEDLE_LEN * sa;

if (needleStyle == 0) {
  drawWedge(GCX, GCY, ca, sa, NEEDLE_LEN, 3, 10, C_NEEDLE);
  spr.fillCircle(GCX, GCY, 6, TFT_WHITE);
  spr.fillCircle(GCX, GCY, 2, C_BG);

} else if (needleStyle == 1) {
  int bx = GCX - 8 * ca, by = GCY + 8 * sa;
  spr.drawLine(bx, by, tx, ty, C_ACCENT);
  spr.drawLine(bx + 1, by, tx + 1, ty, C_ACCENT);
  spr.drawLine(bx, by + 1, tx, ty + 1, C_ACCENT);
  spr.fillCircle(tx, ty, 2, TFT_WHITE);
  spr.fillCircle(GCX, GCY, 5, TFT_WHITE);
  spr.fillCircle(GCX, GCY, 2, C_BG);

} else {
  // Classic: soft shadow, tapered body with a light stripe, round counterweight, chrome hub
  drawWedge(GCX + 2, GCY + 2, ca, sa, NEEDLE_LEN, 4, 12, 0x2104);       // shadow
  drawWedge(GCX, GCY, ca, sa, NEEDLE_LEN, 4, 12, C_NEEDLE);             // body
  int hx = GCX + (NEEDLE_LEN - 6) * ca, hy = GCY - (NEEDLE_LEN - 6) * sa;
  spr.drawLine(GCX, GCY, hx, hy, 0xFEA0);                                // highlight
  spr.fillCircle(GCX - 14 * ca, GCY + 14 * sa, 3, C_NEEDLE);            // counterweight
  spr.fillCircle(GCX, GCY, 8, TFT_LIGHTGREY);                            // chrome ring
  spr.fillCircle(GCX, GCY, 6, 0x2104);
  spr.fillCircle(GCX, GCY, 3, C_NEEDLE);
}
}

// ---------- Bottom half: text, scrolling titles, slide transition ----------
const int LINE_Y[3] = {114, 131, 148};   // top of each 16px text band
const int LINE_W[3] = {146, 124, 86};    // text width that fits the round glass at that height
const int LINE_H = 16;

Content shown;                 // what is on screen now
Content animFrom, animTo;
bool animating = false;
bool marqueeActive = false;
unsigned long animStart = 0;

String mqText[3];
unsigned long mqT0[3] = {0, 0, 0};

String fitText(const String &s, int maxW, int font) {
  if (spr.textWidth(s, font) <= maxW) return s;
  String t = s;
  while (t.length() > 1 && spr.textWidth(t + "...", font) > maxW) t.remove(t.length() - 1);
  return t + "...";
}

int marqueeOffset(int i, int textW) {
  int travel = textW - LINE_W[i] + 6;
  unsigned long travelMs = (unsigned long)travel * 1000UL / MARQUEE_SPEED;
  unsigned long cycle = MARQUEE_PAUSE + travelMs + MARQUEE_PAUSE;
  unsigned long t = (millis() - mqT0[i]) % cycle;
  if (t < MARQUEE_PAUSE) return 0;
  if (t < MARQUEE_PAUSE + travelMs) return (int)((t - MARQUEE_PAUSE) * MARQUEE_SPEED / 1000UL);
  return travel;
}

// Copy the area outside the text window back from the background (clips like round glass)
void restoreOutside(int y, int h, int winL, int winR) {
  uint8_t *dst = (uint8_t *)spr.getPointer();
  uint8_t *src = (uint8_t *)face.getPointer();
  for (int r = y; r < y + h && r < 170; r++) {
    if (r < 0) continue;
    int base = r * 320;
    if (winL > 0)   memcpy(dst + base, src + base, winL);
    if (winR < 319) memcpy(dst + base + winR + 1, src + base + winR + 1, 319 - winR);
  }
}

int rangeKmInt() {
  float km = latestPercent * kmPerPercent - RESERVE_LITERS * KM_PER_LITER;   // learned by the app, or the estimate
  if (km < 0) km = 0;
  return (int)(km + 2.5f) / 5 * 5;                      // rounded to 5 km so it doesn't flicker
}

String rangeText() {
  return String(rangeKmInt()) + " km";
}

// Phone icons from your design (each is one separate state, not a sequence):
// kind 0 = slashed  -> a phone is set up but not connected
// kind 1 = dots     -> a phone is connecting
// kind 2 = "?"      -> no phone has been set up yet
// kind 3 = check    -> phone linked (setup finished)
void drawPhoneGlyph(int x, int y, int w, int h, int kind) {
  int r = h >= 28 ? 4 : 3;
  spr.fillRoundRect(x, y, w, h, r, 0xFFFF);
  spr.fillRoundRect(x + 1, y + 1, w - 2, h - 2, r - 1, 0x4208);
  if (kind == 0) {
    int x1 = x - w * 15 / 100, y1 = y + h * 3 / 100;
    int x2 = x + w * 125 / 100, y2 = y + h * 94 / 100;
    for (int k = -1; k <= 1; k++) spr.drawLine(x1 + k, y1, x2 + k, y2, 0xF206);
  } else if (kind == 1) {
    int ds = h >= 28 ? 3 : 2;
    int gap = h >= 28 ? 5 : 4;
    int x0 = x + (w - (2 * gap + ds)) / 2;
    for (int i = 0; i < 3; i++) {
      bool on = (int)((millis() / 350) % 4) > i;
      spr.fillRect(x0 + i * gap, y + h * 44 / 100, ds, ds, on ? 0xFFFF : 0x7BEF);
    }
  } else if (kind == 2) {
    bool blink = (millis() / 500) & 1;
    spr.setTextDatum(MC_DATUM);
    spr.setTextSize(h >= 28 ? 2 : 1);
    spr.setTextColor(blink ? 0xFFFF : 0xBDF7);
    spr.drawString("?", x + w / 2, y + h / 2);
    spr.setTextSize(1);
  } else {
    int x1 = x + w * 20 / 100, y1 = y + h * 50 / 100;
    int x2 = x + w * 45 / 100, y2 = y + h * 70 / 100;
    int x3 = x + w * 85 / 100, y3 = y + h * 30 / 100;
    for (int k = 0; k < 2; k++) {
      spr.drawLine(x1, y1 + k, x2, y2 + k, TFT_GREEN);
      spr.drawLine(x2, y2 + k, x3, y3 + k, TFT_GREEN);
    }
  }
}

// Built-in arrows, used when Google Maps does not give us its own maneuver icon
void drawNavArrow(int cx, int cy, char dir) {
  uint16_t col = 0xFFFF;
  if (dir == 'L') {
    spr.fillRect(cx + 1, cy - 2, 6, 18, col);
    spr.fillRect(cx - 6, cy - 4, 13, 6, col);
    spr.fillTriangle(cx - 6, cy - 12, cx - 6, cy + 10, cx - 18, cy - 1, col);
  } else if (dir == 'R') {
    spr.fillRect(cx - 7, cy - 2, 6, 18, col);
    spr.fillRect(cx - 7, cy - 4, 13, 6, col);
    spr.fillTriangle(cx + 6, cy - 12, cx + 6, cy + 10, cx + 18, cy - 1, col);
  } else {
    spr.fillRect(cx - 3, cy - 4, 6, 20, col);
    spr.fillTriangle(cx - 11, cy - 4, cx + 11, cy - 4, cx, cy - 16, col);
  }
}

// The maneuver icon from Google Maps: 32x32 bits
void drawNavIcon(int x, int y) {
  for (int r = 0; r < 32; r++) {
    for (int b = 0; b < 4; b++) {
      uint8_t v = navIcon[r * 4 + b];
      if (!v) continue;
      for (int k = 0; k < 8; k++) if (v & (0x80 >> k)) spr.drawPixel(x + b * 8 + k, y + r, 0xFFFF);
    }
  }
}

void drawContent(const Content &c, int xoff, bool steady) {
  if (c.mode == 1) {
    spr.setTextDatum(TC_DATUM);
    spr.setTextColor(TFT_WHITE);
    spr.drawString(rangeText(), GCX + xoff, 112, 4);
    if (c.l[2].length() > 0) {                           // e.g. "NO PHONE" after a button press
      spr.setTextColor(C_ACCENT);
      spr.drawString(c.l[2], GCX + xoff, 148, 2);
    } else {
      drawPhoneGlyph(GCX + xoff - 7, 142, 14, 22, 0);   // slashed phone: not connected
    }
    return;
  }
  if (c.mode == 6) {                                     // firmware update: percent, label, bar
    spr.setTextDatum(TC_DATUM);
    spr.setTextColor(TFT_WHITE);
    spr.drawString(c.l[1], GCX + xoff, 102, 4);
    spr.setTextColor(C_GREY);
    spr.drawString("Updating dash", GCX + xoff, 130, 2);
    return;
  }
  if (c.mode == 5) {                                     // navigation: arrow + distance, street underneath
    int tw = spr.textWidth(c.l[0], 4);
    int ix = GCX + xoff - (32 + 8 + tw) / 2;
    if (navHasIcon) drawNavIcon(ix, 108);
    else            drawNavArrow(ix + 16, 126, navDir);
    spr.setTextDatum(TL_DATUM);
    spr.setTextColor(TFT_WHITE);
    spr.drawString(c.l[0], ix + 40, 113, 4);
    if (c.l[1].length() > 0) {
      spr.setTextDatum(TC_DATUM);
      spr.setTextColor(C_GREY);
      spr.drawString(fitText(c.l[1], 86, 1), GCX + xoff, 150, 1);
    }
    return;
  }
  if (c.mode == 4) {
    drawPhoneGlyph(GCX + xoff - 10, 100, 20, 32, c.icon);
    spr.setTextDatum(TC_DATUM);
    spr.setTextColor(TFT_WHITE);
    spr.drawString(c.l[0], GCX + xoff, 136, 2);
    if (c.l[1].length() > 0) {
      spr.setTextColor(C_GREY);
      spr.drawString(c.l[1], GCX + xoff, 154, 1);
    }
    return;
  }
  for (int i = 0; i < 3; i++) {
    const String &t = c.l[i];
    if (t.length() == 0) continue;
    uint16_t col = (i == 0) ? TFT_WHITE : (i == 1 ? C_GREY : C_ACCENT);
    spr.setTextColor(col);                        // one color = transparent background
    int w = spr.textWidth(t, 2);
    if (w <= LINE_W[i]) {
      spr.setTextDatum(TC_DATUM);
      spr.drawString(t, GCX + xoff, LINE_Y[i], 2);
    } else if (steady) {
      if (mqText[i] != t) { mqText[i] = t; mqT0[i] = millis(); }
      marqueeActive = true;
      spr.setTextDatum(TL_DATUM);
      spr.drawString(t, GCX - LINE_W[i] / 2 - marqueeOffset(i, w), LINE_Y[i], 2);
    } else {
      spr.setTextDatum(TC_DATUM);
      spr.drawString(fitText(t, LINE_W[i], 2), GCX + xoff, LINE_Y[i], 2);
    }
  }
}

// "HH:MM" from the app's clock mode
bool looksLikeClock(const String &s) {
  return s.length() == 5 && s[2] == ':' &&
         isDigit(s[0]) && isDigit(s[1]) && isDigit(s[3]) && isDigit(s[4]);
}

bool sameMain(const Content &a, const Content &b) {
  if (a.mode != b.mode) return false;
  if (a.mode == 0 && looksLikeClock(a.l[0]) && looksLikeClock(b.l[0])) return true;   // minute changes do not slide
  if (a.mode == 1) return true;                          // range changes do not slide
  if (a.mode == 4) return a.icon == b.icon && a.l[0] == b.l[0] && a.l[1] == b.l[1];
  if (a.mode == 5 || a.mode == 6) return true;                          // distance updates do not slide
  return a.l[0] == b.l[0] && a.l[1] == b.l[1];
}

void updateBottomState() {
  Content target = currentTarget();
  if (!animating) {
    if (!sameMain(shown, target)) {
      animFrom = shown; animTo = target;
      animating = true; animStart = millis();
    } else {
      shown = target;                             // same screen: update in place (third line, clock minutes), no sliding
    }
  }
  if (animating) {
    animTo.l[2] = target.l[2];
    if (millis() - animStart >= SLIDE_MS) {
      shown = animTo;
      animating = false;
      for (int i = 0; i < 3; i++) { mqText[i] = ""; }
    }
  }
}

void drawBottom() {
  marqueeActive = false;
  updateBottomState();
  if (animating) {
    float t = (millis() - animStart) / (float)SLIDE_MS;
    if (t > 1) t = 1;
    float e = 1.0f - powf(1.0f - t, 3.0f);        // ease-out
    drawContent(animFrom, (int)(-e * SLIDE_W), false);   // old content slides out to the left
    drawContent(animTo, (int)((1.0f - e) * SLIDE_W), false); // new content slides in from the right
  } else {
    drawContent(shown, 0, true);
  }
  for (int r = 96; r < 170; r++) {                      // round glass edge
    int dy = r - GCY;
    int h2 = R_SCREEN * R_SCREEN - dy * dy;
    int half = h2 > 0 ? (int)sqrtf((float)h2) - 3 : 0;
    restoreOutside(r, 1, GCX - half, GCX + half);
  }
  for (int i = 0; i < 3; i++)                            // narrower window for scrolling titles
    restoreOutside(LINE_Y[i], LINE_H, GCX - LINE_W[i] / 2, GCX + LINE_W[i] / 2);
}

// ---------- Under the hub: music bars when a phone is connected ----------
// Dancing bars while music plays, small still dots when connected but not playing,
// nothing when there is no phone.
float barH[5] = {0, 0, 0, 0, 0};

void drawEqualizer() {
  if (callState != 0 && phoneConnected) {
    // Call animation: green ripples while ringing, steady green dot on a call
    float t = millis() / 1000.0f;
    int cy = GCY + 18;
    if (callState == 1) {
      for (int i = 0; i < 3; i++) {
        float ph = fmodf(t * 1.1f + i * 0.33f, 1.0f);
        int r = 2 + (int)(ph * 9);
        uint16_t col = ph < 0.45f ? TFT_GREEN : (ph < 0.75f ? 0x0320 : C_DIM);
        spr.drawCircle(GCX, cy, r, col);
      }
      spr.fillCircle(GCX, cy, 2, TFT_GREEN);
    } else {
      spr.fillCircle(GCX, cy, 4, TFT_GREEN);
      spr.drawCircle(GCX, cy, 8, 0x0320);
    }
    for (int i = 0; i < 5; i++) barH[i] = 0;
    return;
  }
  bool connected = phoneConnected;
  bool playing = connected && lines[2] == "Playing";
  float t = millis() / 1000.0f;
  const int baseY = GCY + 24;
  for (int i = 0; i < 5; i++) {
    float target;
    if (!connected)   target = 0;
    else if (playing) target = 3 + 10 * (0.5f + 0.5f * sinf(t * (5.0f + i * 1.7f) + i * 1.3f))
                                      * (0.6f + 0.4f * sinf(t * 2.3f + i));
    else              target = 2;
    barH[i] += (target - barH[i]) * 0.35f;
    int h = (int)(barH[i] + 0.5f);
    if (h < 1) continue;
    spr.fillRect(GCX - 16 + i * 7, baseY - h, 4, h, playing ? C_ACCENT : C_GREY);
  }
}

// ---------- Battery ----------
struct Batt {
  bool valid = false;      // gauge answered on I2C
  bool present = false;    // a battery is connected
  int  soc = -1;           // percent
  int  mv = 0;
  int  ma = 0;
  bool charging = false;
} batt;

bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  val = Wire.read();
  return true;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// The charger chip's default is a 2 A charge current and a 40 s watchdog that restores
// that default. Cap the current and switch the watchdog off (re-applied every 10 s).
void chargerConfigure() {
#if LIMIT_CHARGE_CURRENT
  uint8_t v;
  if (i2cRead8(BQ25896_ADDR, 0x07, v)) i2cWrite8(BQ25896_ADDR, 0x07, v & ~0x30);                 // REG07: watchdog off
  if (i2cRead8(BQ25896_ADDR, 0x04, v)) i2cWrite8(BQ25896_ADDR, 0x04, (v & 0x80) | (CHARGE_CURRENT_MA / 64)); // REG04: ICHG, 64 mA/step
#endif
}

bool bqRead16(uint8_t reg, uint16_t &val) {
  Wire.beginTransmission(BQ27220_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BQ27220_ADDR, (uint8_t)2) != 2) return false;
  uint8_t lo = Wire.read();
  uint8_t hi = Wire.read();
  val = (uint16_t)(lo | (hi << 8));
  return true;
}

void readBattery() {
  uint16_t st, v, soc = 0xFFFF, cur = 0;
  if (!bqRead16(0x0A, st) || !bqRead16(0x08, v)) { batt.valid = false; return; }  // BatteryStatus, Voltage
  bqRead16(0x2C, soc);                                                            // StateOfCharge (%)
  bqRead16(0x0C, cur);                                                            // Current (signed mA)
  batt.valid = true;
  batt.mv = v;
  batt.soc = (soc <= 100) ? (int)soc : -1;
  batt.ma = (int16_t)cur;
  bool sane = v > 2500;
  bool discharging = (st & 1) != 0;                                               // DSG
  bool charging = sane && !discharging && batt.ma > 10;

  // Is USB power connected? (charger register 0x0B: VBUS status bits 7:5, power-good bit 2)
  uint8_t r0b = 0;
  bool usbKnown = i2cRead8(BQ25896_ADDR, 0x0B, r0b);
  bool usb = usbKnown ? (((r0b >> 5) != 0) || ((r0b & 0x04) != 0)) : true;

  // Running without USB, or current flowing into the cell, proves a battery is fitted.
  // Require it three reads in a row before remembering it.
  static int proven = 0;
  if (sane && (!usb || charging)) proven++; else proven = 0;
  if (proven >= 3 && !battFitted) {
    battFitted = true;
    prefs.putBool("batt", true);
  }

  batt.present = sane && battFitted;
  batt.charging = batt.present && charging;
  Serial.printf("BAT status=0x%04X  %u mV  soc=%d%%  %d mA  usb=%d fitted=%d present=%d charging=%d\n",
                st, v, batt.soc, batt.ma, usb, battFitted, batt.present, batt.charging);
}

// Small battery icon: level bars, a bolt while charging, a red slash when no battery
void drawBatteryIcon(int x, int y) {
  if (!batt.valid) return;
  const int w = 20, h = 11;
  if (!batt.present) {
    spr.drawRoundRect(x, y, w, h, 2, C_GREY);
    spr.fillRect(x + w, y + 3, 2, 5, C_GREY);
    spr.drawLine(x + 2, y + h + 1, x + w - 2, y - 2, TFT_RED);
    spr.drawLine(x + 3, y + h + 1, x + w - 1, y - 2, TFT_RED);
    return;
  }
  int soc = batt.soc < 0 ? 0 : batt.soc;
  uint16_t col = soc < 15 ? TFT_RED : (soc < 30 ? TFT_ORANGE : TFT_GREEN);
  if (soc < 15 && !batt.charging && !((millis() / 500) & 1)) col = C_DIM;   // blink when very low
  spr.drawRoundRect(x, y, w, h, 2, TFT_WHITE);
  spr.fillRect(x + w, y + 3, 2, 5, TFT_WHITE);
  int fw = (w - 4) * soc / 100;
  if (fw > 0) spr.fillRect(x + 2, y + 2, fw, h - 4, col);
  if (batt.charging) {                                                       // lightning bolt
    spr.fillTriangle(x + 11, y + 1, x + 7, y + 6, x + 10, y + 6, TFT_YELLOW);
    spr.fillTriangle(x + 10, y + 5, x + 13, y + 5, x + 9, y + 10, TFT_YELLOW);
  }
}

// ---------- Frame ----------
// Firmware update: a smooth curved line along the bottom edge fills from left to right
void drawOtaArc() {
  int pct = otaTotal ? (int)(otaWritten * 100 / otaTotal) : 0;
  const int R = 82;
  for (float a = 60; a <= 120; a += 0.5f) {                // track, 60 degrees wide
    float r = a * DEG_TO_RAD;
    spr.fillCircle(GCX + (int)lroundf(R * cosf(r)), GCY + (int)lroundf(R * sinf(r)), 2, C_DIM);
  }
  float filled = pct * 0.6f;                               // 60 degrees = 100 percent
  for (float d = 0; d <= filled; d += 0.5f) {              // progress, same thickness
    float r = (120 - d) * DEG_TO_RAD;
    spr.fillCircle(GCX + (int)lroundf(R * cosf(r)), GCY + (int)lroundf(R * sinf(r)), 2, C_ACCENT);
  }
}

void drawFrame(float needlePct, int percent, bool sweeping) {
  memcpy(spr.getPointer(), face.getPointer(), 320 * 170);   // static background

  // Low fuel warning (blinking pump + text), inside the semicircle
  if (!sweeping && percent < 15 && ((millis() / 500) & 1)) {
    spr.fillRoundRect(GCX - 7, GCY - 46, 14, 18, 2, TFT_RED);
    spr.fillRect(GCX - 4, GCY - 43, 8, 5, C_BG);
    spr.setTextDatum(TC_DATUM);
    spr.setTextColor(TFT_RED, C_BG);
    spr.drawString("LOW", GCX, GCY - 24, 1);
  }

  drawNeedle(needlePct);

  if (setupDone) drawEqualizer();
  if (showBattIcon) drawBatteryIcon(GCX + 26, GCY + 13);

  drawBottom();
  if (otaActive) drawOtaArc();

  spr.pushSprite(0, 0);
}

// ---------- Power off with the side button ----------
const unsigned long OFF_HOLD_MS = 1500;

void powerOff() {
  Serial.println("Powering off");

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("Turning off...", GCX, GCY, 4);
  delay(900);

  NimBLEDevice::deinit(true);                       // stop Bluetooth

  // Screen and backlight off
  tft.writecommand(0x28);                           // display off
  tft.writecommand(0x10);                           // sleep in
  ledcDetachPin(DISPLAY_BL);
  pinMode(DISPLAY_BL, OUTPUT);
  digitalWrite(DISPLAY_BL, LOW);

  // Switch off the board's peripheral power
  digitalWrite(BOARD_PWR_EN, LOW);
  gpio_hold_en((gpio_num_t)DISPLAY_BL);
  gpio_hold_en((gpio_num_t)BOARD_PWR_EN);
  gpio_deep_sleep_hold_en();

  // Running on the battery only? Then cut the battery off from the board = really off.
  // (With USB connected we only sleep, so the battery keeps charging.)
  uint8_t r0b = 0;
  bool usbKnown = i2cRead8(BQ25896_ADDR, 0x0B, r0b);
  bool usb = usbKnown ? (((r0b >> 5) != 0) || ((r0b & 0x04) != 0)) : true;
  if (!usb) {
    uint8_t v;
    if (i2cRead8(BQ25896_ADDR, 0x09, v)) i2cWrite8(BQ25896_ADDR, 0x09, v | 0x20);   // REG09: BATFET_DIS
    delay(500);                                                                      // the board loses power here
  }

  // Still running (USB connected): deep sleep, press the side button to wake
  while (digitalRead(BOARD_USER_KEY) == LOW) delay(10);   // wait for release, or we would wake at once
  delay(100);
  rtc_gpio_pullup_en((gpio_num_t)BOARD_USER_KEY);
  rtc_gpio_pulldown_dis((gpio_num_t)BOARD_USER_KEY);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOARD_USER_KEY, 0);
  esp_deep_sleep_start();
}

void handlePowerButton() {
  static unsigned long downAt = 0;
  bool pressed = digitalRead(BOARD_USER_KEY) == LOW;
  if (pressed) {
    if (downAt == 0) downAt = millis();
    else if (millis() - downAt > OFF_HOLD_MS) powerOff();
  } else {
    if (downAt != 0 && millis() - downAt > 30 && millis() - downAt < 800 && setupWaitingConfirm()) setupConfirm();
    downAt = 0;
  }
}

void setup() {
  setCpuFrequencyMhz(CPU_MHZ);
  Serial.begin(115200);

  // Coming back from "power off": release the pins that were held during deep sleep
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)BOARD_PWR_EN);
  gpio_hold_dis((gpio_num_t)DISPLAY_BL);

  pinMode(BOARD_PWR_EN, OUTPUT);   digitalWrite(BOARD_PWR_EN, HIGH);
  pinMode(BOARD_SD_CS, OUTPUT);    digitalWrite(BOARD_SD_CS, HIGH);
  pinMode(BOARD_LORA_CS, OUTPUT);  digitalWrite(BOARD_LORA_CS, HIGH);
  pinMode(DISPLAY_BL, OUTPUT);     digitalWrite(DISPLAY_BL, HIGH);

  analogReadResolution(12);
  analogSetPinAttenuation(FUEL_ADC_PIN, ADC_11db);
  int first = analogReadMilliVolts(FUEL_ADC_PIN);
  for (int i = 0; i < NUM_READINGS; i++) readings[i] = first;
  total = (long)first * NUM_READINGS;

  pinMode(ENCODER_INA, INPUT_PULLUP);
  pinMode(ENCODER_INB, INPUT_PULLUP);
  pinMode(ENCODER_KEY, INPUT_PULLUP);
  pinMode(BOARD_USER_KEY, INPUT_PULLUP);
  {                                   // if you woke it with the side button, wait until it is released
    unsigned long t0 = millis();
    while (digitalRead(BOARD_USER_KEY) == LOW && millis() - t0 < 4000) delay(10);
  }
  encPrev = (digitalRead(ENCODER_INA) << 1) | digitalRead(ENCODER_INB);
  attachInterrupt(ENCODER_INA, encISR, CHANGE);
  attachInterrupt(ENCODER_INB, encISR, CHANGE);

  Wire.begin(BOARD_I2C_SDA, BOARD_I2C_SCL);
  Wire.setTimeOut(50);
  prefs.begin("dash", false);
  battFitted = prefs.getBool("batt", false);
  setupDone = prefs.getBool("setup", false);
  showBattIcon = prefs.getBool("baticon", true);
  needleStyle = prefs.getInt("nstyle", NEEDLE_STYLE);
  C_ACCENT = (uint16_t)prefs.getUInt("cacc", C_ACCENT);
  C_NEEDLE = (uint16_t)prefs.getUInt("cneed", C_NEEDLE);
  kmPerPercent = prefs.getFloat("kmpp", kmPerPercent);
  backlightDuty = prefs.getInt("brt", BACKLIGHT_DUTY);
  savedBrt = backlightDuty;
  chargerConfigure();

  resetLines();
  shown = currentTarget();

  delay(100);
  tft.init();
  tft.setRotation(1);               // landscape 320x170
  tft.fillScreen(TFT_BLACK);

  ledcSetup(0, 5000, 8);                 // PWM backlight (after tft.init so it takes over the pin)
  ledcAttachPin(DISPLAY_BL, 0);
  ledcWrite(0, backlightDuty);

  spr.setColorDepth(8);
  face.setColorDepth(8);
  spritesOk = spr.createSprite(320, 170) != nullptr && face.createSprite(320, 170) != nullptr;
  Serial.println(spritesOk ? "Sprites OK" : "Sprite allocation FAILED");
  if (spritesOk) buildFace();

  startBle();
  bootT0 = millis();                // startup sweep begins now
}

void loop() {
  handleButton();
  handlePowerButton();
  otaService();

  static unsigned long lastFrame = 0;
  static unsigned long frameInterval = 40;
  if (millis() - lastFrame < frameInterval) { delay(2); return; }   // let the CPU sleep between frames
  lastFrame = millis();

  // Messages from the phone
  String m;
  while (popMsg(m)) handleMessage(m);
  applyIncomingIfSettled();

  // Connection changes
  static bool wasConnected = false;
  if (phoneConnected && !wasConnected) {
    Serial.println("Phone connected");
    incomingDirty = false;
    appReady = false;
    lines[0] = "Connected"; lines[1] = ""; lines[2] = "";
  } else if (!phoneConnected && wasConnected) {
    Serial.println("Phone disconnected");
    incomingDirty = false;
    appReady = false;
    callState = 0;
    callName = "";
    resetLines();
  }
  wasConnected = phoneConnected;

  // Volume from encoder
  int steps = encSteps; encSteps = 0;
  if (steps != 0) Serial.printf("Encoder steps: %d\n", steps);
  if (steps > 0)      { for (int i = 0; i < min(steps, 5); i++)  sendCmd("VOLUP", "VOLUME +"); }
  else if (steps < 0) { for (int i = 0; i < min(-steps, 5); i++) sendCmd("VOLDOWN", "VOLUME -"); }

  // Fuel
  total -= readings[readIndex];
  readings[readIndex] = analogReadMilliVolts(FUEL_ADC_PIN);
  total += readings[readIndex];
  readIndex = (readIndex + 1) % NUM_READINGS;
  int avgMv = total / NUM_READINGS;
  int percent = constrain(map(avgMv, EMPTY_MV, FULL_MV, 0, 100), 0, 100);

  latestPercent = percent;

  // Battery gauge, every 2 seconds
  static unsigned long lastBat = 0;
  if (millis() - lastBat > 2000 || lastBat == 0 || forceBatRead) {
    lastBat = millis();
    forceBatRead = false;
    readBattery();

    static unsigned long lastChg = 0;
    if (lastChg == 0 || millis() - lastChg > 10000) {
      lastChg = millis();
      chargerConfigure();
    }
  }

  // Tell the phone app the fuel level and range
  static unsigned long lastInfo = 0;
  if (phoneConnected && millis() - lastInfo > 2000) {
    lastInfo = millis();
    char b[24];
    snprintf(b, sizeof(b), "FUEL:%d", percent);
    notifyRaw(b);
    snprintf(b, sizeof(b), "RNG:%d", rangeKmInt());
    notifyRaw(b);
    snprintf(b, sizeof(b), "BATFIT:%d", battFitted ? 1 : 0);
    notifyRaw(b);
    snprintf(b, sizeof(b), "SETUPDONE:%d", setupDone ? 1 : 0);
    notifyRaw(b);
    snprintf(b, sizeof(b), "BATICON:%d", showBattIcon ? 1 : 0);
    notifyRaw(b);
    snprintf(b, sizeof(b), "FW:%s", FW_VERSION);
    notifyRaw(b);
    if (batt.valid) {
      if (batt.present) snprintf(b, sizeof(b), "BAT:%d", batt.soc < 0 ? 0 : batt.soc);
      else              snprintf(b, sizeof(b), "BAT:NONE");
      notifyRaw(b);
      snprintf(b, sizeof(b), "CHG:%d", batt.charging ? 1 : 0);
      notifyRaw(b);
    }
  }

  // Needle: startup sweep E -> F -> E, then eases toward the real level
  static float needlePct = 0;
  unsigned long st = millis() - bootT0;
  bool sweeping = st < 2 * SWEEP_MS;
  if (st < SWEEP_MS)            needlePct = 100.0f * easeInOut(st / (float)SWEEP_MS);
  else if (sweeping)            needlePct = 100.0f * (1.0f - easeInOut((st - SWEEP_MS) / (float)SWEEP_MS));
  else                          needlePct += (percent - needlePct) * 0.15f;

  bool showSetup = !setupDone || (phoneConnected && !appReady);   // animated icon screens
  if (spritesOk) drawFrame(needlePct, percent, sweeping);

  // Redraw fast only while something moves, otherwise ~8 fps (much less CPU and SPI work)
  bool playing = phoneConnected && lines[2] == "Playing";
  bool batBlink = showBattIcon && batt.valid && batt.present && batt.soc >= 0 && batt.soc < 15 && !batt.charging;
  bool busy = showSetup || sweeping || animating || marqueeActive || fabsf(needlePct - percent) > 0.2f ||
              playing || callState == 1 || percent < 15 || batBlink || millis() < actionUntil;
  frameInterval = busy ? 40 : 125;

  static unsigned long lastLog = 0;
  if (millis() - lastLog > 1000) {
    Serial.printf("ADC: %d mV  Fuel: %d%%\n", avgMv, percent);
    lastLog = millis();
  }
}