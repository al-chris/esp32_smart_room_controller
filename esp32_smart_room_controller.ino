#define ENABLE_USER_AUTH
#define ENABLE_DATABASE

#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <math.h>
#include "RadarType.h"

// ================= Radar Model =================
// Both modules share the same wiring (5V, GND, module TX -> GPIO16, module RX -> GPIO17)
// and the same 256000 baud UART, so switching is this one line plus swapping the module.
#define RADAR_RD03D   1   // Ai-Thinker RD-03D: x/y tracking, moving targets, optional classifier
#define RADAR_LD2410  2   // Hi-Link LD2410 / LD2410B / LD2410C: moving + stationary (breathing) presence
#define RADAR_MODEL   RADAR_LD2410

#if RADAR_MODEL == RADAR_LD2410
  #define RADAR_NAME "LD2410"
#else
  #define RADAR_NAME "RD-03D"
#endif

#define RADAR_CONNECTED true    // false = use the test button on GPIO33 instead
#define RADAR_DEBUG     true    // per track feature dump on the serial monitor
#define LD2410_WRITE_CONFIG true  // push max gates + module hold time to the LD2410 on boot

// Most 2-channel relay boards (SRD-05VDC-SL-C) are Active-LOW:
// LOW = Relay ON (contacts closed)
// HIGH = Relay OFF (contacts open)
bool lightActiveLow = true;
bool fanActiveLow   = true;

#define RELAY_SELFTEST false
#define FORCE_DEFAULTS_ON_BOOT true

// ================= WiFi / Firebase =================
// WIFI_SSID, WIFI_PASSWORD, WEB_API_KEY, DATABASE_URL, USER_EMAIL, USER_PASS
// live in secrets.h (copy secrets.example.h to get started).
#include "secrets.h"
#define DB_PATH       "/rooms/room1"

void processData(AsyncResult &aResult);
void pollRadar();

UserAuth    user_auth(WEB_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
using AsyncClient = AsyncClientClass;

WiFiClientSecure ssl_client;
AsyncClient      aClient(ssl_client);
RealtimeDatabase Database;

unsigned long lastFirebaseSync = 0;
const unsigned long FIREBASE_SYNC_INTERVAL = 3000;
uint8_t syncCycle = 0;

// ================= Pins =================
#define DHTPIN          4
#define DHTTYPE         DHT22
#define RELAY_LIGHT_PIN 26
#define RELAY_FAN_PIN   27
#define TEST_BUTTON_PIN 33
#define RADAR_RX_PIN    16    // ESP32 INPUT  <- module TX
#define RADAR_TX_PIN    17    // ESP32 OUTPUT -> module RX

// ================= Serial & Timing =================
#define SERIAL_MONITOR_BAUD 115200
#define RADAR_BAUD          256000

DHT dht(DHTPIN, DHTTYPE);
HardwareSerial RadarSerial(2);

// ================= Environment Thresholds =================
float tempThreshold  = 30.0;   // deg C
float humThreshold   = 70.0;   // %RH
bool  fanRequireBoth = true;   // true = Both temp AND hum must exceed threshold

// Hysteresis release band
const float TEMP_HYST = 0.5;
const float HUM_HYST  = 2.0;

// ================= Light Timing =================
const uint32_t LIGHT_ON_DELAY_MS = 3000;

// ================= RD-03D Gating =================
float    speedMinCms     = 0.0;
float    speedMaxCms     = 300.0;
bool     allowStatic     = true;
// Lowered from 60.0 cm to 20.0 cm so desk/bench testing is not rejected as "too near"
float    distMinCm       = 20.0;
float    distMaxCm       = 450.0;
float    angleMaxDeg     = 60.0;

uint16_t confirmFrames   = 5;
uint32_t holdMs          = 8000;
uint16_t clearFrames     = 8;

float    minStraightness = 0.15;
float    stillRadiusMm   = 300.0;
float    jitterFloorMm   = 25.0;
float    assocGateMm     = 900.0;

float    filtAlpha       = 0.45;
float    filtBeta        = 0.10;
float    maxTrackSpeedMm = 3000.0;
float    outlierGateMm   = 700.0;
uint16_t maxOutliers     = 4;
uint32_t motionWindowMs  = 3000;

// ================= LD2410 Gating =================
// distMinCm / distMaxCm / confirmFrames / holdMs above are shared with the LD2410.
uint8_t  ldMoveEnergyMin  = 0;     // 0-100, extra filter on top of the module's own gate thresholds
uint8_t  ldStillEnergyMin = 0;     // raise this if the running fan keeps the room "occupied"
bool     ldUseStill       = true;  // false = only moving targets count (plain motion sensor)
uint8_t  ldMaxGate        = 8;     // 0.75 m per gate, 2-8; 8 = ~6 m
uint16_t ldNoOneSec       = 1;     // module's own hold time; kept short so holdMs does the holding

// ================= Classifier Tunables =================
float    gaitHzMin           = 1.0;
float    gaitHzMax           = 2.8;
float    gaitHzAnimalMin     = 3.0;
float    cadenceJitterOk     = 0.38;
float    turnRateHumanDps    = 60.0;
float    turnRateAnimalDps   = 140.0;
float    continuityHumanMin  = 0.85;
float    continuityAnimalMax = 0.62;
float    bigTargetRangeCm    = 250.0;
float    walkSpeedMinCms     = 25.0;
float    walkSpeedMaxCms     = 180.0;
float    dashSpeedCms        = 220.0;
float    jerkHumanMax        = 10.0;
float    jerkAnimalMin       = 25.0;

uint8_t  humanScoreMin        = 65;
uint8_t  animalScoreMax       = 35;
uint8_t  classCommitN         = 3;
bool     unknownCountsAsHuman = true;
bool     classifierEnabled    = false;

const uint32_t CLASS_EVAL_MS = 500;
const uint32_t CLASS_MIN_AGE = 1200;

// ================= Runtime State =================
float temperatureC   = 0.0;
float humidityPct    = 0.0;
bool  dhtValid       = false;
uint8_t dhtFailCount = 0;
float   tBuf[3], hBuf[3];
uint8_t dhtIdx = 0, dhtSamples = 0;

static float median3(float a, float b, float c) {
  if (a > b) { float s = a; a = b; b = s; }
  if (b > c) { float s = b; b = c; c = s; }
  if (a > b) { float s = a; a = b; b = s; }
  return b;
}

bool  humanPresent     = false;
bool  animalPresent    = false;
bool  rawRadarTarget   = false;
bool  fanOn            = false;
bool  lightOn          = false;
bool  fanEnvTrip       = false;
bool  lightPending     = false;

bool lightOverride     = false;
bool fanOverride       = false;
bool lightManualValue  = false;
bool fanManualValue    = false;

float targetDistanceCM = 0.0;
float targetAngleDeg   = 0.0;
float targetSpeedCms   = 0.0;
float targetRatio      = 0.0;
float targetCadenceHz  = 0.0;
float targetTurnDps    = 0.0;
float targetGroundCms  = 0.0;
uint8_t targetScore    = 0;
int8_t confirmedTrack  = -1;
const char *rejectReason = "idle";

unsigned long lastSensorRead   = 0;
const unsigned long SENSOR_INTERVAL = 2000;

unsigned long lastHumanMs      = 0;
unsigned long lastAnimalMs     = 0;
unsigned long presenceSinceMs  = 0;
bool          prevPresence     = false;
unsigned long lastValidFrameMs = 0;
unsigned long lastDebugMs      = 0;
unsigned long frameCount       = 0;
unsigned long lastFrameArrival = 0;
float         radarFps         = 0.0;
bool          readingStale     = true;

// ================= Relay Helpers =================
static inline void writeRelay(uint8_t pin, bool on, bool activeLow) {
  digitalWrite(pin, (on == activeLow) ? LOW : HIGH);
}

static inline void driveLight(bool on) { writeRelay(RELAY_LIGHT_PIN, on, lightActiveLow); }
static inline void driveFan(bool on)   { writeRelay(RELAY_FAN_PIN,   on, fanActiveLow); }

// ================= RD-03D Frame Decoding =================
static const uint8_t RADAR_HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
uint8_t radarFrame[30];
uint8_t radarIdx = 0;

static const uint8_t CMD_MULTI_TARGET[12] = {
  0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01
};

static float decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  float mag = (float)(raw & 0x7FFF);
  return (raw & 0x8000) ? mag : -mag;
}

// ================= Position Based Tracks =================
Track tracks[MAX_TRACKS];

void clearTrack(uint8_t i) {
  Track &t = tracks[i];
  t.active        = false;
  t.hits          = 0;
  t.misses        = 0;
  t.pathMm        = 0.0;
  t.ratio         = 0.0;
  t.motionOk      = false;
  t.spdCount      = 0;
  t.spdHead       = 0;
  t.headingValid  = false;
  t.lastSpeed     = 0.0;
  t.turnRateDps   = 0.0;
  t.jerk          = 0.0;
  t.fx            = 0.0;
  t.fy            = 0.0;
  t.vx            = 0.0;
  t.vy            = 0.0;
  t.filtReady     = false;
  t.groundSpeed   = 0.0;
  t.dtEma         = 0.05;
  t.outliers      = 0;
  t.fDistCm       = 0.0;
  t.fAngleDeg     = 0.0;
  t.continuity    = 1.0;
  t.maxRangeCm    = 0.0;
  t.cadenceHz     = 0.0;
  t.cadenceJitter = 1.0;
  t.meanSpeed     = 0.0;
  t.speedStd      = 0.0;
  t.score         = 50;
  t.cls           = CLS_NONE;
  t.candidate     = CLS_NONE;
  t.candidateN    = 0;
  t.stickyHuman   = false;
}

void startWindow(uint8_t i, float x, float y, unsigned long now) {
  Track &t = tracks[i];
  t.winStartX = x;  t.winStartY = y;
  t.winStartMs = now;
  t.pathMm = 0.0;
  t.minX = t.maxX = x;
  t.minY = t.maxY = y;
}

void openTrack(uint8_t i, const RadarTarget &d, unsigned long now) {
  clearTrack(i);
  Track &t = tracks[i];
  t.active      = true;
  t.hits        = 1;
  t.x           = d.xMm;
  t.y           = d.yMm;
  t.speedCms    = d.speedCms;
  t.distCm      = d.distCm;
  t.angleDeg    = d.angleDeg;
  t.ratio       = 1.0;
  t.motionOk    = allowStatic;
  t.lastSpeed   = fabs(d.speedCms);
  t.maxRangeCm  = d.distCm;
  t.fx          = d.xMm;
  t.fy          = d.yMm;
  t.vx          = 0.0;
  t.vy          = 0.0;
  t.filtReady   = true;
  t.fDistCm     = d.distCm;
  t.fAngleDeg   = d.angleDeg;
  t.lastFrameMs = now;
  t.dtEma       = 0.05;
  t.bornMs      = now;
  t.lastEvalMs  = now;
  t.cls         = CLS_UNKNOWN;
  startWindow(i, d.xMm, d.yMm, now);
}

void pushSpeed(Track &t, float v, unsigned long now) {
  t.spdHist[t.spdHead] = v;
  t.spdTime[t.spdHead] = now;
  t.spdHead = (t.spdHead + 1) % SPD_HIST;
  if (t.spdCount < SPD_HIST) t.spdCount++;
}

static float wrap180(float a) {
  while (a >  180.0) a -= 360.0;
  while (a < -180.0) a += 360.0;
  return a;
}

void analyseCadence(Track &t) {
  if (t.spdCount < 12) { t.cadenceHz = 0.0; t.cadenceJitter = 1.0; return; }

  uint8_t n = t.spdCount;
  uint8_t start = (t.spdHead + SPD_HIST - n) % SPD_HIST;

  float sum = 0.0;
  for (uint8_t k = 0; k < n; k++) sum += t.spdHist[(start + k) % SPD_HIST];
  float mean = sum / n;

  float var = 0.0;
  for (uint8_t k = 0; k < n; k++) {
    float d = t.spdHist[(start + k) % SPD_HIST] - mean;
    var += d * d;
  }
  float sd = sqrtf(var / n);

  t.meanSpeed = mean;
  t.speedStd  = sd;

  float dead = sd * 0.30;
  if (dead < 2.5) dead = 2.5;

  uint32_t tFirst = t.spdTime[start];
  uint32_t tLast  = t.spdTime[(start + n - 1) % SPD_HIST];
  float spanS = (tLast - tFirst) / 1000.0;
  if (spanS < 0.5) { t.cadenceHz = 0.0; t.cadenceJitter = 1.0; return; }

  bool armed = false;
  uint8_t crossings = 0;
  uint32_t crossTimes[16];

  for (uint8_t k = 0; k < n; k++) {
    uint8_t idx = (start + k) % SPD_HIST;
    float v = t.spdHist[idx];
    if (v < mean - dead) armed = true;
    else if (armed && v > mean + dead) {
      armed = false;
      if (crossings < 16) crossTimes[crossings] = t.spdTime[idx];
      crossings++;
    }
  }

  t.cadenceHz = crossings / spanS;

  if (crossings >= 3) {
    uint8_t m = (crossings < 16) ? crossings : 16;
    float iSum = 0.0;
    for (uint8_t k = 1; k < m; k++) iSum += (crossTimes[k] - crossTimes[k - 1]);
    float iMean = iSum / (m - 1);
    if (iMean > 1.0) {
      float iVar = 0.0;
      for (uint8_t k = 1; k < m; k++) {
        float d = (float)(crossTimes[k] - crossTimes[k - 1]) - iMean;
        iVar += d * d;
      }
      t.cadenceJitter = sqrtf(iVar / (m - 1)) / iMean;
    } else t.cadenceJitter = 1.0;
  } else {
    t.cadenceJitter = 1.0;
  }
}

uint8_t scoreHuman(Track &t) {
  int s = 50;

  if (t.cadenceHz >= gaitHzMin && t.cadenceHz <= gaitHzMax)      s += 20;
  else if (t.cadenceHz > gaitHzAnimalMin)                        s -= 18;
  else if (t.cadenceHz > gaitHzMax)                              s -= 8;

  if (t.cadenceHz > 0.5) {
    if (t.cadenceJitter < cadenceJitterOk)        s += 10;
    else if (t.cadenceJitter > 0.70)              s -= 8;
  }

  if (t.turnRateDps < turnRateHumanDps)           s += 12;
  else if (t.turnRateDps > turnRateAnimalDps)     s -= 18;

  if (t.continuity > continuityHumanMin)          s += 14;
  else if (t.continuity < continuityAnimalMax)    s -= 14;

  if (t.maxRangeCm > bigTargetRangeCm && t.continuity > 0.80)  s += 12;
  if (t.maxRangeCm < 160.0 && t.continuity < 0.75)             s -= 12;

  if (t.groundSpeed >= walkSpeedMinCms && t.groundSpeed <= walkSpeedMaxCms) s += 8;
  if (t.groundSpeed > dashSpeedCms)                                         s -= 15;
  if (t.jerk < jerkHumanMax)        s += 6;
  else if (t.jerk > jerkAnimalMin)  s -= 10;

  if (s < 0)   s = 0;
  if (s > 100) s = 100;
  return (uint8_t)s;
}

void evaluateClass(uint8_t i, unsigned long now) {
  Track &t = tracks[i];
  if (now - t.lastEvalMs < CLASS_EVAL_MS) return;
  t.lastEvalMs = now;
  if (now - t.bornMs < CLASS_MIN_AGE) return;

  analyseCadence(t);

  if (t.stickyHuman) {
    float boxW = t.maxX - t.minX, boxH = t.maxY - t.minY;
    float boxRadius = 0.5 * ((boxW > boxH) ? boxW : boxH);
    if (boxRadius <= stillRadiusMm && t.groundSpeed < walkSpeedMinCms) {
      t.cls = CLS_HUMAN;
      return;
    }
  }

  t.score = scoreHuman(t);

  uint8_t guess;
  if (!t.motionOk && t.cadenceHz > gaitHzAnimalMin && t.cadenceJitter < 0.25) {
    guess = CLS_MECHANICAL;
  } else if (t.score >= humanScoreMin) {
    guess = CLS_HUMAN;
  } else if (t.score <= animalScoreMax) {
    guess = CLS_ANIMAL;
  } else {
    guess = CLS_UNKNOWN;
  }

  if (guess == t.candidate) {
    if (t.candidateN < 255) t.candidateN++;
  } else {
    t.candidate  = guess;
    t.candidateN = 1;
  }

  if (t.candidateN >= classCommitN) {
    t.cls = t.candidate;
    if (t.cls == CLS_HUMAN) t.stickyHuman = true;
  }
}

// ================= Gating Filters =================
bool passesSpeedAndZone(const RadarTarget &t, const char **why) {
  float v = fabs(t.speedCms);

  if (v > speedMaxCms)                 { *why = "too fast";  return false; }
  if (v < speedMinCms && !allowStatic) { *why = "too slow";  return false; }
  if (t.distCm < distMinCm)            { *why = "too near";  return false; }
  if (t.distCm > distMaxCm)            { *why = "too far";   return false; }
  if (fabs(t.angleDeg) > angleMaxDeg)  { *why = "off axis";  return false; }

  return true;
}

void updateMotionPattern(uint8_t i, unsigned long now) {
  Track &t = tracks[i];

  if (now - t.winStartMs >= motionWindowMs) {
    startWindow(i, t.x, t.y, now);
    return;
  }

  float boxW = t.maxX - t.minX;
  float boxH = t.maxY - t.minY;
  float boxRadius = 0.5 * ((boxW > boxH) ? boxW : boxH);

  if (boxRadius <= stillRadiusMm) {
    t.ratio    = 1.0;
    t.motionOk = allowStatic;
    return;
  }

  float dx = t.x - t.winStartX;
  float dy = t.y - t.winStartY;
  float net = sqrtf(dx * dx + dy * dy);

  if (t.pathMm < 1.0) { t.ratio = 1.0; t.motionOk = allowStatic; return; }

  t.ratio    = net / t.pathMm;
  t.motionOk = (t.ratio >= minStraightness);
}

// ================= Frame Pipeline =================
void noteFrameArrival(unsigned long now) {
  frameCount++;
  lastValidFrameMs = now;
  readingStale = false;

  if (lastFrameArrival != 0) {
    float gap = (now - lastFrameArrival) / 1000.0;
    if (gap > 0.004 && gap < 1.0) {
      float fps = 1.0 / gap;
      radarFps = (radarFps <= 0.0) ? fps : (radarFps * 0.9 + fps * 0.1);
    }
  }
  lastFrameArrival = now;
}

void releaseExpiredPresence(unsigned long now) {
  if (humanPresent && (now - lastHumanMs >= holdMs)) {
    humanPresent     = false;
    confirmedTrack   = -1;
    targetDistanceCM = 0.0;
    targetAngleDeg   = 0.0;
    targetSpeedCms   = 0.0;
    targetRatio      = 0.0;
    targetCadenceHz  = 0.0;
    targetGroundCms  = 0.0;
    targetScore      = 0;
  }
  if (animalPresent && (now - lastAnimalMs >= holdMs)) animalPresent = false;
}

void handleRadarFrame(const RadarTarget targets[3]) {
  unsigned long now = millis();
  noteFrameArrival(now);

  rawRadarTarget = targets[0].valid || targets[1].valid || targets[2].valid;

  const char *why = rawRadarTarget ? "gated" : "no target";
  bool humanThisFrame  = false;
  bool animalThisFrame = false;
  bool trackTouched[MAX_TRACKS] = {false, false, false};

  for (uint8_t d = 0; d < 3; d++) {
    const RadarTarget &det = targets[d];
    if (!det.valid) continue;

    const char *gateWhy = "ok";
    if (!passesSpeedAndZone(det, &gateWhy)) { why = gateWhy; continue; }

    int8_t best = -1;
    float  bestDist = assocGateMm;
    float  bestResid = 0.0;

    for (uint8_t i = 0; i < MAX_TRACKS; i++) {
      if (!tracks[i].active || trackTouched[i]) continue;
      Track &c = tracks[i];
      float dt = (now - c.lastFrameMs) / 1000.0;
      if (dt < 0.005) dt = 0.005;
      if (dt > 0.5)   dt = 0.5;
      float px = c.fx + c.vx * dt;
      float py = c.fy + c.vy * dt;
      float resid = sqrtf(powf(det.xMm - px, 2) + powf(det.yMm - py, 2));
      if (resid < bestDist) { bestDist = resid; best = i; bestResid = resid; }
    }

    if (best < 0) {
      for (uint8_t i = 0; i < MAX_TRACKS; i++) {
        if (!tracks[i].active) {
          openTrack(i, det, now);
          trackTouched[i] = true;
          if (!humanThisFrame) why = "building";
          break;
        }
      }
      continue;
    }

    Track &t = tracks[best];
    trackTouched[best] = true;
    t.misses = 0;

    float dt = (now - t.lastFrameMs) / 1000.0;
    if (dt < 0.005) dt = 0.005;
    if (dt > 0.5)   dt = 0.5;
    t.lastFrameMs = now;
    t.dtEma = t.dtEma * 0.9 + dt * 0.1;

    if (bestResid > outlierGateMm) {
      if (++t.outliers < maxOutliers) {
        t.fx += t.vx * dt;
        t.fy += t.vy * dt;
        t.misses = 0;
        if (!humanThisFrame) why = "outlier";
        continue;
      }
      t.fx = det.xMm;  t.fy = det.yMm;
      t.vx = 0.0;      t.vy = 0.0;
      t.outliers = 0;
    } else {
      t.outliers = 0;
    }

    float px = t.fx + t.vx * dt;
    float py = t.fy + t.vy * dt;
    float rx = det.xMm - px;
    float ry = det.yMm - py;

    float prevFx = t.fx, prevFy = t.fy;

    t.fx = px + filtAlpha * rx;
    t.fy = py + filtAlpha * ry;
    t.vx = t.vx + (filtBeta / dt) * rx;
    t.vy = t.vy + (filtBeta / dt) * ry;

    float vmag = sqrtf(t.vx * t.vx + t.vy * t.vy);
    if (vmag > maxTrackSpeedMm) {
      float k = maxTrackSpeedMm / vmag;
      t.vx *= k;  t.vy *= k;  vmag = maxTrackSpeedMm;
    }
    t.groundSpeed = t.groundSpeed * 0.85 + (vmag / 10.0) * 0.15;

    t.x = t.fx;  t.y = t.fy;
    t.fDistCm   = sqrtf(t.fx * t.fx + t.fy * t.fy) / 10.0;
    t.fAngleDeg = (t.fx != 0.0 || t.fy != 0.0)
                  ? atan2f(t.fx, t.fy) * 180.0 / PI : 0.0;
    t.distCm    = t.fDistCm;
    t.angleDeg  = t.fAngleDeg;
    t.speedCms  = det.speedCms;
    if (t.fDistCm > t.maxRangeCm) t.maxRangeCm = t.fDistCm;

    float stepMm = sqrtf(powf(t.fx - prevFx, 2) + powf(t.fy - prevFy, 2));
    if (vmag > 80.0) {
      float heading = atan2f(t.vx, t.vy) * 180.0 / PI;
      if (t.headingValid) {
        float dps = fabs(wrap180(heading - t.lastHeading)) / dt;
        if (dps > 720.0) dps = 720.0;
        t.turnRateDps = t.turnRateDps * 0.92 + dps * 0.08;
      }
      t.lastHeading  = heading;
      t.headingValid = true;
    }
    if (stepMm >= jitterFloorMm) t.pathMm += stepMm;

    float v = fabs(det.speedCms);
    pushSpeed(t, v, now);
    t.jerk = t.jerk * 0.92 + fabs(v - t.lastSpeed) * 0.08;
    t.lastSpeed = v;

    if (t.x < t.minX) t.minX = t.x;
    if (t.x > t.maxX) t.maxX = t.x;
    if (t.y < t.minY) t.minY = t.y;
    if (t.y > t.maxY) t.maxY = t.y;

    if (t.hits < 65000) t.hits++;

    updateMotionPattern(best, now);
    if (classifierEnabled) evaluateClass(best, now);

    if (t.hits >= confirmFrames && t.motionOk) {
      bool countsAsHuman = !classifierEnabled ||
                           (t.cls == CLS_HUMAN) ||
                           (t.cls == CLS_UNKNOWN && unknownCountsAsHuman);

      if (countsAsHuman) {
        humanThisFrame   = true;
        confirmedTrack   = best;
        targetDistanceCM = t.distCm;
        targetAngleDeg   = t.angleDeg;
        targetSpeedCms   = t.speedCms;
        targetRatio      = t.ratio;
        targetCadenceHz  = t.cadenceHz;
        targetTurnDps    = t.turnRateDps;
        targetGroundCms  = t.groundSpeed;
        targetScore      = t.score;
        why              = (t.cls == CLS_HUMAN) ? "human" : "unknown";
        lastHumanMs      = now;
      } else if (t.cls == CLS_ANIMAL) {
        animalThisFrame = true;
        lastAnimalMs    = now;
        if (!humanThisFrame) why = "animal";
      } else if (!humanThisFrame) {
        why = CLASS_NAME[t.cls];
      }
    } else if (!humanThisFrame) {
      why = t.motionOk ? "building" : "oscillating";
    }
  }

  for (uint8_t i = 0; i < MAX_TRACKS; i++) {
    if (!tracks[i].active) continue;
    if (trackTouched[i]) {
      tracks[i].continuity = tracks[i].continuity * 0.94 + 0.06;
    } else {
      tracks[i].continuity = tracks[i].continuity * 0.94;
      if (++tracks[i].misses >= clearFrames) clearTrack(i);
    }
  }

  rejectReason = why;

  if (humanThisFrame)  humanPresent  = true;
  if (animalThisFrame) animalPresent = true;

  releaseExpiredPresence(now);
}

void pollRd03d() {
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();

    if (radarIdx < 4) {
      if (b == RADAR_HDR[radarIdx]) {
        radarFrame[radarIdx++] = b;
      } else {
        radarIdx = (b == RADAR_HDR[0]) ? 1 : 0;
        radarFrame[0] = RADAR_HDR[0];
      }
      continue;
    }

    radarFrame[radarIdx++] = b;

    if (radarIdx == 30) {
      if (radarFrame[28] == 0x55 && radarFrame[29] == 0xCC) {
        RadarTarget targets[3];
        for (uint8_t t = 0; t < 3; t++) {
          uint8_t off = 4 + t * 8;
          float x = decodeSigned(radarFrame[off],     radarFrame[off + 1]);
          float y = decodeSigned(radarFrame[off + 2], radarFrame[off + 3]);
          float v = decodeSigned(radarFrame[off + 4], radarFrame[off + 5]);

          float d = sqrtf(x * x + y * y) / 10.0;

          targets[t].valid    = !(x == 0 && y == 0) && d > 1.0 && d < 900.0;
          targets[t].xMm      = x;
          targets[t].yMm      = y;
          targets[t].speedCms = v;
          targets[t].distCm   = d;
          targets[t].angleDeg = (y != 0 || x != 0)
                                ? atan2f(x, y) * 180.0 / PI : 0.0;
        }
        handleRadarFrame(targets);
      }
      radarIdx = 0;
    }
  }
}

// ================= LD2410 Frame Decoding =================
// Data frame: F4 F3 F2 F1 | payload length (2 bytes, LE) | payload | F8 F7 F6 F5
// Payload (basic mode, 13 bytes): type 0x02, 0xAA, target state, moving cm (2), moving energy,
//                                 still cm (2), still energy, detect cm (2), 0x55, 0x00
// Engineering mode (type 0x01) has the same first 11 bytes, so it decodes too.
// Target state: 0 none, 1 moving, 2 still, 3 moving + still.
static const uint8_t LD_HDR[4]  = {0xF4, 0xF3, 0xF2, 0xF1};
static const uint8_t LD_TAIL[4] = {0xF8, 0xF7, 0xF6, 0xF5};
#define LD_MAX_PAYLOAD 64
uint8_t  ldFrame[6 + LD_MAX_PAYLOAD + 4];
uint16_t ldIdx = 0;
uint16_t ldLen = 0;

static const uint8_t LD_CMD_ENABLE_CONF[14] = {
  0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01
};
static const uint8_t LD_CMD_END_CONF[12] = {
  0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01
};

uint8_t  ldState       = 0;
uint16_t ldMoveCm      = 0;
uint8_t  ldMoveEnergy  = 0;
uint16_t ldStillCm     = 0;
uint8_t  ldStillEnergy = 0;
uint16_t ldHits        = 0;

static bool ldPasses(uint16_t cm, uint8_t energy, uint8_t minEnergy, const char **why) {
  if (cm < distMinCm)     { *why = "too near"; return false; }
  if (cm > distMaxCm)     { *why = "too far";  return false; }
  if (energy < minEnergy) { *why = "weak";     return false; }
  return true;
}

void handleLd2410Frame(const uint8_t *p) {
  unsigned long now = millis();
  noteFrameArrival(now);

  ldState       = p[2];
  ldMoveCm      = (uint16_t)p[3] | ((uint16_t)p[4] << 8);
  ldMoveEnergy  = p[5];
  ldStillCm     = (uint16_t)p[6] | ((uint16_t)p[7] << 8);
  ldStillEnergy = p[8];

  rawRadarTarget = (ldState != 0);

  const char *why = "no target";
  bool moving = false, still = false;

  if (ldState & 0x02) {
    if (!ldUseStill) why = "still ignored";
    else still = ldPasses(ldStillCm, ldStillEnergy, ldStillEnergyMin, &why);
  }
  if (ldState & 0x01) moving = ldPasses(ldMoveCm, ldMoveEnergy, ldMoveEnergyMin, &why);

  bool target = moving || still;
  if (!target)             ldHits = 0;
  else if (ldHits < 65000) ldHits++;

  if (target && ldHits >= confirmFrames) {
    humanPresent     = true;
    lastHumanMs      = now;
    targetDistanceCM = moving ? ldMoveCm : ldStillCm;
    targetAngleDeg   = 0.0;
    targetSpeedCms   = 0.0;
    why = (moving && still) ? "moving+still" : (moving ? "moving" : "still");
  } else if (target) {
    why = "building";
  }

  rejectReason = why;
  releaseExpiredPresence(now);
}

void pollLd2410() {
  while (RadarSerial.available()) {
    uint8_t b = RadarSerial.read();

    if (ldIdx < 4) {
      if (b == LD_HDR[ldIdx]) ldFrame[ldIdx++] = b;
      else ldIdx = (b == LD_HDR[0]) ? 1 : 0;
      continue;
    }

    ldFrame[ldIdx++] = b;

    if (ldIdx == 6) {
      ldLen = (uint16_t)ldFrame[4] | ((uint16_t)ldFrame[5] << 8);
      if (ldLen < 13 || ldLen > LD_MAX_PAYLOAD) ldIdx = 0;
      continue;
    }

    if (ldIdx == 6 + ldLen + 4) {
      const uint8_t *p = ldFrame + 6;
      if (memcmp(ldFrame + 6 + ldLen, LD_TAIL, 4) == 0 &&
          p[1] == 0xAA && p[ldLen - 2] == 0x55) {
        handleLd2410Frame(p);
      }
      ldIdx = 0;
    }
  }
}

// ================= Radar Dispatch =================
void pollRadar() {
#if RADAR_MODEL == RADAR_LD2410
  pollLd2410();
#else
  pollRd03d();
#endif
}

void radarSendCommand(const uint8_t *cmd, size_t len) {
  RadarSerial.write(cmd, len);
  RadarSerial.flush();
  delay(60);
}

void radarFlush() {
  while (RadarSerial.available()) RadarSerial.read();
  radarIdx = 0;
  ldIdx    = 0;
  lastValidFrameMs = millis();
}

void ld2410ApplyConfig() {
  // Command 0x0060: max moving gate (word 0), max still gate (word 1), no-one duration s (word 2)
  uint8_t cmd[30] = {
    0xFD, 0xFC, 0xFB, 0xFA, 0x14, 0x00, 0x60, 0x00,
    0x00, 0x00, ldMaxGate, 0x00, 0x00, 0x00,
    0x01, 0x00, ldMaxGate, 0x00, 0x00, 0x00,
    0x02, 0x00, (uint8_t)(ldNoOneSec & 0xFF), (uint8_t)(ldNoOneSec >> 8), 0x00, 0x00,
    0x04, 0x03, 0x02, 0x01
  };
  radarSendCommand(LD_CMD_ENABLE_CONF, sizeof(LD_CMD_ENABLE_CONF));
  radarSendCommand(cmd, sizeof(cmd));
  radarSendCommand(LD_CMD_END_CONF, sizeof(LD_CMD_END_CONF));
}

void radarKick() {
#if RADAR_MODEL == RADAR_LD2410
  // The LD2410 streams on its own; it only goes quiet if stuck in config mode.
  radarSendCommand(LD_CMD_END_CONF, sizeof(LD_CMD_END_CONF));
#else
  radarSendCommand(CMD_MULTI_TARGET, sizeof(CMD_MULTI_TARGET));
#endif
}

void radarInit() {
  // Expanded RX buffer from 2048 to 4096 bytes to prevent frame truncation during WiFi SSL calls
  RadarSerial.setRxBufferSize(4096);
  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(200);
#if RADAR_MODEL == RADAR_LD2410 && LD2410_WRITE_CONFIG
  ld2410ApplyConfig();
#else
  radarKick();
#endif
  radarFlush();
}

void radarWatchdog() {
  if (!RADAR_CONNECTED) return;

  if (millis() - lastValidFrameMs > 1000) { readingStale = true; radarFps = 0.0; }

  if (millis() - lastValidFrameMs < 5000) return;
  Serial.println("Radar silent for 5 s, re-sending start command.");
  radarKick();
  radarFlush();
}

void dumpTrackFeatures() {
  Serial.printf("  radar %.1f fps%s\n", radarFps, readingStale ? "  [STALE]" : "");
#if RADAR_MODEL == RADAR_LD2410
  Serial.printf("  ld2410 state %u | moving %ucm e%u | still %ucm e%u | hits %u\n",
                ldState, ldMoveCm, ldMoveEnergy, ldStillCm, ldStillEnergy, ldHits);
  return;
#endif
  for (uint8_t i = 0; i < MAX_TRACKS; i++) {
    Track &t = tracks[i];
    if (!t.active) continue;
    Serial.printf("  trk%d %-10s score %3d | cad %.2fHz jit %.2f | turn %3.0f deg/s"
                  " | cont %.2f | ground %.0f radial %.0f cm/s jerk %.1f"
                  " | %.0fcm +-%.0fdeg (max %.0f) | drop %u\n",
                  i, CLASS_NAME[t.cls], t.score, t.cadenceHz, t.cadenceJitter,
                  t.turnRateDps, t.continuity, t.groundSpeed, t.meanSpeed,
                  t.jerk, t.fDistCm, t.fAngleDeg, t.maxRangeCm, t.outliers);
  }
}

// ================= Actuators =================
void applyLight(bool on) { lightOn = on; driveLight(on); }
void applyFan(bool on)   { fanOn   = on; driveFan(on); }

void reassertRelays() { driveLight(lightOn); driveFan(fanOn); }

// ================= Automation =================
void runAutomation() {
  unsigned long now = millis();

  if (RADAR_CONNECTED) {
    pollRadar();
    radarWatchdog();
  } else {
    bool pressed = (digitalRead(TEST_BUTTON_PIN) == LOW);
    if (pressed) { lastHumanMs = now; humanPresent = true; }
    else if (humanPresent && now - lastHumanMs >= holdMs) humanPresent = false;
    rejectReason = pressed ? "button" : "idle";
  }

  // Light delay logic: 3s delay on initial human detection
  if (humanPresent && !prevPresence) presenceSinceMs = now;
  prevPresence = humanPresent;

  bool lightAuto = humanPresent && (now - presenceSinceMs >= LIGHT_ON_DELAY_MS);
  lightPending   = humanPresent && !lightAuto;

  // Fan climate conditions with hysteresis
  bool hot, humid;
  if (fanEnvTrip) {
    hot   = temperatureC > (tempThreshold - TEMP_HYST);
    humid = humidityPct  > (humThreshold  - HUM_HYST);
  } else {
    hot   = temperatureC > tempThreshold;
    humid = humidityPct  > humThreshold;
  }

  fanEnvTrip = fanRequireBoth ? (hot && humid) : (hot || humid);
  bool fanAuto = humanPresent && dhtValid && fanEnvTrip;

  // Apply outputs respecting manual overrides from the frontend
  applyLight(lightOverride ? lightManualValue : lightAuto);
  applyFan(fanOverride ? fanManualValue : fanAuto);
}

// ================= Firebase =================
template <typename T>
bool readIfPresent(const char *path, T &dest) {
  T v = Database.get<T>(aClient, path);
  if (aClient.lastError().code() == 0) { dest = v; return true; }
  return false;
}

void seedgDefaults() {
  Database.set<bool>(aClient, DB_PATH "/lightOverride",    false);
  Database.set<bool>(aClient, DB_PATH "/fanOverride",      false);
  Database.set<bool>(aClient, DB_PATH "/lightManualValue", false);
  Database.set<bool>(aClient, DB_PATH "/fanManualValue",   false);
  Database.set<bool>(aClient, DB_PATH "/light",            false);
  Database.set<bool>(aClient, DB_PATH "/fan",              false);
  Database.set<bool>(aClient, DB_PATH "/occupancy",        false);
  Database.set<String>(aClient, DB_PATH "/mode",           String("auto"));

  bool b; float f; int i;

#if FORCE_DEFAULTS_ON_BOOT
  Database.set<float>(aClient, DB_PATH "/threshold",            tempThreshold);
  Database.set<float>(aClient, DB_PATH "/humidityThreshold",    humThreshold);
  Database.set<bool> (aClient, DB_PATH "/fanRequireBoth",       fanRequireBoth);
  Database.set<int>  (aClient, DB_PATH "/lightDelayMs",         (int)LIGHT_ON_DELAY_MS);
  // Force active-low defaults in Firebase so cloud values do not invert physical relays
  Database.set<bool> (aClient, DB_PATH "/relay/lightActiveLow", lightActiveLow);
  Database.set<bool> (aClient, DB_PATH "/relay/fanActiveLow",   fanActiveLow);
#else
  if (!readIfPresent<float>(DB_PATH "/threshold", f))
      Database.set<float>(aClient, DB_PATH "/threshold", tempThreshold);
  if (!readIfPresent<float>(DB_PATH "/humidityThreshold", f))
      Database.set<float>(aClient, DB_PATH "/humidityThreshold", humThreshold);
  if (!readIfPresent<bool>(DB_PATH "/fanRequireBoth", b))
      Database.set<bool>(aClient, DB_PATH "/fanRequireBoth", fanRequireBoth);
  if (!readIfPresent<bool>(DB_PATH "/relay/lightActiveLow", b))
      Database.set<bool>(aClient, DB_PATH "/relay/lightActiveLow", lightActiveLow);
  if (!readIfPresent<bool>(DB_PATH "/relay/fanActiveLow", b))
      Database.set<bool>(aClient, DB_PATH "/relay/fanActiveLow", fanActiveLow);
#endif

  if (!readIfPresent<float>(DB_PATH "/radar/distMin", f))
      Database.set<float>(aClient, DB_PATH "/radar/distMin", distMinCm);
  if (!readIfPresent<float>(DB_PATH "/radar/distMax", f))
      Database.set<float>(aClient, DB_PATH "/radar/distMax", distMaxCm);
  if (!readIfPresent<float>(DB_PATH "/radar/angleMax", f))
      Database.set<float>(aClient, DB_PATH "/radar/angleMax", angleMaxDeg);
  if (!readIfPresent<int>(DB_PATH "/radar/confirmFrames", i))
      Database.set<int>(aClient, DB_PATH "/radar/confirmFrames", (int)confirmFrames);
  if (!readIfPresent<int>(DB_PATH "/radar/holdMs", i))
      Database.set<int>(aClient, DB_PATH "/radar/holdMs", (int)holdMs);
  if (!readIfPresent<float>(DB_PATH "/radar/minStraightness", f))
      Database.set<float>(aClient, DB_PATH "/radar/minStraightness", minStraightness);
  if (!readIfPresent<float>(DB_PATH "/radar/stillRadiusMm", f))
      Database.set<float>(aClient, DB_PATH "/radar/stillRadiusMm", stillRadiusMm);

  Database.set<String>(aClient, DB_PATH "/radar/model", String(RADAR_NAME));
  if (!readIfPresent<int>(DB_PATH "/radar/ld2410/moveEnergyMin", i))
      Database.set<int>(aClient, DB_PATH "/radar/ld2410/moveEnergyMin", (int)ldMoveEnergyMin);
  if (!readIfPresent<int>(DB_PATH "/radar/ld2410/stillEnergyMin", i))
      Database.set<int>(aClient, DB_PATH "/radar/ld2410/stillEnergyMin", (int)ldStillEnergyMin);
  if (!readIfPresent<bool>(DB_PATH "/radar/ld2410/useStill", b))
      Database.set<bool>(aClient, DB_PATH "/radar/ld2410/useStill", ldUseStill);

  if (!readIfPresent<float>(DB_PATH "/classify/gaitHzMin", f))
      Database.set<float>(aClient, DB_PATH "/classify/gaitHzMin", gaitHzMin);
  if (!readIfPresent<float>(DB_PATH "/classify/gaitHzMax", f))
      Database.set<float>(aClient, DB_PATH "/classify/gaitHzMax", gaitHzMax);
  if (!readIfPresent<float>(DB_PATH "/classify/turnRateHuman", f))
      Database.set<float>(aClient, DB_PATH "/classify/turnRateHuman", turnRateHumanDps);
  if (!readIfPresent<float>(DB_PATH "/classify/turnRateAnimal", f))
      Database.set<float>(aClient, DB_PATH "/classify/turnRateAnimal", turnRateAnimalDps);
  if (!readIfPresent<float>(DB_PATH "/classify/continuityHuman", f))
      Database.set<float>(aClient, DB_PATH "/classify/continuityHuman", continuityHumanMin);
  if (!readIfPresent<float>(DB_PATH "/classify/continuityAnimal", f))
      Database.set<float>(aClient, DB_PATH "/classify/continuityAnimal", continuityAnimalMax);
  if (!readIfPresent<int>(DB_PATH "/classify/humanScoreMin", i))
      Database.set<int>(aClient, DB_PATH "/classify/humanScoreMin", (int)humanScoreMin);
  if (!readIfPresent<int>(DB_PATH "/classify/animalScoreMax", i))
      Database.set<int>(aClient, DB_PATH "/classify/animalScoreMax", (int)animalScoreMax);
  if (!readIfPresent<bool>(DB_PATH "/classify/unknownCountsAsHuman", b))
      Database.set<bool>(aClient, DB_PATH "/classify/unknownCountsAsHuman", unknownCountsAsHuman);
  if (!readIfPresent<bool>(DB_PATH "/classify/enabled", b))
      Database.set<bool>(aClient, DB_PATH "/classify/enabled", classifierEnabled);
  if (!readIfPresent<bool>(DB_PATH "/relay/lightActiveLow", b))
      Database.set<bool>(aClient, DB_PATH "/relay/lightActiveLow", lightActiveLow);
  if (!readIfPresent<bool>(DB_PATH "/relay/fanActiveLow", b))
      Database.set<bool>(aClient, DB_PATH "/relay/fanActiveLow", fanActiveLow);
}

#define RADAR_TICK() do { if (RADAR_CONNECTED) pollRadar(); } while (0)

void syncFirebase() {
  if (!app.ready()) return;

  Database.set<float>(aClient, DB_PATH "/temperature", temperatureC);   RADAR_TICK();
  Database.set<float>(aClient, DB_PATH "/humidity",    humidityPct);    RADAR_TICK();
  Database.set<bool>(aClient, DB_PATH "/occupancy",    humanPresent);   RADAR_TICK();
  Database.set<bool>(aClient, DB_PATH "/light",        lightOn);        RADAR_TICK();
  Database.set<bool>(aClient, DB_PATH "/fan",          fanOn);          RADAR_TICK();
  Database.set<bool>(aClient, DB_PATH "/lightPending", lightPending);   RADAR_TICK();
  Database.set<bool>(aClient, DB_PATH "/envTrip",      fanEnvTrip);     RADAR_TICK();
  Database.set<bool>(aClient, DB_PATH "/dhtOk",        dhtValid);       RADAR_TICK();
  Database.set<String>(aClient, DB_PATH "/mode",
                       String((lightOverride || fanOverride) ? "manual" : "auto"));
  RADAR_TICK();

#if RADAR_MODEL == RADAR_LD2410
  // The LD2410 has no x/y tracks, so the classifier fields are not published.
  if (RADAR_CONNECTED) {
    Database.set<String>(aClient, DB_PATH "/radar/class",       String(humanPresent ? "unknown" : "none")); RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/fps",         radarFps);            RADAR_TICK();
    Database.set<bool>  (aClient, DB_PATH "/radar/stale",       readingStale);        RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/distanceCM",        targetDistanceCM);    RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/targetState", (int)ldState);        RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/moveCm",      (int)ldMoveCm);       RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/moveEnergy",  (int)ldMoveEnergy);   RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/stillCm",     (int)ldStillCm);      RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/stillEnergy", (int)ldStillEnergy);  RADAR_TICK();
    Database.set<bool>  (aClient, DB_PATH "/radar/rawTarget",   rawRadarTarget);      RADAR_TICK();
    Database.set<String>(aClient, DB_PATH "/radar/state",       String(rejectReason));
    RADAR_TICK();
  }
#else
  if (RADAR_CONNECTED) {
    const char *cls = (confirmedTrack >= 0) ? CLASS_NAME[tracks[confirmedTrack].cls]
                                            : (animalPresent ? "animal" : "none");
    Database.set<String>(aClient, DB_PATH "/radar/class",      String(cls));         RADAR_TICK();
    Database.set<bool>  (aClient, DB_PATH "/radar/animal",     animalPresent);       RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/humanScore", (int)targetScore);    RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/cadenceHz",  targetCadenceHz);     RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/turnDps",    targetTurnDps);       RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/fps",        radarFps);            RADAR_TICK();
    Database.set<bool>  (aClient, DB_PATH "/radar/stale",      readingStale);        RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/groundCms",  targetGroundCms);     RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/distanceCM",       targetDistanceCM);    RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/angleDeg",         targetAngleDeg);      RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/speedCms",   targetSpeedCms);      RADAR_TICK();
    Database.set<float> (aClient, DB_PATH "/radar/ratio",      targetRatio);         RADAR_TICK();
    Database.set<int>   (aClient, DB_PATH "/radar/track",      (int)confirmedTrack); RADAR_TICK();
    Database.set<bool>  (aClient, DB_PATH "/radar/rawTarget",  rawRadarTarget);      RADAR_TICK();
    Database.set<String>(aClient, DB_PATH "/radar/state",      String(rejectReason));
    RADAR_TICK();
  }
#endif

  readIfPresent<bool> (DB_PATH "/lightOverride",     lightOverride);    RADAR_TICK();
  readIfPresent<bool> (DB_PATH "/lightManualValue",  lightManualValue); RADAR_TICK();
  readIfPresent<bool> (DB_PATH "/fanOverride",       fanOverride);      RADAR_TICK();
  readIfPresent<bool> (DB_PATH "/fanManualValue",    fanManualValue);   RADAR_TICK();
  readIfPresent<float>(DB_PATH "/threshold",         tempThreshold);    RADAR_TICK();
  readIfPresent<float>(DB_PATH "/humidityThreshold", humThreshold);     RADAR_TICK();
  readIfPresent<bool> (DB_PATH "/fanRequireBoth",    fanRequireBoth);   RADAR_TICK();

  if (++syncCycle >= 5) {
    syncCycle = 0;
    int tmp;
    readIfPresent<float>(DB_PATH "/radar/distMin",         distMinCm);       RADAR_TICK();
    readIfPresent<float>(DB_PATH "/radar/distMax",         distMaxCm);       RADAR_TICK();
    readIfPresent<float>(DB_PATH "/radar/angleMax",        angleMaxDeg);     RADAR_TICK();
    readIfPresent<float>(DB_PATH "/radar/minStraightness", minStraightness); RADAR_TICK();
    readIfPresent<float>(DB_PATH "/radar/stillRadiusMm",   stillRadiusMm);   RADAR_TICK();
    if (readIfPresent<int>(DB_PATH "/radar/confirmFrames", tmp) && tmp > 0)
        confirmFrames = (uint16_t)tmp;
    RADAR_TICK();
    if (readIfPresent<int>(DB_PATH "/radar/holdMs", tmp) && tmp > 0)
        holdMs = (uint32_t)tmp;
    RADAR_TICK();
    if (readIfPresent<int>(DB_PATH "/radar/ld2410/moveEnergyMin", tmp) && tmp >= 0 && tmp <= 100)
        ldMoveEnergyMin = (uint8_t)tmp;
    RADAR_TICK();
    if (readIfPresent<int>(DB_PATH "/radar/ld2410/stillEnergyMin", tmp) && tmp >= 0 && tmp <= 100)
        ldStillEnergyMin = (uint8_t)tmp;
    RADAR_TICK();
    readIfPresent<bool>(DB_PATH "/radar/ld2410/useStill", ldUseStill); RADAR_TICK();

    readIfPresent<float>(DB_PATH "/classify/gaitHzMin",        gaitHzMin);           RADAR_TICK();
    readIfPresent<float>(DB_PATH "/classify/gaitHzMax",        gaitHzMax);           RADAR_TICK();
    readIfPresent<float>(DB_PATH "/classify/turnRateHuman",    turnRateHumanDps);    RADAR_TICK();
    readIfPresent<float>(DB_PATH "/classify/turnRateAnimal",   turnRateAnimalDps);   RADAR_TICK();
    readIfPresent<float>(DB_PATH "/classify/continuityHuman",  continuityHumanMin);  RADAR_TICK();
    readIfPresent<float>(DB_PATH "/classify/continuityAnimal", continuityAnimalMax); RADAR_TICK();
    readIfPresent<bool> (DB_PATH "/classify/unknownCountsAsHuman", unknownCountsAsHuman);
    RADAR_TICK();
    readIfPresent<bool> (DB_PATH "/classify/enabled", classifierEnabled);
    RADAR_TICK();

    bool prevL = lightActiveLow, prevF = fanActiveLow;
    readIfPresent<bool>(DB_PATH "/relay/lightActiveLow", lightActiveLow); RADAR_TICK();
    readIfPresent<bool>(DB_PATH "/relay/fanActiveLow",   fanActiveLow);   RADAR_TICK();
    if (prevL != lightActiveLow || prevF != fanActiveLow) {
      Serial.println("Relay polarity updated, re-asserting outputs.");
      reassertRelays();
    }
    if (readIfPresent<int>(DB_PATH "/classify/humanScoreMin", tmp) && tmp > 0 && tmp <= 100)
        humanScoreMin = (uint8_t)tmp;
    RADAR_TICK();
    if (readIfPresent<int>(DB_PATH "/classify/animalScoreMax", tmp) && tmp >= 0 && tmp < 100)
        animalScoreMax = (uint8_t)tmp;
    RADAR_TICK();
  }

  if (aClient.lastError().code() != 0) {
    Serial.printf("Firebase sync error: %s (code %d)\n",
                  aClient.lastError().message().c_str(),
                  aClient.lastError().code());
  }
}

void processData(AsyncResult &aResult) {
  if (!aResult.isResult()) return;
  if (aResult.isError())
    Serial.printf("Firebase error [%s]: %s (code %d)\n", aResult.uid().c_str(),
                  aResult.error().message().c_str(), aResult.error().code());
}

// ================= Setup and Loop =================
void setup() {
  digitalWrite(RELAY_LIGHT_PIN, lightActiveLow ? HIGH : LOW);
  digitalWrite(RELAY_FAN_PIN,   fanActiveLow   ? HIGH : LOW);
  pinMode(RELAY_LIGHT_PIN, OUTPUT);
  pinMode(RELAY_FAN_PIN,   OUTPUT);
  driveLight(false);
  driveFan(false);
  lightOn = false;
  fanOn   = false;

  Serial.begin(SERIAL_MONITOR_BAUD);
  delay(600);

#if RELAY_SELFTEST
  Serial.println();
  Serial.println("Relay self test:");
  Serial.printf("  light ON  (pin %s)\n", lightActiveLow ? "LOW" : "HIGH");
  driveLight(true);  delay(1200);
  Serial.println("  light OFF");
  driveLight(false); delay(800);
  Serial.printf("  fan ON    (pin %s)\n", fanActiveLow ? "LOW" : "HIGH");
  driveFan(true);    delay(1200);
  Serial.println("  fan OFF");
  driveFan(false);   delay(800);
  Serial.println();
#endif

  dht.begin();

  for (uint8_t i = 0; i < MAX_TRACKS; i++) clearTrack(i);

  if (RADAR_CONNECTED) {
    radarInit();
    Serial.println("Radar: " RADAR_NAME " UART2 initialized.");
  } else {
    pinMode(TEST_BUTTON_PIN, INPUT_PULLUP);
    Serial.println("Debug mode: GPIO33 button simulates presence.");
  }

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.println();
  Serial.print("Connected, IP: "); Serial.println(WiFi.localIP());

  ssl_client.setInsecure();
  ssl_client.setConnectionTimeout(10000);
  ssl_client.setHandshakeTimeout(15);

  initializeApp(aClient, app, getAuth(user_auth), processData, "authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  unsigned long t0 = millis();
  while (!app.ready() && millis() - t0 < 15000) { app.loop(); delay(50); }
  if (app.ready()) seedgDefaults();

  if (RADAR_CONNECTED) radarFlush();

  Serial.println("Boot complete. Waiting for occupant.");
}

void loop() {
  app.loop();

  // Run radar polling and relay logic continuously every loop cycle
  runAutomation();

  unsigned long now = millis();

  // Non-blocking DHT22 reading every 2 seconds
  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      tBuf[dhtIdx] = t;  hBuf[dhtIdx] = h;
      dhtIdx = (dhtIdx + 1) % 3;
      if (dhtSamples < 3) dhtSamples++;

      if (dhtSamples == 3) {
        float tm = median3(tBuf[0], tBuf[1], tBuf[2]);
        float hm = median3(hBuf[0], hBuf[1], hBuf[2]);
        if (!dhtValid) {
          temperatureC = tm;
          humidityPct  = hm;
        } else {
          temperatureC = temperatureC * 0.7 + tm * 0.3;
          humidityPct  = humidityPct  * 0.7 + hm * 0.3;
        }
        dhtValid = true;
      }
      dhtFailCount = 0;
    } else if (++dhtFailCount >= 3) {
      dhtValid = false;
      dhtSamples = 0;
    }

    Serial.printf("T %.1fC  RH %.1f%%%s  human %s  animal %s  (%s)",
                  temperatureC, humidityPct, dhtValid ? "" : " [DHT fault]",
                  humanPresent ? "YES" : "no",
                  animalPresent ? "YES" : "no", rejectReason);
    Serial.printf("  light %s[pin %s]%s  fan %s[pin %s]  mode %s  cls %s\n",
                  lightOn ? "ON " : "off ",
                  digitalRead(RELAY_LIGHT_PIN) ? "HIGH" : "LOW",
                  lightPending ? " (3s delay)" : "",
                  fanOn ? "ON " : "off ",
                  digitalRead(RELAY_FAN_PIN) ? "HIGH" : "LOW",
                  (lightOverride || fanOverride) ? "MANUAL" : "auto",
                  classifierEnabled ? "on" : "OFF");
  }

  if (RADAR_DEBUG && RADAR_CONNECTED && now - lastDebugMs >= 2000) {
    lastDebugMs = now;
    dumpTrackFeatures();
  }

  if (app.ready() && now - lastFirebaseSync >= FIREBASE_SYNC_INTERVAL) {
    lastFirebaseSync = now;
    syncFirebase();
  }

  delay(2);
}