#ifndef RADAR_TYPE_H
#define RADAR_TYPE_H

#include <Arduino.h>

// ================= Classification =================
enum TargetClass : uint8_t {
  CLS_NONE = 0, CLS_UNKNOWN = 1, CLS_MECHANICAL = 2, CLS_ANIMAL = 3, CLS_HUMAN = 4
};
static const char *CLASS_NAME[] = { "none", "unknown", "mechanical", "animal", "human" };

// ================= Radar Target =================
struct RadarTarget {
  bool  valid;
  float xMm, yMm, speedCms, distCm, angleDeg;
};

// ================= Position Based Tracks =================
#define MAX_TRACKS 3
#define SPD_HIST   48

struct Track {
  bool     active;
  float    x, y;
  float    speedCms, distCm, angleDeg;
  uint16_t hits, misses;

  unsigned long winStartMs;
  float    winStartX, winStartY;
  float    pathMm;
  float    minX, maxX, minY, maxY;
  float    ratio;
  bool     motionOk;

  float    spdHist[SPD_HIST];
  uint32_t spdTime[SPD_HIST];
  uint8_t  spdCount, spdHead;

  float    lastHeading;
  bool     headingValid;
  float    lastSpeed;

  float    fx, fy;
  float    vx, vy;
  bool     filtReady;
  float    groundSpeed;
  unsigned long lastFrameMs;
  float    dtEma;
  uint16_t outliers;

  float    turnRateDps;
  float    jerk;
  float    continuity;
  float    maxRangeCm;

  unsigned long bornMs;
  unsigned long lastEvalMs;

  float    cadenceHz, cadenceJitter, meanSpeed, speedStd;
  float    fDistCm, fAngleDeg;
  uint8_t  score;
  uint8_t  cls;
  uint8_t  candidate;
  uint8_t  candidateN;
  bool     stickyHuman;
};

#endif // RADAR_TYPE_H