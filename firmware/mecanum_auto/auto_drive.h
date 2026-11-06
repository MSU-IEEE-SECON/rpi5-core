// auto_drive.h — Pi <-> ESP32 drive protocol + mecanum mixing for AUTO mode.
//
// Plain C++ (no Arduino calls) so it can be unit-tested on a PC:
//   g++ -std=c++17 -I firmware/mecanum_auto firmware/test/test_auto_drive.cpp && ./a.out
//
// Protocol (Pi -> ESP32), one line, 115200 baud, ~20 times a second:
//     D <vx> <vy> <wz>\n
//   all values normalised to -1.0 .. 1.0 (fraction of full speed):
//     vx = forward(+) / backward(-)
//     vy = left(+)    / right(-)      (strafe; mecanum only)
//     wz = turn left / counter-clockwise(+) / turn right(-)
//   The Pi does the m/s -> fraction scaling, so speed calibration lives in
//   ROS params, not in firmware.
#pragma once
#include <math.h>
#include <stdlib.h>

struct DriveCmd {
  float vx, vy, wz;
};

struct Wheels {
  int fl, fr, bl, br;
};

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Parse "D <vx> <vy> <wz>". Returns false (and leaves *out alone) on anything malformed.
static inline bool parseDriveLine(const char* line, DriveCmd* out) {
  if (!line || line[0] != 'D' || line[1] != ' ') return false;
  const char* p = line + 2;
  char* end;
  float v[3];
  for (int i = 0; i < 3; i++) {
    v[i] = strtof(p, &end);
    if (end == p || !isfinite(v[i])) return false;
    p = end;
  }
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '\0') return false;  // trailing junk -> reject
  out->vx = clampf(v[0], -1.0f, 1.0f);
  out->vy = clampf(v[1], -1.0f, 1.0f);
  out->wz = clampf(v[2], -1.0f, 1.0f);
  return true;
}

// Raise any non-zero wheel command to at least minPwm so the motors actually
// turn (small PWM values just hum). Keeps sign; 0 stays 0.
static inline int applyMinPwm(int pwm, int minPwm) {
  if (pwm == 0) return 0;
  int mag = abs(pwm);
  int scaled = minPwm + (mag * (255 - minPwm)) / 255;
  if (scaled > 255) scaled = 255;
  return pwm > 0 ? scaled : -scaled;
}

// Same mixing formula as the manual code (throttle/strafe/rotation in PWM units,
// strafe + = right, rotation + = clockwise), but scaled down together if any
// wheel would exceed 255, so the direction of travel is kept.
static inline Wheels mixMecanum(float throttle, float strafe, float rotation) {
  float fl = throttle + strafe + rotation;
  float fr = throttle - strafe - rotation;
  float bl = throttle - strafe + rotation;
  float br = throttle + strafe - rotation;
  float m = fmaxf(fmaxf(fabsf(fl), fabsf(fr)), fmaxf(fabsf(bl), fabsf(br)));
  if (m > 255.0f) {
    float k = 255.0f / m;
    fl *= k; fr *= k; bl *= k; br *= k;
  }
  Wheels w = {(int)lroundf(fl), (int)lroundf(fr), (int)lroundf(bl), (int)lroundf(br)};
  return w;
}

// AUTO command -> wheel PWMs. Converts from ROS sign conventions to the
// manual code's conventions: ROS +vy is LEFT (manual strafe + is right),
// ROS +wz is CCW/left (manual rotation + is clockwise).
static inline Wheels autoWheels(const DriveCmd& c, int minPwm) {
  Wheels w = mixMecanum(c.vx * 255.0f, -c.vy * 255.0f, -c.wz * 255.0f);
  w.fl = applyMinPwm(w.fl, minPwm);
  w.fr = applyMinPwm(w.fr, minPwm);
  w.bl = applyMinPwm(w.bl, minPwm);
  w.br = applyMinPwm(w.br, minPwm);
  return w;
}
