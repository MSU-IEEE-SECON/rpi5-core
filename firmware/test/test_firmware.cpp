// Host-side tests for the ESP32 AUTO-mode firmware. Not uploaded to the robot.
//
// Run from the repo root:
//   g++ -std=c++17 -I firmware/test/stubs -I firmware/mecanum_auto
//       -x c++ firmware/test/test_firmware.cpp -o fwtest && ./fwtest
//
// Part 1 tests auto_drive.h (parsing + mixing).
// Part 2 compiles the real mecanum_auto.ino against fake Arduino/Bluepad32
// headers and drives it through scenarios: press Y, send commands, stop, override...
#include <cassert>
#include <cstdio>
#include <string>

#include "Arduino.h"
#include "Bluepad32.h"

unsigned long g_millis = 0;
std::map<int, int> g_pwm;
FakeSerial Serial;
BP32Class BP32;

// The sketch itself (Arduino adds its own main; we provide ours)
#include "../mecanum_auto/mecanum_auto.ino"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { printf("  FAIL line %d: %s\n", __LINE__, #cond); fails++; } } while (0)

// signed motor value from the two PWM pins of a motor
static int motor(int p1, int p2) { return g_pwm[p1] - g_pwm[p2]; }
static int FLv() { return motor(FL_1, FL_2); }
static int FRv() { return motor(FR_1, FR_2); }
static int BLv() { return motor(BL_1, BL_2); }
static int BRv() { return motor(BR_1, BR_2); }
static bool allStopped() { return FLv() == 0 && FRv() == 0 && BLv() == 0 && BRv() == 0; }

static void test_parse() {
  puts("parse");
  DriveCmd c{9, 9, 9};
  CHECK(parseDriveLine("D 0.5 -0.25 0.1", &c) && c.vx == 0.5f && c.vy == -0.25f);
  CHECK(parseDriveLine("D 2 -3 0", &c) && c.vx == 1.0f && c.vy == -1.0f);   // clamped
  CHECK(parseDriveLine("D 0.1 0.2 0.3  ", &c));                              // trailing spaces ok
  c = {7, 7, 7};
  CHECK(!parseDriveLine("D 0.1 0.2", &c) && c.vx == 7);                      // missing value
  CHECK(!parseDriveLine("D 0.1 0.2 0.3 x", &c));                            // junk
  CHECK(!parseDriveLine("X 0.1 0.2 0.3", &c));
  CHECK(!parseDriveLine("D nan 0 0", &c));
  CHECK(!parseDriveLine("Translation Sensitivity: 1.1", &c));
  CHECK(!parseDriveLine("", &c));
}

static void test_mix() {
  puts("mix");
  // forward: all wheels forward, equal
  Wheels w = autoWheels({1, 0, 0}, 0);
  CHECK(w.fl == 255 && w.fr == 255 && w.bl == 255 && w.br == 255);
  // ROS +wz (turn LEFT/CCW): left wheels backward, right wheels forward
  w = autoWheels({0, 0, 0.5f}, 0);
  CHECK(w.fl < 0 && w.bl < 0 && w.fr > 0 && w.br > 0);
  // ROS +vy (strafe LEFT): fl,br backward; fr,bl forward (mecanum)
  w = autoWheels({0, 0.5f, 0}, 0);
  CHECK(w.fl < 0 && w.br < 0 && w.fr > 0 && w.bl > 0);
  // forward + hard left turn saturates -> scaled, nothing exceeds 255, still turning left
  w = autoWheels({1, 0, 1}, 0);
  CHECK(abs(w.fl) <= 255 && abs(w.fr) <= 255 && w.fr > w.fl);
  // min pwm: small commands get lifted, zero stays zero
  CHECK(applyMinPwm(0, 60) == 0);
  CHECK(applyMinPwm(10, 60) >= 60 && applyMinPwm(-10, 60) <= -60);
  CHECK(applyMinPwm(255, 60) == 255);
}

static Controller ctl;

static void tick(int n = 1) { for (int i = 0; i < n; i++) loop(); }
static void pressY() { ctl.by = true; tick(); ctl.by = false; tick(); }
static void piSend(float vx, float vy, float wz) {
  char buf[64];
  snprintf(buf, sizeof buf, "D %.3f %.3f %.3f\n", vx, vy, wz);
  Serial.feed(buf);
}

static void test_sketch() {
  puts("sketch scenarios");
  setup();
  CHECK(allStopped());

  // no controller yet: Pi commands are ignored
  piSend(0.5, 0, 0); tick(3);
  CHECK(!autoMode && allStopped());

  BP32.on(&ctl);
  tick();
  CHECK(!autoMode);

  // manual still works exactly like before: stick forward -> all forward
  ctl.ay = -511; tick();
  CHECK(FLv() > 200 && FRv() > 200 && BLv() > 200 && BRv() > 200);
  ctl.ay = 0; tick();
  CHECK(allStopped());

  // Y -> AUTO, motors stay stopped until the Pi speaks
  pressY();
  CHECK(autoMode && allStopped());
  CHECK(Serial.out.find("S AUTO") != std::string::npos);

  // Pi says forward
  piSend(0.5, 0, 0); tick();
  CHECK(FLv() > 0 && FLv() == FRv() && FRv() == BLv() && BLv() == BRv());

  // Pi says turn left -> right side faster than left
  piSend(0.3, 0, 0.4); tick();
  CHECK(FRv() > FLv() && BRv() > BLv());

  // watchdog: Pi goes quiet for > 300 ms -> stop, but stay in AUTO
  for (int i = 0; i < 25; i++) tick();   // 25 * 15 ms = 375 ms
  CHECK(autoMode && allStopped());
  piSend(0.4, 0, 0); tick();
  CHECK(FLv() > 0);                       // resumes on fresh command

  // stick override -> MANUAL immediately, robot follows the stick
  ctl.arx = 300; tick();
  CHECK(!autoMode);
  ctl.arx = 0; tick();
  piSend(0.8, 0, 0); tick(2);
  CHECK(allStopped());                    // Pi ignored in MANUAL

  // B drops out of AUTO
  pressY(); CHECK(autoMode);
  ctl.bb = true; tick(); ctl.bb = false; tick();
  CHECK(!autoMode && allStopped());

  // Y twice toggles back off
  pressY(); CHECK(autoMode);
  pressY(); CHECK(!autoMode);

  // controller disconnect in AUTO -> MANUAL + stopped
  pressY(); piSend(0.5, 0, 0); tick();
  CHECK(autoMode && FLv() > 0);
  BP32.off(&ctl); tick();
  CHECK(!autoMode && allStopped());

  // garbage on the serial line doesn't crash or move anything
  BP32.on(&ctl); tick(); pressY();
  Serial.feed(std::string(500, 'Z') + "\nD 1 1\nhello\n"); tick(3);
  CHECK(autoMode && allStopped());

  // status lines are being sent periodically
  Serial.out.clear(); tick(20);
  CHECK(Serial.out.find("S AUTO") != std::string::npos);
}

int main() {
  test_parse();
  test_mix();
  test_sketch();
  printf(fails ? "\n%d FAILURE(S)\n" : "\nALL FIRMWARE TESTS PASSED\n", fails);
  return fails ? 1 : 0;
}
