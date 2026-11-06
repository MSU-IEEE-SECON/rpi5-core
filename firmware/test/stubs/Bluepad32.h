// Host-side stand-in for Bluepad32, for testing only.
#pragma once
#include "Arduino.h"
#define BP32_MAX_CONTROLLERS 4
#define DPAD_UP 1
#define DPAD_DOWN 2
#define DPAD_RIGHT 4
#define DPAD_LEFT 8
struct Controller {
  bool connected = true; int ax = 0, ay = 0, arx = 0; bool by = false, bb = false; uint8_t dp = 0;
  bool isConnected() { return connected; }
  int axisX() { return ax; } int axisY() { return ay; } int axisRX() { return arx; }
  bool y() { return by; } bool b() { return bb; } uint8_t dpad() { return dp; }
};
typedef Controller* ControllerPtr;
typedef void (*CtlCb)(ControllerPtr);
struct BP32Class {
  CtlCb on = nullptr, off = nullptr;
  void setup(CtlCb a, CtlCb b) { on = a; off = b; }
  void forgetBluetoothKeys() {}
  void update() {}
};
extern BP32Class BP32;
