// Minimal host-side stand-ins for the Arduino API, for testing only.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <deque>
#include <map>
#define OUTPUT 1
template <class T, class L, class H> T constrain(T v, L lo, H hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; }
extern unsigned long g_millis;
inline unsigned long millis() { return g_millis; }
inline void delay(unsigned long ms) { g_millis += ms; }
extern std::map<int,int> g_pwm;
inline void analogWrite(int pin, int v) { g_pwm[pin] = v; }
inline void pinMode(int, int) {}
struct FakeSerial {
  std::deque<char> in; std::string out;
  void begin(long) {}
  int available() { return (int)in.size(); }
  int read() { if (in.empty()) return -1; char c = in.front(); in.pop_front(); return c; }
  template <class T> void print(T v) { out += std::to_string(v); }
  void print(const char* s) { out += s; }
  void print(char c) { out += c; }
  template <class T> void println(T v) { print(v); out += "\n"; }
  void println(const char* s) { out += s; out += "\n"; }
  void feed(const std::string& s) { for (char c : s) in.push_back(c); }
};
extern FakeSerial Serial;
