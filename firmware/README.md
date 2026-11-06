# firmware/ — drivetrain ESP32 code

```
firmware/
├── mecanum/mecanum.ino            ← ORIGINAL manual-control sketch (unchanged, reference)
├── mecanum_auto/                  ← USE THIS: original + AUTO mode (Pi drives over USB)
│   ├── mecanum_auto.ino           ← additions marked // [AUTO]
│   └── auto_drive.h               ← protocol parsing + wheel mixing
├── test/                          ← PC-side tests (not uploaded to the ESP32)
│   ├── test_firmware.cpp
│   └── stubs/                     ← fake Arduino.h / Bluepad32.h for the tests
└── WIRING.md                      ← pins, boards, power (please fill in the ?s)
```

Full guide: `docs/06_drivetrain_link.md`.

## Controls (mecanum_auto)
- Sticks: manual, same as before
- **Y**: toggle AUTO (Pi drives) · **B**: back to manual · **touch a stick**: back to manual
- AUTO stops the motors if the Pi goes quiet for 300 ms, and needs the controller connected

## Protocol (USB serial, 115200)
Pi → ESP32, ~20×/s: `D <vx> <vy> <wz>` each −1.0…1.0 of full speed
(vx + forward, vy + left/strafe, wz + turn left/CCW).
ESP32 → Pi, 5×/s: `S <MANUAL|AUTO> <fl> <fr> <bl> <br>` (wheel PWMs).
Anything else the ESP32 prints is treated as log text.

## Run the tests (any PC with g++, from the repo root)
```
g++ -std=c++17 -I firmware/test/stubs -I firmware/mecanum_auto -x c++ firmware/test/test_firmware.cpp -o fwtest && ./fwtest
```

## Adding / updating code (teammate)
On GitHub: open the folder → **Add file → Upload files** → drag files in →
**Commit changes**. Or with git: `git pull`, edit, `git add -A`, `git commit -m "..."`, `git push`.

If you change the manual driving code, please make the same change in both
sketches (or just switch to `mecanum_auto` as the only one).
