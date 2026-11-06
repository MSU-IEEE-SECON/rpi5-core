# Drivetrain wiring

Known from `mecanum.ino`; please fill in the `?`s.

| Item | Details |
|---|---|
| Microcontroller | **ESP32** (dev board model: ?), Bluepad32 board package |
| USB-serial chip on the ESP32 board | ? (CP2102 or CH340: printed on the chip near the USB port) |
| Motor driver board(s) | ? (2 input wires per motor: e.g. L298N, TB6612, DRV8833, MX1508?) |
| Motors | 4 × ? (voltage, gear ratio), **mecanum wheels**, no encoders |
| Controller | ? (Bluepad32: Xbox / PS4 / PS5 / Switch Pro ...) |
| Battery | ? V, what it powers; how the ESP32 is powered |

## ESP32 pin map (from the code)

| Motor | Wire 1 (forward PWM) | Wire 2 (reverse PWM) |
|---|---|---|
| Front left | GPIO 2 | GPIO 4 |
| Front right | GPIO 33 | GPIO 25 |
| Back left | GPIO 16 | GPIO 17 |
| Back right | GPIO 26 | GPIO 27 |

Note: GPIO 2 is an ESP32 "strapping" pin (and often the on-board LED). It works,
but if the board ever fails to boot/flash with the motor driver attached, that pin
is the first suspect.

## Pi connection
USB cable from the ESP32's USB port → any Pi USB port. Shared ground comes
through the USB cable.

(Photos of the wiring are welcome too: drop them in this folder.)
