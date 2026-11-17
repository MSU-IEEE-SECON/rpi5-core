#!/usr/bin/env python3
"""Reset the ESP32 through its USB-serial chip (EN pin via RTS, like the reset button).

    python3 tools/esp32_reset.py [/dev/esp32]

The team's micro-ROS firmware only connects to an agent it finds after booting; after the
agent restarts it can sit there disconnected. lap.launch resets it right before the agent
starts so every run connects.
"""
import sys
import time

import serial

port = sys.argv[1] if len(sys.argv) > 1 else '/dev/esp32'
try:
    s = serial.Serial()
    s.port, s.baudrate = port, 115200
    s.dtr = False        # IO0 high: normal boot, not the flashing bootloader
    s.rts = False
    s.open()
    s.rts = True         # EN low: hold in reset
    time.sleep(0.15)
    s.rts = False        # release: boot
    s.close()
    print(f'reset ESP32 on {port}')
except Exception as e:  # noqa: BLE001
    print(f'could not reset ESP32 on {port}: {e}')
