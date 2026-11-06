# Hardware

| Part | Details | Status |
|---|---|---|
| Computer | Raspberry Pi 5 | Have it. Use the official 27 W USB-C PSU (weaker supplies brown out under load). |
| Storage | microSD (≥32 GB, A2 preferred) | Ubuntu 24.04.5 LTS, hostname `rover`, user `rover` |
| LiDAR | **Slamtec RPLIDAR C1**, 2D 360°, USB-C | Identified 2026-10-24 via `tools/lidar_identify.py`: 460800 baud, model byte 0x41, FW 1.02, HW rev 18, S/N A2CCE195C1E79ED6B5E29EF1261B4A73, health Good. Shows up as `/dev/rplidar` → `ttyUSB0`. |
| Camera | TBD | For AprilTags |
| Drive base | **ESP32** + 4 mecanum wheels, 2-wire motor driver channels, no encoders, Bluetooth controller (Bluepad32) | Pins + details in `firmware/WIRING.md`. Pi links to it over USB (`docs/06_drivetrain_link.md`). |

## LiDAR: how to tell the model

Each RPLIDAR model uses a fixed baud rate:

| Baud that answers | Likely model |
|---|---|
| 115200 | A1 (A1M8) or A2M8 |
| 256000 | A2M12, A3, S1 |
| 460800 | C1 (ships with a USB-C adapter, so a good guess) |
| 1000000 | S2 / S3 |

The USB adapter is normally a Silicon Labs CP210x (VID:PID 10C4:EA60).
On Windows it needs the CP210x driver; on Ubuntu it works out of the box as
`/dev/ttyUSB0`, and our udev rule also creates `/dev/rplidar`.

**Cable note:** use a USB-C *data* cable. Charge-only cables are common and
the LiDAR just won't show up.
