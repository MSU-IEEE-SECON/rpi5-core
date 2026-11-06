# 04 — LiDAR bring-up

## 1. Identify the model (sticker is gone)

On the laptop *or* the Pi (needs `pip install pyserial` / `sudo apt install python3-serial`):

```bash
python3 tools/lidar_identify.py /dev/rplidar      # Pi
python tools\lidar_identify.py COM5               # Windows (check Device Manager → Ports)
```

It tries each RPLIDAR baud rate and prints the one that answers, plus
firmware, hardware rev, serial number, and health. It does **not** spin the motor.

## 2. Set the baud rate

Edit `ros2_ws/src/rover_bringup/config/lidar.yaml` → `serial_baudrate`
to the value the script found (115200 / 256000 / 460800 / 1000000).
Then rebuild on the Pi:

```bash
cd ~/rover/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

(Or override at launch time: `serial_baudrate:=460800`.)

## 3. Run it

```bash
ros2 launch rover_bringup lidar.launch.py
```

In a second SSH session:

```bash
ros2 topic hz /scan                  # should be ~5–10 Hz
ros2 topic echo /scan --once | head -30
ros2 run rover_bringup scan_monitor  # prints closest obstacle front / left / right / back
```

**Orientation (verified 2026-10-24):** the arrow on top of the C1 is the front (0°).
The C1 needs `inverted: true` in `config/lidar.yaml`; with `false`, left and right
are mirrored. Quick check: book in front of the arrow should read `front` near 0°,
hand on the physical right should read `right` near -90°.

When the LiDAR is mounted, if the arrow doesn't point at the rover's front, set
`laser_yaw` (radians) in `lidar.launch.py`. A TF rotation can't fix a mirror, so that's
what `inverted` is for.

## 4. Seeing the scan graphically
RViz2 needs a desktop. Easiest options:
- Install ROS 2 Jazzy on another Ubuntu machine/VM on the same network with
  the same `ROS_DOMAIN_ID=42`, run `rviz2`, add a LaserScan display on `/scan`.
- Or record a bag on the Pi (`ros2 bag record /scan`) and view it later.

## Troubleshooting
- **`/dev/rplidar` missing:** `ls /dev/ttyUSB*`; check `lsusb` shows a CP210x
  (`10c4:ea60`). Try another cable (charge-only cables are common).
- **Permission denied on the port:** you haven't logged out/in since being
  added to `dialout`.
- **"Operation timeout" / no scan:** wrong baud rate.
- **Motor doesn't spin:** under-powered USB. Use the Pi's USB 3 ports and the
  27 W PSU.
