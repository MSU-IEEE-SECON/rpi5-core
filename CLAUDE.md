# CLAUDE.md: SECON Stock Car rover (rpi5-core)

Read this first. It is the handoff from the claude.ai project chat
("Perception and Autonomy for a lil Rover") that built most of this code (Oct 24 to Nov 17, 2026).

## Who and what

- **Bowen** (perception subsystem owner, MSU ECE, graduating May 2027). Team "SECON Stock Car",
  2027 IEEE SoutheastCon hardware competition (Stock Car Race).
- Teammates: the drivetrain/ESP32 teammate (GitHub `cadentensity`) and an autonomy lead.
- Goal: autonomous laps on a 4 x 8 ft board around a center rail while avoiding toy cars,
  then read an AprilTag, pit stop, and park in the assigned pit box. Rules and scoring:
  `docs/07_arena.md`.

## Rules (from Bowen; follow every one)

1. **Never edit the "Old Secon Bot USB for PI" folder** (in `Desktop\CAPSTONE\The Actual Bot`).
   Read it for reference only.
2. **Never type or store passwords.** Log in to the Pi with an SSH key (see "Pi access"). If a
   password prompt appears anyway (sudo, a new machine), stop and ask Bowen to type it.
3. **Do not control Bowen's screen** (no computer use on his desktop).
4. **SECON Microsoft Teams/SharePoint: read and download only.** Never edit or upload there.
5. **Ask before downloading files.**
6. `rpi5-core` must stay **private**. Check the repo's visibility before pushing anything new.
7. **After every work session, write an After Action Report** (see "After Action Reports").
8. Follow Bowen's writing standards for all docs and reports (see "Writing standards").

## After Action Reports

At the end of each work session (or each test day on the board), write
`docs/aar/YYYY-MM-DD_<short-topic>.md` from `docs/aar/TEMPLATE.md`, and commit it with the code.
Facts only: what was planned, what happened (with numbers from the logs), why, what to keep doing,
what to change, and next steps. Link the log files or commits it is based on.

## Writing standards

<!-- Bowen: paste your writing standards here. Until then: plain language, short sentences,
     numbers with units, say what was tested and what was not, no filler. -->

## Hardware (as of Nov 17)

- **Raspberry Pi 5**, Ubuntu 24.04, ROS 2 Jazzy, `ROS_DOMAIN_ID=42`, user `rover`.
- **RPLIDAR C1** at `/dev/rplidar` (460800 baud), mounted upside down: `laser_yaw` 180, mirror off.
  Only the front ±75° is used (the rear is blocked by the rover). Scan plane 2 in high, so the
  1.5 in center rail is invisible to it.
- **ESP32** at `/dev/esp32`, running the teammate's **micro-ROS** firmware
  (`~/sewer/esp32-firmware/micro_ros_driver` on the Pi). Serial **921600** baud
  (the agent must use `-b 921600`). Subscribes `/cmd_vel` (Twist, real m/s, encoder PID, 300 ms watchdog),
  publishes `/odom` (Float32MultiArray `[x, y, yaw, vx, vy, wz]`), `/imu`, `/wheel_speeds`.
  It only joins an agent it finds after boot, so `tools/esp32_reset.py` pulses RTS before launch.
- **BNO055 IMU** on the ESP32. Not used directly by our code yet (see "Next steps").
- **Logitech C270 USB webcam** at `/dev/video0` (YUYV; its MJPEG frames are broken). The Pi camera
  v2.3 (IMX219) is not set up.
- **PS5 DualSense** paired to the Pi: Triangle = AUTO on/off, Circle or L1 = stop/manual.
- Rover: 10 in long x 10.2 in wide. LiDAR 1 in from the front, centred. Camera 3 in from the front,
  2 in right of centre, 5 in high, tilted about 22 degrees down.

## Pi access and the network

- Pi to laptop over **Ethernet** with Windows Internet Connection Sharing. Fixed address
  **192.168.137.50** (also `rover.local`). Wi-Fi is not usable (eduroam; the hotspot kept dropping).
- **The Pi has no internet** (ICS forwards nothing). Python packages are vendored in `tools/vendor`.
- **Dashboard: http://192.168.137.50:8080** (starts at boot, `pi_setup/rover-dashboard.service`).
- SSH: `ssh rover@192.168.137.50`. Use a key, not a password:
  `ssh-keygen -t ed25519` (once), then
  `type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh rover@192.168.137.50 "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"`
  (Bowen types the password that one time).

## Deploying code (no SSH needed)

1. Change code here, run the tests, commit.
2. `python tools/make_bundle.py` creates `deploy/rover_code.tar.gz` with a `VERSION` stamp.
3. Upload it on the dashboard (**Update code**, then **Upload + rebuild**), or POST it to
   `/api/deploy`. Deploy is refused while a dashboard action is running.
4. Check the version in `/api/status` and the build log in `/api/last_build`.
5. Restart laps: dashboard **Arena laps**, or POST `/api/action {"name":"lap"}`.

Other endpoints: `/api/scan`, `/api/arena`, `/api/log`, `/api/rosout`, `/files/demo.log`,
`/files/live.png` (camera). Full guide: `docs/RUNNING_THE_ROVER.md`. Pi command list:
`pi_setup/rover.sh` (lap, stop, restart, look, snapshot, mask, calibrate, diag, camlive, ...).

## Running a test on the board

1. Rover in the **start box** (bottom right, facing up the course). Ethernet plugged in.
2. Start laps (dashboard). Wait for the log to say the ESP32 and LiDAR are up.
3. Unplug Ethernet, press **Triangle** (AUTO). Circle or L1 stops it.
4. Plug Ethernet back in and read the log on the dashboard.

A battery swap reboots the Pi. Laps do not autostart after a reboot (the dashboard does).

## Code map (`ros2_ws/src/rover_bringup/`)

| What | Where |
|---|---|
| Lap launch (LiDAR, camera, lap_driver, joy, micro-ROS agent) | `launch/lap.launch.py`, `config/lap.yaml` |
| Board map + LiDAR localization (point-to-line ICP, global search) | `rover_bringup/arena.py` |
| Main loop: scan to points, localize, relocalize/LOST, publish `/cmd_vel` | `rover_bringup/lap_driver.py` |
| Racing-line planner, obstacle grid, speed learning | `rover_bringup/path_lap.py` |
| Camera: streaming, floor mapping, paint/car detection | `camera_node.py`, `camera_vision.py`, `v4l2cam.py` |
| Controller AUTO toggle | `rover_bringup/joy_enable.py` |
| Dashboard (web UI, deploy, actions) | `rover_bringup/rover_dashboard.py` |
| LiDAR mount calibration, self mask | `lidar_calib.py`, `calibrate_lidar.py`, `record_self_mask.py` |
| Old hallway demo and Bug2 planner (archived) | `hallway_*.py`, `bug2.py`, `drive_bridge.py` |
| Tests and simulator | `test/` (`python -m pytest ros2_ws/src/rover_bringup/test`; 56 pass) |

## Status on Nov 17

**Works:**
- 3 laps on the empty board in 81 s (30/25/25 s), with no relocalizations, from the start box.
- Dashboard with code upload, live LiDAR, arena view with obstacle cells, and live camera.
- Camera at 10 fps finds the white lane paint and maps it to the board (about 0.7 in RMS
  to 6 ft). No false obstacles on the empty board.

**Not tested yet:**
- Car avoidance on the real board with the current code (LiDAR + camera).
- Camera car detection with real cars.

**Known issues:**
- Driving is still "a bit clunky". Bowen wants it smoother, then faster.
- A few phantom obstacle cells about 2 in inside the walls (try widening `known_gap`).
- The camera shows the center rail's far end near y = 70 in. The map assumes 80 in. Bowen is to
  measure the rail; fix `arena.RAIL` if it really is shorter.
- The white octagon's right side is about 37.5 in from the left wall (13 in right of the rail).
- `pi_setup/rover-dashboard.service` with `SupplementaryGroups=video dialout` is not installed on
  the Pi yet (Bowen ran `usermod -aG video rover` instead).

## Next steps (in order)

1. Cars on the board: check camera detections in **Camera view**, then lap runs with cars.
2. Measure the rail and fix the map if needed.
3. Use the IMU heading in localization (confirm what the firmware's `/odom` yaw rate comes from first).
4. Smoother, faster driving.
5. AprilTags (tag41h12, 68 mm, IDs 1-3) once they are on the board. `dt_apriltags` is vendored and works
   on the Pi (0.4 s per 640x480 frame). Then the pit stop and parking.
