# Setup Log

Newest entries at the bottom. Record anything done to the Pi, the laptop,
or the hardware so the team can retrace it.

---

## 2026-10-24

- Decided on **ROS 2 Jazzy on Ubuntu Server 24.04 LTS (64-bit)** for the Pi 5
  (ready-made `rplidar_ros`, `slam_toolbox`, Nav2, `apriltag_ros` packages).
- Wi-Fi is not usable for the Pi (eduroam needs enterprise login), so the Pi
  connects to the laptop over **direct Ethernet** with Windows Internet
  Connection Sharing. See `02_ethernet_to_pi.md`.
- LiDAR: model sticker scratched off; connects over USB-C. Wrote
  `tools/lidar_identify.py` to identify it by baud rate + GET_INFO.
- Created this folder structure, docs, the Pi bootstrap script, and the
  `rover_bringup` ROS 2 package (LiDAR launch file + `scan_monitor` node).
- Checked "Old Secon Bot USB for PI": Duet 3D printer SD card, no rover code.
  Not modified.

- Installed Raspberry Pi Imager, flashed **Ubuntu 24.04.5 LTS** (kernel 6.8.0-1064-raspi)
  with customisation: hostname `rover`, user `rover`, SSH on. (Customisation dialog
  was fiddly to get to in Imager but worked.)
- Laptop Ethernet = USB dongle "Ethernet 2" (Realtek USB GbE), ICS on → `192.168.137.1`.
- Pi came up at **`192.168.137.133`** (MAC 88:a2:9e:1a:07:73). SSH works:
  `ssh rover@192.168.137.133`. `rover.local` won't resolve until bootstrap installs avahi.
- ⚠ Login banner: *"This power supply is not capable of supplying 5A; power to
  peripherals will be restricted."* USB ports are limited (~600 mA total) on this
  supply, which may not be enough to spin the LiDAR motor. Use the official 27 W
  (5V/5A) Pi 5 PSU.

- Copied `pi_setup/ ros2_ws/ tools/` to `~/rover/` on the Pi, ran the bootstrap inside `tmux`.
- SSH dropped during the system upgrade (systemd/openssh restart) and the Pi came
  back on a **new IP (`192.168.137.151`)**. ICS addresses can change, so use
  `rover.local` (works after bootstrap) or rescan:
  `$p=1..254|%{(New-Object Net.NetworkInformation.Ping).SendPingAsync("192.168.137.$_",500)}; [Threading.Tasks.Task]::WaitAll($p); $p|?{$_.Result.Status -eq 'Success'}|%{$_.Result.Address.ToString()}`
- **Problem 1:** ROS install failed with unmet `-dev` dependencies (`libzstd-dev`,
  `libicu-dev`, …). Cause: this Ubuntu Pi image had only `noble` + `noble-security`
  in `/etc/apt/sources.list.d/ubuntu.sources`, with no `noble-updates`. Fixed by
  changing the line to `Suites: noble noble-updates noble-backports`. The bootstrap
  script now does this automatically.
- **Problem 2:** script stopped at the workspace build with
  `AMENT_TRACE_SETUP_FILES: unbound variable`. Cause: the script used `set -u`, which
  ROS's `setup.bash` doesn't support. Removed `-u` from the script.
- ✅ **ROS 2 Jazzy installed**, `rover_bringup` built, `~/.bashrc` sources ROS,
  `ROS_DOMAIN_ID=42`, udev rule for `/dev/rplidar` in place. Log: `~/rover/bootstrap.log`.

- Rebooted; `ssh rover@rover.local` works. LiDAR shows up as `/dev/rplidar -> ttyUSB0`.
- ✅ **LiDAR identified: RPLIDAR C1** (460800 baud, model 0x41, FW 1.02, HW 18, health Good).
  Set `serial_baudrate: 460800` in `config/lidar.yaml`.
- ⚠ Login banner offers an upgrade to Ubuntu 26.04. **Do not run `do-release-upgrade`.**
  ROS 2 Jazzy only supports 24.04, so the upgrade would break it.

- **Problem 3:** `executable 'rplidar_node' not found`. The apt package
  `ros-jazzy-rplidar-ros` only ships `rplidar_composition` (an older driver that
  predates the C1). Fix: build Slamtec's driver from source:
  `git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git ~/rover/ros2_ws/src/rplidar_ros`.
  The bootstrap script now does this instead of installing the apt package.
  `lidar.launch.py` also auto-detects either executable name.

- ✅ **First live scan works.** `ros2 launch rover_bringup lidar.launch.py` +
  `ros2 run rover_bringup scan_monitor`: ~600 of 720 points valid per scan, stable
  readings (desk test: front 0.35 m, right 0.31 m, back 0.41 m, left changing as
  things moved). Ran fine on the current (non-5A) power supply.
- **Orientation test** (book in front of the arrow on the C1's top, hand on the
  physical right): book read as `front` 0.31 m @ ≈-7°, which is correct (arrow = 0° = front).
  But the hand on the physical **right** read as `left` 0.15 m @ ≈+90°, so **left/right
  were mirrored**. Fix: `inverted: true` in `config/lidar.yaml`. This reverses the
  scan order (angles ≈ symmetric about 0, so front stays front). Needs a re-test.

- **Hallway demo software written** (`docs/05_hallway_demo.md`): `hallway_logic.py`
  (wall line-fit + lateral gap finding), `hallway_follower` node (`/scan` → `/cmd_vel`,
  enable topic, LiDAR watchdog), `start_button` node (GPIO17), `demo.launch.py`,
  autostart service. Tested off-robot: 11 unit tests + 30 simulated drives
  (5 courses × noise × start poses), all pass, no collisions. Bugs found and fixed in
  simulation: obstacle mistaken for a wall; angled robot seeing walls as obstacles;
  passed + upcoming obstacles merging into a false "blocked"; squeezing into
  barely-wide-enough gaps.
- Added `.gitignore` + `docs/TEAM_ACCESS.md` for moving the project to GitHub.

- Teammate's drivetrain code received (`mecanum.ino`): single **ESP32**, Bluepad32
  controller, 4 mecanum wheels, 2-wire driver channels, no encoders. Kept unchanged in
  `firmware/mecanum/`. Wrote `firmware/mecanum_auto/` (same manual driving + AUTO mode:
  Y toggles, B/stick override, 300 ms watchdog, status lines) and the Pi-side
  `drive_bridge` (`/cmd_vel` → `D vx vy wz` over USB at 20 Hz; ESP32 AUTO → `/rover/enable`).
  Tested off-robot: firmware compiled against stub Arduino/Bluepad32 headers and run
  through 9 scenarios; Python protocol tests incl. round-trip through the C++ parser.
- GitHub: repo `BoBoBoweenie/SECON_HELL` created; the Claude GitHub app couldn't get access
  from this session, so the push is pending.

- Teammate's laptop gave the Pi a different IP. Added a fixed second address
  `192.168.137.50` via `/etc/netplan/60-rover-fixed-ip.yaml` (instructions in doc 02).

- **Port mix-up found & fixed:** with the ESP32 plugged in, both devices are Silicon Labs
  (LiDAR adapter = CP2102N serial `de6e6c51…b110`, ESP32 = CP2102 serial `0001`, same
  10c4:ea60 IDs) and `/dev/rplidar` pointed at the **ESP32**. Confirmed with
  `lidar_identify.py` (only ttyUSB1 answered, C1 @ 460800). Pinned the udev rule to the
  LiDAR's serial → `/dev/rplidar -> ttyUSB1`. Repo rule updated to match.

- LiDAR mounted on the bot: rear half is blocked by the robot. Hallway logic now uses
  only the **front 180°** (`fov_deg: 180.0`, new setting in `config/hallway.yaml`).
  Tests added (clutter behind the LiDAR is ignored). With front-only vision the
  simulator exposed 4 weaknesses, all fixed: (1) close obstacles out-voted walls in
  the wall fit → score lines by length, not point count; (2) noise near walls
  hijacked the "nearest obstacle" grouping → ignore points within 10 cm of a wall;
  (3) rover flip-flopped sides around a centred obstacle → stay committed until the
  hall is clear for 5 scans; (4) stopped for things only in the comfort margin →
  separate "would hit" (5 cm) vs "slow down" (8 cm) margins, and cap the angle
  across the hall at 40°. Result: 30/30 simulated drives pass at 0 and 2 cm noise.
  At an extreme 3 cm noise: 10/15 pass, 4 stop early (safe), 1 grazes a corner on the
  0.55 m-half-width "narrow" course. Real C1 readings were steady to ~1 cm.

- Rover is **10 in wide** → `robot_half_width: 0.13`. LiDAR is **not centred** on the
  rover → new `laser_x` / `laser_y` settings (LiDAR position from the rover's centre);
  the logic now works relative to the rover's centre. Test added. With the narrower
  rover: 27/30 simulated drives pass, 0 collisions; the 3 others stop early (safe) and
  are all 25°-crooked starts with noise. Tried "remember hallway width" + "look further
  for walls"; it made things worse (11 early stops), so it was reverted.

- LiDAR is ~10 in **forward** of the rover's centre, ~1 in left → `laser_x: 0.254`,
  `laser_y: 0.025`. Simulator now models the LiDAR offset and a rover body
  (assumed ~22 in long: 0.30 m ahead of centre, 0.25 m behind, rover length still to measure)
  and checks body clearance. Result: **13/30 fail, 6 with body collisions.** Cause: with a
  forward LiDAR and front-180° view, the rover is blind beside its own body and the rear
  swings into obstacles it has already passed while turning. Options under discussion:
  move the LiDAR back/up, or avoid obstacles by **strafing** (mecanum) instead of turning.
  These settings are NOT on the Pi yet (Pi still has the centred-LiDAR version).

- ESP32 now runs `mecanum_auto` (confirmed: `S MANUAL 0 0 0 0` at 5 Hz over USB).
  Opening the port restarts the ESP32 (connect the controller after the Pi connects).
  Background runs need `PYTHONUNBUFFERED=1` or `demo.log` lags behind (added to the
  autostart service): `PYTHONUNBUFFERED=1 nohup ros2 launch rover_bringup demo.launch.py > ~/demo.log 2>&1 &`

## 2026-11-06

- Current code pushed to the team repo `rpi5-core` (branch `on-board`).
- First hallway runs on the rover: much better, but jittery ("drunk driver": wiggles
  left-right, swerves at nothing, stop-and-go, reacts 1 m+ early). The log showed
  `front≈0.15` and `L/R` of 0.01–0.10 almost constantly, i.e. **the rover was seeing
  its own body** and sat in PIVOT/BLOCKED, mostly pivoting right. There was also no
  smoothing, so decisions flipped every scan (10 Hz).
- Fixes: new `record_self_mask` tool + follower mask filter; steering smoothing,
  heading deadband, speed ramp-up, stop hysteresis, speckle filter, calmer defaults
  (see doc 05, "Smooth driving"). 20 unit tests pass. Simulator: 27/30 pass (same
  3 hard cases as before: 25°-crooked starts with noise), steering reversals 232 → 92.
- Code defaults in `hallway_logic.py` now match `hallway.yaml` (LiDAR offset 0).

- Later Nov 6 runs: the ESP32 went silent once (stuck after a force-stop of the
  demo); a reset fixed it, and `drive_bridge` now warns `ESP32 ... SILENT`. Y/B worked.
  The rover still pivoted right at phantom points 4–7 cm from the LiDAR → `min_range`
  raised to 0.12 m. Walls were rarely found (hallway wider than the 1.5 m limit), so
  "down the hallway" was just "wherever it pointed" after each turn.
- New mount: LiDAR on the front, centred, arrow forward, front 180° used.
- Rewrote the behaviour around what we actually want: **hallway mode** (default):
  stay centred, drive down the hall, stop for anything ahead. Walls found up to 3 m.
  It remembers the hallway direction across scans (predicts it from its own turning,
  doubts readings that disagree), uses the remembered width when one wall is missing,
  coasts through a few sparse scans. Obstacle avoiding is still there with
  `avoid_obstacles: true`. 28 unit tests; simulator 54/54 hallway runs.

## 2026-11-08

- Pi found at the fixed address `192.168.137.50` (`rover.local` didn't resolve); use
  `-Pi rover@192.168.137.50`. Hallway-mode code deployed, mask re-recorded.
- Added `rover.ps1 look` (closest thing front/left/back/right, 20 s).
- **New mount reads the scan mirrored front-to-back:** a hand in front read as "back"
  (178°), a hand on the right read as "right" (−87°), and the wall ~7 ft ahead read as
  "back 2.24 m". Fix in `hallway.yaml`: `laser_mirror: true`, `laser_yaw: 3.14159`
  (rover angle = 180° − raw). The self mask stays in raw angles, so it's still valid.
  `look` now shows directions as the driving code sees them.

- Nov 9: Y reached the Pi (`ENABLED: driving`) but the rover sat in BLOCKED: the
  wheels beside the front-mounted LiDAR (±90°, 13–20 cm) counted as "in my path".
  New `body_front` / `body_half_width` settings drop a box around the rover's own body.
  Firmware: Y debounced (500 ms) so one press can't toggle twice (reflash needed).

- LiDAR re-oriented again. Instead of hand-tuning `laser_yaw`/`laser_mirror`, added
  `calibrate_lidar` (`rover.sh calibrate`): reads the empty room (where the rover/mount
  blocks the view), then a box held in front and front-left -> rover front, mirrored or
  not, and the usable arc each side (sides at ±90° are blocked on this mount). Saved to
  `~/rover/lidar_calib.json` on the Pi; overrides hallway.yaml; deploys don't touch it.
  `fov_left_deg`/`fov_right_deg` replace the symmetric 180° view. 35 tests.

## 2026-11-10

- Arena board finished; hallway mode is archived (kept, selectable) — next target is the arena.
- A teammate reorganized the Pi: `~/rover` → `~/rover_archive`, repo cloned at
  `~/repos/rpi5-core` (SSH remote), `arduino-cli` + `~/sewer/esp32-firmware/{compile,upload}.sh`
  flash the ESP32 from the Pi (`/dev/esp32`), plus BNO055 IMU and micro-ROS experiments.
  Controller had stopped working (ESP32 had test firmware) → reflashed our `mecanum_auto`
  from the Pi. **BNO055 IMU is on the rover** (wired to the ESP32).
- Tools moved to the new layout: code in `~/repos/rpi5-core`, settings in `~/rover_data`,
  new `rover.sh setup` (copies LiDAR driver + settings from the archive, fixes ~/.bashrc, builds).
- Pi had no DNS/internet (git pull failed) — code copied from the laptop instead.

### Still to do
- [x] Flash SD card (Ubuntu Server 24.04 LTS 64-bit), hostname `rover`, user `rover`, SSH on
- [x] Enable Internet Connection Sharing (Wi-Fi → Ethernet) on the laptop
- [x] First boot + SSH in
- [x] Copy files + run `bootstrap_ros2_jazzy.sh`
- [x] Reboot, confirm `ssh rover@rover.local` works
- [x] Identify LiDAR model
- [x] First live scan (`ros2 launch rover_bringup lidar.launch.py`)
- [x] Confirm the LiDAR's 0° direction: the arrow on top = front
- [x] Re-test with `inverted: true`: hand on the right reads `right` ≈ -100°
- [ ] After mounting on the rover, set `laser_yaw` if the arrow doesn't face the rover's front
- [ ] Get a 5V/5A (27 W) supply for the Pi
- [ ] Dry-run `demo.launch.py` on the Pi (no motors)
- [ ] Wire start button to GPIO17/GND and test
- [x] Motor link written (ESP32 AUTO mode + drive_bridge)
- [x] Flash `mecanum_auto` to the ESP32
- [ ] Wheels-up test, calibrate (doc 06)
- [x] Measure rover width → `robot_half_width: 0.13` (10 in)
- [ ] Measure LiDAR offset from rover centre → `laser_x` / `laser_y` in `config/hallway.yaml`
- [x] Push project to GitHub (`rpi5-core`, branch `on-board`)
- [ ] Record the self mask on the rover (`ros2 run rover_bringup record_self_mask`)
