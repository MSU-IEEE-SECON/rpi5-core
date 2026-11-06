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
- [ ] Flash `mecanum_auto` to the ESP32, wheels-up test, calibrate (doc 06)
- [ ] Measure rover width → `robot_half_width` in `config/hallway.yaml`
- [ ] Push project to GitHub, add teammate (`docs/TEAM_ACCESS.md`)
