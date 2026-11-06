# 03 — Install ROS 2 Jazzy on the Pi

Everything is in `pi_setup/bootstrap_ros2_jazzy.sh`. It's safe to re-run.

## Copy files to the Pi (from the laptop, in this folder)

```powershell
cd "$HOME\Desktop\CAPSTONE\The Actual Bot"
ssh rover@rover.local "mkdir -p ~/rover"
scp -r pi_setup ros2_ws tools rover@rover.local:~/rover/
```

## Run it (on the Pi)

```bash
bash ~/rover/pi_setup/bootstrap_ros2_jazzy.sh
```

Takes ~15–30 min depending on the connection. A full log is saved to
`~/rover/bootstrap.log`.

## What it does
1. `apt update && full-upgrade`, sets the locale to UTF-8.
2. Installs `avahi-daemon` so `rover.local` works from the laptop.
3. Adds the official ROS 2 apt source (`ros2-apt-source` package).
4. Installs `ros-jazzy-ros-base`, `ros-dev-tools`, and:
   `tf2_tools`, `teleop_twist_keyboard`, `slam_toolbox`.
5. Initialises `rosdep`.
6. Adds the user to `dialout` (serial port access) and installs the udev rule
   that creates `/dev/rplidar`.
7. Clones Slamtec's `rplidar_ros` (ros2 branch, has RPLIDAR C1 support) into the
   workspace and builds `~/rover/ros2_ws` with `colcon`. (The apt `rplidar_ros` for Jazzy
   is an older driver without C1 support.)
8. Adds ROS + workspace sourcing to `~/.bashrc`, sets `ROS_DOMAIN_ID=42`.

**Log out and back in (or reboot) afterwards** so the `dialout` group applies.

## Check it worked

```bash
ros2 doctor --report | head -20
ros2 pkg list | grep -E "rplidar|rover_bringup"
```

## Not installed yet (on purpose)
- **Nav2** (`ros-jazzy-navigation2`): large; add when we get to path planning.
- **Camera / AprilTags** (`camera_ros`, `apriltag_ros`): depends on which
  camera we use. Pi Camera Module support on Ubuntu needs extra setup.
