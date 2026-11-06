#!/usr/bin/env bash
# bootstrap_ros2_jazzy.sh — one-shot setup for the capstone rover Pi 5.
#
# Target: Ubuntu Server 24.04 LTS (noble), 64-bit, on a Raspberry Pi 5.
# Installs ROS 2 Jazzy (ros-base) + the packages we need for LiDAR work,
# sets up serial-port access, and builds ~/rover/ros2_ws.
#
# Safe to re-run. Full log goes to ~/rover/bootstrap.log.
#
# Usage (on the Pi, after copying pi_setup/ and ros2_ws/ into ~/rover/):
#     bash ~/rover/pi_setup/bootstrap_ros2_jazzy.sh

set -eo pipefail   # no -u: ROS setup.bash references unset variables

ROVER_DIR="$HOME/rover"
WS_DIR="$ROVER_DIR/ros2_ws"
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG="$ROVER_DIR/bootstrap.log"
ROS_DISTRO_NAME="jazzy"
DOMAIN_ID=42

mkdir -p "$ROVER_DIR"
exec > >(tee -a "$LOG") 2>&1

step() { echo; echo "==== [$(date +%H:%M:%S)] $* ===="; }

# --- 0. sanity checks -------------------------------------------------------
step "Checking OS"
. /etc/os-release
if [[ "${VERSION_CODENAME:-}" != "noble" ]]; then
  echo "This script expects Ubuntu 24.04 (noble). Found: ${PRETTY_NAME:-unknown}"
  echo "ROS 2 Jazzy binaries are only built for noble. Aborting."
  exit 1
fi
echo "OK: $PRETTY_NAME on $(uname -m)"

step "Checking internet"
if ! ping -c 1 -W 3 8.8.8.8 >/dev/null 2>&1; then
  echo "No internet. Is Internet Connection Sharing on the laptop enabled?"
  echo "See docs/02_ethernet_to_pi.md"
  exit 1
fi
echo "OK"

# --- 1. base system ---------------------------------------------------------
step "Making sure the noble-updates channel is enabled"
# Some Ubuntu Pi images ship with only "noble" + "noble-security". Without
# noble-updates, ROS's -dev dependencies don't match the installed libraries
# ("libzstd-dev : Depends: libzstd1 (= ...) but ... is to be installed").
SRC=/etc/apt/sources.list.d/ubuntu.sources
if [[ -f "$SRC" ]] && ! grep -qE "^Suites:.*\b${VERSION_CODENAME}-updates\b" "$SRC"; then
  sudo cp "$SRC" "$SRC.bak"
  sudo sed -i -E "s/^Suites: ${VERSION_CODENAME}\$/Suites: ${VERSION_CODENAME} ${VERSION_CODENAME}-updates ${VERSION_CODENAME}-backports/" "$SRC"
  echo "added ${VERSION_CODENAME}-updates/-backports (backup: $SRC.bak)"
else
  echo "OK"
fi

step "Updating system packages (this is the slow part)"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get -y full-upgrade

step "Locale (ROS needs UTF-8)"
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

step "Basic tools + mDNS (so rover.local works from the laptop)"
sudo apt-get install -y software-properties-common curl git nano htop \
  python3-pip python3-serial usbutils avahi-daemon
sudo systemctl enable --now avahi-daemon

# --- 2. ROS 2 apt source ----------------------------------------------------
step "Adding the ROS 2 apt source"
sudo add-apt-repository -y universe
if ! dpkg -s ros2-apt-source >/dev/null 2>&1; then
  ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  echo "ros-apt-source version: $ROS_APT_SOURCE_VERSION"
  curl -fL -o /tmp/ros2-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
else
  echo "already installed"
fi
sudo apt-get update

# --- 3. ROS 2 packages ------------------------------------------------------
step "Installing ROS 2 $ROS_DISTRO_NAME + rover packages"
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  ros-dev-tools \
  ros-$ROS_DISTRO_NAME-ros-base \
  ros-$ROS_DISTRO_NAME-tf2-tools \
  ros-$ROS_DISTRO_NAME-tf2-ros \
  ros-$ROS_DISTRO_NAME-teleop-twist-keyboard \
  ros-$ROS_DISTRO_NAME-slam-toolbox \
  ros-$ROS_DISTRO_NAME-rosbag2-storage-mcap

step "rosdep"
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update

# --- 4. serial port access --------------------------------------------------
step "Serial port access for the LiDAR"
sudo usermod -aG dialout "$USER"
sudo cp "$SETUP_DIR/99-rplidar.rules" /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "udev rule installed: CP210x adapter -> /dev/rplidar"

# --- 5. build the workspace -------------------------------------------------
step "Building $WS_DIR"
# shellcheck disable=SC1090
source /opt/ros/$ROS_DISTRO_NAME/setup.bash
if [[ -d "$WS_DIR/src" ]]; then
  # The apt rplidar_ros for Jazzy is an older release (only 'rplidar_composition')
  # without RPLIDAR C1 support, so build Slamtec's current driver from source.
  if [[ ! -d "$WS_DIR/src/rplidar_ros" ]]; then
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git "$WS_DIR/src/rplidar_ros"
  fi
  cd "$WS_DIR"
  rosdep install --from-paths src --ignore-src -y || true
  colcon build --symlink-install
else
  echo "No workspace found at $WS_DIR/src (did you scp ros2_ws?). Skipping build."
fi

# --- 6. shell setup ---------------------------------------------------------
step "Adding ROS to ~/.bashrc"
MARK="# >>> rover ros2 setup >>>"
if ! grep -qF "$MARK" "$HOME/.bashrc"; then
  cat >> "$HOME/.bashrc" <<EOF

$MARK
source /opt/ros/$ROS_DISTRO_NAME/setup.bash
[ -f $WS_DIR/install/setup.bash ] && source $WS_DIR/install/setup.bash
export ROS_DOMAIN_ID=$DOMAIN_ID
export RCUTILS_COLORIZED_OUTPUT=1
# <<< rover ros2 setup <<<
EOF
  echo "added"
else
  echo "already present"
fi

step "Done"
echo "ROS 2 $ROS_DISTRO_NAME installed. ROS_DOMAIN_ID=$DOMAIN_ID"
echo
echo "NEXT:"
echo "  1. Log out and back in (or 'sudo reboot') so the dialout group applies."
echo "  2. Plug in the LiDAR, check:  ls -l /dev/rplidar"
echo "  3. Identify it:               python3 ~/rover/tools/lidar_identify.py /dev/rplidar"
echo "  4. Launch:                    ros2 launch rover_bringup lidar.launch.py"
echo
echo "Log saved to $LOG"
