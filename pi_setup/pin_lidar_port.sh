#!/usr/bin/env bash
# Lock /dev/rplidar to THIS LiDAR's USB adapter (by its serial number).
#
# Why: the default rule (99-rplidar.rules) matches any CP210x USB-serial chip.
# Many ESP32 dev boards use the same chip, so with both plugged in /dev/rplidar
# could point at the ESP32. Pinning the serial number fixes that.
#
# Run with ONLY the LiDAR plugged in (unplug the ESP32 first):
#     bash ~/rover/pi_setup/pin_lidar_port.sh
set -eo pipefail

ports=(/dev/ttyUSB*)
if [[ ${#ports[@]} -ne 1 || ! -e "${ports[0]}" ]]; then
  echo "Need exactly one USB serial device plugged in (just the LiDAR). Found: ${ports[*]}"
  exit 1
fi
dev="${ports[0]}"
props=$(udevadm info -q property -n "$dev")
vid=$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$props")
pid=$(sed -n 's/^ID_MODEL_ID=//p' <<<"$props")
serial=$(sed -n 's/^ID_SERIAL_SHORT=//p' <<<"$props")
if [[ -z "$serial" ]]; then
  echo "This adapter reports no serial number; can't pin it. Use a different USB port"
  echo "for each device and see docs/06_drivetrain_link.md, 'Port mix-ups'."
  exit 1
fi
rule="KERNEL==\"ttyUSB*\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", ATTRS{serial}==\"$serial\", MODE:=\"0666\", GROUP:=\"dialout\", SYMLINK+=\"rplidar\""
echo "LiDAR adapter on $dev: $vid:$pid serial $serial"
echo "# Slamtec RPLIDAR C1 USB adapter, pinned by serial number (pin_lidar_port.sh)" | sudo tee /etc/udev/rules.d/99-rplidar.rules >/dev/null
echo "$rule" | sudo tee -a /etc/udev/rules.d/99-rplidar.rules >/dev/null
sudo udevadm control --reload-rules
sudo udevadm trigger
sleep 1
ls -l /dev/rplidar
echo "Done. Record this serial in docs/hardware.md: $serial"
