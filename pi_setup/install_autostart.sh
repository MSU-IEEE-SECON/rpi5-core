#!/usr/bin/env bash
# Install + enable the rover-demo systemd service (starts the demo at boot).
# Only do this once the motors are wired up and tested.
set -eo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
sudo cp "$DIR/rover-demo.service" /etc/systemd/system/rover-demo.service
sudo systemctl daemon-reload
sudo systemctl enable --now rover-demo
echo "rover-demo enabled. It will start at every boot."
echo "  status:  systemctl status rover-demo"
echo "  log:     journalctl -u rover-demo -f"
echo "  stop it: sudo systemctl stop rover-demo        (until next boot)"
echo "  remove:  sudo systemctl disable --now rover-demo"
