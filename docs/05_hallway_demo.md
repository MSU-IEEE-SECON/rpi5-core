# 05 — Hallway demo: centre in the hallway, steer around obstacles

**Goal:** press a button → rover drives down the hallway, keeps itself centred,
swerves left/right around anything in its path, and stops at the end of the
hallway (or if it gets truly boxed in). Press again → it stops.

## What's built vs. what's missing

| Piece | File | Status |
|---|---|---|
| Hallway + obstacle logic | `rover_bringup/hallway_logic.py` | ✅ written, 11 unit tests + 30 simulated drives pass |
| ROS node (LiDAR in → `/cmd_vel` out) | `rover_bringup/hallway_follower.py` | ✅ written, not yet run on the Pi |
| Start/stop button | `rover_bringup/start_button.py` | ✅ written, **untested**: needs a button wired up |
| One-command launch | `launch/demo.launch.py` | ✅ |
| Start at power-on (untethered) | `pi_setup/rover-demo.service` | ✅ written, leave off until motors work |
| **Motor link** (`/cmd_vel` → ESP32 → wheels) | `drive_bridge.py` + `firmware/mecanum_auto/` | ✅ written + tested off-robot, see **doc 06** |

Everything is written. The controller's **Y** button is the start button (it
puts the ESP32 in AUTO, which enables the autonomy), so the GPIO button is now
optional (`use_button:=true`).

## How it decides (short version)

1. Fits a straight line to each wall (LiDAR points), so it knows how far each
   wall is and whether the rover is angled in the hallway.
2. Looks ~1.2 m ahead for obstacles between the walls and finds the open
   gaps wide enough for the rover. With no obstacles, the gap is the whole hallway,
   so steering to its centre = hallway centring.
3. Steers toward the best gap (least swerve, prefers roomy gaps, sticks with a
   side once chosen so it doesn't dither).
4. Slows down as something gets close in its path. Inside 0.30 m it stops and
   turns in place toward the gap. No gap at all (end of hall) → stops.

States you'll see in the log: `FOLLOW`, `AVOID`, `PIVOT`, `BLOCKED`,
`DISABLED`, `NO_SCAN` (LiDAR data stopped → it stops).

## Step 1: dry run on the Pi (no motors needed)

Update the Pi's copy of the code (from the laptop, in this folder):

```powershell
scp -r ros2_ws/src/rover_bringup rover@rover.local:~/rover/ros2_ws/src/
```

On the Pi:

```bash
cd ~/rover/ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch rover_bringup demo.launch.py use_button:=false
```

Second window:

```bash
ros2 topic pub --once /rover/enable std_msgs/msg/Bool "{data: true}"
```

The first window prints a status line twice a second, e.g.
`FOLLOW v=+0.15 w=-0.04 | L=0.78 R=0.82 front=  -- hall= +1.2deg target_y=-0.02`.
Carry the Pi + LiDAR down a hallway (or move boxes around it) and check it
reacts the right way: `w` positive = it wants to turn **left**, negative = **right**.

Stop: `ros2 topic pub --once /rover/enable std_msgs/msg/Bool "{data: false}"` or Ctrl+C.

## Step 2: the start button

Wire a momentary push button between **GPIO17 (physical pin 11)** and **GND
(pin 9)**. No resistor needed. Then on the Pi:

```bash
sudo apt install -y python3-gpiozero python3-lgpio
ros2 launch rover_bringup demo.launch.py
```

Press → log shows `START` / `ENABLED`. Press again → `STOP`.
Different pin: `button_pin:=27`.

### Button troubleshooting
- **Permission error on `/dev/gpiochip*`:** give the `dialout` group access:
  ```bash
  echo 'SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="dialout", MODE="0660"' | sudo tee /etc/udev/rules.d/99-gpio.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
- **"Cannot determine SOC peripheral base" / wrong chip:** paste the error to
  Claude; Pi 5 GPIO libraries on Ubuntu can need a version tweak.

## Step 3: motors

See `docs/06_drivetrain_link.md`: flash the ESP32, plug it into the Pi,
test with **wheels off the ground**, calibrate speed. Start with
`cruise_speed: 0.10` in `config/hallway.yaml`.

## Step 4: run it untethered

The Ethernet cable can't follow the rover down a hallway. Once motors work:

```bash
bash ~/rover/pi_setup/install_autostart.sh
```

Now: power on → wait ~30 s for boot → connect the controller → press **Y**. Log afterwards with
`journalctl -u rover-demo -f` when plugged back in.

## Tuning

**Field of view:** on our mount the rear half of the LiDAR is blocked by the robot,
so the logic only uses the **front 180°** (`fov_deg: 180.0` in `config/hallway.yaml`;
360 = everything). If the robot's own parts still show up at the sides, narrow it
(e.g. 170) or tell Claude to add a "robot footprint" filter.

Settings live in `config/hallway.yaml` (all commented). Change live while
running, e.g. `ros2 param set /hallway_follower cruise_speed 0.2`.

Try changes in the simulator first, on any computer with Python 3 (no ROS):

```bash
cd ros2_ws/src/rover_bringup
python3 -m pytest test/test_hallway_logic.py -q          # unit tests (pip install pytest)
python3 test/sim_drive.py --course zigzag --noise 0.02   # full simulated drive
```

Courses: `default`, `zigzag`, `narrow`, `empty`, `doorways`. Also
`--start-y 0.3 --start-yaw 25` to start off-centre and angled.

**Measure our rover** and set `robot_half_width` (half its width, in metres).
The default 0.15 assumes a 30 cm wide rover with the LiDAR in the middle.

## Known limitations (fine for a demo, worth knowing)
- Tight squeezes: with very noisy readings it can graze a corner when the gap is
  only ~20 cm wider than the rover. Keep demo obstacles at least ~25 cm from walls.
- Only sees the front 180°: it can't see things beside/behind its rear half.
- Reactive only: no map. It handles what it sees within ~1.2 m.
- Assumes the LiDAR sits near the rover's centre of rotation. A long rover may
  clip an obstacle's corner with its rear while turning back.
- Glass, mirrors and very dark objects can be invisible to the LiDAR.
- Objects lower than the LiDAR's scan plane aren't seen.
