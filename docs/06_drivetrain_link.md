# 06 — Linking the Pi to the drivetrain (ESP32)

The drivetrain is an **ESP32** running Bluepad32 (Bluetooth controller) driving
**4 mecanum wheels**, each through a 2-wire driver channel (no encoders).
Original code: `firmware/mecanum/mecanum.ino`.

The Pi doesn't touch the motors. It sends drive commands over a **USB cable** to
the ESP32, the same way the controller sticks do.

```
RPLIDAR ──USB──▶ Pi 5: hallway_follower ─/cmd_vel─▶ drive_bridge ──USB──▶ ESP32 ──▶ motor drivers ──▶ 4 wheels
                                    ▲                    │                ▲
                                    └── /rover/enable ◀──┘ (AUTO/MANUAL)  └── Bluetooth controller
```

## How you drive it

| Do this on the controller | What happens |
|---|---|
| Sticks (normal) | Manual driving, exactly like before |
| **Y** (triangle on PS) | **AUTO**: the Pi drives. The Pi's autonomy starts automatically |
| **Y** again, **B**, or **touch any stick** | Back to MANUAL instantly, Pi ignored |
| Controller disconnects | Motors stop, MANUAL |

Safety built into the ESP32: in AUTO, if the Pi's commands stop for 300 ms → motors stop.
AUTO is only possible with a controller connected (it's the kill switch).

## What changed in the firmware

`firmware/mecanum_auto/mecanum_auto.ino` = the original sketch + an AUTO mode.
Every addition is marked `// [AUTO]`; manual driving code is unchanged.
`auto_drive.h` (same folder) holds the protocol parsing + wheel mixing.

Tested on a PC (`firmware/test/test_firmware.cpp`): the real sketch is compiled
against fake Arduino/Bluepad32 headers and run through manual driving, Y/B
toggling, Pi commands, the 300 ms watchdog, stick override, disconnects and
junk serial data. All pass. **Not yet run on the real ESP32.**

## Step 1: flash the ESP32 (teammate)

Open `firmware/mecanum_auto/mecanum_auto.ino` in the Arduino IDE (the folder
must keep that name; `auto_drive.h` sits next to it), same board settings as
before → Upload. Check in the Serial Monitor (115200): you should see
`[AUTO] Press Y to let the Pi drive...` and `S MANUAL 0 0 0 0` lines.

Then with **wheels off the ground**: drive manually like normal. It should feel
identical to the old code.

## Step 2: plug the ESP32 into the Pi

USB cable from the ESP32's USB port into one of the Pi's USB ports. Then on the Pi:

```bash
ls -l /dev/rplidar /dev/serial/by-id/
```

- `/dev/rplidar` must still point at the LiDAR. If both devices show the same
  `Silicon_Labs_CP2102` name, fix it once: unplug the ESP32, run
  `bash ~/rover/pi_setup/pin_lidar_port.sh`, plug the ESP32 back in.
- Plugging the Pi into the ESP32 can reset it (the Bluetooth controller then
  re-pairs). Normal; it only happens when the cable/port is opened.

Power: the ESP32 can be powered from both the robot battery and USB at the same time on
most dev boards. If the board's 5V/VIN pin is fed directly from a regulator
**without a diode**, ask before connecting both.

## Step 3: first test (wheels off the ground!)

On the Pi, update and build (see TEAM_ACCESS.md, or scp `ros2_ws/src/rover_bringup`), then:

```bash
ros2 run rover_bringup drive_bridge --ros-args --params-file ~/rover/ros2_ws/src/rover_bringup/config/drive.yaml
```
It should say `auto-detected ESP32 on /dev/serial/by-id/...: connected` and
`drivetrain mode: DISCONNECTED -> MANUAL`.

Second window, send a slow forward command continuously:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
```
Nothing moves (MANUAL). **Press Y** → bridge logs `MANUAL -> AUTO` → all 4 wheels
turn forward slowly. Touch a stick → stops / manual. Ctrl+C the `pub` → wheels
stop within 0.3 s.

Check directions while in AUTO:
| Command | Expect |
|---|---|
| `"{linear: {x: 0.1}}"` | all wheels forward |
| `"{angular: {z: 0.5}}"` | spins **left** (counter-clockwise from above) |
| `"{linear: {y: 0.1}}"` | strafes **left** |

If a direction is backwards, the fix is in the ESP32 (swap that motor's two
pins), not the Pi. The manual sticks will have the same issue.

## Step 4: calibrate speed (no encoders, so measure)

1. Floor, clear space, tape measure. AUTO, then:
   `ros2 topic pub -t 30 -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}}"`
   (runs ~3 s). Measure distance `d` metres.
   That command asked for 0.25 m/s, i.e. 50 % power with the 0.5 default.
   Real speed at 50 % = d / 3, so set **max_linear_mps = 2 × d / 3**.
2. Same idea for spin: `"{angular: {z: 1.25}}"` for 3 s, count the turns `n`.
   **max_angular_radps = 2 × (n × 6.28) / 3**.
3. Put the numbers in `config/drive.yaml`.
4. `AUTO_MIN_PWM` in the firmware (default 60): the lowest PWM where all wheels
   still turn. If slow commands just hum, raise it.

## Step 5: the demo

```bash
ros2 launch rover_bringup demo.launch.py
```
Press **Y** → it drives the hallway. Touch a stick / B → stop.

## Port mix-ups
`ros2 run rover_bringup drive_bridge` says "several possible ESP32 ports" → set
`port:` in `config/drive.yaml` to the ESP32's `/dev/serial/by-id/...` path.

## Later: use the mecanum wheels
The protocol already carries sideways speed (`linear.y`). The hallway logic
could strafe to recentre instead of turning. Worth trying once the basics work.
