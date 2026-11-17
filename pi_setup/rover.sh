#!/bin/bash
# rover.sh — one-word commands for the rover, run ON THE PI.
# (From the laptop, use rover.ps1 in the repo root, which calls this over SSH.)
#
#   bash ~/repos/rpi5-core/pi_setup/rover.sh setup    one time per Pi: LiDAR driver, data folder, ~/.bashrc, build
#   bash ~/repos/rpi5-core/pi_setup/rover.sh build    rebuild the ROS package after new code
#   bash ~/repos/rpi5-core/pi_setup/rover.sh mask     record where the rover's own body is (open space!)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh start    start arena laps in the background (log: ~/demo.log)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh stop     stop the demo
#   bash ~/repos/rpi5-core/pi_setup/rover.sh restart  stop + start again (same mode)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh hallway  the old hallway demo (archived)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh lap      stop + start ARENA LAPS (right lane, press Y)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh status   running or not + last log lines
#   bash ~/repos/rpi5-core/pi_setup/rover.sh log      follow the log live (Ctrl+C to quit watching)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh look     20 s of closest thing front/left/back/right (direction check)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh calibrate  find the rover's front + usable arc (hold a box up when told)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh snapshot   save 2 s of raw scans to ~/rover_data/snapshots
#   bash ~/repos/rpi5-core/pi_setup/rover.sh dashboard  install/restart the web dashboard (http://<pi>:8080)

# The code lives in the git clone (git pull to update). Per-rover settings
# (self mask, LiDAR calibration) live in ~/rover_data so pulls never touch them.
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS=$ROOT/ros2_ws
export ROVER_DATA=~/rover_data
LOG=~/demo.log
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1
# job control: background launches get their own process group and keep Ctrl+C
# (SIGINT) working, so 'stop' can shut them down cleanly instead of force-killing
set -m
# SSH commands don't run ~/.bashrc fully, so set ROS up here
source /opt/ros/jazzy/setup.bash
[ -f ~/sewer/microros_ws/install/setup.bash ] && source ~/sewer/microros_ws/install/setup.bash   # team micro-ROS agent
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash

running() { pgrep -f "ros2 launch rover_bringup" > /dev/null; }
# which launch is up (so the LiDAR tools bring back the same one). Arena laps are the default;
# the hallway demo is archived (start it with: rover.sh hallway)
current_launch() { pgrep -af "ros2 launch rover_bringup" | grep -q demo.launch.py && echo demo.launch.py || echo lap.launch.py; }

stop_demo() {
    if running; then
        pkill -INT -f "ros2 launch rover_bringup"
        for _ in 1 2 3 4 5 6 7 8 9 10; do running || break; sleep 0.5; done
        running && pkill -KILL -f "ros2 launch rover_bringup"
    fi
    # stray nodes from an earlier crash
    pkill -INT -f "rover_bringup/lib/rover_bringup" 2>/dev/null
    pkill -INT -f "rplidar" 2>/dev/null
    # what lap.launch starts for the team's micro-ROS ESP32 (frees /dev/esp32 for the next run)
    pkill -INT -f "micro_ros_agent serial" 2>/dev/null
    pkill -INT -f "sewer/pad_teleop.py" 2>/dev/null
    pkill -INT -f "joy/joy_node" 2>/dev/null
    sleep 1
    echo "stopped"
}

start_demo() {
    if running; then echo "already running (use: restart)"; return; fi
    # the team's micro-ROS ESP32 only joins an agent it finds after booting: reset it first
    [ "${LAUNCH:-lap.launch.py}" = lap.launch.py ] && [ -e /dev/esp32 ] && python3 $ROOT/tools/esp32_reset.py /dev/esp32
    nohup ros2 launch rover_bringup ${LAUNCH:-lap.launch.py} > $LOG 2>&1 &
    sleep 6
    if running; then
        echo "laps started. Rover in the right lane facing up; Triangle on the controller = AUTO, Circle = stop."
        grep -E "Self mask|No self mask|ERROR" $LOG | tail -3
    else
        echo "demo FAILED to start, last log lines:"; tail -20 $LOG
    fi
}

case "$1" in
    setup)
        mkdir -p $ROVER_DATA
        for f in self_mask.json lidar_calib.json; do   # bring over settings from the old ~/rover
            for old in ~/rover_archive ~/rover; do
                [ ! -f $ROVER_DATA/$f ] && [ -f $old/$f ] && cp $old/$f $ROVER_DATA/ && echo "copied $old/$f"
            done
        done
        if [ ! -d $WS/src/rplidar_ros ]; then          # Slamtec LiDAR driver (not in git)
            for old in ~/rover_archive ~/rover; do
                [ -d $old/ros2_ws/src/rplidar_ros ] && cp -r $old/ros2_ws/src/rplidar_ros $WS/src/ && echo "copied LiDAR driver from $old" && break
            done
            [ -d $WS/src/rplidar_ros ] || git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git $WS/src/rplidar_ros
        fi
        # LiDAR name (/dev/rplidar): keep our rule alongside any others (e.g. a teammate's /dev/esp32 rule)
        if [ ! -f /etc/udev/rules.d/99-rplidar.rules ]; then
            sudo cp $ROOT/pi_setup/99-rplidar.rules /etc/udev/rules.d/ && sudo udevadm control --reload \
                && sudo udevadm trigger && echo "installed /dev/rplidar rule"
        fi
        # login shells: load THIS workspace
        sed -i "s#/home/rover/rover/ros2_ws/install/setup.bash#$WS/install/setup.bash#g" ~/.bashrc
        grep -q ROVER_DATA ~/.bashrc || echo "export ROVER_DATA=$ROVER_DATA" >> ~/.bashrc
        cd $WS && colcon build --symlink-install && echo "setup OK: workspace $WS, settings in $ROVER_DATA"
        bash "$0" dashboard
        ;;
    dashboard)
        # web dashboard as a service: starts at boot, restarted after each deploy
        if ! cmp -s $ROOT/pi_setup/rover-dashboard.service /etc/systemd/system/rover-dashboard.service; then
            sudo cp $ROOT/pi_setup/rover-dashboard.service /etc/systemd/system/ && sudo systemctl daemon-reload \
                && sudo systemctl enable rover-dashboard && echo "installed the dashboard service"
        fi
        sudo systemctl restart rover-dashboard && sleep 2
        systemctl is-active --quiet rover-dashboard \
            && echo "dashboard running: open http://192.168.137.50:8080 on the laptop" \
            || { echo "dashboard FAILED:"; journalctl -u rover-dashboard -n 15 --no-pager; }
        ;;
    build)
        cd $WS && colcon build --symlink-install --packages-select rover_bringup && echo "build OK"
        # dashboard picks up the new code: systemd restarts it within a few seconds (no sudo needed)
        pkill -f "rover_bringup.rover_dashboard" 2>/dev/null && echo "dashboard restarting"
        ;;
    mask)
        # always use our own short LiDAR run (the demo is paused and brought back)
        was_running=0
        running && { was_running=1; LAUNCH=$(current_launch); stop_demo > /dev/null; }
        echo "starting the LiDAR for a moment..."
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_mask.log 2>&1 &
        sleep 6
        echo "Keep everything except the rover >0.3 m away (the rear half does not matter)..."
        timeout 30 ros2 run rover_bringup record_self_mask || {
            echo "No mask saved: the LiDAR sent no scans. LiDAR log:"; tail -5 /tmp/lidar_mask.log; }
        stop_demo > /dev/null
        if [ $was_running = 1 ]; then LAUNCH=$LAUNCH start_demo; fi
        ;;
    calibrate)
        # find the rover's front + usable arc with a box held up (demo paused, then brought back)
        was_running=0
        running && { was_running=1; LAUNCH=$(current_launch); stop_demo > /dev/null; }
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_cal.log 2>&1 &
        sleep 6
        timeout 150 ros2 run rover_bringup calibrate_lidar
        stop_demo > /dev/null
        if [ $was_running = 1 ]; then LAUNCH=$LAUNCH start_demo; fi
        ;;
    snapshot)
        # save ~2 s of raw scans to ~/rover_data/snapshots (demo paused, then brought back)
        was_running=0
        running && { was_running=1; LAUNCH=$(current_launch); stop_demo > /dev/null; }
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_snap.log 2>&1 &
        sleep 6
        timeout 30 ros2 run rover_bringup scan_snapshot
        stop_demo > /dev/null
        if [ $was_running = 1 ]; then LAUNCH=$LAUNCH start_demo; fi
        ;;
    look)
        # 20 s of "closest thing front / left / back / right" (demo paused, then brought back)
        was_running=0
        running && { was_running=1; LAUNCH=$(current_launch); stop_demo > /dev/null; }
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_look.log 2>&1 &
        sleep 6
        echo "Closest thing in each direction, as the DRIVING code sees it (0 deg = rover front, + = left):"
        timeout 20 ros2 run rover_bringup scan_monitor --ros-args -r __node:=hallway_follower \
            --params-file "$(ros2 pkg prefix rover_bringup)/share/rover_bringup/config/hallway.yaml" 2>&1 | sed 's/.*\]: //'
        stop_demo > /dev/null
        if [ $was_running = 1 ]; then LAUNCH=$LAUNCH start_demo; fi
        ;;
    start)   LAUNCH=lap.launch.py start_demo ;;                   # arena laps (same as lap)
    lap)     stop_demo; LAUNCH=lap.launch.py start_demo ;;        # arena laps (Bug2)
    hallway) stop_demo; LAUNCH=demo.launch.py start_demo ;;       # archived hallway demo
    diag)    # where is each rover program stuck? (Python stacks go into ~/demo.log)
        for pat in drive_bridge lap_driver; do
            for pid in $(pgrep -f "lib/rover_bringup/$pat"); do
                echo "== $pat pid $pid threads:"; for t in /proc/$pid/task/*; do
                    echo "  $(basename $t) $(cat $t/comm) wchan=$(cat $t/wchan 2>/dev/null)"; done
                kill -USR1 $pid 2>/dev/null
            done
        done
        sleep 1; echo "== serial ports:"; ls -l /dev/esp32 /dev/rplidar 2>&1
        for f in /proc/[0-9]*/fd/*; do l=$(readlink $f 2>/dev/null); case "$l" in /dev/ttyUSB*) echo "  $l used by $(cat /proc/$(echo $f | cut -d/ -f3)/comm)";; esac; done
        echo "== stacks (from demo.log):"; tail -n 60 $LOG | grep -A40 -m2 "Thread\|Current thread" ;;
    fwinfo)  # the team's ESP32 firmware + Pi teleop on this Pi (read-only)
        S=~/sewer
        echo "== ROS packages: $(ros2 pkg list 2>/dev/null | grep -E '^(micro_ros_agent|joy|joy_linux)$' | tr '\n' ' ')"
        ls -d ~/sewer/microros_ws/install 2>&1
        for g in $S/esp32-firmware $S/rpi5-core; do
            [ -d "$g/.git" ] && { echo "== git $g"; git -C "$g" log --format='%ad %h %s' --date=short -6; git -C "$g" branch -a 2>/dev/null | head -5; }
        done
        for f in $S/esp32-firmware/README.md $S/esp32-firmware/micro_ros_driver/*.ino $S/esp32-firmware/micro_ros_driver/*.h \
                 $S/esp32-firmware/compile.sh $S/esp32-firmware/upload.sh $S/pad_teleop.py $S/start_joy_node.sh \
                 $S/start_pad_teleop.sh $S/rover.urdf; do
            [ -f "$f" ] && { echo "=====FILE $f"; cat "$f"; }
        done ;;
    camcheck) # is the Pi camera usable on this Ubuntu? (read-only checks + one test picture)
        echo "== devices:"; ls /dev/video* /dev/media* 2>&1 | head -20
        echo "== tools:"; for t in rpicam-still libcamera-still cam v4l2-ctl ffmpeg; do printf '%s: %s\n' $t "$(command -v $t || echo -)"; done
        echo "== python:"; python3 -c "import picamera2; print('picamera2 OK')" 2>&1 | tail -1
        python3 -c "import cv2; print('opencv', cv2.__version__)" 2>&1 | tail -1
        echo "== packages:"; dpkg -l 2>/dev/null | grep -iE 'libcamera|rpicam|v4l-utils|opencv' | awk '{print $2, $3}' | head
        echo "== kernel:"; journalctl -k -b --no-pager 2>/dev/null | grep -iE 'imx219|camera|unicam|rp1-cfe|csi' | tail -8
        grep -iE 'camera|dtoverlay|imx' /boot/firmware/config.txt 2>/dev/null
        mkdir -p $ROVER_DATA/camera; rm -f $ROVER_DATA/camera/test.jpg
        if command -v rpicam-still >/dev/null; then rpicam-still -n -t 800 --width 1280 --height 960 -o $ROVER_DATA/camera/test.jpg 2>&1 | tail -4
        elif command -v libcamera-still >/dev/null; then libcamera-still -n -t 800 -o $ROVER_DATA/camera/test.jpg 2>&1 | tail -4
        elif command -v ffmpeg >/dev/null && [ -e /dev/video0 ]; then ffmpeg -y -loglevel error -f v4l2 -i /dev/video0 -frames:v 1 $ROVER_DATA/camera/test.jpg 2>&1 | tail -4
        fi
        [ -s $ROVER_DATA/camera/test.jpg ] && echo "PICTURE OK: $(du -h $ROVER_DATA/camera/test.jpg | cut -f1)" || echo "no picture" ;;
    netcheck) # can the Pi reach the internet through the laptop? (for installing camera software)
        ip -4 route | head -3
        python3 $ROOT/tools/nettest.py
        echo "== ping 8.8.8.8:"; ping -c 2 -W 2 8.8.8.8 2>&1 | tail -2
        echo "== DNS:"; getent hosts ports.ubuntu.com github.com pypi.org 2>&1 || echo "DNS FAILED"
        resolvectl status 2>/dev/null | grep -E 'DNS Servers|Current DNS' | head -4
        echo "== apt candidates:"; apt-cache policy rpicam-apps rpicam-apps-lite libcamera-tools python3-libcamera python3-opencv python3-picamera2 2>/dev/null | grep -E '^[a-z]|Candidate' ;;
    tagtest) python3 $ROOT/tools/apriltag_test.py ;;   # find AprilTags in one webcam picture
    camshot) python3 $ROOT/tools/webcam.py $ROVER_DATA/camera/test ${CAM_ARGS:---fmt YUYV} ;;   # USB webcam test picture (grayscale PNG)
    camlive) # webcam view on the dashboard for 3 min (lane paint + obstacles drawn on it)
        if running; then echo "laps are running: the camera is already on (see the Camera card)"
        else echo "camera view for 3 minutes (Stop ends it)"; timeout 180 ros2 run rover_bringup camera_node 2>&1 | tail -20; fi ;;
    stop)    stop_demo ;;
    restart) L=$(current_launch); stop_demo; LAUNCH=$L start_demo ;;   # same mode as before
    status)
        if running; then echo "demo RUNNING"; else echo "demo not running"; fi
        [ -f $LOG ] && tail -8 $LOG
        ;;
    log)     tail -n 30 -f $LOG ;;
    *)       sed -n 2,20p "$0" ;;
esac
