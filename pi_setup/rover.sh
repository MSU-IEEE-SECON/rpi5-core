#!/bin/bash
# rover.sh - one-word commands for the rover, run ON THE PI.
# (From the laptop, use rover.ps1 in the repo root, which calls this over SSH.)
#
#   bash ~/repos/rpi5-core/pi_setup/rover.sh setup    one time per Pi: LiDAR driver, data folder, ~/.bashrc, build
#   bash ~/repos/rpi5-core/pi_setup/rover.sh build    rebuild the ROS package after new code
#   bash ~/repos/rpi5-core/pi_setup/rover.sh mask     record where the rover's own body is (open space!)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh start    start the demo in the background (log: ~/demo.log)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh stop     stop the demo
#   bash ~/repos/rpi5-core/pi_setup/rover.sh restart  stop + start
#   bash ~/repos/rpi5-core/pi_setup/rover.sh status   running or not + last log lines
#   bash ~/repos/rpi5-core/pi_setup/rover.sh log      follow the log live (Ctrl+C to quit watching)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh look     20 s of closest thing front/left/back/right (direction check)
#   bash ~/repos/rpi5-core/pi_setup/rover.sh calibrate  find the rover's front + usable arc (hold a box up when told)

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
[ -f $WS/install/setup.bash ] && source $WS/install/setup.bash

running() { pgrep -f "ros2 launch rover_bringup" > /dev/null; }

stop_demo() {
    if running; then
        pkill -INT -f "ros2 launch rover_bringup"
        for _ in 1 2 3 4 5 6 7 8 9 10; do running || break; sleep 0.5; done
        running && pkill -KILL -f "ros2 launch rover_bringup"
    fi
    # stray nodes from an earlier crash
    pkill -INT -f "rover_bringup/lib/rover_bringup" 2>/dev/null
    pkill -INT -f "rplidar" 2>/dev/null
    sleep 1
    echo "stopped"
}

start_demo() {
    if running; then echo "already running (use: restart)"; return; fi
    nohup ros2 launch rover_bringup demo.launch.py > $LOG 2>&1 &
    sleep 6
    if running; then
        echo "demo started. Connect the controller, press Y to drive."
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
        # login shells: load THIS workspace
        sed -i "s#/home/rover/rover/ros2_ws/install/setup.bash#$WS/install/setup.bash#g" ~/.bashrc
        grep -q ROVER_DATA ~/.bashrc || echo "export ROVER_DATA=$ROVER_DATA" >> ~/.bashrc
        cd $WS && colcon build --symlink-install && echo "setup OK: workspace $WS, settings in $ROVER_DATA"
        ;;
    build)
        cd $WS && colcon build --symlink-install --packages-select rover_bringup && echo "build OK"
        ;;
    mask)
        # always use our own short LiDAR run (the demo is paused and brought back)
        was_running=0
        running && { was_running=1; stop_demo > /dev/null; }
        echo "starting the LiDAR for a moment..."
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_mask.log 2>&1 &
        sleep 6
        echo "Keep everything except the rover >0.3 m away (the rear half does not matter)..."
        timeout 30 ros2 run rover_bringup record_self_mask || {
            echo "No mask saved: the LiDAR sent no scans. LiDAR log:"; tail -5 /tmp/lidar_mask.log; }
        stop_demo > /dev/null
        [ $was_running = 1 ] && start_demo
        ;;
    calibrate)
        # find the rover's front + usable arc with a box held up (demo paused, then brought back)
        was_running=0
        running && { was_running=1; stop_demo > /dev/null; }
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_cal.log 2>&1 &
        sleep 6
        timeout 150 ros2 run rover_bringup calibrate_lidar
        stop_demo > /dev/null
        [ $was_running = 1 ] && start_demo
        ;;
    look)
        # 20 s of "closest thing front / left / back / right" (demo paused, then brought back)
        was_running=0
        running && { was_running=1; stop_demo > /dev/null; }
        nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_look.log 2>&1 &
        sleep 6
        echo "Closest thing in each direction, as the DRIVING code sees it (0 deg = rover front, + = left):"
        timeout 20 ros2 run rover_bringup scan_monitor --ros-args -r __node:=hallway_follower \
            --params-file "$(ros2 pkg prefix rover_bringup)/share/rover_bringup/config/hallway.yaml" 2>&1 | sed 's/.*\]: //'
        stop_demo > /dev/null
        [ $was_running = 1 ] && start_demo
        ;;
    start)   start_demo ;;
    stop)    stop_demo ;;
    restart) stop_demo; start_demo ;;
    status)
        if running; then echo "demo RUNNING"; else echo "demo not running"; fi
        [ -f $LOG ] && tail -8 $LOG
        ;;
    log)     tail -n 30 -f $LOG ;;
    *)       sed -n 2,16p "$0" ;;
esac
