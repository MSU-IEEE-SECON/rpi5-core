#!/bin/bash
# rover.sh — one-word commands for the rover, run ON THE PI.
# (From the laptop, use rover.ps1 in the repo root, which calls this over SSH.)
#
#   bash ~/rover/pi_setup/rover.sh build    rebuild the ROS package after new code
#   bash ~/rover/pi_setup/rover.sh mask     record where the rover's own body is (open space!)
#   bash ~/rover/pi_setup/rover.sh start    start the demo in the background (log: ~/demo.log)
#   bash ~/rover/pi_setup/rover.sh stop     stop the demo
#   bash ~/rover/pi_setup/rover.sh restart  stop + start
#   bash ~/rover/pi_setup/rover.sh status   running or not + last log lines
#   bash ~/rover/pi_setup/rover.sh log      follow the log live (Ctrl+C to quit watching)

WS=~/rover/ros2_ws
LOG=~/demo.log
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1
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
    build)
        cd $WS && colcon build --symlink-install --packages-select rover_bringup && echo "build OK"
        ;;
    mask)
        was_running=0
        if running; then was_running=1; else
            echo "starting the LiDAR for a moment..."
            nohup ros2 launch rover_bringup lidar.launch.py > /tmp/lidar_mask.log 2>&1 &
            sleep 6
        fi
        echo "Keep everything except the rover >0.5 m away..."
        timeout 30 ros2 run rover_bringup record_self_mask
        if [ $was_running = 0 ]; then stop_demo > /dev/null; pkill -INT -f "lidar.launch.py"; fi
        [ $was_running = 1 ] && echo "now run: restart  (so the follower loads the mask)"
        ;;
    start)   start_demo ;;
    stop)    stop_demo ;;
    restart) stop_demo; start_demo ;;
    status)
        if running; then echo "demo RUNNING"; else echo "demo not running"; fi
        [ -f $LOG ] && tail -8 $LOG
        ;;
    log)     tail -n 30 -f $LOG ;;
    *)       sed -n 2,13p "$0" ;;
esac
