"""Arena laps.

    ros2 launch rover_bringup lap.launch.py                 # team micro-ROS ESP32 (default)
    ros2 launch rover_bringup lap.launch.py drive:=bridge   # old mecanum_auto ESP32 sketch

drive:=microros (the ESP32 runs the team's micro_ros_driver firmware since Nov 14):
  LiDAR + webcam (camera:=false to skip) + lap_driver + micro-ROS agent (/dev/esp32 @ 921600) + joy_node + pad_teleop.py
  (hold L1 to drive by hand) + joy_enable (Triangle = AUTO, Circle / L1 / a stick = manual).
drive:=bridge: LiDAR + lap_driver + drive_bridge (Y on the controller = AUTO).
Rover in the RIGHT lane facing up it, then switch to AUTO.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TELEOP = os.path.expanduser('~/sewer/pad_teleop.py')


def have(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def generate_launch_description():
    share = get_package_share_directory('rover_bringup')
    micro = LaunchConfigurationEquals('drive', 'microros')
    bridge = LaunchConfigurationEquals('drive', 'bridge')
    acts = [
        DeclareLaunchArgument('drive', default_value='microros'),
        DeclareLaunchArgument('esp_port', default_value='/dev/esp32'),
        DeclareLaunchArgument('camera', default_value='true'),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lidar.launch.py')))]),
        Node(package='rover_bringup', executable='lap_driver', name='lap_driver', output='screen',
             parameters=[os.path.join(share, 'config', 'lap.yaml')]),
        # webcam: lane paint + cars on the floor (lap_driver adds the cars to its map)
        Node(package='rover_bringup', executable='camera_node', name='camera_node', output='screen',
             condition=LaunchConfigurationEquals('camera', 'true')),
        # --- old sketch: our serial bridge ---
        Node(package='rover_bringup', executable='drive_bridge', name='drive_bridge', output='screen',
             parameters=[os.path.join(share, 'config', 'drive.yaml')], condition=bridge),
        # --- team micro-ROS firmware ---
        Node(package='rover_bringup', executable='joy_enable', name='joy_enable', output='screen',
             condition=micro),
    ]
    if have('micro_ros_agent'):
        acts.append(Node(package='micro_ros_agent', executable='micro_ros_agent', name='micro_ros_agent',
                         output='screen', condition=micro,
                         arguments=['serial', '--dev', LaunchConfiguration('esp_port'), '-b', '921600']))
    else:
        acts.append(LogInfo(msg='micro_ros_agent package NOT found (source ~/sewer/microros_ws?)',
                            condition=micro))
    if have('joy'):
        acts.append(Node(package='joy', executable='joy_node', name='joy_node', output='screen',
                         parameters=[{'autorepeat_rate': 20.0}], condition=micro))
    else:
        acts.append(LogInfo(msg='joy package NOT found: no controller', condition=micro))
    if os.path.exists(TELEOP):
        acts.append(ExecuteProcess(cmd=['python3', TELEOP, '--ros-args', '-p', 'dpad_speed:=0.2'],
                                   name='pad_teleop', output='screen', condition=micro))
    return LaunchDescription(acts)
