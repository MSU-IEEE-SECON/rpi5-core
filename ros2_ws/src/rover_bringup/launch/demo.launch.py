"""Hallway demo: LiDAR + hallway follower + drive bridge to the ESP32.

    ros2 launch rover_bringup demo.launch.py                      # normal
    ros2 launch rover_bringup demo.launch.py use_drive:=false     # dry run, no ESP32
    ros2 launch rover_bringup demo.launch.py use_button:=true     # also a GPIO17 button

Starts DISABLED. To go: press Y on the controller (ESP32 -> AUTO), or the GPIO
button, or:
    ros2 topic pub --once /rover/enable std_msgs/msg/Bool "{data: true}"
Stop: Y / B / touch a stick on the controller.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rover_bringup')
    hallway_cfg = os.path.join(share, 'config', 'hallway.yaml')
    drive_cfg = os.path.join(share, 'config', 'drive.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_button', default_value='false'),
        DeclareLaunchArgument('use_drive', default_value='true'),
        DeclareLaunchArgument('button_pin', default_value='17'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'lidar.launch.py'))),

        Node(package='rover_bringup', executable='hallway_follower',
             name='hallway_follower', output='screen', parameters=[hallway_cfg]),

        Node(package='rover_bringup', executable='drive_bridge', name='drive_bridge',
             output='screen', parameters=[drive_cfg],
             condition=IfCondition(LaunchConfiguration('use_drive'))),

        Node(package='rover_bringup', executable='start_button', name='start_button',
             output='screen', condition=IfCondition(LaunchConfiguration('use_button')),
             parameters=[{'button_pin': ParameterValue(
                 LaunchConfiguration('button_pin'), value_type=int)}]),
    ])
