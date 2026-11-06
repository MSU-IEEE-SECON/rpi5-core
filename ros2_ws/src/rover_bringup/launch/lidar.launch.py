"""Bring up the RPLIDAR and a static transform base_link -> laser.

Usage:
    ros2 launch rover_bringup lidar.launch.py
    ros2 launch rover_bringup lidar.launch.py serial_baudrate:=460800
    ros2 launch rover_bringup lidar.launch.py serial_port:=/dev/ttyUSB0

Port and baud come from config/lidar.yaml unless overridden on the command line.

The base_link -> laser transform says where the LiDAR sits on the rover.
x/y/z in metres from the rover's centre, yaw in radians. If "front" in the
scan doesn't match the rover's front, change laser_yaw (e.g. 3.14159).
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_rplidar_executable():
    """The rplidar_ros executable name differs between releases.

    Newer Slamtec releases install 'rplidar_node'; some binary builds install
    'rplidar_composition'. Use whichever one is actually there.
    """
    lib_dir = os.path.join(get_package_prefix('rplidar_ros'), 'lib', 'rplidar_ros')
    for name in ('rplidar_node', 'rplidar_composition'):
        if os.path.exists(os.path.join(lib_dir, name)):
            return name
    found = sorted(os.listdir(lib_dir)) if os.path.isdir(lib_dir) else []
    raise RuntimeError(f'No rplidar executable in {lib_dir} (found: {found})')


def make_rplidar(context):
    config = os.path.join(
        get_package_share_directory('rover_bringup'), 'config', 'lidar.yaml')
    overrides = {}
    port = LaunchConfiguration('serial_port').perform(context)
    baud = LaunchConfiguration('serial_baudrate').perform(context)
    if port:
        overrides['serial_port'] = port
    if baud:
        overrides['serial_baudrate'] = int(baud)
    params = [config] + ([overrides] if overrides else [])
    return [Node(
        package='rplidar_ros',
        executable=find_rplidar_executable(),
        name='rplidar_node',
        output='screen',
        parameters=params,
    )]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('serial_port', default_value='',
                              description='Override serial port (default from lidar.yaml)'),
        DeclareLaunchArgument('serial_baudrate', default_value='',
                              description='Override baud: 115200 A1/A2M8, 256000 A2M12/A3/S1, '
                                          '460800 C1, 1000000 S2/S3'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.10'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
    ]

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    return LaunchDescription(args + [OpaqueFunction(function=make_rplidar), laser_tf])
