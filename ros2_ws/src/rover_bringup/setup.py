from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rover_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bowen',
    maintainer_email='bowengoff1@gmail.com',
    description='Launch files, config and utility nodes for the capstone rover.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'scan_monitor = rover_bringup.scan_monitor:main',
            'hallway_follower = rover_bringup.hallway_follower:main',
            'start_button = rover_bringup.start_button:main',
            'drive_bridge = rover_bringup.drive_bridge:main',
            'record_self_mask = rover_bringup.record_self_mask:main',
            'calibrate_lidar = rover_bringup.calibrate_lidar:main',
            'scan_snapshot = rover_bringup.scan_snapshot:main',
            'lap_driver = rover_bringup.lap_driver:main',
            'joy_enable = rover_bringup.joy_enable:main',
            'camera_node = rover_bringup.camera_node:main',
        ],
    },
)
