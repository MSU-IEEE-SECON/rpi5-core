"""scan_snapshot — save ~2 s of raw LiDAR scans to a file for offline analysis.

    bash ~/repos/rpi5-core/pi_setup/rover.sh snapshot      (on the Pi)
    .\\rover.ps1 snapshot -Pi rover@192.168.137.50          (laptop: also copies it back)

Writes ~/rover_data/snapshots/scan_<time>.json and latest.json:
raw angles/ranges exactly as the driver publishes them (no calibration applied),
plus a copy of the current lidar_calib.json so the file is self-describing.
"""
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rover_bringup import lidar_calib


class Snapshot(Node):

    def __init__(self):
        super().__init__('scan_snapshot')
        self.declare_parameter('scans', 20)
        self.want = int(self.get_parameter('scans').value)
        self.scans, self.meta = [], None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        if self.meta is None:
            self.meta = {'angle_min': msg.angle_min, 'angle_increment': msg.angle_increment,
                         'range_min': msg.range_min, 'range_max': msg.range_max,
                         'n': len(msg.ranges)}
        self.scans.append([round(r, 4) if r == r and r != float('inf') else None
                           for r in msg.ranges])


def main(args=None):
    rclpy.init(args=args)
    node = Snapshot()
    t_end = time.time() + 15.0
    try:
        while len(node.scans) < node.want and time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    if not node.scans:
        print('No scans received: is the LiDAR running?')
        raise SystemExit(1)
    try:
        calib = lidar_calib.load()
    except Exception:  # noqa: BLE001
        calib = None
    out_dir = os.path.expanduser(os.path.join(os.environ.get('ROVER_DATA', '~/rover_data'),
                                              'snapshots'))
    os.makedirs(out_dir, exist_ok=True)
    data = dict(node.meta, scans=node.scans, calib=calib,
                made=time.strftime('%Y-%m-%d %H:%M:%S'))
    name = os.path.join(out_dir, time.strftime('scan_%Y%m%d_%H%M%S.json'))
    for path in (name, os.path.join(out_dir, 'latest.json')):
        with open(path, 'w') as f:
            json.dump(data, f)
    print(f'Saved {len(node.scans)} scans to {name}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
