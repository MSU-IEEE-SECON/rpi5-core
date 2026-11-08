"""scan_monitor — prints the closest obstacle in four sectors from /scan.

A quick sanity check for the LiDAR and the starting point for obstacle
avoidance. Angles follow ROS convention in the laser frame:
0 rad = straight ahead (+x), positive = counter-clockwise (to the left).

    ros2 run rover_bringup scan_monitor
    ros2 run rover_bringup scan_monitor --ros-args -p sector_width_deg:=60.0
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SECTORS = {          # centre angle in degrees
    'front': 0.0,
    'left': 90.0,
    'back': 180.0,
    'right': -90.0,
}


def wrap_deg(a):
    """Wrap an angle to (-180, 180]."""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


class ScanMonitor(Node):

    def __init__(self):
        super().__init__('scan_monitor')
        self.declare_parameter('sector_width_deg', 40.0)
        self.declare_parameter('print_period_s', 0.5)
        # same direction fix as hallway_follower (rover.sh look loads hallway.yaml)
        self.declare_parameter('laser_yaw', 0.0)
        self.declare_parameter('laser_mirror', False)
        self.latest = None
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(self.get_parameter('print_period_s').value, self.report)
        self.get_logger().info('Waiting for /scan ...')

    def on_scan(self, msg: LaserScan):
        self.latest = msg

    def sector_min(self, msg, centre_deg, width_deg):
        best = math.inf
        best_angle = None
        half = width_deg / 2.0
        yaw = math.degrees(float(self.get_parameter('laser_yaw').value))
        sign = -1.0 if bool(self.get_parameter('laser_mirror').value) else 1.0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            ang = wrap_deg(yaw + sign * math.degrees(msg.angle_min + i * msg.angle_increment))
            if abs(wrap_deg(ang - centre_deg)) <= half and r < best:
                best, best_angle = r, ang
        return best, best_angle

    def report(self):
        msg = self.latest
        if msg is None:
            return
        width = self.get_parameter('sector_width_deg').value
        valid = sum(1 for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max)
        parts = []
        for name, centre in SECTORS.items():
            d, a = self.sector_min(msg, centre, width)
            parts.append(f'{name:>5}: ' + ('  ---  ' if a is None else f'{d:5.2f} m @{a:+6.1f}°'))
        self.get_logger().info(' | '.join(parts) + f'  ({valid}/{len(msg.ranges)} pts)')


def main(args=None):
    rclpy.init(args=args)
    node = ScanMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
