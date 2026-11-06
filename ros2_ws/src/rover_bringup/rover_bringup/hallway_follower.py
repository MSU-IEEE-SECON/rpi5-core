"""hallway_follower — drive down a hallway, keep centred, steer around obstacles.

Subscribes:  /scan           sensor_msgs/LaserScan   (from the RPLIDAR)
             /rover/enable   std_msgs/Bool           (start/stop; button or command line)
Publishes:   /cmd_vel        geometry_msgs/Twist     (linear.x m/s, angular.z rad/s)
             /hallway/state  std_msgs/String         (FOLLOW/AVOID/PIVOT/BLOCKED/...)

Starts DISABLED. It only drives after /rover/enable gets `true`:
    ros2 topic pub --once /rover/enable std_msgs/msg/Bool "{data: true}"
    ros2 topic pub --once /rover/enable std_msgs/msg/Bool "{data: false}"

Safety:
  * publishes zero velocity when disabled, when BLOCKED, and if /scan stops
    arriving for more than scan_timeout_s (LiDAR unplugged, driver crashed...)
  * the motor driver should ALSO stop on its own if /cmd_vel goes quiet
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from rover_bringup.hallway_logic import HallwayLogic, Params, scan_to_xy

# Latched (transient-local) so a late-starting node still gets the last enable state.
ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class HallwayFollower(Node):

    def __init__(self):
        super().__init__('hallway_follower')
        # every field of Params becomes a ROS parameter (see config/hallway.yaml)
        defaults = Params()
        for name, val in vars(defaults).items():
            self.declare_parameter(name, val)
        self.declare_parameter('laser_yaw', 0.0)        # rad, LiDAR rotation on the rover
        self.declare_parameter('scan_timeout_s', 0.5)
        self.declare_parameter('start_enabled', False)
        self.declare_parameter('log_period_s', 0.5)

        self.logic = HallwayLogic(self._read_params())
        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.last_scan_time = None
        self.last_log = 0.0
        self.last_state = 'DISABLED'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'hallway/state', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, 'rover/enable', self.on_enable, ENABLE_QOS)
        self.create_timer(0.1, self.watchdog)
        self.add_on_set_parameters_callback(self._on_params)

        self.get_logger().info(
            'hallway_follower ready, DISABLED. Start with the button or:  '
            'ros2 topic pub --once /rover/enable std_msgs/msg/Bool "{data: true}"')

    # -- parameters -----------------------------------------------------------
    def _read_params(self):
        p = Params()
        for name in vars(p):
            setattr(p, name, type(getattr(p, name))(self.get_parameter(name).value))
        return p

    def _on_params(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if hasattr(self.logic.p, prm.name):
                setattr(self.logic.p, prm.name, type(getattr(self.logic.p, prm.name))(prm.value))
        return SetParametersResult(successful=True)

    # -- callbacks ------------------------------------------------------------
    def on_enable(self, msg: Bool):
        if msg.data != self.enabled:
            self.enabled = msg.data
            self.get_logger().info('ENABLED: driving' if self.enabled else 'DISABLED: stopping')
            if not self.enabled:
                self.stop()

    def on_scan(self, msg: LaserScan):
        self.last_scan_time = self.get_clock().now()
        yaw = float(self.get_parameter('laser_yaw').value)
        angles = [msg.angle_min + i * msg.angle_increment + yaw for i in range(len(msg.ranges))]
        pts = scan_to_xy(angles, msg.ranges, self.logic.p.min_range, msg.range_max)
        d = self.logic.decide(pts)

        state = d.state if self.enabled else 'DISABLED'
        self.state_pub.publish(String(data=state))
        if self.enabled:
            cmd = Twist()
            cmd.linear.x = float(d.linear)
            cmd.angular.z = float(d.angular)
            self.cmd_pub.publish(cmd)
        self._log(state, d)

    def watchdog(self):
        if not self.enabled:
            return
        now = self.get_clock().now()
        stale = (self.last_scan_time is None or
                 (now - self.last_scan_time).nanoseconds * 1e-9 >
                 float(self.get_parameter('scan_timeout_s').value))
        if stale:
            self.stop()
            if self.last_state != 'NO_SCAN':
                self.get_logger().warn('No /scan data: stopping')
                self.last_state = 'NO_SCAN'
            self.state_pub.publish(String(data='NO_SCAN'))

    # -- helpers --------------------------------------------------------------
    def stop(self):
        self.cmd_pub.publish(Twist())

    def _log(self, state, d):
        t = self.get_clock().now().nanoseconds * 1e-9
        if state != self.last_state or t - self.last_log >= float(
                self.get_parameter('log_period_s').value):
            i = d.info
            fmt = lambda v: '  -- ' if v is None or not math.isfinite(v) else f'{v:4.2f}'  # noqa: E731
            self.get_logger().info(
                f"{state:8s} v={d.linear:+.2f} w={d.angular:+.2f} | "
                f"L={fmt(i.get('L'))} R={fmt(i.get('R'))} front={fmt(i.get('front'))} "
                f"hall={i.get('hall_deg', 0.0):+5.1f}deg target_y={fmt(i.get('target_y'))}")
            self.last_log = t
            self.last_state = state

    def destroy_node(self):
        try:
            self.stop()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HallwayFollower()
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
