"""joy_enable — AUTO on/off from the game controller (it now talks to the Pi, not the ESP32).

The team's ESP32 firmware is a micro-ROS node (cmd_vel in m/s with encoder speed control,
odom, imu); the controller is read on the Pi by `joy_node`, and `pad_teleop.py` drives
while L1 is held. This node adds the AUTO switch that used to be Y on the ESP32:

  * auto_button (default 2 = Triangle on a PlayStation pad): toggle AUTO
  * stop_button (default 1 = Circle), holding the teleop enable button (L1), or a stick
    pushed past `stick_cancel`: back to MANUAL immediately
  * no /joy for `joy_timeout` s (controller died): MANUAL

Publishes /rover/enable (latched Bool, what lap_driver listens to) and /drive/mode.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def pick(seq, i, default=0):
    return seq[i] if 0 <= i < len(seq) else default


class JoyEnable(Node):

    def __init__(self):
        super().__init__('joy_enable')
        self.declare_parameter('auto_button', 2)
        self.declare_parameter('stop_button', 1)
        self.declare_parameter('teleop_enable_button', 4)
        self.declare_parameter('stick_axes', [0, 1, 3, 4])
        self.declare_parameter('stick_cancel', 0.5)
        self.declare_parameter('joy_timeout', 1.0)
        self.auto = False
        self.prev = []
        self.last_joy = None
        self.enable_pub = self.create_publisher(Bool, 'rover/enable', ENABLE_QOS)
        self.mode_pub = self.create_publisher(String, 'drive/mode', ENABLE_QOS)
        self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.create_timer(0.5, self.check_timeout)
        self.set_auto(False, 'start')

    def p(self, name):
        return self.get_parameter(name).value

    def set_auto(self, on, why):
        changed = on != self.auto
        self.auto = on
        self.enable_pub.publish(Bool(data=on))
        self.mode_pub.publish(String(data='AUTO' if on else 'MANUAL'))
        if changed or why == 'start':
            self.get_logger().info(f"drivetrain mode: {'AUTO' if on else 'MANUAL'} ({why})")

    def on_joy(self, msg):
        self.last_joy = self.get_clock().now()
        b = list(msg.buttons)
        pressed = lambda i: pick(b, i) == 1 and pick(self.prev, i) != 1  # noqa: E731
        why = None
        if self.auto:
            if pick(b, self.p('stop_button')) == 1:
                why = f"stop button {self.p('stop_button')}"
            elif pick(b, self.p('teleop_enable_button')) == 1:
                why = f"teleop button {self.p('teleop_enable_button')} held"
            else:
                for a in self.p('stick_axes'):
                    if abs(pick(msg.axes, a, 0.0)) > self.p('stick_cancel'):
                        why = f'stick axis {a} = {pick(msg.axes, a, 0.0):+.2f}'
                        break
        if why and not pressed(self.p('auto_button')):
            self.set_auto(False, f'controller took over: {why}')
        elif pressed(self.p('auto_button')):
            self.set_auto(not self.auto, 'auto button')
        self.prev = b

    def check_timeout(self):
        if self.auto and self.last_joy is not None:
            age = (self.get_clock().now() - self.last_joy).nanoseconds * 1e-9
            if age > self.p('joy_timeout'):
                self.set_auto(False, 'controller lost')


def main(args=None):
    rclpy.init(args=args)
    node = JoyEnable()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.set_auto(False, 'shutdown')
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
