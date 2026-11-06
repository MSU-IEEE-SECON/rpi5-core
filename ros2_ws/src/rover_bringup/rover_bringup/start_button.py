"""start_button — a physical push button toggles driving on/off.

Press once: publishes /rover/enable = true  (rover starts)
Press again: publishes /rover/enable = false (rover stops)

Wiring (default, BCM numbering):
    one leg of a momentary push button -> GPIO17 (physical pin 11)
    other leg                          -> GND    (physical pin 9 or 14)
No resistor needed: the Pi's internal pull-up is used. Pressed = pin pulled to GND.

Optional status LED: set led_pin (e.g. 27 = physical pin 13) through a ~330 ohm
resistor to GND. On = driving.

Needs:  sudo apt install -y python3-gpiozero python3-lgpio
UNTESTED on this Pi 5 / Ubuntu combo. If it errors about GPIO access, see
docs/05_hallway_demo.md, "Button troubleshooting".
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class StartButton(Node):

    def __init__(self):
        super().__init__('start_button')
        self.declare_parameter('button_pin', 17)
        self.declare_parameter('led_pin', -1)          # -1 = no LED
        self.declare_parameter('bounce_s', 0.05)
        self.enabled = False
        self.pub = self.create_publisher(Bool, 'rover/enable', ENABLE_QOS)
        self.pub.publish(Bool(data=False))

        try:
            from gpiozero import LED, Button
        except ImportError:
            self.get_logger().error(
                'gpiozero not installed: sudo apt install -y python3-gpiozero python3-lgpio')
            raise

        pin = int(self.get_parameter('button_pin').value)
        self.button = Button(pin, pull_up=True,
                             bounce_time=float(self.get_parameter('bounce_s').value))
        self.button.when_pressed = self.toggle
        led_pin = int(self.get_parameter('led_pin').value)
        self.led = LED(led_pin) if led_pin >= 0 else None
        self.get_logger().info(f'Start button on GPIO{pin}. Press to start / stop.')

    def toggle(self):
        # gpiozero calls this from its own thread; publishing from there is fine in rclpy
        self.enabled = not self.enabled
        self.pub.publish(Bool(data=self.enabled))
        if self.led:
            self.led.value = self.enabled
        self.get_logger().info('START' if self.enabled else 'STOP')

    def destroy_node(self):
        try:
            self.pub.publish(Bool(data=False))
            if self.led:
                self.led.off()
        except Exception:  # noqa: BLE001
            pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StartButton()
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
