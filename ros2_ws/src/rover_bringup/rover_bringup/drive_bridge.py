"""drive_bridge — sends /cmd_vel to the ESP32 drivetrain over USB serial.

Subscribes:  /cmd_vel        geometry_msgs/Twist  (linear.x, linear.y, angular.z)
Publishes:   /drive/mode     std_msgs/String      MANUAL / AUTO / DISCONNECTED
             /rover/enable   std_msgs/Bool        (latched) true while the ESP32 is in
                                                  AUTO, so pressing Y on the controller
                                                  starts the autonomy
Serial:      115200 baud, protocol in drive_protocol.py / firmware/mecanum_auto/auto_drive.h

Sends a command every 1/send_rate_hz seconds (also when nothing changes: the
ESP32 stops the motors if commands stop for 300 ms). If /cmd_vel itself goes
stale (> cmd_timeout_s) it sends zeros.

Speed calibration: max_linear_mps / max_strafe_mps / max_angular_radps are how
fast the rover goes at FULL power. Measure them (docs/06_drivetrain_link.md) and
set them in config/drive.yaml.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from rover_bringup.drive_protocol import format_drive_line, parse_status, pick_port

ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class DriveBridge(Node):

    def __init__(self):
        super().__init__('drive_bridge')
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_linear_mps', 0.5)
        self.declare_parameter('max_strafe_mps', 0.4)
        self.declare_parameter('max_angular_radps', 2.5)
        self.declare_parameter('send_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_s', 0.5)
        self.declare_parameter('enable_from_controller', True)
        self.declare_parameter('echo_esp32_log', True)

        self.ser = None
        self.rx = b''
        self.mode = 'DISCONNECTED'
        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.last_port_warn = 0.0

        self.mode_pub = self.create_publisher(String, 'drive/mode', 10)
        self.enable_pub = self.create_publisher(Bool, 'rover/enable', ENABLE_QOS)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        rate = float(self.get_parameter('send_rate_hz').value)
        self.create_timer(1.0 / rate, self.tick)

    # -- serial ---------------------------------------------------------------
    def ensure_open(self):
        if self.ser is not None:
            return True
        now = self.now_s()
        port, msg = pick_port(str(self.get_parameter('port').value))
        if port is None:
            if now - self.last_port_warn > 5.0:
                self.get_logger().warn(msg)
                self.last_port_warn = now
            return False
        try:
            import serial
            s = serial.Serial()
            s.port = port
            s.baudrate = int(self.get_parameter('baud').value)
            s.timeout = 0
            s.write_timeout = 0.05
            # don't toggle DTR/RTS: on most ESP32 dev boards that resets the chip
            s.dtr = False
            s.rts = False
            s.open()
            self.ser = s
            self.get_logger().info(f'{msg}: connected')
            return True
        except Exception as e:  # noqa: BLE001
            if now - self.last_port_warn > 5.0:
                self.get_logger().warn(f'cannot open {port}: {e}')
                self.last_port_warn = now
            return False

    def close(self, why):
        if self.ser is not None:
            self.get_logger().warn(f'ESP32 serial closed: {why}')
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
        self.ser = None
        self.set_mode('DISCONNECTED')

    # -- ROS ------------------------------------------------------------------
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.now_s()

    def tick(self):
        if not self.ensure_open():
            return
        fresh = (self.last_cmd_time is not None and
                 self.now_s() - self.last_cmd_time <= float(self.get_parameter('cmd_timeout_s').value))
        c = self.last_cmd if fresh else Twist()
        line = format_drive_line(
            c.linear.x, c.linear.y, c.angular.z,
            float(self.get_parameter('max_linear_mps').value),
            float(self.get_parameter('max_strafe_mps').value),
            float(self.get_parameter('max_angular_radps').value))
        try:
            self.ser.write(line.encode())
            self.read_lines()
        except Exception as e:  # noqa: BLE001
            self.close(str(e))

    def read_lines(self):
        data = self.ser.read(self.ser.in_waiting or 0)
        if not data:
            return
        self.rx += data
        if len(self.rx) > 4096:
            self.rx = self.rx[-1024:]
        *lines, self.rx = self.rx.split(b'\n')
        for raw in lines:
            line = raw.decode(errors='replace').strip()
            if not line:
                continue
            st = parse_status(line)
            if st:
                self.set_mode(st[0])
            elif self.get_parameter('echo_esp32_log').value:
                self.get_logger().info(f'esp32: {line}')

    def set_mode(self, mode):
        if mode == self.mode:
            return
        self.get_logger().info(f'drivetrain mode: {self.mode} -> {mode}')
        self.mode = mode
        self.mode_pub.publish(String(data=mode))
        if self.get_parameter('enable_from_controller').value:
            self.enable_pub.publish(Bool(data=(mode == 'AUTO')))

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def destroy_node(self):
        try:
            if self.ser is not None:
                self.ser.write(b'D 0 0 0\n')
        except Exception:  # noqa: BLE001
            pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriveBridge()
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
