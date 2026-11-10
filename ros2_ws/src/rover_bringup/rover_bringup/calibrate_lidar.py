"""calibrate_lidar — work out the rover's front/left/right and the usable front arc.

Run with the rover on the floor or a table, LiDAR spinning, nothing within ~1 m
except the rover itself (the laptop cable behind it is fine):

    bash ~/repos/rpi5-core/pi_setup/rover.sh calibrate       (on the Pi)
    .\\rover.ps1 calibrate -Pi rover@192.168.137.50  (from the laptop)

Steps (it tells you when):
  1. hands off: 3 s of the empty room -> where the rover/mount blocks the view
  2. hold a box/book ~50 cm straight in FRONT of the rover until it says "got it"
  3. take it away, then hold it ~50 cm to the front-LEFT (about 45 deg) until "got it"
It saves ~/rover_data/lidar_calib.json (front direction, mirrored or not, usable arc).
hallway_follower and `rover.sh look` use it from the next start.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rover_bringup import lidar_calib as C


class Calibrate(Node):

    def __init__(self):
        super().__init__('calibrate_lidar')
        self.scans = []
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg):
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        self.scans.append(C.scan_bins(angles, msg.ranges))

    def collect(self, seconds):
        start = len(self.scans)
        t_end = time.time() + seconds
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.scans[start:]

    def wait_until_gone(self, base, where, timeout=20.0):
        """After 'got it': wait until nothing is held up near `where` for 1 s."""
        clear, t_end = 0, time.time() + timeout
        while time.time() < t_end and clear < 10:
            n = len(self.scans)
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.scans) == n:
                continue
            a = C.find_object(self.scans[-1], base)
            clear = clear + 1 if (a is None or abs(C.wrap_deg(a - where)) > 20.0) else 0
        if clear < 10:
            raise TimeoutError('the object was never taken away')
        print('    ok, it is gone.', flush=True)

    def wait_for_object(self, base, what, timeout=30.0, not_near=None):
        """Wait until the object is seen in the same place for 5 scans in a row.

        not_near: ignore anything within 20 deg of this raw angle (the last spot).
        """
        print(f'\n>>> Hold a box/book about 50 cm (1.5 ft) to the {what} of the rover,', flush=True)
        print('    at the height of the LiDAR, and keep it still...', flush=True)
        seen, t_end = [], time.time() + timeout
        while time.time() < t_end:
            n = len(self.scans)
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.scans) == n:
                continue
            a = C.find_object(self.scans[-1], base)
            if a is not None and not_near is not None and abs(C.wrap_deg(a - not_near)) < 20.0:
                a = None
            if a is None or (seen and abs(C.wrap_deg(a - seen[-1])) > 8.0):
                seen = [] if a is None else [a]
                continue
            seen.append(a)
            if len(seen) >= 5:
                x = sum(math.cos(math.radians(v)) for v in seen)
                y = sum(math.sin(math.radians(v)) for v in seen)
                got = math.degrees(math.atan2(y, x))
                print(f'    got it (raw LiDAR angle {got:+.0f} deg). Take it away.', flush=True)
                self.wait_until_gone(base, got)
                return got
        raise TimeoutError(f'did not see anything to the {what} within {timeout:.0f} s '
                           '(hold it 30-100 cm away, at the LiDAR\'s height, and keep still)')


def main(args=None):
    rclpy.init(args=args)
    node = Calibrate()
    ok = False
    try:
        print('Waiting for the LiDAR...', flush=True)
        if not node.collect(4.0):
            print('No scans from the LiDAR. Is it plugged in and spinning?')
            return
        print('\n>>> Hands off, nothing within 1 m of the rover: reading the empty room (3 s)...',
              flush=True)
        empty = node.collect(3.0)
        base = C.baseline(empty)
        blocked = C.blocked_bins(empty)

        raw_front = node.wait_for_object(base, 'FRONT')
        raw_left = node.wait_for_object(base, 'front-LEFT (about 45 deg)', not_near=raw_front)
        yaw, mirrored = C.orientation(raw_front, raw_left)
        left, right = C.free_arc(blocked, yaw, mirrored)

        calib = {'laser_yaw_deg': round(yaw, 1), 'laser_mirror': mirrored,
                 'fov_left_deg': round(left, 1), 'fov_right_deg': round(right, 1),
                 'raw_front_deg': round(raw_front, 1), 'raw_left_deg': round(raw_left, 1),
                 'blocked_raw_bins': sorted(blocked),
                 'made': time.strftime('%Y-%m-%d %H:%M')}
        path = C.save(calib)
        print('\n=== Calibration saved:', path)
        print(f'  rover front = raw LiDAR {raw_front:+.0f} deg; scan mirrored: {mirrored}')
        print(f'  usable arc: {left:.0f} deg to the left, {right:.0f} deg to the right of straight ahead')
        if left < 60 or right < 60:
            print('  NOTE: less than 60 deg on a side: the rover will be fairly blind beside itself.')
        print('  The demo uses this from its next start (rover.sh restart).')
        ok = True
    except (TimeoutError, ValueError) as e:
        print(f'\nCalibration NOT saved: {e}. Run it again.')
    except KeyboardInterrupt:
        print('\nCancelled, nothing saved.')
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass
    if not ok:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
