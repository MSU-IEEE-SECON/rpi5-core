"""record_self_mask — learn where the rover's OWN body shows up in the LiDAR scan.

The LiDAR sees bits of the rover (mount posts, wires, the chassis edge) at
fixed angles, very close. Those look like obstacles right in front of the
robot, which makes it swerve and stop at nothing. This tool records them once
so hallway_follower can ignore them.

How to use (rover in OPEN space, nothing within ~0.5 m of it, LiDAR running):
    ros2 launch rover_bringup lidar.launch.py      # (or demo.launch.py, not driving)
    ros2 run rover_bringup record_self_mask
It watches ~30 scans, prints the angles it found, saves ~/rover/self_mask.json
and exits. Restart the demo afterwards so the follower loads it.

Re-run it whenever something on the rover moves (LiDAR re-mounted, new parts).

How it works: angles are split into 1 degree bins. A bin is "self" if a return
closer than max_range shows up there in at least half of the scans. The mask
stores the furthest such return per bin; the follower drops anything at that
angle no further than that (+ a small margin). Things further away at the
same angle are still seen.
"""
import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def build_mask(scans, max_range, bin_deg, min_fraction=0.5):
    """scans: list of [(angle_rad, range_m), ...]. Returns {bin_index: max_range_m}."""
    n_bins = int(round(360.0 / bin_deg))
    hits = {}   # bin -> list of ranges (one per scan, the closest that scan saw)
    for scan in scans:
        seen = {}
        for a, r in scan:
            if not math.isfinite(r) or r <= 0.0 or r > max_range:
                continue
            b = int(math.floor(math.degrees(a) % 360.0 / bin_deg)) % n_bins
            seen[b] = min(r, seen.get(b, math.inf))
        for b, r in seen.items():
            hits.setdefault(b, []).append(r)
    need = max(1, int(math.ceil(min_fraction * len(scans))))
    return {b: max(rs) for b, rs in hits.items() if len(rs) >= need}


def sectors(mask, bin_deg):
    """Group masked bins into readable angle ranges (degrees, -180..180)."""
    out, run = [], []
    for b in sorted(mask):
        if run and b != run[-1] + 1:
            out.append(run)
            run = []
        run.append(b)
    if run:
        out.append(run)
    # join a run that wraps through 0
    if len(out) > 1 and out[0][0] == 0 and out[-1][-1] == int(round(360 / bin_deg)) - 1:
        out[0] = out.pop() + out[0]

    def deg(b):
        d = b * bin_deg
        return d - 360.0 if d >= 180.0 else d
    return [(deg(r[0]), deg(r[-1]) + bin_deg, max(mask[b] for b in r)) for r in out]


class RecordSelfMask(Node):

    def __init__(self):
        super().__init__('record_self_mask')
        self.declare_parameter('scans', 30)
        self.declare_parameter('max_range', 0.40)      # m, anything closer could be the rover
        self.declare_parameter('bin_deg', 1.0)
        self.declare_parameter('out', '~/rover/self_mask.json')
        self.want = int(self.get_parameter('scans').value)
        self.scans = []
        self.done = False
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info(
            f'Recording {self.want} scans. Keep everything except the rover itself '
            f'more than {self.get_parameter("max_range").value + 0.1:.1f} m away...')

    def on_scan(self, msg: LaserScan):
        if self.done:
            return
        self.scans.append([(msg.angle_min + i * msg.angle_increment, r)
                           for i, r in enumerate(msg.ranges)])
        if len(self.scans) >= self.want:
            self.done = True
            self.save()

    def save(self):
        bin_deg = float(self.get_parameter('bin_deg').value)
        mask = build_mask(self.scans, float(self.get_parameter('max_range').value), bin_deg)
        path = os.path.expanduser(str(self.get_parameter('out').value))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as f:
            json.dump({'bin_deg': bin_deg, 'bins': {str(b): r for b, r in mask.items()}},
                      f, indent=1)
        if not mask:
            self.get_logger().info('No part of the rover seen within range. '
                                   f'Saved an empty mask to {path}')
        else:
            self.get_logger().info(f'Rover body seen in {len(mask)} degrees. Saved {path}')
            for a0, a1, r in sectors(mask, bin_deg):
                self.get_logger().info(f'   {a0:+6.0f} .. {a1:+6.0f} deg, up to {r:.2f} m')
        self.get_logger().info('Restart the demo so hallway_follower loads it.')


def main(args=None):
    rclpy.init(args=args)
    node = RecordSelfMask()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
