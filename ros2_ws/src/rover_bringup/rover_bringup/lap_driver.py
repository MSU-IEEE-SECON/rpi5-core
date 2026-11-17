"""lap_driver — laps of the SECON arena: LiDAR localization + Bug2 obstacle avoidance.

Subscribes:  /scan, /rover/enable (Y on the controller)
Publishes:   /cmd_vel (linear.x fwd, linear.y left/strafe, angular.z), /lap/state

Place the rover in the RIGHT lane (rail on its left, outer wall on its right),
roughly facing up the lane, anywhere from the bottom crossing to mid-lane.
Press Y: it finds itself on the arena from the walls (~0.2 s), then drives
counter-clockwise laps around the center rail, steering around cars (Bug2),
never entering the pit, and stops after `laps` laps. B / a stick = manual.

It stops (LOST) if the walls stop matching the map for ~1 s.
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String

from rover_bringup import arena as A
from rover_bringup import lidar_calib
from rover_bringup.bug2 import Bug2Lap, LapParams
from rover_bringup.path_lap import PathLap, PathParams, SpeedGain

ENABLE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


class LapDriver(Node):

    def __init__(self):
        super().__init__('lap_driver')
        for name, val in vars(LapParams()).items():
            self.declare_parameter(name, val)
        self.declare_parameter('min_range', 0.12)
        self.declare_parameter('laser_yaw_deg', 180.0)     # overridden by lidar_calib.json
        self.declare_parameter('laser_mirror', False)
        self.declare_parameter('self_mask_file', os.path.join(
            os.environ.get('ROVER_DATA', '~/rover_data'), 'self_mask.json'))
        # where to look for the rover when Y is pressed (board metres / deg)
        # (start box in the pit, or anywhere in the bottom half of the right lane)
        self.declare_parameter('search_x', [A.LANE_R - 0.12, A.X1 - 0.10])
        self.declare_parameter('search_y', [0.30, A.RAIL_Y0 + 0.5])
        self.declare_parameter('search_th_deg', [60.0, 120.0])
        self.declare_parameter('log_period_s', 0.5)
        # 'path' = racing line shifted around cars (smooth, default); 'bug2' = the old stop-turn-go
        self.declare_parameter('planner', 'path')
        for name, val in vars(PathParams()).items():
            self.declare_parameter('path_' + name, val)
        # learn how fast the rover really moves vs what it's told (motors are not calibrated and
        # not linear), and scale the commands so it goes the speed the planner asked for
        self.declare_parameter('adapt_speed', True)
        # add cars the webcam sees (camera_node) to the obstacle map
        self.declare_parameter('use_camera', True)

        p = LapParams()
        for name in vars(p):
            setattr(p, name, type(getattr(p, name))(self.get_parameter(name).value))
        self.yaw = math.radians(float(self.get_parameter('laser_yaw_deg').value))
        self.mirror = bool(self.get_parameter('laser_mirror').value)
        c = lidar_calib.load()
        if c:
            self.yaw = math.radians(float(c['laser_yaw_deg']))
            self.mirror = bool(c['laser_mirror'])
            p.view_deg = min(float(c['fov_left_deg']), float(c['fov_right_deg']))
            self.get_logger().info(f"LiDAR calibration: mirrored {self.mirror}, view +-{p.view_deg:.0f} deg")
        else:
            self.get_logger().warn('No LiDAR calibration: using lap_driver defaults')
        self.p = p
        self.mask_bin, self.mask = self._load_mask()

        # the rail shows up when the scan plane is low enough; unmatched it costs nothing
        pp = PathParams()
        for name in vars(pp):
            setattr(pp, name, type(getattr(pp, name))(self.get_parameter('path_' + name).value))
        pp.lidar_fwd = p.lidar_fwd
        self.planner_name = str(self.get_parameter('planner').value)
        self.loc = A.Localizer(A.WALLS + A.RAIL)
        self.gain = SpeedGain()
        self.prev_pose = None
        self.relocs = 0
        self.planner = PathLap(pp) if self.planner_name == 'path' else Bug2Lap(p)
        self.get_logger().info(f'planner: {self.planner_name}')
        self.enabled = False
        self.ready = False
        self.cmd = (0.0, 0.0, 0.0)
        self.last_t = None
        self.last_log = 0.0
        self.state = 'WAIT'
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'lap/state', 10)
        self.info_pub = self.create_publisher(String, 'lap/info', 10)   # JSON, for the dashboard
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Bool, 'rover/enable', self.on_enable, ENABLE_QOS)
        # wheel-encoder odometry from the team's micro-ROS ESP32: [x, y, yaw, vx, vy, wz]
        # (body frame speeds). When fresh, the localizer predicts motion from what the wheels
        # really did instead of what was commanded.
        self.odom = None
        self.odom_t = 0.0
        self.create_subscription(Float32MultiArray, 'odom', self.on_odom, qos_profile_sensor_data)
        self.cam_pts, self.cam_t, self.cam_used = None, 0.0, 0
        self.create_subscription(Float32MultiArray, 'camera/obstacles', self.on_camera,
                                 qos_profile_sensor_data)
        self.get_logger().info('lap_driver ready. Rover in the RIGHT lane facing up it, press Y.')

    # ---- inputs -------------------------------------------------------------
    def _load_mask(self):
        path = os.path.expanduser(str(self.get_parameter('self_mask_file').value))
        try:
            with open(path) as f:
                d = json.load(f)
            return float(d['bin_deg']), {int(b): float(r) for b, r in d['bins'].items()}
        except Exception:  # noqa: BLE001
            return 1.0, {}

    def _local_points(self, msg):
        """Scan -> points in the LiDAR's rover-aligned frame (x forward, y left)."""
        n = len(msg.ranges)
        raw = msg.angle_min + np.arange(n) * msg.angle_increment
        r = np.asarray(msg.ranges, float)
        ok = np.isfinite(r) & (r >= float(self.get_parameter('min_range').value)) & (r < 8.0)
        if self.mask:
            nb = int(round(360.0 / self.mask_bin))
            b = (np.floor(np.degrees(raw) % 360.0 / self.mask_bin).astype(int)) % nb
            lim = np.array([max(self.mask.get((k + d) % nb, -1.0) for d in (-1, 0, 1)) for k in b])
            ok &= ~((lim > 0) & (r <= lim + 0.03))
        a = self.yaw + (-raw if self.mirror else raw)
        a = np.arctan2(np.sin(a), np.cos(a))
        ok &= np.abs(a) <= math.radians(self.p.view_deg)
        return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1)

    def on_odom(self, msg):
        if len(msg.data) >= 6:
            self.odom = (float(msg.data[3]), float(msg.data[4]), float(msg.data[5]))
            self.odom_t = time.monotonic()

    def on_camera(self, msg):
        self.cam_pts = np.asarray(msg.data, float).reshape(-1, 2)
        self.cam_t = time.monotonic()

    def _feed_camera(self, centre, now):
        """Newest webcam obstacle points (rover frame) -> board frame -> planner map, once each."""
        if (self.cam_pts is None or now - self.cam_t > 0.25 or self.planner_name != 'path'
                or not bool(self.get_parameter('use_camera').value)):
            return
        pts, self.cam_pts = self.cam_pts, None
        x, y, th = centre
        c, s = math.cos(th), math.sin(th)
        world = pts @ np.array([[c, s], [-s, c]]) + [x, y]
        cam_xy = (x + 0.05 * c, y + 0.05 * s)
        self.planner.add_camera(world, cam_xy, centre)
        self.cam_used += len(pts)

    def on_enable(self, msg):
        if msg.data and not self.enabled:
            self._dumped = False
            self.enabled, self.ready = True, False      # re-localize on the next scan
            self.get_logger().info('ENABLED: finding the rover on the arena...')
        elif not msg.data and self.enabled:
            self.enabled = False
            self.stop()
            self._dump_obstacles('disabled')
            self.get_logger().info(f'DISABLED (laps done: {self.planner.laps_done})')

    # ---- main loop ----------------------------------------------------------
    def on_scan(self, msg):
        now = time.monotonic()
        dt = 0.1 if self.last_t is None else min(0.3, now - self.last_t)
        self.last_t = now
        pts = self._local_points(msg)
        f = self.p.lidar_fwd
        if not self.enabled:
            self._publish_state('DISABLED')
            return
        if not self.ready:
            sx = self.get_parameter('search_x').value
            sy = self.get_parameter('search_y').value
            st = self.get_parameter('search_th_deg').value
            pose, q = self.loc.global_search(pts, sx, sy, st)
            if pose is None or q < 0.5:
                self.stop()
                self._log('LOCATING', f'no good match yet (quality {q:.2f}); is it in the right lane facing up?')
                return
            self.loc.pose = pose
            centre = (pose[0] - f * math.cos(pose[2]), pose[1] - f * math.sin(pose[2]), pose[2])
            self.planner.reset(centre)
            self.ready = True
            self.get_logger().info(f'Found: x={centre[0]:.2f} y={centre[1]:.2f} '
                                   f'heading={math.degrees(centre[2]):.0f} deg (quality {q:.2f}). Driving.')
        vx, vy, wz = self.cmd            # what the planner asked for (before scaling)
        mx, my, mz = self.odom if (self.odom and now - self.odom_t < 0.3) else (vx, vy, wz)
        before = self.loc.pose
        self.loc.update(pts, (mx * dt, (my + mz * f) * dt, mz * dt))
        if self.loc.bad_scans == 0 and self.loc.quality >= 0.6:
            self.gain.update(before, self.loc.pose, (vx, vy, wz), dt, f)
        if self.loc.bad_scans > 4:
            # lost track (fast turn, clutter): search around where it should be before giving up
            self.stop()
            px, py, pth = self.loc.pose
            pose, q = self.loc.global_search(pts, (px - 0.25, px + 0.25), (py - 0.25, py + 0.25),
                                             (math.degrees(pth) - 40, math.degrees(pth) + 40),
                                             step=0.04, th_step=5.0)
            if pose is not None and q >= 0.6 and A.X0 < pose[0] < A.X1 and A.T < pose[1] < A.Y_TOP:
                self.loc.pose, self.loc.quality, self.loc.bad_scans = pose, q, 0
                self.relocs += 1
                self.get_logger().info(f'RELOCATED (#{self.relocs}) x={pose[0]:.2f} y={pose[1]:.2f} '
                                       f'hdg={math.degrees(pose[2]):+.0f} q={q:.2f}')
            elif self.loc.bad_scans > 15:
                self._log('LOST', f'walls stopped matching the map (quality {self.loc.quality:.2f}): stopped')
                return
            else:
                self._log('LOCATING', f'match {q:.2f}, searching...')
                return
        ex, ey, eth = self.loc.pose
        c, s = math.cos(eth), math.sin(eth)
        world = pts @ np.array([[c, s], [-s, c]]) + [ex, ey]
        centre = (ex - f * c, ey - f * s, eth)
        if self.planner_name == 'path':
            self._feed_camera(centre, now)
            vx, vy, wz, state, info = self.planner.step(centre, world, dt)
        else:
            vx, vy, wz, state, info = self.planner.step(centre, world)
        self.cmd = (vx, vy, wz)
        # send scaled commands so the rover really moves at the planned speed
        sx, sy, sz = self.gain.scale((vx, vy, wz)) if bool(self.get_parameter('adapt_speed').value) \
            else (vx, vy, wz)
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(sx), float(sy), float(sz)
        self.cmd_pub.publish(t)
        self.info_pub.publish(String(data=json.dumps({
            'obs': self._obs_cells(), 'offset': info.get('offset'), 'scale': [round(self.gain.t, 2), round(self.gain.r, 2)],
            'state': state, 'x': round(centre[0], 3), 'y': round(centre[1], 3), 'th': round(eth, 3),
            'lidar': [round(ex, 3), round(ey, 3)], 'leg': info.get('leg'), 'laps': info.get('laps'),
            'q': round(self.loc.quality, 2), 'cam': self.cam_used, 'v': [round(vx, 2), round(vy, 2), round(wz, 2)]})))
        self._log(state, f"x={centre[0]:.2f} y={centre[1]:.2f} hdg={math.degrees(eth):+4.0f} "
                         f"leg={info.get('leg')} laps={info.get('laps')} q={self.loc.quality:.2f} "
                         f"off={info.get('offset')} gain={self.gain.t:.2f}/{self.gain.r:.2f} "
                         f"v=({vx:+.2f},{vy:+.2f},{wz:+.2f})")
        if state == 'DONE':
            self.stop()
            if not getattr(self, '_dumped', False):
                self._dump_obstacles('done')
                self._dumped = True

    def _dump_obstacles(self, why):
        """Save what the planner thought was in the way (for analysis after a run)."""
        try:
            o = self.planner.obstacles() if hasattr(self.planner, 'obstacles') else []
            cells = sorted({(round(float(a) / A.IN), round(float(b) / A.IN)) for a, b in o})
            path = os.path.join(os.path.expanduser(os.environ.get('ROVER_DATA', '~/rover_data')),
                                'last_obstacles.json')
            with open(path, 'w') as f:
                json.dump({'why': why, 'time': time.time(), 'cells_in': cells}, f)
            self.get_logger().info(f'obstacle map ({why}): {len(cells)} 1-in cells: ' +
                                   ' '.join(f'{x},{y}' for x, y in cells[:200]))
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f'could not save obstacle map: {e}')

    def _obs_cells(self):
        """Obstacle map for the dashboard (2 cm cells, at most 400), refreshed once a second."""
        t = time.monotonic()
        if t - getattr(self, '_obs_t', 0.0) < 1.0:
            return getattr(self, '_obs_cache', [])
        self._obs_t = t
        cells = []
        if hasattr(self.planner, 'obstacles'):
            o = self.planner.obstacles()
            if len(o):
                q = np.unique(np.round(o / 0.02).astype(int), axis=0)[:400] * 0.02
                cells = [[round(float(a), 2), round(float(b), 2)] for a, b in q]
        self._obs_cache = cells
        return cells

    # ---- helpers ------------------------------------------------------------
    def stop(self):
        self.cmd = (0.0, 0.0, 0.0)
        self.cmd_pub.publish(Twist())

    def _publish_state(self, state):
        self.state_pub.publish(String(data=state))

    def _log(self, state, text):
        self._publish_state(state)
        t = time.monotonic()
        if state != self.state or t - self.last_log >= float(self.get_parameter('log_period_s').value):
            self.get_logger().info(f'{state:8s} {text}')
            self.last_log, self.state = t, state

    def destroy_node(self):
        try:
            self.stop()
        except Exception:  # noqa: BLE001
            pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LapDriver()
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
