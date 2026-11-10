"""Hallway following + obstacle avoidance, as plain Python (no ROS).

Kept separate from the ROS node so it can be unit-tested with simulated scans
(see test/test_hallway_logic.py) and tuned without the robot.

Coordinate frame (ROS convention, = the LiDAR frame after our `inverted: true` fix):
    x = forward (arrow on top of the C1), y = left, angles CCW-positive.

How it works ("follow the gap", lateral version):
  1. Find the walls: fit the longest straight line on each side (RANSAC), which
     gives the hallway's angle relative to the robot and the distance to each
     wall. Everything after this is done in a "hallway frame" lined up with the
     walls, so an angled robot doesn't mistake the walls ahead for obstacles.
     A wall missing (doorway / open area) is mirrored from the other side so the
     robot doesn't veer into doorways.
  2. Look at points ahead (and just beside) that are between the walls. Those
     are obstacles. Their y positions split [-R, L] into lateral gaps.
  3. Keep the gaps wide enough for the robot, pick the one needing the least
     swerve (with a bit of "stick with the side I already chose").
  4. Steer toward that gap's centre. With no obstacles the only gap is the whole
     hallway, so this IS hallway centring.
  5. Slow down as something gets close in the robot's path; stop if it's
     within stop_dist and turn in place toward the gap; stop completely (BLOCKED)
     if there's no gap at all (e.g. end of the hallway).
"""
import math
import random
from dataclasses import dataclass, field


@dataclass
class Params:
    cruise_speed: float = 0.15      # m/s, normal forward speed
    slow_speed: float = 0.06        # m/s, speed just before stop_dist
    max_turn: float = 0.6           # rad/s, angular speed limit
    kp_heading: float = 1.0         # (rad/s) per rad of heading error to the target
    pivot_turn: float = 0.5         # rad/s, turn-in-place speed when stopped by an obstacle
    robot_half_width: float = 0.13  # m, half the rover's width (10 in rover)
    side_margin: float = 0.08       # m, extra clearance each side
    lookahead: float = 1.0          # m, how far ahead obstacles are considered
    steer_lookahead: float = 0.8    # m, aim point distance (bigger = gentler steering)
    slow_dist: float = 0.8          # m, start slowing for things in the path
    stop_dist: float = 0.30         # m, stop forward motion inside this
    max_wall_dist: float = 3.0      # m, side readings beyond this = "no wall" (big enough for a wide hallway)
    wall_fit_x: float = 2.0         # m, |x| window of points used to find the walls
    wall_tol: float = 0.04          # m, how far a point can be from a wall line and count
    min_wall_span: float = 0.6      # m, a "wall" must be at least this long
    ransac_iters: int = 60          # line-fit attempts per side per scan
    behind_x: float = 0.20          # m, things this far behind the LiDAR still count as "beside"
    obstacle_band: float = 0.5      # m, obstacles within this of the nearest one are grouped
    side_stop: float = 0.05         # m, won't turn toward a side closer than robot_half_width + this
    stick_bonus: float = 0.15       # m, preference for keeping the previously chosen side
    width_bonus: float = 0.6        # preference for wider gaps (score -= this * gap width)
    min_range: float = 0.12         # m, ignore returns closer than this (C1 unreliable <0.1 m, and it's our own rover)
    wall_margin: float = 0.10       # m, points within this of a wall line count as wall, not obstacle
    clear_scans_reset: int = 5      # scans with nothing between the walls before forgetting the chosen side
    hit_margin: float = 0.05        # m, clearance beyond robot_half_width that counts as "would hit"
    max_hall_yaw_deg: float = 40.0  # deg, never angle more than this across the hallway
    laser_x: float = 0.0             # m, LiDAR position relative to the rover's centre: + = forward
    laser_y: float = 0.0             # m, LiDAR position relative to the rover's centre: + = left
    fov_deg: float = 180.0          # deg, use points within +-fov/2 of ahead (rear blocked by mount)
    # --- smoothing (stops the 'drunk driver' wiggle) ---
    turn_smoothing: float = 0.35    # 0..1, how fast steering follows a new decision (1 = instantly)
    heading_deadband_deg: float = 2.0  # deg, ignore heading errors smaller than this
    accel: float = 0.3              # m/s^2, max speed-up rate (slowing down is instant)
    scan_dt: float = 0.1            # s, time between scans (C1 = 10 Hz)
    stop_release: float = 0.10      # m, once stopped, path must be this much past stop_dist to go again
    min_obstacle_pts: int = 3       # an obstacle must be at least this many points (drops single-point noise)
    coast_scans: int = 3            # sparse scans in a row to coast through before stopping
    # --- the rover's own body, in LiDAR coordinates (ignored) ---
    body_front: float = 0.07        # m, how far the rover sticks out IN FRONT of the LiDAR
    body_half_width: float = 0.22   # m, half the rover's full width incl. wheels (+ a little)
    # --- mode ---
    avoid_obstacles: bool = False   # False = simple hallway mode: stay centred, go, stop for anything ahead
    # --- remembering which way the hallway goes ---
    hall_gate_deg: float = 15.0     # deg, a new wall reading this far from the expected direction is doubted
    hall_gain: float = 0.5          # 0..1, how much each new wall reading corrects the remembered direction
    hall_relock_scans: int = 4      # doubted readings in a row before believing them anyway


@dataclass
class Decision:
    linear: float
    angular: float
    state: str                      # FOLLOW / AVOID / PIVOT / BLOCKED / NO_DATA
    info: dict = field(default_factory=dict)


def scan_to_xy(angles, ranges, min_range, max_range=12.0):
    """Polar (rad, m) -> list of (x, y), dropping invalid returns."""
    pts = []
    for a, r in zip(angles, ranges):
        if r is None or not math.isfinite(r) or r < min_range or r > max_range:
            continue
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts


def drop_speckles(pts, min_pts, dist=0.06):
    """Drop lone returns (dust, cable flicker, edge "mixed pixels").

    pts are in scan order, so a real surface has neighbours right next to it in
    the list. A point is kept if at least min_pts-1 of the 2 points either side
    of it are within `dist`. min_pts <= 1 turns this off.
    """
    if min_pts <= 1 or len(pts) < 5:
        return pts
    need, d2, n, out = min_pts - 1, dist * dist, len(pts), []
    for i, (x, y) in enumerate(pts):
        k = 0
        for j in (i - 2, i - 1, i + 1, i + 2):
            if 0 <= j < n:
                dx, dy = pts[j][0] - x, pts[j][1] - y
                if dx * dx + dy * dy < d2:
                    k += 1
        if k >= need:
            out.append((x, y))
    return out


class HallwayLogic:

    def __init__(self, params: Params = None):
        self.p = params or Params()
        self.last_side = 0  # +1 left, -1 right, 0 none
        self.clear_scans = 0
        self.prev_w = 0.0      # last angular command (for smoothing)
        self.prev_v = 0.0      # last linear command (for the accel limit)
        self.stopped = False   # in PIVOT/BLOCKED last scan (for stop hysteresis)
        self.reset_tracking()
        self._rng = random.Random(0)  # deterministic line fitting

    def reset_tracking(self):
        """Forget the remembered hallway direction / width (e.g. after manual driving)."""
        self.phi_est = None    # remembered hallway angle relative to the robot (rad)
        self.doubt = 0         # wall readings in a row that disagreed with phi_est
        self.width = None      # remembered hallway width (m)
        self.last_LR = None    # last (L, R) we trusted

    def track_hall(self, phi_m, L, R, mode):
        """Combine this scan's wall reading with what we remember.

        Between scans the robot turned by about prev_w * dt, so the hallway should
        now appear rotated the other way by that much. A reading that agrees with
        that prediction refines it; one that doesn't (a doorway, a person, a wall
        fit on the wrong thing) is ignored unless it keeps happening. With no walls
        in view at all we keep driving on the remembered direction instead of
        assuming "straight ahead is down the hallway".
        Returns (phi, L, R, mode).
        """
        p = self.p
        pred = None if self.phi_est is None else self.phi_est - self.prev_w * p.scan_dt
        if pred is not None and self.last_LR is not None:
            # we also drifted sideways while angled: yaw vs hall = -pred
            drift = self.prev_v * p.scan_dt * math.sin(-pred)
            self.last_LR = (self.last_LR[0] - drift, self.last_LR[1] + drift)
        if mode == 'none':
            if pred is None:
                return 0.0, L, R, 'none'
            self.phi_est = pred
            if self.last_LR:
                return pred, self.last_LR[0], self.last_LR[1], 'remembered'
            return pred, L, R, 'remembered'
        if pred is not None and abs(phi_m - pred) > math.radians(p.hall_gate_deg):
            self.doubt += 1
            if self.doubt < p.hall_relock_scans:
                self.phi_est = pred
                if self.last_LR:
                    return pred, self.last_LR[0], self.last_LR[1], 'doubted'
                return pred, L, R, 'doubted'
            pred = None  # it keeps saying so: believe it
        self.doubt = 0
        self.phi_est = phi_m if pred is None else pred + p.hall_gain * (phi_m - pred)
        # one wall only: use the remembered width instead of mirroring it
        if mode == 'both':
            w = L + R
            self.width = w if self.width is None else 0.8 * self.width + 0.2 * w
        elif self.width:
            if mode == 'left_only':
                R = max(0.0, self.width - L)
            elif mode == 'right_only':
                L = max(0.0, self.width - R)
        self.last_LR = (L, R)
        return self.phi_est, L, R, mode

    # -- steps 0-1: find the walls (robust line fit) -------------------------
    def fit_wall(self, side_pts):
        """RANSAC line fit y = m*x + b on one side's points.

        Scores each candidate line by how LONG a stretch it covers (number of
        5 cm bins along x that contain an inlier), not by raw point count. Close
        objects produce many densely packed points, so counting points would let a
        30 cm box face near the robot outvote a long wall further away.
        Returns (m, b, inliers, coverage) or None if no convincing wall.
        """
        p = self.p
        n = len(side_pts)
        if n < 8:
            return None
        rng = self._rng
        max_m = math.tan(math.radians(50))
        best, best_cov = None, 0
        for _ in range(p.ransac_iters):
            (x1, y1), (x2, y2) = side_pts[rng.randrange(n)], side_pts[rng.randrange(n)]
            if abs(x2 - x1) < 0.2:
                continue
            m = (y2 - y1) / (x2 - x1)
            if abs(m) > max_m:
                continue
            b = y1 - m * x1
            k = math.sqrt(1 + m * m)
            inl = [(x, y) for x, y in side_pts if abs(y - (m * x + b)) / k < p.wall_tol]
            cov = len({int(math.floor(x / 0.05)) for x, _ in inl})
            if cov > best_cov:
                best, best_cov = inl, cov
        if not best or len(best) < 8:
            return None
        xs = [x for x, _ in best]
        if max(xs) - min(xs) < p.min_wall_span:
            return None
        # least-squares refit on the inliers
        mx = sum(xs) / len(best)
        my = sum(y for _, y in best) / len(best)
        sxx = sum((x - mx) ** 2 for x in xs)
        m = sum((x - mx) * (y - my) for x, y in best) / sxx
        return m, my - m * mx, best, best_cov

    def find_walls(self, pts):
        """Returns (phi, L, R, mode).

        phi = angle of the hallway relative to the robot (rad, + = walls run to
        the robot's left), L/R = perpendicular distance to left/right wall.
        """
        p = self.p
        w = p.wall_fit_x
        left = [(x, y) for x, y in pts if abs(x) <= w and 0 < y <= p.max_wall_dist]
        right = [(x, y) for x, y in pts if abs(x) <= w and -p.max_wall_dist <= y < 0]
        fl, fr = self.fit_wall(left), self.fit_wall(right)

        fits = [f for f in (fl, fr) if f]
        if not fits:
            return 0.0, p.max_wall_dist, p.max_wall_dist, 'none'
        tot = sum(f[3] for f in fits)
        phi = sum(math.atan(f[0]) * f[3] for f in fits) / tot

        def dist(f, sign):
            m, b = f[0], f[1]
            # perpendicular distance from the robot (origin) to the line
            return min(p.max_wall_dist, abs(b) / math.sqrt(1 + m * m)) if sign * b > 0 else None

        L = dist(fl, +1) if fl else None
        R = dist(fr, -1) if fr else None
        if L is None and R is None:
            return phi, p.max_wall_dist, p.max_wall_dist, 'none'
        if L is None:
            return phi, R, R, 'right_only'
        if R is None:
            return phi, L, L, 'left_only'
        return phi, L, R, 'both'

    # -- steps 2-3 ------------------------------------------------------------
    def gaps(self, pts, L, R):
        """Lateral gaps between the walls, considering only the nearest "band" of
        obstacles ahead. Staggered obstacles further down the hall are dealt with
        when we get to them, instead of being flattened together with the near ones.
        """
        p = self.p
        m = p.wall_margin  # points this close to a wall are the wall (or noise on it)
        ahead = [(x, y) for x, y in pts
                 if 0.0 < x < p.lookahead and -R + m < y < L - m]
        if ahead:
            nearest = min(x for x, _ in ahead)
            ahead = [(x, y) for x, y in ahead if x < nearest + p.obstacle_band]
        inner = sorted(y for _, y in ahead)
        edges = [-R] + inner + [L]
        need = 2 * (p.robot_half_width + p.side_margin)
        out = []
        for lo, hi in zip(edges, edges[1:]):
            if hi - lo >= need:
                out.append((lo, hi))
        return out, len(inner)

    def side_clearance(self, pts):
        """Closest lateral distance to anything right beside the robot (left, right)."""
        p = self.p
        near = [(x, y) for x, y in pts if -p.behind_x <= x <= p.robot_half_width]
        left = min((y for x, y in near if y > 0), default=math.inf)
        right = min((-y for x, y in near if y < 0), default=math.inf)
        return left, right

    def choose_gap(self, gaps):
        # Once we've committed to passing an obstacle on one side, stay committed
        # while that side still has a usable gap (prevents last-moment side flips).
        if self.last_side:
            same = [g for g in gaps if (0.5 * (g[0] + g[1])) * self.last_side > 0.05]
            if same:
                gaps = same
        best, best_score = None, None
        for lo, hi in gaps:
            c = 0.5 * (lo + hi)
            # least swerve, but strongly prefer roomier gaps over barely-fits ones
            score = abs(c) - self.p.width_bonus * (hi - lo)
            if self.last_side and math.copysign(1, c) == self.last_side and abs(c) > 0.05:
                score -= self.p.stick_bonus
            if best_score is None or score < best_score:
                best, best_score = (lo, hi), score
        return best

    # -- steps 4-5 ------------------------------------------------------------
    def path_distance(self, pts, half):
        """Nearest x of anything ahead within +-half of the robot's centreline."""
        ahead = [x for x, y in pts if x > 0.0 and abs(y) <= half]
        return min(ahead) if ahead else math.inf

    def in_fov(self, pts):
        """Keep only points inside the usable field of view (front half by default)."""
        if self.p.fov_deg >= 360.0:
            return pts
        half = math.radians(self.p.fov_deg / 2.0) + 1e-6
        return [(x, y) for x, y in pts if abs(math.atan2(y, x)) <= half]

    def decide(self, pts) -> Decision:
        """Raw decision + smoothing, so one odd scan can't jerk the wheels."""
        p = self.p
        d = self._decide_raw(pts)
        if d.state == 'NO_DATA':
            # one sparse scan (open area, glare): coast briefly instead of a jerky stop
            self.nodata = getattr(self, 'nodata', 0) + 1
            if self.nodata <= p.coast_scans and not self.stopped:
                self.prev_w *= 0.5
                return Decision(self.prev_v, self.prev_w, 'COAST', d.info)
        else:
            self.nodata = 0
        self.stopped = d.state in ('PIVOT', 'BLOCKED')
        a = max(0.0, min(1.0, p.turn_smoothing))
        if d.state == 'NO_DATA' or (d.state == 'BLOCKED'):
            w = 0.0
        elif d.state == 'PIVOT':
            w = d.angular  # pivots are deliberate, full rate
        else:
            w = self.prev_w + a * (d.angular - self.prev_w)
        v = d.linear if d.linear <= self.prev_v else min(d.linear, self.prev_v + p.accel * p.scan_dt)
        self.prev_w, self.prev_v = w, v
        return Decision(v, w, d.state, d.info)

    def _decide_raw(self, pts) -> Decision:
        p = self.p
        pts = self.in_fov(pts)  # field of view is about the LiDAR itself
        # drop the rover's own body/wheels: a box beside and behind the LiDAR
        if p.body_half_width > 0.0:
            pts = [(x, y) for x, y in pts
                   if not (x <= p.body_front and abs(y) <= p.body_half_width)]
        # From here on, work relative to the rover's CENTRE (not the LiDAR), so
        # centring, clearances and the path corridor are about the rover's body.
        if p.laser_x or p.laser_y:
            pts = [(x + p.laser_x, y + p.laser_y) for x, y in pts]
        pts = drop_speckles(pts, p.min_obstacle_pts)
        if len(pts) < 20:
            return Decision(0.0, 0.0, 'NO_DATA', {'points': len(pts)})

        # Work out wall/gap geometry in a "hallway frame" lined up with the walls,
        # so an angled robot doesn't see the walls ahead as obstacles.
        phi, L, R, wall_mode = self.track_hall(*self.find_walls(pts))
        c, s = math.cos(phi), math.sin(phi)
        hpts = [(x * c + y * s, -x * s + y * c) for x, y in pts]

        gaps, n_inner = self.gaps(hpts, L, R)
        # robot frame: what's in our actual path. "soft" includes the comfort margin
        # (slow down for it); "hit" is what the body would actually touch (stop for it).
        front = self.path_distance(pts, p.robot_half_width + p.side_margin)
        front_hit = self.path_distance(pts, p.robot_half_width + p.hit_margin)
        hit_pts = [(x, y) for x, y in pts if x > 0.0 and abs(y) <= p.robot_half_width + p.side_margin]
        if hit_pts:  # where the nearest thing in our path is (for the log)
            hx, hy = min(hit_pts)
            front_at = (math.degrees(math.atan2(hy - p.laser_y, hx - p.laser_x)),
                        math.hypot(hx - p.laser_x, hy - p.laser_y))
        else:
            front_at = None
        info = {'L': L, 'R': R, 'walls': wall_mode, 'front': front, 'gaps': len(gaps),
                'front_at': front_at,
                'hall_deg': math.degrees(phi)}

        if not p.avoid_obstacles:
            # simple hallway mode: aim for the middle between the walls
            target_y = 0.5 * (L - R) if wall_mode != 'none' else 0.0
        elif not gaps:
            return Decision(0.0, 0.0, 'BLOCKED', info)
        else:
            lo, hi = self.choose_gap(gaps)
            target_y = 0.5 * (lo + hi)
        info['target_y'] = target_y

        # aim point in the hallway frame -> robot frame
        ax, ay = p.steer_lookahead, target_y
        rx, ry = ax * c - ay * s, ax * s + ay * c
        heading_err = math.atan2(ry, rx)
        # Keep the robot within +-max_hall_yaw of the hallway direction: robot yaw
        # relative to the hall is -phi; after turning by heading_err it'd be -phi + err.
        lim = math.radians(p.max_hall_yaw_deg)
        heading_err = max(-lim, min(lim, heading_err - phi)) + phi
        db = math.radians(p.heading_deadband_deg)
        heading_err = 0.0 if abs(heading_err) < db else heading_err - math.copysign(db, heading_err)
        angular = max(-p.max_turn, min(p.max_turn, p.kp_heading * heading_err))

        # don't swing into something that's right beside us
        cl_left, cl_right = self.side_clearance(pts)
        info['side_clear'] = (cl_left, cl_right)
        if angular > 0 and cl_left < p.robot_half_width + p.side_stop:
            angular = 0.0
        elif angular < 0 and cl_right < p.robot_half_width + p.side_stop:
            angular = 0.0

        in_path = front < p.lookahead
        # Remember which side we're passing on until the hallway ahead has been
        # clear for a few scans (not just until the obstacle leaves our straight-
        # ahead corridor, which happens every time we angle away from it).
        if n_inner:
            self.clear_scans = 0
            if self.last_side == 0 and abs(target_y) > 0.05:
                self.last_side = 1 if target_y > 0 else -1
        else:
            self.clear_scans += 1
            if self.clear_scans >= p.clear_scans_reset:
                self.last_side = 0

        # hysteresis: once stopped, don't creep forward until clearly clear
        stop_at = p.stop_dist + (p.stop_release if self.stopped else 0.0)
        if front_hit <= stop_at and not p.avoid_obstacles:
            return Decision(0.0, 0.0, 'BLOCKED', info)  # end of the hallway / something ahead: stop
        if front_hit <= stop_at:
            # something the body would hit: stop and turn in place toward the gap,
            # unless that would angle us past max_hall_yaw (then just wait: BLOCKED)
            yaw_rel_hall = -phi
            turn = math.copysign(p.pivot_turn, target_y)
            if abs(target_y) < 0.02 or (math.copysign(1, turn) * yaw_rel_hall >= lim):
                return Decision(0.0, 0.0, 'BLOCKED', info)
            return Decision(0.0, turn, 'PIVOT', info)
        front = max(front, p.stop_dist)  # only in the comfort margin: creep, don't stop

        if front >= p.slow_dist:
            linear = p.cruise_speed
        else:
            t = (front - p.stop_dist) / (p.slow_dist - p.stop_dist)
            linear = p.slow_speed + t * (p.cruise_speed - p.slow_speed)
        # ease off when turning hard
        linear *= max(0.4, 1.0 - 0.5 * abs(angular) / p.max_turn)

        state = 'AVOID' if p.avoid_obstacles and n_inner and in_path else 'FOLLOW'
        return Decision(linear, angular, state, info)
