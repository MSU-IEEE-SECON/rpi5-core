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
    max_turn: float = 0.8           # rad/s, angular speed limit
    kp_heading: float = 1.5         # (rad/s) per rad of heading error to the target
    pivot_turn: float = 0.5         # rad/s, turn-in-place speed when stopped by an obstacle
    robot_half_width: float = 0.15  # m, half the rover's width
    side_margin: float = 0.08       # m, extra clearance each side
    lookahead: float = 1.2          # m, how far ahead obstacles are considered
    steer_lookahead: float = 0.8    # m, aim point distance (bigger = gentler steering)
    slow_dist: float = 1.0          # m, start slowing for things in the path
    stop_dist: float = 0.30         # m, stop forward motion inside this
    max_wall_dist: float = 1.5      # m, side readings beyond this = "no wall"
    wall_fit_x: float = 1.5         # m, |x| window of points used to find the walls
    wall_tol: float = 0.04          # m, how far a point can be from a wall line and count
    min_wall_span: float = 0.6      # m, a "wall" must be at least this long
    ransac_iters: int = 60          # line-fit attempts per side per scan
    behind_x: float = 0.20          # m, things this far behind the LiDAR still count as "beside"
    obstacle_band: float = 0.5      # m, obstacles within this of the nearest one are grouped
    side_stop: float = 0.05         # m, won't turn toward a side closer than robot_half_width + this
    stick_bonus: float = 0.15       # m, preference for keeping the previously chosen side
    width_bonus: float = 0.6        # preference for wider gaps (score -= this * gap width)
    min_range: float = 0.06         # m, ignore returns closer than this (C1 min ~0.05)


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


class HallwayLogic:

    def __init__(self, params: Params = None):
        self.p = params or Params()
        self.last_side = 0  # +1 left, -1 right, 0 none
        self._rng = random.Random(0)  # deterministic line fitting

    # -- steps 0-1: find the walls (robust line fit) -------------------------
    def fit_wall(self, side_pts):
        """RANSAC line fit y = m*x + b on one side's points.

        Picks the line with the most inliers, so a long wall beats a short box
        face. Returns (m, b, inliers) or None if no convincing wall.
        """
        p = self.p
        n = len(side_pts)
        if n < 8:
            return None
        rng = self._rng
        max_m = math.tan(math.radians(50))
        best = None
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
            if best is None or len(inl) > len(best):
                best = inl
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
        return m, my - m * mx, best

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
        tot = sum(len(f[2]) for f in fits)
        phi = sum(math.atan(f[0]) * len(f[2]) for f in fits) / tot

        def dist(f, sign):
            m, b, _ = f
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
        ahead = [(x, y) for x, y in pts
                 if 0.0 < x < p.lookahead and -R + 0.05 < y < L - 0.05]
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
    def path_distance(self, pts):
        """Nearest x of anything directly in the robot's path."""
        half = self.p.robot_half_width + self.p.side_margin
        ahead = [x for x, y in pts if x > 0.0 and abs(y) <= half]
        return min(ahead) if ahead else math.inf

    def decide(self, pts) -> Decision:
        p = self.p
        if len(pts) < 20:
            return Decision(0.0, 0.0, 'NO_DATA', {'points': len(pts)})

        # Work out wall/gap geometry in a "hallway frame" lined up with the walls,
        # so an angled robot doesn't see the walls ahead as obstacles.
        phi, L, R, wall_mode = self.find_walls(pts)
        c, s = math.cos(phi), math.sin(phi)
        hpts = [(x * c + y * s, -x * s + y * c) for x, y in pts]

        gaps, n_inner = self.gaps(hpts, L, R)
        front = self.path_distance(pts)          # robot frame: what's in our actual path
        info = {'L': L, 'R': R, 'walls': wall_mode, 'front': front, 'gaps': len(gaps),
                'hall_deg': math.degrees(phi)}

        if not gaps:
            self.last_side = 0
            return Decision(0.0, 0.0, 'BLOCKED', info)

        lo, hi = self.choose_gap(gaps)
        target_y = 0.5 * (lo + hi)
        info['target_y'] = target_y

        # aim point in the hallway frame -> robot frame
        ax, ay = p.steer_lookahead, target_y
        rx, ry = ax * c - ay * s, ax * s + ay * c
        heading_err = math.atan2(ry, rx)
        angular = max(-p.max_turn, min(p.max_turn, p.kp_heading * heading_err))

        # don't swing into something that's right beside us
        cl_left, cl_right = self.side_clearance(pts)
        info['side_clear'] = (cl_left, cl_right)
        if angular > 0 and cl_left < p.robot_half_width + p.side_stop:
            angular = 0.0
        elif angular < 0 and cl_right < p.robot_half_width + p.side_stop:
            angular = 0.0

        in_path = front < p.lookahead
        if in_path and abs(target_y) > 0.05:
            self.last_side = 1 if target_y > 0 else -1
        elif not in_path:
            self.last_side = 0

        if front <= p.stop_dist:
            if abs(target_y) < 0.02:     # gap is dead ahead but something's in the way
                return Decision(0.0, 0.0, 'BLOCKED', info)
            turn = math.copysign(p.pivot_turn, target_y)
            return Decision(0.0, turn, 'PIVOT', info)

        if front >= p.slow_dist:
            linear = p.cruise_speed
        else:
            t = (front - p.stop_dist) / (p.slow_dist - p.stop_dist)
            linear = p.slow_speed + t * (p.cruise_speed - p.slow_speed)
        # ease off when turning hard
        linear *= max(0.4, 1.0 - 0.5 * abs(angular) / p.max_turn)

        state = 'AVOID' if n_inner and in_path else 'FOLLOW'
        return Decision(linear, angular, state, info)
