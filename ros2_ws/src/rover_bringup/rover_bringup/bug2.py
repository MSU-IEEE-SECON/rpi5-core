"""Bug2 lap driving for the arena (no ROS, testable off-robot).

Each side of the lap is one Bug2 leg: the "m-line" runs from where the leg
started to the next corner. The rover drives along it; if something blocks the
way it follows the obstacle's edge (strafing, mecanum) until it is back ON the
m-line, CLOSER to the corner than where it got blocked, with a clear path
ahead, then carries on. The center rail (invisible to the LiDAR) and a fence
across the pit openings are added as virtual obstacles, so Bug2 steers around
them like any other.

The rover keeps facing along the current leg (LiDAR looking ahead) and turns
in place at each corner. Poses are the ROVER CENTRE in board coordinates
(see arena.py); obstacle points are in the same frame.
"""
import math
from dataclasses import dataclass

import numpy as np

from rover_bringup import arena as A


@dataclass
class LapParams:
    speed: float = 0.18            # m/s cruise
    slow_speed: float = 0.08       # m/s sidestepping / near a corner
    follow_speed: float = 0.14     # m/s rolling past an obstacle
    turn_rate: float = 1.0         # rad/s max
    kp_heading: float = 1.5        # 2.0 wobbled +-15 deg on the real rover (Nov 13)
    kp_line: float = 1.5           # pull back onto the m-line (1/s)
    half_len: float = 0.127        # rover body half length (m): 10 in long (measured Nov 14)
    half_wid: float = 0.13         # half width incl. wheels (10 in rover)
    clearance: float = 0.012       # keep this gap between body and anything (lanes are tight:
                                   # a car mid-lane leaves ~11 in for a 10 in rover)
    probe: float = 0.15            # look this far ahead of the body for blocking
    side_probe: float = 0.012      # ...and this far when moving sideways (about one step)
    follow_gap: float = 0.035      # boundary-follow distance from the obstacle
    corner_tol: float = 0.06       # m, corner reached
    heading_tol_deg: float = 8.0   # turn in place until within this
    leave_tol: float = 0.04        # m from the m-line counts as "on it"
    laps: int = 3
    lidar_fwd: float = 0.102       # LiDAR 1 in from the front = 4 in ahead of the body centre
    view_deg: float = 83.0         # LiDAR sees +- this much (overridden by the calibration)
    memory: bool = True            # remember obstacles on the board (the LiDAR can't see beside the body)
    mem_cell: float = 0.01         # m, memory grid
    mem_hits: int = 3              # scans a cell must be seen in before it counts


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bug2Lap:

    def __init__(self, params=None, route=A.LAP):
        self.p = params or LapParams()
        self.route = list(route)
        self.mem = np.zeros((int(A.W / self.p.mem_cell) + 2, int(A.L / self.p.mem_cell) + 2), np.int16)
        self.reset(None)

    def reset(self, pose):
        """pose = rover centre (x, y, th). Picks the next corner ahead of the rover."""
        self.leg = 0
        self.mode = 'TURN'
        self.laps_done = 0
        self.hit = None
        self.side = 1
        self.offset, self.squeeze, self.stuck = None, False, 0
        self.start = None if pose is None else (pose[0], pose[1])
        if pose is not None:
            self.leg = self._first_leg(pose)

    def _first_leg(self, pose):
        # the corner the rover is heading toward: smallest angle off its heading, ahead of it
        best, bi = 1e9, 0
        for i, (gx, gy) in enumerate(self.route):
            d = math.hypot(gx - pose[0], gy - pose[1])
            if d < self.p.corner_tol:
                continue
            off = abs(wrap(math.atan2(gy - pose[1], gx - pose[0]) - pose[2]))
            if off + 0.2 * d < best:
                best, bi = off + 0.2 * d, i
        return bi

    # ---- geometry helpers ---------------------------------------------------
    def _body_frame(self, pts, pose):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        rel = pts - [x, y]
        return np.stack([rel[:, 0] * c + rel[:, 1] * s, -rel[:, 0] * s + rel[:, 1] * c], 1)

    def _blocked(self, B, direction, dist=None, tight=False):
        """Is anything in the way if the body moves `dist` along `direction` (body frame unit vec)?
        tight=True: no clearance margin (only actual contact counts)."""
        p = self.p
        dist = p.probe if dist is None else dist
        m = 0.0 if tight else p.clearance
        hl, hw = p.half_len + m, p.half_wid + m
        dx, dy = direction
        B = B[B @ np.array([dx, dy]) > 0.0]     # only what's on the side we're moving toward
        for k in np.linspace(0.0, dist, 5):
            X, Y = B[:, 0] - dx * k, B[:, 1] - dy * k
            if np.any((np.abs(X) <= hl) & (np.abs(Y) <= hw)):
                return True
        return False

    def _body_dist(self, B):
        """Distance from each point to the body rectangle (0 inside)."""
        ex = np.maximum(np.abs(B[:, 0]) - self.p.half_len, 0)
        ey = np.maximum(np.abs(B[:, 1]) - self.p.half_wid, 0)
        return np.hypot(ex, ey)

    # ---- main step ----------------------------------------------------------
    def step(self, pose, obstacle_pts):
        """pose: rover centre (x, y, th). obstacle_pts: (N,2) world points (LiDAR).

        Returns (vx_fwd, vy_left, wz, state, info) in the ROVER's frame.
        """
        p = self.p
        x, y, th = pose
        if self.laps_done >= p.laps:
            return 0.0, 0.0, 0.0, 'DONE', {'laps': self.laps_done}
        scan = np.asarray(obstacle_pts, float).reshape(-1, 2)
        pts = [scan, A.RAIL_PTS, A.FENCE_PTS, A.CHAMFER_PTS]
        if p.memory:
            mem = self._remember(scan)
            if len(mem):
                # the live scan covers what's in view; use memory only beside/behind the LiDAR
                lx_, ly_ = x + p.lidar_fwd * math.cos(th), y + p.lidar_fwd * math.sin(th)
                bearing = np.arctan2(mem[:, 1] - ly_, mem[:, 0] - lx_) - th
                bearing = np.abs(np.arctan2(np.sin(bearing), np.cos(bearing)))
                mem = mem[bearing > math.radians(p.view_deg - 5)]
            pts.append(mem)
        pts = np.vstack(pts)
        B = self._body_frame(pts, pose)
        # 'hard' = things that must never be touched: the walls, the rail, the pit fence
        hard = np.vstack([A.RAIL_PTS, A.FENCE_PTS, A.CHAMFER_PTS, scan[(scan[:, 0] < A.X0 + 0.03) | (scan[:, 0] > A.X1 - 0.03)
                                                         | (scan[:, 1] > A.Y_TOP - 0.03)
                                                         | (scan[:, 1] < A.Y_SEP + A.T + 0.03)]])
        self._hard = self._body_frame(hard, pose)
        gx, gy = self.route[self.leg]
        if self.start is None:
            self.start = (x, y)
        leg_dir = math.atan2(gy - self.start[1], gx - self.start[0])
        dist_goal = math.hypot(gx - x, gy - y)
        info = {'leg': self.leg, 'goal': (gx, gy), 'dist_goal': dist_goal, 'laps': self.laps_done}

        # corner reached -> next leg
        if dist_goal < p.corner_tol:
            self.start = (gx, gy)
            if self.leg == len(self.route) - 1:
                self.laps_done += 1
            self.leg = (self.leg + 1) % len(self.route)
            self.mode, self.hit = 'TURN', None
            return 0.0, 0.0, 0.0, 'CORNER', info

        # face along the leg first (turn in place at corners)
        want_th = math.atan2(gy - y, gx - x) if self.mode != 'FOLLOW' else leg_dir
        herr = wrap(want_th - th)
        wz = max(-p.turn_rate, min(p.turn_rate, p.kp_heading * herr))
        if self.mode == 'TURN':
            if abs(herr) > math.radians(p.heading_tol_deg):
                return 0.0, 0.0, wz, 'TURN', info
            self.mode = 'GOTO'

        # direction to the goal, in the body frame
        gb = np.array([math.cos(want_th - th), math.sin(want_th - th)])

        if self.mode == 'GOTO':
            if not self._blocked(B, gb, min(p.probe, dist_goal)):
                # head for the corner; pull back onto the m-line
                lx, ly = self.start
                mx, my = math.cos(leg_dir), math.sin(leg_dir)
                cross = (x - lx) * my - (y - ly) * mx          # + = right of the line
                v = min(p.speed, 0.15 + 0.6 * dist_goal) if dist_goal > 0.25 else p.slow_speed
                # cross > 0 = right of the line -> move toward the line's left normal (-my, mx)
                side_v = max(-0.08, min(0.08, p.kp_line * cross))
                wvx = v * math.cos(want_th) + side_v * (-my)
                wvy = v * math.sin(want_th) + side_v * mx
                vx, vy = (wvx * math.cos(th) + wvy * math.sin(th),
                          -wvx * math.sin(th) + wvy * math.cos(th))
                info['cross'] = cross
                return vx, vy, wz, 'GOTO', info
            # blocked: remember the hit point, pick a side
            self.mode = 'FOLLOW'
            self.hit = (x, y, dist_goal)
            self.offset, self.squeeze, self.stuck = None, False, 0

        # FOLLOW the obstacle's edge. In a lane the obstacles are small (toy cars),
        # so "following the boundary" = keep facing along the leg, strafe out to a
        # gap beside it, roll forward alongside it, and step back toward the m-line
        # as soon as there's room. The gap is picked by trying sideways offsets.
        lx, ly = self.start
        mx, my = math.cos(leg_dir), math.sin(leg_dir)
        cross = (x - lx) * my - (y - ly) * mx              # + = right of the m-line
        info['cross'] = cross
        fwd = np.array([1.0, 0.0])
        on_line = abs(cross) < p.leave_tol
        closer = dist_goal < self.hit[2] - 0.03
        look = min(p.probe, dist_goal)
        if on_line and closer and not self._blocked(B, gb, look):
            self.mode = 'GOTO'
            info['left_obstacle'] = True
            return 0.0, 0.0, wz, 'LEAVE', info
        if not self._blocked(self._hard if self.squeeze else B, fwd, look):
            # clear ahead: roll forward; drift back toward the m-line where there's room
            v = p.follow_speed if not self.squeeze else p.slow_speed
            back = np.array([0.0, 1.0 if cross > 0 else -1.0])
            vy = 0.0
            if (abs(cross) > p.leave_tol and not self._blocked(B, back, p.side_probe)
                    and not self._blocked(B - back * 0.03, fwd)):
                vy = back[1] * min(v, p.kp_line * abs(cross))
            self.stuck = 0
            return v, vy, wz, 'SQUEEZE' if self.squeeze else 'FOLLOW', info
        # blocked ahead: strafe toward the chosen gap
        if self.offset is None or abs(self.offset) < 0.005:
            self.offset, self.squeeze = self._find_gap(B, look)
            info['gap'] = (self.offset, self.squeeze)
        step = math.copysign(min(abs(self.offset), p.slow_speed * 0.1), self.offset)
        if self._blocked(self._hard if self.squeeze else B,
                         np.array([0.0, math.copysign(1.0, self.offset)]), p.side_probe):
            self.offset = None                 # can't go that way after all: re-plan
            self.stuck += 1
            if self.stuck > 30:                # boxed in for 3 s: accept a tighter squeeze
                self.squeeze = True
            return 0.0, 0.0, wz, 'FOLLOW', info
        self.offset -= step
        return 0.0, step / 0.1, wz, 'SQUEEZE' if self.squeeze else 'FOLLOW', info

    def _find_gap(self, B, look):
        """Smallest sideways shift (m, + = left) that clears the way ahead.

        If no shift clears it (a car in the middle of a lane leaves less room than
        a 10 in rover needs: about 1 placement in 5), pick the shift with the most
        room and go through anyway ('squeeze': it may nudge the car, which costs
        that lap's no-contact bonus, but never touches the walls or the rail).
        """
        p = self.p
        best, best_room = 0.0, -1.0
        for o in sorted(np.arange(-0.32, 0.3201, 0.01), key=abs):
            S = B - [0.0, o]
            # path from here to the offset must be clear too (body sliding sideways)
            if not self._blocked(S, np.array([1.0, 0.0]), look) and not self._lane_blocked(B, o):
                return float(o), False
            room = self._room(S, look)
            if room > best_room:
                best, best_room = float(o), room
        return best, True

    def _lane_blocked(self, B, o):
        """Would sliding sideways by o run into something?"""
        p = self.p
        hl, hw = p.half_len + p.clearance, p.half_wid + p.clearance
        lo, hi = (min(0.0, o) - hw, max(0.0, o) + hw)
        return bool(np.any((np.abs(B[:, 0]) <= hl) & (B[:, 1] >= lo) & (B[:, 1] <= hi)))

    def _room(self, S, look):
        """Smallest distance from the body (slid forward by look) to any point."""
        p = self.p
        best = 1.0
        for k in np.linspace(0.0, look, 4):
            X, Y = S[:, 0] - k, S[:, 1]
            ex = np.maximum(np.abs(X) - p.half_len, 0)
            ey = np.maximum(np.abs(Y) - p.half_wid, 0)
            d = np.hypot(ex, ey)
            ahead = X > -p.half_len
            if ahead.any():
                best = min(best, float(d[ahead].min()))
        return best

    def _remember(self, scan):
        """Add this scan's non-wall points to the board memory; return remembered obstacles.

        The LiDAR (front-mounted, +-83 deg) can't see what's beside or behind the
        body, so a car being passed would vanish; remembering it in board
        coordinates (we know where we are) keeps the rover from swinging into it.
        """
        c = self.p.mem_cell
        if len(scan):
            far = np.ones(len(scan), bool)
            # skip points on the known walls (they're always there anyway)
            far &= (scan[:, 0] > A.X0 + 0.03) & (scan[:, 0] < A.X1 - 0.03)
            far &= (scan[:, 1] > A.Y_SEP + A.T + 0.03) & (scan[:, 1] < A.Y_TOP - 0.03)
            ij = np.floor(scan[far] / c).astype(int)
            ok = (ij[:, 0] >= 0) & (ij[:, 0] < self.mem.shape[0]) & (ij[:, 1] >= 0) & (ij[:, 1] < self.mem.shape[1])
            ij = np.unique(ij[ok], axis=0)
            if len(ij):
                self.mem[ij[:, 0], ij[:, 1]] = np.minimum(self.mem[ij[:, 0], ij[:, 1]] + 1, 20)
        cells = (np.argwhere(self.mem >= self.p.mem_hits) + 0.5) * c
        return cells

    def _pick_side(self, B):
        """Go around on the side with more room (fewer points close on that side)."""
        ahead = (B[:, 0] > 0) & (B[:, 0] < 0.6)
        left = ((B[:, 1] > 0) & (B[:, 1] < 0.45) & ahead).sum()
        right = ((B[:, 1] < 0) & (B[:, 1] > -0.45) & ahead).sum()
        # side=+1 -> tangent turns so we pass with the obstacle on our right (go left)
        return 1 if left <= right else -1
