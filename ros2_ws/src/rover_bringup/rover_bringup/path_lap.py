"""Racing-line laps for the arena: a fixed smooth loop, shifted sideways around obstacles.

(No ROS, testable off-robot.) Replaces the stop-turn-go Bug2 driving:

* One closed path through the lane centres with ROUNDED corners (arena.py geometry).
  The rover turns while it drives (mecanum), facing along the path so the LiDAR looks
  ahead, and slows down for the curves instead of stopping.
* Obstacles shift the path sideways. Toy cars seen by the LiDAR go into a board map
  (they don't move during a run, so laps 2 and 3 already know them). Every scan, a small
  dynamic program picks a lateral offset for the next metre of path: as close to the
  lane centre as possible, never touching a wall/rail/corner board/pit fence, avoiding
  cars, and changing gradually (no jerks). If a car leaves no room, it squeezes past
  slowly, touching the car rather than a wall.
* Smooth control: pure-pursuit style tracking of the shifted path, speed and
  acceleration limits.

Poses are the ROVER CENTRE (x, y, th) in board metres (see arena.py).
"""
import math
from dataclasses import dataclass

import numpy as np

from rover_bringup import arena as A


@dataclass
class PathParams:
    speed: float = 0.20            # m/s on the straights
    corner_speed: float = 0.12     # m/s in the rounded corners
    squeeze_speed: float = 0.09    # m/s when the plan has to touch a car
    accel: float = 0.35            # m/s^2 speed change limit (smooth starts/stops)
    max_wz: float = 1.2            # rad/s
    max_strafe: float = 0.12       # m/s sideways
    k_lat: float = 2.5             # 1/s pull onto the (shifted) path
    k_head: float = 1.5            # 1/s heading correction
    lead: float = 0.06             # m reference point ahead of the projection
    preview: float = 0.3           # s: look this far ahead for the turn feed-forward (motor lag)
    k_pre: float = 1.0             # 1/s: also steer toward the heading the path will have then
    half_len: float = 0.127        # rover body half length: 10 in long (measured Nov 14)
    half_wid: float = 0.13         # rover body half width incl. wheels (10.2 in)
    wall_clear: float = 0.022      # never closer than this to walls/rail/board/fence. 1.2 cm let
                                   # the rover graze the wall when a car pushed the plan to the edge;
                                   # above 2.3 cm the racing line itself breaks it (bottom-right corner)
    heading_tol: float = 0.14      # rad (~8 deg): walls must stay clear even this far off heading
    car_clear: float = 0.03        # wanted gap to cars
    max_offset: float = 0.17       # m sideways from the racing line, either way (walls limit it)
    offset_step: float = 0.01      # m offset resolution
    plan_ds: float = 0.05          # m between planned points
    horizon: float = 1.5           # m planned ahead
    slope: float = 0.8             # max offset change per metre travelled
    w_centre: float = 30.0         # cost of being off the racing line (per m^2)
    w_smooth: float = 2.0          # cost of offset slope (per (m/m)^2)
    car_cost: float = 1000.0       # cost of a planned point inside the car_clear margin
    car_hard_cost: float = 20000.0  # extra cost per 2 cm of body overlapping a car
    laps: int = 3
    lidar_fwd: float = 0.102       # LiDAR 1 in from the front = 4 in ahead of the body centre
    cell: float = 0.01             # obstacle map cell (m)
    hits: int = 3                  # scans a cell must be seen in to count as an obstacle
    obstacle_range: float = 1.6    # m: farther LiDAR points are too uncertain to map
    known_gap: float = 0.045       # m: points this close to known geometry are not obstacles


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---- the racing line ---------------------------------------------------------------
Y_TOP_LINE = (A.RAIL_Y1 + A.Y_TOP) / 2          # middle of the top crossing
Y_BOT_LINE = (A.Y_SEP + A.T + A.RAIL_Y0) / 2    # middle of the pit-side crossing
START_X = (A.SEP_X1 + A.X1) / 2                  # middle of the right-hand pit opening
# corners CCW (up the right lane first) and their fillet radii; the bottom-left one is big
# so the curve stays clear of the 45-degree corner board there
RADIUS_BR = 0.17
CORNERS = [((A.LANE_R, Y_TOP_LINE), 0.17), ((A.LANE_L, Y_TOP_LINE), 0.17),
           ((A.LANE_L, Y_BOT_LINE), 0.26), ((A.LANE_R, Y_BOT_LINE), RADIUS_BR)]


def build_path(corners=CORNERS, step=0.01):
    """Closed loop with rounded corners -> (P (N,2), T (N,2) unit tangents, K (N,) curvature, s (N,))."""
    pts, kap = [], []
    n = len(corners)
    fil = []
    for i in range(n):
        c = np.array(corners[i][0], float)
        r = corners[i][1]
        a = c - np.array(corners[i - 1][0], float)
        b = np.array(corners[(i + 1) % n][0], float) - c
        a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
        turn = math.atan2(a[0] * b[1] - a[1] * b[0], a @ b)        # + = left
        t = r * math.tan(abs(turn) / 2)
        p_in, p_out = c - a * t, c + b * t
        left = np.array([-a[1], a[0]]) * (1 if turn > 0 else -1)
        centre = p_in + left * r
        fil.append((p_in, p_out, centre, r, turn))
    for i in range(n):
        p_in, p_out, centre, r, turn = fil[i]
        # arc
        a0 = math.atan2(p_in[1] - centre[1], p_in[0] - centre[0])
        m = max(2, int(abs(turn) * r / step))
        for k in range(m):
            ang = a0 + turn * k / m
            pts.append(centre + r * np.array([math.cos(ang), math.sin(ang)]))
            kap.append(math.copysign(1.0 / r, turn))
        # straight to the next corner's arc
        q_in = fil[(i + 1) % n][0]
        L = np.linalg.norm(q_in - p_out)
        m = max(1, int(L / step))
        for k in range(m):
            pts.append(p_out + (q_in - p_out) * k / m)
            kap.append(0.0)
    P = np.array(pts)
    d = np.roll(P, -1, 0) - P
    seg = np.linalg.norm(d, axis=1)
    T = d / seg[:, None]
    s = np.concatenate([[0.0], np.cumsum(seg)[:-1]])
    return P, T, np.array(kap), s, float(seg.sum())


STATIC_SEGS = A.WALLS + A.RAIL + A.PIT_FENCE        # never touch (WALLS has the corner board)
STATIC_PTS = A.sample_segments(STATIC_SEGS, 0.01)


def _rect_hits(pts, cx, cy, th, hl, hw):
    """Boolean per pose: any of pts inside the rectangle(s). cx, cy, th: (M,) arrays."""
    c, s = np.cos(th), np.sin(th)
    rx = pts[None, :, 0] - cx[:, None]
    ry = pts[None, :, 1] - cy[:, None]
    bx = rx * c[:, None] + ry * s[:, None]
    by = -rx * s[:, None] + ry * c[:, None]
    return np.any((np.abs(bx) <= hl) & (np.abs(by) <= hw), axis=1)


def _rect_overlap(pts, cx, cy, th, hl, hw):
    """Per pose: how far (m) the deepest of pts reaches sideways into the rectangle, 0 if none."""
    c, s = np.cos(th), np.sin(th)
    rx = pts[None, :, 0] - cx[:, None]
    ry = pts[None, :, 1] - cy[:, None]
    bx = rx * c[:, None] + ry * s[:, None]
    by = -rx * s[:, None] + ry * c[:, None]
    pen = np.where(np.abs(bx) <= hl, hw - np.abs(by), 0.0)
    return np.clip(pen.max(axis=1), 0.0, None)


class PathLap:

    def __init__(self, params=None):
        self.p = p = params or PathParams()
        self.P, self.T, self.K, self.s, self.L = build_path()
        self.N = np.stack([-self.T[:, 1], self.T[:, 0]], 1)      # left normal (toward the inside)
        self.offsets = np.round(np.arange(-p.max_offset, p.max_offset + 1e-9, p.offset_step), 4)
        # plan grid along the loop and its static feasibility table (done once)
        self.gs = np.arange(0.0, self.L, p.plan_ds)
        gi = np.searchsorted(self.s, self.gs).clip(0, len(self.s) - 1)
        self.g_idx = gi
        self.static_ok = np.zeros((len(self.gs), len(self.offsets)), bool)
        th = np.arctan2(self.T[gi, 1], self.T[gi, 0])
        for j, d in enumerate(self.offsets):
            cx = self.P[gi, 0] + d * self.N[gi, 0]
            cy = self.P[gi, 1] + d * self.N[gi, 1]
            hit = np.zeros(len(gi), bool)
            for k0 in range(0, len(gi), 16):
                sl = slice(k0, k0 + 16)
                # the real heading wanders a few degrees off the path: check that too
                for dth in (-p.heading_tol, 0.0, p.heading_tol):
                    hit[sl] |= _rect_hits(STATIC_PTS, cx[sl], cy[sl], th[sl] + dth,
                                          p.half_len + p.wall_clear, p.half_wid + p.wall_clear)
            self.static_ok[:, j] = ~hit
        # known geometry distance check for mapping (cars vs walls), 2 cm grid
        self._known = A.sample_segments(STATIC_SEGS + [(A.X0, A.T, A.X1, A.T)], 0.02)
        self.grid = np.zeros((int(A.W / p.cell) + 2, int(A.L / p.cell) + 2), np.int16)
        self.reset(None)

    # ---- state -----------------------------------------------------------------
    def reset(self, pose):
        self.grid[:] = 0              # new run: cars may have been moved, forget the old map
        self.laps_done = 0
        self.progress = 0.0           # metres travelled along the loop since the start
        self.k = None                 # index of the current projection on the path
        self.v = 0.0                  # current commanded speed (ramped)
        self.plan_d = np.zeros(int(self.p.horizon / self.p.plan_ds) + 1)
        self.plan_s0 = 0.0
        self.squeezing = False
        # starting in the start box (pit side of the separator): drive straight out through the
        # right-hand opening first, then join the racing line in the right lane
        self.start_leg = pose is not None and pose[1] < A.Y_SEP + A.T + 0.10
        self.start_x = START_X
        if pose is not None and not self.start_leg:
            self.k = self._project(pose[0], pose[1], None)

    def _project(self, x, y, k_prev):
        if k_prev is None:
            cand = np.arange(len(self.P))
        else:
            span = int(0.4 / 0.01)
            cand = (k_prev + np.arange(-span // 4, span)) % len(self.P)
        d = np.hypot(self.P[cand, 0] - x, self.P[cand, 1] - y)
        return int(cand[int(np.argmin(d))])

    # ---- obstacle map ------------------------------------------------------------
    def _remember(self, pts, lidar_xy, pose=None):
        p = self.p
        if len(pts) == 0:
            return
        if pose is not None:
            # never map the rover itself (wheels/frame the self mask misses): on the real
            # rover these left a trail of fake obstacles along its own path, so every later
            # pass thought it was squeezing past a car and crawled (Nov 14)
            c, s = math.cos(pose[2]), math.sin(pose[2])
            rx, ry = pts[:, 0] - pose[0], pts[:, 1] - pose[1]
            bx, by = rx * c + ry * s, -rx * s + ry * c
            pts = pts[~((np.abs(bx) <= p.half_len + 0.04) & (np.abs(by) <= p.half_wid + 0.04))]
            if len(pts) == 0:
                return
        r = np.hypot(pts[:, 0] - lidar_xy[0], pts[:, 1] - lidar_xy[1])
        pts = pts[r < p.obstacle_range]
        inside = ((pts[:, 0] > A.X0 + 0.02) & (pts[:, 0] < A.X1 - 0.02) &
                  (pts[:, 1] > A.Y_SEP + A.T + 0.02) & (pts[:, 1] < A.Y_TOP - 0.02))
        pts = pts[inside]
        if len(pts) == 0:
            return
        # not an obstacle if it's known geometry (wall, rail, corner board)
        far = np.ones(len(pts), bool)
        for k0 in range(0, len(pts), 256):
            q = pts[k0:k0 + 256]
            dmin = np.min(np.hypot(q[:, None, 0] - self._known[None, :, 0],
                                   q[:, None, 1] - self._known[None, :, 1]), axis=1)
            far[k0:k0 + 256] = dmin > p.known_gap
        pts = pts[far]
        ix = (pts[:, 0] / p.cell).astype(int)
        iy = (pts[:, 1] / p.cell).astype(int)
        cells = np.unique(np.stack([ix, iy], 1), axis=0)
        self.grid[cells[:, 0], cells[:, 1]] += 1

    def add_camera(self, pts, cam_xy, pose):
        """Floor points of cars seen by the webcam (board frame). Same map as the LiDAR,
        but each point marks a 2x2 cm patch: the camera's floor estimate wobbles by about a
        centimetre from frame to frame, and a car has to be seen in `hits` frames to count."""
        pts = np.asarray(pts, float).reshape(-1, 2)
        if len(pts) == 0:
            return
        c = self.p.cell
        snap = (np.floor(pts / (2 * c)) * 2 + 0.5) * c          # lower-left cell of the patch
        quad = np.concatenate([snap + [dx * c, dy * c] for dx in (0, 1) for dy in (0, 1)])
        self._remember(np.unique(np.round(quad / c).astype(int), axis=0) * c + 0.5 * c,
                       cam_xy, pose)

    def obstacles(self):
        ix, iy = np.nonzero(self.grid >= self.p.hits)
        return np.stack([(ix + 0.5) * self.p.cell, (iy + 0.5) * self.p.cell], 1)

    # ---- planning ---------------------------------------------------------------
    def _plan(self, s_now, d_now):
        """Lateral offsets for the next `horizon` metres (dynamic programming)."""
        p = self.p
        H = len(self.plan_d)
        ss = (s_now + np.arange(H) * p.plan_ds) % self.L
        gi = np.round(ss / p.plan_ds).astype(int) % len(self.gs)
        ok = self.static_ok[gi]                          # (H, J)
        obs = self.obstacles()
        car = np.zeros_like(ok)
        depth = np.zeros(ok.shape)      # sideways overlap of the body with a car, in units of 2 cm
        if len(obs):
            idx = self.g_idx[gi]
            th = np.arctan2(self.T[idx, 1], self.T[idx, 0])
            # only obstacles near the horizon
            box_lo = self.P[idx].min(0) - 0.4
            box_hi = self.P[idx].max(0) + 0.4
            obs = obs[np.all((obs >= box_lo) & (obs <= box_hi), axis=1)]
            if len(obs):
                for j, d in enumerate(self.offsets):
                    cx = self.P[idx, 0] + d * self.N[idx, 0]
                    cy = self.P[idx, 1] + d * self.N[idx, 1]
                    car[:, j] = _rect_hits(obs, cx, cy, th, p.half_len + p.car_clear,
                                           p.half_wid + p.car_clear)
                    if car[:, j].any():
                        m = car[:, j]
                        depth[m, j] = _rect_overlap(obs, cx[m], cy[m], th[m],
                                                    p.half_len, p.half_wid) / 0.02
        # a margin-only touch costs car_cost; each 2 cm of real body overlap costs more on top,
        # so when a car can't be cleared the plan still moves toward its edge (with a flat
        # cost every touching offset looked equal and the plan drove through the middle)
        node = (p.w_centre * self.offsets[None, :] ** 2 + np.where(car, p.car_cost, 0.0)
                + p.car_hard_cost * depth)
        node = np.where(ok, node, np.inf)
        J = len(self.offsets)
        dd = self.offsets[None, :] - self.offsets[:, None]          # (prev, next)
        trans = p.w_smooth * (dd / p.plan_ds) ** 2 * p.plan_ds
        trans = np.where(np.abs(dd) <= p.slope * p.plan_ds + 1e-9, trans, np.inf)
        # start: the offset the rover is at now
        j0 = int(np.argmin(np.abs(self.offsets - np.clip(d_now, -p.max_offset, p.max_offset))))
        cost = np.full(J, np.inf)
        cost[j0] = 0.0
        back = np.zeros((H, J), int)
        for h in range(1, H):
            tot = cost[:, None] + trans                              # prev x next
            back[h] = np.argmin(tot, axis=0)
            cost = tot[back[h], np.arange(J)] + node[h]
        if not np.isfinite(cost).any():
            return None, True
        j = int(np.argmin(cost))
        path = [j]
        for h in range(H - 1, 0, -1):
            j = back[h, j]
            path.append(j)
        path.reverse()
        d = self.offsets[path]
        near = max(2, int(0.35 / p.plan_ds))               # only contact coming up soon counts
        touches = bool(car[np.arange(near), path[:near]].any())
        return d, touches

    # ---- main step ----------------------------------------------------------------
    def step(self, pose, obstacle_pts, dt=0.1):
        """pose: rover centre (x, y, th). obstacle_pts: (N,2) world points from the LiDAR.
        Returns (vx_fwd, vy_left, wz, state, info) in the ROVER's frame."""
        p = self.p
        x, y, th = pose
        if self.laps_done >= p.laps:
            self.v = 0.0
            return 0.0, 0.0, 0.0, 'DONE', {'laps': self.laps_done, 'leg': None}
        lidar = (x + p.lidar_fwd * math.cos(th), y + p.lidar_fwd * math.sin(th))
        self._remember(np.asarray(obstacle_pts, float).reshape(-1, 2), lidar, pose)
        if self.start_leg:
            if y > Y_BOT_LINE + 0.10:                 # out of the pit: join the racing line
                self.start_leg = False
                self.k = self._project(x, y, None)
            else:
                return self._start_step(pose, dt)

        k_prev = self.k
        self.k = self._project(x, y, k_prev)
        if k_prev is not None:
            ds = (self.s[self.k] - self.s[k_prev]) % self.L
            if ds > self.L / 2:
                ds -= self.L                                    # moved backwards a bit
            self.progress += ds
            self.laps_done = max(self.laps_done, int(self.progress // self.L))
        s_now = self.s[self.k]
        e = np.array([x, y]) - self.P[self.k]
        d_now = float(e @ self.N[self.k])

        # start the new plan from where the previous plan says we should be now (keeps the
        # plan steady; the tracking controller takes care of the small error to it)
        adv = ((s_now - self.plan_s0) % self.L) if k_prev is not None else 0.0
        d_prev = float(np.interp(adv, np.arange(len(self.plan_d)) * p.plan_ds, self.plan_d))
        d_start = d_prev if abs(d_prev - d_now) < 0.10 else d_now
        plan, touches = self._plan(s_now, d_start)
        state = 'PATH'
        if plan is None:                     # no static-safe offsets at all (shouldn't happen)
            plan, touches = np.zeros(len(self.plan_d)), True
            state = 'BLOCKED'
        self.plan_d, self.plan_s0 = plan, s_now
        self.squeezing = touches

        # reference point a little ahead on the shifted path
        s_ref = (s_now + p.lead) % self.L
        kr = int(np.searchsorted(self.s, s_ref)) % len(self.P)
        d_ref = float(np.interp(p.lead, np.arange(len(plan)) * p.plan_ds, plan))
        ref = self.P[kr] + d_ref * self.N[kr]
        t_hat = self.T[kr]
        th_ref = math.atan2(t_hat[1], t_hat[0])

        # speed: slow for curves ahead, for sideways shifts, and when squeezing
        ahead = (self.k + np.arange(0, 30)) % len(self.P)      # next 0.3 m
        v_tgt = p.corner_speed if np.max(np.abs(self.K[ahead])) > 1e-6 else p.speed
        slope_ahead = np.max(np.abs(np.diff(plan[:6]))) / p.plan_ds if len(plan) > 1 else 0.0
        v_tgt = min(v_tgt, p.speed * (1.0 - 0.6 * min(1.0, slope_ahead / p.slope)))
        if touches:
            v_tgt = min(v_tgt, p.squeeze_speed)
            state = 'SQUEEZE'
        if self.progress + p.lead >= p.laps * self.L:      # finishing: brake to the line
            v_tgt = min(v_tgt, max(0.03, (p.laps * self.L - self.progress) * 1.5))
        dv = p.accel * dt
        self.v = min(self.v + dv, v_tgt) if v_tgt > self.v else max(self.v - 2 * dv, v_tgt)

        err = ref - np.array([x, y])
        err_perp = err - (err @ t_hat) * t_hat
        vb = self.v * t_hat + p.k_lat * err_perp
        c, s = math.cos(th), math.sin(th)
        vx = c * vb[0] + s * vb[1]
        vy = -s * vb[0] + c * vb[1]
        vy = max(-p.max_strafe, min(p.max_strafe, vy))
        # curvature feed-forward PREVIEWED ahead: the motors lag (~0.2-0.3 s), so start and end
        # turning a little early, or the rover overshoots out of every corner (it then swings
        # its nose into cars right after the turn)
        kp = (self.k + int(self.v * p.preview / 0.01)) % len(self.P)
        th_pre = math.atan2(self.T[kp, 1], self.T[kp, 0])
        wz = self.v * self.K[kp] + p.k_head * wrap(th_ref - th) + p.k_pre * wrap(th_pre - th_ref)
        wz = max(-p.max_wz, min(p.max_wz, wz))
        # big heading error (start, after a bump): turn mostly in place
        if abs(wrap(th_ref - th)) > math.radians(35):
            vx, vy = 0.3 * vx, 0.3 * vy
        leg = self._leg(s_now)
        info = {'laps': self.laps_done, 'leg': leg, 'offset': round(d_ref, 3),
                'progress': round(self.progress, 2), 'obstacles': int((self.grid >= p.hits).sum())}
        return float(vx), float(vy), float(wz), state, info

    def _start_step(self, pose, dt):
        """Straight up out of the start box, centred in the right-hand pit opening."""
        p = self.p
        x, y, th = pose
        dv = p.accel * dt
        self.v = min(self.v + dv, p.speed)
        err_x = self.start_x - x
        vb = np.array([max(-p.max_strafe, min(p.max_strafe, p.k_lat * err_x)), self.v])
        c, s = math.cos(th), math.sin(th)
        vx = c * vb[0] + s * vb[1]
        vy = -s * vb[0] + c * vb[1]
        wz = max(-p.max_wz, min(p.max_wz, p.k_head * wrap(math.pi / 2 - th)))
        if abs(wrap(math.pi / 2 - th)) > math.radians(25):
            vx, vy = 0.3 * vx, 0.3 * vy
        return float(vx), float(vy), float(wz), 'START', {'laps': 0, 'leg': 'start', 'offset': 0.0,
                                                          'progress': 0.0, 'obstacles': 0}

    def _leg(self, s):
        """0 right lane, 1 top crossing, 2 left lane, 3 pit-side crossing."""
        x, y = self.P[int(np.searchsorted(self.s, s)) % len(self.P)]
        if y > Y_TOP_LINE - 0.12:
            return 1
        if y < Y_BOT_LINE + 0.12:
            return 3
        return 0 if x > A.RAIL_X else 2


class SpeedGain:
    """Learns how fast the rover really moves per unit of command (uncalibrated, non-linear
    motors), from consecutive localized poses, so commands can be scaled to get the planned
    speed. scale = (translation gain, rotation gain); send planned / gain."""

    def __init__(self, rate=0.05, lo=0.4, hi=3.0):
        self.rate, self.lo, self.hi = rate, lo, hi
        self.t, self.r = 1.0, 1.0

    def update(self, before, after, planned, dt, lidar_fwd):
        if before is None or after is None or dt <= 0:
            return
        vx, vy, wz = planned
        moved = math.hypot(after[0] - before[0], after[1] - before[1]) / dt
        sp = math.hypot(vx, vy + wz * lidar_fwd)
        if sp > 0.06:
            g = moved / (sp / self.t)
            self.t += self.rate * (min(self.hi, max(self.lo, g)) - self.t)
        turned = abs(wrap(after[2] - before[2])) / dt
        if abs(wz) > 0.4:
            g = turned / (abs(wz) / self.r)
            self.r += self.rate * (min(self.hi, max(self.lo, g)) - self.r)

    def scale(self, cmd):
        vx, vy, wz = cmd
        return vx / self.t, vy / self.t, wz / self.r
