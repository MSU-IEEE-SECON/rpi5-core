"""2027 SECON arena geometry + LiDAR localization (no ROS, testable off-robot).

Board coordinates, metres: origin = board corner at the PIT end, left side
(looking from the pit toward the course). x across (0 .. 1.219), y along the
board (0 = pit end, 2.438 = far end). From the manual's field drawing:

    y 2.419 ┌──────────── top wall ────────────┐
            │                                  │
            │      center rail x=0.61,         │   racecourse
            │      y 1.108 .. 2.022            │   (laps go around the rail)
            │      (1.5 in tall: the LiDAR     │
            │       does NOT see it)           │
    y 0.73  ├──open──┤ separator ├────open─────┤   openings = pit entrances
            │  pit boxes             start box │   pit area (avoid)
    y 0.019 └──────────────────────────────────┘

Everything here is within the manual's +-1 in tolerance at best; the
localizer only needs the walls roughly right.
"""
import math

import numpy as np

IN = 0.0254
W = 48 * IN                    # board width
L = 96 * IN                    # board length
T = 0.75 * IN                  # wall thickness (1x lumber)
X0, X1 = T, W - T              # inner faces of the side walls
Y_TOP = L - T                  # inner face of the top wall
Y_SEP = 28 * IN                # pit / course boundary
SEP_X0, SEP_X1 = 15 * IN, W - 15.02 * IN   # separator wall piece between the openings
RAIL_X = W / 2
RAIL_Y0 = Y_SEP + (68 - 36) / 2 * IN
RAIL_Y1 = RAIL_Y0 + 36 * IN
RAIL_HALF = 0.375 * IN

# Walls the LiDAR can see (tall enough). Segments (x1, y1, x2, y2).
WALLS = [
    (X0, T, X0, Y_TOP), (X1, T, X1, Y_TOP),          # side walls (full length)
    (X0, Y_TOP, X1, Y_TOP),                          # far end wall
    (X0, T, X1, T),                                  # pit end wall
    (SEP_X0, Y_SEP, SEP_X1, Y_SEP),                  # separator piece, pit side
    (SEP_X0, Y_SEP + T, SEP_X1, Y_SEP + T),          # separator piece, course side
    (SEP_X0, Y_SEP, SEP_X0, Y_SEP + T), (SEP_X1, Y_SEP, SEP_X1, Y_SEP + T),
]
# 45-degree board in the bottom-left corner of the course, from the left wall down to the
# separator's left end. Not in the manual drawing as a wall; measured from a LiDAR
# snapshot on the real board (Nov 13, +-1 in). It closes the left pit opening.
CHAMFERS = [(X0, 40.3 * IN, 15.0 * IN, 26.5 * IN)]
WALLS = WALLS + CHAMFERS
# Known obstacles the LiDAR can NOT see -> fed to the planner as virtual points.
RAIL = [(RAIL_X - RAIL_HALF, RAIL_Y0, RAIL_X - RAIL_HALF, RAIL_Y1),
        (RAIL_X + RAIL_HALF, RAIL_Y0, RAIL_X + RAIL_HALF, RAIL_Y1),
        (RAIL_X - RAIL_HALF, RAIL_Y0, RAIL_X + RAIL_HALF, RAIL_Y0),
        (RAIL_X - RAIL_HALF, RAIL_Y1, RAIL_X + RAIL_HALF, RAIL_Y1)]
# Don't go into the pit: a virtual fence across both openings.
PIT_FENCE = [(X0, Y_SEP + T, SEP_X0, Y_SEP + T), (SEP_X1, Y_SEP + T, X1, Y_SEP + T)]

LANE_R = (RAIL_X + RAIL_HALF + X1) / 2     # right lane centre (x)
LANE_L = (X0 + RAIL_X - RAIL_HALF) / 2     # left lane centre
# Counter-clockwise lap (up the right lane, rail on the left), rover-centre waypoints.
# Corners: where the rover turns in place. Turning sweeps a ~0.20 m radius
# (30 x 25 cm body), and the crossings must clear the rail ends by half a body
# width (0.14 m):  bottom gap 0.36 m (fence .. rail end), top gap 0.40 m.
CORNER_LOW = Y_SEP + T + 0.205             # 0.21 from the pit fence, 2.6 cm past the rail end
CORNER_HIGH = Y_TOP - 0.229                # 0.23 from the top wall, 2.9 cm past the rail end
# Bottom-left turn moved right of the lane centre: 0.23 m from the corner board and the
# separator (turning sweep 0.21 m), legs in and out clear the rail end by 2.5 cm.
CORNER_LOW_LEFT = (0.42, 0.96)
LAP = [(LANE_R, CORNER_HIGH), (LANE_L, CORNER_HIGH), CORNER_LOW_LEFT, (LANE_R, CORNER_LOW)]


def sample_segments(segs, step=0.02):
    pts = []
    for x1, y1, x2, y2 in segs:
        n = max(1, int(math.hypot(x2 - x1, y2 - y1) / step))
        pts += [(x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(n + 1)]
    return np.array(pts)


RAIL_PTS = sample_segments(RAIL)
FENCE_PTS = sample_segments(PIT_FENCE)
CHAMFER_PTS = sample_segments(CHAMFERS)


class Localizer:
    """Tracks the LiDAR's pose (x, y, heading) on the arena by matching each scan
    to the known walls (point-to-line ICP), starting from the last pose moved by
    the commanded velocity. Toy cars and other unknown things are rejected as
    outliers (too far from any wall)."""

    def __init__(self, walls=WALLS):
        s = np.array(walls, dtype=float)
        self.a = s[:, :2]
        d = s[:, 2:] - s[:, :2]
        self.len2 = np.maximum((d ** 2).sum(1), 1e-12)
        self.d = d
        self.pose = None
        self.quality = 0.0     # fraction of scan points that matched a wall
        self.min_quality = 0.35
        self.bad_scans = 0     # scans in a row we didn't trust

    def _match(self, P):
        """P: (N,2) world points -> nearest point on any wall, its normal, distance."""
        ap = P[:, None, :] - self.a[None, :, :]                       # N,S,2
        t = np.clip((ap * self.d[None]).sum(2) / self.len2[None], 0, 1)
        q = self.a[None] + t[..., None] * self.d[None]                 # N,S,2
        dist = np.linalg.norm(P[:, None, :] - q, axis=2)
        j = dist.argmin(1)
        i = np.arange(len(P))
        qn = q[i, j]
        diff = P - qn
        dn = np.linalg.norm(diff, axis=1)
        seg = self.d[j]
        nrm = np.stack([-seg[:, 1], seg[:, 0]], 1) / np.sqrt(self.len2[j])[:, None]
        return qn, nrm, dn

    def icp(self, local_pts, pose, iters=10, gates=(0.15, 0.10, 0.06)):
        x, y, th = pose
        L_ = np.asarray(local_pts, float)
        if len(L_) < 10:
            return pose, 0.0
        for k in range(iters):
            gate = gates[min(k, len(gates) - 1)]
            c, s = math.cos(th), math.sin(th)
            P = L_ @ np.array([[c, s], [-s, c]]) + [x, y]
            q, n, dn = self._match(P)
            ok = dn < gate
            if ok.sum() < 8:
                return (x, y, th), float(ok.mean())
            Pk, qk, nk = P[ok], q[ok], n[ok]
            r = ((Pk - qk) * nk).sum(1)
            rel = Pk - [x, y]
            J = np.stack([nk[:, 0], nk[:, 1], (nk * np.stack([-rel[:, 1], rel[:, 0]], 1)).sum(1)], 1)
            H = J.T @ J + np.diag([1e-4, 1e-4, 1e-4])
            dx, dy, dth = np.linalg.solve(H, -J.T @ r)
            x, y, th = x + dx, y + dy, th + dth
            if abs(dx) + abs(dy) < 1e-4 and abs(dth) < 1e-4:
                break
        c, s = math.cos(th), math.sin(th)
        P = L_ @ np.array([[c, s], [-s, c]]) + [x, y]
        _, _, dn = self._match(P)
        return (x, y, math.atan2(math.sin(th), math.cos(th))), float((dn < 0.04).mean())

    def global_search(self, local_pts, x_range, y_range, th_range, step=0.05, th_step=6.0):
        """Find the pose from scratch inside a box (used once at start / on Y press)."""
        L_ = np.asarray(local_pts, float).reshape(-1, 2)
        if len(L_) < 10:
            return None, 0.0
        if len(L_) > 60:
            L_ = L_[:: len(L_) // 60]
        best = (-1, None)
        for th in np.radians(np.arange(th_range[0], th_range[1] + 1e-6, th_step)):
            c, s = math.cos(th), math.sin(th)
            R = L_ @ np.array([[c, s], [-s, c]])
            for x in np.arange(x_range[0], x_range[1] + 1e-6, step):
                for y in np.arange(y_range[0], y_range[1] + 1e-6, step):
                    _, _, dn = self._match(R + [x, y])
                    score = (dn < 0.05).mean()
                    if score > best[0]:
                        best = (score, (x, y, th))
        pose, q = self.icp(L_, best[1])
        return pose, q

    def update(self, local_pts, odom_delta=None):
        """odom_delta = (dx_fwd, dy_left, dth) the rover was told to move since last scan."""
        if self.pose is None:
            return None
        x, y, th = self.pose
        if odom_delta is not None:
            f, l, dth = odom_delta
            x += f * math.cos(th) - l * math.sin(th)
            y += f * math.sin(th) + l * math.cos(th)
            th += dth
        pose, q = self.icp(local_pts, (x, y, th))
        jump = math.hypot(pose[0] - x, pose[1] - y)
        if q < self.min_quality or jump > 0.12 or abs(math.atan2(math.sin(pose[2] - th),
                                                                 math.cos(pose[2] - th))) > 0.35:
            # bad match (blocked view, lots of clutter): trust the motion estimate this scan
            self.pose, self.quality = (x, y, th), q
            self.bad_scans += 1
        else:
            self.pose, self.quality = pose, q
            self.bad_scans = 0
        return self.pose


def ray_cast(ox, oy, angles, segs=WALLS, max_r=12.0):
    """Distance from (ox, oy) along each angle (rad, board frame) to the first segment."""
    S = np.array(segs, float)
    a, e = S[:, :2], S[:, 2:] - S[:, :2]
    d = np.stack([np.cos(angles), np.sin(angles)], 1)                    # R,2
    den = d[:, None, 0] * e[None, :, 1] - d[:, None, 1] * e[None, :, 0]  # R,S
    ax, ay = a[None, :, 0] - ox, a[None, :, 1] - oy
    with np.errstate(divide='ignore', invalid='ignore'):
        t = (ax * e[None, :, 1] - ay * e[None, :, 0]) / den
        u = (ax * d[:, None, 1] - ay * d[:, None, 0]) / den
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(ok, t, np.inf).min(1)
    t[t > max_r] = np.inf
    return t


def see_through(local_pts, pose, segs=WALLS):
    """Fraction of scan points that would be BEHIND a known wall from this pose
    (impossible: the wall would have blocked them). Tells mirror/rotated poses apart."""
    x, y, th = pose
    L = np.asarray(local_pts, float)
    r = np.hypot(L[:, 0], L[:, 1])
    ang = np.arctan2(L[:, 1], L[:, 0]) + th
    exp = ray_cast(x, y, ang, segs)
    return float((r > exp + 0.06).mean())


SEPARATOR = [s for s in WALLS if min(s[1], s[3]) >= Y_SEP - 0.01 and max(s[1], s[3]) <= Y_SEP + T + 0.01]
OUTER_WALLS = [s for s in WALLS if s not in SEPARATOR]


def wall_penalty(local_pts, pose):
    """How impossible is this pose? Fraction of points that would lie behind a wall.

    The outer walls always block. The separator in front of the pit sometimes doesn't
    quite (the scan plane can skim over its top from some spots), so seeing past it is
    only held against a pose in proportion to how little of it shows up: a pose that
    sees *through* the separator without a single hit on it is implausible; one that
    sees most of it and past a few inches is fine.
    """
    x, y, th = pose
    L = np.asarray(local_pts, float)
    r = np.hypot(L[:, 0], L[:, 1])
    ang = np.arctan2(L[:, 1], L[:, 0]) + th
    outer = r > ray_cast(x, y, ang, OUTER_WALLS) + 0.06
    to_sep = ray_cast(x, y, ang, SEPARATOR)
    past_sep = (r > to_sep + 0.06) & ~outer
    on_sep = np.abs(r - to_sep) < 0.04
    crossing = past_sep.sum() + on_sep.sum()
    seen_frac = on_sep.sum() / crossing if crossing else 1.0
    return float(outer.mean() + past_sep.mean() * (1.0 - seen_frac))
