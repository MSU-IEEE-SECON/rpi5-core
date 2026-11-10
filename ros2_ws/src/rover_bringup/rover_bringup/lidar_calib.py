"""LiDAR calibration: which way is the rover's front, and which arc can it see?

Pure functions (no ROS) so they can be tested off the robot, plus load/save of
~/rover_data/lidar_calib.json, which hallway_follower and scan_monitor read on start.

Rover angle convention: 0 deg = rover front, + = left (counter-clockwise).
    rover_angle = yaw + sign * raw_angle,   sign = -1 if mirrored else +1

The calibration file lives on the Pi (not in the repo), so a code deploy never
overwrites it. Re-run calibration whenever the LiDAR is moved or rotated.
"""
import json
import math
import os

DEFAULT_PATH = os.path.join(os.environ.get('ROVER_DATA', '~/rover_data'), 'lidar_calib.json')
BIN_DEG = 1.0
N_BINS = int(round(360.0 / BIN_DEG))


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def to_bin(angle_rad):
    return int(math.floor(math.degrees(angle_rad) % 360.0 / BIN_DEG)) % N_BINS


def bin_centre_deg(b):
    return wrap_deg((b + 0.5) * BIN_DEG)


def scan_bins(angles, ranges, rmin=0.05, rmax=12.0):
    """One scan -> {bin: closest range}."""
    out = {}
    for a, r in zip(angles, ranges):
        if math.isfinite(r) and rmin <= r <= rmax:
            b = to_bin(a)
            if r < out.get(b, math.inf):
                out[b] = r
    return out


def baseline(scans):
    """Median range per bin over several scans (bins seen in < half the scans are left out)."""
    per = {}
    for s in scans:
        for b, r in s.items():
            per.setdefault(b, []).append(r)
    need = max(1, len(scans) // 2)
    return {b: sorted(rs)[len(rs) // 2] for b, rs in per.items() if len(rs) >= need}


def find_object(scan, base, min_drop=0.15, near=0.2, far=1.5, min_bins=2):
    """Where is the thing the person is holding up? Returns raw angle (deg) or None.

    Looks for bins that got clearly closer than the empty-room baseline, groups
    neighbouring bins, and returns the centre of the closest group.
    """
    hits = sorted(b for b, r in scan.items()
                  if near <= r <= far and r < base.get(b, math.inf) - min_drop)
    if not hits:
        return None
    groups, cur = [], [hits[0]]
    for b in hits[1:]:
        if b - cur[-1] <= 2:
            cur.append(b)
        else:
            groups.append(cur)
            cur = [b]
    groups.append(cur)
    if len(groups) > 1 and groups[0][0] <= 1 and groups[-1][-1] >= N_BINS - 2:
        groups[0] = groups.pop() + groups[0]          # join a group that wraps through 0
    # the held-up object is the CLOSEST group (the person holding it stands further back)
    groups = [g for g in groups if len(g) >= min_bins]
    if not groups:
        return None
    best = min(groups, key=lambda g: min(scan[b] for b in g))
    x = sum(math.cos(math.radians(bin_centre_deg(b))) for b in best)
    y = sum(math.sin(math.radians(bin_centre_deg(b))) for b in best)
    return math.degrees(math.atan2(y, x))


def orientation(raw_front_deg, raw_left_deg):
    """From where FRONT and front-LEFT objects showed up -> (yaw_deg, mirrored)."""
    d = wrap_deg(raw_left_deg - raw_front_deg)
    if abs(d) < 15.0 or abs(d) > 165.0:
        raise ValueError(f'front and left readings are only {d:+.0f} deg apart: '
                         'put the second object clearly to the LEFT side')
    mirrored = d < 0.0                    # left should be counter-clockwise (+) of front
    sign = -1.0 if mirrored else 1.0
    yaw = wrap_deg(-sign * raw_front_deg)  # rover front (0) = yaw + sign * raw_front
    return yaw, mirrored


def raw_to_rover(raw_deg, yaw_deg, mirrored):
    return wrap_deg(yaw_deg + (-1.0 if mirrored else 1.0) * raw_deg)


def blocked_bins(scans, block_range=0.30, frac=0.7):
    """Bins where the rover/mount itself sits: a close return in most scans."""
    count = {}
    for s in scans:
        for b, r in s.items():
            if r <= block_range:
                count[b] = count.get(b, 0) + 1
    need = frac * len(scans)
    return {b for b, c in count.items() if c >= need}


def free_arc(blocked_raw_bins, yaw_deg, mirrored, margin_deg=3.0, limit_deg=120.0):
    """Walk left and right from the rover's front until hitting the rover/mount.

    Returns (left_deg, right_deg): usable angles each side of straight ahead.
    """
    blocked_rover = {int(math.floor(raw_to_rover(bin_centre_deg(b), yaw_deg, mirrored)))
                     for b in blocked_raw_bins}

    def walk(step):
        a = 0
        while abs(a) < limit_deg:
            nxt = a + step
            # a single blocked degree could be a cable; need 2 in a row to stop
            if nxt in blocked_rover and (nxt + step) in blocked_rover:
                break
            a = nxt
        return max(0.0, abs(a) - margin_deg)
    return walk(+1), walk(-1)


def save(calib, path=DEFAULT_PATH):
    path = os.path.expanduser(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(calib, f, indent=1)
    return path


def load(path=DEFAULT_PATH):
    """Returns the calibration dict, or None if there isn't one."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)
