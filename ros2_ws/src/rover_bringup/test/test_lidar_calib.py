"""Off-robot tests for the LiDAR calibration math (rover_bringup/lidar_calib.py)."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_bringup import lidar_calib as C  # noqa: E402


def fake_scan(yaw_deg, mirrored, blocked_rover=(), objects=(), room=2.5, rng=None):
    """A scan as the LiDAR would report it, for a rover whose true layout we know.

    yaw/mirrored = the TRUE mounting; blocked_rover = rover-angle ranges (lo, hi) hidden
    by the mount at 0.12 m; objects = [(rover_angle_deg, range_m, half_width_deg)].
    """
    rng = rng or random.Random(1)
    sign = -1.0 if mirrored else 1.0
    angles, ranges = [], []
    for i in range(720):
        raw = -180.0 + i * 0.5
        rov = C.wrap_deg(yaw_deg + sign * raw)
        r = room + rng.gauss(0, 0.01)
        if any(lo <= rov <= hi for lo, hi in blocked_rover):
            r = 0.12
        for oa, orr, ow in objects:
            if abs(C.wrap_deg(rov - oa)) <= ow:
                r = min(r, orr)
        angles.append(math.radians(raw))
        ranges.append(r)
    return C.scan_bins(angles, ranges)


def run_calibration(true_yaw, true_mirror, blocked):
    empty = [fake_scan(true_yaw, true_mirror, blocked) for _ in range(10)]
    base = C.baseline(empty)
    front = C.find_object(fake_scan(true_yaw, true_mirror, blocked, [(0.0, 0.5, 6)]), base)
    left = C.find_object(fake_scan(true_yaw, true_mirror, blocked, [(45.0, 0.5, 6)]), base)
    yaw, mirrored = C.orientation(front, left)
    arc = C.free_arc(C.blocked_bins(empty), yaw, mirrored)
    return yaw, mirrored, arc


def check(true_yaw, true_mirror):
    blocked = [(80.0, 180.0), (-180.0, -75.0)]            # sides+rear hidden by the mount
    yaw, mirrored, (left, right) = run_calibration(true_yaw, true_mirror, blocked)
    assert mirrored == true_mirror
    for rov in (0.0, 45.0, -60.0):                      # every direction maps back correctly
        sign = -1.0 if true_mirror else 1.0
        raw = sign * (rov - true_yaw)                    # what the LiDAR reports for that direction
        assert abs(C.wrap_deg(C.raw_to_rover(raw, yaw, mirrored) - rov)) < 2.0
    assert 74 <= left <= 79 and 69 <= right <= 74          # 80 / 75 deg minus the margin


def test_plain_mount():
    check(0.0, False)


def test_rotated_mount():
    check(90.0, False)
    check(-135.0, False)


def test_mirrored_mount():
    check(180.0, True)                                   # what the Nov 8 mount was
    check(37.0, True)


def test_left_too_close_to_front_is_rejected():
    try:
        C.orientation(10.0, 15.0)
    except ValueError:
        return
    raise AssertionError('should have refused')


def test_nothing_held_up_finds_nothing():
    empty = [fake_scan(0, False) for _ in range(5)]
    assert C.find_object(fake_scan(0, False), C.baseline(empty)) is None


def test_person_behind_the_box_does_not_confuse_it():
    empty = [fake_scan(0, False) for _ in range(5)]
    base = C.baseline(empty)
    # box 0.5 m ahead (narrow), person 1.0 m away a bit to the side (wider)
    scan = fake_scan(0, False, objects=[(0.0, 0.5, 5), (20.0, 1.0, 12)])
    assert abs(C.find_object(scan, base)) < 3.0
