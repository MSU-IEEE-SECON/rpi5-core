"""Off-robot tests for arena localization + Bug2 laps (rover_bringup/arena.py, bug2.py)."""
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.dirname(__file__))
from rover_bringup import arena as A  # noqa: E402
from rover_bringup.bug2 import Bug2Lap  # noqa: E402
import sim_arena as S  # noqa: E402


def scan_at(x, y, th, segs, noise=0.01, seed=0):
    ang = np.radians(np.arange(-83, 83.01, 0.5))
    r = S.ray_cast(x, y, ang + th, segs) + np.random.default_rng(seed).normal(0, noise, len(ang))
    ok = np.isfinite(r) & (r > 0.12)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], 1)


def test_localizer_tracks_from_a_rough_guess():
    true = (A.LANE_R + 0.02, 1.30, math.radians(92))
    pts = scan_at(*true, A.WALLS)
    loc = A.Localizer()
    pose, q = loc.icp(pts, (true[0] - 0.08, true[1] + 0.08, true[2] - math.radians(8)))
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.02
    assert abs(pose[2] - true[2]) < math.radians(2)
    assert q > 0.8


def test_global_search_finds_the_rover_in_the_right_lane():
    true = (A.LANE_R - 0.05, A.CORNER_LOW + 0.2, math.radians(85))
    pts = scan_at(*true, A.WALLS)
    pose, q = A.Localizer().global_search(pts, (A.LANE_R - 0.22, A.LANE_R + 0.22),
                                          (A.CORNER_LOW - 0.15, A.RAIL_Y0 + 0.5), (55, 125))
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.03


def test_toy_cars_do_not_throw_off_the_localizer():
    true = (A.LANE_R, 1.25, math.pi / 2)
    car = S.box(A.LANE_R + 0.05, 1.55, 0.09, 0.13)
    pts = scan_at(*true, A.WALLS + car)
    pose, q = A.Localizer().icp(pts, (true[0] + 0.05, true[1] - 0.05, true[2] + 0.05))
    assert math.hypot(pose[0] - true[0], pose[1] - true[1]) < 0.02


def test_planner_sidesteps_a_car_instead_of_driving_into_it():
    b = Bug2Lap()
    pose = (A.LANE_R, 1.20, math.pi / 2)
    b.reset(pose)
    car = np.array([[A.LANE_R + dx, 1.46] for dx in np.linspace(-0.04, 0.04, 9)])  # dead ahead
    vx, vy, wz, state, _ = b.step(pose, car)
    assert state in ('FOLLOW', 'SQUEEZE') and vx == 0.0 and abs(vy) > 0


def test_three_clean_laps_on_an_empty_track():
    os.environ['SIM_CARS'] = '0'
    try:
        r = S.run(seed=3)
    finally:
        del os.environ['SIM_CARS']
    assert r['result'] == 'OK' and r['laps'] == 3 and r['contacts'] == 0 and r['time'] < 150


def test_never_enters_the_pit():
    os.environ['SIM_CARS'] = '0'
    try:
        r = S.run(seed=5, gain=(1.3, 0.7, 1.3))      # badly calibrated motors
    finally:
        del os.environ['SIM_CARS']
    assert r['result'] != 'PIT'
