import math

import numpy as np

from rover_bringup import camera_vision as cv
from rover_bringup.path_lap import PathLap, PathParams


def test_lut_round_trip():
    m = cv.CameraModel()
    i, j = np.nonzero(m.valid)
    sel = np.linspace(0, len(i) - 1, 50).astype(int)
    uv, front = m.project(m.lut[i[sel], j[sel]])
    assert front.all()
    assert np.allclose(uv[:, 0], j[sel] * 2 + 0.5, atol=0.05)
    assert np.allclose(uv[:, 1], i[sel] * 2 + 0.5, atol=0.05)


def test_floor_ahead_and_near_bottom():
    m = cv.CameraModel()
    bottom = m.lut[-1, m.w // 2]
    assert 0.15 < bottom[0] < 0.25 and abs(bottom[1]) < 0.06      # ~6 in ahead of the lens
    assert not m.valid[0].any()                                     # top row: beyond 1.5 m


def _frame(m, paint=None, car=None):
    Y = np.full((m.h, m.w), 130.0)
    U = np.full_like(Y, 128.0)
    V = np.full_like(Y, 128.0)
    if paint is not None:
        Y[paint] = 230
    if car is not None:
        U[car], V[car] = 90, 190
    return Y, U, V


def test_detects_paint_and_car_contact():
    m = cv.CameraModel()
    d = cv.Detector(m)
    paint = (slice(150, 200), slice(60, 70))
    car = (slice(120, 160), slice(180, 230))
    r = d.run(*_frame(m, paint, car))
    assert len(r['lines']) > 200
    obs = r['obstacles']
    assert len(obs) >= 40
    # floor contact = bottom row of the car blob
    want = m.lut[159, 205]
    assert np.min(np.hypot(*(obs - want).T)) < 0.01
    r = d.run(*_frame(m))
    assert len(r['lines']) == 0 and len(r['obstacles']) == 0


def test_camera_points_reach_the_planner_map():
    lap = PathLap(PathParams())
    pose = (0.98, 1.0, math.pi / 2)
    lap.reset(pose)
    car = np.array([[0.98, 1.4], [1.0, 1.4]])
    for _ in range(3):
        lap.add_camera(car, (0.98, 1.05), pose)
    obs = lap.obstacles()
    assert len(obs) and np.min(np.hypot(obs[:, 0] - 0.99, obs[:, 1] - 1.4)) < 0.03
