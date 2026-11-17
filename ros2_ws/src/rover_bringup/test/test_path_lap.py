"""Racing-line planner: the loop clears every wall, and the plan steps around a car."""
import math

import numpy as np

from rover_bringup import arena as A
from rover_bringup import path_lap as PL


def test_racing_line_clears_static_geometry_everywhere():
    pl = PL.PathLap()
    j0 = int(np.argmin(np.abs(pl.offsets)))
    assert pl.static_ok[:, j0].all()                 # centre line is always legal
    assert 3.0 < pl.L < 4.0                          # one lap is ~3.5 m


def test_plan_shifts_around_a_car_and_back():
    pl = PL.PathLap()
    # a car on the rail side of the right lane, 0.5 m ahead of a rover heading up the lane
    car = np.array([[x, y] for x in np.arange(0.70, 0.80, 0.01) for y in np.arange(1.50, 1.63, 0.01)])
    pose = (A.LANE_R, 1.00, math.pi / 2)
    pl.reset(pose)
    for _ in range(4):
        pl._remember(car, (A.LANE_R, 1.13))
    vx, vy, wz, state, info = pl.step(pose, car)
    d = pl.plan_d
    assert state in ('PATH', 'SQUEEZE')
    beside = d[int(0.55 / pl.p.plan_ds)]             # planned offset next to the car
    assert beside < -0.05                            # moved right (away from the rail)
    assert abs(d[-1]) < 0.03                         # and back to the line afterwards


def test_speed_gain_learns_fast_motors():
    g = PL.SpeedGain(rate=0.2)
    pose = (0.9, 1.0, math.pi / 2)
    for _ in range(60):                              # rover really goes 1.9x what is sent
        sent = g.scale((0.2, 0.0, 0.0))[0]
        new = (pose[0], pose[1] + 1.9 * sent * 0.1, pose[2])
        g.update(pose, new, (0.2, 0.0, 0.0), 0.1, 0.13)
        pose = new
    assert abs(g.t - 1.9) < 0.1


def test_own_body_is_never_mapped():
    pl = PL.PathLap()
    pose = (A.LANE_R, 1.30, math.pi / 2)
    pl.reset(pose)
    wheels = np.array([[A.LANE_R + sx * 0.12, 1.30 + fy] for sx in (-1, 1) for fy in (-0.07, 0.06)])
    for _ in range(5):
        pl._remember(wheels, (A.LANE_R, 1.40), pose)
    assert len(pl.obstacles()) == 0
