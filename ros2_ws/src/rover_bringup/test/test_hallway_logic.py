"""Tests for hallway_logic using a tiny 2D ray-casting simulator.

Run on the laptop or Pi (no ROS needed):
    cd ros2_ws/src/rover_bringup && python3 -m pytest test/test_hallway_logic.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_bringup.hallway_logic import HallwayLogic, Params, scan_to_xy  # noqa: E402

N_BEAMS = 720


def ray_segment(ox, oy, dx, dy, x1, y1, x2, y2):
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    if abs(den) < 1e-12:
        return None
    t = ((x1 - ox) * ey - (y1 - oy) * ex) / den
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / den
    if t > 0 and 0 <= u <= 1:
        return t
    return None


def box(cx, cy, w, h):
    x0, x1, y0, y1 = cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def simulate(segments, max_range=12.0):
    """Robot at origin facing +x. Returns (angles, ranges)."""
    angles, ranges = [], []
    for i in range(N_BEAMS):
        a = -math.pi + i * 2 * math.pi / N_BEAMS
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for s in segments:
            t = ray_segment(0, 0, dx, dy, *s)
            if t is not None and t < best:
                best = t
        angles.append(a)
        ranges.append(best if best <= max_range else math.inf)
    return angles, ranges


def hallway(left_y=0.8, right_y=-0.8, x0=-3, x1=6, left_gap=None):
    """Two long walls. left_gap=(xa, xb) cuts a doorway into the left wall."""
    segs = [(x0, right_y, x1, right_y)]
    if left_gap:
        segs += [(x0, left_y, left_gap[0], left_y), (left_gap[1], left_y, x1, left_y)]
    else:
        segs.append((x0, left_y, x1, left_y))
    return segs


# The geometry tests below place the LiDAR at the rover's centre, with the
# original distances, so they test the behaviour itself rather than our mount.
CENTRED = dict(laser_x=0.0, laser_y=0.0, stop_dist=0.30, slow_dist=1.0,
               lookahead=1.2, steer_lookahead=0.8)


def centred(**kw):
    return Params(**{**CENTRED, **kw})


def run(segments, logic=None):
    logic = logic or HallwayLogic(centred())
    a, r = simulate(segments)
    return logic.decide(scan_to_xy(a, r, logic.p.min_range))


def test_centered_goes_straight():
    d = run(hallway(0.8, -0.8))
    assert d.state == 'FOLLOW'
    assert abs(d.angular) < 0.05
    assert d.linear > 0.12


def test_close_to_right_wall_turns_left():
    d = run(hallway(1.1, -0.5))          # robot is 0.3 m right of centre
    assert d.state == 'FOLLOW'
    assert d.angular > 0.2               # +angular = turn left (CCW)


def test_close_to_left_wall_turns_right():
    d = run(hallway(0.5, -1.1))
    assert d.angular < -0.2


def test_obstacle_slightly_right_goes_left():
    d = run(hallway() + box(0.9, -0.1, 0.3, 0.3))
    assert d.state == 'AVOID'
    assert d.angular > 0.2
    assert 0 < d.linear < centred().cruise_speed


def test_obstacle_slightly_left_goes_right():
    d = run(hallway() + box(0.9, 0.1, 0.3, 0.3))
    assert d.state == 'AVOID'
    assert d.angular < -0.2


def test_obstacle_dead_centre_picks_a_side_and_sticks():
    logic = HallwayLogic(centred())
    first = run(hallway() + box(0.9, 0.0, 0.3, 0.3), logic)
    assert first.state == 'AVOID' and abs(first.angular) > 0.2
    # slightly favour the other side; hysteresis should keep the first choice
    nudge = 0.03 if first.angular > 0 else -0.03
    second = run(hallway() + box(0.9, nudge, 0.3, 0.3), logic)
    assert math.copysign(1, second.angular) == math.copysign(1, first.angular)


def test_very_close_obstacle_pivots_without_driving():
    d = run(hallway() + box(0.25, -0.05, 0.1, 0.3))
    assert d.state == 'PIVOT'
    assert d.linear == 0.0 and d.angular > 0


def test_end_of_hallway_blocks():
    d = run(hallway() + [(0.6, -0.8, 0.6, 0.8)])   # wall straight across
    assert d.state == 'BLOCKED'
    assert d.linear == 0.0 and d.angular == 0.0


def test_too_narrow_gap_blocks():
    # obstacle leaves only 0.2 m each side: rover needs 0.46 m
    d = run(hallway(0.4, -0.4) + box(0.8, 0.0, 0.3, 0.4))
    assert d.state == 'BLOCKED'


def test_doorway_on_left_does_not_veer():
    d = run(hallway(0.8, -0.8, left_gap=(-1.0, 1.0)))
    assert abs(d.angular) < 0.15


def test_no_data_stops():
    logic = HallwayLogic(centred())
    d = logic.decide([])
    assert d.state == 'NO_DATA' and d.linear == 0.0 and d.angular == 0.0


def test_rear_half_is_ignored():
    """Clutter behind the LiDAR (robot body on our mount) must not affect anything."""
    clean = run(hallway(0.8, -0.8))
    behind = run(hallway(0.8, -0.8) + box(-0.15, 0.0, 0.1, 0.3) + box(-0.4, 0.5, 0.2, 0.2))
    assert behind.state == 'FOLLOW'
    assert abs(behind.angular - clean.angular) < 0.05
    assert abs(behind.linear - clean.linear) < 0.01


def test_full_circle_option_still_works():
    logic = HallwayLogic(centred(fov_deg=360.0))
    d = run(hallway(0.8, -0.8), logic)
    assert d.state == 'FOLLOW' and abs(d.angular) < 0.05


def test_offset_lidar_still_centres_the_rover():
    """LiDAR mounted 10 cm left of the rover's centre, rover itself centred:
    the LiDAR sees the left wall 10 cm closer, but the rover should go straight."""
    logic = HallwayLogic(centred(laser_y=0.10))
    d = run(hallway(0.7, -0.9), logic)   # walls as seen from the LiDAR
    assert d.state == 'FOLLOW' and abs(d.angular) < 0.05
    # without the offset setting it would wrongly steer right (away from the closer wall)
    d0 = run(hallway(0.7, -0.9))
    assert d0.angular < -0.1
