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
# Single-scan tests check the geometry, so smoothing is switched off here
# (instant steering + speed) and the older, stronger steering gains are used.
# Smoothing has its own tests at the bottom.
CENTRED = dict(laser_x=0.0, laser_y=0.0, stop_dist=0.30, slow_dist=1.0,
               lookahead=1.2, steer_lookahead=0.8, kp_heading=1.5, max_turn=0.8,
               turn_smoothing=1.0, accel=100.0, heading_deadband_deg=0.0)


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


# ---- smoothing (the "drunk driver" fixes) -----------------------------------

def face(x, half=0.15):
    """Flat face across the path at distance x."""
    return [(x, -half, x, half)]



def test_steering_is_smoothed_and_converges():
    logic = HallwayLogic(Params())          # real defaults, smoothing on
    seg = hallway(0.7, -0.9)                 # off-centre: wants to turn right
    a, r = simulate(seg)
    pts = scan_to_xy(a, r, logic.p.min_range)
    raw = logic._decide_raw(pts).angular
    ws = [logic.decide(pts).angular for _ in range(15)]
    assert abs(ws[0]) < abs(raw)            # first step is only part-way
    assert abs(ws[-1] - raw) < 0.02         # ...but it gets there
    assert all(w <= 0.0 for w in ws)        # never swings the wrong way


def test_one_bad_scan_barely_moves_the_steering():
    logic = HallwayLogic(Params())
    a, r = simulate(hallway(0.8, -0.8))
    good = scan_to_xy(a, r, logic.p.min_range)
    for _ in range(10):
        logic.decide(good)
    # one scan with a phantom object just right of centre
    bad = good + [(0.6, -0.05 + 0.01 * i) for i in range(5)]
    w = logic.decide(bad).angular
    assert abs(w) < 0.25


def test_small_heading_errors_are_ignored():
    logic = HallwayLogic(Params())
    d = logic._decide_raw(scan_to_xy(*simulate(hallway(0.81, -0.79)), logic.p.min_range))
    assert d.angular == 0.0


def test_speed_ramps_up_but_brakes_instantly():
    logic = HallwayLogic(Params())
    pts = scan_to_xy(*simulate(hallway(0.8, -0.8)), logic.p.min_range)
    v1 = logic.decide(pts).linear
    assert v1 <= logic.p.accel * logic.p.scan_dt + 1e-9
    for _ in range(20):
        v = logic.decide(pts).linear
    assert abs(v - logic.p.cruise_speed) < 1e-6
    wall = scan_to_xy(*simulate(hallway(0.8, -0.8) + face(0.25)), logic.p.min_range)
    assert logic.decide(wall).linear == 0.0


def test_stop_has_hysteresis():
    logic = HallwayLogic(Params())
    stop = logic.p.stop_dist
    close = scan_to_xy(*simulate(hallway(0.8, -0.8) + face(stop - 0.02)), logic.p.min_range)
    just_past = scan_to_xy(*simulate(hallway(0.8, -0.8) + face(stop + 0.04)), logic.p.min_range)
    assert logic.decide(close).state in ('PIVOT', 'BLOCKED')
    assert logic.decide(just_past).state in ('PIVOT', 'BLOCKED')   # still holding
    fresh = HallwayLogic(Params())
    assert fresh.decide(just_past).state not in ('PIVOT', 'BLOCKED')


def test_speckles_are_dropped():
    from rover_bringup.hallway_logic import drop_speckles
    line = [(1.0, 0.01 * i) for i in range(20)]
    lone = [(0.5, 0.0)]
    out = drop_speckles(line[:10] + lone + line[10:], 3)
    assert (0.5, 0.0) not in out and len(out) == 20
