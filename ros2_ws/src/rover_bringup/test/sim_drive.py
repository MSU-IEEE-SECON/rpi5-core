"""Drive a simulated rover down a simulated hallway using hallway_logic.

No ROS needed. Useful for tuning Params before trying it on the robot.

    cd ros2_ws/src/rover_bringup
    python3 test/sim_drive.py                 # default course
    python3 test/sim_drive.py --course zigzag --noise 0.01 --start-yaw 20

Prints a trace and a summary. Exit code 1 if the rover hits something or
fails to reach the end of the hallway.
"""
import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)
from rover_bringup.hallway_logic import HallwayLogic, Params, scan_to_xy  # noqa: E402
import test_hallway_logic as T  # noqa: E402

COURSES = {
    # name: (half_width, obstacles [(x, y, w, h)], end_wall_x)
    'default': (0.8, [(2.0, -0.15, 0.3, 0.3), (4.5, 0.25, 0.3, 0.3), (7.0, 0.0, 0.3, 0.3)], 10.5),
    'zigzag': (0.9, [(1.8, 0.3, 0.3, 0.3), (3.4, -0.3, 0.3, 0.3), (5.0, 0.3, 0.3, 0.3),
                     (6.6, -0.3, 0.3, 0.3)], 9.0),
    'narrow': (0.55, [(2.5, 0.2, 0.2, 0.2)], 6.0),
    'empty': (0.8, [], 8.0),
    'doorways': (0.8, [(3.0, -0.1, 0.3, 0.3)], 8.0),
}


def build(course):
    hw, obstacles, end_x = COURSES[course]
    gap = (1.0, 2.0) if course == 'doorways' else None
    world = T.hallway(hw, -hw, -3, end_x + 1, left_gap=gap)
    world.append((end_x, -hw, end_x, hw))
    for o in obstacles:
        world += T.box(*o)
    return world, hw, end_x


def to_robot(world, x, y, th):
    c, s = math.cos(-th), math.sin(-th)
    out = []
    for x1, y1, x2, y2 in world:
        a = ((x1 - x) * c - (y1 - y) * s, (x1 - x) * s + (y1 - y) * c)
        b = ((x2 - x) * c - (y2 - y) * s, (x2 - x) * s + (y2 - y) * c)
        out.append((*a, *b))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--course', default='default', choices=sorted(COURSES))
    ap.add_argument('--noise', type=float, default=0.0, help='range noise std-dev (m)')
    ap.add_argument('--start-y', type=float, default=0.0)
    ap.add_argument('--start-yaw', type=float, default=0.0, help='degrees')
    ap.add_argument('--seconds', type=float, default=150.0)
    ap.add_argument('--quiet', action='store_true')
    args = ap.parse_args()

    world, hw, end_x = build(args.course)
    logic = HallwayLogic(Params())
    rng = random.Random(1)
    x, y, th = 0.0, args.start_y, math.radians(args.start_yaw)
    dt = 0.1
    robot_r = logic.p.robot_half_width
    closest = math.inf
    states = {}
    stopped_at = None

    for k in range(int(args.seconds / dt)):
        a, r = T.simulate(to_robot(world, x, y, th))
        if args.noise:
            r = [v + rng.gauss(0, args.noise) if math.isfinite(v) else v for v in r]
        pts = scan_to_xy(a, r, logic.p.min_range)
        d = logic.decide(pts)
        states[d.state] = states.get(d.state, 0) + 1
        closest = min(closest, min(math.hypot(px, py) for px, py in pts))
        if not args.quiet and k % 30 == 0:
            print(f"t={k * dt:5.1f}s x={x:5.2f} y={y:+.2f} yaw={math.degrees(th):+6.1f} "
                  f"{d.state:7s} v={d.linear:.2f} w={d.angular:+.2f}")
        if d.state == 'BLOCKED':
            stopped_at = x
            break
        th += d.angular * dt
        x += d.linear * math.cos(th) * dt
        y += d.linear * math.sin(th) * dt

    hit = closest < robot_r
    # it should stop roughly `lookahead` before the end wall
    reached_end = stopped_at is not None and stopped_at > end_x - logic.p.lookahead - 0.4
    print(f"\ncourse={args.course} noise={args.noise} start_y={args.start_y} "
          f"start_yaw={args.start_yaw}")
    print(f"states={states}")
    print(f"closest approach from LiDAR centre: {closest:.2f} m "
          f"(rover half-width {robot_r} m) -> {'COLLISION' if hit else 'ok'}")
    print(f"stopped at x={stopped_at if stopped_at is None else round(stopped_at, 2)} "
          f"(end wall at {end_x}) -> {'reached end' if reached_end else 'DID NOT REACH END'}")
    sys.exit(0 if (reached_end and not hit) else 1)


if __name__ == '__main__':
    main()
