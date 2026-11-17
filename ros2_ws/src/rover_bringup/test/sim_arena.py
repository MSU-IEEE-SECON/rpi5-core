#!/usr/bin/env python3
"""Simulated laps on the 2027 SECON arena: localizer + Bug2, end to end.

    python3 test/sim_arena.py                       # one run, random cars
    python3 test/sim_arena.py --seed 7 --plot out.png
    python3 test/sim_arena.py --batch 40            # many runs, summary

Models: LiDAR at the front of a 30 x 28 cm rover, 0.5 deg beams, sees only
+-83 deg (mount), range noise; the center rail is INVISIBLE to it (too low);
3 toy cars in random quadrants; motors that move 0.7-1.3x what was asked
(uncalibrated) plus noise. Fails on: any body contact with walls/rail/cars,
entering the pit, or not finishing 3 laps in 3 minutes.
"""
import argparse
import math
import os
import random
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_bringup import arena as A  # noqa: E402
from rover_bringup.bug2 import Bug2Lap, LapParams  # noqa: E402
from rover_bringup.path_lap import PathLap, SpeedGain  # noqa: E402

IN = A.IN
LIDAR_FWD = 0.102         # LiDAR ahead of the rover centre (measured Nov 14)
BODY_HL, BODY_HW = 0.127, 0.13  # true body half length / width (10 x 10.2 in, measured Nov 14)
CAR = (0.09, 0.13)        # toy car footprint (m)


def box(cx, cy, w, h, ang=0.0):
    c, s = math.cos(ang), math.sin(ang)
    pts = [(cx + c * dx - s * dy, cy + s * dx + c * dy)
           for dx, dy in ((-w / 2, -h / 2), (w / 2, -h / 2), (w / 2, h / 2), (-w / 2, h / 2))]
    return [(*pts[i], *pts[(i + 1) % 4]) for i in range(4)]


def quadrants():
    qx = [(A.X0 + 2 * IN, A.X0 + 18 * IN), (A.X1 - 18 * IN, A.X1 - 2 * IN)]
    qy = [(A.RAIL_Y0, A.RAIL_Y0 + 16 * IN), (A.RAIL_Y1 - 16 * IN, A.RAIL_Y1)]
    return [(x, y) for x in qx for y in qy]


def place_cars(rng, n=3):
    n = int(os.environ.get('SIM_CARS', n))
    cars = []
    for (x0, x1), (y0, y1) in rng.sample(quadrants(), n):
        cx = rng.uniform(x0 + CAR[0] / 2, x1 - CAR[0] / 2)
        cy = rng.uniform(y0 + CAR[1] / 2, y1 - CAR[1] / 2)
        cars.append(box(cx, cy, CAR[0], CAR[1], rng.uniform(-0.15, 0.15)))
    return cars


def passable(car, need=2 * BODY_HW + 0.024):
    """Can a rover body get past this car without touching it (gap on either side)?"""
    xs = [c[0] for c in car]
    lo, hi = min(xs), max(xs)
    if hi < A.RAIL_X:           # left lane: wall .. rail
        return lo - A.X0 >= need or (A.RAIL_X - A.RAIL_HALF) - hi >= need
    return lo - (A.RAIL_X + A.RAIL_HALF) >= need or A.X1 - hi >= need


def pit_dividers():
    # rough: the 1x1 dividers between the pit boxes (seen through the openings)
    segs = []
    for xin in (6.5, 18.5, 21.5, 33.5):
        segs.append((xin * IN, A.T, xin * IN, A.T + 12 * IN))
    return segs


def ray_cast(ox, oy, angles, segs, max_r=12.0):
    S = np.array(segs, float)
    a, b = S[:, :2], S[:, 2:]
    e = b - a
    out = np.full(len(angles), np.inf)
    for i, ang in enumerate(angles):
        dx, dy = math.cos(ang), math.sin(ang)
        den = dx * e[:, 1] - dy * e[:, 0]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = ((a[:, 0] - ox) * e[:, 1] - (a[:, 1] - oy) * e[:, 0]) / den
            u = ((a[:, 0] - ox) * dy - (a[:, 1] - oy) * dx) / den
        ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
        if ok.any():
            out[i] = t[ok].min()
    out[out > max_r] = np.inf
    return out


def body_hits(pose, segs, hl=BODY_HL, hw=BODY_HW):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    for x1, y1, x2, y2 in segs:
        n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.01))
        for k in range(n + 1):
            px, py = x1 + (x2 - x1) * k / n - x, y1 + (y2 - y1) * k / n - y
            bx, by = px * c + py * s, -px * s + py * c
            if abs(bx) <= hl and abs(by) <= hw:
                return True
    return False


def in_pit(pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    for dx, dy in ((BODY_HL, BODY_HW), (BODY_HL, -BODY_HW), (-BODY_HL, BODY_HW), (-BODY_HL, -BODY_HW)):
        cy = y + s * dx + c * dy
        if cy < A.Y_SEP + A.T - 0.005:
            return True
    return False


def run(seed=0, noise=0.01, gain=None, params=None, plot=None, verbose=False, time_limit=180.0,
        planner_name=None, view_deg=None, lag=None):
    planner_name = planner_name or os.environ.get('SIM_PLANNER', 'path')
    view_deg = view_deg or float(os.environ.get('SIM_VIEW', 75))
    lag = float(os.environ.get('SIM_LAG', 0.25)) if lag is None else lag   # motor time constant (s)
    rng = random.Random(seed)
    cars = place_cars(rng)
    seen = A.WALLS + pit_dividers() + [s for c in cars for s in c]
    car_segs = [s for c in cars for s in c]
    hard = A.WALLS + A.RAIL
    ghi = float(os.environ.get('SIM_GAIN_HI', 1.3))
    g = gain or (rng.uniform(0.7, ghi), rng.uniform(0.7, ghi), rng.uniform(0.7, ghi))
    adapt = os.environ.get('SIM_ADAPT', '1') == '1' and planner_name == 'path'
    sg = SpeedGain()
    prev_est = None
    start_box = os.environ.get('SIM_START', 'box') == 'box'
    if start_box:          # in the start box, facing up toward the right-hand pit opening
        pose = [(A.SEP_X1 + A.X1) / 2 + rng.uniform(-0.02, 0.02), 0.50 + rng.uniform(-0.05, 0.05),
                math.pi / 2 + math.radians(rng.uniform(-5, 5))]
    else:
        pose = [A.LANE_R + rng.uniform(-0.03, 0.03), A.CORNER_LOW + rng.uniform(0, 0.05),
                math.pi / 2 + math.radians(rng.uniform(-8, 8))]
    left_pit = not start_box
    angles_local = np.radians(np.arange(-view_deg, view_deg + 0.01, 0.5))
    planner = PathLap() if planner_name == 'path' else Bug2Lap(params or LapParams())
    from rover_bringup.arena import Localizer
    loc = Localizer()
    dt, t = 0.1, 0.0
    trace, states = [], {}
    est_err = []
    cmd = (0.0, 0.0, 0.0)
    vel = [0.0, 0.0, 0.0]          # actual (lagged) velocity
    jerk = []
    result = 'TIMEOUT'
    contacts = 0
    car_contacts = 0
    while t < time_limit:
        lx = pose[0] + LIDAR_FWD * math.cos(pose[2])
        ly = pose[1] + LIDAR_FWD * math.sin(pose[2])
        r = ray_cast(lx, ly, angles_local + pose[2], seen)
        r = r + np.random.default_rng(int(t * 10) + seed * 7919).normal(0, noise, len(r))
        ok = np.isfinite(r) & (r > 0.12)
        local = np.stack([r[ok] * np.cos(angles_local[ok]), r[ok] * np.sin(angles_local[ok])], 1)
        if loc.pose is None:
            est, q = loc.global_search(local, (A.LANE_R - 0.12, A.X1 - 0.10),
                                       (0.30, A.RAIL_Y0 + 0.5), (60, 120))
            loc.pose = est
            planner.reset((est[0] - LIDAR_FWD * math.cos(est[2]),
                           est[1] - LIDAR_FWD * math.sin(est[2]), est[2]))
        else:
            vx, vy, wz = cmd
            before = loc.pose
            loc.update(local, (vx * dt, (vy + wz * LIDAR_FWD) * dt, wz * dt))
            if adapt and loc.bad_scans == 0 and loc.quality >= 0.6:
                sg.update(before, loc.pose, cmd, dt, LIDAR_FWD)
        ex, ey, eth = loc.pose
        c, s = math.cos(eth), math.sin(eth)
        world = local @ np.array([[c, s], [-s, c]]) + [ex, ey]
        centre = (ex - LIDAR_FWD * c, ey - LIDAR_FWD * s, eth)
        est_err.append(math.hypot(centre[0] - pose[0], centre[1] - pose[1]))
        vx, vy, wz, state, info = planner.step(centre, world)
        states[state] = states.get(state, 0) + 1
        cmd = (vx, vy, wz)
        if adapt:
            vx, vy, wz = sg.scale(cmd)      # what is actually sent to the motors
        if state == 'DONE':
            result = 'OK'
            break
        # true motion
        nr = np.random.default_rng(int(t * 10) + 13 + seed)
        a = 1.0 if lag <= 0 else min(1.0, dt / lag)
        new = [vel[0] + (vx * g[0] - vel[0]) * a, vel[1] + (vy * g[1] - vel[1]) * a,
               vel[2] + (wz * g[2] - vel[2]) * a]
        jerk.append(math.hypot(new[0] - vel[0], new[1] - vel[1]) / dt)
        vel = new
        tvx = vel[0] + nr.normal(0, 0.005)
        tvy = vel[1] + nr.normal(0, 0.005)
        twz = vel[2] + nr.normal(0, 0.02)
        th = pose[2]
        pose[0] += (tvx * math.cos(th) - tvy * math.sin(th)) * dt
        pose[1] += (tvx * math.sin(th) + tvy * math.cos(th)) * dt
        pose[2] = math.atan2(math.sin(th + twz * dt), math.cos(th + twz * dt))
        trace.append((pose[0], pose[1], state))
        if body_hits(pose, car_segs):
            car_contacts += 1          # cars get nudged (counted, costs the lap's bonus)
        if body_hits(pose, hard):
            contacts += 1
            if contacts > 30:          # stuck against something
                result = 'STUCK'
                break
        if not left_pit and pose[1] - BODY_HL > A.Y_SEP + A.T:
            left_pit = True                  # out of the start box: from now on the pit is off limits
        if left_pit and in_pit(pose):
            result = 'PIT'
            break
        t += dt
    out = {'seed': seed, 'result': result, 'time': round(t, 1), 'laps': planner.laps_done,
           'gain': tuple(round(v, 2) for v in g), 'max_loc_err_cm': round(100 * max(est_err), 1), 'contacts': contacts, 'car_steps': car_contacts,
           'learned_gain': (round(sg.t, 2), round(sg.r, 2)) if adapt else None,
           'passable': all(passable(c) for c in cars),
           'accel_p95': round(float(np.percentile(jerk, 95)), 2) if jerk else 0,
           'states': states}
    if plot:
        draw(plot, trace, cars, out)
    if verbose:
        print(out)
    return out


def draw(path, trace, cars, out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(4, 8))
    for segs, col, lw in ((A.WALLS, 'k', 2), (A.RAIL, 'tab:gray', 2), (pit_dividers(), 'k', 1),
                          (A.PIT_FENCE, 'tab:red', 1)):
        for x1, y1, x2, y2 in segs:
            ax.plot([x1, x2], [y1, y2], col, lw=lw)
    for c in cars:
        xs = [s[0] for s in c] + [c[0][0]]
        ys = [s[1] for s in c] + [c[0][1]]
        ax.fill(xs, ys, color='tab:orange')
    colors = {'PATH': 'tab:blue', 'BLOCKED': 'k', 'GOTO': 'tab:blue', 'FOLLOW': 'tab:green', 'TURN': 'tab:purple', 'LEAVE': 'tab:green',
              'CORNER': 'tab:purple', 'SQUEEZE': 'tab:red'}
    for st in colors:
        P = [(x, y) for x, y, s in trace if s == st]
        if P:
            ax.scatter(*zip(*P), s=1, c=colors[st], label=st)
    ax.set_aspect('equal')
    ax.set_title(f"{out['result']}  {out['laps']} laps  {out['time']} s", fontsize=9)
    ax.legend(fontsize=6, loc='lower left')
    plt.tight_layout()
    plt.savefig(path, dpi=110)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--batch', type=int, default=0)
    ap.add_argument('--noise', type=float, default=0.01)
    ap.add_argument('--plot')
    a = ap.parse_args()
    if a.batch:
        res = [run(seed=s, noise=a.noise) for s in range(a.batch)]
        for r in res:
            if r['result'] != 'OK' or r['contacts']:
                print(r)
        ok = [r for r in res if r['result'] == 'OK']
        print(f"{len(ok)}/{len(res)} OK", {k: sum(r['result'] == k for r in res)
                                           for k in ('STUCK', 'PIT', 'TIMEOUT')},
              'clean %d' % sum(r['result'] == 'OK' and not r['contacts'] for r in res),
              'mean time %.0f s' % (sum(r['time'] for r in ok) / max(1, len(ok))),
              'worst loc err %.1f cm' % max(r['max_loc_err_cm'] for r in res),
              '| no car touch %d' % sum(r['result'] == 'OK' and not r['car_steps'] for r in res),
              '| accel p95 %.2f m/s2' % (sum(r['accel_p95'] for r in res) / len(res)))
    else:
        run(seed=a.seed, noise=a.noise, plot=a.plot, verbose=True)
