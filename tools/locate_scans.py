#!/usr/bin/env python3
"""Locate the rover on the SECON arena from LiDAR snapshots, and export everything.

    python3 tools/locate_scans.py scan_*.json --out lidar_export
    python3 tools/locate_scans.py scan_*.json --out lidar_export --truth truth.csv

For each snapshot (from `rover.ps1 snapshot`):
  * converts the raw scan to points in the rover frame (x forward, y left)
    using the LiDAR calibration saved inside the file (override with --yaw/--mirror)
  * searches the WHOLE racecourse for the pose whose known walls best match the scan
    (arena geometry in ros2_ws/src/rover_bringup/rover_bringup/arena.py)
  * reports the best pose + match quality, and how far the 2nd-best distinct pose is
    (the board is left/right symmetric to the LiDAR because the center rail is too low
    to see, so a mirrored pose can score nearly as well: flagged AMBIGUOUS)
Writes to --out: poses.csv, points_<scan>.csv (rover + board coords), <scan>.png
overlays, and README.txt explaining the formats. Optional truth.csv
(file,x_in,y_in,heading_deg measured on the board) adds the error columns.
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'rover_bringup'))
from rover_bringup import arena as A  # noqa: E402

IN = A.IN


def scan_points(d, yaw_deg=None, mirror=None, fov=None, min_r=0.15, use_scans=None):
    c = d.get('calib') or {}
    yaw = math.radians(c.get('laser_yaw_deg', 180.0) if yaw_deg is None else yaw_deg)
    mir = c.get('laser_mirror', False) if mirror is None else mirror
    fov = min(c.get('fov_left_deg', 83.0), c.get('fov_right_deg', 83.0)) if fov is None else fov
    scans = d['scans'] if use_scans is None else d['scans'][:use_scans]
    n = d['n']
    raw = d['angle_min'] + np.arange(n) * d['angle_increment']
    R = np.array([[np.nan if v is None else v for v in s] for s in scans], float)
    with np.errstate(all='ignore'):
        import warnings
        warnings.simplefilter('ignore', RuntimeWarning)
        r = np.nanmedian(R, axis=0)                 # median over the ~20 scans: less noise
    a = yaw + (-raw if mir else raw)
    a = np.arctan2(np.sin(a), np.cos(a))
    ok = np.isfinite(r) & (r >= min_r) & (r < 8.0) & (np.abs(a) <= math.radians(fov))
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1), (yaw, mir, fov)


def locate(pts, loc):
    """Whole-course search (coarse grid then ICP), keep the best few distinct poses."""
    L = pts[:: max(1, len(pts) // 70)]
    cands = []
    for th in np.radians(np.arange(0, 360, 6)):
        c, s = math.cos(th), math.sin(th)
        Rp = L @ np.array([[c, s], [-s, c]])
        for x in np.arange(A.X0 + 0.08, A.X1 - 0.07, 0.05):
            for y in np.arange(A.Y_SEP + 0.10, A.Y_TOP - 0.07, 0.05):
                _, _, dn = loc._match(Rp + [x, y])
                cands.append(((dn < 0.05).mean(), x, y, th))
    cands.sort(key=lambda t: -t[0])
    refined = []
    for score, x, y, th in cands[:40]:
        pose, q = loc.icp(pts, (x, y, th))
        if not (A.X0 < pose[0] < A.X1 and A.T < pose[1] < A.Y_TOP):
            continue
        q = q - 2.0 * A.wall_penalty(pts, pose)     # seeing through a known wall = wrong pose
        if all(math.hypot(pose[0] - p[0], pose[1] - p[1]) > 0.10 or
               abs(math.atan2(math.sin(pose[2] - p[2]), math.cos(pose[2] - p[2]))) > 0.3
               for p, _ in refined):
            refined.append((pose, q))
    refined.sort(key=lambda t: -t[1])
    return refined


def plot(path, pts, pose, alt, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(4.2, 8))
    for segs, col, lw in ((A.WALLS, 'k', 2), (A.RAIL, 'tab:gray', 1)):
        for x1, y1, x2, y2 in segs:
            ax.plot([x1 / IN, x2 / IN], [y1 / IN, y2 / IN], col, lw=lw)
    for pz, col, lab in ((alt, 'tab:orange', '2nd best'), (pose, 'tab:blue', 'best')):
        if pz is None:
            continue
        x, y, th = pz
        c, s = math.cos(th), math.sin(th)
        W = pts @ np.array([[c, s], [-s, c]]) + [x, y]
        ax.scatter(W[:, 0] / IN, W[:, 1] / IN, s=3, c=col, label=f'scan ({lab})')
        ax.arrow(x / IN, y / IN, 4 * c, 4 * s, head_width=1.2, color=col)
    ax.set_aspect('equal')
    ax.set_xlabel('x (in) across the board')
    ax.set_ylabel('y (in) from the pit end')
    ax.set_title(title, fontsize=8)
    ax.legend(fontsize=6, loc='lower left')
    plt.tight_layout()
    plt.savefig(path, dpi=100)
    plt.close(fig)


README = """LiDAR snapshots from the rover on the 2027 SECON arena
=====================================================

Board coordinates (all files): origin = board corner at the PIT end, left side when
looking from the pit toward the racecourse. x = across the board (0..48 in),
y = along the board (0..96 in; pit/course boundary at y = 28 in, far wall 95.25 in).
quality = fraction of points on a known wall, minus a penalty for points that would lie
BEHIND a known wall from that pose (impossible), which is what rules out mirror/rotated fits.
Heading: degrees counter-clockwise from +x (90 = facing up the board, away from the pit).
Pose = position of the LiDAR's centre (it sits ~5 in ahead of the rover's centre).

poses.csv            one row per snapshot: best pose (in + deg), match quality (fraction of
                     scan points within 4 cm of a known wall), 2nd-best distinct pose and its
                     quality, AMBIGUOUS flag. The center rail is too low for the LiDAR, so the
                     course looks left/right symmetric: if AMBIGUOUS, the 2nd pose (usually the
                     mirror image) fits almost as well; pick using which lane it was really in.
points_<scan>.csv    the scan (median of ~20 rotations): angle_deg (0 = rover front, + = left),
                     range_m, x_rover_m/y_rover_m (x forward, y left), x_board_in/y_board_in
                     (placed with the best pose).
<scan>.png           top-down: the scan drawn at the best pose (blue) and 2nd best (orange).
raw/                 the original snapshot JSON (raw driver angles/ranges + calibration used).
The center rail is in the map too (it shows up when the LiDAR sits low enough).
Geometry used: ros2_ws/src/rover_bringup/rover_bringup/arena.py (from the game manual drawing,
+-1 in).
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('scans', nargs='+')
    ap.add_argument('--out', default='lidar_export')
    ap.add_argument('--yaw', type=float)
    ap.add_argument('--mirror', choices=['true', 'false'])
    ap.add_argument('--truth', help='csv: file,x_in,y_in,heading_deg (measured)')
    a = ap.parse_args()
    os.makedirs(os.path.join(a.out, 'raw'), exist_ok=True)
    truth = {}
    if a.truth:
        for row in csv.DictReader(open(a.truth)):
            truth[os.path.basename(row['file'])] = row
    # match against the walls AND the center rail (the rail shows up when the LiDAR sits low
    # enough; if it doesn't, an unmatched rail costs nothing). The see-through check uses the
    # tall walls only, since the beam may pass over the rail.
    loc = A.Localizer(A.WALLS + A.RAIL)
    rows = []
    for path in a.scans:
        d = json.load(open(path))
        name = os.path.splitext(os.path.basename(path))[0]
        mir = None if a.mirror is None else a.mirror == 'true'
        pts, (yaw, mir_used, fov) = scan_points(d, a.yaw, mir)
        found = locate(pts, loc)
        best, q = found[0] if found else ((math.nan,) * 3, 0.0)
        alt, q2 = found[1] if len(found) > 1 else (None, 0.0)
        amb = alt is not None and q2 > q - 0.08
        q2 = max(q2, 0)
        row = {'file': os.path.basename(path), 'n_points': len(pts),
               'x_in': round(best[0] / IN, 1), 'y_in': round(best[1] / IN, 1),
               'heading_deg': round(math.degrees(best[2]) % 360, 1), 'quality': round(max(q, 0), 2),
               'alt_x_in': '' if alt is None else round(alt[0] / IN, 1),
               'alt_y_in': '' if alt is None else round(alt[1] / IN, 1),
               'alt_heading_deg': '' if alt is None else round(math.degrees(alt[2]) % 360, 1),
               'alt_quality': round(q2, 2), 'ambiguous': 'AMBIGUOUS' if amb else '',
               'lidar_yaw_deg': round(math.degrees(yaw), 1), 'mirrored': mir_used}
        t = truth.get(os.path.basename(path))
        if t:
            row['err_in'] = round(math.hypot(best[0] / IN - float(t['x_in']), best[1] / IN - float(t['y_in'])), 1)
            row['err_deg'] = round(abs((math.degrees(best[2]) - float(t['heading_deg']) + 180) % 360 - 180), 1)
        rows.append(row)
        # points file
        c, s = math.cos(best[2]), math.sin(best[2])
        W = pts @ np.array([[c, s], [-s, c]]) + [best[0], best[1]]
        with open(os.path.join(a.out, f'points_{name}.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['angle_deg', 'range_m', 'x_rover_m', 'y_rover_m', 'x_board_in', 'y_board_in'])
            for (px, py), (bx, by) in zip(pts, W):
                w.writerow([round(math.degrees(math.atan2(py, px)), 2), round(math.hypot(px, py), 4),
                            round(px, 4), round(py, 4), round(bx / IN, 2), round(by / IN, 2)])
        plot(os.path.join(a.out, f'{name}.png'), pts, best, alt if amb else None,
             f"{name}\nx={row['x_in']} in  y={row['y_in']} in  hdg={row['heading_deg']}  q={row['quality']} {row['ambiguous']}")
        with open(os.path.join(a.out, 'raw', os.path.basename(path)), 'w') as f:
            json.dump(d, f)
        print(f"{row['file']}: x={row['x_in']} in, y={row['y_in']} in, heading {row['heading_deg']} deg, "
              f"quality {row['quality']} {row['ambiguous']}"
              + (f"  (2nd: x={row['alt_x_in']} y={row['alt_y_in']} q={row['alt_quality']})" if amb else '')
              + (f"  err {row.get('err_in')} in / {row.get('err_deg')} deg" if t else ''))
    keys = list(rows[0].keys()) if rows else []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(a.out, 'poses.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(a.out, 'README.txt'), 'w') as f:
        f.write(README)
    print(f'Wrote {a.out}/ (poses.csv, points_*.csv, *.png, raw/, README.txt)')


if __name__ == '__main__':
    main()
