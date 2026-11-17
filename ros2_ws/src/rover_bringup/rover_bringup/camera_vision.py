"""Webcam -> floor: white lane lines and coloured/dark things (cars) on the board.

No ROS and no OpenCV, just numpy, so it can be tested off the robot.

Geometry: a pinhole camera at a known spot on the rover, looking down at the flat board.
Every pixel below the horizon maps to one point on the floor, in the ROVER frame
(x forward, y left, metres, origin = rover centre). The mapping is computed once
(`CameraModel.lut`). Things standing up (cars, walls) map to points behind them, so for
obstacles only the lowest pixel of each blob column is used (where it touches the floor).

Calibration (Nov 14, start box, Logitech C270 640x480): fitted to the right wall's base
and top edge, the far wall and a lane line, with the LiDAR pose known. Error about
0.7 in RMS out to 6 ft.
"""
import json
import math
import os
import warnings

import numpy as np

IN = 0.0254
DEFAULT_CALIB = {
    'width': 640, 'height': 480,
    'f': 700.0, 'cx': 320.0, 'cy': 240.0,
    'cam_fwd': 2.0 * IN,      # from the rover centre (10 in body, camera 3 in from the front)
    'cam_left': -2.0 * IN,    # 2 in right of centre
    'cam_z': 5.0 * IN,
    'yaw_deg': 4.0,           # + = camera points left of the rover's heading
    'pitch_deg': 21.8,        # down
    'roll_deg': -0.2,
    'max_range': 1.5,         # m from the camera: farther floor is too coarse to use
}
CALIB_PATH = os.path.expanduser('~/rover_data/camera_calib.json')


def load_calib(path=CALIB_PATH):
    c = dict(DEFAULT_CALIB)
    try:
        with open(path) as f:
            c.update(json.load(f))
    except (OSError, ValueError):
        pass
    return c


class CameraModel:
    def __init__(self, calib=None, step=2):
        """step: work on every `step`-th pixel (2 -> 320x240 from 640x480)."""
        c = dict(DEFAULT_CALIB, **(calib or {}))
        self.c, self.step = c, step
        self.w, self.h = c['width'] // step, c['height'] // step
        yaw, pitch, roll = (math.radians(c[k]) for k in ('yaw_deg', 'pitch_deg', 'roll_deg'))
        fwd = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
        up = np.array([0.0, 0.0, 1.0])
        f2 = fwd * math.cos(pitch) - up * math.sin(pitch)
        u2 = up * math.cos(pitch) + fwd * math.sin(pitch)
        r3 = right * math.cos(roll) + u2 * math.sin(roll)
        u3 = u2 * math.cos(roll) - right * math.sin(roll)
        self.axes = (f2, r3, u3)
        self.pos = np.array([c['cam_fwd'], c['cam_left'], c['cam_z']])
        u = (np.arange(self.w) * step + (step - 1) / 2.0 - c['cx']) / c['f']
        v = (np.arange(self.h) * step + (step - 1) / 2.0 - c['cy']) / c['f']
        U, V = np.meshgrid(u, v)
        d = f2[None, None] + r3[None, None] * U[..., None] - u3[None, None] * V[..., None]
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -self.pos[2] / d[..., 2]
        gx = self.pos[0] + t * d[..., 0]
        gy = self.pos[1] + t * d[..., 1]
        rng = np.hypot(gx - self.pos[0], gy - self.pos[1])
        self.valid = (d[..., 2] < -1e-3) & (rng < c['max_range'])
        self.lut = np.where(self.valid[..., None], np.stack([gx, gy], -1), np.nan)

    def project(self, pts):
        """Rover-frame floor points (N,2) -> pixel (u, v) in the full-size image."""
        f2, r3, u3 = self.axes
        P = np.column_stack([pts, np.zeros(len(pts))]) - self.pos
        z = P @ f2
        return np.column_stack([self.c['cx'] + self.c['f'] * (P @ r3) / z,
                                self.c['cy'] - self.c['f'] * (P @ u3) / z]), z > 0


def yuyv_planes(data, w, h, step=2):
    """YUYV 4:2:2 bytes -> Y, U, V planes at (h/step, w/step) (step 2 = no maths, just slicing)."""
    a = np.frombuffer(data[:w * h * 2], np.uint8).reshape(h, w // 2, 4)
    if step == 2:
        a = a[::2]
        return a[..., 0], a[..., 1], a[..., 3]
    y = a[..., 0::2].reshape(h, w)
    return y, np.repeat(a[..., 1], 2, 1), np.repeat(a[..., 3], 2, 1)


def rgb_to_yuv(rgb):
    """(h, w, 3) uint8 -> Y, U, V float planes (BT.601, like the webcam)."""
    r, g, b = (rgb[..., i].astype(np.float32) for i in range(3))
    y = 0.299 * r + 0.587 * g + 0.114 * b
    return y, (b - y) * 0.564 + 128, (r - y) * 0.713 + 128


def _box3(m):
    """Count of True in each 3x3 neighbourhood."""
    p = np.pad(m.astype(np.int16), 1)
    return sum(p[i:i + m.shape[0], j:j + m.shape[1]] for i in range(3) for j in range(3))


class Detector:
    """Per frame: white (lane paint), obstacle (coloured or dark) masks and their floor points."""

    def __init__(self, model, white_margin=45, chroma_min=24, dark_ratio=0.55):
        self.m = model
        self.white_margin, self.chroma_min, self.dark_ratio = white_margin, chroma_min, dark_ratio

    def run(self, Y, U, V):
        m = self.m
        Y = Y.astype(np.float32)
        chroma = np.abs(U.astype(np.float32) - 128) + np.abs(V.astype(np.float32) - 128)
        valid = m.valid
        # floor brightness per row (the floor fills most of each row inside the valid area;
        # exposure and lighting make it change a lot from top to bottom of the picture)
        Yv = np.where(valid, Y, np.nan)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore')
            floor = np.nanmedian(Yv, axis=1)
        floor = np.where(np.isnan(floor), 128, floor)[:, None]
        white = valid & (Y > floor + self.white_margin) & (chroma < 20)
        obst = valid & ((chroma > self.chroma_min) | (Y < floor * self.dark_ratio))
        obst &= _box3(obst) >= 6                     # drop speckle
        white &= _box3(white) >= 4
        # obstacle floor contact: lowest obstacle pixel in each column (and the one above it
        # must also be an obstacle, so a single noisy row can't make a point)
        h = obst.shape[0]
        rows = np.arange(h)[:, None]
        low = np.where(obst & np.roll(obst, 1, axis=0), rows, -1).max(0)
        cols = np.nonzero(low > 0)[0]
        opts = m.lut[low[cols], cols] if len(cols) else np.zeros((0, 2))
        wi, wj = np.nonzero(white)
        lpts = m.lut[wi, wj]
        return {'white': white, 'obst': obst,
                'obstacles': opts[~np.isnan(opts[:, 0])] if len(opts) else opts,
                'lines': lpts[~np.isnan(lpts[:, 0])]}


def overlay(Y, U, V, res):
    """Picture for the dashboard: the camera image (gray) with lane paint in cyan and
    obstacles in red. Returns (h, w, 3) uint8."""
    g = np.clip(Y.astype(np.float32), 0, 255)
    rgb = np.stack([g, g, g], -1)
    u = U.astype(np.float32) - 128
    v = V.astype(np.float32) - 128
    rgb = np.stack([g + 1.402 * v, g - 0.344 * u - 0.714 * v, g + 1.772 * u], -1)
    rgb = np.clip(rgb, 0, 255)
    rgb[res['white']] = rgb[res['white']] * 0.4 + np.array([0, 255, 255]) * 0.6
    rgb[res['obst']] = rgb[res['obst']] * 0.3 + np.array([255, 0, 0]) * 0.7
    return rgb.astype(np.uint8)
