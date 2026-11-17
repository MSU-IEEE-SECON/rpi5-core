#!/usr/bin/env python3
"""Look for SECON AprilTags (tagStandard41h12) in one webcam picture.

    python3 tools/apriltag_test.py [--family tagStandard41h12] [--dev /dev/video0]

Prints each tag's id, centre (pixels), size and corner points, and saves the picture it used
as ~/rover_data/camera/test.png (shown on the dashboard).
"""
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'vendor'))
import numpy as np  # noqa: E402
import webcam  # noqa: E402
from dt_apriltags import Detector  # noqa: E402


def main():
    fam, dev = 'tagStandard41h12', None
    a = sys.argv[1:]
    if '--family' in a:
        fam = a[a.index('--family') + 1]
    if '--dev' in a:
        dev = a[a.index('--dev') + 1]
    if dev is None:
        dev, _ = webcam.find_camera()
        if dev is None:
            print('no USB camera found')
            return 1
    t0 = time.time()
    data, w, h = webcam.grab(dev, None, 640, 480, 20, 'YUYV')
    img = np.frombuffer(data[0::2][:w * h], np.uint8).reshape(h, w)
    out = os.path.expanduser('~/rover_data/camera/test')
    for ext in ('.jpg', '.png'):
        if os.path.exists(out + ext):
            os.remove(out + ext)
    webcam.png_gray(out + '.png', w, h, bytes(img))
    t1 = time.time()
    det = Detector(families=fam, nthreads=2, quad_decimate=1.0)
    tags = det.detect(img)
    print(f'{w}x{h} picture in {t1 - t0:.1f} s, detection {time.time() - t1:.2f} s: {len(tags)} {fam} tag(s)')
    for t in tags:
        c = t.corners
        side = float(np.mean([np.linalg.norm(c[i] - c[(i + 1) % 4]) for i in range(4)]))
        print(f'  id {t.tag_id}: centre ({t.center[0]:.0f}, {t.center[1]:.0f}) px, side {side:.0f} px, '
              f'decision margin {t.decision_margin:.0f}, corners {np.round(c).astype(int).tolist()}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
