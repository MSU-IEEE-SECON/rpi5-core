#!/usr/bin/env python3
"""Grab a picture from a USB webcam with nothing but the Python standard library (V4L2).

    python3 tools/webcam.py [out_base] [--dev /dev/videoN] [--size 640x480]

Finds the first UVC (USB) camera, asks for MJPEG (saved as out_base.jpg) and falls back to
YUYV (saved as a grayscale out_base.png). Prints what it found. No OpenCV needed, so it works
on the Pi before any camera software is installed.
"""
import fcntl
import mmap
import os
import select
import struct
import sys
import time
import zlib

VIDIOC_QUERYCAP = 0x80685600
VIDIOC_ENUM_FMT = 0xC0405602
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
BUF_TYPE_CAPTURE = 1
MEMORY_MMAP = 1
CAP_VIDEO_CAPTURE = 0x1


def fourcc(s):
    return struct.unpack('<I', s.encode())[0]


MJPG, YUYV = fourcc('MJPG'), fourcc('YUYV')


def querycap(fd):
    buf = bytearray(104)
    fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
    driver = buf[0:16].split(b'\0')[0].decode()
    card = buf[16:48].split(b'\0')[0].decode()
    caps, dcaps = struct.unpack_from('<II', buf, 84)
    return driver, card, dcaps or caps


def formats(fd):
    out = []
    for i in range(32):
        buf = bytearray(64)
        struct.pack_into('<II', buf, 0, i, BUF_TYPE_CAPTURE)
        try:
            fcntl.ioctl(fd, VIDIOC_ENUM_FMT, buf)
        except OSError:
            break
        pf = struct.unpack_from('<I', buf, 44)[0]
        out.append(struct.pack('<I', pf).decode(errors='replace'))
    return out


def find_camera():
    for n in sorted(os.listdir('/dev')):
        if not n.startswith('video'):
            continue
        dev = '/dev/' + n
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            driver, card, caps = querycap(fd)
            if driver == 'uvcvideo' and caps & CAP_VIDEO_CAPTURE and formats(fd):
                return dev, card
        except OSError:
            pass
        finally:
            os.close(fd)
    return None, None


def png_gray(path, w, h, pixels):
    raw = b''.join(b'\0' + pixels[y * w:(y + 1) * w] for y in range(h))
    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 0, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def png_rgb(path, w, h, rgb):
    """rgb: bytes of h*w*3."""
    raw = b''.join(b'\0' + rgb[y * w * 3:(y + 1) * w * 3] for y in range(h))
    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    with open(path, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def yuyv_to_rgb(data, w, h):
    """YUYV 4:2:2 -> (h, w, 3) uint8 RGB (numpy)."""
    import numpy as np
    a = np.frombuffer(data[:w * h * 2], np.uint8).reshape(h, w // 2, 4).astype(np.float32)
    y = np.empty((h, w), np.float32)
    y[:, 0::2], y[:, 1::2] = a[..., 0], a[..., 2]
    u = np.repeat(a[..., 1], 2, axis=1) - 128.0
    v = np.repeat(a[..., 3], 2, axis=1) - 128.0
    rgb = np.stack([y + 1.402 * v, y - 0.344 * u - 0.714 * v, y + 1.772 * u], -1)
    return np.clip(rgb, 0, 255).astype(np.uint8)


def grab(dev, out_base, w=640, h=480, warmup=30, want=None):
    fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
    try:
        fmts = formats(fd)
        pf = {'MJPG': MJPG, 'YUYV': YUYV}.get(want) or (MJPG if 'MJPG' in fmts else YUYV)
        fmt = bytearray(208)
        struct.pack_into('<I', fmt, 0, BUF_TYPE_CAPTURE)
        struct.pack_into('<IIII', fmt, 8, w, h, pf, 1)          # width height pixfmt field=none
        fcntl.ioctl(fd, VIDIOC_S_FMT, fmt)
        w, h, pf = struct.unpack_from('<III', fmt, 8)
        req = bytearray(20)
        struct.pack_into('<III', req, 0, 2, BUF_TYPE_CAPTURE, MEMORY_MMAP)
        fcntl.ioctl(fd, VIDIOC_REQBUFS, req)
        nbuf = struct.unpack_from('<I', req, 0)[0]
        maps = []
        for i in range(nbuf):
            b = bytearray(88)
            struct.pack_into('<II', b, 0, i, BUF_TYPE_CAPTURE)
            struct.pack_into('<I', b, 60, MEMORY_MMAP)
            fcntl.ioctl(fd, VIDIOC_QUERYBUF, b)
            offset = struct.unpack_from('<I', b, 64)[0]
            length = struct.unpack_from('<I', b, 72)[0]
            maps.append(mmap.mmap(fd, length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE,
                                  offset=offset))
            fcntl.ioctl(fd, VIDIOC_QBUF, b)
        fcntl.ioctl(fd, VIDIOC_STREAMON, struct.pack('<I', BUF_TYPE_CAPTURE))
        data = None
        for k in range(warmup + 1):             # let auto exposure settle
            select.select([fd], [], [], 3.0)
            b = bytearray(88)
            struct.pack_into('<II', b, 0, 0, BUF_TYPE_CAPTURE)
            struct.pack_into('<I', b, 60, MEMORY_MMAP)
            fcntl.ioctl(fd, VIDIOC_DQBUF, b)
            idx, used = struct.unpack_from('<II', b, 0)[0], struct.unpack_from('<I', b, 8)[0]
            if k == warmup:
                data = bytes(maps[idx][:used])
            fcntl.ioctl(fd, VIDIOC_QBUF, b)
        fcntl.ioctl(fd, VIDIOC_STREAMOFF, struct.pack('<I', BUF_TYPE_CAPTURE))
        for m in maps:
            m.close()
    finally:
        os.close(fd)
    if out_base is None:                        # caller wants the raw frame
        return data, w, h
    if pf == MJPG:
        path = out_base + '.jpg'
        with open(path, 'wb') as f:
            f.write(data)
    else:
        path = out_base + '.png'
        try:
            png_rgb(path, w, h, yuyv_to_rgb(data, w, h).tobytes())
        except ImportError:
            png_gray(path, w, h, data[0::2][:w * h])
    return path, w, h, struct.pack('<I', pf).decode(), fmts


def main():
    args = sys.argv[1:]
    out = os.path.expanduser('~/rover_data/camera/test')
    dev, size, want, warm = None, (640, 480), None, 30
    i = 0
    while i < len(args):
        if args[i] == '--dev':
            dev = args[i + 1]; i += 2
        elif args[i] == '--fmt':
            want = args[i + 1].upper(); i += 2
        elif args[i] == '--warmup':
            warm = int(args[i + 1]); i += 2
        elif args[i] == '--size':
            size = tuple(int(v) for v in args[i + 1].split('x')); i += 2
        else:
            out = args[i]; i += 1
    os.makedirs(os.path.dirname(out), exist_ok=True)
    card = ''
    if dev is None:
        dev, card = find_camera()
        if dev is None:
            print('no USB (UVC) camera found in /dev/video*. What is there:')
            for n in sorted(os.listdir('/dev')):
                if n.startswith('video'):
                    d = '/dev/' + n
                    try:
                        fd = os.open(d, os.O_RDWR | os.O_NONBLOCK)
                        try:
                            print(f'  {d}: {querycap(fd)[:2]} formats {formats(fd)}')
                        finally:
                            os.close(fd)
                    except OSError as e:
                        print(f'  {d}: cannot open ({e.strerror})')
            if os.path.isdir('/dev/v4l/by-id'):
                print('  by-id:', os.listdir('/dev/v4l/by-id'))
            os.system('id; lsusb 2>/dev/null | grep -iv hub')
            return 1
    for ext in ('.jpg', '.png'):
        if os.path.exists(out + ext):
            os.remove(out + ext)
    t = time.time()
    path, w, h, f, fmts = grab(dev, out, size[0], size[1], warm, want)
    print(f'camera {dev} ({card}) formats {fmts}: saved {w}x{h} {f} -> {path} '
          f'({os.path.getsize(path) // 1024} kB, {time.time() - t:.1f} s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
