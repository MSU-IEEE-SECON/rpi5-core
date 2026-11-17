"""USB webcam streaming with only the standard library (V4L2 ioctls + mmap).

Same approach as tools/webcam.py, but the camera stays open and streaming, and `read()`
returns the newest frame (frames that piled up are dropped, so there is no lag).
"""
import fcntl
import mmap
import os
import select
import struct
import zlib

VIDIOC_QUERYCAP = 0x80685600
VIDIOC_ENUM_FMT = 0xC0405602
VIDIOC_S_FMT = 0xC0D05605
VIDIOC_S_PARM = 0xC0CC5616
VIDIOC_REQBUFS = 0xC0145608
VIDIOC_QUERYBUF = 0xC0585609
VIDIOC_QBUF = 0xC058560F
VIDIOC_DQBUF = 0xC0585611
VIDIOC_STREAMON = 0x40045612
VIDIOC_STREAMOFF = 0x40045613
BUF_TYPE_CAPTURE = 1
MEMORY_MMAP = 1
CAP_VIDEO_CAPTURE = 0x1
YUYV = struct.unpack('<I', b'YUYV')[0]


def find_camera():
    """First UVC (USB) capture device, e.g. '/dev/video0', or None."""
    for n in sorted(os.listdir('/dev')):
        if not n.startswith('video'):
            continue
        dev = '/dev/' + n
        try:
            fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        except OSError:
            continue
        try:
            buf = bytearray(104)
            fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
            driver = buf[0:16].split(b'\0')[0].decode()
            caps, dcaps = struct.unpack_from('<II', buf, 84)
            fmt = bytearray(64)
            struct.pack_into('<II', fmt, 0, 0, BUF_TYPE_CAPTURE)
            fcntl.ioctl(fd, VIDIOC_ENUM_FMT, fmt)
            if driver == 'uvcvideo' and (dcaps or caps) & CAP_VIDEO_CAPTURE:
                return dev
        except OSError:
            pass
        finally:
            os.close(fd)
    return None


class Camera:
    def __init__(self, dev=None, width=640, height=480, fps=15, nbuf=4):
        self.dev = dev or find_camera()
        if self.dev is None:
            raise OSError('no USB camera found')
        self.fd = os.open(self.dev, os.O_RDWR | os.O_NONBLOCK)
        fmt = bytearray(208)
        struct.pack_into('<I', fmt, 0, BUF_TYPE_CAPTURE)
        struct.pack_into('<IIII', fmt, 8, width, height, YUYV, 1)
        fcntl.ioctl(self.fd, VIDIOC_S_FMT, fmt)
        self.width, self.height, pf = struct.unpack_from('<III', fmt, 8)
        if pf != YUYV:
            raise OSError('camera does not do YUYV')
        try:                                    # frame rate (timeperframe = 1/fps)
            parm = bytearray(204)
            struct.pack_into('<I', parm, 0, BUF_TYPE_CAPTURE)
            struct.pack_into('<II', parm, 12, 1, fps)
            fcntl.ioctl(self.fd, VIDIOC_S_PARM, parm)
        except OSError:
            pass
        req = bytearray(20)
        struct.pack_into('<III', req, 0, nbuf, BUF_TYPE_CAPTURE, MEMORY_MMAP)
        fcntl.ioctl(self.fd, VIDIOC_REQBUFS, req)
        self.maps = []
        for i in range(struct.unpack_from('<I', req, 0)[0]):
            b = self._buf(i)
            fcntl.ioctl(self.fd, VIDIOC_QUERYBUF, b)
            offset = struct.unpack_from('<I', b, 64)[0]
            length = struct.unpack_from('<I', b, 72)[0]
            self.maps.append(mmap.mmap(self.fd, length, mmap.MAP_SHARED,
                                       mmap.PROT_READ | mmap.PROT_WRITE, offset=offset))
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b)
        fcntl.ioctl(self.fd, VIDIOC_STREAMON, struct.pack('<I', BUF_TYPE_CAPTURE))

    @staticmethod
    def _buf(i=0):
        b = bytearray(88)
        struct.pack_into('<II', b, 0, i, BUF_TYPE_CAPTURE)
        struct.pack_into('<I', b, 60, MEMORY_MMAP)
        return b

    def read(self, timeout=1.0):
        """Newest frame as YUYV bytes, or None if nothing arrived within `timeout` s."""
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return None
        data = None
        while True:                             # drain: keep only the newest frame
            b = self._buf()
            try:
                fcntl.ioctl(self.fd, VIDIOC_DQBUF, b)
            except BlockingIOError:
                break
            idx, used = struct.unpack_from('<I', b, 0)[0], struct.unpack_from('<I', b, 8)[0]
            data = bytes(self.maps[idx][:used])
            fcntl.ioctl(self.fd, VIDIOC_QBUF, b)
        return data

    def close(self):
        try:
            fcntl.ioctl(self.fd, VIDIOC_STREAMOFF, struct.pack('<I', BUF_TYPE_CAPTURE))
        except OSError:
            pass
        for m in self.maps:
            m.close()
        os.close(self.fd)


def write_png(path, rgb):
    """(h, w, 3) uint8 numpy array -> PNG, written atomically (the dashboard may be reading)."""
    h, w = rgb.shape[:2]
    data = rgb.tobytes()
    raw = b''.join(b'\0' + data[y * w * 3:(y + 1) * w * 3] for y in range(h))

    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    tmp = path + '.tmp'
    with open(tmp, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                + chunk(b'IDAT', zlib.compress(raw, 1)) + chunk(b'IEND', b''))
    os.replace(tmp, path)
