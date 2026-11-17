#!/usr/bin/env python3
"""Where does the Pi's internet stop? Raw socket tests, no DNS needed for most of them."""
import socket
import struct
import time


def tcp(host, port, t=3):
    s = socket.socket()
    s.settimeout(t)
    t0 = time.time()
    try:
        s.connect((host, port))
        return f'OK ({(time.time() - t0) * 1000:.0f} ms)'
    except Exception as e:  # noqa: BLE001
        return f'FAIL ({e})'
    finally:
        s.close()


def dns(server, name='example.com', t=3):
    q = struct.pack('>HHHHHH', 0x1234, 0x0100, 1, 0, 0, 0)
    q += b''.join(bytes([len(p)]) + p.encode() for p in name.split('.')) + b'\0' + struct.pack('>HH', 1, 1)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(t)
    try:
        s.sendto(q, (server, 53))
        r, _ = s.recvfrom(512)
        n = struct.unpack('>H', r[6:8])[0]
        return f'OK ({n} answers)'
    except Exception as e:  # noqa: BLE001
        return f'FAIL ({e})'
    finally:
        s.close()


print('laptop 192.168.137.1 tcp:53 ', tcp('192.168.137.1', 53, 2))
print('DNS via laptop             ', dns('192.168.137.1'))
print('DNS via 8.8.8.8 (udp)      ', dns('8.8.8.8'))
print('DNS via 1.1.1.1 (udp)      ', dns('1.1.1.1'))
print('tcp 1.1.1.1:443            ', tcp('1.1.1.1', 443))
print('tcp 140.82.112.3:443 github', tcp('140.82.112.3', 443))
print('tcp 91.189.91.82:80 ubuntu ', tcp('91.189.91.82', 80))
