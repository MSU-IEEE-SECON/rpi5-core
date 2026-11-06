"""Pi <-> ESP32 drive protocol helpers (plain Python, unit-tested).

Must match firmware/mecanum_auto/auto_drive.h:

Pi -> ESP32   "D <vx> <vy> <wz>\\n"   each -1.0..1.0 (fraction of full speed)
              vx + forward, vy + left (strafe), wz + turn left (CCW)
ESP32 -> Pi   "S <MANUAL|AUTO> <fl> <fr> <bl> <br>\\n"   wheel PWMs -255..255
              (any other line is ESP32 debug chatter)
"""
import glob
import math
import os


def _frac(value, full_scale):
    if full_scale <= 0 or not math.isfinite(value):
        return 0.0
    return max(-1.0, min(1.0, value / full_scale))


def format_drive_line(vx_mps, vy_mps, wz_radps, max_linear, max_strafe, max_angular):
    """ROS velocities -> protocol line. Scales by the full-speed calibration values."""
    return 'D {:.3f} {:.3f} {:.3f}\n'.format(
        _frac(vx_mps, max_linear), _frac(vy_mps, max_strafe), _frac(wz_radps, max_angular))


def parse_status(line):
    """'S AUTO 1 2 3 4' -> ('AUTO', (1, 2, 3, 4)); anything else -> None."""
    parts = line.strip().split()
    if len(parts) != 6 or parts[0] != 'S' or parts[1] not in ('AUTO', 'MANUAL'):
        return None
    try:
        wheels = tuple(int(p) for p in parts[2:])
    except ValueError:
        return None
    return parts[1], wheels


def pick_port(requested='auto', exclude=('/dev/rplidar',)):
    """Find the ESP32's serial port.

    requested='auto': every USB serial port except the LiDAR's. Prefers the
    stable /dev/serial/by-id/ names. Returns (port, message); port None on failure.
    """
    if requested and requested != 'auto':
        return requested, f'using configured port {requested}'
    excluded = {os.path.realpath(p) for p in exclude if os.path.exists(p)}
    by_id = sorted(glob.glob('/dev/serial/by-id/*'))
    candidates = [p for p in by_id if os.path.realpath(p) not in excluded]
    if not by_id:
        raw = sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))
        candidates = [p for p in raw if os.path.realpath(p) not in excluded]
    if len(candidates) == 1:
        return candidates[0], f'auto-detected ESP32 on {candidates[0]}'
    if not candidates:
        return None, ('no serial port found for the ESP32 (is it plugged into the Pi? '
                      f'LiDAR excluded: {sorted(excluded) or "none"})')
    return None, ('several possible ESP32 ports, set the "port" parameter to one of: '
                  + ', '.join(candidates))
