"""Tests for the Pi side of the Pi <-> ESP32 protocol (no ROS needed).

    cd ros2_ws/src/rover_bringup && python3 -m pytest test/test_drive_protocol.py -q
"""
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from rover_bringup.drive_protocol import format_drive_line, parse_status, pick_port  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))


def test_format_scales_and_clamps():
    assert format_drive_line(0.25, 0.0, 1.25, 0.5, 0.4, 2.5) == 'D 0.500 0.000 0.500\n'
    assert format_drive_line(5.0, -5.0, -9.0, 0.5, 0.4, 2.5) == 'D 1.000 -1.000 -1.000\n'
    assert format_drive_line(float('nan'), 0, 0, 0.5, 0.4, 2.5) == 'D 0.000 0.000 0.000\n'
    assert format_drive_line(0.1, 0, 0, 0.0, 0.4, 2.5) == 'D 0.000 0.000 0.000\n'  # bad calibration


def test_parse_status():
    assert parse_status('S AUTO 120 -95 0 255') == ('AUTO', (120, -95, 0, 255))
    assert parse_status('S MANUAL 0 0 0 0\r') == ('MANUAL', (0, 0, 0, 0))
    assert parse_status('MODE AUTO (Y button)') is None
    assert parse_status('Translation Sensitivity: 1.10') is None
    assert parse_status('S AUTO 1 2 3') is None
    assert parse_status('S FAST 1 2 3 4') is None


def test_pick_port_explicit():
    assert pick_port('/dev/ttyUSB7')[0] == '/dev/ttyUSB7'


def test_lines_round_trip_through_real_firmware_parser():
    """The Pi's lines must be accepted by the C++ parser in auto_drive.h."""
    src = os.path.join(REPO, 'firmware', 'mecanum_auto')
    prog = r'''
    #include "auto_drive.h"
    #include <cstdio>
    int main(int, char** argv) {
      DriveCmd c;
      if (!parseDriveLine(argv[1], &c)) return 2;
      printf("%.3f %.3f %.3f", c.vx, c.vy, c.wz);
      return 0;
    }'''
    import pytest
    if not shutil.which('g++'):
        pytest.skip('no C++ compiler (g++ not on PATH)')
    exe = os.path.join(tempfile.gettempdir(), '_parse_check' + ('.exe' if os.name == 'nt' else ''))
    r = subprocess.run(['g++', '-std=c++17', '-I', src, '-x', 'c++', '-', '-o', exe],
                       input=prog.encode(), capture_output=True)
    if r.returncode != 0:
        pytest.skip('C++ compile failed: ' + r.stderr.decode()[:200])
    line = format_drive_line(0.2, -0.1, 0.5, 0.5, 0.4, 2.5).rstrip('\n')
    out = subprocess.run([exe, line], capture_output=True, text=True)
    assert out.returncode == 0, line
    assert out.stdout == '0.400 -0.250 0.200'
