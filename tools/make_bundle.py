#!/usr/bin/env python3
"""Pack the rover code into one file the dashboard can install ("Update code" box).

    python3 tools/make_bundle.py            -> deploy/rover_code.tar.gz

Contains ros2_ws/src, pi_setup, tools, firmware, docs and a VERSION stamp, with LF line
endings for the Pi. Build output and caches are left out.
"""
import io
import os
import subprocess
import sys
import tarfile
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
DIRS = ['ros2_ws/src', 'pi_setup', 'tools', 'firmware', 'docs']
SKIP_DIRS = {'__pycache__', '.pytest_cache', 'build', 'install', 'log'}
TEXT = ('.py', '.sh', '.yaml', '.yml', '.xml', '.cfg', '.txt', '.md', '.service', '.ino', '.h', '.cpp', '.json')


def version():
    try:
        h = subprocess.run(['git', '-C', ROOT, 'rev-parse', '--short', 'HEAD'], capture_output=True,
                           text=True, timeout=5).stdout.strip()
    except Exception:  # noqa: BLE001
        h = ''
    return time.strftime('%Y-%m-%d %H:%M') + (f' ({h})' if h else '')


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, 'deploy', 'rover_code.tar.gz')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    n = 0
    with tarfile.open(out, 'w:gz') as tf:
        for d in DIRS:
            for dirpath, dirnames, filenames in os.walk(os.path.join(ROOT, d)):
                dirnames[:] = [x for x in dirnames if x not in SKIP_DIRS and not x.startswith('.')]
                for fn in filenames:
                    if fn.endswith(('.pyc', '.png', '.zip')) or (fn.startswith('.') and fn != '.so'):
                        continue
                    path = os.path.join(dirpath, fn)
                    arc = os.path.relpath(path, ROOT).replace(os.sep, '/')
                    data = open(path, 'rb').read()
                    if fn.endswith(TEXT):
                        data = data.replace(b'\r\n', b'\n')
                    info = tarfile.TarInfo(arc)
                    info.size, info.mtime = len(data), int(time.time())
                    info.mode = 0o755 if fn.endswith('.sh') else 0o644
                    tf.addfile(info, io.BytesIO(data))
                    n += 1
        v = (version() + '\n').encode()
        info = tarfile.TarInfo('VERSION')
        info.size, info.mtime = len(v), int(time.time())
        tf.addfile(info, io.BytesIO(v))
    print(f'{out}: {n} files, {os.path.getsize(out) // 1024} kB, version {v.decode().strip()}')


if __name__ == '__main__':
    main()
