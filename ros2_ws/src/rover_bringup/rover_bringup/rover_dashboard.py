"""rover_dashboard — control the rover from a web browser instead of the terminal.

Runs on the Pi (started at boot by rover-dashboard.service). On the laptop, open
    http://192.168.137.50:8080
Buttons run the same commands as rover.sh (laps, start, stop, look, snapshot, ...),
and the page shows live status, the LiDAR scan, the rover on the arena (in lap
mode) and the demo log. Needs nothing from the internet (plain Python + rclpy).

Subscribes: /scan, /rover/enable, /drive/mode, /lap/state, /lap/info, /hallway/state
E-STOP: tells the driving code to stop (/rover/enable false, zero /cmd_vel), then
stops the demo (the ESP32 halts the motors 0.3 s after commands stop).

Started with `python3 -m rover_bringup.rover_dashboard` (not `ros2 run`) so that
`rover.sh stop` doesn't kill it along with the demo.
"""
import collections
import io
import json
import math
import os
import subprocess
import tarfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from rover_bringup import arena as A
from rover_bringup import lidar_calib

ROOT = os.environ.get('ROVER_ROOT', os.path.expanduser('~/repos/rpi5-core'))
DATA = os.path.expanduser(os.environ.get('ROVER_DATA', '~/rover_data'))
LOG = os.path.expanduser('~/demo.log')
PORT = int(os.environ.get('ROVER_DASHBOARD_PORT', '8080'))
# button -> rover.sh command
ACTIONS = {'lap': 'lap', 'start': 'hallway', 'stop': 'stop', 'restart': 'restart',
           'look': 'look', 'snapshot': 'snapshot', 'mask': 'mask', 'calibrate': 'calibrate',
           'diag': 'diag', 'fwinfo': 'fwinfo', 'camcheck': 'camcheck', 'netcheck': 'netcheck', 'camshot': 'camshot', 'tagtest': 'tagtest',
           'camlive': 'camlive'}


# ---- code upload (deploy from the browser) --------------------------------------------
DEPLOY_DIRS = ('ros2_ws/src/', 'pi_setup/', 'tools/', 'firmware/', 'docs/', 'VERSION')
MAX_BUNDLE = 30 * 1024 * 1024


def install_bundle(data, root, keep_dir):
    """Unpack an uploaded .tar.gz of the repo over `root` (like the old scp merge).

    Only paths under DEPLOY_DIRS are accepted (no absolute paths, no '..', no links).
    The files it replaces are saved first in keep_dir/backup_<time>.tar.gz.
    Returns (ok, message, list_of_files).
    """
    if len(data) > MAX_BUNDLE:
        return False, f'bundle too big ({len(data) // 1024} kB)', []
    try:
        tf = tarfile.open(fileobj=io.BytesIO(data), mode='r:gz')
        members = tf.getmembers()
    except Exception as e:  # noqa: BLE001
        return False, f'not a .tar.gz: {e}', []
    files = []
    for m in members:
        name = m.name.lstrip('./')
        if not name or m.isdir():
            continue
        if (m.name.startswith('/') or '..' in name.split('/') or not (m.isfile())
                or not any(name == d or name.startswith(d) for d in DEPLOY_DIRS)):
            return False, f'refusing path in bundle: {m.name}', []
        files.append((m, name))
    if not files:
        return False, 'bundle has no files', []
    os.makedirs(keep_dir, exist_ok=True)
    stamp = time.strftime('%Y%m%d_%H%M%S')
    with tarfile.open(os.path.join(keep_dir, f'backup_{stamp}.tar.gz'), 'w:gz') as bk:
        for _, name in files:
            path = os.path.join(root, name)
            if os.path.isfile(path):
                bk.add(path, arcname=name)
    for m, name in files:
        path = os.path.join(root, name)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        src = tf.extractfile(m)
        tmp = path + '.upload'
        with open(tmp, 'wb') as f:
            f.write(src.read())
        os.chmod(tmp, 0o755 if name.endswith('.sh') else 0o644)
        os.replace(tmp, path)
    # keep only the 5 newest backups
    bks = sorted(b for b in os.listdir(keep_dir) if b.startswith('backup_'))
    for b in bks[:-5]:
        os.remove(os.path.join(keep_dir, b))
    return True, f'installed {len(files)} files (backup_{stamp}.tar.gz kept)', [n for _, n in files]


def deployed_version(root):
    try:
        with open(os.path.join(root, 'VERSION')) as f:
            return f.read().strip()
    except OSError:
        return None


def rover_points(angle_min, angle_inc, ranges, yaw, mirror, mask_bin=1.0, mask=None,
                 min_range=0.12, max_range=8.0):
    """Raw scan -> [(x_fwd, y_left, range)] in the rover frame, self mask removed."""
    out = []
    nb = int(round(360.0 / mask_bin))
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and min_range <= r < max_range):
            continue
        raw = angle_min + i * angle_inc
        if mask:
            b = int(math.floor(math.degrees(raw) % 360.0 / mask_bin)) % nb
            lim = max(mask.get((b + d) % nb, -1.0) for d in (-1, 0, 1))
            if lim > 0 and r <= lim + 0.03:
                continue
        a = yaw + (-raw if mirror else raw)
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


class Model:
    """Everything the page shows. Filled by ROS callbacks, read by HTTP threads."""

    def __init__(self):
        self.lock = threading.Lock()
        self.scan = []                 # rover-frame points (x, y)
        self.scan_times = collections.deque(maxlen=40)
        self.enable = None
        self.mode = None
        self.lap_state = None
        self.lap_state_t = 0.0
        self.hall_state = None
        self.hall_state_t = 0.0
        self.lap_info = None
        self.lap_info_t = 0.0
        self.cam_lines = []          # webcam lane-paint points, rover frame
        self.cam_lines_t = 0.0
        self.cam_obs = []
        self.cam_obs_t = 0.0
        self.cam_info = None
        self.cam_info_t = 0.0
        self.action = None             # {'name', 'started', 'rc'}
        self.action_out = collections.deque(maxlen=1500)
        self.proc = None
        self.out_tail = 80
        self.estop_cb = None           # set by the ROS side
        self.debug_cb = None           # ROS graph info (set by the ROS side)
        self.scan_msgs = 0
        self.rosout = collections.deque(maxlen=400)   # every node's log lines, live from /rosout
        self.last_error = None
        self.reload_calib()

    def reload_calib(self):
        c = lidar_calib.load() or {}
        self.calib = c
        self.yaw = math.radians(float(c.get('laser_yaw_deg', 180.0)))
        self.mirror = bool(c.get('laser_mirror', False))
        self.fov = (float(c.get('fov_left_deg', 83.0)), float(c.get('fov_right_deg', 83.0)))
        self.mask_bin, self.mask = 1.0, {}
        try:
            with open(os.path.join(DATA, 'self_mask.json')) as f:
                d = json.load(f)
            self.mask_bin = float(d['bin_deg'])
            self.mask = {int(b): float(r) for b, r in d['bins'].items()}
        except Exception:  # noqa: BLE001
            pass
        self.calib_mtime = self._calib_mtime()

    def _calib_mtime(self):
        try:
            return (os.path.getmtime(os.path.join(DATA, 'lidar_calib.json')),
                    os.path.getmtime(os.path.join(DATA, 'self_mask.json')))
        except OSError:
            return None

    # ---- inputs ---------------------------------------------------------------
    def on_scan(self, angle_min, angle_inc, ranges):
        self.scan_msgs += 1
        try:
            self._on_scan(angle_min, angle_inc, ranges)
        except Exception as e:  # noqa: BLE001
            self.last_error = f'on_scan: {e!r}'

    def _on_scan(self, angle_min, angle_inc, ranges):
        pts = rover_points(angle_min, angle_inc, ranges, self.yaw, self.mirror,
                           self.mask_bin, self.mask)
        with self.lock:
            self.scan = pts[::2] if len(pts) > 400 else pts
            self.scan_times.append(time.monotonic())

    def set(self, key, value):
        with self.lock:
            setattr(self, key, value)
            if hasattr(self, key + '_t'):
                setattr(self, key + '_t', time.monotonic())

    # ---- actions --------------------------------------------------------------
    def run_action(self, name, cmd=None):
        if name == 'estop':
            if self.estop_cb:
                self.estop_cb()
            name = 'stop'
        if cmd is None and name not in ACTIONS:
            return False, f'unknown action {name}'
        with self.lock:
            busy = self.proc is not None and self.proc.poll() is None
            if busy and name not in ('stop',):
                return False, f"busy: '{self.action['name']}' is still running"
            if busy:                     # stop wins over whatever is running
                try:
                    self.proc.terminate()
                except Exception:  # noqa: BLE001
                    pass
            self.action_out.clear()
            self.action = {'name': name, 'started': time.time(), 'rc': None}
            try:
                self.proc = subprocess.Popen(
                    cmd or ['bash', os.path.join(ROOT, 'pi_setup', 'rover.sh'), ACTIONS[name]],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                    text=True, bufsize=1, start_new_session=True)
            except OSError as e:
                self.action['rc'] = -1
                self.action_out.append(f'could not run rover.sh: {e}')
                return False, str(e)
            proc = self.proc
        threading.Thread(target=self._pump, args=(proc,), daemon=True).start()
        return True, f'running {name}'

    def deploy(self, data):
        """Install an uploaded code bundle, then rebuild (output also in deploy/last_build.log)."""
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return False, f"busy: '{self.action['name']}' is still running"
        keep = os.path.join(DATA, 'deploy')
        ok, msg, files = install_bundle(data, ROOT, keep)
        if not ok:
            return False, msg
        build = (f"set -o pipefail; sed -i 's/\\r$//' {ROOT}/pi_setup/*.sh; echo '{msg}'; "
                 f"echo 'version: {deployed_version(ROOT) or '?'}'; "
                 f"if bash {ROOT}/pi_setup/rover.sh build 2>&1 | tee {keep}/last_build.log; then "
                 f"echo 'BUILD OK. The dashboard restarts in a few seconds; restart laps to use the new code.'; "
                 f"else echo 'BUILD FAILED (old code still installed in build/; see above)'; exit 1; fi")
        ok2, msg2 = self.run_action('deploy', cmd=['bash', '-c', build])
        return ok2, msg + ('; building...' if ok2 else f'; build not started: {msg2}')

    def _pump(self, proc):
        for line in proc.stdout:
            with self.lock:
                self.action_out.append(line.rstrip('\n'))
        rc = proc.wait()
        with self.lock:
            if self.proc is proc:
                self.action['rc'] = rc
        if self._calib_mtime() != self.calib_mtime:      # mask/calibrate wrote new files
            self.reload_calib()

    def set_mirror(self, mirror):
        """Flip the LiDAR left/right setting in lidar_calib.json (driving code reads it on start)."""
        c = lidar_calib.load() or {'laser_yaw_deg': 180.0, 'fov_left_deg': 83.0, 'fov_right_deg': 83.0}
        c['laser_mirror'] = bool(mirror)
        lidar_calib.save(c)
        self.reload_calib()
        return True, f"Mirrored set to {'yes' if mirror else 'no'}. Restart laps/demo to use it."

    # ---- outputs --------------------------------------------------------------
    def status(self):
        now = time.monotonic()
        with self.lock:
            st = [t for t in self.scan_times if now - t < 2.0]
            rate = (len(st) - 1) / (st[-1] - st[0]) if len(st) > 2 and st[-1] > st[0] else 0.0
            lap_fresh = now - self.lap_info_t < 2.0
            return {
                'running': launch_running(),
                'devices': {'lidar': os.path.exists('/dev/rplidar'), 'esp32': os.path.exists('/dev/esp32')},
                'scan_hz': round(rate, 1),
                'auto': self.enable,
                'mode': self.mode,
                'lap_state': self.lap_state if now - self.lap_state_t < 2.0 else None,
                'hall_state': self.hall_state if now - self.hall_state_t < 2.0 else None,
                'lap': self.lap_info if lap_fresh else None,
                'calib': {'mirror': self.mirror, 'yaw_deg': round(math.degrees(self.yaw), 1),
                          'fov': self.fov, 'found': bool(self.calib), 'mask_bins': len(self.mask)},
                'action': dict(self.action) if self.action else None,
                'version': deployed_version(ROOT),
                'camera': self.cam_info if now - self.cam_info_t < 5.0 else None,
                'cam_age': live_age(),
                'action_out': list(self.action_out)[-int(self.out_tail):],
                'time': time.time(),
            }

    def scan_json(self):
        with self.lock:
            # the last scan stays on screen after the LiDAR stops (marked with its age)
            age = time.monotonic() - self.scan_times[-1] if self.scan_times else None
            now = time.monotonic()
            return {'pts': [[round(x, 3), round(y, 3)] for x, y in self.scan],
                    'age_s': None if age is None else round(age, 1),
                    'fov': self.fov,
                    'cam_lines': self.cam_lines if now - self.cam_lines_t < 1.0 else [],
                    'cam_obs': self.cam_obs if now - self.cam_obs_t < 1.0 else []}


def live_age():
    try:
        return round(time.time() - os.path.getmtime(os.path.join(DATA, 'camera', 'live.png')), 1)
    except OSError:
        return None


def launch_running():
    try:
        out = subprocess.run(['pgrep', '-af', 'ros2 launch rover_bringup'], capture_output=True,
                             text=True, timeout=2).stdout
    except Exception:  # noqa: BLE001
        return None
    for line in out.splitlines():
        for name in ('lap', 'demo', 'lidar'):
            if f'{name}.launch.py' in line:
                return name
    return 'yes' if out.strip() else None


def tail(path, n=60):
    try:
        with open(path, 'rb') as f:
            f.seek(0, 2)
            f.seek(max(0, f.tell() - 200 * n))
            lines = f.read().decode(errors='replace').splitlines()
        return lines[-n:]
    except OSError:
        return ['(no log yet: ~/demo.log)']


def arena_json():
    try:
        from rover_bringup.path_lap import build_path
        line = [[round(float(x), 3), round(float(y), 3)] for x, y in build_path()[0][::4]]
    except Exception:  # noqa: BLE001
        line = A.LAP
    return {'walls': A.WALLS, 'rail': A.RAIL, 'fence': A.PIT_FENCE, 'lap': line,
            'W': A.W, 'L': A.L, 'y_sep': A.Y_SEP, 'lidar_fwd': 0.13}


# ---- HTTP -------------------------------------------------------------------
def make_handler(model):

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):   # quiet
            pass

        def _send(self, body, ctype='application/json', code=200, extra=None):
            if not isinstance(body, (bytes, bytearray)):
                body = json.dumps(body).encode() if ctype == 'application/json' else body.encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            p = self.path.split('?')[0]
            if p == '/':
                self._send(PAGE, 'text/html; charset=utf-8')
            elif p == '/api/status':
                self._send(model.status())
            elif p == '/api/scan':
                self._send(model.scan_json())
            elif p == '/api/arena':
                self._send(arena_json())
            elif p == '/api/debug':
                env = {k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_LOCALHOST_ONLY',
                       'ROS_AUTOMATIC_DISCOVERY_RANGE', 'ROS_STATIC_PEERS', 'FASTRTPS_DEFAULT_PROFILES_FILE',
                       'CYCLONEDDS_URI', 'FASTDDS_BUILTIN_TRANSPORTS', 'ROVER_DATA', 'ROVER_ROOT', 'USER', 'HOME')}
                d = {'env': env, 'scan_msgs': model.scan_msgs, 'last_error': model.last_error, 'pid': os.getpid()}
                if model.debug_cb:
                    try:
                        d.update(model.debug_cb())
                    except Exception as e:  # noqa: BLE001
                        d['graph_error'] = repr(e)
                self._send(d)
            elif p == '/api/rosout':
                with model.lock:
                    lines = list(model.rosout)
                q = self.path.split('node=')[1].split('&')[0] if 'node=' in self.path else ''
                self._send({'lines': [l for l in lines if q in l][-150:]})
            elif p == '/files/action.txt':
                with model.lock:
                    name = model.action['name'] if model.action else 'none'
                    text = '\n'.join(model.action_out) + '\n'
                self._send(text.encode(), 'text/plain; charset=utf-8', extra={
                    'Content-Disposition': f'attachment; filename="rover_{name}_{time.strftime("%m%d_%H%M%S")}.txt"'})
            elif p == '/api/action_out':
                with model.lock:
                    self._send({'action': dict(model.action) if model.action else None,
                                'lines': list(model.action_out)})
            elif p == '/api/last_build':
                self._send({'lines': tail(os.path.join(DATA, 'deploy', 'last_build.log'), 80)})
            elif p == '/api/log':
                self._send({'lines': tail(LOG)})
            elif p == '/files/live.png':
                try:
                    with open(os.path.join(DATA, 'camera', 'live.png'), 'rb') as f:
                        data = f.read()
                except OSError:
                    return self._send({'error': 'no live picture (camera not running)'}, code=404)
                self._send(data, 'image/png')
            elif p == '/files/camera.jpg':
                for ext, ctype in (('jpg', 'image/jpeg'), ('png', 'image/png')):
                    path = os.path.join(DATA, 'camera', 'test.' + ext)
                    if os.path.exists(path):
                        with open(path, 'rb') as f:
                            self._send(f.read(), ctype)
                        break
                else:
                    self._send({'error': 'no picture yet (run camshot)'}, code=404)
            elif p in ('/files/demo.log', '/files/snapshot.json'):
                path = LOG if p.endswith('.log') else os.path.join(DATA, 'snapshots', 'latest.json')
                try:
                    with open(path, 'rb') as f:
                        data = f.read()
                except OSError:
                    return self._send({'error': 'not found'}, code=404)
                stamp = time.strftime('%m%d_%H%M%S')
                name = f'demo_{stamp}.log' if p.endswith('.log') else f'scan_{stamp}.json'
                self._send(data, 'application/octet-stream',
                           extra={'Content-Disposition': f'attachment; filename="{name}"'})
            else:
                self._send({'error': 'not found'}, code=404)

        def do_POST(self):
            if self.path == '/api/deploy':
                n = int(self.headers.get('Content-Length', '0'))
                if n <= 0 or n > MAX_BUNDLE:
                    return self._send({'ok': False, 'msg': f'bad size {n}'}, code=400)
                data = self.rfile.read(n)
                ok, msg = model.deploy(data)
                return self._send({'ok': ok, 'msg': msg}, code=200 if ok else 409)
            if self.path not in ('/api/action', '/api/calib'):
                return self._send({'error': 'not found'}, code=404)
            try:
                n = int(self.headers.get('Content-Length', '0'))
                body = json.loads(self.rfile.read(n) or b'{}')
            except Exception:  # noqa: BLE001
                return self._send({'ok': False, 'msg': 'bad request'}, code=400)
            if self.path == '/api/calib':
                if not isinstance(body.get('laser_mirror'), bool):
                    return self._send({'ok': False, 'msg': 'laser_mirror must be true/false'}, code=400)
                ok, msg = model.set_mirror(body['laser_mirror'])
            else:
                ok, msg = model.run_action(str(body.get('name', '')))
            self._send({'ok': ok, 'msg': msg}, code=200 if ok else 409)

    return Handler


def serve(model, port=PORT):
    srv = ThreadingHTTPServer(('0.0.0.0', port), make_handler(model))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


# ---- ROS --------------------------------------------------------------------
def main(args=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, Float32MultiArray, String

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # The dashboard found the LiDAR's /scan publisher but got none of its data over Fast DDS
    # shared memory (Nov 13), so it talks over UDP only. Set just for this node: the commands
    # it runs (rover.sh) get the normal environment back.
    had = os.environ.get('FASTDDS_BUILTIN_TRANSPORTS')
    if os.environ.get('ROVER_DASHBOARD_SHM') != '1':
        os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = 'UDPv4'
    rclpy.init(args=args)
    node = Node('rover_dashboard')
    if had is None:
        os.environ.pop('FASTDDS_BUILTIN_TRANSPORTS', None)
    else:
        os.environ['FASTDDS_BUILTIN_TRANSPORTS'] = had
    model = Model()
    node.create_subscription(LaserScan, 'scan', lambda m: model.on_scan(
        m.angle_min, m.angle_increment, list(m.ranges)), qos_profile_sensor_data)
    node.create_subscription(Bool, 'rover/enable', lambda m: model.set('enable', m.data), latched)
    node.create_subscription(String, 'drive/mode', lambda m: model.set('mode', m.data), 10)
    node.create_subscription(String, 'lap/state', lambda m: model.set('lap_state', m.data), 10)
    node.create_subscription(String, 'hallway/state', lambda m: model.set('hall_state', m.data), 10)

    def on_info(m):
        try:
            model.set('lap_info', json.loads(m.data))
        except ValueError:
            pass
    node.create_subscription(String, 'lap/info', on_info, 10)

    def pairs(m):
        d = list(m.data)
        return [[round(d[i], 3), round(d[i + 1], 3)] for i in range(0, len(d) - 1, 2)]
    node.create_subscription(Float32MultiArray, 'camera/lines',
                             lambda m: model.set('cam_lines', pairs(m)[::2]), qos_profile_sensor_data)
    node.create_subscription(Float32MultiArray, 'camera/obstacles',
                             lambda m: model.set('cam_obs', pairs(m)), qos_profile_sensor_data)

    def on_cam_info(m):
        try:
            model.set('cam_info', json.loads(m.data))
        except ValueError:
            pass
    node.create_subscription(String, 'camera/info', on_cam_info, 10)

    from rcl_interfaces.msg import Log
    levels = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}

    def on_rosout(m):
        if m.name == 'rover_dashboard':
            return
        line = f"{time.strftime('%H:%M:%S')} {levels.get(m.level, m.level)} [{m.name}] {m.msg}"
        with model.lock:
            model.rosout.append(line)
    node.create_subscription(Log, 'rosout', on_rosout, QoSProfile(
        depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    cmd_pub = node.create_publisher(Twist, 'cmd_vel', 10)

    def estop():
        # short-lived latched publisher so a stale "false" doesn't linger for later runs
        pub = node.create_publisher(Bool, 'rover/enable', latched)
        pub.publish(Bool(data=False))
        for _ in range(3):
            cmd_pub.publish(Twist())
        node.get_logger().warn('E-STOP pressed')
        threading.Timer(2.0, lambda: node.destroy_publisher(pub)).start()
    model.estop_cb = estop
    model.debug_cb = lambda: {
        'topics': sorted(n for n, _ in node.get_topic_names_and_types()),
        'nodes': sorted(node.get_node_names()),
        'scan_publishers': node.count_publishers('scan'),
        'dashboard_transport': 'shared memory + UDP' if os.environ.get('ROVER_DASHBOARD_SHM') == '1' else 'UDPv4 only',
    }

    serve(model)
    node.get_logger().info(f'dashboard on http://0.0.0.0:{PORT}  (laptop: http://192.168.137.50:{PORT})')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Rover Dashboard</title>
<style>
:root{--bg:#f4f5f7;--card:#fff;--ink:#1d2330;--mute:#6b7385;--line:#dde1e8;--ok:#1f9d55;--warn:#c98a00;
--bad:#d23b3b;--acc:#2f6fdb;--pts:#2f6fdb;--wall:#1d2330;--rail:#8a93a6}
@media (prefers-color-scheme:dark){:root{--bg:#12151b;--card:#1b2029;--ink:#e6e9ef;--mute:#8d96a8;
--line:#2c3340;--pts:#6aa3ff;--wall:#d7dce5;--rail:#6b7385}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);
font:14px/1.4 system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
header{display:flex;align-items:center;gap:12px;padding:12px 16px;flex-wrap:wrap}
h1{font-size:18px;margin:0 8px 0 0}
.pill{padding:3px 10px;border-radius:99px;background:var(--card);border:1px solid var(--line);font-size:12px}
.pill b{font-weight:600}.ok{color:var(--ok)}.bad{color:var(--bad)}.warn{color:var(--warn)}
main{display:grid;grid-template-columns:minmax(260px,330px) 1fr 1fr;gap:12px;padding:0 16px 16px}
@media (max-width:1000px){main{grid-template-columns:1fr}}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px}
.card h2{font-size:13px;text-transform:uppercase;letter-spacing:.04em;color:var(--mute);margin:0 0 8px}
button{font:inherit;padding:9px 10px;border-radius:8px;border:1px solid var(--line);background:var(--bg);
color:var(--ink);cursor:pointer;text-align:left}button:hover{border-color:var(--acc)}
button:disabled{opacity:.45;cursor:default}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.btn-main{background:var(--acc);color:#fff;border-color:var(--acc)}.btn-main small{color:#dfe8ff}
#estop{width:100%;padding:16px;font-size:18px;font-weight:700;background:var(--bad);color:#fff;
border:none;text-align:center;margin-bottom:10px}
small{color:var(--mute);display:block}
pre{margin:0;max-height:260px;overflow:auto;font:12px/1.35 ui-monospace,Consolas,monospace;white-space:pre-wrap}
canvas{width:100%;display:block}
.kv{display:grid;grid-template-columns:auto 1fr;gap:2px 10px;font-size:13px}.kv span:nth-child(odd){color:var(--mute)}
.wide{grid-column:2/4}@media (max-width:1000px){.wide{grid-column:auto}}
a{color:var(--acc)}
</style></head><body>
<header><h1>Rover</h1>
<span class="pill" id="p-conn">connecting…</span>
<span class="pill" id="p-run"></span><span class="pill" id="p-lidar"></span>
<span class="pill" id="p-esp"></span><span class="pill" id="p-mode"></span>
</header>
<main>
<section class="card">
<button id="estop" onclick="act('estop')">STOP</button>
<h2>Drive</h2>
<div class="grid">
<button class="btn-main" onclick="act('lap')">Arena laps<small>right lane, then Y</small></button>
<button onclick="act('start')">Hallway demo<small>archived mode</small></button>
<button onclick="act('restart')">Restart<small>same mode again</small></button>
<button onclick="act('stop')">Stop<small>shut the demo down</small></button>
</div>
<h2 style="margin-top:14px">LiDAR tools</h2>
<div class="grid">
<button onclick="act('look')">Look<small>20 s direction check</small></button>
<button onclick="act('snapshot')">Snapshot<small>save 2 s of scans</small></button>
<button onclick="act('mask','Rover in open space, nothing within 0.3 m?')">Self mask<small>open space!</small></button>
<button onclick="act('calibrate','Calibrate: empty space, then hold a box where the output says. Go?')">Calibrate<small>box in front, then left</small></button>
</div>
<h2 style="margin-top:14px">Files</h2>
<a href="/files/demo.log">Download demo.log</a> · <a href="/files/snapshot.json">latest snapshot</a>
<h2 style="margin-top:14px">Update code</h2>
<input type="file" id="bundle" accept=".gz,.tgz">
<button id="deploy-btn" onclick="deploy()" style="margin-top:6px">Upload + rebuild</button>
<small id="version"></small>
<h2 style="margin-top:14px">Settings</h2><div class="kv" id="calib"></div>
</section>
<section class="card"><h2>LiDAR (rover view, front = up)</h2><canvas id="radar" height="420"></canvas>
<small id="radar-note"></small><pre id="radar-text" style="margin-top:6px;max-height:110px"></pre></section>
<section class="card"><h2>Arena</h2><canvas id="board" height="420"></canvas><div class="kv" id="lap"></div></section>
<section class="card"><h2>Camera</h2><img id="cam" style="width:100%;display:block;background:#000;aspect-ratio:4/3" alt="">
<small id="cam-note">The camera runs during laps, or press Camera view.</small>
<div class="grid" style="margin-top:6px"><button onclick="act('camlive')">Camera view<small>3 min, rover parked</small></button></div></section>
<section class="card"><h2 id="act-title">Last command</h2><pre id="act-out">(none yet)</pre></section>
<section class="card wide"><h2>demo.log</h2><pre id="log"></pre></section>
</main>
<script>
const $=id=>document.getElementById(id);let arena=null,status=null,scan={pts:[],fov:[83,83]};
const css=v=>getComputedStyle(document.documentElement).getPropertyValue(v).trim();
function pill(el,label,val,cls){el.innerHTML=label+' <b class="'+cls+'">'+val+'</b>'}
async function j(u,o){const r=await fetch(u,o);return r.json()}
async function act(name,ask){if(ask&&!confirm(ask))return;
 const r=await j('/api/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name})});
 if(!r.ok)alert(r.msg);poll()}
async function flipMirror(m){if(!confirm('Set Mirrored to '+(m?'yes':'no')+'? (Upside-down LiDAR on this rover = no.)'))return;
 const r=await j('/api/calib',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({laser_mirror:m})});alert(r.msg);poll()}
async function deploy(){const f=$('bundle').files[0];if(!f){alert('Pick the rover_code.tar.gz file first');return}
 $('deploy-btn').disabled=true;try{const r=await fetch('/api/deploy',{method:'POST',headers:{'Content-Type':'application/gzip'},body:f}).then(r=>r.json());
 $('version').textContent=r.msg}catch(e){$('version').textContent='upload failed: '+e}$('deploy-btn').disabled=false;poll()}
function fit(c){const w=c.clientWidth,d=devicePixelRatio||1;
 if(c.width!==Math.round(w*d)){c.width=Math.round(w*d);c.height=Math.round(420*d)}return d}
function drawRadar(){const c=$('radar'),d=fit(c),g=c.getContext('2d'),W=c.width,H=c.height;
 g.clearRect(0,0,W,H);const cx=W/2,cy=H*0.62,R=Math.min(W/2,cy)-8*d,maxr=1.5,s=R/maxr;
 g.strokeStyle=css('--line');g.fillStyle=css('--mute');g.lineWidth=d;g.font=(11*d)+'px system-ui';
 for(let r=0.5;r<=maxr+1e-6;r+=0.5){g.beginPath();g.arc(cx,cy,r*s,0,7);g.stroke();g.fillText(r+' m',cx+3*d,cy-r*s+12*d)}
 const fl=scan.fov[0]*Math.PI/180,fr=scan.fov[1]*Math.PI/180;
 g.fillStyle=css('--acc')+'14';g.beginPath();g.moveTo(cx,cy);g.arc(cx,cy,R,-Math.PI/2-fl,-Math.PI/2+fr);g.fill();
 g.fillStyle=css('--ink');g.fillRect(cx-0.127*s,cy-0.03*s,0.254*s,0.3*s);   // body, LiDAR ~at the front
 let n=0;for(const[x,y]of scan.pts){const a=Math.atan2(y,x);const inv=(a>=0?a<=fl:-a<=fr);
  const px=cx-y*s,py=cy-x*s;if(Math.hypot(x,y)>maxr)continue;n++;
  g.fillStyle=inv?css('--pts'):css('--mute');g.fillRect(px-1.5*d,py-1.5*d,3*d,3*d)}
 const live=scan.age_s!==null&&scan.age_s<1.5;
 $('radar-note').textContent=scan.pts.length?((live?'LIVE: ':'LAST SCAN from '+new Date(Date.now()-scan.age_s*1000).toLocaleTimeString()+' (LiDAR stopped): ')+scan.pts.length+
  ' points. Shaded = arc the driving code uses, grey = ignored.'):'No scans yet: the LiDAR only runs during laps / demo / Look / Snapshot.'}
function drawBoard(){const c=$('board'),d=fit(c),g=c.getContext('2d'),W=c.width,H=c.height;g.clearRect(0,0,W,H);
 if(!arena)return;const s=Math.min((W-20*d)/arena.W,(H-20*d)/arena.L),ox=(W-arena.W*s)/2,oy=H-10*d;
 const P=(x,y)=>[ox+x*s,oy-y*s];const seg=(L,col,lw,dash)=>{g.strokeStyle=col;g.lineWidth=lw*d;g.setLineDash(dash||[]);
  for(const[a,b,c2,e]of L){g.beginPath();g.moveTo(...P(a,b));g.lineTo(...P(c2,e));g.stroke()}g.setLineDash([])};
 g.fillStyle=css('--bad')+'12';const[a0,b0]=P(0,arena.y_sep);g.fillRect(a0,b0,arena.W*s,arena.y_sep*s);
 seg(arena.walls,css('--wall'),2.5);seg(arena.rail,css('--rail'),2);seg(arena.fence,css('--bad'),1.5,[4*d,4*d]);
 g.strokeStyle=css('--acc')+'88';g.setLineDash([2*d,5*d]);g.beginPath();arena.lap.forEach((p,i)=>i?g.lineTo(...P(...p)):g.moveTo(...P(...p)));
 g.closePath();g.stroke();g.setLineDash([]);g.fillStyle=css('--mute');g.font=(11*d)+'px system-ui';
 g.fillText('PIT (stay out)',ox+6*d,oy-arena.y_sep*s/2);
 const L=status&&status.lap;if(!L){g.fillText('Rover shows here during arena laps',ox+6*d,oy-arena.L*s/2);return}
 if(L.obs){g.fillStyle=css('--bad')+'aa';for(const[ox_,oy_]of L.obs){const[px,py]=P(ox_,oy_);g.fillRect(px-1.5*d,py-1.5*d,3*d,3*d)}}
 const[lx,ly]=L.lidar,th=L.th,c0=Math.cos(th),s0=Math.sin(th);
 const fl=scan.fov[0]*Math.PI/180,fr=scan.fov[1]*Math.PI/180;
 for(const[x,y]of scan.pts){const a=Math.atan2(y,x);g.fillStyle=(a>=0?a<=fl:-a<=fr)?css('--pts'):css('--mute');
  const wx=lx+x*c0-y*s0,wy=ly+x*s0+y*c0;const[px,py]=P(wx,wy);g.fillRect(px-1.2*d,py-1.2*d,2.4*d,2.4*d)}
 const cc=Math.cos(th),cs=Math.sin(th),cam=(pts,col,r)=>{g.fillStyle=col;for(const[x,y]of pts||[]){
  const[px,py]=P(L.x+x*cc-y*cs,L.y+x*cs+y*cc);g.fillRect(px-r*d,py-r*d,2*r*d,2*r*d)}};
 cam(scan.cam_lines,'#22c7d6',1);cam(scan.cam_obs,'#ff7a00',1.6);
 g.save();g.translate(...P(L.x,L.y));g.rotate(-th);g.fillStyle=css('--ok')+'cc';g.fillRect(-0.15*s,-0.127*s,0.3*s,0.254*s);
 g.fillStyle='#fff';g.beginPath();g.moveTo(0.15*s,0);g.lineTo(0.05*s,-0.07*s);g.lineTo(0.05*s,0.07*s);g.fill();g.restore()}
function kv(el,o){el.innerHTML=Object.entries(o).map(([k,v])=>'<span>'+k+'</span><span>'+v+'</span>').join('')}
function render(){const s=status;if(!s)return;
 if(!$('deploy-btn').disabled&&!($('version').textContent||'').startsWith('installed'))$('version').textContent='Code on the Pi: '+(s.version||'unknown');
 pill($('p-run'),'Demo',s.running?(s.running==='lap'?'LAPS':s.running==='demo'?'HALLWAY':s.running.toUpperCase()):'off',s.running?'ok':'');
 pill($('p-lidar'),'LiDAR',s.devices.lidar?(s.scan_hz?s.scan_hz+' Hz':'plugged in'):'MISSING',s.devices.lidar?'ok':'bad');
 pill($('p-esp'),'ESP32',s.devices.esp32?(s.mode==='DISCONNECTED'?'silent':'plugged in'):'MISSING',s.devices.esp32&&s.mode!=='DISCONNECTED'?'ok':'bad');
 const st=s.lap_state||s.hall_state;pill($('p-mode'),'Driving',s.auto?('AUTO'+(st?' · '+st:'')):(s.running?'manual (press Y)':'—'),s.auto?'warn':'');
 kv($('calib'),{'LiDAR front':s.calib.yaw_deg+'°','Mirrored':(s.calib.mirror?'yes':'no')+
  ' <button style="padding:1px 8px;font-size:12px" onclick="flipMirror('+!s.calib.mirror+')">change</button>',
  'View arc':'+'+s.calib.fov[0]+'° / −'+s.calib.fov[1]+'°','Self mask':s.calib.mask_bins?s.calib.mask_bins+' bins':'none',
  'Calibration':s.calib.found?'loaded':'not found (defaults)'});
 const L=s.lap,IN=0.0254;kv($('lap'),L?{'State':L.state,'Position':(L.x/IN).toFixed(1)+', '+(L.y/IN).toFixed(1)+' in',
  'Heading':(L.th*180/Math.PI).toFixed(0)+'°','Leg / laps':L.leg+' / '+L.laps,'Match q':L.q}:{});
 if(s.action){const a=s.action;$('act-title').textContent='Last command: '+a.name+(a.rc===null?' (running…)':a.rc===0?' (done)':' (exit '+a.rc+')');
  const lk=s.action_out.filter(l=>l.startsWith('front:')).slice(-1)[0];$('radar-text').textContent=lk?('Last Look reading: '+lk):'';
  const o=$('act-out'),end=o.scrollTop+o.clientHeight>=o.scrollHeight-5;o.textContent=s.action_out.join('\n')||'…';if(end)o.scrollTop=o.scrollHeight}}
let camBusy=false;function camRefresh(){const s=status;const a=s&&s.cam_age;
 $('cam-note').textContent=a===null||a===undefined?'No camera picture yet. The camera runs during laps, or press Camera view.':
  (a<3?'LIVE':'last picture '+Math.round(a)+' s ago')+(s.camera?' · '+s.camera.fps+' fps · '+s.camera.lines+' paint px · '+s.camera.obstacles+' obstacle px':'')+' · cyan = white paint, red = car/obstacle';
 if(a===null||a===undefined||camBusy||(a>3&&$('cam').src))return;camBusy=true;const im=new Image();
 im.onload=()=>{$('cam').src=im.src;camBusy=false};im.onerror=()=>{camBusy=false};im.src='/files/live.png?t='+Date.now()}
async function poll(){try{status=await j('/api/status');camRefresh();pill($('p-conn'),'Pi','connected','ok');render()}
 catch(e){pill($('p-conn'),'Pi','not reachable','bad')}}
async function pollScan(){try{scan=await j('/api/scan')}catch(e){scan={pts:[],fov:scan.fov}}drawRadar();drawBoard()}
async function pollLog(){try{const r=await j('/api/log'),o=$('log'),end=o.scrollTop+o.clientHeight>=o.scrollHeight-5;
 o.textContent=r.lines.join('\n');if(end)o.scrollTop=o.scrollHeight}catch(e){}}
j('/api/arena').then(a=>{arena=a;drawBoard()});poll();pollScan();pollLog();
setInterval(poll,700);setInterval(pollScan,250);setInterval(pollLog,1500);addEventListener('resize',()=>{drawRadar();drawBoard()});
</script></body></html>
"""

if __name__ == '__main__':
    main()
