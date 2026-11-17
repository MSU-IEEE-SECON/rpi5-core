"""Dashboard: scan conversion, action whitelist, HTTP endpoints (no ROS needed)."""
import json
import math
import os
import tempfile
import time
import urllib.request

import pytest

from rover_bringup import rover_dashboard as D


def test_rover_points_yaw_and_mirror():
    # one return straight ahead of the LiDAR's raw 0 deg, LiDAR mounted backwards
    pts = D.rover_points(0.0, math.radians(1), [1.0], yaw=math.pi, mirror=False)
    assert pts[0][0] == pytest.approx(-1.0) and abs(pts[0][1]) < 1e-9
    # raw +90 deg, mirrored, yaw 180 -> rover angle 180 - 90 = +90 (left)
    pts = D.rover_points(math.radians(90), 0.01, [0.5], yaw=math.pi, mirror=True)
    assert pts[0][0] == pytest.approx(0.0, abs=1e-9) and pts[0][1] == pytest.approx(0.5)


def test_rover_points_mask_and_range():
    ranges = [0.05, 0.15, 0.5, float('inf'), float('nan')]
    pts = D.rover_points(0.0, math.radians(1), ranges, 0.0, False, 1.0, {1: 0.16})
    assert len(pts) == 1 and pts[0][0] == pytest.approx(0.5 * math.cos(math.radians(2)))


@pytest.fixture
def server(monkeypatch):
    root = tempfile.mkdtemp()
    os.makedirs(os.path.join(root, 'pi_setup'))
    with open(os.path.join(root, 'pi_setup', 'rover.sh'), 'w') as f:
        f.write('echo "ran $1"\n')
    monkeypatch.setattr(D, 'ROOT', root)
    m = D.Model()
    srv = D.serve(m, 0)
    yield m, f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown()


def _post(url, name):
    req = urllib.request.Request(url + '/api/action', json.dumps({'name': name}).encode(),
                                 {'Content-Type': 'application/json'})
    try:
        return json.load(urllib.request.urlopen(req))
    except urllib.error.HTTPError as e:
        return json.load(e)


def test_http_and_actions(server):
    m, url = server
    assert b'Rover' in urllib.request.urlopen(url + '/').read()
    m.on_scan(0.0, math.radians(1), [1.0] * 360)
    sc = json.load(urllib.request.urlopen(url + '/api/scan'))
    assert len(sc['pts']) > 100 and sc['age_s'] < 1.0
    assert len(json.load(urllib.request.urlopen(url + '/api/arena'))['walls']) == len(D.A.WALLS)
    assert _post(url, 'rm -rf')['ok'] is False          # only known commands
    assert _post(url, 'snapshot')['ok'] is True
    for _ in range(50):
        st = json.load(urllib.request.urlopen(url + '/api/status'))
        if st['action']['rc'] is not None:
            break
        time.sleep(0.05)
    assert st['action']['rc'] == 0 and st['action_out'] == ['ran snapshot']


def test_estop_calls_ros_side_then_stops(server):
    m, url = server
    hit = []
    m.estop_cb = lambda: hit.append(1)
    r = _post(url, 'estop')
    assert r['ok'] and hit == [1] and m.action['name'] == 'stop'


def test_set_mirror_writes_calib(server, monkeypatch, tmp_path):
    m, url = server
    from rover_bringup import lidar_calib
    path = str(tmp_path / 'lidar_calib.json')
    monkeypatch.setattr(lidar_calib, 'DEFAULT_PATH', path)
    monkeypatch.setattr(lidar_calib.save, '__defaults__', (path,))
    monkeypatch.setattr(lidar_calib.load, '__defaults__', (path,))
    req = urllib.request.Request(url + '/api/calib', json.dumps({'laser_mirror': False}).encode())
    assert json.load(urllib.request.urlopen(req))['ok']
    assert json.load(open(path))['laser_mirror'] is False and m.mirror is False


def _bundle(entries):
    import io
    import tarfile
    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode='w:gz') as tf:
        for name, data in entries:
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tf.addfile(info, io.BytesIO(data))
    return buf.getvalue()


def test_install_bundle_installs_and_backs_up(tmp_path):
    root, keep = tmp_path / 'root', tmp_path / 'keep'
    (root / 'pi_setup').mkdir(parents=True)
    (root / 'pi_setup' / 'rover.sh').write_text('old')
    ok, msg, files = D.install_bundle(_bundle([('pi_setup/rover.sh', b'new'),
                                               ('ros2_ws/src/x/a.py', b'print(1)'),
                                               ('VERSION', b'v1\n')]), str(root), str(keep))
    assert ok, msg
    assert (root / 'pi_setup' / 'rover.sh').read_text() == 'new'
    assert (root / 'ros2_ws' / 'src' / 'x' / 'a.py').exists()
    assert D.deployed_version(str(root)) == 'v1'
    assert any(p.name.startswith('backup_') for p in keep.iterdir())


def test_install_bundle_refuses_bad_paths(tmp_path):
    for bad in ('../evil.sh', '/etc/passwd', 'home/rover/.bashrc'):
        ok, msg, _ = D.install_bundle(_bundle([(bad, b'x')]), str(tmp_path), str(tmp_path / 'k'))
        assert not ok and 'refusing' in msg
    ok, msg, _ = D.install_bundle(b'not a tarball', str(tmp_path), str(tmp_path / 'k'))
    assert not ok
