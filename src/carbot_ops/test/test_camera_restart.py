"""Camera restart plan + worker with fake subprocesses (no ROS, nothing is killed)."""
import os
import types

from carbot_ops.camera_restart import CameraRestart
from helpers import CAMERAS, OLD_SESSION_CAMERAS


def cfg(tmp_path, **kw):
    d = {'enabled': True, 'kill_patterns': ['hobot_codec'], 'grace_s': 2.0,
         'root_helper_dir': str(tmp_path / 'helpers'), 'settle_s': 0.0,
         'timeout_s': 5.0, 'log_dir': str(tmp_path / 'logs')}
    d.update(kw)
    return d


def install(tmp_path):
    h = tmp_path / 'helpers'
    h.mkdir()
    (h / 'kill_stale.sh').write_text('#!/bin/bash\n')


class FakeProc:
    def __init__(self, alive=True):
        self.alive, self.terminated = alive, False

    def poll(self):
        return None if self.alive else 1

    def terminate(self):
        self.terminated = True


def test_plan_uses_sudo_helper_and_only_the_astra(tmp_path, monkeypatch):
    install(tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    r = CameraRestart(cfg(tmp_path), CAMERAS, '/share', is_root=False)
    p = r.plan('/tmp/p.txt')
    assert p[0]['cmd'][:2] == ['sudo', '-n'] and p[0]['cmd'][2].endswith('kill_stale.sh')
    assert len(p) == 2 and not any('mipi' in s['what'] for s in p)             # kill_stale + the Astra, no MIPI
    assert p[-1]['cmd'][:2] == ['ros2', 'launch'] and p[-1]['cmd'][2:4] == ['carbot_bringup', 'astra_rgb.launch.py']
    assert p[-1]['cmd'][4:] == ['width:=640', 'height:=480', 'fps:=15']       # colour only, from cameras.yaml


def test_refuses_without_helpers(tmp_path):
    r = CameraRestart(cfg(tmp_path), CAMERAS, '/share', is_root=False)
    assert 'install_root_helpers' in r.start()
    assert 'switched off' in CameraRestart(cfg(tmp_path, enabled=False), CAMERAS, '/s', is_root=False).start()


def test_worker_success_and_driver_death(tmp_path):
    install(tmp_path)
    ran, spawned = [], []

    def run(cmd, **kw):
        ran.append(cmd)
        return types.SimpleNamespace(returncode=0, stdout='[kill_stale] no stale processes\n', stderr='')

    def popen(cmd, **kw):
        p = FakeProc(alive=not any('astra' in str(c) for c in cmd))
        spawned.append(p)
        return p
    r = CameraRestart(cfg(tmp_path), CAMERAS, '/share', popen=popen, run=run, is_root=False, sleep=lambda s: None)
    r._do()
    st = r.snapshot()
    assert len(ran) == 1 and len(spawned) == 1
    assert st['state'] == 'failed' and '1 driver' in st['message']
    assert os.path.isdir(tmp_path / 'logs')
    r.stop()
    assert all(p.terminated for p in spawned) and len(ran) == 2       # kill helper again at stop


def test_kill_failure_reported(tmp_path):
    install(tmp_path)
    r = CameraRestart(cfg(tmp_path), CAMERAS, '/share', popen=lambda *a, **k: FakeProc(),
                      run=lambda *a, **k: types.SimpleNamespace(returncode=1, stdout='', stderr='sudo: a password is required'),
                      is_root=False, sleep=lambda s: None)
    r._worker()
    assert r.snapshot()['state'] == 'failed' and 'sudo -n refused' in r.snapshot()['message']


def test_plan_ignores_old_session_side_cameras(tmp_path):
    install(tmp_path)
    what = [s['what'] for s in CameraRestart(cfg(tmp_path), OLD_SESSION_CAMERAS, '/share', is_root=False).plan('/tmp/p.txt')]
    assert not any('mipi' in w or '/cam_' in w for w in what)                # removed cameras are never started
    assert any('Astra' in w for w in what)


def test_plan_skips_a_disabled_astra(tmp_path):
    install(tmp_path)
    cams = dict(CAMERAS, sensors={n: dict(s, enabled=False) for n, s in CAMERAS['sensors'].items()})
    what = [s['what'] for s in CameraRestart(cfg(tmp_path), cams, '/share', is_root=False).plan('/tmp/p.txt')]
    assert not any('Astra' in w for w in what)
