"""Camera restart plan + worker with fake subprocesses (no ROS, nothing is killed)."""
import os
import types

from carbot_ops.camera_restart import CameraRestart
from helpers import CAMERAS


def cfg(tmp_path, **kw):
    d = {'enabled': True, 'kill_patterns': ['mipi_cam', 'hobot_codec'], 'grace_s': 2.0,
         'root_helper_dir': str(tmp_path / 'helpers'), 'delay_mipi_second_s': 2.0, 'settle_s': 0.0,
         'timeout_s': 5.0, 'log_dir': str(tmp_path / 'logs')}
    d.update(kw)
    return d


def install(tmp_path):
    h = tmp_path / 'helpers'
    h.mkdir()
    for f in ('kill_stale.sh', 'run_mipi_cam.sh'):
        (h / f).write_text('#!/bin/bash\n')


class FakeProc:
    def __init__(self, alive=True):
        self.alive, self.terminated = alive, False

    def poll(self):
        return None if self.alive else 1

    def terminate(self):
        self.terminated = True


def test_plan_uses_sudo_helpers_and_explicit_size(tmp_path, monkeypatch):
    install(tmp_path)
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    r = CameraRestart(cfg(tmp_path), CAMERAS, '/share', is_root=False)
    p = r.plan('/tmp/p.txt')
    assert p[0]['cmd'][:2] == ['sudo', '-n'] and p[0]['cmd'][2].endswith('kill_stale.sh')
    mipi = [s for s in p if 'mipi_cam' in s['what']]
    assert [m['cmd'][3] for m in mipi] == ['/cam_imx219', '/cam_ov5647']      # channel 0 then 2
    assert mipi[0]['cmd'][5:7] == ['960', '544'] and mipi[0]['cmd'][7] == '1'
    assert mipi[1]['delay'] == 2.0
    assert p[-1]['cmd'][:3] == ['ros2', 'launch', 'astra_camera']


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
        p = FakeProc(alive='astra_camera' not in cmd)
        spawned.append(p)
        return p
    r = CameraRestart(cfg(tmp_path), CAMERAS, '/share', popen=popen, run=run, is_root=False, sleep=lambda s: None)
    r._do()
    st = r.snapshot()
    assert len(ran) == 1 and len(spawned) == 3
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
