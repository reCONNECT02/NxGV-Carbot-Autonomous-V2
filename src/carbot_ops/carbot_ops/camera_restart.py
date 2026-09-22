"""'Restart camera drivers' (calibration step 1 button).

Same helpers and the same rules as the launch (carbot_bringup/stack.py):
  1. kill_stale.sh with the CAMERA patterns only (mipi_cam, hobot_codec,
     websocket, the Astra container) -- Camera_Setup.md: Ctrl+C does not always
     kill them, and a stale mipi_cam makes the next one fail with
     "There are no available host".
  2. run_mipi_cam.sh per MIPI sensor (as ROOT through the sudoers-allowed copy
     in root_helper_dir; image_width/height always passed; DDS settings passed
     as arguments because sudo drops the environment), second camera delayed.
  3. The Astra through its base launch file (user, not root).

The restarted drivers are children of calibration_wizard, so Ctrl+C in the
launch terminal stops them too; if the wizard exits any other way, stop()
runs the kill helper again, and the next launch's kill_stale cleans anything left.
"""
import os
import subprocess
import tempfile
import threading
import time
from typing import Callable, Dict, List, Optional

from carbot_common.data import sensor_enabled

CFG_KEYS = ('enabled', 'kill_patterns', 'grace_s', 'root_helper_dir', 'delay_mipi_second_s',
            'settle_s', 'timeout_s', 'log_dir')


class CameraRestart:

    def __init__(self, cfg: Dict, cameras: Dict, share_scripts: str,
                 popen: Callable = subprocess.Popen, run: Callable = subprocess.run,
                 is_root: Optional[bool] = None, sleep: Callable = time.sleep):
        miss = [k for k in CFG_KEYS if k not in cfg]
        if miss:
            raise ValueError('ops.yaml calibration_wizard.restart_cameras: missing ' + ', '.join(miss))
        self.cfg, self.cameras, self.share = cfg, cameras, share_scripts
        self.popen, self.run, self.sleep = popen, run, sleep
        self.is_root = (os.geteuid() == 0) if is_root is None else is_root
        self.children: List = []
        self.logs: List = []
        self.lock = threading.Lock()
        self.state = {'name': 'restart_cameras', 'state': 'idle', 'message': '', 'log': [], 'started': 0.0}

    # ------------------------------------------------------------------ commands
    def helper(self, script: str, args: List[str]) -> List[str]:
        d = str(self.cfg['root_helper_dir'] or '')
        inst = os.path.join(d, script) if d else ''
        if inst and os.path.isfile(inst):
            return ([] if self.is_root else ['sudo', '-n']) + [inst] + [str(a) for a in args]
        return ['bash', os.path.join(self.share, script)] + [str(a) for a in args]

    def helpers_installed(self) -> bool:
        d = str(self.cfg['root_helper_dir'] or '')
        return self.is_root or (bool(d) and os.path.isfile(os.path.join(d, 'kill_stale.sh'))
                                and os.path.isfile(os.path.join(d, 'run_mipi_cam.sh')))

    def mipi_sensors(self) -> List[Dict]:
        out = []
        for name, s in (self.cameras.get('sensors') or {}).items():
            if s.get('driver') != 'mipi_cam' or not sensor_enabled(self.cameras, name):
                continue
            for k in ('namespace', 'channel', 'image_width', 'image_height'):
                if k not in s:
                    raise ValueError(f'cameras.yaml sensors.{name}.{k} missing (image_width/height must ALWAYS be set)')
            out.append(dict(s, name=name))
        return sorted(out, key=lambda s: int(s['channel']))

    def plan(self, patterns_file: str) -> List[Dict]:
        env = os.environ
        steps = [{'what': 'kill stale camera processes', 'kind': 'wait',
                  'cmd': self.helper('kill_stale.sh', [patterns_file, self.cfg['grace_s']])}]
        for i, s in enumerate(self.mipi_sensors()):
            steps.append({'what': f'start mipi_cam {s["namespace"]} (MIPI ch {s["channel"]}, as root)',
                          'kind': 'spawn', 'delay': float(self.cfg['delay_mipi_second_s']) if i else 0.0,
                          'log': f'mipi_{s["name"]}',
                          'cmd': self.helper('run_mipi_cam.sh', [
                              s['namespace'], s['channel'], s['image_width'], s['image_height'],
                              env.get('ROS_DOMAIN_ID', '0'), env.get('ROS_LOCALHOST_ONLY', '0'),
                              env.get('FASTRTPS_DEFAULT_PROFILES_FILE', ''), ''])})
        astra = (self.cameras.get('sensors') or {}).get('astra')
        if astra and sensor_enabled(self.cameras, 'astra'):
            steps.append({'what': 'start Astra Pro (base launch file)', 'kind': 'spawn', 'delay': 0.0,
                          'log': 'astra',
                          'cmd': ['ros2', 'launch', 'astra_camera', astra.get('launch_file', 'astra_mini.launch.py')]})
        return steps

    # ------------------------------------------------------------------ run
    def _log(self, line: str) -> None:
        with self.lock:
            self.state['log'] = (self.state['log'] + [line])[-30:]

    def _set(self, state: str, message: str) -> None:
        with self.lock:
            self.state['state'], self.state['message'] = state, message

    def snapshot(self) -> Dict:
        with self.lock:
            return {k: (list(v) if isinstance(v, list) else v) for k, v in self.state.items()}

    def busy(self) -> bool:
        return self.snapshot()['state'] == 'running'

    def start(self) -> Optional[str]:
        """Start in a background thread. Returns an error message to refuse."""
        if not bool(self.cfg['enabled']):
            return 'Restart camera drivers is switched off (ops.yaml restart_cameras.enabled)'
        if self.busy():
            return 'A camera restart is already running'
        if not self.helpers_installed():
            return ('Root helpers are not installed, so root-owned mipi_cam processes cannot be stopped or started '
                    'from here. Run once:  sudo bash tools/setup/install_root_helpers.sh sunrise  then relaunch.')
        with self.lock:
            self.state.update({'state': 'running', 'message': 'starting', 'log': [], 'started': time.time()})
        threading.Thread(target=self._worker, daemon=True).start()
        return None

    def _worker(self) -> None:
        try:
            self._do()
        except Exception as e:  # noqa: BLE001  report, never crash the wizard
            self._log(f'ERROR {e!r}')
            self._set('failed', f'Restart failed: {e}')

    def _do(self) -> None:
        fd, pat = tempfile.mkstemp(prefix='carbot_cam_patterns_', suffix='.txt')
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write('\n'.join(str(p) for p in self.cfg['kill_patterns']) + '\n')
        os.makedirs(str(self.cfg['log_dir']), exist_ok=True)
        self._drop_children()               # our own earlier restart: stop it cleanly first
        for st in self.plan(pat):
            if st.get('delay'):
                self.sleep(st['delay'])
            self._log(f'{st["what"]}: {" ".join(st["cmd"])}')
            self._set('running', st['what'])
            if st['kind'] == 'wait':
                r = self.run(st['cmd'], capture_output=True, text=True, timeout=float(self.cfg['timeout_s']))
                for line in ((r.stdout or '') + (r.stderr or '')).splitlines()[-8:]:
                    self._log('  ' + line)
                if r.returncode != 0:
                    raise RuntimeError(f'kill_stale.sh exited {r.returncode} (sudo -n refused? re-run '
                                       'tools/setup/install_root_helpers.sh)')
            else:
                logf = open(os.path.join(str(self.cfg['log_dir']), f'camera_restart_{st["log"]}.log'), 'w')
                p = self.popen(st['cmd'], stdout=logf, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
                with self.lock:
                    self.children.append(p)
                    self.logs.append(logf)
        self._set('running', 'waiting for the drivers to publish')
        self.sleep(float(self.cfg['settle_s']))
        dead = [c for c in self.children if c.poll() is not None]
        try:
            os.remove(pat)
        except OSError:
            pass
        if dead:
            self._set('failed', f'{len(dead)} driver(s) exited right away: see {self.cfg["log_dir"]}/camera_restart_*.log '
                                '(mipi: "There are no available host" = still in use; "create_and_run_vflow" = not root)')
        else:
            self._set('done', 'Drivers restarted. Check the camera rows turn green (about 5 s).')

    def _drop_children(self) -> int:
        with self.lock:
            kids, self.children = self.children, []
            logs, self.logs = self.logs, []
        for p in kids:
            try:
                p.terminate()
            except Exception:  # noqa: BLE001
                pass
        for f in logs:
            try:
                f.close()
            except Exception:  # noqa: BLE001
                pass
        return len(kids)

    def stop(self) -> None:
        """Wizard shutdown: stop the drivers we started (root ones via the kill helper)."""
        if not self._drop_children():
            return
        try:
            fd, pat = tempfile.mkstemp(prefix='carbot_cam_patterns_', suffix='.txt')
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                f.write('\n'.join(str(p) for p in self.cfg['kill_patterns']) + '\n')
            self.run(self.helper('kill_stale.sh', [pat, self.cfg['grace_s']]), capture_output=True,
                     timeout=float(self.cfg['timeout_s']))
            os.remove(pat)
        except Exception:  # noqa: BLE001  best effort at shutdown
            pass
