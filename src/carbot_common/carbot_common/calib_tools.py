"""Shared plumbing for the phase-3 calibration CLIs (steps 6, 10, 11) and the
phase-8 wizard. Same session layout as carbot_perception.calib_io (steps 3-4):

  <session>/NN_<step_id>.yaml      result of one step
  <session>/params_overlay.yaml    ROS parameter overrides (node-namespaced, last wins)
  <session>/data/<file>.yaml       data-file copies (uwb.yaml, cameras.yaml)
  <session>/captures/...           raw data used (for Redo / offline replay)
  <session>/summary.yaml           step status (calibration_store.update_step)
"""
import argparse
import copy
import json
import os
import shutil
from typing import Any, Dict, Iterable, List, Optional

import yaml

from . import calibration_store as cs


def bringup_config_dir(override: str = '') -> str:
    if override:
        return override
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('carbot_bringup'), 'config')
    except Exception:  # noqa: BLE001  source checkout without ROS
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, '..', '..', 'carbot_bringup', 'config'))


def common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument('--session', default='',
                   help='calibration session folder name (default: the ACTIVE session if it '
                        'exists, else a new timestamped one). Use the same name for every step.')
    p.add_argument('--new-session', action='store_true', help='always start a new session')
    p.add_argument('--data-root', default='', help='default $CARBOT_DATA or /home/sunrise/carbot_data')
    p.add_argument('--config-dir', default='', help='carbot_bringup/config (auto)')
    p.add_argument('--activate', action='store_true',
                   help='make this session ACTIVE so the launch files load it')
    p.add_argument('--yes', action='store_true', help='non-interactive: accept defaults')


def open_session_from_args(a) -> str:
    root = cs.data_root(a.data_root)
    if a.session:
        return cs.open_session(root, a.session)
    active = cs.active_session(root)
    if active and not a.new_session:
        return active
    return cs.open_session(root)


def load_yaml(path: str) -> Dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def step_cfg(config_dir: str, step_id: str) -> Dict:
    steps = load_yaml(os.path.join(config_dir, 'data', 'calibration_steps.yaml'))
    for s in steps.get('steps', []):
        if s['id'] == step_id:
            return s
    raise KeyError(f'calibration step {step_id} not in calibration_steps.yaml')


def floor_board_dicts(target: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Step-4 `target` -> board dicts with `centre_m` in base_link (what FloorBoard, the solver,
    the sheet generator and the layout picture all use).

    In calibration_steps.yaml `boards[].centre_m` is the board centre on the printed layout,
    measured from the layout's reference line (the rear-axle line when axle_offset_m is 0).
    The car's rear axle is placed `axle_offset_m` BEHIND that line (2026-09-24: 0.150, so the
    front camera sees its whole board), which puts every board `axle_offset_m` further FORWARD of
    the axle: base_link x = centre_m x + axle_offset_m. y and yaw do not change. `axle_offset_m` is
    required (a missing key is an error). The result also carries `sheet_centre_m` (the YAML value)."""
    if 'axle_offset_m' not in target:
        raise KeyError('calibration_steps.yaml extrinsics_ipm.target.axle_offset_m is missing '
                       '(metres the rear axle sits behind the sheet reference line; 0.0 = as printed)')
    off = float(target['axle_offset_m'])
    out = []
    for b in target['boards']:
        d = copy.deepcopy(b)
        cx, cy = (float(v) for v in b['centre_m'])
        d['sheet_centre_m'] = [cx, cy]
        d['centre_m'] = [round(cx + off, 6), cy]
        out.append(d)
    return out


def effective_data(session: str, config_dir: str, root: str, fname: str) -> Dict:
    """This session's copy of a data file, else the ACTIVE session's, else the repo default."""
    own = os.path.join(session, 'data', fname)
    if os.path.isfile(own):
        return load_yaml(own)
    src = cs.data_override(cs.active_session(root), fname) or os.path.join(config_dir, 'data', fname)
    print(f'[calib] {fname}: starting from {src}')
    return copy.deepcopy(load_yaml(src))


def save_data(session: str, fname: str, doc: Dict) -> str:
    path = os.path.join(session, 'data', fname)
    if os.path.isfile(path):
        shutil.copy2(path, path + '.bak')
    cs.write_yaml(path, doc)
    return path


def merge_data(session: str, fname: str, base_doc: Dict, updates: Dict) -> str:
    """Set `updates` (deep-merged) in <session>/data/<fname>.

    A session data file REPLACES the repo file when the session is loaded, so it
    must be complete: start from this session's copy if an earlier step already
    wrote one (keeps that step's keys), else from `base_doc` (the file the
    launch loaded)."""
    own = os.path.join(session, 'data', fname)
    doc = load_yaml(own) if os.path.isfile(own) else copy.deepcopy(base_doc)
    return save_data(session, fname, _deep_merge(doc, copy.deepcopy(updates)))


def _deep_merge(dst: Dict, src: Dict) -> Dict:
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def _nest(flat: Dict[str, Any]) -> Dict:
    """{'imu.yaw': 1} -> {'imu': {'yaw': 1}} (ROS parameter-file nesting)."""
    out: Dict = {}
    for k, v in flat.items():
        cur = out
        parts = k.split('.')
        for p in parts[:-1]:
            cur = cur.setdefault(p, {})
        cur[parts[-1]] = v
    return out


def merge_overlay(session: str, node: str, params: Dict[str, Any]) -> str:
    """Add ROS parameter overrides for `node` to <session>/params_overlay.yaml."""
    path = os.path.join(session, 'params_overlay.yaml')
    doc = load_yaml(path) if os.path.isfile(path) else {}
    _deep_merge(doc.setdefault(node, {}).setdefault('ros__parameters', {}), _nest(params))
    cs.write_yaml(path, doc)
    return path


def overlay_value(session: str, node: str, name: str, default=None):
    path = os.path.join(session, 'params_overlay.yaml')
    if not os.path.isfile(path):
        return default
    cur = (load_yaml(path).get(node) or {}).get('ros__parameters') or {}
    for p in name.split('.'):
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur


def write_step(session: str, index: int, step_id: str, doc: Dict) -> str:
    fname = f'{index:02d}_{step_id}.yaml'
    cs.write_yaml(os.path.join(session, fname), doc)
    return fname


def capture_path(session: str, name: str) -> str:
    d = os.path.join(session, 'captures')
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, name)


def write_jsonl(path: str, rows: Iterable[Dict]) -> None:
    with open(path, 'w', encoding='utf-8') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')


def read_jsonl(path: str) -> List[Dict]:
    with open(path, 'r', encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def ask(prompt: str, default: str = '', yes: bool = False) -> str:
    if yes:
        print(f'{prompt} [{default}] (auto)')
        return default
    try:
        v = input(f'{prompt} [{default}]: ').strip()
    except EOFError:
        return default
    return v or default


def pause(prompt: str, yes: bool = False) -> None:
    if yes:
        return
    try:
        input(prompt + '  (Enter) ')
    except EOFError:
        pass


def finish(session: str, root: str, step_id: str, passed: bool, fname: str,
           activate: bool, **extra) -> None:
    cs.update_step(session, step_id, 'PASS' if passed else 'FAIL', fname, **extra)
    if activate:
        cs.set_active(root, session)
    print(f'\n[{step_id}] {"PASS" if passed else "FAIL"}  -> {os.path.join(session, fname)}')
    if activate:
        print(f'[{step_id}] session {os.path.basename(session)} is now ACTIVE')


# --------------------------------------------------------------------------- ROS helpers
class RemoteParams:
    """Read / set parameters of another running node (e.g. servo_controller)."""

    def __init__(self, node, target: str, timeout: float = 3.0):
        from rcl_interfaces.srv import GetParameters, SetParameters
        self.node, self.target, self.timeout = node, target.lstrip('/'), timeout
        self.get_cli = node.create_client(GetParameters, f'/{self.target}/get_parameters')
        self.set_cli = node.create_client(SetParameters, f'/{self.target}/set_parameters')

    def _call(self, cli, req):
        import rclpy
        if not cli.wait_for_service(timeout_sec=self.timeout):
            return None
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout)
        return fut.result()

    def get(self, names: List[str]) -> Optional[Dict[str, Any]]:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        res = self._call(self.get_cli, GetParameters.Request(names=names))
        if res is None:
            return None
        return {n: parameter_value_to_python(v) for n, v in zip(names, res.values)}

    def set(self, values: Dict[str, Any]) -> bool:
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        req = SetParameters.Request(parameters=[Parameter(k, value=v).to_parameter_msg()
                                                for k, v in values.items()])
        res = self._call(self.set_cli, req)
        return res is not None and all(r.successful for r in res.results)


def check_uwb_env(uwb_doc: Dict) -> List[str]:
    """Environment problems that make /uwb3/input_json invisible (UWB_Handoff section 8)."""
    agent = uwb_doc.get('agent') or {}
    want = str(agent.get('domain_id', 1))
    probs = []
    if os.environ.get('ROS_LOCALHOST_ONLY', '0') == '1':
        probs.append('ROS_LOCALHOST_ONLY=1 in this terminal: run  export ROS_LOCALHOST_ONLY=0 && ros2 daemon stop')
    if os.environ.get('ROS_DOMAIN_ID', '0') != want:
        probs.append(f'ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "(unset = 0)")} but the tag uses '
                     f'{want}: run  export ROS_DOMAIN_ID={want}')
    for p in probs:
        print('ENV ERROR:', p)
    return probs
