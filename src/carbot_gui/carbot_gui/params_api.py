"""Tuning tab backend (calibrate mode only).

Pattern taken from the base dashboard (risabot_automode/dashboard.py,
_get_client / _ros_get_param / _ros_set_param): a dedicated helper node with
its own executor thread does the parameter service calls, so camera or map
callbacks in the GUI node never block on them.

* catalogue(): every key of carbot_bringup/config/params/*.yaml, per file and
  node (nested keys dotted), with the session overlay value if any.
* get_live(node, names) / set_live(node, name, value): typed like the live
  value (no string guessing), so a double stays a double.
* save(session, node, name, value): merges into <session>/params_overlay.yaml,
  which the launch applies last (stack.param_file_list).
"""
import os
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import yaml

SKIP_FILES = ('drivers.yaml',)          # launch-time settings, not live node parameters


def flatten(d: Dict, prefix: str = '') -> Dict[str, Any]:
    out = {}
    for k, v in (d or {}).items():
        key = f'{prefix}{k}'
        if isinstance(v, dict):
            out.update(flatten(v, key + '.'))
        else:
            out[key] = v
    return out


def nest_set(d: Dict, dotted: str, value: Any) -> None:
    parts = dotted.split('.')
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = value


def load_overlay(session: str) -> Dict:
    path = os.path.join(session, 'params_overlay.yaml') if session else ''
    if path and os.path.isfile(path):
        with open(path, encoding='utf-8') as f:
            return yaml.safe_load(f) or {}
    return {}


def catalogue(params_dir: str, session: str = '') -> List[Dict]:
    """[{file, node, key, default, saved}] for every parameter in the repo YAML."""
    overlay = load_overlay(session)
    rows = []
    for fn in sorted(os.listdir(params_dir)):
        if not fn.endswith('.yaml') or fn in SKIP_FILES:
            continue
        with open(os.path.join(params_dir, fn), encoding='utf-8') as f:
            doc = yaml.safe_load(f) or {}
        for node, body in doc.items():
            if not isinstance(body, dict) or node.startswith('/'):
                continue                    # '/**' common block: read-only (applies to every node)
            flat = flatten(body.get('ros__parameters', {}))
            ov = flatten((overlay.get(node) or {}).get('ros__parameters', {}))
            for k, v in flat.items():
                rows.append({'file': fn, 'node': node, 'key': k, 'default': v,
                             'saved': ov.get(k, v), 'overlay': k in ov})
    return rows


def save(session: str, node: str, key: str, value: Any) -> str:
    if not session or not os.path.isdir(session):
        raise RuntimeError('No calibration session is loaded: start one in the Calibration tab first')
    doc = load_overlay(session)
    body = doc.setdefault(node, {}).setdefault('ros__parameters', {})
    nest_set(body, key, value)
    path = os.path.join(session, 'params_overlay.yaml')
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
    os.replace(tmp, path)
    return path


def coerce(value: Any, like: Any) -> Any:
    """Convert a browser value to the type of the current value."""
    if isinstance(like, bool):
        return value if isinstance(value, bool) else str(value).strip().lower() in ('1', 'true', 'yes', 'on')
    if isinstance(like, int):
        return int(float(value))
    if isinstance(like, float):
        return float(value)
    if isinstance(like, list):
        if isinstance(value, str):
            value = yaml.safe_load(value)
        if not isinstance(value, list):
            raise ValueError('expected a list')
        if like and isinstance(like[0], float):
            return [float(x) for x in value]
        return value
    return str(value)


class ParamClient:
    """Parameter services through a helper node spun in its own thread."""

    def __init__(self, name: str = 'gui_param_helper'):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        self.node = rclpy.create_node(name)
        self.ex = SingleThreadedExecutor()
        self.ex.add_node(self.node)
        self.lock = threading.Lock()
        self.clients = {}
        threading.Thread(target=self.ex.spin, daemon=True).start()

    def _client(self, node: str, kind: str):
        from rcl_interfaces.srv import GetParameters, SetParameters
        n = node if node.startswith('/') else '/' + node
        with self.lock:
            key = (n, kind)
            if key not in self.clients:
                t, s = (GetParameters, '/get_parameters') if kind == 'get' else (SetParameters, '/set_parameters')
                self.clients[key] = self.node.create_client(t, n + s)
            return self.clients[key]

    @staticmethod
    def _wait(fut, timeout: float):
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def get_live(self, node: str, names: List[str], timeout: float = 1.5) -> Tuple[Optional[Dict], str]:
        from rcl_interfaces.srv import GetParameters
        from rclpy.parameter import parameter_value_to_python
        c = self._client(node, 'get')
        if not c.wait_for_service(timeout_sec=0.2):
            return None, f'/{node} is not running'
        res = self._wait(c.call_async(GetParameters.Request(names=list(names))), timeout)
        if res is None:
            return None, 'timeout'
        return {n: parameter_value_to_python(v) for n, v in zip(names, res.values)}, ''

    def set_live(self, node: str, name: str, value: Any, timeout: float = 1.5) -> Tuple[bool, str]:
        from rcl_interfaces.srv import SetParameters
        from rclpy.parameter import Parameter
        c = self._client(node, 'set')
        if not c.wait_for_service(timeout_sec=0.2):
            return False, f'/{node} is not running'
        p = Parameter(name, value=value).to_parameter_msg()
        res = self._wait(c.call_async(SetParameters.Request(parameters=[p])), timeout)
        if res is None:
            return False, 'timeout'
        r = res.results[0]
        return bool(r.successful), r.reason or ('ok' if r.successful else 'rejected')

    def destroy(self):
        self.ex.shutdown()
        self.node.destroy_node()
