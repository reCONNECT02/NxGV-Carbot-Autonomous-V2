"""Calibration step 1 (sensor health) checks -- pure, no rclpy, unit tested.

The wizard builds a *snapshot* from what system_monitor, uwb_ranges and the
servo_controller extension already publish (nothing extra is subscribed), and
evaluate() turns it into one row per check with the reason and the fix when it
fails. aggregate() combines the snapshots of a Run into the saved result.
race_supervisor preflight (phase 8, later page) reuses build_checks/evaluate.

snapshot = {
  'health_age_s': float | None,        # age of the last SystemHealth (None = never)
  'topics': {topic: {'hz', 'age', 'latency'}},   # hz < 0 = not watched right now
  'procs': [(label, pid)],             # system_monitor camera_process_* ('astra_camera <name>', ...)
  'agent': bool | None,                # micro-ROS agent process running
  'battery_v': float | None,
  'uwb': {'link': bool, 'hz': float, 'unknown': str,
          'anchors': {id: {'seen': bool, 'age': float}}} | None,
  'env': monitor_core.env_network(...),
}
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional

from carbot_common import topics as T
from carbot_common.data import sensor_enabled

PASS_KEYS = ('camera_rate_min_ratio', 'lidar_min_hz', 'odom_min_hz', 'imu_min_hz', 'uwb_min_hz',
             'battery_min_v', 'max_age_s', 'processes', 'enforce_min_rates')

SENSOR_LABEL = {'astra': 'Astra Pro'}


@dataclass
class Check:
    key: str
    label: str
    kind: str                  # topic | anchors | battery | processes | network
    topic: str = ''
    min_hz: float = 0.0
    expected_hz: float = 0.0
    sensor: str = ''           # camera sensor name (cameras.yaml)
    extra: Dict = field(default_factory=dict)


class ConfigError(ValueError):
    """calibration_steps.yaml / cameras.yaml / uwb.yaml is missing something."""


def _need(d: Dict, keys, where: str) -> None:
    miss = [k for k in keys if k not in (d or {})]
    if miss:
        raise ConfigError(f'{where}: missing {", ".join(miss)}')


def build_checks(cameras: Dict, uwb: Dict, pass_cfg: Dict) -> List[Check]:
    """One Check per row of the step-1 table, from the YAML only."""
    _need(pass_cfg, PASS_KEYS, 'calibration_steps.yaml sensor_health.pass')
    _need(cameras, ('sensors', 'roles', 'roles_confirmed', 'roles_confirmed_for'), 'cameras.yaml')
    _need(uwb, ('anchors', 'agent'), 'uwb.yaml')
    out: List[Check] = []
    enforce = bool(pass_cfg['enforce_min_rates'])
    roles = cameras['roles'] or {}
    for role in T.CAMERA_ROLES:
        sensor = roles.get(role)
        if not sensor or sensor not in cameras['sensors']:
            raise ConfigError(f'cameras.yaml roles.{role} = {sensor!r} is not a sensor')
    ratio = float(pass_cfg['camera_rate_min_ratio'])
    for role in T.CAMERA_ROLES:
        sensor = roles[role]
        if not sensor_enabled(cameras, sensor):
            continue                           # enabled: false -> no row
        s = cameras['sensors'][sensor]
        _need(s, ('image_topic', 'expected_hz'), f'cameras.yaml sensors.{sensor}')
        where = 'front'
        name = SENSOR_LABEL.get(sensor, sensor)
        exp = float(s['expected_hz'])
        out.append(Check(f'cam_{sensor}', f'{name} ({where})', 'topic', s['image_topic'],
                         min_hz=exp * ratio, expected_hz=exp, sensor=sensor,
                         extra={'driver': s.get('driver', ''), 'enforce': enforce}))
    for key, label, topic, k in (('lidar', 'LiDAR · T-mini Plus', T.SCAN, 'lidar_min_hz'),
                                 ('odom', 'Wheel odometry', T.ODOM, 'odom_min_hz'),
                                 ('imu', 'IMU', T.IMU_RPY, 'imu_min_hz'),
                                 ('uwb_tag', 'UWB tag via micro-ROS agent', T.UWB_INPUT_JSON, 'uwb_min_hz')):
        mn = float(pass_cfg[k])
        out.append(Check(key, label, 'topic', topic, min_hz=mn, expected_hz=mn, extra={'enforce': enforce}))
    ids = [str(a['id']) for a in uwb['anchors']]
    out.append(Check('uwb_anchors', 'UWB anchors', 'anchors', extra={'ids': ids}))
    out.append(Check('battery', 'Battery', 'battery', T.VEHICLE_BATTERY,
                     extra={'min_v': float(pass_cfg['battery_min_v'])}))
    pr = pass_cfg['processes'] or {}
    _need(pr, ('astra_pattern', 'forbidden'), 'sensor_health.pass.processes')
    out.append(Check('processes', 'Camera processes (no duplicates)', 'processes',
                     extra={'astra': str(pr['astra_pattern']),
                            'forbidden': [str(x) for x in pr['forbidden']]}))
    out.append(Check('network', 'ROS network settings', 'network',
                     extra={'domain': int(uwb['agent'].get('domain_id', 1))}))
    return out


# ----------------------------------------------------------------------------- evaluation
def _row(c: Check, state: str, measured: str, limit: str, why: str = '', fix: str = '',
         value: Optional[float] = None, detail: str = '') -> Dict:
    return {'key': c.key, 'label': c.label, 'topic': c.topic, 'state': state, 'measured': measured,
            'limit': limit, 'why': why, 'fix': fix, 'value': value, 'detail': detail}


def _topic_fix(c: Check, never: bool, snap: Dict) -> str:
    if c.key.startswith('cam_'):
        return ('Check the Astra Pro USB cable (USB 3 port, not through a hub), then press '
                '"Restart camera drivers". Terminal check: ros2 topic hz /camera/color/image_raw')
    if c.key == 'lidar':
        return ('Check the T-mini Plus USB cable and that it spins. The port is set in drivers.yaml '
                '(ydlidar_ros2_driver_node.port, /dev/serial/by-id/...); run  ls /dev/serial/by-id/ '
                'and compare. Look for ydlidar errors in the launch terminal.')
    if c.key in ('odom', 'imu'):
        return ('Published by the base servo_controller: check  ros2 node list | grep servo_controller, '
                'the USB cable to the expansion board, and that the motor battery switch is ON.')
    if c.key == 'uwb_tag':
        env = snap.get('env') or {}
        if env.get('problems'):
            return 'Fix the ROS network first (row "ROS network settings").'
        if snap.get('agent') is False:
            return ('The micro-ROS agent is not running: look for [run_uwb_agent] errors in the launch '
                    'terminal (agent built in ~/uros_ws? see docs/SETUP.md).')
        return ('Power the ESP32 tag and watch its serial log: "connecting to ..." = wrong WiFi / 5 GHz '
                'hotspot; "no agent at ..." = TagConfig.h MICROROS_AGENT_IP differs from  hostname -I  '
                'on the RDK (UWB_Handoff.md section 13).')
    return ''


def _eval_topic(c: Check, snap: Dict, max_age: float) -> Dict:
    enforce = c.extra.get('enforce', True)
    lim = f'≥ {c.min_hz:.1f} Hz' if enforce else 'publishing'
    if snap.get('health_age_s') is None:
        return _row(c, 'wait', '—', lim, 'No fresh /carbot/system/health from system_monitor',
                    'Wait a few seconds after launch. If it stays like this, check  ros2 node list | grep '
                    'system_monitor  and the launch terminal for its error.')
    t = (snap.get('topics') or {}).get(c.topic)
    if t is None:
        return _row(c, 'bad', '—', lim, f'system_monitor does not watch {c.topic}',
                    'Add it to ops.yaml system_monitor.watch_topics (and watch_expected_hz), then relaunch.')
    hz, age = float(t.get('hz', 0.0)), float(t.get('age', -1.0))
    if hz < 0:
        return _row(c, 'wait', 'not watched', lim, 'system_monitor paused this topic (race armed)', '')
    never = age < 0
    measured = f'{hz:.1f} Hz'
    if never:
        return _row(c, 'bad', 'never', lim, f'No message on {c.topic} since system_monitor started',
                    _topic_fix(c, True, snap), value=0.0)
    if age > max_age:
        return _row(c, 'bad', f'{hz:.1f} Hz', lim,
                    f'Last message {age:.1f} s ago (limit {max_age:.1f} s): the driver stopped',
                    _topic_fix(c, False, snap), value=0.0, detail=f'age {age:.2f} s')
    if hz < c.min_hz and enforce:
        why = f'{hz:.1f} Hz is below {c.min_hz:.1f} Hz'
        fix = ('Too slow: look at the "Camera processes" row (duplicates halve the rate) and CPU in the '
               'System health tab. Then press "Restart camera drivers".') if c.key.startswith('cam_') \
            else _topic_fix(c, False, snap)
        return _row(c, 'bad', measured, lim, why, fix, value=hz, detail=f'age {age:.2f} s')
    lat = t.get('latency', -1.0)
    det = f'age {age:.2f} s' + (f' · latency {lat:.0f} ms' if lat is not None and lat >= 0 else '')
    if hz < c.min_hz:
        det += f' · below {c.min_hz:.1f} Hz (not enforced)'
    return _row(c, 'ok', measured, lim, value=hz, detail=det)


def _eval_anchors(c: Check, snap: Dict) -> Dict:
    ids = c.extra['ids']
    lim = f'{len(ids)} of {len(ids)}'
    u = snap.get('uwb')
    if u is None:
        return _row(c, 'bad', '—', lim, 'No /carbot/uwb/status (uwb_ranges node silent)',
                    'Check  ros2 node list | grep uwb_ranges  and the launch terminal.')
    an = u.get('anchors') or {}
    seen = [a for a in ids if (an.get(a) or {}).get('seen')]
    missing = [a for a in ids if a not in seen]
    measured = f'{len(seen)} of {len(ids)}'
    if not u.get('link'):
        return _row(c, 'bad', measured, lim, 'No UWB data from the tag (agent link down)',
                    'Fix the "UWB tag" row first.', value=float(len(seen)))
    if missing:
        fix = (f'Power anchor{"s" if len(missing) > 1 else ""} {", ".join(missing)} and give it line of sight. '
               'If every anchor works alone but only one appears with all on, the tag lacks the anchor '
               'pre-registration firmware (UWB_Handoff.md 4.6).')
        if u.get('unknown') and u.get('unknown') not in ('', '0000'):
            fix += f' The tag hears unknown anchor {u["unknown"]}: add its ID to RangeProtocol.h ANCHOR_IDS.'
        return _row(c, 'bad', measured, lim, 'Not seen: ' + ', '.join(missing), fix, value=float(len(seen)),
                    detail=' · '.join(ids))
    return _row(c, 'ok', measured, lim, value=float(len(seen)), detail=' · '.join(ids))


def _eval_battery(c: Check, snap: Dict) -> Dict:
    mn = c.extra['min_v']
    v = snap.get('battery_v')
    lim = f'≥ {mn:.1f} V'
    if v is None:
        return _row(c, 'bad', '—', lim, f'No {T.VEHICLE_BATTERY} (servo_controller carbot extension)',
                    'Check servo_controller is running and base_nodes.yaml carbot_battery_rate_hz > 0.')
    if v < mn:
        return _row(c, 'bad', f'{v:.2f} V', lim, f'{v:.2f} V is below {mn:.1f} V',
                    'Charge or swap the battery before calibrating: a low pack changes motor speed and '
                    'servo behaviour, so steps 7 and 8 would be wrong.', value=v)
    return _row(c, 'ok', f'{v:.2f} V', lim, value=v)


def _eval_procs(c: Check, snap: Dict) -> Dict:
    if snap.get('health_age_s') is None:
        return _row(c, 'wait', '—', '0 duplicates', 'Waiting for system_monitor', '')
    procs = snap.get('procs') or []
    astra, forb = c.extra['astra'], c.extra['forbidden']
    problems, count = [], 0
    ap = [p for lab, p in procs if lab.startswith(astra)]
    if len(ap) > 1:
        problems.append(f'{len(ap)} × {astra} (PIDs {", ".join(map(str, ap))})')
        count += len(ap) - 1
    for f in forb:
        fp = [p for lab, p in procs if lab.startswith(f)]
        if fp:
            problems.append(f'{f} running (PID {", ".join(map(str, fp))}), not used by this stack')
            count += len(fp)
    detail = ', '.join(f'{lab} {p}' for lab, p in procs) or 'none found'
    if problems:
        return _row(c, 'bad', str(count), '0', '; '.join(problems),
                    'An older launch or the old websocket viewer is still running. Press "Restart camera '
                    'drivers": it kills stale hobot_codec / websocket / Astra processes first. '
                    'Terminal check: ps -ef | grep -E "codec|websocket|astra" | grep -v grep',
                    value=float(count), detail=detail)
    return _row(c, 'ok', '0', '0', value=0.0, detail=detail)


def _eval_network(c: Check, snap: Dict) -> Dict:
    env = snap.get('env') or {}
    want = c.extra['domain']
    measured = f'domain {env.get("domain_id") or "unset"} · localhost_only {env.get("localhost_only", "?")}'
    lim = f'domain {want} · localhost_only 0'
    if env.get('problems'):
        return _row(c, 'bad', measured, lim, '; '.join(env['problems']),
                    'The launch sets these from uwb.yaml. If this row fails, something overrides them: run '
                    '  grep -rn "ROS_LOCALHOST_ONLY\\|ROS_DOMAIN_ID" ~/.bashrc /etc/profile.d /etc/environment  '
                    'remove the override, open a new terminal, then  ros2 daemon stop  and relaunch.')
    return _row(c, 'ok', measured, lim)


def evaluate(checks: List[Check], snap: Dict, max_age_s: float) -> List[Dict]:
    out = []
    for c in checks:
        if c.kind == 'topic':
            out.append(_eval_topic(c, snap, max_age_s))
        elif c.kind == 'anchors':
            out.append(_eval_anchors(c, snap))
        elif c.kind == 'battery':
            out.append(_eval_battery(c, snap))
        elif c.kind == 'processes':
            out.append(_eval_procs(c, snap))
        elif c.kind == 'network':
            out.append(_eval_network(c, snap))
    return out


def aggregate(samples: List[List[Dict]], min_ok_fraction: float) -> Dict:
    """Combine the rows of every snapshot taken during Run.

    A check passes when it was ok in at least min_ok_fraction of the samples
    (a single dropped message must not fail the step). The reported row is the
    last FAILING sample if the check failed (so Why/Fix describe the problem),
    else the last sample."""
    if not samples:
        return {'passed': False, 'checks': [], 'n_ok': 0, 'n': 0,
                'summary': 'no measurement (system_monitor silent)'}
    keys = [r['key'] for r in samples[-1]]
    rows = []
    for k in keys:
        series = [r for s in samples for r in s if r['key'] == k]
        oks = sum(1 for r in series if r['state'] == 'ok')
        frac = oks / len(series) if series else 0.0
        passed = frac >= min_ok_fraction
        vals = sorted(r['value'] for r in series if isinstance(r.get('value'), (int, float)))
        pick = series[-1] if passed else next((r for r in reversed(series) if r['state'] != 'ok'), series[-1])
        row = dict(pick, passed=passed, ok_fraction=round(frac, 2))
        if vals:
            row['median'] = round(vals[len(vals) // 2], 3)
        rows.append(row)
    n_ok = sum(1 for r in rows if r['passed'])
    return {'passed': n_ok == len(rows) and bool(rows), 'checks': rows, 'n_ok': n_ok, 'n': len(rows),
            'samples': len(samples),
            'summary': f'{n_ok} of {len(rows)} checks passed'}
