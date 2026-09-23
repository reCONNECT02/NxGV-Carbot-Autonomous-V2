"""Calibration step 8 - speed feedforward + PID (calibration_steps.yaml id speed_pid).

Runs with calibrate.launch.py up. The car DRIVES ITSELF straight, forward and
back alternately (so it stays in about 1.5 m): keep the e-stop ready.

  sweep   CALIBRATION_RAW duty steps (procedure.sweep_duties, alternating
          direction), steady /odom speed per step -> least-squares feedforward
          duty = static_duty + duty_per_mps * |v|  (set LIVE on command_owner)
  verify  CALIBRATION closed-loop steps to procedure.step_targets_mps through
          the owner's PID: steady error and overshoot; a failing run retunes
          (overshoot: kp x0.7 ki x0.8; steady error: ki x1.5) and repeats
          (max_runs).
Writes command_owner.speed_pid.* / feedforward.* and the tunnel_bridge copy.
Note: servo_controller reports speeds below odom_velocity_deadband (0.02 m/s)
as 0, so creep speeds under 2 cm/s cannot be measured.

  ros2 run carbot_control calib_speed
  ros2 run carbot_control calib_speed --replay <session>/captures/speed.json
"""
import argparse
import json

from carbot_common import calib_tools as ct
from carbot_common import calibration_store as cs

from . import calib_core as cc

STEP_ID = 'speed_pid'
OWNER, BRIDGE = 'command_owner', 'tunnel_bridge'
FF_KEYS = ('duty_per_mps', 'static_duty')
PID_KEYS = ('kp', 'ki', 'kd', 'integral_limit')
# params_overlay keys this step writes (calibration_steps.yaml speed_pid.writes); also used
# by the wizard page (carbot_ops.step_speed_pid) for Save and Keep previous
OVERLAY_KEYS = {OWNER: [f'feedforward.{k}' for k in FF_KEYS] + [f'speed_pid.{k}' for k in PID_KEYS],
                BRIDGE: [f'command_owner_feedforward.{k}' for k in FF_KEYS]}


def overlays(ff: dict, pid: dict) -> dict:
    """{node: {param: value}} written for a feedforward fit + PID gains (ff rounded to 4 places)."""
    f = {'feedforward.duty_per_mps': round(ff['duty_per_mps'], 4),
         'feedforward.static_duty': round(ff['static_duty'], 4)}
    return {OWNER: dict(f, **{f'speed_pid.{k}': v for k, v in pid.items()}),
            BRIDGE: {'command_owner_' + k: v for k, v in f.items()}}


def write_overlay(session: str, ff: dict, pid: dict) -> str:
    """Merge overlays(ff, pid) into <session>/params_overlay.yaml. -> its path."""
    path = ''
    for node, params in overlays(ff, pid).items():
        path = ct.merge_overlay(session, node, params)
    return path


def analyse(cap: dict, cfg: dict) -> dict:
    ff = cc.fit_feedforward([tuple(s) for s in cap['sweep']])
    runs = cap.get('verify', [])
    last = runs[-1] if runs else {'metrics': []}
    metrics = [cc.StepMetrics(**m) for m in last['metrics']]
    verdict = cc.pid_verdict(metrics, ff, cfg['pass'])
    return {'feedforward': {'duty_per_mps': ff.duty_per_mps, 'static_duty': ff.static_duty,
                            'fit_residual_mps': ff.residual_mps, 'points': ff.points, 'reason': ff.reason},
            'pid': last.get('pid', cap.get('pid_start')), 'verify_runs': runs, **verdict}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ct.common_args(ap)
    ap.add_argument('--replay', default='', help='capture JSON from an earlier run (no ROS)')
    a = ap.parse_args(argv)
    cfg_dir = ct.bringup_config_dir(a.config_dir)
    cfg = ct.step_cfg(cfg_dir, STEP_ID)
    root = cs.data_root(a.data_root)
    session = ct.open_session_from_args(a)
    if a.replay:
        cap = json.load(open(a.replay))
    else:
        cap = capture(a, cfg)
        if cap is None:
            return 1
        with open(ct.capture_path(session, 'speed.json'), 'w') as f:
            json.dump(cap, f, indent=1)
    res = analyse(cap, cfg)
    for k, v in res['checks'].items():
        print(f'  {k:16s} {"PASS" if v else "FAIL"}')
    ff, pid = res['feedforward'], res['pid']
    print(f'  feedforward duty = {ff["static_duty"]:.3f} + {ff["duty_per_mps"]:.3f} * |v|;  pid {pid}')
    if ff['duty_per_mps'] > 0:
        write_overlay(session, ff, pid)
    fname = ct.write_step(session, int(cfg['index']), STEP_ID, cc.plain(dict(res, step=STEP_ID)))
    ct.finish(session, root, STEP_ID, bool(res['passed']), fname, a.activate)
    return 0 if res['passed'] else 2


def capture(a, cfg):
    import rclpy
    from .calib_drive import Driver
    proc, pas = cfg['procedure'], cfg['pass']
    rclpy.init()
    drv = Driver(rclpy, 'carbot_calib_speed')
    owner = ct.RemoteParams(drv.param_node, 'command_owner')
    try:
        if (owner.get(['mode']) or {}).get('mode') != 'calibrate':
            print('command_owner is not in calibrate mode: start  ros2 launch carbot_bringup calibrate.launch.py')
            return None
        pid = owner.get(['speed_pid.kp', 'speed_pid.ki', 'speed_pid.kd', 'speed_pid.integral_limit']) or {}
        pid = {k.split('.')[-1]: float(v) for k, v in pid.items()}
        cap = {'sweep': [], 'verify': [], 'pid_start': dict(pid)}
        hold, steady = float(proc['hold_s']), float(proc['steady_s'])
        ct.pause('Car on a straight clear floor (1.5 m free ahead AND behind), e-stop ready. Duty sweep', a.yes)
        for i, duty in enumerate(float(d) for d in proc['sweep_duties']):
            d = duty if i % 2 == 0 else -duty
            r = drv.run('CALIBRATION_RAW', d, 0.0, f'step 8 sweep {d:+.2f}', lambda dd, y, el: el >= hold,
                        hold + 1.0)
            if r['aborted']:
                print(f'[sweep] aborted: {r["aborted"]}')
                return None
            rows = r['rows']
            v = cc.steady_speed([q['t'] for q in rows], [q['v'] for q in rows], rows[-1]['t'] if rows else 0,
                                steady) if rows else 0.0
            # steady window = the last `steady` seconds BEFORE the stop command
            drive_rows = [q for q in rows if q['cmd'][1] != 0.0]
            if drive_rows:
                v = cc.steady_speed([q['t'] for q in drive_rows], [q['v'] for q in drive_rows],
                                    drive_rows[-1]['t'], steady)
            cap['sweep'].append([d, v])
            print(f'[sweep] duty {d:+.3f} -> {v:+.3f} m/s')
        ff = cc.fit_feedforward([tuple(s) for s in cap['sweep']])
        if not ff.ok:
            print(f'[sweep] {ff.reason}')
            return cap
        owner.set({'feedforward.duty_per_mps': ff.duty_per_mps, 'feedforward.static_duty': ff.static_duty})
        print(f'[sweep] feedforward set live: static {ff.static_duty:.3f}, {ff.duty_per_mps:.3f} duty per m/s')
        for run in range(int(proc['max_runs'])):
            owner.set({f'speed_pid.{k}': v for k, v in pid.items()})
            metrics = []
            for i, tgt in enumerate(float(x) for x in proc['step_targets_mps']):
                tg = tgt if i % 2 == 0 else -tgt
                r = drv.run('CALIBRATION', tg, 0.0, f'step 8 verify {tg:+.2f}', lambda dd, y, el: el >= hold,
                            hold + 1.0)
                if r['aborted']:
                    print(f'[verify] aborted: {r["aborted"]}')
                    return None
                rows = [q for q in r['rows'] if q['cmd'][1] != 0.0]
                m = cc.step_metrics([q['t'] for q in rows], [q['v'] for q in rows], tg,
                                    rows[-1]['t'] if rows else 0.0, steady)
                metrics.append(m)
                print(f'[verify] {tg:+.3f}: steady {m.steady:+.3f}, error {m.steady_error * 100:.1f} cm/s, '
                      f'overshoot {m.overshoot_pct:.0f} %')
            cap['verify'].append({'pid': dict(pid), 'metrics': [m.as_dict() for m in metrics]})
            verdict = cc.pid_verdict(metrics, ff, pas)
            if verdict['passed']:
                break
            pid['kp'], pid['ki'], why = cc.retune(pid['kp'], pid['ki'], verdict)
            print(f'[verify] retune: {why}')
        return cap
    finally:
        drv.close()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
