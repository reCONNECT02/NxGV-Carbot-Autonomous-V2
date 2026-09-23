"""Block 14: V4 safety checks, first failing check is the veto."""
import numpy as np
from carbot_control.safety_core import SafetyCfg, SafetyCore, SafetyInputs


def good(t=10.0, **kw):
    d = dict(t=t, odom_t=t - 0.05, local_sigma=0.01, local_sigma_t=t - 0.1, grid_t=t - 0.1, connected=400)
    d.update(kw)
    return SafetyInputs(**d)


def test_all_good_allows_motion():
    r = SafetyCore(SafetyCfg()).evaluate(good())
    assert r.motion_allowed and r.veto_check == '' and {c.name for c in r.checks} >= {
        'e_stop', 'motion_fresh', 'local_sigma', 'camera_fresh', 'route_identity', 'road_mask', 'tunnel_clearance'}


def test_each_veto():
    cfg = SafetyCfg()
    cases = [(dict(estop=True), 'e_stop'), (dict(odom_t=9.7), 'motion_fresh'),
             (dict(local_sigma=0.05), 'local_sigma'), (dict(local_sigma_t=9.0), 'local_sigma'),
             (dict(grid_t=9.5), 'camera_fresh'),
             (dict(branch_hold=True, branch_reason='Planned branch does not match'), 'route_identity')]
    for kw, name in cases:
        r = SafetyCore(cfg).evaluate(good(**kw))
        assert not r.motion_allowed and r.veto_check == name, (kw, r.veto_check)


def test_recoverable_branch_and_parking_do_not_veto():
    cfg = SafetyCfg()
    assert SafetyCore(cfg).evaluate(good(branch_hold=True, branch_reason=cfg.recoverable_branch_reason)).motion_allowed
    assert SafetyCore(cfg).evaluate(good(branch_hold=True, branch_reason='x', parking=True)).motion_allowed


def test_road_mask_needs_dwell():
    core = SafetyCore(SafetyCfg())
    assert core.evaluate(good(t=10.0, connected=10)).motion_allowed
    assert core.evaluate(good(t=10.4, connected=10)).motion_allowed
    r = core.evaluate(good(t=10.6, connected=10))
    assert r.veto_check == 'road_mask'
    assert core.evaluate(good(t=10.7, connected=300)).motion_allowed


def test_tunnel_forward_clearance():
    core = SafetyCore(SafetyCfg())
    ang = np.linspace(-np.pi, np.pi, 360, endpoint=False)
    rng = np.full(360, 0.5)
    assert core.evaluate(good(in_tunnel=True, scan_t=9.9, scan_ranges=rng, scan_angles=ang)).motion_allowed
    rng2 = rng.copy()
    rng2[np.abs(ang) < 0.1] = 0.20
    r = core.evaluate(good(in_tunnel=True, scan_t=9.9, scan_ranges=rng2, scan_angles=ang))
    assert r.veto_check == 'tunnel_clearance'
    r = core.evaluate(good(in_tunnel=True, scan_t=9.0, scan_ranges=rng, scan_angles=ang))
    assert r.veto_check == 'tunnel_clearance' and 'unavailable' in r.veto_reason
    rng3 = rng.copy()
    rng3[np.abs(ang - np.pi) < 0.1] = 0.1                        # obstacle BEHIND: fine
    assert core.evaluate(good(in_tunnel=True, scan_t=9.9, scan_ranges=rng3, scan_angles=ang)).motion_allowed
    assert core.evaluate(good(in_tunnel=False, scan_t=0.0)).motion_allowed      # outside the tunnel: n/a


# --------------------------------------------------------------------------- status heartbeat (BACKLOG #50)
def test_publish_due_change_is_immediate_heartbeat_is_periodic():
    from carbot_control.safety_core import publish_due
    k = (True, '', '')
    assert publish_due(None, k, 0.0, 0.04)                            # first status always goes out
    assert not publish_due((0.0, k), k, 0.02, 0.04)                   # unchanged, too early
    assert publish_due((0.0, k), k, 0.04, 0.04)                       # heartbeat
    assert publish_due((0.0, k), (False, 'lidar', 'stale'), 0.001, 0.04)   # a veto is never delayed
    assert publish_due((0.0, (False, 'lidar', 'stale')), k, 0.001, 0.04)   # nor is the release
