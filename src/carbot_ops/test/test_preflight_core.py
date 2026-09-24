"""race_supervisor pure logic (no ROS)."""
import math

from carbot_ops import preflight_core as pc

NODES = ('command_owner', 'safety_monitor')


def cfg(**kw):
    base = dict(camera_min_rate_ratio=0.8, camera_max_age_s=0.5, lidar_min_hz=6.0, lidar_max_age_s=0.5,
                uwb_anchor_max_age_s=1.0, battery_min_v=10.8, start_position_tolerance_m=0.20,
                start_heading_tolerance_deg=15.0, start_pose_check_enabled=True, ready_hold_s=1.0, status_max_age_s=3.0,
                pose_max_age_s=1.0, startup_grace_s=2.0, required_nodes=NODES,
                warn_only_nodes=('bpu_detector',))
    base.update(kw)
    return pc.Cfg(**base)


def good(now=10.0, **kw):
    s = pc.Snapshot(
        now=now, camera_name='astra', camera=(15.0, 15.0, 0.05), camera_report_age_s=0.5,
        lidar_rate_hz=10.0, lidar_age_s=0.05, uwb_link_ok=True, uwb_report_age_s=0.5,
        uwb_anchor_ids=('1782', '1783'), uwb_anchor_age_s=(0.2, 0.3), uwb_anchor_seen=(True, True),
        uwb_anchors_surveyed=True, uwb_offsets_calibrated=True, battery_v=12.1, battery_age_s=0.5,
        pose=(7.18, 1.30, math.radians(180.0)), pose_age_s=0.1, start_pose=(7.18, 1.30, 180.0),
        safety_allowed=True, safety_age_s=0.1,
        node_levels={n: (0, 0.5) for n in NODES})
    for k, v in kw.items():
        setattr(s, k, v)
    return s


def failing(checks):
    return [c.name for c in checks if not c.ok]


def test_all_green():
    assert failing(pc.evaluate(cfg(), good())) == []


def test_camera_slow_and_missing():
    assert failing(pc.evaluate(cfg(), good(camera=(5.0, 15.0, 0.05)))) == ['camera_front']
    assert failing(pc.evaluate(cfg(), good(camera=(15.0, 15.0, 0.9)))) == ['camera_front']
    assert failing(pc.evaluate(cfg(), good(camera=None))) == ['camera_front']


def test_lidar():
    assert failing(pc.evaluate(cfg(), good(lidar_rate_hz=2.0))) == ['lidar']
    assert failing(pc.evaluate(cfg(), good(lidar_age_s=-1.0))) == ['lidar']


def test_uwb_anchor_missing_and_link_down():
    s = good(uwb_anchor_seen=(True, False))
    assert failing(pc.evaluate(cfg(), s)) == ['uwb_anchor_1783']
    assert failing(pc.evaluate(cfg(), good(uwb_link_ok=False))) == ['uwb_link']
    assert failing(pc.evaluate(cfg(), good(uwb_offsets_calibrated=False))) == ['uwb_calibrated']


def test_battery():
    assert failing(pc.evaluate(cfg(), good(battery_v=10.0))) == ['battery']
    assert failing(pc.evaluate(cfg(), good(battery_v=None))) == ['battery']


def test_start_pose_position_and_heading():
    far = good(pose=(7.18 + 0.5, 1.30, math.radians(180.0)))
    assert failing(pc.evaluate(cfg(), far)) == ['start_pose']
    turned = good(pose=(7.18, 1.30, math.radians(150.0)))
    assert failing(pc.evaluate(cfg(), turned)) == ['start_pose']
    wrap = good(pose=(7.18, 1.30, math.radians(-175.0)), start_pose=(7.18, 1.30, 180.0))
    assert failing(pc.evaluate(cfg(), wrap)) == []
    assert failing(pc.evaluate(cfg(), good(pose=None))) == ['start_pose']


def test_nodes_stub_error_and_silent():
    s = good(node_levels={'command_owner': (3, 0.5), 'safety_monitor': (2, 0.5)})
    assert failing(pc.evaluate(cfg(), s)) == ['node_command_owner', 'node_safety_monitor']
    s = good(node_levels={'command_owner': (0, 0.5)})
    assert failing(pc.evaluate(cfg(), s)) == ['node_safety_monitor']
    s = good(node_levels={'command_owner': (0, 9.0), 'safety_monitor': (0, 0.5)})
    assert failing(pc.evaluate(cfg(), s)) == ['node_command_owner']


def test_detector_error_is_only_a_warning():
    s = good()
    s.node_levels['bpu_detector'] = (2, 0.5)
    rows = pc.evaluate(cfg(), s)
    assert failing(rows) == []
    assert [c.name for c in rows if c.name == 'node_bpu_detector']


def test_takeover_estop_roles_safety():
    assert failing(pc.evaluate(cfg(), good(takeover=True))) == ['manual_takeover']
    assert failing(pc.evaluate(cfg(), good(estop=True))) == ['e_stop']
    assert failing(pc.evaluate(cfg(), good(unconfirmed_roles=['front']))) == ['camera_roles']
    s = good(safety_allowed=False, safety_veto='road_mask')
    assert failing(pc.evaluate(cfg(), s)) == ['safety_monitor']


def run(m, now, missing=(), snap=None):
    return m.step(now, missing, pc.evaluate(m.cfg, snap or good(now=now)))


def test_state_walk_and_ready_hold():
    m = pc.Machine(cfg(), t0=0.0)
    assert run(m, 1.0) == pc.LOADING
    assert run(m, 3.0) == pc.PREFLIGHT
    assert run(m, 3.5) == pc.PREFLIGHT
    assert run(m, 4.1) == pc.READY
    assert run(m, 5.0, snap=good(battery_v=9.0)) == pc.NOT_READY
    assert run(m, 5.5) == pc.PREFLIGHT       # hold restarts after a failure
    assert run(m, 6.6) == pc.READY


def test_calibration_missing_blocks():
    m = pc.Machine(cfg(), t0=0.0)
    assert run(m, 9.0, missing=['uwb_survey']) == pc.CAL_MISSING
    ok, _ = m.start()
    assert not ok and not m.started


def test_start_only_at_ready_and_once():
    m = pc.Machine(cfg(), t0=0.0)
    run(m, 3.0)
    assert m.start()[0] is False
    run(m, 4.5)
    assert m.state == pc.READY
    ok, _ = m.start()
    assert ok and m.state == pc.RUNNING and m.started
    assert m.start()[0] is False
    assert run(m, 20.0, snap=good(battery_v=1.0)) == pc.RUNNING    # nothing un-arms after START


def test_estop_and_finish_after_start_only():
    m = pc.Machine(cfg(), t0=0.0)
    m.estop()
    m.mission_complete()
    assert m.state == pc.LOADING
    run(m, 3.0)
    run(m, 4.5)
    m.start()
    m.estop()
    assert m.state == pc.ESTOPPED
    m2 = pc.Machine(cfg(), t0=0.0)
    run(m2, 3.0)
    run(m2, 4.5)
    m2.start()
    m2.mission_complete()
    assert m2.state == pc.FINISHED


def test_start_pose_check_can_be_assumed():
    far = good(pose=(0.0, 0.0, 0.0))
    assert failing(pc.evaluate(cfg(start_pose_check_enabled=False), far)) == []
    row = [c for c in pc.evaluate(cfg(start_pose_check_enabled=False), good(pose=None)) if c.name == 'start_pose'][0]
    assert row.ok and 'assumed' in row.detail
