"""system_monitor pure helpers (no ROS)."""
import struct

from carbot_ops import monitor_core as mc


def test_rate_meter_steady_30hz():
    m = mc.RateMeter(2.0)
    for i in range(90):
        m.add(i / 30.0)
    assert abs(m.rate_hz(89 / 30.0) - 30.0) < 0.5
    assert m.age_s(89 / 30.0) == 0.0


def test_rate_meter_goes_to_zero_when_stopped():
    m = mc.RateMeter(2.0)
    for i in range(30):
        m.add(i / 10.0)
    assert m.rate_hz(2.9) > 9.0
    assert m.rate_hz(10.0) == 0.0
    assert abs(m.age_s(10.0) - 7.1) < 1e-6


def test_rate_meter_never():
    m = mc.RateMeter(1.0)
    assert m.rate_hz(5.0) == 0.0 and m.age_s(5.0) == -1.0


def test_cdr_stamp_little_and_big_endian():
    le = bytes([0, 1, 0, 0]) + struct.pack('<iI', 1758600000, 250_000_000) + b'rest'
    be = bytes([0, 0, 0, 0]) + struct.pack('>iI', 1758600000, 5) + b'rest'
    assert mc.cdr_stamp(le) == (1758600000, 250_000_000)
    assert mc.cdr_stamp(be) == (1758600000, 5)
    assert mc.cdr_stamp(b'') is None
    assert mc.cdr_stamp(bytes([7, 7, 0, 0]) + b'\0' * 8) is None
    assert mc.cdr_stamp(bytes([0, 1, 0, 0]) + struct.pack('<iI', 0, 0)) is None


def test_latency():
    st = (100, 500_000_000)
    assert abs(mc.latency_ms(st, 100.6, 1e5) - 100.0) < 1e-6
    assert mc.latency_ms(st, 5000.0, 1000.0) == -1.0
    assert mc.latency_ms(None, 1.0, 1e5) == -1.0


def test_scan_processes_filters_wrappers_and_labels_namespaces():
    procs = [
        (10, 'sudo', ['sudo', '-n', '/usr/local/lib/carbot/kill_stale.sh', '/tmp/patterns.txt', '2']),
        (11, 'ros2', ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', 'carbot_bringup', 'astra_rgb.launch.py']),
        (12, 'component_conta', ['/opt/ros/humble/lib/rclcpp_components/component_container', '--ros-args',
                                 '-r', '__node:=astra_camera_container', '-r', '__ns:=/']),
        (13, 'component_conta', ['/opt/ros/humble/lib/rclcpp_components/component_container', '--ros-args',
                                 '-r', '__node:=astra_camera_container', '-r', '__ns:=/old']),
        (14, 'hobot_codec_rep', ['hobot_codec_republish']),
        (15, 'vim', ['vim', 'notes.txt']),
    ]
    out = mc.scan_processes(procs, ['astra_camera', 'hobot_codec', 'websocket'],
                            ['sudo', 'bash', 'ros2', 'python3'])
    # wrappers (sudo, `ros2 launch`) are skipped; the same driver in two namespaces counts as two labels
    assert out == [('astra_camera /', 12), ('astra_camera /old', 13), ('hobot_codec', 14)]


def test_read_number(tmp_path):
    p = tmp_path / 't'
    p.write_text('52123\n')
    assert mc.read_number(str(p)) == 52123.0
    p.write_text('ratio: 37%\n')
    assert mc.read_number(str(p)) == 37.0
    assert mc.read_number(str(tmp_path / 'missing')) == -1.0


def test_env_network(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '1')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '0')
    assert mc.env_network(1)['ok']
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    e = mc.env_network(1)
    assert not e['ok'] and len(e['problems']) == 2
