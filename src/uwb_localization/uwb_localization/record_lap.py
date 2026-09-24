"""Record a lap of Haffiz's UWB position into lap.csv for tools/map/map_builder.py.

    ros2 run uwb_localization record_lap                    # -> ./lap.csv, Ctrl-C to stop
    ros2 run uwb_localization record_lap -o ~/lap.csv --tag  # tag antenna, not the rear axle

Reads /carbot/uwb/position (the uwb_ranges node must be running: calibrate.launch.py
starts it). Each fix is moved from the tag antenna to the rear axle with the filter's
velocity heading (uwb.yaml tag.mount_xy_m); fixes slower than --min-speed are
skipped (no heading, and standing still piles up points). Then, on a laptop:

    python tools/map/map_builder.py edit lap.csv     # fit + drag, s = save track_map.yaml

The wizard's step 11 "UWB lap" does the same recording + fit on the car and also
writes uwb.yaml track_to_venue; use this tool when the road shape itself needs
editing. After a map edit, re-run steps 11 and 12 (mission planner).
UWB here is read-only: nothing reaches the servo.
"""
import argparse
import math
import os
import signal

from .positioning import rear_axle_from_tag


def main(argv=None):
    ap = argparse.ArgumentParser(description='Record Haffiz UWB positions into lap.csv (venue metres)')
    ap.add_argument('-o', '--out', default='lap.csv')
    ap.add_argument('--tag', action='store_true', help='write the tag antenna position (no lever-arm shift)')
    ap.add_argument('--min-speed', type=float, default=0.05, help='m/s; slower fixes are skipped')
    ap.add_argument('--uwb-yaml', default='', help='uwb.yaml for the tag mount (default: the ACTIVE calibration '
                    'session\'s data/uwb.yaml, else the installed one)')
    a = ap.parse_args(argv)

    import rclpy
    from carbot_common import topics as T
    from carbot_common import calibration_store as cs
    from carbot_common.calib_tools import bringup_config_dir
    from carbot_common.data import load_yaml
    from nav_msgs.msg import Odometry

    uwb_path = a.uwb_yaml or cs.data_override(cs.active_session(cs.data_root()), 'uwb.yaml') or \
        os.path.join(bringup_config_dir(), 'data', 'uwb.yaml')
    lever = (0.0, 0.0)
    if not a.tag:
        lever = tuple(float(v) for v in load_yaml(uwb_path)['tag']['mount_xy_m'])
        print(f'[record_lap] tag mount {lever} from {uwb_path}', flush=True)
    rclpy.init()
    node = rclpy.create_node('carbot_record_lap')
    out = open(a.out, 'w', encoding='utf-8')
    out.write(f'# x,y venue metres, {"tag antenna" if a.tag else "rear axle"}, Haffiz filtered UWB '
              f'({T.UWB_POSITION}); lever {lever}\n')
    n = {'kept': 0, 'slow': 0}

    def on_pos(m: Odometry):
        v = m.twist.twist.linear
        sp = math.hypot(v.x, v.y)
        if sp < a.min_speed:
            n['slow'] += 1
            return
        xy = (m.pose.pose.position.x, m.pose.pose.position.y)
        if not a.tag:
            xy = rear_axle_from_tag(xy, math.atan2(v.y, v.x), lever)
        out.write(f'{xy[0]:.4f},{xy[1]:.4f}\n')
        n['kept'] += 1
        if n['kept'] % 50 == 0:
            out.flush()
            print(f'[record_lap] {n["kept"]} points ({n["slow"]} skipped: too slow)', flush=True)

    node.create_subscription(Odometry, T.UWB_POSITION, on_pos, 10)
    print(f'[record_lap] recording {T.UWB_POSITION} -> {os.path.abspath(a.out)}. Walk the car once around '
          'the track, then Ctrl-C.', flush=True)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        print(f'[record_lap] saved {n["kept"]} points to {os.path.abspath(a.out)}. Next: '
              f'python tools/map/map_builder.py edit {a.out}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
