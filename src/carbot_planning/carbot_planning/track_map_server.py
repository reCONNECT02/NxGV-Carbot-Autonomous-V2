"""BLOCK 01 - Set the map, start and goals (V4 core.js DEFAULTS, Course).

Loads data/track_map.yaml (prior map, track frame) and publishes it latched as
JSON for the GUI and any late joiner. Car geometry is in common.yaml (vehicle.*).
"""
from carbot_common import topics as T
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from std_msgs.msg import String

SPEC = BlockSpec(
    node='track_map_server', block='01', title='Setup: map, start, goals', phase=4,
    required=['publish_period_s', 'data.track_map', 'vehicle.car_length_m', 'vehicle.wheelbase_m',
              'vehicle.min_turning_radius_m'],
    pubs=[(String, T.TRACK_MAP_JSON, LATCHED)],
)


def main(args=None):
    run_stub(SPEC, args=args)
