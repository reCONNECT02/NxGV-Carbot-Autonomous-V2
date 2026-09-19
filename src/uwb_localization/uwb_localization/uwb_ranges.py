"""UWB range parser (feeds BLOCK 06 only).

Per UWB_Handoff.md section 11:
* subscribe BEST_EFFORT to /uwb3/input_json (a RELIABLE subscriber gets nothing)
* per anchor: skip repeated sample_seq, stamp = arrival - age_ms,
  range_corrected = raw - range_offset_m, flattened by anchor/tag z difference
* empty links = no measurement (never (0, 0))
* publishes UwbRanges, UwbStatus (link, per-anchor age) and, for the GUI only,
  a raw pairwise trilateration fix (tools/uwb/uwb_xy.py method).
Anchor positions and offsets: data/uwb.yaml (calibration step 10 overrides).
Phase 3 implements the parser.
"""
from carbot_common import topics as T
from carbot_common.qos import UWB
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import UwbRanges, UwbStatus
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

SPEC = BlockSpec(
    node='uwb_ranges', block='06', title='UWB range parser', phase=3,
    required=['link_timeout_s', 'anchor_timeout_s', 'expected_rate_hz', 'max_range_age_ms',
              'min_range_m', 'max_range_m', 'publish_raw_fix', 'data.uwb'],
    subs=[(String, T.UWB_INPUT_JSON, UWB)],
    pubs=[(UwbRanges, T.UWB_RANGES, 10), (UwbStatus, T.UWB_STATUS, 10),
          (PointStamped, T.UWB_RAW_FIX, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
