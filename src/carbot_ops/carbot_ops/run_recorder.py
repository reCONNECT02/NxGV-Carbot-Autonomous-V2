"""Run recorder: rosbag start/stop, list, replay (phase 8).

Race mode auto-records every run into <data_root>/bags/. Replay feeds every GUI
tab. Topic list: ops.yaml run_recorder.topics (raw camera images excluded).
"""
from carbot_common import topics as T
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.srv import RecordControl
from std_msgs.msg import String

SPEC = BlockSpec(
    node='run_recorder', block='', title='Run recorder', phase=8,
    required=['storage_id', 'max_bag_size_mb', 'topics', 'data_root'],
    pubs=[(String, T.RECORD_STATE, 10)],
)


def _setup(node):
    def control(req, resp):
        resp.ok = False
        resp.message = 'recording is implemented in phase 8'
        return resp
    node.create_service(RecordControl, T.RECORD_CONTROL_SRV, control)


def main(args=None):
    run_stub(SPEC, extra=_setup, args=args)
