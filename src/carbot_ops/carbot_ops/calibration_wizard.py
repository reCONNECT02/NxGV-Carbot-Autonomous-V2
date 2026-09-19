"""Guided calibration wizard backend (calibrate.launch.py only, phase 8).

Runs the 12 steps of data/calibration_steps.yaml in order. Each step: RUN ->
result + pass/fail -> SAVE or REDO; a step cannot be skipped until it passes
unless the user chooses KEEP_PREVIOUS (a PASS from an earlier session).
Results go to a new timestamped session folder (carbot_common.calibration_store);
older sessions are kept for rollback.

Phase-1 stub: publishes the step list (all PENDING); actions are refused.
"""
from carbot_common import calibration_store as cs
from carbot_common import topics as T
from carbot_common.data import load_data
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import CalibrationState, CalibrationStepState, NodeStatus
from carbot_interfaces.srv import CalibrationAction

SPEC = BlockSpec(
    node='calibration_wizard', block='', title='Calibration wizard', phase=8,
    required=['session_format', 'allow_keep_previous', 'data_root', 'data.calibration_steps'],
    subs=[(NodeStatus, T.STATUS, 50)],
    pubs=[(CalibrationState, T.CALIBRATION_STATE, LATCHED)],
)


def _setup(node):
    steps = load_data(node, 'calibration_steps')['steps']
    root = cs.data_root(node.p('data_root'))
    previous = cs.load_summary(cs.active_session(root)).get('steps', {})
    st = CalibrationState()
    st.header.stamp = node.get_clock().now().to_msg()
    st.session = '(not started)'
    st.current_index = 1
    for s in steps:
        e = CalibrationStepState()
        e.index = int(s['index'])
        e.id = s['id']
        e.title = s['title']
        e.required = bool(s.get('required', True))
        e.status = 'PENDING'
        prev = (previous.get(s['id']) or {}).get('status', '')
        e.result_summary = f'previous: {prev}' if prev else 'no previous result'
        st.steps.append(e)
    node.publishers_by_topic[T.CALIBRATION_STATE].publish(st)

    def action(req, resp):
        resp.ok = False
        resp.passed = False
        resp.message = 'calibration steps are implemented in phase 8'
        return resp
    node.create_service(CalibrationAction, T.CALIBRATION_ACTION_SRV, action)


def main(args=None):
    run_stub(SPEC, extra=_setup, args=args)
