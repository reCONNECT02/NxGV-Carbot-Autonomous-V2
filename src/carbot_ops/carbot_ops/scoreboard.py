"""Scoreboard: the 13 rubric challenges with the result detected during a run
(phase 7/8). Any manual intervention (e-stop, input after START) marks the
current challenge FAIL (0 marks), as the rulebook requires.
Phase-1 stub: publishes all 13 challenges as PENDING from challenges.yaml.
"""
from carbot_common import topics as T
from carbot_common.data import load_data
from carbot_common.qos import LATCHED
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import (ChallengeResult, CommandOwnerState, MissionEvent, MissionState,
                                   Scoreboard)
from std_msgs.msg import Bool

SPEC = BlockSpec(
    node='scoreboard', block='', title='Scoreboard', phase=8,
    required=['rate_hz', 'data.challenges'],
    subs=[(MissionEvent, T.MISSION_EVENTS, 50), (MissionState, T.MISSION_STATE, LATCHED),
          (Bool, T.E_STOP, 10), (CommandOwnerState, T.OWNER_STATE, 10),
          (Bool, T.RACE_ARMED, LATCHED)],
    pubs=[(Scoreboard, T.SCOREBOARD, LATCHED)],
)


def _setup(node):
    board = Scoreboard()
    for c in load_data(node, 'challenges')['challenges']:
        board.challenges.append(ChallengeResult(id=int(c['id']), name=c['name'],
                                                max_marks=int(c['marks'][0]), state='PENDING'))

    def tick():
        board.header.stamp = node.get_clock().now().to_msg()
        node.publishers_by_topic[T.SCOREBOARD].publish(board)
    node.create_timer(1.0 / float(node.p('rate_hz')), tick)


def main(args=None):
    run_stub(SPEC, extra=_setup, args=args)
