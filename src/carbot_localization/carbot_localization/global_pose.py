"""BLOCK 06 - Let UWB help in the background (V4 Estimator.globalPose).

Keeps a separate coarse course position = local pose + a slowly corrected
offset. One EKF update per fresh anchor range, each individually innovation-
gated (UWB_Handoff section 11); distinctive visual geometry also corrects it.
Used for route identity (block 09 branch check), never for steering.
"""
from carbot_common import topics as T
from carbot_common.stub import BlockSpec, run_stub
from carbot_interfaces.msg import LocalizationStatus, UwbRanges
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

SPEC = BlockSpec(
    node='global_pose', block='06', title='Coarse UWB-Aided Position', phase=3,
    required=['rate_hz', 'initial_variance_m2', 'growth_per_s_m2', 'growth_per_m_m2',
              'range_sigma_m', 'range_gate_chi2', 'position_gate_chi2', 'use_per_range_updates',
              'skip_repeated_sample_seq', 'latency_compensation', 'visual_landmark_min_rank',
              'visual_landmark_sigma_m', 'max_uwb_age_s', 'data.uwb', 'data.track_map'],
    subs=[(Odometry, T.LOCAL_POSE, 10), (LocalizationStatus, T.LOCAL_STATUS, 10),
          (UwbRanges, T.UWB_RANGES, 10)],
    pubs=[(PoseWithCovarianceStamped, T.GLOBAL_POSE, 10),
          (LocalizationStatus, T.LOCALIZATION_STATUS, 10)],
)


def main(args=None):
    run_stub(SPEC, args=args)
