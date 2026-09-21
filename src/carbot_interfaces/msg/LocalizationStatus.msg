# Blocks 05/06 -> GUI Localization tab, safety (block 14). /carbot/localization/status
std_msgs/Header header
float32 local_sigma_m   # V4 Estimator.sigma (lateral confidence)
float32 global_sigma_m  # V4 Estimator.globalSigma
float32 global_offset_x_m
float32 global_offset_y_m
float32 uwb_residual_m
float32 uwb_gain
bool uwb_last_accepted
float32 uwb_age_s
uint32 visual_matches
float32 visual_rank
bool visual_enabled
float32 distance_travelled_m
# ---- added in phase 3 (extension, nothing renamed) ----
uint32 visual_updates   # block 05: accepted camera-to-map corrections since start
bool imu_ok             # block 05: /imu/rpy fresh (else heading from /odom yaw)
float32 heading_rad     # block 05: local heading (track frame)
uint32 uwb_accepted     # block 06: per-range updates accepted since start
uint32 uwb_rejected     # block 06: per-range updates gated out since start
uint32 uwb_reacquires   # block 06: gate lock-outs resolved by inflating the variance
string state            # e.g. RUNNING, WAITING_INPUT, DEAD_RECKONING, NO_UWB
